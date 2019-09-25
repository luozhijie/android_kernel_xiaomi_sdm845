/*
 *	linux/mm/madvise.c
 *
 * Copyright (C) 1999  Linus Torvalds
 * Copyright (C) 2002  Christoph Hellwig
 */

#include <linux/mman.h>
#include <linux/pagemap.h>
#include <linux/syscalls.h>
#include <linux/mempolicy.h>
#include <linux/page-isolation.h>
#include <linux/hugetlb.h>
#include <linux/falloc.h>
#include <linux/sched.h>
#include <linux/ksm.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/blkdev.h>
#include <linux/backing-dev.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/page_idle.h>
#include <linux/mmu_notifier.h>
#include "internal.h"

#include <asm/tlb.h>

#include "internal.h"

/*
 * Any behaviour which results in changes to the vma->vm_flags needs to
 * take mmap_sem for writing. Others, which simply traverse vmas, need
 * to only take it for reading.
 */
static int madvise_need_mmap_write(int behavior)
{
	switch (behavior) {
	case MADV_REMOVE:
	case MADV_WILLNEED:
	case MADV_DONTNEED:
	case MADV_FREE:
	case MADV_COLD:
		return 0;
	default:
		/* be safe, default to 1. list exceptions explicitly */
		return 1;
	}
}

/*
 * We can potentially split a vm area into separate
 * areas, each area with its own behavior.
 */
static long madvise_behavior(struct vm_area_struct *vma,
		     struct vm_area_struct **prev,
		     unsigned long start, unsigned long end, int behavior)
{
	struct mm_struct *mm = vma->vm_mm;
	int error = 0;
	pgoff_t pgoff;
	unsigned long new_flags = vma->vm_flags;

	switch (behavior) {
	case MADV_NORMAL:
		new_flags = new_flags & ~VM_RAND_READ & ~VM_SEQ_READ;
		break;
	case MADV_SEQUENTIAL:
		new_flags = (new_flags & ~VM_RAND_READ) | VM_SEQ_READ;
		break;
	case MADV_RANDOM:
		new_flags = (new_flags & ~VM_SEQ_READ) | VM_RAND_READ;
		break;
	case MADV_DONTFORK:
		new_flags |= VM_DONTCOPY;
		break;
	case MADV_DOFORK:
		if (vma->vm_flags & VM_IO) {
			error = -EINVAL;
			goto out;
		}
		new_flags &= ~VM_DONTCOPY;
		break;
	case MADV_WIPEONFORK:
		/* MADV_WIPEONFORK is only supported on anonymous memory. */
		if (vma->vm_file || vma->vm_flags & VM_SHARED) {
			error = -EINVAL;
			goto out;
		}
		new_flags |= VM_WIPEONFORK;
		break;
	case MADV_KEEPONFORK:
		new_flags &= ~VM_WIPEONFORK;
		break;
	case MADV_DONTDUMP:
		new_flags |= VM_DONTDUMP;
		break;
	case MADV_DODUMP:
		if (!is_vm_hugetlb_page(vma) && new_flags & VM_SPECIAL) {
			error = -EINVAL;
			goto out;
		}
		new_flags &= ~VM_DONTDUMP;
		break;
	case MADV_MERGEABLE:
	case MADV_UNMERGEABLE:
		error = ksm_madvise(vma, start, end, behavior, &new_flags);
		if (error)
			goto out;
		break;
	case MADV_HUGEPAGE:
	case MADV_NOHUGEPAGE:
		error = hugepage_madvise(vma, &new_flags, behavior);
		if (error)
			goto out;
		break;
	}

	if (new_flags == vma->vm_flags) {
		*prev = vma;
		goto out;
	}

	pgoff = vma->vm_pgoff + ((start - vma->vm_start) >> PAGE_SHIFT);
	*prev = vma_merge(mm, *prev, start, end, new_flags, vma->anon_vma,
			  vma->vm_file, pgoff, vma_policy(vma),
			  vma->vm_userfaultfd_ctx, vma_get_anon_name(vma));
	if (*prev) {
		vma = *prev;
		goto success;
	}

	*prev = vma;

	if (start != vma->vm_start) {
		error = split_vma(mm, vma, start, 1);
		if (error)
			goto out;
	}

	if (end != vma->vm_end) {
		error = split_vma(mm, vma, end, 0);
		if (error)
			goto out;
	}

success:
	/*
	 * vm_flags is protected by the mmap_sem held in write mode.
	 */

	vm_write_begin(vma);
	WRITE_ONCE(vma->vm_flags, new_flags);
	vm_write_end(vma);
out:
	if (error == -ENOMEM)
		error = -EAGAIN;
	return error;
}

#ifdef CONFIG_SWAP
static int swapin_walk_pmd_entry(pmd_t *pmd, unsigned long start,
	unsigned long end, struct mm_walk *walk)
{
	pte_t *orig_pte;
	struct vm_area_struct *vma = walk->private;
	unsigned long index;

	if (pmd_none_or_trans_huge_or_clear_bad(pmd))
		return 0;

	for (index = start; index != end; index += PAGE_SIZE) {
		pte_t pte;
		swp_entry_t entry;
		struct page *page;
		spinlock_t *ptl;

		orig_pte = pte_offset_map_lock(vma->vm_mm, pmd, start, &ptl);
		pte = *(orig_pte + ((index - start) / PAGE_SIZE));
		pte_unmap_unlock(orig_pte, ptl);

		if (pte_present(pte) || pte_none(pte))
			continue;
		entry = pte_to_swp_entry(pte);
		if (unlikely(non_swap_entry(entry)))
			continue;

		page = read_swap_cache_async(entry, GFP_HIGHUSER_MOVABLE,
								vma, index);
		if (page)
			put_page(page);
	}

	return 0;
}

static void force_swapin_readahead(struct vm_area_struct *vma,
		unsigned long start, unsigned long end)
{
	struct mm_walk walk = {
		.mm = vma->vm_mm,
		.pmd_entry = swapin_walk_pmd_entry,
		.private = vma,
	};

	walk_page_range(start, end, &walk);

	lru_add_drain();	/* Push any new pages onto the LRU now */
}

static void force_shm_swapin_readahead(struct vm_area_struct *vma,
		unsigned long start, unsigned long end,
		struct address_space *mapping)
{
	pgoff_t index;
	struct page *page;
	swp_entry_t swap;

	for (; start < end; start += PAGE_SIZE) {
		index = ((start - vma->vm_start) >> PAGE_SHIFT) + vma->vm_pgoff;

		page = find_get_entry(mapping, index);
		if (!radix_tree_exceptional_entry(page)) {
			if (page)
				put_page(page);
			continue;
		}
		swap = radix_to_swp_entry(page);
		page = read_swap_cache_async(swap, GFP_HIGHUSER_MOVABLE,
								NULL, 0);
		if (page)
			put_page(page);
	}

	lru_add_drain();	/* Push any new pages onto the LRU now */
}
#endif		/* CONFIG_SWAP */

/*
 * Schedule all required I/O operations.  Do not wait for completion.
 */
static long madvise_willneed(struct vm_area_struct *vma,
			     struct vm_area_struct **prev,
			     unsigned long start, unsigned long end)
{
	struct file *file = vma->vm_file;

	*prev = vma;
#ifdef CONFIG_SWAP
	if (!file) {
		force_swapin_readahead(vma, start, end);
		return 0;
	}

	if (shmem_mapping(file->f_mapping)) {
		force_shm_swapin_readahead(vma, start, end,
					file->f_mapping);
		return 0;
	}
#else
	if (!file)
		return -EBADF;
#endif

	if (IS_DAX(file_inode(file))) {
		/* no bad return value, but ignore advice */
		return 0;
	}

	start = ((start - vma->vm_start) >> PAGE_SHIFT) + vma->vm_pgoff;
	if (end > vma->vm_end)
		end = vma->vm_end;
	end = ((end - vma->vm_start) >> PAGE_SHIFT) + vma->vm_pgoff;

	force_page_cache_readahead(file->f_mapping, file, start, end - start);
	return 0;
}

static int madvise_free_pte_range(pmd_t *pmd, unsigned long addr,
				unsigned long end, struct mm_walk *walk)

{
	struct mmu_gather *tlb = walk->private;
	struct mm_struct *mm = tlb->mm;
	struct vm_area_struct *vma = walk->vma;
	spinlock_t *ptl;
	pte_t *orig_pte, *pte, ptent;
	struct page *page;
	int nr_swap = 0;
	unsigned long next;

	next = pmd_addr_end(addr, end);
	if (pmd_trans_huge(*pmd))
		if (madvise_free_huge_pmd(tlb, vma, pmd, addr, next))
			goto next;

	if (pmd_trans_unstable(pmd))
		return 0;

	orig_pte = pte = pte_offset_map_lock(mm, pmd, addr, &ptl);
	flush_tlb_batched_pending(mm);
	arch_enter_lazy_mmu_mode();
	for (; addr != end; pte++, addr += PAGE_SIZE) {
		ptent = *pte;

		if (pte_none(ptent))
			continue;
		/*
		 * If the pte has swp_entry, just clear page table to
		 * prevent swap-in which is more expensive rather than
		 * (page allocation + zeroing).
		 */
		if (!pte_present(ptent)) {
			swp_entry_t entry;

			entry = pte_to_swp_entry(ptent);
			if (non_swap_entry(entry))
				continue;
			nr_swap--;
			free_swap_and_cache(entry);
			pte_clear_not_present_full(mm, addr, pte, tlb->fullmm);
			continue;
		}

		page = vm_normal_page(vma, addr, ptent);
		if (!page)
			continue;

		/*
		 * If pmd isn't transhuge but the page is THP and
		 * is owned by only this process, split it and
		 * deactivate all pages.
		 */
		if (PageTransCompound(page)) {
			if (page_mapcount(page) != 1)
				goto out;
			get_page(page);
			if (!trylock_page(page)) {
				put_page(page);
				goto out;
			}
			pte_unmap_unlock(orig_pte, ptl);
			if (split_huge_page(page)) {
				unlock_page(page);
				put_page(page);
				pte_offset_map_lock(mm, pmd, addr, &ptl);
				goto out;
			}
			unlock_page(page);
			put_page(page);
			pte = pte_offset_map_lock(mm, pmd, addr, &ptl);
			pte--;
			addr -= PAGE_SIZE;
			continue;
		}

		VM_BUG_ON_PAGE(PageTransCompound(page), page);

		if (PageSwapCache(page) || PageDirty(page)) {
			if (!trylock_page(page))
				continue;
			/*
			 * If page is shared with others, we couldn't clear
			 * PG_dirty of the page.
			 */
			if (page_mapcount(page) != 1) {
				unlock_page(page);
				continue;
			}

			if (PageSwapCache(page) && !try_to_free_swap(page)) {
				unlock_page(page);
				continue;
			}

			ClearPageDirty(page);
			unlock_page(page);
		}

		if (pte_young(ptent) || pte_dirty(ptent)) {
			/*
			 * Some of architecture(ex, PPC) don't update TLB
			 * with set_pte_at and tlb_remove_tlb_entry so for
			 * the portability, remap the pte with old|clean
			 * after pte clearing.
			 */
			ptent = ptep_get_and_clear_full(mm, addr, pte,
							tlb->fullmm);

			ptent = pte_mkold(ptent);
			ptent = pte_mkclean(ptent);
			set_pte_at(mm, addr, pte, ptent);
			if (PageActive(page))
				deactivate_page(page);
			tlb_remove_tlb_entry(tlb, pte, addr);
		}
	}
out:
	if (nr_swap) {
		if (current->mm == mm)
			sync_mm_rss(mm);

		add_mm_counter(mm, MM_SWAPENTS, nr_swap);
	}
	arch_leave_lazy_mmu_mode();
	pte_unmap_unlock(orig_pte, ptl);
	cond_resched();
next:
	return 0;
}

static void madvise_free_page_range(struct mmu_gather *tlb,
			     struct vm_area_struct *vma,
			     unsigned long addr, unsigned long end)
{
	struct mm_walk free_walk = {
		.pmd_entry = madvise_free_pte_range,
		.mm = vma->vm_mm,
		.private = tlb,
	};

	vm_write_begin(vma);
	tlb_start_vma(tlb, vma);
	walk_page_range(addr, end, &free_walk);
	tlb_end_vma(tlb, vma);
	vm_write_end(vma);
}

static int madvise_free_single_vma(struct vm_area_struct *vma,
			unsigned long start_addr, unsigned long end_addr)
{
	unsigned long start, end;
	struct mm_struct *mm = vma->vm_mm;
	struct mmu_gather tlb;

	if (vma->vm_flags & (VM_LOCKED|VM_HUGETLB|VM_PFNMAP))
		return -EINVAL;

	/* MADV_FREE works for only anon vma at the moment */
	if (!vma_is_anonymous(vma))
		return -EINVAL;

	start = max(vma->vm_start, start_addr);
	if (start >= vma->vm_end)
		return -EINVAL;
	end = min(vma->vm_end, end_addr);
	if (end <= vma->vm_start)
		return -EINVAL;

	lru_add_drain();
	tlb_gather_mmu(&tlb, mm, start, end);
	update_hiwater_rss(mm);

	mmu_notifier_invalidate_range_start(mm, start, end);
	madvise_free_page_range(&tlb, vma, start, end);
	mmu_notifier_invalidate_range_end(mm, start, end);
	tlb_finish_mmu(&tlb, start, end);

	return 0;
}

static long madvise_free(struct vm_area_struct *vma,
			     struct vm_area_struct **prev,
			     unsigned long start, unsigned long end)
{
	*prev = vma;
	return madvise_free_single_vma(vma, start, end);
}

static int madvise_cold_pte_range(pmd_t *pmd, unsigned long addr,
				unsigned long end, struct mm_walk *walk)
{
	struct mmu_gather *tlb = walk->private;
	struct mm_struct *mm = tlb->mm;
	struct vm_area_struct *vma = walk->vma;
	pte_t *orig_pte, *pte, ptent;
	spinlock_t *ptl;
	struct page *page;

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	if (pmd_trans_huge(*pmd)) {
		pmd_t orig_pmd;
		unsigned long next = pmd_addr_end(addr, end);

		ptl = pmd_trans_huge_lock(pmd, vma);
		if (!ptl)
			return 0;

		orig_pmd = *pmd;
		if (is_huge_zero_pmd(orig_pmd))
			goto huge_unlock;

		page = pmd_page(orig_pmd);
		if (next - addr != HPAGE_PMD_SIZE) {
			int err;

			if (page_mapcount(page) != 1)
				goto huge_unlock;

			get_page(page);
			spin_unlock(ptl);
			lock_page(page);
			err = split_huge_page(page);
			unlock_page(page);
			put_page(page);
			if (!err)
				goto regular_page;
			return 0;
		}

		if (pmd_young(orig_pmd)) {
			pmdp_invalidate(vma, addr, pmd);
			orig_pmd = pmd_mkold(orig_pmd);

			set_pmd_at(mm, addr, pmd, orig_pmd);
			tlb_remove_pmd_tlb_entry(tlb, pmd, addr);
		}

		test_and_clear_page_young(page);
		deactivate_page(page);
huge_unlock:
		spin_unlock(ptl);
		return 0;
	}

	if (pmd_trans_unstable(pmd))
		return 0;
regular_page:
#endif
	orig_pte = pte = pte_offset_map_lock(mm, pmd, addr, &ptl);
	flush_tlb_batched_pending(mm);
	arch_enter_lazy_mmu_mode();
	for (; addr < end; pte++, addr += PAGE_SIZE) {
		ptent = *pte;

		if (pte_none(ptent))
			continue;

		if (!pte_present(ptent))
			continue;

		page = vm_normal_page(vma, addr, ptent);
		if (!page)
			continue;

		/*
		 * Creating a THP page is expensive so split it only if we
		 * are sure it's worth. Split it if we are only owner.
		 */
		if (PageTransCompound(page)) {
			if (page_mapcount(page) != 1)
				break;
			get_page(page);
			if (!trylock_page(page)) {
				put_page(page);
				break;
			}
			pte_unmap_unlock(orig_pte, ptl);
			if (split_huge_page(page)) {
				unlock_page(page);
				put_page(page);
				pte_offset_map_lock(mm, pmd, addr, &ptl);
				break;
			}
			unlock_page(page);
			put_page(page);
			pte = pte_offset_map_lock(mm, pmd, addr, &ptl);
			pte--;
			addr -= PAGE_SIZE;
			continue;
		}

		VM_BUG_ON_PAGE(PageTransCompound(page), page);

		if (pte_young(ptent)) {
			ptent = ptep_get_and_clear_full(mm, addr, pte,
							tlb->fullmm);
			ptent = pte_mkold(ptent);
			set_pte_at(mm, addr, pte, ptent);
			tlb_remove_tlb_entry(tlb, pte, addr);
		}

		/*
		 * We are deactivating a page for accelerating reclaiming.
		 * VM couldn't reclaim the page unless we clear PG_young.
		 * As a side effect, it makes confuse idle-page tracking
		 * because they will miss recent referenced history.
		 */
		test_and_clear_page_young(page);
		deactivate_page(page);
	}

	arch_leave_lazy_mmu_mode();
	pte_unmap_unlock(orig_pte, ptl);
	cond_resched();

	return 0;
}

static void madvise_cold_page_range(struct mmu_gather *tlb,
			     struct vm_area_struct *vma,
			     unsigned long addr, unsigned long end)
{
	struct mm_walk cold_walk = {
		.pmd_entry = madvise_cold_pte_range,
		.mm = vma->vm_mm,
		.private = tlb,
	};

	vm_write_begin(vma);
	tlb_start_vma(tlb, vma);
	walk_page_range(addr, end, &cold_walk);
	tlb_end_vma(tlb, vma);
	vm_write_end(vma);
}

static long madvise_cold(struct vm_area_struct *vma,
			struct vm_area_struct **prev,
			unsigned long start_addr, unsigned long end_addr)
{
	struct mm_struct *mm = vma->vm_mm;
	struct mmu_gather tlb;

	*prev = vma;
	if (!can_madv_lru_vma(vma))
		return -EINVAL;

	lru_add_drain();
	tlb_gather_mmu(&tlb, mm, start_addr, end_addr);
	madvise_cold_page_range(&tlb, vma, start_addr, end_addr);
	tlb_finish_mmu(&tlb, start_addr, end_addr);

	return 0;
}


