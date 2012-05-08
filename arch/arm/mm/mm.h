#ifdef CONFIG_MMU

/* the upper-most page table pointer */
extern pmd_t *top_pmd;

#define TOP_PTE(x)	pte_offset_kernel(top_pmd, x)

static inline pmd_t *pmd_off_k(unsigned long virt)
{
	return pmd_offset(pud_offset(pgd_offset_k(virt), virt), virt);
}

struct mem_type;

const struct mem_type *get_mem_type(unsigned int type);

extern void __flush_dcache_page(struct address_space *mapping, struct page *page);

#endif

struct map_desc;

void __init bootmem_init(void);
void arm_mm_memblock_reserve(void);
void __init create_mapping(struct map_desc *md);
