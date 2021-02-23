// SPDX-License-Identifier: GPL-2.0

#include <linux/mm.h>
#include <linux/rkp.h>

#include <asm/pgtable.h>

bool rkp_started __rkp_ro = false;

sparse_bitmap_for_kernel_t* rkp_s_bitmap_ro __rkp_ro = 0;
sparse_bitmap_for_kernel_t* rkp_s_bitmap_dbl __rkp_ro = 0;
sparse_bitmap_for_kernel_t* rkp_s_bitmap_buffer __rkp_ro = 0;

/* init/main.c */
void __init rkp_init(void)
{
	struct rkp_init init_data;

	memset((void *)&init_data, 0, sizeof(struct rkp_init));
	/* initialized rkp_init struct */
	init_data.magic = RKP_INIT_MAGIC;
	init_data.vmalloc_start = VMALLOC_START;
#ifndef CONFIG_RKP_FIMC_CHECK
	init_data.no_fimc_verify = 1;
#endif
	init_data.fimc_phys_addr = 0;
	init_data._text = (u64)_text;
	init_data._etext = (u64)_etext;
	init_data._srodata = (u64)__start_rodata;
	init_data._erodata = (u64)__end_rodata;
	init_data.large_memory = 0;

	init_data.vmalloc_end = (u64)high_memory;
	init_data.init_mm_pgd = (u64)__pa(swapper_pg_dir);
	init_data.id_map_pgd = (u64)__pa(idmap_pg_dir);
	init_data.zero_pg_addr = (u64)__pa(empty_zero_page);

#ifdef CONFIG_UNMAP_KERNEL_AT_EL0
	init_data.tramp_pgd = (u64)__pa(tramp_pg_dir);
	init_data.tramp_valias = (u64)TRAMP_VALIAS;
#endif
	uh_call(UH_APP_RKP, RKP_GET_RO_BITMAP, (u64)&rkp_s_bitmap_ro, 0, 0, 0);
	uh_call(UH_APP_RKP, RKP_GET_DBL_BITMAP, (u64)&rkp_s_bitmap_dbl, 0, 0, 0);

	uh_call(UH_APP_RKP, RKP_START, (u64)&init_data, (u64)kimage_voffset, 0, 0);
	rkp_started = true;
}

void rkp_deferred_init(void)
{
	uh_call(UH_APP_RKP, RKP_DEFERRED_START, 0, 0, 0, 0);
}

/* RO BUFFER */
void __init rkp_robuffer_init(void)
{
	uh_call(UH_APP_RKP, RKP_GET_BUFFER_BITMAP, (u64)&rkp_s_bitmap_buffer, 0, 0, 0);
}

