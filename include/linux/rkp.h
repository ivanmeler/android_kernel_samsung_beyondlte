#ifndef _RKP_H
#define _RKP_H

#ifndef __ASSEMBLY__
#include <linux/uh.h>

/* uH_RKP Command ID */
enum __RKP_CMD_ID{
	RKP_START = 0x01,
	RKP_DEFERRED_START = 0x02,
	RKP_WRITE_PGT1 = 0x03,
	RKP_WRITE_PGT2 = 0x04,
	RKP_WRITE_PGT3 = 0x05,
	RKP_EMULT_TTBR0 = 0x06,
	RKP_EMULT_TTBR1 = 0x07,
	RKP_EMULT_DORESUME = 0x08,
	RKP_FREE_PGD = 0x09,
	RKP_NEW_PGD = 0x0A,
	RKP_KASLR_MEM = 0x0B,
	RKP_FIMC_VERIFY = 0x0C,
	/* CFP cmds */
	RKP_JOPP_INIT = 0x0D,
	RKP_ROPP_INIT = 0x0E,
	RKP_ROPP_SAVE = 0x0F,
	RKP_ROPP_RELOAD = 0x10,
	/* RKP robuffer cmds*/
	RKP_ROBUFFER_ALLOC = 0x11,
	RKP_ROBUFFER_FREE = 0x12,
	RKP_GET_RO_BITMAP = 0x13,
	RKP_GET_DBL_BITMAP = 0x14,
	RKP_GET_BUFFER_BITMAP = 0x15,
	/* dynamic load */
	RKP_DYNAMIC_LOAD = 0x20,
	RKP_MODULE_LOAD = 0x21,
	RKP_BPF_LOAD = 0x22,
#ifdef CONFIG_RKP_TEST
	CMD_ID_TEST_GET_PAR = 0x81,
	CMD_ID_TEST_GET_RO = 0x83,
	CMD_ID_TEST_GET_VA_XN,
	CMD_ID_TEST_GET_VMM_INFO,
#endif
};

#ifdef CONFIG_RKP_TEST
#define RKP_INIT_MAGIC		0x5afe0002
#else
#define RKP_INIT_MAGIC		0x5afe0001
#endif

#define RKP_FIMC_FAIL		0x10
#define RKP_FIMC_SUCCESS	0xa5

#define SPARSE_UNIT_BIT (30)
#define SPARSE_UNIT_SIZE (1<<SPARSE_UNIT_BIT)

#define RKP_DYN_COMMAND_BREAKDOWN_BEFORE_INIT	0x00
#define RKP_DYN_COMMAND_INS						0x01
#define RKP_DYN_COMMAND_RM						0x10

#define RKP_DYN_FIMC				0x02
#define RKP_DYN_FIMC_COMBINED		0x03
#define RKP_DYN_MODULE				0x04

struct rkp_init { //copy from uh (app/rkp/rkp.h)
	u32 magic;
	u64 vmalloc_start;
	u64 vmalloc_end;
	u64 init_mm_pgd;
	u64 id_map_pgd;
	u64 zero_pg_addr;
	u64 rkp_pgt_bitmap;
	u64 rkp_dbl_bitmap;
	u32 rkp_bitmap_size;
	u32 no_fimc_verify;
	u64 fimc_phys_addr;
	u64 _text;
	u64 _etext;
	u64 extra_memory_addr;
	u32 extra_memory_size;
	u64 physmap_addr; //not used. what is this for?
	u64 _srodata;
	u64 _erodata;
	u32 large_memory;
	u64 tramp_pgd;
	u64 tramp_valias;
};

typedef struct sparse_bitmap_for_kernel {
	u64 start_addr;
	u64 end_addr;
	u64 maxn;
	char **map;
} sparse_bitmap_for_kernel_t;

typedef struct dynamic_load_struct {
	u32 type;
	u64 binary_base;
	u64 binary_size;
	u64 code_base1;
	u64 code_size1;
	u64 code_base2;
	u64 code_size2;
} rkp_dynamic_load_t;

extern sparse_bitmap_for_kernel_t* rkp_s_bitmap_ro;
extern sparse_bitmap_for_kernel_t* rkp_s_bitmap_dbl;
extern sparse_bitmap_for_kernel_t* rkp_s_bitmap_buffer;

typedef struct rkp_init rkp_init_t;
extern bool rkp_started;

extern void __init rkp_init(void);
extern void rkp_deferred_init(void);
extern void rkp_robuffer_init(void);

static inline u64 uh_call_static(u64 app_id, u64 cmd_id, u64 arg1)
{
	register u64 ret __asm__("x0") = app_id;
	register u64 cmd __asm__("x1") = cmd_id;
	register u64 arg __asm__("x2") = arg1;

	__asm__ volatile (
			"hvc	0\n"
			: "+r"(ret), "+r"(cmd), "+r"(arg)
			);

	return ret;
}

static inline void *rkp_ro_alloc(void)
{
	u64 addr = (u64)uh_call_static(UH_APP_RKP, RKP_ROBUFFER_ALLOC, 0);
	if (!addr)
		return 0;
	return (void *)__phys_to_virt(addr);
}

static inline void rkp_ro_free(void *free_addr)
{
	uh_call_static(UH_APP_RKP, RKP_ROBUFFER_FREE, (u64)free_addr);
}

static inline u8 rkp_check_bitmap(u64 pa, sparse_bitmap_for_kernel_t *kernel_bitmap)
{
	u8 val;
	u64 offset, map_loc, bit_offset;
	char *map;

	if (!kernel_bitmap || !kernel_bitmap->map)
		return 0;

	offset = pa - kernel_bitmap->start_addr;
	map_loc = ((offset % SPARSE_UNIT_SIZE) / PAGE_SIZE) >> 3;
	bit_offset = ((offset % SPARSE_UNIT_SIZE) / PAGE_SIZE) % 8;

	if (kernel_bitmap->maxn <= (offset >> SPARSE_UNIT_BIT)) 
		return 0;

	map = kernel_bitmap->map[(offset >> SPARSE_UNIT_BIT)];
	if (!map)
		return 0;

	val = (u8)((*(u64 *)(&map[map_loc])) >> bit_offset) & ((u64)1);
	return val;
}

static inline unsigned int is_rkp_ro_page(u64 va)
{
	return rkp_check_bitmap(__pa(va), rkp_s_bitmap_buffer);
}

static inline u8 rkp_is_pg_protected(u64 va)
{
	return rkp_check_bitmap(__pa(va), rkp_s_bitmap_ro);
}

static inline u8 rkp_is_pg_dbl_mapped(u64 pa)
{
	return rkp_check_bitmap(pa, rkp_s_bitmap_dbl);
}

/* allocation */
static inline phys_addr_t rkp_ro_alloc_phys(void)
{
	phys_addr_t ret = 0;

	uh_call(UH_APP_RKP, RKP_ROBUFFER_ALLOC, (u64)&ret, 0, 0, 0);

	return ret;
}

#endif //__ASSEMBLY__
#endif //_RKP_H
