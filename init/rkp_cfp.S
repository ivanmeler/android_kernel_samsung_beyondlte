/*
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/memory.h>

.section .hyperdrive, "ax"
.global _start_hyperdrive
_start_hyperdrive:
# Need a nop otherwise GCC keeps _start_hyperdrive label but NOT
# jopp_springboard_bl label.
nop

#ifdef CONFIG_RKP_CFP_JOPP

#define RRX x16
#define RRX_32 w16
#define JOPP_EXCETPION_FIMC_START 	(0xffffff80fa000000)
#define JOPP_EXCETPION_FIMC_END 	(0xffffff80fa780000)

/*
 * Load the word just before the function instructions.
 * Halt execution if the word doesn't match the magic number placed at function
 * entries.
 */
	.macro	springboard_blr, reg
	jopp_springboard_blr_\reg:
	push	RRX, xzr
	ldr	RRX_32, [\reg, #-4]
	subs	RRX_32, RRX_32, #0xbe7, lsl #12
	cmp	RRX_32, #0xbad
	b.ne	1f
	pop	RRX, xzr
	br 	\reg

	/* 
	 * JOPP for FIMC region
	 */
1:	mov_q	RRX, JOPP_EXCETPION_FIMC_START
	cmp 	\reg, RRX
	b.lt	1f
	mov_q	RRX, JOPP_EXCETPION_FIMC_END
	cmp 	\reg, RRX
	b.hs	1f
	pop	RRX, xzr
	br 	\reg
	/* 
	 * JOPP for MODULE region
	 */
1:	mov_q	RRX, MODULES_VADDR
	cmp 	\reg, RRX
	b.lt	2f
	mov_q	RRX, MODULES_END
	cmp 	\reg, RRX
	b.hs	2f
	pop	RRX, xzr
	br 	\reg
2: // dead
	.inst	0xdeadc0de //crash for sure
	.endm

	// Only add springboards for a register rX if there exists a "BLR rX" instruction in vmlinux to instrument
	springboard_blr x0
	springboard_blr	x1
	springboard_blr	x2
	springboard_blr	x3
	springboard_blr	x4
	springboard_blr	x5
	springboard_blr	x6
	springboard_blr	x7
	springboard_blr	x8
	springboard_blr	x9
	springboard_blr	x10
	springboard_blr	x11
	springboard_blr	x12
	springboard_blr	x13
	springboard_blr	x14
	springboard_blr	x15
	/*
	 * x16, x17 are reserved.
	 * Undefine ROPP will free x16 and x17
	 * JOPP only needs no registers
	 */
#ifndef CONFIG_RKP_CFP_ROPP
	springboard_blr	x16
	springboard_blr	x17
#endif
	springboard_blr	x18
	springboard_blr	x19
	springboard_blr	x20
	springboard_blr	x21
	springboard_blr	x22
	springboard_blr	x23
	springboard_blr	x24
	springboard_blr	x25
	springboard_blr	x26
	springboard_blr	x27
	springboard_blr	x28
	springboard_blr	x29
	springboard_blr	x30
	// Don't need springboards for x31 (not allowed in RET by assembler)

#endif


.global _end_hyperdrive
_end_hyperdrive:

.section .text
