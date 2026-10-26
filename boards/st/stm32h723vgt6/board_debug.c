/*
 * STM32H723VGT6 board - debug connectivity fix
 *
 * Direct register writes to DBGMCU, no HAL function dependency.
 * DBGMCU base: 0x5C001000 (STM32H7 D3 domain)
 * CR offset 0x004:
 *   bit 22 = D1DBGCKEN (0x00400000)
 *   bit 21 = D3DBGCKEN (0x00200000)
 *   bit  1 = DBG_STOP_D1 (0x00000002)
 *   bit  0 = DBG_SLEEP_D1 (0x00000001)
 *
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>

#define DBGMCU_BASE  0x5C001000UL
#define DBGMCU_CR    (*(volatile uint32_t *)(DBGMCU_BASE + 0x004))

/* Bits we need: D1 debug clock + D3 debug clock + sleep debug + stop debug */
#define DBGMCU_CR_NEEDED  (0x00400000U | 0x00200000U | 0x00000002U | 0x00000001U)

static int mc02_debug_init(void)
{
	DBGMCU_CR |= DBGMCU_CR_NEEDED;
	return 0;
}

/* Priority 2: runs AFTER soc_config.c (PRE_KERNEL_1 prio 1) */
SYS_INIT(mc02_debug_init, PRE_KERNEL_1, 2);
