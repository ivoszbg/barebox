// SPDX-License-Identifier: GPL-2.0-only AND BSD-1-Clause
// SPDX-FileCopyrightText: 2017, Microchip Corporation

#include <mach/at91sam9263_matrix.h>
#include <mach/barebox-arm.h>
#include <mach/at91_rstc.h>
#include <mach/at91_wdt.h>
#include <mach/sam92_ll.h>

static void sam9263_pmc_init(const struct sam92_pmc_config *config)
{
	at91_pmc_init(IOMEM(AT91SAM926X_BASE_PMC), 0);

	/* Initialize PLL charge pump, must be done before PLLAR/PLLBR */
	at91_pmc_init_pll(IOMEM(AT91SAM926X_BASE_PMC), AT91SAM9_PMC_ICPPLLA | AT91SAM9_PMC_ICPPLLB);

	/* Setting PLL A and divider A */
	at91_pmc_cfg_plla(IOMEM(AT91SAM926X_BASE_PMC),
			  AT91_PMC_MUL_(config->mula) |
			  AT91_PMC_OUT_2 |		// 190 to 240 MHz		
			  config->diva,			// Divider
			  0);

	/* Selection of Master Clock and Processor Clock */
	 
	/* PCK = PLLA = 2 * MCK */
	at91_pmc_cfg_mck(IOMEM(AT91SAM926X_BASE_PMC),
			 AT91_PMC_CSS_SLOW
			 | AT91_PMC_PRES_1
			 | AT91SAM9_PMC_MDIV_2
			 | AT91_PMC_PDIV_1,
			 0);

	/* Switch MCK on PLLA output */
	at91_pmc_cfg_mck(IOMEM(AT91SAM926X_BASE_PMC),
			 AT91_PMC_CSS_PLLA
			 | AT91_PMC_PRES_1
			 | AT91SAM9_PMC_MDIV_2
			 | AT91_PMC_PDIV_1,
			 0);
}

static inline void matrix_wr(unsigned int offset, const unsigned int value)
{
	writel(value, IOMEM(AT91SAM9263_BASE_MATRIX + offset));
}

static void sam9263_matrix_init(void)
{
	/* Bus Matrix Master Configuration Register */
	matrix_wr(AT91SAM9263_MATRIX_MCFG0, AT91SAM9263_MATRIX_ULBT_SIXTEEN);	/* OHCI */
	matrix_wr(AT91SAM9263_MATRIX_MCFG1, AT91SAM9263_MATRIX_ULBT_EIGHT);	/* ISI */
	matrix_wr(AT91SAM9263_MATRIX_MCFG2, AT91SAM9263_MATRIX_ULBT_EIGHT);	/* 2D */
	matrix_wr(AT91SAM9263_MATRIX_MCFG3, AT91SAM9263_MATRIX_ULBT_EIGHT);	/* DMAC */
	matrix_wr(AT91SAM9263_MATRIX_MCFG4, AT91SAM9263_MATRIX_ULBT_FOUR);	/* MACB */
	matrix_wr(AT91SAM9263_MATRIX_MCFG5, AT91SAM9263_MATRIX_ULBT_SIXTEEN);	/* LCDC */
	matrix_wr(AT91SAM9263_MATRIX_MCFG6, AT91SAM9263_MATRIX_ULBT_SINGLE);	/* PDC */
	matrix_wr(AT91SAM9263_MATRIX_MCFG7, AT91SAM9263_MATRIX_ULBT_EIGHT);	/* DBUS */
	matrix_wr(AT91SAM9263_MATRIX_MCFG8, AT91SAM9263_MATRIX_ULBT_EIGHT);	/* IBUS */

	/* Bus Matrix Slave Configuration Registers */

	/* ROM */
	matrix_wr(AT91SAM9263_MATRIX_SCFG0,
		  AT91SAM9263_MATRIX_ARBT_FIXED_PRIORITY
		  | AT91SAM9263_MATRIX_FIXED_DEFMSTR_ARM926I
		  | AT91SAM9263_MATRIX_DEFMSTR_TYPE_LAST
		  | AT91SAM9263_MATRIX_SLOT_CYCLE_(32));

	/* RAM80K */
	matrix_wr(AT91SAM9263_MATRIX_SCFG1,
		  AT91SAM9263_MATRIX_ARBT_FIXED_PRIORITY
		  | AT91SAM9263_MATRIX_FIXED_DEFMSTR_EMAC
		  | AT91SAM9263_MATRIX_DEFMSTR_TYPE_LAST
		  | AT91SAM9263_MATRIX_SLOT_CYCLE_(32));

	/* RAM16K */
	matrix_wr(AT91SAM9263_MATRIX_SCFG2,
		  AT91SAM9263_MATRIX_ARBT_FIXED_PRIORITY
		  | AT91SAM9263_MATRIX_FIXED_DEFMSTR_USB
		  | AT91SAM9263_MATRIX_DEFMSTR_TYPE_LAST
		  | AT91SAM9263_MATRIX_SLOT_CYCLE_(16));

	/* PERIPHERALS */
	matrix_wr(AT91SAM9263_MATRIX_SCFG3,
		  AT91SAM9263_MATRIX_ARBT_FIXED_PRIORITY
		  | AT91SAM9263_MATRIX_FIXED_DEFMSTR_PDC
		  | AT91SAM9263_MATRIX_DEFMSTR_TYPE_LAST
		  | AT91SAM9263_MATRIX_SLOT_CYCLE_(4));

	/* EBI0 */
	matrix_wr(AT91SAM9263_MATRIX_SCFG4,
		  AT91SAM9263_MATRIX_ARBT_ROUND_ROBIN
		  | AT91SAM9263_MATRIX_FIXED_DEFMSTR_ARM926I
		  | AT91SAM9263_MATRIX_DEFMSTR_TYPE_LAST
		  | AT91SAM9263_MATRIX_SLOT_CYCLE_(32));

	/* EBI1 */
	matrix_wr(AT91SAM9263_MATRIX_SCFG5,
		  AT91SAM9263_MATRIX_ARBT_FIXED_PRIORITY
		  | AT91SAM9263_MATRIX_FIXED_DEFMSTR_LCDC
		  | AT91SAM9263_MATRIX_DEFMSTR_TYPE_LAST
		  | AT91SAM9263_MATRIX_SLOT_CYCLE_(64));

	/* APB */
	matrix_wr(AT91SAM9263_MATRIX_SCFG6,
		  AT91SAM9263_MATRIX_ARBT_FIXED_PRIORITY
		  | AT91SAM9263_MATRIX_FIXED_DEFMSTR_ARM926D
		  | AT91SAM9263_MATRIX_DEFMSTR_TYPE_LAST
		  | AT91SAM9263_MATRIX_SLOT_CYCLE_(4));

	/* ROM */
	matrix_wr(AT91SAM9263_MATRIX_PRAS0,
		  AT91SAM9263_MATRIX_M0PR_(1)
		  | AT91SAM9263_MATRIX_M1PR_(0)
		  | AT91SAM9263_MATRIX_M2PR_(2)
		  | AT91SAM9263_MATRIX_M3PR_(1)
		  | AT91SAM9263_MATRIX_M4PR_(0)
		  | AT91SAM9263_MATRIX_M5PR_(3)
		  | AT91SAM9263_MATRIX_M6PR_(2)
		  | AT91SAM9263_MATRIX_M7PR_(3));

	matrix_wr(AT91SAM9263_MATRIX_PRBS0, AT91SAM9263_MATRIX_M8PR_(0));

	/* RAM80K */
	matrix_wr(AT91SAM9263_MATRIX_PRAS1,
		  AT91SAM9263_MATRIX_M0PR_(1)
		  | AT91SAM9263_MATRIX_M1PR_(2)
		  | AT91SAM9263_MATRIX_M2PR_(1)
		  | AT91SAM9263_MATRIX_M3PR_(3)
		  | AT91SAM9263_MATRIX_M4PR_(0)
		  | AT91SAM9263_MATRIX_M5PR_(0)
		  | AT91SAM9263_MATRIX_M6PR_(3)
		  | AT91SAM9263_MATRIX_M7PR_(0));

	matrix_wr(AT91SAM9263_MATRIX_PRBS1, AT91SAM9263_MATRIX_M8PR_(2));

       /* RAM16K */
	matrix_wr(AT91SAM9263_MATRIX_PRAS2,
		  AT91SAM9263_MATRIX_M0PR_(1)
		  | AT91SAM9263_MATRIX_M1PR_(0)
		  | AT91SAM9263_MATRIX_M2PR_(2)
		  | AT91SAM9263_MATRIX_M3PR_(1)
		  | AT91SAM9263_MATRIX_M4PR_(0)
		  | AT91SAM9263_MATRIX_M5PR_(3)
		  | AT91SAM9263_MATRIX_M6PR_(3)
		  | AT91SAM9263_MATRIX_M7PR_(2));

	matrix_wr(AT91SAM9263_MATRIX_PRBS2, AT91SAM9263_MATRIX_M8PR_(0));

	/* PERIPHERALS */
	matrix_wr(AT91SAM9263_MATRIX_PRAS3,
		  AT91SAM9263_MATRIX_M0PR_(0)
		  | AT91SAM9263_MATRIX_M1PR_(1)
		  | AT91SAM9263_MATRIX_M2PR_(0)
		  | AT91SAM9263_MATRIX_M3PR_(2)
		  | AT91SAM9263_MATRIX_M4PR_(1)
		  | AT91SAM9263_MATRIX_M5PR_(0)
		  | AT91SAM9263_MATRIX_M6PR_(3)
		  | AT91SAM9263_MATRIX_M7PR_(2));

	matrix_wr(AT91SAM9263_MATRIX_PRBS3, AT91SAM9263_MATRIX_M8PR_(3));

	/* EBI0 */
	matrix_wr(AT91SAM9263_MATRIX_PRAS4,
		  AT91SAM9263_MATRIX_M0PR_(1)
		  | AT91SAM9263_MATRIX_M1PR_(3)
		  | AT91SAM9263_MATRIX_M2PR_(0)
		  | AT91SAM9263_MATRIX_M3PR_(2)
		  | AT91SAM9263_MATRIX_M4PR_(3)
		  | AT91SAM9263_MATRIX_M5PR_(0)
		  | AT91SAM9263_MATRIX_M6PR_(0)
		  | AT91SAM9263_MATRIX_M7PR_(1));

	matrix_wr(AT91SAM9263_MATRIX_PRBS4, AT91SAM9263_MATRIX_M8PR_(2));

	/* EBI1 */
	matrix_wr(AT91SAM9263_MATRIX_PRAS5,
		  AT91SAM9263_MATRIX_M0PR_(0)
		  | AT91SAM9263_MATRIX_M1PR_(1)
		  | AT91SAM9263_MATRIX_M2PR_(0)
		  | AT91SAM9263_MATRIX_M3PR_(0)
		  | AT91SAM9263_MATRIX_M4PR_(3)
		  | AT91SAM9263_MATRIX_M5PR_(2)
		  | AT91SAM9263_MATRIX_M6PR_(3)
		  | AT91SAM9263_MATRIX_M7PR_(2));

	matrix_wr(AT91SAM9263_MATRIX_PRBS5, AT91SAM9263_MATRIX_M8PR_(1));

	/* APB */
	matrix_wr(AT91SAM9263_MATRIX_PRAS6,
		  AT91SAM9263_MATRIX_M0PR_(1)
		  | AT91SAM9263_MATRIX_M1PR_(0)
		  | AT91SAM9263_MATRIX_M2PR_(2)
		  | AT91SAM9263_MATRIX_M3PR_(1)
		  | AT91SAM9263_MATRIX_M4PR_(0)
		  | AT91SAM9263_MATRIX_M5PR_(0)
		  | AT91SAM9263_MATRIX_M6PR_(3)
		  | AT91SAM9263_MATRIX_M7PR_(3));

	matrix_wr(AT91SAM9263_MATRIX_PRBS6, AT91SAM9263_MATRIX_M8PR_(2));
}

static void sam9263_rstc_init(void)
{
	writel(AT91_RSTC_KEY | AT91_RSTC_URSTEN, IOMEM(AT91SAM926X_BASE_RSTC + AT91_RSTC_MR));
}

void sam9263_lowlevel_init(const struct sam92_pmc_config *config)
{
	at91_wdt_disable(IOMEM(AT91SAM9263_BASE_WDT));
	sam9263_pmc_init(config);
	sam9263_matrix_init();
	sam9263_rstc_init();
}