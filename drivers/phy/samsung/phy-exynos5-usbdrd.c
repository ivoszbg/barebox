// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos5 SoC series USB DRD PHY driver
 *
 * Phy provider for USB 3.0 DRD controller on Exynos5 SoC series
 *
 * Copyright (C) 2014 Samsung Electronics Co., Ltd.
 * Author: Vivek Gautam <gautam.vivek@samsung.com>
 */

#include <common.h>
#include <init.h>
#include <io.h>
#include <of.h>
#include <errno.h>
#include <driver.h>
#include <malloc.h>
#include <linux/usb/phy.h>
#include <linux/phy/phy.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <mfd/syscon.h>
#include <regulator.h>

/* Exynos USB PHY registers */
#define EXYNOS5_FSEL_9MHZ6		0x0
#define EXYNOS5_FSEL_10MHZ		0x1
#define EXYNOS5_FSEL_12MHZ		0x2
#define EXYNOS5_FSEL_19MHZ2		0x3
#define EXYNOS5_FSEL_20MHZ		0x4
#define EXYNOS5_FSEL_24MHZ		0x5
#define EXYNOS5_FSEL_26MHZ		0x6
#define EXYNOS5_FSEL_50MHZ		0x7

/* Exynos5: USB 3.0 DRD PHY registers */
#define EXYNOS5_DRD_LINKSYSTEM			0x04
#define LINKSYSTEM_XHCI_VERSION_CONTROL		BIT(27)
#define LINKSYSTEM_FLADJ_MASK			(0x3f << 1)
#define LINKSYSTEM_FLADJ(_x)			((_x) << 1)

#define EXYNOS5_DRD_PHYUTMI			0x08
#define PHYUTMI_OTGDISABLE			BIT(6)
#define PHYUTMI_FORCESUSPEND			BIT(1)
#define PHYUTMI_FORCESLEEP			BIT(0)

#define EXYNOS5_DRD_PHYPIPE			0x0c

#define EXYNOS5_DRD_PHYCLKRST			0x10
#define PHYCLKRST_EN_UTMISUSPEND		BIT(31)
#define PHYCLKRST_SSC_REFCLKSEL_MASK		(0xff << 23)
#define PHYCLKRST_SSC_REFCLKSEL(_x)		((_x) << 23)
#define PHYCLKRST_SSC_RANGE_MASK		(0x03 << 21)
#define PHYCLKRST_SSC_RANGE(_x)			((_x) << 21)
#define PHYCLKRST_SSC_EN			BIT(20)
#define PHYCLKRST_REF_SSP_EN			BIT(19)
#define PHYCLKRST_REF_CLKDIV2			BIT(18)
#define PHYCLKRST_MPLL_MULTIPLIER_MASK		(0x7f << 11)
#define PHYCLKRST_MPLL_MULTIPLIER_100MHZ_REF	(0x19 << 11)
#define PHYCLKRST_MPLL_MULTIPLIER_50M_REF	(0x32 << 11)
#define PHYCLKRST_MPLL_MULTIPLIER_24MHZ_REF	(0x68 << 11)
#define PHYCLKRST_MPLL_MULTIPLIER_20MHZ_REF	(0x7d << 11)
#define PHYCLKRST_MPLL_MULTIPLIER_19200KHZ_REF	(0x02 << 11)
#define PHYCLKRST_FSEL_PIPE_MASK		(0x7 << 8)
#define PHYCLKRST_FSEL_UTMI_MASK		(0x7 << 5)
#define PHYCLKRST_FSEL(_x)			((_x) << 5)
#define PHYCLKRST_FSEL_PAD_100MHZ		(0x27 << 5)
#define PHYCLKRST_FSEL_PAD_24MHZ		(0x2a << 5)
#define PHYCLKRST_FSEL_PAD_20MHZ		(0x31 << 5)
#define PHYCLKRST_FSEL_PAD_19_2MHZ		(0x38 << 5)
#define PHYCLKRST_RETENABLEN			BIT(4)
#define PHYCLKRST_REFCLKSEL_MASK		(0x03 << 2)
#define PHYCLKRST_REFCLKSEL_PAD_REFCLK		(0x2 << 2)
#define PHYCLKRST_REFCLKSEL_EXT_REFCLK		(0x3 << 2)
#define PHYCLKRST_PORTRESET			BIT(1)
#define PHYCLKRST_COMMONONN			BIT(0)

#define EXYNOS5_DRD_PHYREG0			0x14
#define PHYREG0_SSC_REF_CLK_SEL			BIT(21)
#define PHYREG0_SSC_RANGE			BIT(20)
#define PHYREG0_CR_WRITE			BIT(19)
#define PHYREG0_CR_READ				BIT(18)
#define PHYREG0_CR_DATA_IN(_x)			((_x) << 2)
#define PHYREG0_CR_CAP_DATA			BIT(1)
#define PHYREG0_CR_CAP_ADDR			BIT(0)

#define EXYNOS5_DRD_PHYREG1			0x18
#define PHYREG1_CR_DATA_OUT(_x)			((_x) << 1)
#define PHYREG1_CR_ACK				BIT(0)

#define EXYNOS5_DRD_PHYPARAM0			0x1c
#define PHYPARAM0_REF_USE_PAD			BIT(31)
#define PHYPARAM0_REF_LOSLEVEL_MASK		(0x1f << 26)
#define PHYPARAM0_REF_LOSLEVEL			(0x9 << 26)

#define EXYNOS5_DRD_PHYPARAM1			0x20
#define PHYPARAM1_PCS_TXDEEMPH_MASK		(0x1f << 0)
#define PHYPARAM1_PCS_TXDEEMPH			(0x1c)

#define EXYNOS5_DRD_PHYTERM			0x24

#define EXYNOS5_DRD_PHYTEST			0x28
#define PHYTEST_POWERDOWN_SSP			BIT(3)
#define PHYTEST_POWERDOWN_HSP			BIT(2)

#define EXYNOS5_DRD_PHYADP			0x2c

#define EXYNOS5_DRD_PHYUTMICLKSEL		0x30
#define PHYUTMICLKSEL_UTMI_CLKSEL		BIT(2)

#define EXYNOS5_DRD_PHYRESUME			0x34

#define EXYNOS5_DRD_LINKPORT			0x44

/* USB 3.0 DRD PHY SS Function Control Reg; accessed by CR_PORT */
#define EXYNOS5_DRD_PHYSS_LOSLEVEL_OVRD_IN		(0x15)
#define LOSLEVEL_OVRD_IN_LOS_BIAS_5420			(0x5 << 13)
#define LOSLEVEL_OVRD_IN_LOS_BIAS_DEFAULT		(0x0 << 13)
#define LOSLEVEL_OVRD_IN_EN				(0x1 << 10)
#define LOSLEVEL_OVRD_IN_LOS_LEVEL_DEFAULT		(0x9 << 0)

#define EXYNOS5_DRD_PHYSS_TX_VBOOSTLEVEL_OVRD_IN	(0x12)
#define TX_VBOOSTLEVEL_OVRD_IN_VBOOST_5420		(0x5 << 13)
#define TX_VBOOSTLEVEL_OVRD_IN_VBOOST_DEFAULT		(0x4 << 13)

#define EXYNOS5_DRD_PHYSS_LANE0_TX_DEBUG		(0x1010)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_19M2_20M		(0x4 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_24M		(0x8 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_25M_26M		(0x8 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_48M_50M_52M	(0x20 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_62M5		(0x20 << 4)
#define LANE0_TX_DEBUG_RXDET_MEAS_TIME_96M_100M		(0x40 << 4)

/* Exynos850: USB DRD PHY registers */
#define EXYNOS850_DRD_LINKCTRL			0x04
#define LINKCTRL_FORCE_RXELECIDLE		BIT(18)
#define LINKCTRL_FORCE_PHYSTATUS		BIT(17)
#define LINKCTRL_FORCE_PIPE_EN			BIT(16)
#define LINKCTRL_FORCE_QACT			BIT(8)
#define LINKCTRL_BUS_FILTER_BYPASS(_x)		((_x) << 4)

#define EXYNOS850_DRD_LINKPORT			0x08
#define LINKPORT_HOST_NUM_U3			GENMASK(19, 16)
#define LINKPORT_HOST_NUM_U2			GENMASK(15, 12)

#define EXYNOS850_DRD_CLKRST			0x20
/*
 * On versions without SS ports (like E850), bit 3 is for the 2.0 phy (HS),
 * while on versions with (like gs101), bits 2 and 3 are for the 3.0 phy (SS)
 * and bits 12 & 13 for the 2.0 phy.
 */
#define CLKRST_PHY20_SW_POR			BIT(13)
#define CLKRST_PHY20_SW_POR_SEL			BIT(12)
#define CLKRST_LINK_PCLK_SEL			BIT(7)
#define CLKRST_PHY_SW_RST			BIT(3)
#define CLKRST_PHY_RESET_SEL			BIT(2)
#define CLKRST_PORT_RST				BIT(1)
#define CLKRST_LINK_SW_RST			BIT(0)

#define EXYNOS850_DRD_SSPPLLCTL			0x30
#define SSPPLLCTL_FSEL				GENMASK(2, 0)

#define EXYNOS850_DRD_UTMI			0x50
#define UTMI_FORCE_VBUSVALID			BIT(5)
#define UTMI_FORCE_BVALID			BIT(4)
#define UTMI_DP_PULLDOWN			BIT(3)
#define UTMI_DM_PULLDOWN			BIT(2)
#define UTMI_FORCE_SUSPEND			BIT(1)
#define UTMI_FORCE_SLEEP			BIT(0)

#define EXYNOS850_DRD_HSP			0x54
#define HSP_FSV_OUT_EN				BIT(24)
#define HSP_VBUSVLDEXTSEL			BIT(13)
#define HSP_VBUSVLDEXT				BIT(12)
#define HSP_EN_UTMISUSPEND			BIT(9)
#define HSP_COMMONONN				BIT(8)

#define EXYNOS850_DRD_HSPPARACON		0x58
#define HSPPARACON_TXVREF			GENMASK(31, 28)
#define HSPPARACON_TXRISE			GENMASK(25, 24)
#define HSPPARACON_TXRES			GENMASK(22, 21)
#define HSPPARACON_TXPREEMPPULSE		BIT(20)
#define HSPPARACON_TXPREEMPAMP			GENMASK(19, 18)
#define HSPPARACON_TXHSXV			GENMASK(17, 16)
#define HSPPARACON_TXFSLS			GENMASK(15, 12)
#define HSPPARACON_SQRX				GENMASK(10, 8)
#define HSPPARACON_OTG				GENMASK(6, 4)
#define HSPPARACON_COMPDIS			GENMASK(2, 0)

#define EXYNOS850_DRD_HSP_TEST			0x5c
#define HSP_TEST_SIDDQ				BIT(24)

/* Exynos9 - GS101 */
#define EXYNOS850_DRD_SECPMACTL			0x48
#define SECPMACTL_PMA_ROPLL_REF_CLK_SEL		GENMASK(13, 12)
#define SECPMACTL_PMA_LCPLL_REF_CLK_SEL		GENMASK(11, 10)
#define SECPMACTL_PMA_REF_FREQ_SEL		GENMASK(9, 8)
#define SECPMACTL_PMA_LOW_PWR			BIT(4)
#define SECPMACTL_PMA_TRSV_SW_RST		BIT(3)
#define SECPMACTL_PMA_CMN_SW_RST		BIT(2)
#define SECPMACTL_PMA_INIT_SW_RST		BIT(1)
#define SECPMACTL_PMA_APB_SW_RST		BIT(0)

/* PMA registers */
#define EXYNOS9_PMA_USBDP_CMN_REG0008		0x0020
#define CMN_REG0008_OVRD_AUX_EN			BIT(3)
#define CMN_REG0008_AUX_EN			BIT(2)

#define EXYNOS9_PMA_USBDP_CMN_REG00B8		0x02e0
#define CMN_REG00B8_LANE_MUX_SEL_DP		GENMASK(3, 0)

#define EXYNOS9_PMA_USBDP_CMN_REG01C0		0x0700
#define CMN_REG01C0_ANA_LCPLL_LOCK_DONE		BIT(7)
#define CMN_REG01C0_ANA_LCPLL_AFC_DONE		BIT(6)

/* these have similar register layout, for lanes 0 and 2 */
#define EXYNOS9_PMA_USBDP_TRSV_REG03C3			0x0f0c
#define EXYNOS9_PMA_USBDP_TRSV_REG07C3			0x1f0c
#define TRSV_REG03C3_LN0_MON_RX_CDR_AFC_DONE		BIT(3)
#define TRSV_REG03C3_LN0_MON_RX_CDR_CAL_DONE		BIT(2)
#define TRSV_REG03C3_LN0_MON_RX_CDR_FLD_PLL_MODE_DONE	BIT(1)
#define TRSV_REG03C3_LN0_MON_RX_CDR_LOCK_DONE		BIT(0)

/* TRSV_REG0413 and TRSV_REG0813 have similar register layout */
#define EXYNOS9_PMA_USBDP_TRSV_REG0413		0x104c
#define TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN	BIT(7)
#define TRSV_REG0413_OVRD_LN1_TX_RXD_EN		BIT(5)

#define EXYNOS9_PMA_USBDP_TRSV_REG0813		0x204c
#define TRSV_REG0813_OVRD_LN3_TX_RXD_COMP_EN	BIT(7)
#define TRSV_REG0813_OVRD_LN3_TX_RXD_EN		BIT(5)

/* PCS registers */
#define EXYNOS9_PCS_NS_VEC_PS1_N1		0x010c
#define EXYNOS9_PCS_NS_VEC_PS2_N0		0x0110
#define EXYNOS9_PCS_NS_VEC_PS3_N0		0x0118
#define NS_VEC_NS_REQ				GENMASK(31, 24)
#define NS_VEC_ENABLE_TIMER			BIT(22)
#define NS_VEC_SEL_TIMEOUT			GENMASK(21, 20)
#define NS_VEC_INV_MASK				GENMASK(19, 16)
#define NS_VEC_COND_MASK			GENMASK(11, 8)
#define NS_VEC_EXP_COND				GENMASK(3, 0)

#define EXYNOS9_PCS_OUT_VEC_2			0x014c
#define EXYNOS9_PCS_OUT_VEC_3			0x0150
#define PCS_OUT_VEC_B9_DYNAMIC			BIT(19)
#define PCS_OUT_VEC_B9_SEL_OUT			BIT(18)
#define PCS_OUT_VEC_B8_DYNAMIC			BIT(17)
#define PCS_OUT_VEC_B8_SEL_OUT			BIT(16)
#define PCS_OUT_VEC_B7_DYNAMIC			BIT(15)
#define PCS_OUT_VEC_B7_SEL_OUT			BIT(14)
#define PCS_OUT_VEC_B6_DYNAMIC			BIT(13)
#define PCS_OUT_VEC_B6_SEL_OUT			BIT(12)
#define PCS_OUT_VEC_B5_DYNAMIC			BIT(11)
#define PCS_OUT_VEC_B5_SEL_OUT			BIT(10)
#define PCS_OUT_VEC_B4_DYNAMIC			BIT(9)
#define PCS_OUT_VEC_B4_SEL_OUT			BIT(8)
#define PCS_OUT_VEC_B3_DYNAMIC			BIT(7)
#define PCS_OUT_VEC_B3_SEL_OUT			BIT(6)
#define PCS_OUT_VEC_B2_DYNAMIC			BIT(5)
#define PCS_OUT_VEC_B2_SEL_OUT			BIT(4)
#define PCS_OUT_VEC_B1_DYNAMIC			BIT(3)
#define PCS_OUT_VEC_B1_SEL_OUT			BIT(2)
#define PCS_OUT_VEC_B0_DYNAMIC			BIT(1)
#define PCS_OUT_VEC_B0_SEL_OUT			BIT(0)

#define EXYNOS9_PCS_TIMEOUT_0			0x0170

#define EXYNOS9_PCS_TIMEOUT_3			0x017c

#define EXYNOS9_PCS_EBUF_PARAM			0x0304
#define EBUF_PARAM_SKP_REMOVE_TH_EMPTY_MODE	GENMASK(29, 24)

#define EXYNOS9_PCS_BACK_END_MODE_VEC		0x030c
#define BACK_END_MODE_VEC_FORCE_EBUF_EMPTY_MODE	BIT(1)
#define BACK_END_MODE_VEC_DISABLE_DATA_MASK	BIT(0)

#define EXYNOS9_PCS_RX_CONTROL			0x03f0
#define RX_CONTROL_EN_BLOCK_ALIGNER_TYPE_B	BIT(22)

#define EXYNOS9_PCS_RX_CONTROL_DEBUG		0x03f4
#define RX_CONTROL_DEBUG_EN_TS_CHECK		BIT(5)
#define RX_CONTROL_DEBUG_NUM_COM_FOUND		GENMASK(3, 0)

#define EXYNOS9_PCS_LOCAL_COEF			0x040c
#define LOCAL_COEF_PMA_CENTER_COEF		GENMASK(21, 16)
#define LOCAL_COEF_LF				GENMASK(13, 8)
#define LOCAL_COEF_FS				GENMASK(5, 0)

#define EXYNOS9_PCS_HS_TX_COEF_MAP_0		0x0410
#define HS_TX_COEF_MAP_0_SSTX_DEEMP		GENMASK(17, 12)
#define HS_TX_COEF_MAP_0_SSTX_LEVEL		GENMASK(11, 6)
#define HS_TX_COEF_MAP_0_SSTX_PRE_SHOOT		GENMASK(5, 0)


#define KHZ	1000
#define MHZ	(KHZ * KHZ)

#define PHY_TUNING_ENTRY_PHY(o, m, v) {	\
		.off = (o),		\
		.mask = (m),		\
		.val = (v),		\
		.region = PTR_PHY	\
	}

#define PHY_TUNING_ENTRY_PCS(o, m, v) {	\
		.off = (o),		\
		.mask = (m),		\
		.val = (v),		\
		.region = PTR_PCS	\
	}

#define PHY_TUNING_ENTRY_PMA(o, m, v) {	\
		.off = (o),		\
		.mask = (m),		\
		.val = (v),		\
		.region = PTR_PMA,	\
	}

#define PHY_TUNING_ENTRY_LAST { .region = PTR_INVALID }

#define for_each_phy_tune(tune) \
	for (; (tune)->region != PTR_INVALID; ++(tune))

struct exynos5_usbdrd_phy_tuning {
	u32 off;
	u32 mask;
	u32 val;
	char region;
#define PTR_INVALID	0
#define PTR_PHY		1
#define PTR_PCS		2
#define PTR_PMA		3
};

enum exynos5_usbdrd_phy_tuning_state {
	PTS_UTMI_POSTINIT,
	PTS_PIPE3_PREINIT,
	PTS_PIPE3_INIT,
	PTS_PIPE3_POSTINIT,
	PTS_PIPE3_POSTLOCK,
	PTS_MAX,
};

enum exynos5_usbdrd_phy_id {
	EXYNOS5_DRDPHY_UTMI,
	EXYNOS5_DRDPHY_PIPE3,
	EXYNOS5_DRDPHYS_NUM,
};

struct phy_usb_instance;
struct exynos5_usbdrd_phy;

struct exynos5_usbdrd_phy_config {
	u32 id;
	void (*phy_isol)(struct phy_usb_instance *inst, bool isolate);
	void (*phy_init)(struct exynos5_usbdrd_phy *phy_drd);
	unsigned int (*set_refclk)(struct phy_usb_instance *inst);
};

struct exynos5_usbdrd_phy_drvdata {
	const struct exynos5_usbdrd_phy_config *phy_cfg;
	const struct exynos5_usbdrd_phy_tuning **phy_tunes;
	const struct phy_ops *phy_ops;
	const char * const *clk_names;
	int n_clks;
	const char * const *core_clk_names;
	int n_core_clks;
	const char * const *regulator_names;
	int n_regulators;
	u32 pmu_offset_usbdrd0_phy;
	u32 pmu_offset_usbdrd0_phy_ss;
	u32 pmu_offset_usbdrd1_phy;
};

/**
 * struct exynos5_usbdrd_phy - driver data for USB 3.0 PHY
 * @dev: pointer to device instance of this platform device
 * @reg_phy: usb phy controller register memory base
 * @reg_pcs: usb phy physical coding sublayer register memory base
 * @reg_pma: usb phy physical media attachment register memory base
 * @clks: clocks for register access
 * @core_clks: core clocks for phy (ref, pipe3, utmi+, ITP, etc. as required)
 * @drv_data: pointer to SoC level driver data structure
 * @phys: array for 'EXYNOS5_DRDPHYS_NUM' number of PHY
 *	    instances each with its 'phy' and 'phy_cfg'.
 * @extrefclk: frequency select settings when using 'separate
 *	       reference clocks' for SS and HS operations
 * @regulators: regulators for phy
 */
struct exynos5_usbdrd_phy {
	struct device *dev;
	void __iomem *reg_phy;
	void __iomem *reg_pcs;
	void __iomem *reg_pma;
	struct clk_bulk_data *clks;
	struct clk_bulk_data *core_clks;
	const struct exynos5_usbdrd_phy_drvdata *drv_data;
	struct phy_usb_instance {
		struct phy *phy;
		u32 index;
		struct regmap *reg_pmu;
		u32 pmu_offset;
		const struct exynos5_usbdrd_phy_config *phy_cfg;
	} phys[EXYNOS5_DRDPHYS_NUM];
	u32 extrefclk;
	struct regulator_bulk_data *regulators;
};

static inline
struct exynos5_usbdrd_phy *to_usbdrd_phy(struct phy_usb_instance *inst)
{
	return container_of((inst), struct exynos5_usbdrd_phy,
			    phys[(inst)->index]);
}

#define EXYNOS4_PHY_ENABLE			(1 << 0)

static void exynos5_usbdrd_phy_isol(struct phy_usb_instance *inst,
				    bool isolate)
{
	unsigned int val;

	if (!inst->reg_pmu)
		return;

	val = isolate ? 0 : EXYNOS4_PHY_ENABLE;

	regmap_update_bits(inst->reg_pmu, inst->pmu_offset,
			   EXYNOS4_PHY_ENABLE, val);
}

/*
 * Sets the utmi phy's clk as EXTREFCLK (XXTI) which is internal clock
 * from clock core. Further sets the FSEL values for HighSpeed operations.
 */
static unsigned int
exynos5_usbdrd_utmi_set_refclk(struct phy_usb_instance *inst)
{
	u32 reg;
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	/* restore any previous reference clock settings */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	reg &= ~PHYCLKRST_REFCLKSEL_MASK;
	reg |=	PHYCLKRST_REFCLKSEL_EXT_REFCLK;

	reg &= ~PHYCLKRST_FSEL_UTMI_MASK |
		PHYCLKRST_MPLL_MULTIPLIER_MASK |
		PHYCLKRST_SSC_REFCLKSEL_MASK;
	//reg |= PHYCLKRST_FSEL(phy_drd->extrefclk);
	return reg;
}

#define PHYRESUME_FORCE_QACT			BIT(9)

#define EXYNOS8895_DRD_PHYSELECTION		0x6c
#define PHYSEL_UTMI_CLK				BIT(4)
#define PHYSEL_PIPE_CLK				BIT(3)
#define PHYSEL_UTMI				BIT(2)
#define PHYSEL_PIPE				BIT(1)
#define PHYSEL_SIDEBAND				BIT(0)

#define PHYCLKRST_FSEL_MASK			(0x3f << 5)

#define PHYUTMI_VBUSVLDEXTSEL			BIT(10)
#define PHYUTMI_VBUSVLDEXT			BIT(9)
#define PHYUTMI_DRVVBUS				BIT(4)

#define LINKPORT_HOST_U3_PORT_DISABLE		BIT(8)
#define LINKPORT_HOST_PORT_OVCR_U3		BIT(5)
#define LINKPORT_HOST_PORT_OVCR_U2		BIT(4)
#define LINKPORT_HOST_PORT_OVCR_U3_SEL		BIT(3)
#define LINKPORT_HOST_PORT_OVCR_U2_SEL		BIT(2)

#define LINKSYSTEM_FORCE_VBUSVALID		BIT(8)
#define LINKSYSTEM_FORCE_BVALID			BIT(7)

static void exynos8895_usbdrd_utmi_init(struct exynos5_usbdrd_phy *phy_drd)
{
	u32 reg;

	/* Select the PHY MUX */
	reg = readl(phy_drd->reg_phy + EXYNOS8895_DRD_PHYSELECTION);
	reg &= ~(PHYSEL_PIPE | PHYSEL_PIPE_CLK | PHYSEL_UTMI_CLK |
		 PHYSEL_UTMI | PHYSEL_SIDEBAND);
	writel(reg, phy_drd->reg_phy + EXYNOS8895_DRD_PHYSELECTION);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	/* Assert a port reset */
	reg |= PHYCLKRST_PORTRESET;

	/* Select Reference clock source path */
	reg &= ~PHYCLKRST_REFCLKSEL_MASK;
	reg |= PHYCLKRST_REFCLKSEL_PAD_REFCLK;

	/* Select ref clk */
	reg &= ~PHYCLKRST_FSEL_MASK;
	//reg |= PHYCLKRST_FSEL(phy_drd->extrefclk);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);
	reg |= PHYCLKRST_COMMONONN | PHYCLKRST_EN_UTMISUSPEND;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);
	/* Dessert the port reset */
	reg &= ~PHYCLKRST_PORTRESET;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);
	reg &= ~(PHYTEST_POWERDOWN_HSP | PHYTEST_POWERDOWN_SSP);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);
	/* Enable UTMI+ */
	reg &= ~(PHYUTMI_FORCESUSPEND | PHYUTMI_FORCESLEEP |
		 UTMI_DP_PULLDOWN | UTMI_DM_PULLDOWN);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);
}

static int exynos8895_usbdrd_phy_init(struct phy *phy)
{
	int ret;
	u32 reg;
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);
#if 0
	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;
#endif
	/* Reset USB 3.0 PHY */
	writel(0x0, phy_drd->reg_phy + EXYNOS5_DRD_PHYREG0);
	writel(0x0, phy_drd->reg_phy + EXYNOS5_DRD_PHYRESUME);

	/*
	 * Setting the Frame length Adj value[6:1] to default 0x20
	 * See xHCI 1.0 spec, 5.2.4
	 */
	reg =	LINKSYSTEM_XHCI_VERSION_CONTROL |
		LINKSYSTEM_FLADJ(0x20);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_LINKSYSTEM);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM0);
	/* Select PHY CLK source */
	reg &= ~PHYPARAM0_REF_USE_PAD;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYPARAM0);

	/*
	 * Disable HWACG (hardware auto clock gating control). This will force
	 * QACTIVE signal in Q-Channel interface to HIGH level, to make sure
	 * the PHY clock is not gated by the hardware.
	 */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYRESUME);
	reg |= PHYRESUME_FORCE_QACT;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYRESUME);

	/* UTMI or PIPE3 specific init */
	inst->phy_cfg->phy_init(phy_drd);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_LINKSYSTEM);
	reg |= LINKSYSTEM_FORCE_BVALID | LINKSYSTEM_FORCE_VBUSVALID;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_LINKSYSTEM);

	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);
	reg |= PHYUTMI_VBUSVLDEXTSEL | PHYUTMI_VBUSVLDEXT |
	       PHYUTMI_OTGDISABLE;
	reg &= ~PHYUTMI_DRVVBUS;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);

	/* Set up OVC */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_LINKPORT);
	reg |= LINKPORT_HOST_PORT_OVCR_U2_SEL |
	       LINKPORT_HOST_PORT_OVCR_U3_SEL;
	reg &= ~LINKPORT_HOST_U3_PORT_DISABLE;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_LINKPORT);

	/* reference clock settings */
	reg = inst->phy_cfg->set_refclk(inst);
#if 0
	clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);
#endif
	return 0;
}

static int exynos5_usbdrd_phy_exit(struct phy *phy)
{
	int ret;
	u32 reg;
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_clks, phy_drd->clks);
	if (ret)
		return ret;

	reg =	PHYUTMI_OTGDISABLE |
		PHYUTMI_FORCESUSPEND |
		PHYUTMI_FORCESLEEP;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYUTMI);

	/* Resetting the PHYCLKRST enable bits to reduce leakage current */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);
	reg &= ~(PHYCLKRST_REF_SSP_EN |
		 PHYCLKRST_SSC_EN |
		 PHYCLKRST_COMMONONN);
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYCLKRST);

	/* Control PHYTEST to remove leakage current */
	reg = readl(phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);
	reg |=	PHYTEST_POWERDOWN_SSP |
		PHYTEST_POWERDOWN_HSP;
	writel(reg, phy_drd->reg_phy + EXYNOS5_DRD_PHYTEST);


	//clk_bulk_disable_unprepare(phy_drd->drv_data->n_clks, phy_drd->clks);

	return 0;
}

static int exynos5_usbdrd_phy_power_on(struct phy *phy)
{
	int ret;
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	pr_info("lololollolololololololololoo\n\n\n\n");
	dev_dbg(phy_drd->dev, "Request to power_on usbdrd_phy phy\n");

	ret = clk_bulk_prepare_enable(phy_drd->drv_data->n_core_clks,
				      phy_drd->core_clks);
	if (ret)
		return ret;

	/* Enable VBUS supply */
	//ret = regulator_bulk_enable(phy_drd->drv_data->n_regulators,
	//			    phy_drd->regulators);
	if (ret) {
		dev_err(phy_drd->dev, "Failed to enable PHY regulator(s)\n");
		goto fail_vbus;
	}

	/* Power-on PHY */
	inst->phy_cfg->phy_isol(inst, false);

	return 0;

fail_vbus:
	clk_bulk_disable_unprepare(phy_drd->drv_data->n_core_clks,
				   phy_drd->core_clks);

	return ret;
}

static int exynos5_usbdrd_phy_power_off(struct phy *phy)
{
	struct phy_usb_instance *inst = phy_get_drvdata(phy);
	struct exynos5_usbdrd_phy *phy_drd = to_usbdrd_phy(inst);

	dev_dbg(phy_drd->dev, "Request to power_off usbdrd_phy phy\n");

	/* Power-off the PHY */
	inst->phy_cfg->phy_isol(inst, true);

	/* Disable VBUS supply */
	//regulator_bulk_disable(phy_drd->drv_data->n_regulators,
	//		       phy_drd->regulators);

	clk_bulk_disable_unprepare(phy_drd->drv_data->n_core_clks,
				   phy_drd->core_clks);

	return 0;
}

static struct phy *exynos5_usbdrd_phy_xlate(struct device *dev,
					    struct of_phandle_args *args)
{
	struct exynos5_usbdrd_phy *phy_drd;
	int ret;

	ret = dev_get_drvdata(dev, (const void **)&phy_drd);

	if (WARN_ON(args->args[0] >= EXYNOS5_DRDPHYS_NUM))
		return ERR_PTR(-ENODEV);

	return phy_drd->phys[args->args[0]].phy;
}

static const struct phy_ops exynos8895_usbdrd_phy_ops = {
	.init		= exynos8895_usbdrd_phy_init,
	.exit		= exynos5_usbdrd_phy_exit,
	.power_on	= exynos5_usbdrd_phy_power_on,
	.power_off	= exynos5_usbdrd_phy_power_off,
};

static const char * const exynos5_regulator_names[] = {
	"vbus", "vbus-boost",
};

static const char * const exynos8895_core_clk_names[] = {
	"ref", "aclk", "ahb", "core", "xiu",
};

static const char * const exynos8895_clk_names[] = {
	"phy",
};

static const struct exynos5_usbdrd_phy_config phy_cfg_exynos8895[] = {
	{
		.id		= EXYNOS5_DRDPHY_UTMI,
		.phy_isol	= exynos5_usbdrd_phy_isol,
		.phy_init	= exynos8895_usbdrd_utmi_init,
		.set_refclk	= exynos5_usbdrd_utmi_set_refclk,
	},
};

#define EXYNOS5_USBDRD_PHY_CONTROL				0x0704
#define EXYNOS5433_USBHOST30_PHY_CONTROL			(0x0728)

static const struct exynos5_usbdrd_phy_drvdata exynos8895_usbdrd_phy = {
	.phy_cfg		= phy_cfg_exynos8895,
	.phy_ops		= &exynos8895_usbdrd_phy_ops,
	.pmu_offset_usbdrd0_phy	= EXYNOS5_USBDRD_PHY_CONTROL,
	.pmu_offset_usbdrd1_phy	= EXYNOS5433_USBHOST30_PHY_CONTROL,
	.clk_names		= exynos8895_clk_names,
	.n_clks			= ARRAY_SIZE(exynos8895_clk_names),
	.core_clk_names		= exynos8895_core_clk_names,
	.n_core_clks		= ARRAY_SIZE(exynos8895_core_clk_names),
	.regulator_names	= exynos5_regulator_names,
	.n_regulators		= ARRAY_SIZE(exynos5_regulator_names),
};

static const struct of_device_id exynos5_usbdrd_phy_of_match[] = {
	{ .compatible = "samsung,exynos8895-usbdrd-phy",
	  .data = &exynos8895_usbdrd_phy },
	{ },
};
MODULE_DEVICE_TABLE(of, exynos5_usbdrd_phy_of_match);

/**
 * regulator_bulk_set_supply_names - initialize the 'supply' fields in an array
 *                                   of regulator_bulk_data structs
 *
 * @consumers: array of regulator_bulk_data entries to initialize
 * @supply_names: array of supply name strings
 * @num_supplies: number of supply names to initialize
 *
 * Note: the 'consumers' array must be the size of 'num_supplies'.
 */
void regulator_bulk_set_supply_names(struct regulator_bulk_data *consumers,
				     const char *const *supply_names,
				     unsigned int num_supplies)
{
	unsigned int i;

	for (i = 0; i < num_supplies; i++)
		consumers[i].supply = supply_names[i];
}
EXPORT_SYMBOL_GPL(regulator_bulk_set_supply_names);

static int exynos5_usbdrd_phy_probe(struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct exynos5_usbdrd_phy *phy_drd;
	struct phy_provider *phy_provider;
	const struct exynos5_usbdrd_phy_drvdata *drv_data;
	struct regmap *reg_pmu;
	u32 pmu_offset;
	int i, ret;
	int channel;

	phy_drd = xzalloc(sizeof(*phy_drd));
	if (!phy_drd)
		return -ENOMEM;

	dev->priv = phy_drd;
	phy_drd->dev = dev;

	drv_data = device_get_match_data(dev);
	if (!drv_data)
		return -EINVAL;
	phy_drd->drv_data = drv_data;

	/* DTB with just a single region */
	phy_drd->reg_phy = dev_request_mem_resource(dev, 0);
	if (IS_ERR(phy_drd->reg_phy))
		return PTR_ERR(phy_drd->reg_phy);

	reg_pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						   "samsung,pmu-syscon");
	if (IS_ERR(reg_pmu)) {
		dev_err(dev, "Failed to lookup PMU regmap\n");
		return PTR_ERR(reg_pmu);
	}

	/* Get regulators */
	phy_drd->regulators = kzalloc(sizeof(phy_drd->regulators), GFP_KERNEL);
	if (!phy_drd->regulators)
		return -ENOMEM;

	regulator_bulk_set_supply_names(phy_drd->regulators,
					drv_data->regulator_names,
					drv_data->n_regulators);
	ret = regulator_bulk_get(dev, drv_data->n_regulators,
				      phy_drd->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	dev_vdbg(dev, "Creating usbdrd_phy phy\n");

	for (i = 0; i < EXYNOS5_DRDPHYS_NUM; i++) {
		struct phy *phy = phy_create(dev, NULL, drv_data->phy_ops);

		if (IS_ERR(phy)) {
			dev_err(dev, "Failed to create usbdrd_phy phy\n");
			return PTR_ERR(phy);
		}
		phy_drd->phys[i].phy = phy;
		phy_drd->phys[i].index = i;
		phy_drd->phys[i].reg_pmu = reg_pmu;
		pmu_offset = drv_data->pmu_offset_usbdrd0_phy;
		phy_drd->phys[i].pmu_offset = pmu_offset;
		phy_drd->phys[i].phy_cfg = &drv_data->phy_cfg[i];
		phy_set_drvdata(phy, &phy_drd->phys[i]);
	}
	phy_provider = of_phy_provider_register(dev,
						     exynos5_usbdrd_phy_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(phy_drd->dev, "Failed to register phy provider\n");
		return PTR_ERR(phy_provider);
	}

	return 0;
}

static struct driver exynos5_usb3drd_phy = {
	.probe	= exynos5_usbdrd_phy_probe,
	.name	= "exynos5-usbdrd-phy",
	.of_compatible = exynos5_usbdrd_phy_of_match,
};
coredevice_platform_driver(exynos5_usb3drd_phy);
