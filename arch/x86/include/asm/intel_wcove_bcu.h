#ifndef __INTEL_WCOVE_BCU_H__
#define __INTEL_WCOVE_BCU_H__

#define DRIVER_NAME		"wcove_bcu"
#define DEVICE_NAME		"bxt_wcove_bcu"

/* IRQ registers */
#define IRQLVL1_REG		0x4E02
#define BCUIRQ_REG		0x4E07
#define MIRQLVL1_REG            0x4E0E

/*IRQ Mask Register*/
#define MBCUIRQ_REG		0x4E15

/* Status registers */
#define S_BCUIRQ_REG		0x4E8B
#define S_BCUCTRL_REG		0x4E8C

/* Voltage Trip Point Configuration Register */
#define VWARN1_CFG_REG		0x4E80
#define VWARN2_CFG_REG		0x4E81
#define VCRIT_CFG_REG		0x4E82

/* Current Trip Point Configuration Register */
#define ICCMAXVSYS_CFG_REG	0x4E8D
#define ICCMAXVNN_CFG_REG	0x4E90
#define ICCMAXVCC_CFG_REG	0x4E91

/* Output Pin Behavior Register */
#define BCUDISW2_BEH_REG	0x4E83
#define BCUDISCRIT_BEH_REG	0x4E84

#define MAX_VOLTAGE_TRIP_POINTS	3
#define MAX_CURRENT_TRIP_POINTS	3

#define VWARN1_EN		BIT(3)
#define ICCMAXVCC_EN		BIT(7)

#define MVCRIT			BIT(2)
#define MVWARN2			BIT(1)
#define MVWARN1			BIT(0)
#define BCU_DEF_INTR_MASK	(MVWARN2 | MVWARN1 | MVCRIT)

#define VWARN1			BIT(0)
#define VWARN2			BIT(1)
#define VCRIT			BIT(2)
#define GSMPULSE		BIT(3)
#define TXPWRTH			BIT(4)
#define BCU_INTS		(VWARN1 | VWARN2 | VCRIT | GSMPULSE | TXPWRTH)

#define S_VWARN1		BIT(0)
#define S_VWARN2		BIT(1)
#define S_VCRIT			BIT(2)

#define S_BCUDISW2		BIT(1)
#define S_BCUDISCRIT		BIT(0)

/* Max length of the register name string */
#define MAX_REGNAME_LEN		20

/* Max number register from platform config */
#define MAX_BCUCFG_REGS		9

/* delay interval for unmasking vwarnb interrupt */
#define VWARN2_INTR_EN_DELAY	(30 * HZ)

/* check whether bit is sticky or not by checking bit 2 */
#define IS_BCUDISB_STICKY(data)		(!!(data & BIT(2)))

/* Check  BCUDISB Output Pin enable on assertion of VWARN1 crossing */
#define IS_ASSRT_ON_BCUDISB(data)	(!!(data & BIT(0)))

/* Macro to get the mode of access for the BCU registers	*/
#define MODE(r)	(((r != BCUIRQ_REG) && (r != IRQLVL1_REG) && \
			(r != S_BCUIRQ_REG))	\
			? (S_IRUGO | S_IWUSR) : S_IRUGO)

/* Generic macro to assign the parameters (reg name and address) */
#define reg_info(x)	{ .name = #x, .addr = x, .mode = MODE(x) }

#define IS_BATTERY(psy) (psy->desc->type == POWER_SUPPLY_TYPE_BATTERY)
#define IS_CHARGER(psy) (psy->desc->type == POWER_SUPPLY_TYPE_USB ||\
			psy->desc->type == POWER_SUPPLY_TYPE_USB_CDP || \
			psy->desc->type == POWER_SUPPLY_TYPE_USB_DCP || \
			psy->desc->type == POWER_SUPPLY_TYPE_USB_ACA || \
			psy->desc->type == POWER_SUPPLY_TYPE_USB_TYPEC)

enum psy_type {
	PSY_TYPE_UNKNOWN,
	PSY_TYPE_BATTERY,
	PSY_TYPE_CHARGER,
};

/**
 * These values are read from platform.
 * platform get these entries - default register configurations
 * BCU is programmed to these default values during boot time.
 */
struct wcpmic_bcu_config_data {
	u16 addr;
	u8 data;
};

struct wcove_bcu_platform_data {
	struct wcpmic_bcu_config_data config[MAX_BCUCFG_REGS];
	int num_regs;
};

struct bcu_reg_info {
	char	name[MAX_REGNAME_LEN];	/* register name   */
	u16	addr;			/* offset address  */
	u16	mode;			/* permission mode */
};

#endif /* __INTEL_WCOVE_BCU_H__ */
