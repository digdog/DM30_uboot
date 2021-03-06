#ifndef __ASM_AVR32_ARCH_GPIO_IMPL_H__
#define __ASM_AVR32_ARCH_GPIO_IMPL_H__

/* Register offsets */
struct gpio_regs {
	u32	GPER;
	u32	GPERS;
	u32	GPERC;
	u32	GPERT;
	u32	PMR0;
	u32	PMR0S;
	u32	PMR0C;
	u32	PMR0T;
	u32	PMR1;
	u32	PMR1S;
	u32	PMR1C;
	u32	PMR1T;
	u32	__reserved0[4];
	u32	ODER;
	u32	ODERS;
	u32	ODERC;
	u32	ODERT;
	u32	OVR;
	u32	OVRS;
	u32	OVRC;
	u32	OVRT;
	u32	PVR;
	u32	__reserved_PVRS;
	u32	__reserved_PVRC;
	u32	__reserved_PVRT;
	u32	PUER;
	u32	PUERS;
	u32	PUERC;
	u32	PUERT;
	u32	PDER;
	u32	PDERS;
	u32	PDERC;
	u32	PDERT;
	u32	IER;
	u32	IERS;
	u32	IERC;
	u32	IERT;
	u32	IMR0;
	u32	IMR0S;
	u32	IMR0C;
	u32	IMR0T;
	u32	IMR1;
	u32	IMR1S;
	u32	IMR1C;
	u32	IMR1T;
	u32	GFER;
	u32	GFERS;
	u32	GFERC;
	u32	GFERT;
	u32	IFR;
	u32	__reserved_IFRS;
	u32	IFRC;
	u32	__reserved_IFRT;
	u32	ODMER;
	u32	ODMERS;
	u32	ODMERC;
	u32	ODMERT;
	u32	__reserved1[4];
	u32	ODCR0;
	u32	ODCR0S;
	u32	ODCR0C;
	u32	ODCR0T;
	u32	ODCR1;
	u32	ODCR1S;
	u32	ODCR1C;
	u32	ODCR1T;
	u32	__reserved2[4];
	u32	OSRR0;
	u32	OSRR0S;
	u32	OSRR0C;
	u32	OSRR0T;
	u32	__reserved3[8];
	u32	STER;
	u32	STERS;
	u32	STERC;
	u32	STERT;
	u32	__reserved4[35];
	u32	VERSION;
};

#endif /* __ASM_AVR32_ARCH_GPIO_IMPL_H__ */
