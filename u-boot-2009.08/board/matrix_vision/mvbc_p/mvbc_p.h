#ifndef __MVBC_H__
#define __MVBC_H__

#define LED_G0          MPC5XXX_GPIO_SIMPLE_PSC2_0
#define LED_G1          MPC5XXX_GPIO_SIMPLE_PSC2_1
#define LED_Y           MPC5XXX_GPIO_SIMPLE_PSC2_2
#define LED_R           MPC5XXX_GPIO_SIMPLE_PSC2_3
#define ARB_X_EN        MPC5XXX_GPIO_WKUP_PSC2_4

#define FPGA_DIN        MPC5XXX_GPIO_SIMPLE_PSC3_0
#define FPGA_CCLK       MPC5XXX_GPIO_SIMPLE_PSC3_1
#define FPGA_CONF_DONE  MPC5XXX_GPIO_SIMPLE_PSC3_2
#define FPGA_CONFIG     MPC5XXX_GPIO_SIMPLE_PSC3_3
#define FPGA_STATUS     MPC5XXX_GPIO_SINT_PSC3_4

#define MAN_RST         MPC5XXX_GPIO_WKUP_PSC6_0
#define WD_TS           MPC5XXX_GPIO_WKUP_PSC6_1
#define WD_WDI          MPC5XXX_GPIO_SIMPLE_PSC6_2
#define COP_PRESENT     MPC5XXX_GPIO_SIMPLE_PSC6_3
#define FACT_RST        MPC5XXX_GPIO_WKUP_6
#define FLASH_RBY       MPC5XXX_GPIO_WKUP_7

#define SIMPLE_DDR      (LED_G0 | LED_G1 | LED_Y | LED_R | \
			 FPGA_DIN | FPGA_CCLK | FPGA_CONFIG | WD_WDI)
#define SIMPLE_DVO      (FPGA_CONFIG)
#define SIMPLE_ODE      (FPGA_CONFIG | LED_G0 | LED_G1 | LED_Y | LED_R)
#define SIMPLE_GPIOEN   (LED_G0 | LED_G1 | LED_Y | LED_R | \
			 FPGA_DIN | FPGA_CCLK | FPGA_CONF_DONE | FPGA_CONFIG |\
			 WD_WDI | COP_PRESENT)

#define SINT_ODE        0
#define SINT_DDR        0
#define SINT_DVO        0
#define SINT_INTEN      0
#define SINT_ITYPE      0
#define SINT_GPIOEN     (FPGA_STATUS)

#define WKUP_ODE        (MAN_RST)
#define WKUP_DIR        (ARB_X_EN|MAN_RST|WD_TS)
#define WKUP_DO         (ARB_X_EN|MAN_RST|WD_TS)
#define WKUP_EN         (ARB_X_EN|MAN_RST|WD_TS|FACT_RST|FLASH_RBY)

#endif
