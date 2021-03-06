/* ****************************************************************
 * Common defs for reg spec for chip ka_of
 * Auto-generated by trex2: DO NOT HAND-EDIT!!
 * ****************************************************************
 */

#ifndef HAL_KA_OF_AUTO_H
#define HAL_KA_OF_AUTO_H


/* ----------------------------------------------------------------
 * For block: 'ofem'
 */

/* ---- Block instance addressing (for block-select) */
#define OFEM_BLOCK_ADDR_BIT_L 6
#define OFEM_BLOCK_ADDR_BIT_H 9
#define OFEM_BLOCK_ADDR_WIDTH 4

#define  OFEM_ADDR  0x0

/* ---- Reg addressing (within block) */
#define OFEM_REG_ADDR_BIT_L 2
#define OFEM_REG_ADDR_BIT_H 5
#define OFEM_REG_ADDR_WIDTH 4


/* ================================================================
 * ---- Register KA_OF_OFEM_REVISION */
#define SAND_HAL_KA_OF_OFEM_REVISION_OFFSET    0x000
#ifndef SAND_HAL_KA_OF_OFEM_REVISION_NO_TEST_MASK
#define SAND_HAL_KA_OF_OFEM_REVISION_NO_TEST_MASK    0x000
#endif
#define SAND_HAL_KA_OF_OFEM_REVISION_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_REVISION_MSB     31
#define SAND_HAL_KA_OF_OFEM_REVISION_LSB      0

/* ================================================================
 * ---- Register KA_OF_OFEM_RESET */
#define SAND_HAL_KA_OF_OFEM_RESET_OFFSET    0x004
#ifndef SAND_HAL_KA_OF_OFEM_RESET_NO_TEST_MASK
#define SAND_HAL_KA_OF_OFEM_RESET_NO_TEST_MASK    0x000
#endif
#define SAND_HAL_KA_OF_OFEM_RESET_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_RESET_MSB     31
#define SAND_HAL_KA_OF_OFEM_RESET_LSB      0

/* ================================================================
 * ---- Register KA_OF_OFEM_CNTL */
#define SAND_HAL_KA_OF_OFEM_CNTL_OFFSET    0x018
#ifndef SAND_HAL_KA_OF_OFEM_CNTL_NO_TEST_MASK
#define SAND_HAL_KA_OF_OFEM_CNTL_NO_TEST_MASK    0x000
#endif
#define SAND_HAL_KA_OF_OFEM_CNTL_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_CNTL_MSB     31
#define SAND_HAL_KA_OF_OFEM_CNTL_LSB      0

/* ================================================================
 * ---- Register KA_OF_OFEM_MAC_FLOW_CTL */
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_OFFSET    0x01c
#ifndef SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_NO_TEST_MASK
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_NO_TEST_MASK    0x000
#endif
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MSB     31
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_LSB      0

/* ================================================================
 * ---- Register KA_OF_OFEM_INTERRUPT */
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_OFFSET    0x008
#ifndef SAND_HAL_KA_OF_OFEM_INTERRUPT_NO_TEST_MASK
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_NO_TEST_MASK    0x000
#endif
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MSB     31
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_LSB      0

/* ================================================================
 * ---- Register KA_OF_OFEM_INTERRUPT_MASK */
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_OFFSET    0x00c
#ifndef SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_NO_TEST_MASK
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_NO_TEST_MASK    0x000
#endif
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MSB     31
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_LSB      0

/* ================================================================
 * ---- Register KA_OF_OFEM_SCRATCH */
#define SAND_HAL_KA_OF_OFEM_SCRATCH_OFFSET    0x010
#ifndef SAND_HAL_KA_OF_OFEM_SCRATCH_NO_TEST_MASK
#define SAND_HAL_KA_OF_OFEM_SCRATCH_NO_TEST_MASK    0x000
#endif
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MSB     31
#define SAND_HAL_KA_OF_OFEM_SCRATCH_LSB      0

/* ================================================================
 * ---- Register KA_OF_OFEM_SCRATCH_MASK */
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_OFFSET    0x014
#ifndef SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_NO_TEST_MASK
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_NO_TEST_MASK    0x000
#endif
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_MSB     31
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_LSB      0

/* ================================================================
 * Field info for register KA_OF_OFEM_REVISION */
#define SAND_HAL_KA_OF_OFEM_REVISION_IDENTIFICATION_MASK    0x0000ff00
#define SAND_HAL_KA_OF_OFEM_REVISION_IDENTIFICATION_SHIFT    8
#define SAND_HAL_KA_OF_OFEM_REVISION_IDENTIFICATION_MSB    15
#define SAND_HAL_KA_OF_OFEM_REVISION_IDENTIFICATION_LSB    8
#define SAND_HAL_KA_OF_OFEM_REVISION_IDENTIFICATION_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_REVISION_IDENTIFICATION_DEFAULT    0x00000024
#define SAND_HAL_KA_OF_OFEM_REVISION_REVISION_MASK    0x000000ff
#define SAND_HAL_KA_OF_OFEM_REVISION_REVISION_SHIFT    0
#define SAND_HAL_KA_OF_OFEM_REVISION_REVISION_MSB    7
#define SAND_HAL_KA_OF_OFEM_REVISION_REVISION_LSB    0
#define SAND_HAL_KA_OF_OFEM_REVISION_REVISION_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_REVISION_REVISION_DEFAULT    0x00000000

/* ================================================================
 * Field info for register KA_OF_OFEM_RESET */
#define SAND_HAL_KA_OF_OFEM_RESET_I2C_MUX0_RESET_N_MASK    0x00000004
#define SAND_HAL_KA_OF_OFEM_RESET_I2C_MUX0_RESET_N_SHIFT    2
#define SAND_HAL_KA_OF_OFEM_RESET_I2C_MUX0_RESET_N_MSB    2
#define SAND_HAL_KA_OF_OFEM_RESET_I2C_MUX0_RESET_N_LSB    2
#define SAND_HAL_KA_OF_OFEM_RESET_I2C_MUX0_RESET_N_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_RESET_I2C_MUX0_RESET_N_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_RESET_LOCH0_RESET_N_MASK    0x00000002
#define SAND_HAL_KA_OF_OFEM_RESET_LOCH0_RESET_N_SHIFT    1
#define SAND_HAL_KA_OF_OFEM_RESET_LOCH0_RESET_N_MSB    1
#define SAND_HAL_KA_OF_OFEM_RESET_LOCH0_RESET_N_LSB    1
#define SAND_HAL_KA_OF_OFEM_RESET_LOCH0_RESET_N_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_RESET_LOCH0_RESET_N_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_RESET_MAC0_RESET_N_MASK    0x00000001
#define SAND_HAL_KA_OF_OFEM_RESET_MAC0_RESET_N_SHIFT    0
#define SAND_HAL_KA_OF_OFEM_RESET_MAC0_RESET_N_MSB    0
#define SAND_HAL_KA_OF_OFEM_RESET_MAC0_RESET_N_LSB    0
#define SAND_HAL_KA_OF_OFEM_RESET_MAC0_RESET_N_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_RESET_MAC0_RESET_N_DEFAULT    0x00000000

/* ================================================================
 * Field info for register KA_OF_OFEM_CNTL */
#define SAND_HAL_KA_OF_OFEM_CNTL_TEMP_LED_MASK    0x000000c0
#define SAND_HAL_KA_OF_OFEM_CNTL_TEMP_LED_SHIFT    6
#define SAND_HAL_KA_OF_OFEM_CNTL_TEMP_LED_MSB    7
#define SAND_HAL_KA_OF_OFEM_CNTL_TEMP_LED_LSB    6
#define SAND_HAL_KA_OF_OFEM_CNTL_TEMP_LED_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_CNTL_TEMP_LED_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_CNTL_FAULT_LED_MASK    0x00000030
#define SAND_HAL_KA_OF_OFEM_CNTL_FAULT_LED_SHIFT    4
#define SAND_HAL_KA_OF_OFEM_CNTL_FAULT_LED_MSB    5
#define SAND_HAL_KA_OF_OFEM_CNTL_FAULT_LED_LSB    4
#define SAND_HAL_KA_OF_OFEM_CNTL_FAULT_LED_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_CNTL_FAULT_LED_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_R_LED_MASK    0x0000000c
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_R_LED_SHIFT    2
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_R_LED_MSB    3
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_R_LED_LSB    2
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_R_LED_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_R_LED_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_L_LED_MASK    0x00000003
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_L_LED_SHIFT    0
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_L_LED_MSB    1
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_L_LED_LSB    0
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_L_LED_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_CNTL_RS232_L_LED_DEFAULT    0x00000000

/* ================================================================
 * Field info for register KA_OF_OFEM_MAC_FLOW_CTL */
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_LOCH_APS_SEL_MASK    0x00000100
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_LOCH_APS_SEL_SHIFT    8
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_LOCH_APS_SEL_MSB    8
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_LOCH_APS_SEL_LSB    8
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_LOCH_APS_SEL_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_LOCH_APS_SEL_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_FR_MASK    0x00000010
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_FR_SHIFT    4
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_FR_MSB    4
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_FR_LSB    4
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_FR_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_FR_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_ADDR_MASK    0x0000000f
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_ADDR_SHIFT    0
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_ADDR_MSB    3
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_ADDR_LSB    0
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_ADDR_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_MAC_FLOW_CTL_MACA_TXPAUSE_ADDR_DEFAULT    0x00000000

/* ================================================================
 * Field info for register KA_OF_OFEM_INTERRUPT */
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC_TIMEOUT_MASK    0x00000100
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC_TIMEOUT_SHIFT    8
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC_TIMEOUT_MSB    8
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC_TIMEOUT_LSB    8
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC_TIMEOUT_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC_TIMEOUT_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_A_LOSOUT_N_MASK    0x00000080
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_A_LOSOUT_N_SHIFT    7
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_A_LOSOUT_N_MSB    7
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_A_LOSOUT_N_LSB    7
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_A_LOSOUT_N_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_A_LOSOUT_N_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_B_LOSOUT_N_MASK    0x00000040
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_B_LOSOUT_N_SHIFT    6
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_B_LOSOUT_N_MSB    6
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_B_LOSOUT_N_LSB    6
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_B_LOSOUT_N_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_QK_B_LOSOUT_N_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_NR_MASK    0x00000020
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_NR_SHIFT    5
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_NR_MSB    5
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_NR_LSB    5
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_NR_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_NR_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_NR_MASK    0x00000010
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_NR_SHIFT    4
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_NR_MSB    4
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_NR_LSB    4
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_NR_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_NR_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_INT_N_MASK    0x00000008
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_INT_N_SHIFT    3
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_INT_N_MSB    3
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_INT_N_LSB    3
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_INT_N_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_A_INT_N_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_INT_N_MASK    0x00000004
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_INT_N_SHIFT    2
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_INT_N_MSB    2
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_INT_N_LSB    2
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_INT_N_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_XFP_B_INT_N_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC1_INT_N_MASK    0x00000002
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC1_INT_N_SHIFT    1
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC1_INT_N_MSB    1
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC1_INT_N_LSB    1
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC1_INT_N_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC1_INT_N_DEFAULT    0x00000000
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC0_INT_N_MASK    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC0_INT_N_SHIFT    0
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC0_INT_N_MSB    0
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC0_INT_N_LSB    0
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC0_INT_N_TYPE (SAND_HAL_TYPE_READ)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MAC0_INT_N_DEFAULT    0x00000000

/* ================================================================
 * Field info for register KA_OF_OFEM_INTERRUPT_MASK */
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC_TIMEOUT_DISINT_MASK    0x00000100
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC_TIMEOUT_DISINT_SHIFT    8
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC_TIMEOUT_DISINT_MSB    8
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC_TIMEOUT_DISINT_LSB    8
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC_TIMEOUT_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC_TIMEOUT_DISINT_DEFAULT    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_A_LOSOUT_N_DISINT_MASK    0x00000080
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_A_LOSOUT_N_DISINT_SHIFT    7
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_A_LOSOUT_N_DISINT_MSB    7
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_A_LOSOUT_N_DISINT_LSB    7
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_A_LOSOUT_N_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_A_LOSOUT_N_DISINT_DEFAULT    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_B_LOSOUT_N_DISINT_MASK    0x00000040
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_B_LOSOUT_N_DISINT_SHIFT    6
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_B_LOSOUT_N_DISINT_MSB    6
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_B_LOSOUT_N_DISINT_LSB    6
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_B_LOSOUT_N_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_QK_B_LOSOUT_N_DISINT_DEFAULT    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_NR_DISINT_MASK    0x00000020
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_NR_DISINT_SHIFT    5
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_NR_DISINT_MSB    5
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_NR_DISINT_LSB    5
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_NR_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_NR_DISINT_DEFAULT    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_NR_DISINT_MASK    0x00000010
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_NR_DISINT_SHIFT    4
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_NR_DISINT_MSB    4
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_NR_DISINT_LSB    4
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_NR_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_NR_DISINT_DEFAULT    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_INT_N_DISINT_MASK    0x00000008
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_INT_N_DISINT_SHIFT    3
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_INT_N_DISINT_MSB    3
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_INT_N_DISINT_LSB    3
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_INT_N_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_A_INT_N_DISINT_DEFAULT    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_INT_N_DISINT_MASK    0x00000004
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_INT_N_DISINT_SHIFT    2
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_INT_N_DISINT_MSB    2
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_INT_N_DISINT_LSB    2
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_INT_N_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_XFP_B_INT_N_DISINT_DEFAULT    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC1_INT_N_DISINT_MASK    0x00000002
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC1_INT_N_DISINT_SHIFT    1
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC1_INT_N_DISINT_MSB    1
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC1_INT_N_DISINT_LSB    1
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC1_INT_N_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC1_INT_N_DISINT_DEFAULT    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC0_INT_N_DISINT_MASK    0x00000001
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC0_INT_N_DISINT_SHIFT    0
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC0_INT_N_DISINT_MSB    0
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC0_INT_N_DISINT_LSB    0
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC0_INT_N_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_INTERRUPT_MASK_MAC0_INT_N_DISINT_DEFAULT    0x00000001

/* ================================================================
 * Field info for register KA_OF_OFEM_SCRATCH */
#define SAND_HAL_KA_OF_OFEM_SCRATCH_TEST_BITS_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_SCRATCH_TEST_BITS_SHIFT    0
#define SAND_HAL_KA_OF_OFEM_SCRATCH_TEST_BITS_MSB    31
#define SAND_HAL_KA_OF_OFEM_SCRATCH_TEST_BITS_LSB    0
#define SAND_HAL_KA_OF_OFEM_SCRATCH_TEST_BITS_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_SCRATCH_TEST_BITS_DEFAULT    0x00000000

/* ================================================================
 * Field info for register KA_OF_OFEM_SCRATCH_MASK */
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_TEST_BITS_DISINT_MASK    0xffffffff
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_TEST_BITS_DISINT_SHIFT    0
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_TEST_BITS_DISINT_MSB    31
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_TEST_BITS_DISINT_LSB    0
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_TEST_BITS_DISINT_TYPE (SAND_HAL_TYPE_WRITE)
#define SAND_HAL_KA_OF_OFEM_SCRATCH_MASK_TEST_BITS_DISINT_DEFAULT    0xffffffff

#endif /* matches #ifndef HAL_KA_OF_AUTO_H */
