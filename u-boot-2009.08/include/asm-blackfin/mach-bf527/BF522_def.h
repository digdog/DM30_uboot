/* DO NOT EDIT THIS FILE
 * Automatically generated by generate-def-headers.xsl
 * DO NOT EDIT THIS FILE
 */

#ifndef __BFIN_DEF_ADSP_BF522_proc__
#define __BFIN_DEF_ADSP_BF522_proc__

#include "../mach-common/ADSP-EDN-core_def.h"

#include "ADSP-EDN-BF52x-extended_def.h"

#define PLL_CTL                        0xFFC00000 /* PLL Control Register */
#define PLL_DIV                        0xFFC00004 /* PLL Divide Register */
#define VR_CTL                         0xFFC00008 /* Voltage Regulator Control Register */
#define PLL_STAT                       0xFFC0000C /* PLL Status Register */
#define PLL_LOCKCNT                    0xFFC00010 /* PLL Lock Count Register */
#define CHIPID                         0xFFC00014
#define SWRST                          0xFFC00100 /* Software Reset Register */
#define SYSCR                          0xFFC00104 /* System Configuration register */
#define SRAM_BASE_ADDR                 0xFFE00000 /* SRAM Base Address (Read Only) */
#define DMEM_CONTROL                   0xFFE00004 /* Data memory control */
#define DCPLB_STATUS                   0xFFE00008 /* Data Cache Programmable Look-Aside Buffer Status */
#define DCPLB_FAULT_ADDR               0xFFE0000C /* Data Cache Programmable Look-Aside Buffer Fault Address */
#define DCPLB_ADDR0                    0xFFE00100 /* Data Cache Protection Lookaside Buffer 0 */
#define DCPLB_ADDR1                    0xFFE00104 /* Data Cache Protection Lookaside Buffer 1 */
#define DCPLB_ADDR2                    0xFFE00108 /* Data Cache Protection Lookaside Buffer 2 */
#define DCPLB_ADDR3                    0xFFE0010C /* Data Cache Protection Lookaside Buffer 3 */
#define DCPLB_ADDR4                    0xFFE00110 /* Data Cache Protection Lookaside Buffer 4 */
#define DCPLB_ADDR5                    0xFFE00114 /* Data Cache Protection Lookaside Buffer 5 */
#define DCPLB_ADDR6                    0xFFE00118 /* Data Cache Protection Lookaside Buffer 6 */
#define DCPLB_ADDR7                    0xFFE0011C /* Data Cache Protection Lookaside Buffer 7 */
#define DCPLB_ADDR8                    0xFFE00120 /* Data Cache Protection Lookaside Buffer 8 */
#define DCPLB_ADDR9                    0xFFE00124 /* Data Cache Protection Lookaside Buffer 9 */
#define DCPLB_ADDR10                   0xFFE00128 /* Data Cache Protection Lookaside Buffer 10 */
#define DCPLB_ADDR11                   0xFFE0012C /* Data Cache Protection Lookaside Buffer 11 */
#define DCPLB_ADDR12                   0xFFE00130 /* Data Cache Protection Lookaside Buffer 12 */
#define DCPLB_ADDR13                   0xFFE00134 /* Data Cache Protection Lookaside Buffer 13 */
#define DCPLB_ADDR14                   0xFFE00138 /* Data Cache Protection Lookaside Buffer 14 */
#define DCPLB_ADDR15                   0xFFE0013C /* Data Cache Protection Lookaside Buffer 15 */
#define DCPLB_DATA0                    0xFFE00200 /* Data Cache 0 Status */
#define DCPLB_DATA1                    0xFFE00204 /* Data Cache 1 Status */
#define DCPLB_DATA2                    0xFFE00208 /* Data Cache 2 Status */
#define DCPLB_DATA3                    0xFFE0020C /* Data Cache 3 Status */
#define DCPLB_DATA4                    0xFFE00210 /* Data Cache 4 Status */
#define DCPLB_DATA5                    0xFFE00214 /* Data Cache 5 Status */
#define DCPLB_DATA6                    0xFFE00218 /* Data Cache 6 Status */
#define DCPLB_DATA7                    0xFFE0021C /* Data Cache 7 Status */
#define DCPLB_DATA8                    0xFFE00220 /* Data Cache 8 Status */
#define DCPLB_DATA9                    0xFFE00224 /* Data Cache 9 Status */
#define DCPLB_DATA10                   0xFFE00228 /* Data Cache 10 Status */
#define DCPLB_DATA11                   0xFFE0022C /* Data Cache 11 Status */
#define DCPLB_DATA12                   0xFFE00230 /* Data Cache 12 Status */
#define DCPLB_DATA13                   0xFFE00234 /* Data Cache 13 Status */
#define DCPLB_DATA14                   0xFFE00238 /* Data Cache 14 Status */
#define DCPLB_DATA15                   0xFFE0023C /* Data Cache 15 Status */
#define DTEST_COMMAND                  0xFFE00300 /* Data Test Command Register */
#define DTEST_DATA0                    0xFFE00400 /* Data Test Data Register */
#define DTEST_DATA1                    0xFFE00404 /* Data Test Data Register */
#define IMEM_CONTROL                   0xFFE01004 /* Instruction Memory Control */
#define ICPLB_STATUS                   0xFFE01008 /* Instruction Cache Programmable Look-Aside Buffer Status */
#define ICPLB_FAULT_ADDR               0xFFE0100C /* Instruction Cache Programmable Look-Aside Buffer Fault Address */
#define ICPLB_ADDR0                    0xFFE01100 /* Instruction Cacheability Protection Lookaside Buffer 0 */
#define ICPLB_ADDR1                    0xFFE01104 /* Instruction Cacheability Protection Lookaside Buffer 1 */
#define ICPLB_ADDR2                    0xFFE01108 /* Instruction Cacheability Protection Lookaside Buffer 2 */
#define ICPLB_ADDR3                    0xFFE0110C /* Instruction Cacheability Protection Lookaside Buffer 3 */
#define ICPLB_ADDR4                    0xFFE01110 /* Instruction Cacheability Protection Lookaside Buffer 4 */
#define ICPLB_ADDR5                    0xFFE01114 /* Instruction Cacheability Protection Lookaside Buffer 5 */
#define ICPLB_ADDR6                    0xFFE01118 /* Instruction Cacheability Protection Lookaside Buffer 6 */
#define ICPLB_ADDR7                    0xFFE0111C /* Instruction Cacheability Protection Lookaside Buffer 7 */
#define ICPLB_ADDR8                    0xFFE01120 /* Instruction Cacheability Protection Lookaside Buffer 8 */
#define ICPLB_ADDR9                    0xFFE01124 /* Instruction Cacheability Protection Lookaside Buffer 9 */
#define ICPLB_ADDR10                   0xFFE01128 /* Instruction Cacheability Protection Lookaside Buffer 10 */
#define ICPLB_ADDR11                   0xFFE0112C /* Instruction Cacheability Protection Lookaside Buffer 11 */
#define ICPLB_ADDR12                   0xFFE01130 /* Instruction Cacheability Protection Lookaside Buffer 12 */
#define ICPLB_ADDR13                   0xFFE01134 /* Instruction Cacheability Protection Lookaside Buffer 13 */
#define ICPLB_ADDR14                   0xFFE01138 /* Instruction Cacheability Protection Lookaside Buffer 14 */
#define ICPLB_ADDR15                   0xFFE0113C /* Instruction Cacheability Protection Lookaside Buffer 15 */
#define ICPLB_DATA0                    0xFFE01200 /* Instruction Cache 0 Status */
#define ICPLB_DATA1                    0xFFE01204 /* Instruction Cache 1 Status */
#define ICPLB_DATA2                    0xFFE01208 /* Instruction Cache 2 Status */
#define ICPLB_DATA3                    0xFFE0120C /* Instruction Cache 3 Status */
#define ICPLB_DATA4                    0xFFE01210 /* Instruction Cache 4 Status */
#define ICPLB_DATA5                    0xFFE01214 /* Instruction Cache 5 Status */
#define ICPLB_DATA6                    0xFFE01218 /* Instruction Cache 6 Status */
#define ICPLB_DATA7                    0xFFE0121C /* Instruction Cache 7 Status */
#define ICPLB_DATA8                    0xFFE01220 /* Instruction Cache 8 Status */
#define ICPLB_DATA9                    0xFFE01224 /* Instruction Cache 9 Status */
#define ICPLB_DATA10                   0xFFE01228 /* Instruction Cache 10 Status */
#define ICPLB_DATA11                   0xFFE0122C /* Instruction Cache 11 Status */
#define ICPLB_DATA12                   0xFFE01230 /* Instruction Cache 12 Status */
#define ICPLB_DATA13                   0xFFE01234 /* Instruction Cache 13 Status */
#define ICPLB_DATA14                   0xFFE01238 /* Instruction Cache 14 Status */
#define ICPLB_DATA15                   0xFFE0123C /* Instruction Cache 15 Status */
#define ITEST_COMMAND                  0xFFE01300 /* Instruction Test Command Register */
#define ITEST_DATA0                    0xFFE01400 /* Instruction Test Data Register */
#define ITEST_DATA1                    0xFFE01404 /* Instruction Test Data Register */
#define EVT0                           0xFFE02000 /* Event Vector 0 ESR Address */
#define EVT1                           0xFFE02004 /* Event Vector 1 ESR Address */
#define EVT2                           0xFFE02008 /* Event Vector 2 ESR Address */
#define EVT3                           0xFFE0200C /* Event Vector 3 ESR Address */
#define EVT4                           0xFFE02010 /* Event Vector 4 ESR Address */
#define EVT5                           0xFFE02014 /* Event Vector 5 ESR Address */
#define EVT6                           0xFFE02018 /* Event Vector 6 ESR Address */
#define EVT7                           0xFFE0201C /* Event Vector 7 ESR Address */
#define EVT8                           0xFFE02020 /* Event Vector 8 ESR Address */
#define EVT9                           0xFFE02024 /* Event Vector 9 ESR Address */
#define EVT10                          0xFFE02028 /* Event Vector 10 ESR Address */
#define EVT11                          0xFFE0202C /* Event Vector 11 ESR Address */
#define EVT12                          0xFFE02030 /* Event Vector 12 ESR Address */
#define EVT13                          0xFFE02034 /* Event Vector 13 ESR Address */
#define EVT14                          0xFFE02038 /* Event Vector 14 ESR Address */
#define EVT15                          0xFFE0203C /* Event Vector 15 ESR Address */
#define ILAT                           0xFFE0210C /* Interrupt Latch Register */
#define IMASK                          0xFFE02104 /* Interrupt Mask Register */
#define IPEND                          0xFFE02108 /* Interrupt Pending Register */
#define IPRIO                          0xFFE02110 /* Interrupt Priority Register */
#define TCNTL                          0xFFE03000 /* Core Timer Control Register */
#define TPERIOD                        0xFFE03004 /* Core Timer Period Register */
#define TSCALE                         0xFFE03008 /* Core Timer Scale Register */
#define TCOUNT                         0xFFE0300C /* Core Timer Count Register */
#define L1_DATA_A_SRAM 0xFF800000 /* 0xFF800000 -> 0xFF803FFF Data Bank A SRAM */
#define L1_DATA_A_SRAM_SIZE (0xFF803FFF - 0xFF800000 + 1)
#define L1_DATA_A_SRAM_END (L1_DATA_A_SRAM + L1_DATA_A_SRAM_SIZE)
#define L1_DATA_B_SRAM 0xFF900000 /* 0xFF900000 -> 0xFF903FFF Data Bank B SRAM */
#define L1_DATA_B_SRAM_SIZE (0xFF903FFF - 0xFF900000 + 1)
#define L1_DATA_B_SRAM_END (L1_DATA_B_SRAM + L1_DATA_B_SRAM_SIZE)
#define L1_INST_SRAM 0xFFA00000 /* 0xFFA00000 -> 0xFFA07FFF Instruction Bank A SRAM */
#define L1_INST_SRAM_SIZE (0xFFA07FFF - 0xFFA00000 + 1)
#define L1_INST_SRAM_END (L1_INST_SRAM + L1_INST_SRAM_SIZE)
#define L1_SRAM_SCRATCH 0xFFB00000 /* 0xFFB00000 -> 0xFFB00FFF Scratchpad SRAM */
#define L1_SRAM_SCRATCH_SIZE (0xFFB00FFF - 0xFFB00000 + 1)
#define L1_SRAM_SCRATCH_END (L1_SRAM_SCRATCH + L1_SRAM_SCRATCH_SIZE)
#define SYSMMR_BASE 0xFFC00000 /* 0xFFC00000 -> 0xFFFFFFFF MMR registers */
#define SYSMMR_BASE_SIZE (0xFFFFFFFF - 0xFFC00000 + 1)
#define SYSMMR_BASE_END (SYSMMR_BASE + SYSMMR_BASE_SIZE)

#endif /* __BFIN_DEF_ADSP_BF522_proc__ */
