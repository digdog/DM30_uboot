/* DO NOT EDIT THIS FILE
 * Automatically generated by generate-def-headers.xsl
 * DO NOT EDIT THIS FILE
 */

#ifndef __BFIN_DEF_ADSP_BF532_proc__
#define __BFIN_DEF_ADSP_BF532_proc__

#include "../mach-common/ADSP-EDN-core_def.h"

#include "../mach-common/ADSP-EDN-extended_def.h"

#define L1_INST_SRAM 0xFFA08000 /* 0xFFA08000 -> 0xFFA0BFFF Instruction Bank A SRAM */
#define L1_INST_SRAM_SIZE (0xFFA0BFFF - 0xFFA08000 + 1)
#define L1_INST_SRAM_END (L1_INST_SRAM + L1_INST_SRAM_SIZE)
#define L1_SRAM_SCRATCH 0xFFB00000 /* 0xFFB00000 -> 0xFFB00FFF Scratchpad SRAM */
#define L1_SRAM_SCRATCH_SIZE (0xFFB00FFF - 0xFFB00000 + 1)
#define L1_SRAM_SCRATCH_END (L1_SRAM_SCRATCH + L1_SRAM_SCRATCH_SIZE)
#define SYSMMR_BASE 0xFFC00000 /* 0xFFC00000 -> 0xFFFFFFFF MMR registers */
#define SYSMMR_BASE_SIZE (0xFFFFFFFF - 0xFFC00000 + 1)
#define SYSMMR_BASE_END (SYSMMR_BASE + SYSMMR_BASE_SIZE)

#endif /* __BFIN_DEF_ADSP_BF532_proc__ */
