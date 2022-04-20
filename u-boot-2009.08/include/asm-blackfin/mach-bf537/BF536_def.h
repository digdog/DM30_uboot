/* DO NOT EDIT THIS FILE
 * Automatically generated by generate-def-headers.xsl
 * DO NOT EDIT THIS FILE
 */

#ifndef __BFIN_DEF_ADSP_BF536_proc__
#define __BFIN_DEF_ADSP_BF536_proc__

#include "../mach-common/ADSP-EDN-core_def.h"

#include "ADSP-EDN-BF534-extended_def.h"

#define EMAC_OPMODE                    0xFFC03000 /* Operating Mode Register */
#define EMAC_ADDRLO                    0xFFC03004 /* Address Low (32 LSBs) Register */
#define EMAC_ADDRHI                    0xFFC03008 /* Address High (16 MSBs) Register */
#define EMAC_HASHLO                    0xFFC0300C /* Multicast Hash Table Low (Bins 31-0) Register */
#define EMAC_HASHHI                    0xFFC03010 /* Multicast Hash Table High (Bins 63-32) Register */
#define EMAC_STAADD                    0xFFC03014 /* Station Management Address Register */
#define EMAC_STADAT                    0xFFC03018 /* Station Management Data Register */
#define EMAC_FLC                       0xFFC0301C /* Flow Control Register */
#define EMAC_VLAN1                     0xFFC03020 /* VLAN1 Tag Register */
#define EMAC_VLAN2                     0xFFC03024 /* VLAN2 Tag Register */
#define EMAC_WKUP_CTL                  0xFFC0302C /* Wake-Up Control/Status Register */
#define EMAC_WKUP_FFMSK0               0xFFC03030 /* Wake-Up Frame Filter 0 Byte Mask Register */
#define EMAC_WKUP_FFMSK1               0xFFC03034 /* Wake-Up Frame Filter 1 Byte Mask Register */
#define EMAC_WKUP_FFMSK2               0xFFC03038 /* Wake-Up Frame Filter 2 Byte Mask Register */
#define EMAC_WKUP_FFMSK3               0xFFC0303C /* Wake-Up Frame Filter 3 Byte Mask Register */
#define EMAC_WKUP_FFCMD                0xFFC03040 /* Wake-Up Frame Filter Commands Register */
#define EMAC_WKUP_FFOFF                0xFFC03044 /* Wake-Up Frame Filter Offsets Register */
#define EMAC_WKUP_FFCRC0               0xFFC03048 /* Wake-Up Frame Filter 0,1 CRC-16 Register */
#define EMAC_WKUP_FFCRC1               0xFFC0304C /* Wake-Up Frame Filter 2,3 CRC-16 Register */
#define EMAC_SYSCTL                    0xFFC03060 /* EMAC System Control Register */
#define EMAC_SYSTAT                    0xFFC03064 /* EMAC System Status Register */
#define EMAC_RX_STAT                   0xFFC03068 /* RX Current Frame Status Register */
#define EMAC_RX_STKY                   0xFFC0306C /* RX Sticky Frame Status Register */
#define EMAC_RX_IRQE                   0xFFC03070 /* RX Frame Status Interrupt Enables Register */
#define EMAC_TX_STAT                   0xFFC03074 /* TX Current Frame Status Register */
#define EMAC_TX_STKY                   0xFFC03078 /* TX Sticky Frame Status Register */
#define EMAC_TX_IRQE                   0xFFC0307C /* TX Frame Status Interrupt Enables Register */
#define EMAC_MMC_CTL                   0xFFC03080 /* MMC Counter Control Register */
#define EMAC_MMC_RIRQS                 0xFFC03084 /* MMC RX Interrupt Status Register */
#define EMAC_MMC_RIRQE                 0xFFC03088 /* MMC RX Interrupt Enables Register */
#define EMAC_MMC_TIRQS                 0xFFC0308C /* MMC TX Interrupt Status Register */
#define EMAC_MMC_TIRQE                 0xFFC03090 /* MMC TX Interrupt Enables Register */
#define EMAC_RXC_OK                    0xFFC03100 /* RX Frame Successful Count */
#define EMAC_RXC_FCS                   0xFFC03104 /* RX Frame FCS Failure Count */
#define EMAC_RXC_ALIGN                 0xFFC03108 /* RX Alignment Error Count */
#define EMAC_RXC_OCTET                 0xFFC0310C /* RX Octets Successfully Received Count */
#define EMAC_RXC_DMAOVF                0xFFC03110 /* Internal MAC Sublayer Error RX Frame Count */
#define EMAC_RXC_UNICST                0xFFC03114 /* Unicast RX Frame Count */
#define EMAC_RXC_MULTI                 0xFFC03118 /* Multicast RX Frame Count */
#define EMAC_RXC_BROAD                 0xFFC0311C /* Broadcast RX Frame Count */
#define EMAC_RXC_LNERRI                0xFFC03120 /* RX Frame In Range Error Count */
#define EMAC_RXC_LNERRO                0xFFC03124 /* RX Frame Out Of Range Error Count */
#define EMAC_RXC_LONG                  0xFFC03128 /* RX Frame Too Long Count */
#define EMAC_RXC_MACCTL                0xFFC0312C /* MAC Control RX Frame Count */
#define EMAC_RXC_OPCODE                0xFFC03130 /* Unsupported Op-Code RX Frame Count */
#define EMAC_RXC_PAUSE                 0xFFC03134 /* MAC Control Pause RX Frame Count */
#define EMAC_RXC_ALLFRM                0xFFC03138 /* Overall RX Frame Count */
#define EMAC_RXC_ALLOCT                0xFFC0313C /* Overall RX Octet Count */
#define EMAC_RXC_TYPED                 0xFFC03140 /* Type/Length Consistent RX Frame Count  */
#define EMAC_RXC_SHORT                 0xFFC03144 /* RX Frame Fragment Count - Byte Count x < 64 */
#define EMAC_RXC_EQ64                  0xFFC03148 /* Good RX Frame Count - Byte Count x = 64 */
#define EMAC_RXC_LT128                 0xFFC0314C /* Good RX Frame Count - Byte Count  64 <= x < 128 */
#define EMAC_RXC_LT256                 0xFFC03150 /* Good RX Frame Count - Byte Count 128 <= x < 256 */
#define EMAC_RXC_LT512                 0xFFC03154 /* Good RX Frame Count - Byte Count 256 <= x < 512 */
#define EMAC_RXC_LT1024                0xFFC03158 /* Good RX Frame Count - Byte Count 512 <= x < 1024 */
#define EMAC_RXC_GE1024                0xFFC0315C /* Good RX Frame Count - Byte Count x >= 1024 */
#define EMAC_TXC_OK                    0xFFC03180 /* TX Frame Successful Count */
#define EMAC_TXC_1COL                  0xFFC03184 /* TX Frames Successful After Single Collision Count */
#define EMAC_TXC_GT1COL                0xFFC03188 /* TX Frames Successful After Multiple Collisions Count */
#define EMAC_TXC_OCTET                 0xFFC0318C /* TX Octets Successfully Received Count */
#define EMAC_TXC_DEFER                 0xFFC03190 /* TX Frame Delayed Due To Busy Count */
#define EMAC_TXC_LATECL                0xFFC03194 /* Late TX Collisions Count */
#define EMAC_TXC_XS_COL                0xFFC03198 /* TX Frame Failed Due To Excessive Collisions Count */
#define EMAC_TXC_DMAUND                0xFFC0319C /* Internal MAC Sublayer Error TX Frame Count */
#define EMAC_TXC_CRSERR                0xFFC031A0 /* Carrier Sense Deasserted During TX Frame Count */
#define EMAC_TXC_UNICST                0xFFC031A4 /* Unicast TX Frame Count */
#define EMAC_TXC_MULTI                 0xFFC031A8 /* Multicast TX Frame Count */
#define EMAC_TXC_BROAD                 0xFFC031AC /* Broadcast TX Frame Count */
#define EMAC_TXC_XS_DFR                0xFFC031B0 /* TX Frames With Excessive Deferral Count */
#define EMAC_TXC_MACCTL                0xFFC031B4 /* MAC Control TX Frame Count */
#define EMAC_TXC_ALLFRM                0xFFC031B8 /* Overall TX Frame Count */
#define EMAC_TXC_ALLOCT                0xFFC031BC /* Overall TX Octet Count */
#define EMAC_TXC_EQ64                  0xFFC031C0 /* Good TX Frame Count - Byte Count x = 64 */
#define EMAC_TXC_LT128                 0xFFC031C4 /* Good TX Frame Count - Byte Count  64 <= x < 128 */
#define EMAC_TXC_LT256                 0xFFC031C8 /* Good TX Frame Count - Byte Count 128 <= x < 256 */
#define EMAC_TXC_LT512                 0xFFC031CC /* Good TX Frame Count - Byte Count 256 <= x < 512 */
#define EMAC_TXC_LT1024                0xFFC031D0 /* Good TX Frame Count - Byte Count 512 <= x < 1024 */
#define EMAC_TXC_GE1024                0xFFC031D4 /* Good TX Frame Count - Byte Count x >= 1024 */
#define EMAC_TXC_ABORT                 0xFFC031D8 /* Total TX Frames Aborted Count */
#define L1_INST_SRAM 0xFFA00000 /* 0xFFA00000 -> 0xFFA07FFF Instruction Bank A SRAM */
#define L1_INST_SRAM_SIZE (0xFFA07FFF - 0xFFA00000 + 1)
#define L1_INST_SRAM_END (L1_INST_SRAM + L1_INST_SRAM_SIZE)
#define L1_SRAM_SCRATCH 0xFFB00000 /* 0xFFB00000 -> 0xFFB00FFF Scratchpad SRAM */
#define L1_SRAM_SCRATCH_SIZE (0xFFB00FFF - 0xFFB00000 + 1)
#define L1_SRAM_SCRATCH_END (L1_SRAM_SCRATCH + L1_SRAM_SCRATCH_SIZE)
#define SYSMMR_BASE 0xFFC00000 /* 0xFFC00000 -> 0xFFFFFFFF MMR registers */
#define SYSMMR_BASE_SIZE (0xFFFFFFFF - 0xFFC00000 + 1)
#define SYSMMR_BASE_END (SYSMMR_BASE + SYSMMR_BASE_SIZE)

#endif /* __BFIN_DEF_ADSP_BF536_proc__ */
