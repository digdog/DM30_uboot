/**
 * @date   April 18, 2005
 *
 * @brief  IXP400 NPE Microcode Image file
 *
 * This file was generated by the IxNpeDlImageGen tool.
 * It contains a NPE microcode image suitable for use
 * with the NPE Downloader (IxNpeDl) component in the
 * IXP400 Access Driver software library.
 *
 * @par
 * IXP400 SW Release version 2.0
 *
 * -- Copyright Notice --
 *
 * @par
 * Copyright 2001-2005, Intel Corporation.
 * All rights reserved.
 *
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * @par
 * -- End of Copyright Notice --
*/

/**
 * @defgroup IxNpeMicrocode IXP400 NPE Microcode Image Library
 *
 * @brief Library containing a set of NPE firmware images, for use
 * with NPE Downloader s/w component
 *
 * @{
 */

/**
 * @def IX_NPE_IMAGE_INCLUDE
 *
 * @brief Wrap the following Image identifiers with "#if IX_NPE_IMAGE_INCLUDE ... #endif" to include the image in the library
 */
#define IX_NPE_IMAGE_INCLUDE 1

/**
 * @def IX_NPE_IMAGE_OMIT
 *
 * @brief Wrap the following Image identifiers with "#if IX_NPE_IMAGE_OMIT ... #endif" to OMIT the image from the library
 */
#define IX_NPE_IMAGE_OMIT    0


#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_HSS0
 *
 * @brief NPE Image Id for NPE-A with HSS-0 Only feature.  It supports 32 channelized and 4 packetized.
 */
#define IX_NPEDL_NPEIMAGE_NPEA_HSS0 0x00010000
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_HSS0_ATM_SPHY_1_PORT
 *
 * @brief NPE Image Id for NPE-A with HSS-0 and ATM feature. For HSS, it supports 16/32 channelized and 4/0 packetized. For ATM, it supports AAL5, AAL0 and OAM for UTOPIA SPHY, 1 logical port, 32 VCs. It also has Fast Path support.
 */
#define IX_NPEDL_NPEIMAGE_NPEA_HSS0_ATM_SPHY_1_PORT 0x00020000
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_HSS0_ATM_MPHY_1_PORT
 *
 * @brief NPE Image Id for NPE-A with HSS-0 and ATM feature. For HSS, it supports 16/32 channelized and 4/0 packetized. For ATM, it supports AAL5, AAL0 and OAM for UTOPIA MPHY, 1 logical port, 32 VCs. It also has Fast Path support.
 */
#define IX_NPEDL_NPEIMAGE_NPEA_HSS0_ATM_MPHY_1_PORT 0x00030000
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_ATM_MPHY_12_PORT
 *
 * @brief NPE Image Id for NPE-A with ATM-Only feature. It supports AAL5, AAL0 and OAM for UTOPIA MPHY, 12 logical ports, 32 VCs. It also has Fast Path support.
 */
#define IX_NPEDL_NPEIMAGE_NPEA_ATM_MPHY_12_PORT 0x00040000
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_DMA
 *
 * @brief NPE Image Id for NPE-A with DMA-Only feature.
 */
#define IX_NPEDL_NPEIMAGE_NPEA_DMA 0x00150100
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_HSS_2_PORT
 *
 * @brief NPE Image Id for NPE-A with HSS-0 and HSS-1 feature. Each HSS port supports 32 channelized and 4 packetized.
 */
#define IX_NPEDL_NPEIMAGE_NPEA_HSS_2_PORT 0x00090000
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_ETH
 *
 * @brief NPE Image Id for NPE-A with Basic Ethernet Rx/Tx which includes: MAC_FILTERING, MAC_LEARNING, SPANNING_TREE, FIREWALL
 */
#define IX_NPEDL_NPEIMAGE_NPEA_ETH 0x10800200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_ETH_LEARN_FILTER_SPAN_FIREWALL
 *
 * @brief NPE Image Id for NPE-A with Basic Ethernet Rx/Tx which includes: MAC_FILTERING, MAC_LEARNING, SPANNING_TREE, FIREWALL
 */
#define IX_NPEDL_NPEIMAGE_NPEA_ETH_LEARN_FILTER_SPAN_FIREWALL 0x10800200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_ETH_LEARN_FILTER_SPAN_FIREWALL_VLAN_QOS
 *
 * @brief NPE Image Id for NPE-A with Ethernet Rx/Tx which includes: MAC_FILTERING, MAC_LEARNING, SPANNING_TREE, FIREWALL, VLAN_QOS
 */
#define IX_NPEDL_NPEIMAGE_NPEA_ETH_LEARN_FILTER_SPAN_FIREWALL_VLAN_QOS 0x10810200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEA_ETH_SPAN_FIREWALL_VLAN_QOS_HDR_CONV
 *
 * @brief NPE Image Id for NPE-A with Ethernet Rx/Tx which includes: SPANNING_TREE, FIREWALL, VLAN_QOS, HEADER_CONVERSION
 */
#define IX_NPEDL_NPEIMAGE_NPEA_ETH_SPAN_FIREWALL_VLAN_QOS_HDR_CONV 0x10820200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEB_ETH
 *
 * @brief NPE Image Id for NPE-B with Basic Ethernet Rx/Tx which includes: MAC_FILTERING, MAC_LEARNING, SPANNING_TREE, FIREWALL
 */
#define IX_NPEDL_NPEIMAGE_NPEB_ETH 0x01000200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEB_ETH_LEARN_FILTER_SPAN_FIREWALL
 *
 * @brief NPE Image Id for NPE-B with Basic Ethernet Rx/Tx which includes: MAC_FILTERING, MAC_LEARNING, SPANNING_TREE, FIREWALL
 */
#define IX_NPEDL_NPEIMAGE_NPEB_ETH_LEARN_FILTER_SPAN_FIREWALL 0x01000200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEB_ETH_LEARN_FILTER_SPAN_FIREWALL_VLAN_QOS
 *
 * @brief NPE Image Id for NPE-B with Ethernet Rx/Tx which includes: MAC_FILTERING, MAC_LEARNING, SPANNING_TREE, FIREWALL, VLAN_QOS
 */
#define IX_NPEDL_NPEIMAGE_NPEB_ETH_LEARN_FILTER_SPAN_FIREWALL_VLAN_QOS 0x01010200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEB_ETH_SPAN_FIREWALL_VLAN_QOS_HDR_CONV
 *
 * @brief NPE Image Id for NPE-B with Ethernet Rx/Tx which includes: SPANNING_TREE, FIREWALL, VLAN_QOS, HEADER_CONVERSION
 */
#define IX_NPEDL_NPEIMAGE_NPEB_ETH_SPAN_FIREWALL_VLAN_QOS_HDR_CONV 0x01020200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEB_DMA
 *
 * @brief NPE Image Id for NPE-B with DMA-Only feature.
 */
#define IX_NPEDL_NPEIMAGE_NPEB_DMA 0x01020100
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEC_ETH
 *
 * @brief NPE Image Id for NPE-C with Basic Ethernet Rx/Tx which includes: MAC_FILTERING, MAC_LEARNING, SPANNING_TREE, FIREWALL
 */
#define IX_NPEDL_NPEIMAGE_NPEC_ETH 0x02000200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEC_ETH_LEARN_FILTER_SPAN_FIREWALL
 *
 * @brief NPE Image Id for NPE-C with Basic Ethernet Rx/Tx which includes: MAC_FILTERING, MAC_LEARNING, SPANNING_TREE, FIREWALL
 */
#define IX_NPEDL_NPEIMAGE_NPEC_ETH_LEARN_FILTER_SPAN_FIREWALL 0x02000200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEC_ETH_LEARN_FILTER_SPAN_FIREWALL_VLAN_QOS
 *
 * @brief NPE Image Id for NPE-C with Ethernet Rx/Tx which includes: MAC_FILTERING, MAC_LEARNING, SPANNING_TREE, FIREWALL, VLAN_QOS
 */
#define IX_NPEDL_NPEIMAGE_NPEC_ETH_LEARN_FILTER_SPAN_FIREWALL_VLAN_QOS 0x02010200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEC_ETH_SPAN_FIREWALL_VLAN_QOS_HDR_CONV
 *
 * @brief NPE Image Id for NPE-C with Ethernet Rx/Tx which includes: SPANNING_TREE, FIREWALL, VLAN_QOS, HEADER_CONVERSION
 */
#define IX_NPEDL_NPEIMAGE_NPEC_ETH_SPAN_FIREWALL_VLAN_QOS_HDR_CONV 0x02020200
#endif

#if IX_NPE_IMAGE_INCLUDE
/**
 * @def IX_NPEDL_NPEIMAGE_NPEC_DMA
 *
 * @brief NPE Image Id for NPE-C with DMA-Only feature.
 */
#define IX_NPEDL_NPEIMAGE_NPEC_DMA 0x02080100
#endif

/* Number of NPE firmware images in this library */
#define IX_NPE_MICROCODE_AVAILABLE_VERSIONS_COUNT 17

/* Location of Microcode Images */
#ifdef IX_NPE_MICROCODE_FIRMWARE_INCLUDED
#ifdef IX_NPEDL_READ_MICROCODE_FROM_FILE

extern UINT32* ixNpeMicrocode_binaryArray;

#else

extern unsigned IxNpeMicrocode_array[];

#endif
#endif

/*
 * sr: undef all but the bare minimum to reduce flash usage for U-Boot
 */
#undef IX_NPEDL_NPEIMAGE_NPEA_HSS0
#undef IX_NPEDL_NPEIMAGE_NPEA_HSS0_ATM_SPHY_1_PORT
#undef IX_NPEDL_NPEIMAGE_NPEA_HSS0_ATM_MPHY_1_PORT
#undef IX_NPEDL_NPEIMAGE_NPEA_ATM_MPHY_12_PORT
#undef IX_NPEDL_NPEIMAGE_NPEA_DMA
#undef IX_NPEDL_NPEIMAGE_NPEA_HSS_2_PORT
#undef IX_NPEDL_NPEIMAGE_NPEA_ETH
#undef IX_NPEDL_NPEIMAGE_NPEA_ETH_LEARN_FILTER_SPAN_FIREWALL
#undef IX_NPEDL_NPEIMAGE_NPEA_ETH_LEARN_FILTER_SPAN_FIREWALL_VLAN_QOS
#undef IX_NPEDL_NPEIMAGE_NPEA_ETH_SPAN_FIREWALL_VLAN_QOS_HDR_CONV
#undef IX_NPEDL_NPEIMAGE_NPEB_ETH
#undef IX_NPEDL_NPEIMAGE_NPEB_ETH_LEARN_FILTER_SPAN_FIREWALL
/* #undef IX_NPEDL_NPEIMAGE_NPEB_ETH_LEARN_FILTER_SPAN_FIREWALL_VLAN_QOS */
#undef IX_NPEDL_NPEIMAGE_NPEB_ETH_SPAN_FIREWALL_VLAN_QOS_HDR_CONV
#undef IX_NPEDL_NPEIMAGE_NPEB_DMA
#undef IX_NPEDL_NPEIMAGE_NPEC_ETH
#undef IX_NPEDL_NPEIMAGE_NPEC_ETH_LEARN_FILTER_SPAN_FIREWALL
/* #undef IX_NPEDL_NPEIMAGE_NPEC_ETH_LEARN_FILTER_SPAN_FIREWALL_VLAN_QOS */
#undef IX_NPEDL_NPEIMAGE_NPEC_ETH_SPAN_FIREWALL_VLAN_QOS_HDR_CONV
#undef IX_NPEDL_NPEIMAGE_NPEC_DMA

/**
 * @} defgroup IxNpeMicrocode
 */
