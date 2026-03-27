/*
 * ==========================================================================
 *
 *       Filename:  it8951_i80.c
 *
 *    Description:  it8951 chip protocol implement. independent with host.
 *
 *        Version:  0.01
 *        Created:  2017年07月26日 10时58分55秒
 *
 *         Author:  YHZHong 
 *        Company:  
 *
 * ==========================================================================
 */
#include <common.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/io.h>
#include "it8951_i80.h"

// Host controller function. implement by host.
extern inline void LCDWaitForReady(void);
extern void LCDWriteCmdCode(TWord cmd);
extern void LCDWriteData(TWord data);
extern TWord LCDReadData(void);
extern void LCDSendCmdArg(TWord cmdCode, TWord *pArg, TWord numArg);

//-----------------------------------------------------------
//Host Cmd 1 - SYS_RUN
//-----------------------------------------------------------
void IT8951SystemRun()
{
    LCDWriteCmdCode(IT8951_TCON_SYS_RUN);
}

//-----------------------------------------------------------
//Host Cmd 2 - STANDBY
//-----------------------------------------------------------
void IT8951StandBy()
{
    LCDWriteCmdCode(IT8951_TCON_STANDBY);
}

//-----------------------------------------------------------
//Host Cmd 3 - SLEEP
//-----------------------------------------------------------
void IT8951Sleep()
{
    LCDWriteCmdCode(IT8951_TCON_SLEEP);
}

//-----------------------------------------------------------
//Host Cmd 4 - REG_RD
//-----------------------------------------------------------
TWord IT8951ReadReg(TWord usRegAddr)
{
    TWord usData;

    //----------I80 Mode-------------
    //Send Cmd and Register Address
    LCDWriteCmdCode(IT8951_TCON_REG_RD);
    LCDWriteData(usRegAddr);
    //Read data from Host Data bus
    usData = LCDReadData();
    return usData;
}

//-----------------------------------------------------------
//Host Cmd 5 - REG_WR
//-----------------------------------------------------------
void IT8951WriteReg(TWord usRegAddr, TWord usValue)
{
    //I80 Mode
    //Send Cmd , Register Address and Write Value
    LCDWriteCmdCode(IT8951_TCON_REG_WR);
    LCDWriteData(usRegAddr);
    LCDWriteData(usValue);
}

//-----------------------------------------------------------
//Host Cmd 6 - MEM_BST_RD_T
//-----------------------------------------------------------
void IT8951MemBurstReadTrigger(TDWord ulMemAddr, TDWord ulReadSize)
{
    TWord usArg[4];

    //Setting Arguments for Memory Burst Read
    usArg[0] = (TWord)(ulMemAddr & 0x0000FFFF); //addr[15:0]
    usArg[1] = (TWord)((ulMemAddr >> 16) & 0x0000FFFF); //addr[25:16]
    usArg[2] = (TWord)(ulReadSize & 0x0000FFFF); //Cnt[15:0]
    usArg[3] = (TWord)((ulReadSize >> 16) & 0x0000FFFF); //Cnt[25:16]
                                                         //Send Cmd and Arg
    LCDSendCmdArg(IT8951_TCON_MEM_BST_RD_T, usArg, 4);
}

//-----------------------------------------------------------
//Host Cmd 7 - MEM_BST_RD_S
//-----------------------------------------------------------
void IT8951MemBurstReadStart()
{
    LCDWriteCmdCode(IT8951_TCON_MEM_BST_RD_S);
}

//-----------------------------------------------------------
//Host Cmd 8 - MEM_BST_WR
//-----------------------------------------------------------
void IT8951MemBurstWrite(TDWord ulMemAddr, TDWord ulWriteSize)
{
    TWord usArg[4];

    //Setting Arguments for Memory Burst Write
    usArg[0] = (TWord)(ulMemAddr & 0x0000FFFF); //addr[15:0]
    usArg[1] = (TWord)((ulMemAddr >> 16) & 0x0000FFFF); //addr[25:16]
    usArg[2] = (TWord)(ulWriteSize & 0x0000FFFF); //Cnt[15:0]
    usArg[3] = (TWord)((ulWriteSize >> 16) & 0x0000FFFF); //Cnt[25:16]
                                                          //Send Cmd and Arg
    LCDSendCmdArg(IT8951_TCON_MEM_BST_WR, usArg, 4);
}

//-----------------------------------------------------------
//Host Cmd 9 - MEM_BST_END
//-----------------------------------------------------------
void IT8951MemBurstEnd(void)
{
    LCDWriteCmdCode(IT8951_TCON_MEM_BST_END);
}

//-----------------------------------------------------------
//Example of Memory Burst Write
//-----------------------------------------------------------
// ****************************************************************************************
// Function name: IT8951MemBurstWriteProc( )
//
// Description:
//   IT8951 Burst Write procedure
//
// Arguments:
//      TDWord ulMemAddr: IT8951 Memory Target Address
//      TDWord ulWriteSize: Write Size (Unit: Word)
//      TByte* pDestBuf - Buffer of Sent data
// Return Values:
//   NULL.
// Note:
//
// ****************************************************************************************
void IT8951MemBurstWriteProc(TDWord ulMemAddr, TDWord ulWriteSize, TWord *pSrcBuf)
{

    TDWord i;

    //Send Burst Write Start Cmd and Args
    IT8951MemBurstWrite(ulMemAddr, ulWriteSize);

    //Burst Write Data
    for (i = 0; i < ulWriteSize; i++) {
        LCDWriteData(pSrcBuf[i]);
    }

    //Send Burst End Cmd
    IT8951MemBurstEnd();
}

// ****************************************************************************************
// Function name: IT8951MemBurstReadProc( )
//
// Description:
//   IT8951 Burst Read procedure
//
// Arguments:
//      TDWord ulMemAddr: IT8951 Read Memory Address
//      TDWord ulReadSize: Read Size (Unit: Word)
//      TByte* pDestBuf - Buffer for storing Read data
// Return Values:
//   NULL.
// Note:
//
// ****************************************************************************************
void IT8951MemBurstReadProc(TDWord ulMemAddr, TDWord ulReadSize, TWord *pDestBuf)
{
    TDWord i;

    //Send Burst Read Start Cmd and Args
    IT8951MemBurstReadTrigger(ulMemAddr, ulReadSize);

    //Burst Read Fire
    IT8951MemBurstReadStart();

    //Burst Read Data
    for (i = 0; i < ulReadSize; i++) {
        pDestBuf[i] = LCDReadData();
    }

    //Send Burst End Cmd
    IT8951MemBurstEnd(); //the same with IT8951MemBurstEnd()

}

//-----------------------------------------------------------
//Host Cmd 10 - LD_IMG
//-----------------------------------------------------------
void IT8951LoadImgStart(IT8951LdImgInfo *pstLdImgInfo)
{
    TWord usArg;

    //Setting Argument for Load image start
    usArg = (pstLdImgInfo->usEndianType << 8)
        | (pstLdImgInfo->usPixelFormat << 4)
        | (pstLdImgInfo->usRotate);
    //Send Cmd
    LCDWriteCmdCode(IT8951_TCON_LD_IMG);
    //Send Arg
    LCDWriteData(usArg);
}

//-----------------------------------------------------------
//Host Cmd 11 - LD_IMG_AREA
//-----------------------------------------------------------
void IT8951LoadImgAreaStart(IT8951LdImgInfo *pstLdImgInfo, IT8951AreaImgInfo *pstAreaImgInfo)
{
    TWord usArg[5];

    //Setting Argument for Load image start
    usArg[0] = (pstLdImgInfo->usEndianType << 8)
        | (pstLdImgInfo->usPixelFormat << 4)
        | (pstLdImgInfo->usRotate);
    usArg[1] = pstAreaImgInfo->usX;
    usArg[2] = pstAreaImgInfo->usY;
    usArg[3] = pstAreaImgInfo->usWidth;
    usArg[4] = pstAreaImgInfo->usHeight;
    //Send Cmd and Args
    LCDSendCmdArg(IT8951_TCON_LD_IMG_AREA, usArg, 5);
}

//-----------------------------------------------------------
//Host Cmd 12 - LD_IMG_END
//-----------------------------------------------------------
void IT8951LoadImgEnd(void)
{
    LCDWriteCmdCode(IT8951_TCON_LD_IMG_END);
}

//--------------------------------------------------
//3.5. Initial Functions
//--------------------------------------------------
//-----------------------------------------------------------
//Initial function - 1
//-----------------------------------------------------------
void IT8951GetSystemInfo(void *pBuf)
{
#if 1
    TWord *pusWord = (TWord *)pBuf;
    I80IT8951DevInfo *pstDevInfo;
    TWord i;

RETRY:
    //Send I80 CMD
    LCDWriteCmdCode(USDEF_I80_CMD_GET_DEV_INFO);

    //I80 interface - Single Read available
    for (i = 0; i < sizeof(I80IT8951DevInfo) / 2; i++) {
        pusWord[i] = LCDReadData();
    }

    //Show Device information of IT8951
    pstDevInfo = (I80IT8951DevInfo *)pBuf;
#else
	I80IT8951DevInfo *pstDevInfo=(I80IT8951DevInfo*)pBuf;
	pstDevInfo->usPanelW = (TWord)800;
	pstDevInfo->usPanelH = (TWord)600;
	pstDevInfo->usImgBufAddrL = (TWord)0xA838;
	pstDevInfo->usImgBufAddrH = (TWord)0x0011;
	strcpy( pstDevInfo->usFWVersion, "SG_L.v.011");
	strcpy( pstDevInfo->usLUTVersion, "M641");
#endif
    
    printf("Panel(W,H) = (%d,%d)\r\n",
           pstDevInfo->usPanelW, pstDevInfo->usPanelH);
    printf("Image Buffer Address = %X\r\n",
           pstDevInfo->usImgBufAddrL | (pstDevInfo->usImgBufAddrH << 16));
    //Show Firmware and LUT Version
    printf("FW Version = %s\r\n", (char*)pstDevInfo->usFWVersion);
    printf("LUT Version = %s\r\n", (char*)pstDevInfo->usLUTVersion);

/*
    // Check the data receive from IT8951.
    TWord testData[20] = {
        0x320, 0x258, 0xd4d0, 0xf, 0x4753, 0x4c5f, 0x762e, 0x302e, 0x3100,
        0x0, 0x0, 0x0, 0x364d, 0x3134, 0x5400, 0x0, 0x0, 0x0, 0x0, 0x0};

    for (i = 0; i < sizeof(I80IT8951DevInfo) / 2; i++) {
        if (pusWord[i] != testData[i]) {
	    printf("IT8951GetSystemInfo is wrong.\r\n");
            goto RETRY;
        }

    }
*/
    //printf("IT8951GetSystemInfo Done.\r\n");
}

//-----------------------------------------------------------
//Initial function 2 - Set Image buffer base address
//-----------------------------------------------------------
void IT8951SetImgBufBaseAddr(TDWord ulImgBufAddr)
{
    TWord usWordH = (TWord)((ulImgBufAddr >> 16) & 0x0000FFFF);
    TWord usWordL = (TWord)(ulImgBufAddr & 0x0000FFFF);

    //Write LISAR Reg
    IT8951WriteReg(LISAR + 2, usWordH);
    IT8951WriteReg(LISAR, usWordL);
}
//-----------------------------------------------------
// 3.6. Display Functions
//-----------------------------------------------------

//-----------------------------------------------------------
//Display function 1 - Wait for LUT Engine Finish
//                     Polling Display Engine Ready by LUTNo
//-----------------------------------------------------------
void IT8951WaitForDisplayReady()
{
    //Check IT8951 Register LUTAFSR => NonZero ? Busy, 0 - Free
    while (IT8951ReadReg(LUTAFSR));
}

//-----------------------------------------------------------
//Display function 2 - Load Image Area process
//-----------------------------------------------------------
void IT8951HostAreaPackedPixelWrite(IT8951LdImgInfo *pstLdImgInfo, IT8951AreaImgInfo *pstAreaImgInfo)
{
    TDWord i, j;
    //Source buffer address of Host
    TWord *pusFrameBuf = (TWord *)pstLdImgInfo->ulStartFBAddr;

    //Set Image buffer(IT8951) Base address
    IT8951SetImgBufBaseAddr(pstLdImgInfo->ulImgBufBaseAddr);
    //Send Load Image start Cmd
    IT8951LoadImgAreaStart(pstLdImgInfo, pstAreaImgInfo);
    //Host Write Data
    for (j = 0; j < pstAreaImgInfo->usHeight; j++) {
        for (i = 0; i < pstAreaImgInfo->usWidth / 2; i++) {
	    //Write a Word(2-Bytes) for each time
            LCDWriteData(*pusFrameBuf);
            pusFrameBuf++;
        }
    }
    //Send Load Img End Command
    IT8951LoadImgEnd();
}

//-----------------------------------------------------------
//Display functions 3 - Application for Display panel Area
//Mode 0 - Initial : Panel refresh to white and ignore previous image data
//Mode 1 DU : Previous 16 Gray to Current 2 Gray (Fast, Black/White only)
//Mode 2 GC : Previous 16 Gray to Current 16 Gray
//Mode 3 GL : Previous 16 Gray to Current 16 Gray, but white to white do nothing (Gray 15 to Gray 15)
//Mode 4 A2 : Previous 2 Gray to Current 2 Gray (the Fastest, Black White only)
//-----------------------------------------------------------
void IT8951DisplayArea(TWord usX, TWord usY, TWord usW, TWord usH, TWord usDpyMode)
{
    //Send I80 Display Command (User defined command of IT8951)
    LCDWriteCmdCode(USDEF_I80_CMD_DPY_AREA); //0x0034
                                             //Write arguments
    LCDWriteData(usX);
    LCDWriteData(usY);
    LCDWriteData(usW);
    LCDWriteData(usH);
    LCDWriteData(usDpyMode);
}

void IT8951DisplayAreaEx(TWord usX, TWord usY, TWord usW, TWord usH, TWord usDpyMode, TWord value)
{
    //Send I80 Display Command (User defined command of IT8951)
    LCDWriteCmdCode(0x003A); 
                                             //Write arguments
    LCDWriteData(usX);
    LCDWriteData(usY);
    LCDWriteData(usW);
    LCDWriteData(usH);
    LCDWriteData(usDpyMode);
    LCDWriteData(value);	//
}


void IT8951DisplayAreaBuf(TWord usX, TWord usY, TWord usW, TWord usH, TWord usDpyMode, TDWord ulDpyBufAddr)
{
    //Send I80 Display Command (User defined command of IT8951)
    LCDWriteCmdCode(USDEF_I80_CMD_DPY_BUF_AREA); //0x0037
   
    //Write arguments
    LCDWriteData(usX);
    LCDWriteData(usY);
    LCDWriteData(usW);
    LCDWriteData(usH);
    LCDWriteData(usDpyMode);
    LCDWriteData((TWord)(ulDpyBufAddr & 0x0000FFFF));       //Display Buffer Base address[15:0]
    LCDWriteData((TWord)((ulDpyBufAddr >> 16) & 0x0000FFFF)); //Display Buffer Base address[26:16]
 
}

void IT8951PowerOn(void)
{
   LCDWriteCmdCode(USDEF_I80_CMD_POWER);
   LCDWriteData(1);
}

void IT8951PowerOff(void)
{
   LCDWriteCmdCode(USDEF_I80_CMD_POWER);
   LCDWriteData(0);
}

TWord IT8951GetTemperature()
{
    TWord temper0, temper1;

    //Send I80 Temperature (User defined command of IT8951)
    LCDWriteCmdCode(USDEF_I80_CMD_TEMPER); //0x0040                                           

    //Write arguments
    LCDWriteData(0); 

    temper0 = LCDReadData();
    temper1 = LCDReadData();

    printf ("IT8951 Temper0: %d.\n", (int)temper0);
    printf ("IT8951 Temper1: %d.\n", (int)temper1);

    return temper0;
}

void IT8951SetTemperature(TWord temper)
{
    //Send I80 Temperature (User defined command of IT8951)
    LCDWriteCmdCode(USDEF_I80_CMD_TEMPER); //0x0040                                           

    //Write arguments
    LCDWriteData(1); 
    LCDWriteData(temper); 
}

TWord IT8951GetVCOM()
{
    TWord vcom;

    //Send I80 VCOM (User defined command of IT8951)
    LCDWriteCmdCode(USDEF_I80_CMD_VCOM); //0x0039

    //Write arguments
    LCDWriteData(0); 

    vcom = LCDReadData();
    return vcom;
}

void IT8951SetVCOM(TWord vcom)
{
    //Send I80 Temperature (User defined command of IT8951)
    LCDWriteCmdCode(USDEF_I80_CMD_VCOM); //0x0039

    //Write arguments
    LCDWriteData(1); 
    LCDWriteData(vcom); 
}


#if 0
//----------------------------------------------------------------
//3.7. Test Functions
//----------------------------------------------------------------
void HostInit();

//Global structures and variables
I80IT8951DevInfo gstI80DevInfo;
TByte *gpFrameBuf; //Host Source Frame buffer
TDWord gulImgBufAddr; //IT8951 Image buffer address

//-----------------------------------------------------------
//Test function 1 - Software Initial flow for testing
//-----------------------------------------------------------
void HostInit()
{
    //Get Device Info
    IT8951GetSystemInfo(&gstI80DevInfo);

    //Host Frame Buffer allocation
    gpFrameBuf = malloc(gstI80DevInfo.usPanelW * gstI80DevInfo.usPanelH);
    //Get Image Buffer Address of IT8951
    gulImgBufAddr = gstI80DevInfo.usImgBufAddrL | (gstI80DevInfo.usImgBufAddrH << 16);

    //Set to Enable I80 Packed mode
    IT8951WriteReg(I80CPCR, 0x0001);
}


void IT8951DumpRegisters()
{
	TWord data = 0;

	printf("\n>>>>IT8951 REG DUMP<<<<\n");

	data = IT8951ReadReg(I80CPCR);
	printf("    I80CPCR=0X%X\n", data);

	data = IT8951ReadReg(LUT0EWHR);
	printf("    LUT0EWHR=0X%X\n", data);

	data = IT8951ReadReg(LUT0XYR);
	printf("    LUT0XYR=0X%X\n", data);

	data = IT8951ReadReg(LUT0BADDR);
	printf("    LUT0BADDR=0X%X\n", data);

	data = IT8951ReadReg(LUT0MFN);
	printf("    LUT0MFN=0X%X\n", data);

	data = IT8951ReadReg(LUT01AF);
	printf("    LUT01AF=0X%X\n", data);

	data = IT8951ReadReg(UP0SR);
	printf("    UP0SR=0X%X\n", data);

	data = IT8951ReadReg(UP1SR);
	printf("    UP1SR=0X%X\n", data);

	data = IT8951ReadReg(LUT0ABFRV);
	printf("    LUT0ABFRV=0X%X\n", data);

	data = IT8951ReadReg(UPBBADDR);
	printf("    UPBBADDR=0X%X\n", data);

	data = IT8951ReadReg(LUT0IMXY);
	printf("    LUT0IMXY=0X%X\n", data);

	data = IT8951ReadReg(LUTAFSR);
	printf("    LUTAFSR=0X%X\n", data);

	data = IT8951ReadReg(MCSR);
	printf("    MCSR=0X%X\n", data);

	data = IT8951ReadReg(LISAR);
	printf("    LISAR=0X%X\n", data);

	data = IT8951ReadReg(LISAR + 2);
	printf("    LISAR + 2=0X%X\n", data);

	printf(">>>>END<<<<\n\n");

}


//-----------------------------------------------------------
//Test function 2 - Example of Display Flow
//-----------------------------------------------------------
void IT8951DisplayExample()
{
    IT8951LdImgInfo stLdImgInfo;
    IT8951AreaImgInfo stAreaImgInfo;

    //Host Initial
    HostInit();

    //--------------------------------------------------------------------------------------------
    //      Regular display - Display Any Gray colors with Mode 2 or others
    //--------------------------------------------------------------------------------------------
    //Preparing buffer to All black (8 bpp image)
    //or you can create your image pattern here..
    //memset(gpFrameBuf, 0x00, gstI80DevInfo.usPanelW * gstI80DevInfo.usPanelH);
    int i;
    for (i = 0; i < 8; i++) {
        memset(gpFrameBuf + i * 800 * 75, (i*2) << 4, 800*75);
    }

    IT8951WaitForDisplayReady();

    //Setting Load image information
    stLdImgInfo.ulStartFBAddr    = (TDWord)gpFrameBuf;
    stLdImgInfo.usEndianType     = IT8951_LDIMG_L_ENDIAN;
    stLdImgInfo.usPixelFormat    = IT8951_8BPP;
    stLdImgInfo.usRotate         = IT8951_ROTATE_0;
    stLdImgInfo.ulImgBufBaseAddr = gulImgBufAddr;
    //Set Load Area
    stAreaImgInfo.usX      = 0;
    stAreaImgInfo.usY      = 0;
    stAreaImgInfo.usWidth  = gstI80DevInfo.usPanelW;
    stAreaImgInfo.usHeight = gstI80DevInfo.usPanelH;
    //Load Image from Host to IT8951 Image Buffer
    IT8951HostAreaPackedPixelWrite(&stLdImgInfo, &stAreaImgInfo); //Display function 2

    //Display Area ? (x,y,w,h) with mode 2 for fast gray clear mode - depends on current waveform
    IT8951DisplayArea(0, 0, gstI80DevInfo.usPanelW, gstI80DevInfo.usPanelH, 2);
}
#endif

