//******************************************************************************
//	Copyright(c) Eyenix Co.,Ltd. 2003-
//
//	File Name:		spi_flash.c
//	Description:	
//	Author:			YG Kim, YK Jang, Eyenix Co.,Ltd.
//
//	Note:			FTDX library using MPSSE for SPI and flash
//
//	Ver Date		Author		Changes
//	--- ----		------		-------
//	0.1	16xxxx		YK Jang		first designed
//	0.2	160730		ygkim		Redesigned and optimized performance
// -----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
//#include <unistd.h>
#include <string.h>
#include <time.h>
#include <Winsock2.h>			// ntohl
#include "ftd2xx.h"
#include "spi_base.h"

int debug_spi = 0;

#ifdef _SPI_DEBUG
#define DEBUG(...) 		if(debug_spi) { fprintf(stdout, "$SPI>    "); 	fprintf(stdout, __VA_ARGS__); }
#define	TRACE(...) 	  	if(debug_spi) { fprintf(stdout, "         "); 	fprintf(stdout, __VA_ARGS__); }
#else
#define DEBUG(...)
#define TRACE(...)
#endif


// the linefeed character speeds up. But, dont know why??						// $CMT-ygkim-160720
//#define FASTMODE

#ifdef FASTMODE
#define DBG_CHAR	0x0A	// linefeed												
#else
#define DBG_CHAR 	0x0D	// carriage return
#endif


//------------------------------------------------------------------------------
// flash information: 
// Let's add more commands and more flash devices
//
typedef struct {
    char* 	name;
    uint 	dev_id;

    uint	blocksize;
    uint	sectorsize;
    uint	pagesize;

    uchar	chip_erase_cmd;
    uchar	block_erase_cmd;
    uchar	sector_erase_cmd;
    /* Reserved to add more commands */
} tFlashInfo;

const tFlashInfo flash_info[] = {
    //name							dev_id		bsize		ssize	psize	cerase	berase	serase
    { "WINBOND W25Q128 Quad 128Mb", 0x00EF4018, 64 * 1024, 4 * 1024, 256, 0xC7, 0xD8, 0x20 },
    { "EON W25Q128 Quad 128Mb", 0x001C7018, 64 * 1024, 4 * 1024, 256, 0xC7, 0xD8, 0x20 },
    { "EON W25Q128 Quad 128Mb", 0x1c701800, 64 * 1024, 4 * 1024, 256, 0xC7, 0xD8, 0x20 },
    { "EON W25Q128 Quad 128Mb", 0x00ffffff, 64 * 1024, 4 * 1024, 256, 0xC7, 0xD8, 0x20 },
    { "ISSI W25Q128 Quad 64Mb", 0x9d6017, 64 * 1024, 4 * 1024, 256, 0xC7, 0xD8, 0x20 },
    { "ISSI W25Q128 Quad 128Mb", 0x9d6018, 64 * 1024, 4 * 1024, 256, 0xC7, 0xD8, 0x20 },
    { "ISSI W25Q128 Quad 128Mb", 0x1c3013, 64 * 1024, 4 * 1024, 256, 0xC7, 0xD8, 0x20 },
    { NULL, 0, 0, 0, 0, 0, 0, 0 }	// the last
};

#define	FLASH_MANUF(dev_id)		(uchar)((dev_id)>>16)
#define	FLASH_TYPE(dev_id)		(uchar)((dev_id)>>8)
#define	FLASH_SIZE(dev_id)		(uchar)((dev_id))

const tFlashInfo* pflashInfo = NULL;
uint blocksize = 0;				// for fast access
uint sectorsize = 0;
uint pagesize = 0;


//------------------------------------------------------------------------------
// ftdx
#define CMD_SCS			0x40  	// Write TMS/CS
#define CMD_SDI			0x20  	// Read TDI/DI
#define CMD_SDO			0x10  	// Write TDO/DO
#define CMD_LSB			0x08  	// LSB first
#define CMD_SDI_NEG		0x04  	// Sample TDI/DI on negative TCK/SK edge
#define CMD_BITMODE		0x02  	// Write bits, not bytes
#define CMD_SDO_NEG		0x01  	// Write TDO/DO on negative TCK/SK edge

//									CMD_SCS|CMD_SDI|CMD_SDO|CMD_LSB|CMD_SDI_NEG|CMD_BITMODE|CMD_SDO_NEG		//		TMS TDO TDI	BIT				
//									40h 	20h		10h		08h		04h			02h			01h				//									
#define	CMD_SCS_SDO_BITS	 	(	CMD_SCS		   		   				   	   |CMD_BITMODE|CMD_SDO_NEG	)   //		o	o		 o	0x43:0100 0011	
#define	CMD_SDO_BITS			(		   		   	CMD_SDO				   	   |CMD_BITMODE|CMD_SDO_NEG	)   //			o		 o	0x13:0001 0011	

#define	CMD_SCS_SDO_SDI_BITS 	(	CMD_SCS|CMD_SDI	   	   				   	   |CMD_BITMODE				)   //		o	o	o	 o	0x66:0110 0110	
#define	CMD_SDO_SDI_BITS		(		   	CMD_SDI|CMD_SDO			 	   	   |CMD_BITMODE|CMD_SDO_NEG	)   //			o	o	 o	0x33:0011 0011	

#define	CMD_SDO_BYTES			(					CMD_SDO								   |CMD_SDO_NEG	)   //			o		 	0x11:0001 0001	
#define	CMD_SDO_SDI_BYTES		(			CMD_SDI|CMD_SDO								   |CMD_SDO_NEG	)   //			o		 	0x31:0011 0001
//#define	CMD_SDI_BYTES		(			CMD_SDI			       |CMD_SDI_NEG							)   //			o		 	0x24:0010 0100
#define	CMD_SDI_BYTES			(			CMD_SDI			      										)   //			o		 	0x20:0010 0000

#define	CMD_SEND_IMMEDIATE	0x87

#define	FTSPI_BUF_LEN 	(64*1024)
uchar fts_wbuf[FTSPI_BUF_LEN];
uchar fts_rbuf[FTSPI_BUF_LEN];
FT_HANDLE fts_handle;
uint fts_wcnt = 0;

int time_to_erase = 0;
int time_to_write = 0;
int time_to_verify = 0;


//******************************************************************************
// FTDX functions
//------------------------------------------------------------------------------
int spi_ftdx_init(uint clkdiv, uint rd_timeout, uint wr_timeout, uint latency)
{
    FT_STATUS err = 0;
    DWORD rbuf_len;				// # of read bytes in the FT2232H receive buffer
    DWORD rcnt, wcnt;		// actual bytes read count
    int i;

    // Get the number of FTDI devices											// $CMT-ygkim-160803: can skip
    printf("Checking for FTDI devices: ");
    DWORD ndev;					// # of devices
    err = FT_CreateDeviceInfoList(&ndev);
    if (err != FT_OK) {
        printf("Getting the number of devices\n");
        return -1;
    }
    if (ndev<1) {
        printf("No FTDI devices\n");
        return -1;
    }
    printf("%d devices found\n", ndev);

    // Open "EN673 channel B for SPI"
    // For this application note, we'll assume the first device is a FT2232H or FT4232H
    // Further checks can be made against the device descriptions, locations, serial numbers, etc. before opening the port.
    printf("Open ENX channel B for MPSSE-SPI\n");
    //err = FT_Open(1, &fts_handle);
    err = FT_OpenEx("EN673B", FT_OPEN_BY_SERIAL_NUMBER, &fts_handle);
    if (err != FT_OK) err = FT_OpenEx("ENXB", FT_OPEN_BY_SERIAL_NUMBER, &fts_handle);
    if (err != FT_OK) {
        printf("Failed to open, error %d\n", err);
        return -1;
    }

    // Get driver version
    DWORD drv_ver;				// driver version
    err = FT_GetDriverVersion(fts_handle, &drv_ver);
    if (err != FT_OK) {
        printf("Reading driver version\n");
        return -1;
    }
    printf("Driver version: 0x%x, %x.%x.%x\n", drv_ver, (drv_ver >> 16) & 0xff, (drv_ver >> 8) & 0xff, drv_ver & 0xff);

    // Configure port parameters
    printf("Configuring port for MPSSE use...\n");
    err |= FT_ResetDevice(fts_handle);

    // Purge USB receive buffer first by reading out all old data from FT2232H receive buffer
    err |= FT_GetQueueStatus(fts_handle, &rbuf_len);				// Get the number of bytes

    if ((err == FT_OK) && (rbuf_len > 0))
        err |= FT_Read(fts_handle, fts_rbuf, rbuf_len, &rcnt);	// read out all old data

    err |= FT_SetUSBParameters(fts_handle, 65536, 65535);		// Set USB request transfer sizes : for write 64KB, for read 64KB-1	
    err |= FT_SetChars(fts_handle, 0, 0, 0, 0);					// Disable event and error characters
    err |= FT_SetTimeouts(fts_handle, rd_timeout, wr_timeout);	// Set the r/w timeouts in ms
    err |= FT_SetLatencyTimer(fts_handle, latency);				// Set the latency timer: at least 2ms (default is 16ms)
    err |= FT_SetBitMode(fts_handle, 0x0, 0x00);					// Reset controller
    err |= FT_SetBitMode(fts_handle, 0x0, 0x02);					// Enable MPSSE mode

    if (err != FT_OK) {
		printf("Initializing the MPSSE: %d\n", err);
        FT_Close(fts_handle);
        return FALSE;
    }

    // Wait for all the USB stuff to complete and work
    Sleep(50);

    //--------------------------------------------------------------------------
    // At this point, the MPSSE is ready for commands
    printf("MPSSE is ready for commands\n");

    //--------------------------------------------------------------------------
    // Synchronize the MPSSE by sending a bogus opcode (0xAA)
    // : MPSSE will respond with "Bad Command" (0xFA) followed by the bogus opcode itself.
    // Send off the BAD commands
    fts_wbuf[0] = 0xAA;
    err = FT_Write(fts_handle, fts_wbuf, 1, &wcnt);

    do {
        err = FT_GetQueueStatus(fts_handle, &rbuf_len);
    } while ((rbuf_len == 0) && (err == FT_OK));
    //} while ((err == FT_OK) && rbuf_len!=2);

    uint bCmdEcho = 0;
    err = FT_Read(fts_handle, fts_rbuf, rbuf_len, &rcnt);

    // Check if Bad command and echo command received
    for (i = 0; i<rcnt - 1; i++) {
        if ((fts_rbuf[i] == 0xFA) && (fts_rbuf[i + 1] == 0xAA)) {
            bCmdEcho = 1;
            break;
        }
    }
    if (bCmdEcho == 0) {
        printf("Synchronizing the MPSSE: ");
        for (i = 0; i<rcnt; i++)
            printf("%x ", fts_rbuf[i]);
        printf("\n");
        FT_Close(fts_handle);
        return -1;
    }


    //--------------------------------------------------------------------------
    // Configure the MPSSE settings for SPI
    //	: Multple commands can be sent to the MPSSE with one FT_Write
    printf("Configuring the MPSSE for SPI\n");

    // Send HS-specific commands
    fts_wbuf[0] = 0x8A;										// Use 60MHz master clock (disable divide by 5)
    fts_wbuf[1] = 0x97;										// Turn off adaptive clocking (may be needed for ARM)
    fts_wbuf[2] = 0x8D;										// Disable three-phase clocking
    err = FT_Write(fts_handle, fts_wbuf, 3, &wcnt);
    if (err != FT_OK) {
        printf("Set up the Hi-Speed specific commands\n");
        return -1;
    }

    // Low GPIO config commands: Set all pins as inputs
    // : Set initial states of the MPSSE interface - low byte, both pin directions and output values
    //	Pin 	Signal	Dir		Init. value @ output
    //	ADBUS0	TCK/SK	in		0							
    //	ADBUS1	TDO/DO	in		0							
    //	ADBUS2	TDI/DI	in		0							
    //	ADBUS3	TMS/CS	in		1	default 1				
    //	ADBUS4	GPIOL0	in		0							
    //	ADBUS5	GPIOL1	in		0							
    //	ADBUS6	GPIOL2	in		0							
    //	ADBUS7	GPIOL3	in		0	used for Board reset	
    fts_wbuf[0] = 0x80;										// Set data bits low-byte of MPSSE port
    fts_wbuf[1] = 0x08;										// Initial value @ outputs
    fts_wbuf[2] = 0x00;										// Direction
    err = FT_Write(fts_handle, fts_wbuf, 3, &wcnt);
    if (err != FT_OK) {
        printf("Set initial states of the MPSSE interface fail\n");
        return -1;
    }

    // Set SCK frequency: SCK = 60MHz /((1 + [(1 +0xValueH*256) OR 0xValueL])*2)
    printf("Clock divisor: %d\n", clkdiv);
    fts_wbuf[0] = 0x86;										// Command to set clock divisor
    fts_wbuf[1] = clkdiv & 0xFF;							// Set 0xValueL of clock divisor
    fts_wbuf[2] = (clkdiv >> 8) & 0xFF;						// Set 0xValueH of clock divisor
    err = FT_Write(fts_handle, fts_wbuf, 3, &wcnt);
    if (err != FT_OK) {
        printf("Set SCK frequency\n");
        return -1;
    }

    Sleep(20);

    // Disable internal loop-back
    fts_wbuf[0] = 0x85;										// Disable loopback
    err = FT_Write(fts_handle, fts_wbuf, 1, &wcnt);
    if (err != FT_OK) {
        printf("Disable internal loop-back\n");
        return -1;
    }

    Sleep(30);
    return 0;
}

//------------------------------------------------------------------------------
//
int spi_ftdx_reset_target(void)
{
    FT_STATUS err = 0;
    DWORD wcnt;

    //test reset
    /*
    int i;
    for(;;) {
    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0xC0;	//	{RSTN,...,CS,DI,DO,CK} = {11000000}
    fts_wbuf[2] = 0xC0;	//	{RSTN,...,CS,DI,DO,CK} = {ooiiiiii}		RSTN=1 as output
    err = FT_Write(fts_handle, fts_wbuf, 3, &wcnt);
    usleep(100000);

    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0x00; //	{RSTN,...,CS,DI,DO,CK} = {00000000}
    fts_wbuf[2] = 0xC0;	//	{RSTN,...,CS,DI,DO,CK} = {ooiiiiii}		RSTN=0 as output
    err|= FT_Write(fts_handle, fts_wbuf, 3, &wcnt);
    usleep(100000);
    }
    */

    // RESET Signal
    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0x00; //	{RSTN,...,CS,DI,DO,CK} = {00000000}
    fts_wbuf[2] = 0x8F; //	{RSTN,...,CS,DI,DO,CK} = {oiiioooo}:	RSTN=0 for global reset
    err |= FT_Write(fts_handle, fts_wbuf, 3, &wcnt);

    Sleep(50);																// $CMT-ygkim-160804: wait for reset

    //	Exit QuadMode
    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0x06;	//	{RSTN,...,CS,DI,DO,CK} = {00000110}		CS=0, DI=DO=1, DQ2&3 already pull-up
    fts_wbuf[2] = 0x8F;
    err |= FT_Write(fts_handle, fts_wbuf, 3, &wcnt);

    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0x07;	//	{RSTN,...,CS,DI,DO,CK} = {00000111}		CK=1
    fts_wbuf[2] = 0x8F;
    err |= FT_Write(fts_handle, fts_wbuf, 3, &wcnt);

    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0x06;	//	{RSTN,...,CS,DI,DO,CK} = {00000110}		CK=0
    fts_wbuf[2] = 0x8F;
    err |= FT_Write(fts_handle, fts_wbuf, 3, &wcnt);

    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0x07;	//	{RSTN,...,CS,DI,DO,CK} = {00000111}		CK=1
    fts_wbuf[2] = 0x8F;
    err |= FT_Write(fts_handle, fts_wbuf, 3, &wcnt);

    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0x06;	//	{RSTN,...,CS,DI,DO,CK} = {00000110}		CK=0
    fts_wbuf[2] = 0x8F;
    err |= FT_Write(fts_handle, fts_wbuf, 3, &wcnt);

    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0x08;	//	{RSTN,...,CS,DI,DO,CK} = {00001000}		CS=1, DI=DO=0
    fts_wbuf[2] = 0x8F;
    err |= FT_Write(fts_handle, fts_wbuf, 3, &wcnt);

    // SPI default configuration
    fts_wbuf[0] = 0x80;
    fts_wbuf[1] = 0x08;	//	{RSTN,...,CS,DI,DO,CK} = {00001000}		
    fts_wbuf[2] = 0x8B;	//	{RSTN,...,CS,DI,DO,CK} = {oiiioioo}		DI as input
    err |= FT_Write(fts_handle, fts_wbuf, 3, &wcnt);

    return err;
}

inline int spi_ftdx_flush()
{
    ASSERT(fts_wcnt<FTSPI_BUF_LEN - 1);

    FT_STATUS err = 0;
    DWORD wcnt;

    if (fts_wcnt)
        err = FT_Write(fts_handle, fts_wbuf, fts_wcnt, &wcnt);
    fts_wcnt = 0;
    return err;
}

int spi_ftdx_toggle_cs(int cs)														// $TODO-ygkim-160729: jtag_ftdx_write_bit/jtag_ftdx_tms_write_bits¾²ÀÚ!
{
    FT_STATUS err = 0;
    //uint wcnt;

    fts_wbuf[fts_wcnt++] = 0x80;
    fts_wbuf[fts_wcnt++] = cs ? 0x08 :	//	{RSTN,...,CS,DI,DO,CK} = {00001000}
        0x00;	// 	{RSTN,...,CS,DI,DO,CK} = {00000000}
    fts_wbuf[fts_wcnt++] = 0x8B;			//	{RSTN,...,CS,DI,DO,CK} = {oiiioioo}

    //err = FT_Write(fts_handle, fts_wbuf, 3, &wcnt);
    return err;
}

// bytes
int spi_ftdx_write_bytes(uchar* wbuf, ushort len)
{
    FT_STATUS err = 0;
    //uint wcnt;

    fts_wbuf[fts_wcnt++] = CMD_SDO_BYTES;
    fts_wbuf[fts_wcnt++] = (uchar)(len - 1);
    fts_wbuf[fts_wcnt++] = (uchar)((len - 1) >> 8);
    memcpy(&fts_wbuf[fts_wcnt], wbuf, len);
    fts_wcnt += len;

    //err = FT_Write(fts_handle, fts_wbuf, len+3, &wcnt);
    return err;
}

/*
int spi_ftdx_read_write_bytes(uchar* rbuf, ushort len)
{
FT_STATUS err;
uint wcnt;

fts_wbuf[fts_wcnt++] = CMD_SDI_BYTES;
fts_wbuf[fts_wcnt++] = (uchar) (len-1);
fts_wbuf[fts_wcnt++] = (uchar)((len-1) >> 8);

err = FT_Write(fts_handle, fts_wbuf, 3, &wcnt);
return err;
}
*/

int spi_ftdx_read_bytes_cmd(ushort len)
{
    FT_STATUS err = 0;
    //uint wcnt;

    fts_wbuf[fts_wcnt++] = CMD_SDI_BYTES;
    fts_wbuf[fts_wcnt++] = (uchar)(len - 1);
    fts_wbuf[fts_wcnt++] = (uchar)((len - 1) >> 8);

    //err = FT_Write(fts_handle, fts_wbuf, 3, &wcnt); 
    return err;
}

int spi_ftdx_read_bytes(uchar* rbuf, ushort len)
{
    FT_STATUS err = 0;
    DWORD rcnt;
    uint wcnt=0;				

    fts_wbuf[fts_wcnt++] = CMD_SDI_BYTES;
    fts_wbuf[fts_wcnt++] = (uchar)(len - 1);
    fts_wbuf[fts_wcnt++] = (uchar)((len - 1) >> 8);
    fts_wbuf[fts_wcnt++] = CMD_SEND_IMMEDIATE;
    //err = FT_Write(fts_handle, fts_wbuf, 4, &wcnt); 
    err |= spi_ftdx_flush();														// $CMT-ygkim-160802: write before reading

    //usleep(2000);

    do {
        err |= FT_GetQueueStatus(fts_handle, &rcnt);
        if (wcnt++ > MAX_RETRY_CNT) break;
    } while ((err == FT_OK) && (rcnt != len));

    err |= FT_Read(fts_handle, rbuf, len, &rcnt);
    return err;
}

int spi_ftdx_release_target(void)
{
    FT_STATUS err = 0;
    //uint wcnt;

    fts_wbuf[fts_wcnt++] = 0x80;
    fts_wbuf[fts_wcnt++] = 0x80; 	//	{RSTN,...,CS,DI,DO,CK} = {10000000}		
    fts_wbuf[fts_wcnt++] = 0x80; 	//	{RSTN,...,CS,DI,DO,CK} = {oiiiiiii}		RSTN=1 as output
    err |= spi_ftdx_flush();														// $CMT-ygkim-160802: write before closing
    //err = FT_Write(fts_handle, fts_wbuf, 3, &wcnt);
    return err;
}

int spi_ftdx_release_port(void)													// $CMT-ygkim-160909: all ports as inputs
{
    FT_STATUS err = 0;
    // Low GPIO config commands: Set all pins as inputs
    // : Set initial states of the MPSSE interface - low byte, both pin directions and output values
    //	Pin 	Signal	Dir		Init. value @ output
    //	ADBUS0	TCK/SK	in		0							
    //	ADBUS1	TDO/DO	in		0							
    //	ADBUS2	TDI/DI	in		0							
    //	ADBUS3	TMS/CS	in		1	default 1				
    //	ADBUS4	GPIOL0	in		0							
    //	ADBUS5	GPIOL1	in		0							
    //	ADBUS6	GPIOL2	in		0							
    //	ADBUS7	GPIOL3	in		0	used for Board reset	
    fts_wbuf[fts_wcnt++] = 0x80;										// Set data bits low-byte of MPSSE port
    fts_wbuf[fts_wcnt++] = 0x08;										// Initial value @ outputs
    fts_wbuf[fts_wcnt++] = 0x00;										// Direction
    err |= spi_ftdx_flush();														// $CMT-ygkim-160802: write before closing
    //err = FT_Write(fts_handle, fts_wbuf, 3, &wcnt);	
    //if(err != FT_OK) {
    //	ERROR("Set initial states of the MPSSE interface fail\n");
    //	return -1;					
    //}
    return err;
}

int spi_ftdx_close(void)
{
    FT_STATUS err = 0;
    err |= spi_ftdx_flush();														// $CMT-ygkim-160802: write before closing
    //    err = FT_SetBitMode(fts_handle, 0x0, 0x00); 			// reset controller
    err |= FT_Close(fts_handle);
    return err;
}


//******************************************************************************
// flash low-level functions using ftdx functions
//------------------------------------------------------------------------------
//		
int spi_flash_reset(void)			// software reset command seq.				// $CMT-ygkim-160801: @p.80
{
    FT_STATUS err = 0;
    uchar wdat;

    wdat = 0x66;					// "Enable reset" command
    err |= spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(&wdat, 1);
    err |= spi_ftdx_toggle_cs(1);

    wdat = 0x99;					// "reset" command
    err |= spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(&wdat, 1);
    err |= spi_ftdx_toggle_cs(1);
    err |= spi_ftdx_flush();														// $CMT-ygkim-160802: write before start
    Sleep(50);																// $CMT-ygkim-160801: >= 30us @p.80

    return err;
}

int spi_flash_write_en(void)
{
    FT_STATUS err = 0;
    uchar wdat = 0x06;				// "write enable" command

    err = spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(&wdat, 1);
    err |= spi_ftdx_toggle_cs(1);
    return err;
}

int spi_flash_write_wrsr(uchar wdat)
{
    FT_STATUS err = 0;
    uchar wbuf[2];
    wbuf[0] = 0x01;					// "write status register" command
    wbuf[1] = wdat;					// data

    err = spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(wbuf, 2);
    err |= spi_ftdx_toggle_cs(1);
    return err;
}


int spi_flash_read_id(uint* id)
{
    FT_STATUS err = 0;
    uchar wdat = 0x9f;				// "read identification" command			// $CMT-ygkim-160729: @p.67
    uint rdat;

    err = spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(&wdat, 1);

    err |= spi_ftdx_read_bytes((uchar*)&rdat, 3);

    err |= spi_ftdx_toggle_cs(1);
    //    printf("ID %08x\n", rdat);
    *id = ntohl(rdat << 8);
    return err;
}

int spi_flash_wait_for_busy_slow(void)
{
    FT_STATUS err = 0;
    uchar wdat = 0x05;			// "read status register" command 				// $CMT-ygkim-160729: @ p.26
    uint wcnt = 0;

    err = spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(&wdat, 1);

    do {
        err |= spi_ftdx_read_bytes(fts_rbuf, 1);	// can read continuously		// $CMT-ygkim-160729: Figure 8a @ p.31
        if (wcnt++ > MAX_RETRY_CNT) break;
    } while (fts_rbuf[0] & 0x1);

    err |= spi_ftdx_toggle_cs(1);
    return err;
}

int spi_flash_wait_for_busy(void)
{
    FT_STATUS err = 0;
    uchar wdat = 0x05;				// "read status register1" command 			// $CMT-ygkim-160729: @p.26

    BENCHMARK("spi_flash_wait_for_busy-read SR1 command",
        err = spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(&wdat, 1);
    err |= spi_ftdx_flush();														// $CMT-ygkim-160802: write before reading
    );

    uchar wbuf[4] = { CMD_SDI_BYTES, 0, 0, CMD_SEND_IMMEDIATE };
    uchar rdat = 0;
    DWORD cnt;
    uint wcnt = 0;

    BENCHMARK("spi_flash_wait_for_busy-read SR1",
    do {
        err |= FT_Write(fts_handle, wbuf, 4, &cnt);

    //    do {																	// $CMT-ygkim-160802: skip to make faster
    //    	err|= FT_GetQueueStatus(fts_handle, &rcnt);
    //    } while ((err == FT_OK) && (rcnt != 1));

        err |= FT_Read(fts_handle, &rdat, 1, &cnt);

        if (wcnt++ > MAX_RETRY_CNT) break;
    } while (rdat & 0x1);
    );

    err |= spi_ftdx_toggle_cs(1);
    return err;
}




//******************************************************************************
// flash mid-level functions
//------------------------------------------------------------------------------
//
int spi_flash_remove_protect(void)
{
    DEBUG("spi_flash_remove_protect\n");
    FT_STATUS err = 0;
    err = spi_flash_write_en();
    err |= spi_flash_write_wrsr(0x00);
    err |= spi_flash_wait_for_busy();
    return err;
}

int spi_flash_erase_block(uint adr)
{
    DEBUG("spi_flash_erase_block: adr=0x%x\n", adr);
    ASSERT((adr&(blocksize - 1)) == 0);
    FT_STATUS err = 0;
    err |= spi_flash_write_en();													// $CHK-ygkim-160729: can skip?

    uchar wbuf[4];
    wbuf[0] = pflashInfo->block_erase_cmd;	// "block erase" command			// $CMT-ygkim-160729: @p.55							
    wbuf[1] = (uchar)(adr >> 16);
    wbuf[2] = (uchar)(adr >> 8);
    wbuf[3] = (uchar)(adr);
    err |= spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(wbuf, 4);
    err |= spi_ftdx_toggle_cs(1);

    err |= spi_flash_wait_for_busy_slow();
    return err;
}

int spi_flash_erase_sector(uint adr)
{
    DEBUG("spi_flash_erase_sector: adr=0x%x\n", adr);
    ASSERT((adr&(sectorsize - 1)) == 0);
    FT_STATUS err = 0;
    err |= spi_flash_write_en();

    uchar wbuf[4];
    wbuf[0] = pflashInfo->sector_erase_cmd;	// "sector erase" command			// $CMT-ygkim-160729: @p.53							
    wbuf[1] = (uchar)(adr >> 16);
    wbuf[2] = (uchar)(adr >> 8);
    wbuf[3] = (uchar)(adr);
    err |= spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(wbuf, 4);
    err |= spi_ftdx_toggle_cs(1);

    err |= spi_flash_wait_for_busy_slow();
    return err;
}

int spi_flash_write_page(uchar* wbuf, uint adr, uint len)
{
    DEBUG("spi_flash_write_page: adr=0x%x, len=0x%x\n", adr, len);
    ASSERT((adr&(pagesize - 1)) == 0);
    FT_STATUS err = 0;

    BENCHMARK("spi_flash_write_en",
        err |= spi_flash_write_en();
    );

    uchar pbuf[4 + 256];
    pbuf[0] = 0x02;					// "page program" command					// $CMT-ygkim-160729: p.50
    pbuf[1] = (uchar)(adr >> 16);
    pbuf[2] = (uchar)(adr >> 8);
    pbuf[3] = (uchar)(adr);
    memcpy(&pbuf[4], wbuf, len);

    BENCHMARK("spi_flash_write_page",
        err |= spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(pbuf, len + 4);
    err |= spi_ftdx_toggle_cs(1);
    );

    BENCHMARK("spi_flash_wait_for_busy",
        err |= spi_flash_wait_for_busy();
    );

    return err;
}

int spi_flash_read(uchar* rbuf, uint adr, uint len)
{
    DEBUG("spi_flash_read: adr=0x%x, len=0x%x\n", adr, len);
    FT_STATUS err = 0;

    uchar wbuf[4];
    wbuf[0] = 0x03;					// read command
    wbuf[1] = (uchar)(adr >> 16);
    wbuf[2] = (uchar)(adr >> 8);
    wbuf[3] = (uchar)(adr);

    err |= spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(wbuf, 4);

    err |= spi_ftdx_read_bytes(rbuf, len);

    err |= spi_ftdx_toggle_cs(1);
    return err;
}

//******************************************************************************
// flash top-level functions
//------------------------------------------------------------------------------
int spi_flash_read_bin(char* fname, uchar** buf, int* filesize)
{
    FILE* fd = fopen(fname, "rb");
    if (fd == NULL)	return -1;

    fseek(fd, 0, SEEK_END);
    *filesize = ftell(fd);
    fseek(fd, 0, SEEK_SET);

    *buf = (uchar*)malloc(*filesize);
    if (*buf == NULL) {
        ERROR_("Allocating memory for the file %s\n", fname);
        return -1;
    }

    if (fread(*buf, 1, *filesize, fd) < *filesize) {  // 1 long read will be faster than many short ones
        ERROR_("Reading entire the file %s\n", fname);
        return -1;
    }
    fclose(fd);
    return 0;
}

int spi_flash_erase(uint adr, uint len)
{
    DEBUG("spi_flash_erase: adr=0x%x, len=0x%x\n", adr, len);

    FT_STATUS err = 0;
    uint total_len = len;

    uint bench_tot_start_tick = bench_tic();

    for (; len >= blocksize; len -= blocksize, adr += blocksize) {
        err |= spi_flash_erase_block(adr);
//        printf("block 0x%x \n", adr);
        printf("Erasing... %.2f%%%c", ((double)(total_len - len) / total_len)*100.0, DBG_CHAR); fflush(stdout);
    }

    for (; len >= sectorsize; len -= sectorsize, adr += sectorsize) {
        err |= spi_flash_erase_sector(adr);
//        printf("block 0x%x \n", adr);
        printf("Erasing... %.2f%%%c", ((double)(total_len - len) / total_len)*100.0, DBG_CHAR); fflush(stdout);
    }

    if (len>0) err |= spi_flash_erase_sector(adr);

    uint bench_tot_end_tick = bench_tic();
    time_to_erase = bench_tot_end_tick - bench_tot_start_tick;
    return err;
}

int spi_flash_write(uchar* wbuf, uint adr, uint len)
{
    DEBUG("spi_flash_write: adr=0x%x, len=0x%x\n", adr, len);

    FT_STATUS err = 0;
    uint total_len = len;
    uint bench_tot_start_tick = bench_tic();

    for (; len >= pagesize; len -= pagesize, adr += pagesize, wbuf += pagesize) {
        BENCHMARK("spi_flash_write_page",
            err |= spi_flash_write_page(wbuf, adr, pagesize);
        );
        double p = ((double)(total_len - len) / (double)total_len)*100.0;
        if ((int)(p / 10))
            printf("Writing... %.2f%%%c", p, DBG_CHAR);
    }
    if (len>0) err |= spi_flash_write_page(wbuf, adr, len);

    uint bench_tot_end_tick = bench_tic();
    time_to_write = bench_tot_end_tick - bench_tot_start_tick;

    return err;
}

int spi_flash_verify(uchar* wbuf, uint adr, uint len)
{
    DEBUG("spi_flash_verify: adr=0x%x, len=0x%x\n", adr, len);
    FT_STATUS err = 0;
    uint total_len = len;
    uchar rbuf[4096];
    uint rlen;
    int i;

    uint bench_tot_start_tick = bench_tic();

    for (; len; len -= rlen, adr += rlen, wbuf += rlen) {
        if (len >= 4096) 	rlen = 4096;
        else			rlen = len;

        err |= spi_flash_read(rbuf, adr, rlen);

        for (i = 0; i<rlen; i++) {
            if (rbuf[i] != wbuf[i]) {
                printf("0x%x : 0x%x != 0x%x\n", adr + i, rbuf[i], wbuf[i]);
                return -1;
            }
        }
        printf("Verifing... %.2f%%%c", ((double)(total_len - len) / (double)total_len)*100.0, DBG_CHAR); fflush(stdout);
    }

    uint bench_tot_end_tick = bench_tic();
    time_to_verify = bench_tot_end_tick - bench_tot_start_tick;

    return err;
}


int spi_flash_download(char* fname, uint adr)
{
    FT_STATUS err = 0;
    int i;

    // Setup variables
    fts_wcnt = 0;

    time_to_erase = 0;
    time_to_write = 0;
    time_to_verify = 0;

    // 1. File Open
    uchar* filedata = NULL;
    int filesize;

    err |= spi_flash_read_bin(fname, &filedata, &filesize);
    if (err) {
        printf("Open File %s\n", fname);
        if (filedata != NULL) free(filedata);
        return -1;
    }

    // 2. Reset Target
    err = spi_ftdx_reset_target();
    if (err) {
        printf("Reset target\n");
        if (filedata != NULL) free(filedata);
        return -1;
    }

    // 3. Reset Flash
    err = spi_flash_reset();
    if (err)	{
        printf("Reset flash\n");
        if (filedata != NULL) free(filedata);
        return -1;
    }

    // 4. Read Flash ID
    uint dev_id;
    err = spi_flash_read_id(&dev_id);
    if (err) {
        printf("Read flash ID\n");
        if (filedata != NULL) free(filedata);
        return FALSE;
    }

    err = 1;
    for (i = 0; flash_info[i].name; i++) {
        if (flash_info[i].dev_id == dev_id) {
            pflashInfo = &flash_info[i];
            blocksize = pflashInfo->blocksize;			// for fast access
            sectorsize = pflashInfo->sectorsize;
            pagesize = pflashInfo->pagesize;
            printf("flash: %s, id=0x%x, block size=0x%x, sector size=0x%x, page size = 0x%x\n",
                pflashInfo->name, dev_id, blocksize, sectorsize, pagesize);
            err = 0;
            break;
        }
    }
    if (err) {
        printf("Not supported flash: ID = 0x%x\n", dev_id);
        if (filedata != NULL) free(filedata);
        return -1;
    }

    // 5. Remove Protects
    err = spi_flash_remove_protect();
    if (err) {
        printf("Remove flash protects\n");
        if (filedata != NULL) free(filedata);
        return -1;
    }
    //filesize = 256*4;
    // 6. Erase
    err = spi_flash_erase(adr, filesize);
    if (err) {
        printf("Erase flash\n");
        if (filedata != NULL) free(filedata);
        return -1;
    }

    // 7. Write
    err = spi_flash_write(filedata, adr, filesize);
    if (err) {
        printf("Write flash\n");
        if (filedata != NULL) free(filedata);
        return -1;
    }

    // 8. Verify
    err = spi_flash_verify(filedata, adr, filesize);
    if (err) {
        printf("Verify flash\n");
        if (filedata != NULL) free(filedata);
        return -1;
    }

    // 9. Release target
    /*
    err = spi_ftdx_release_target();
    if (err) {
    ERROR("Release target\n");
    return -1;
    }
    */
    // 10. Release all IO to input
    err = spi_ftdx_release_port();
    if (err) {
        printf("Release port\n");
        if (filedata != NULL) free(filedata);
        return -1;
    }

    printf("\n");
    printf("File: %s, %d bytes @ adr = 0x%x\n", fname, filesize, adr);
    printf("Erasing time: %d bytes @ %.2f sec\n", filesize, (double)(time_to_erase) / 1000);
    printf("Writing time: %d bytes @ %.2f sec, %5.2f KB/sec\n", filesize, (double)(time_to_write) / 1000, ((double)(filesize)) / 1024.0 / ((double)(time_to_write) / 1000.0));
    printf("Verifying time: %d bytes @ %.2f sec\n", filesize, (double)(time_to_verify) / 1000);
    printf("Done!\n");
    if (filedata != NULL) free(filedata);

    return 0;
}


//******************************************************************************
// register top-level functions
//------------------------------------------------------------------------------
int spi_reg_read(ushort adr, uint* rdat)
{
    DEBUG("spi_reg_read: adr=0x%x\n", adr);
    FT_STATUS err = 0;

    uchar wbuf[2];
    wbuf[0] = (uchar)(adr >> 8);		// address
    wbuf[1] = (uchar)(adr);

    err |= spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(wbuf, 2);

    uint rbuf;
    err |= spi_ftdx_read_bytes((uchar*)&rbuf, 4);

    err |= spi_ftdx_toggle_cs(1);
    *rdat = ntohl(rbuf);
    return err;
}


int spi_reg_write(ushort adr, uint wdat)
{
    DEBUG("spi_reg_write: adr=0x%x\n", adr);
    FT_STATUS err = 0;

    uchar wbuf[2];
    wbuf[0] = (uchar)(adr >> 8);		// address
    wbuf[1] = (uchar)(adr);

    err |= spi_ftdx_toggle_cs(0);
    err |= spi_ftdx_write_bytes(wbuf, 2);

    uint wbuf2 = ntohl(wdat);
    err |= spi_ftdx_write_bytes((uchar*)&wbuf2, 4);
    err |= spi_ftdx_toggle_cs(1);

    err |= spi_ftdx_flush();
    return err;
}
