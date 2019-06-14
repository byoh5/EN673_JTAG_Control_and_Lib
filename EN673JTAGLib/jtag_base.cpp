/* Copyright (c) 2015-2025 Eyenix Corporation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. The name of Eyenix may not be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an Eyenix
* product.
*
* THIS SOFTWARE IS PROVIDED BY EYENIX "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL EYENIX BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
*/
#include "stdafx.h"
#include <windows.h>
#include <stdio.h>
#include "ftd2xx.h"
#include "spr_defs.h"
#include "EN673JTAG.h"
#include "jtag_base.h"
#include "jtag_tap.h"
#include "jtag_module.h"
#include "jtag_rw.h"
#include "jtag_example.h"
#include "jtag_debug.h"
#include "jtag_def.h"



BYTE byOutputBuffer[65536];		// Buffer to hold MPSSE commands and data to be sent to the FT2232H
BYTE byInputBuffer[65536];		// Buffer to hold data read from the FT2232H
FT_STATUS ftStatus;				// Result of each D2XX call
FT_HANDLE ftHandle;				// Handle of the FTDI device
DWORD dwNumDevs;				// The number of devices

DWORD dwCount = 0;				// General loop index
DWORD dwNumBytesToSend = 0;		// Index to the output buffer
DWORD dwNumBytesSent = 0;		// Count of actual bytes sent - used with FT_Write
DWORD dwNumBytesToRead = 0;		// Number of bytes available to read in the driver's input buffer
DWORD dwNumBytesRead = 0;		// Count of actual bytes read - used with FT_Read

DWORD dwClockDivisor = 0x10;	// Value of clock divisor, SCL Frequency = 60/((1+0x05DB)*2) (MHz) = 20khz

UINT32 MPSSE_GETListNum(void)
{
	DWORD devnum = 0;
	ftStatus = FT_CreateDeviceInfoList(&devnum);
	// Get the number of FTDI devices
	return (UINT32)(devnum/4);
}


UINT32 MPSSE_init_withDescirption(UINT32 clkdiv,UINT8* desciption)
{

	// Does an FTDI device exist?

	printf("Checking for FTDI devices...\n");

	ftStatus = FT_CreateDeviceInfoList(&dwNumDevs);

	printf("Device %d\n", dwNumDevs);
	// Get the number of FTDI devices
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in getting the number of devices\n");
		return ERR_MPSSE_INIT;					// Exit with error
	}

	if (dwNumDevs < 1)				// Exit if we don't see any
	{
		printf("There are no FTDI devices installed\n");
		return ERR_MPSSE_INIT;					// Exist with error
	}

	// Open the port - For this application note, we'll assume the first device is a FT2232H or FT4232H
	// Further checks can be made against the device descriptions, locations, serial numbers, etc. 
	//		before opening the port.

	//printf("\nAssume first device has the MPSSE and open it...\n");
	printf("Description %s\n", desciption);
	ftStatus = FT_OpenEx(desciption, FT_OPEN_BY_DESCRIPTION, &ftHandle);
	if (ftStatus != FT_OK)
	{
		printf("Open Failed with error %d\n", ftStatus);
		return ERR_MPSSE_INIT;					// Exit with error
	}

	FT_DEVICE ftDevice;
	DWORD deviceID;
	char SerialNumber[16];
	char Description[64];

	ftStatus = FT_GetDeviceInfo(
		ftHandle,
		&ftDevice,
		&deviceID,
		SerialNumber,
		Description,
		NULL
		);
	if (ftStatus == FT_OK) {
		printf(" ID=0x%x\n", deviceID);
		printf(" SerialNumber=%s\n", SerialNumber);
		printf(" Description=%s\n", Description);
	}
	else {
		// FT_GetDeviceType FAILED!
	}


	// Configure port parameters

	printf("\nConfiguring port for MPSSE use...\n");

	ftStatus |= FT_ResetDevice(ftHandle);
	//Reset USB device

#if 1
	//Purge USB receive buffer first by reading out all old data from FT2232H receive buffer
	ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
	// Get the number of bytes in the FT2232H receive buffer
	if ((ftStatus == FT_OK) && (dwNumBytesToRead > 0))
		FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
	//Read out the data from FT2232H receive buffer
	ftStatus |= FT_SetUSBParameters(ftHandle, 65536, 65535);
	//Set USB request transfer sizes to 64K
	ftStatus |= FT_SetChars(ftHandle, false, 0, false, 0);
	//Disable event and error characters
	ftStatus |= FT_SetTimeouts(ftHandle, 5000, 5000);
	//Sets the read and write timeouts in milliseconds 
	ftStatus |= FT_SetLatencyTimer(ftHandle, 0);
	//Set the latency timer (default is 16mS)
	ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x00);
	//Reset controller
	ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x02);
#endif // 0

#if 0 // FT2232D
	//Purge USB receive buffer first by reading out all old data from FT2232H receive buffer
	ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
	// Get the number of bytes in the FT2232H receive buffer
	if ((ftStatus == FT_OK) && (dwNumBytesToRead > 0))
		FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
	//Read out the data from FT2232H receive buffer
	ftStatus |= FT_SetUSBParameters(ftHandle, 128, 128);
	//Set USB request transfer sizes to 64K
	ftStatus |= FT_SetChars(ftHandle, false, 0, false, 0);
	//Disable event and error characters
	ftStatus |= FT_SetTimeouts(ftHandle, 0, 5000);
	//Sets the read and write timeouts in milliseconds 
	ftStatus |= FT_SetLatencyTimer(ftHandle, 16);
	//Set the latency timer (default is 16mS)
	ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x00);
	//Reset controller
	ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x02);
	//Enable MPSSE mode  
#endif // 1 // FT2232D

	if (ftStatus != FT_OK)
	{
		printf("Error in initializing the MPSSE %d\n", ftStatus);
		FT_Close(ftHandle);
		return ERR_MPSSE_INIT;					// Exit with error
	}

	Sleep(50); // Wait for all the USB stuff to complete and work		

	/*
	// -----------------------------------------------------------
	// At this poUINT32, the MPSSE is ready for commands
	// -----------------------------------------------------------
	*/

	/*
	// -----------------------------------------------------------
	// Synchronize the MPSSE by sending a bogus opcode (0xAA),
	//		The MPSSE will respond with "Bad Command" (0xFA) followed by
	//		the bogus opcode itself.
	// -----------------------------------------------------------
	*/
	byOutputBuffer[dwNumBytesToSend++] = 0xAA;//'\xAA';
	//Add bogus command ‘xAA?to the queue
	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	// Send off the BAD commands
	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er
	do
	{
		ftStatus = FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
		// Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));
	//or Timeout

	bool bCommandEchod = false;

	ftStatus = FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
	//Read out the data from input buffer
	for (dwCount = 0; dwCount < dwNumBytesRead - 1; dwCount++)
		//Check if Bad command and echo command received
	{
		if ((byInputBuffer[dwCount] == 0xFA) && (byInputBuffer[dwCount + 1] == 0xAA))
		{
			bCommandEchod = true;
			break;
		}
	}
	if (bCommandEchod == false)
	{
		printf("Error in synchronizing the MPSSE\n");
		FT_Close(ftHandle);
		return ERR_MPSSE_INIT;					// Exit with error
	}

	/*
	// -----------------------------------------------------------
	// Configure the MPSSE settings for JTAG
	//		Multple commands can be sent to the MPSSE with one FT_Write
	// -----------------------------------------------------------
	*/
	dwNumBytesToSend = 0;			// Start with a fresh index


	// Set up the Hi-Speed specific commands for the FTx232H

#if 1  //for the FTx232H
	byOutputBuffer[dwNumBytesToSend++] = 0x8A;
	// Use 60MHz master clock (disable divide by 5)
	byOutputBuffer[dwNumBytesToSend++] = 0x97;
	// Turn off adaptive clocking (may be needed for ARM)
	byOutputBuffer[dwNumBytesToSend++] = 0x8D;
	// Disable three-phase clocking
	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	// Send off the HS-specific commands
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in Set up the Hi-Speed specific commands for the FTx232H \n");
		return ERR_MPSSE_INIT;					// Exit with error
	}
	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er  
#endif // 1  //for the FTx232H


	// Set initial states of the MPSSE UINT32erface - low byte, both pin directions and output values
	//		Pin name	Signal	Direction	Config	Initial State	Config
	//		ADBUS0		TCK		output		1		low				0
	//		ADBUS1		TDI		output		1		low				0
	//		ADBUS2		TDO		input		0						0
	//		ADBUS3		TMS		output		1		high			1
	//		ADBUS4		GPIOL0	input		0						0
	//		ADBUS5		GPIOL1	input		0						0
	//		ADBUS6		GPIOL2	input		0						0
	//		ADBUS7		GPIOL3	input		0						0

	byOutputBuffer[dwNumBytesToSend++] = 0x80;
	// Set data bits low-byte of MPSSE port
	byOutputBuffer[dwNumBytesToSend++] = 0xf8;
	// Initial state config above
	byOutputBuffer[dwNumBytesToSend++] = 0x0B;
	// Direction config above

	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	// Send off the low GPIO config commands
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in Set initial states of the MPSSE UINT32erface fail\n");
		return ERR_MPSSE_INIT;					// Exit with error
	}
	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er


	// Set initial states of the MPSSE UINT32erface - high byte, both pin directions and output values
	//		Pin name	Signal	Direction	Config	Initial State	Config
	//		ACBUS0		GPIOH0	input		0						0
	//		ACBUS1		GPIOH1	input		0						0
	//		ACBUS2		GPIOH2	input		0						0
	//		ACBUS3		GPIOH3	input		0						0
	//		ACBUS4		GPIOH4	input		0						0
	//		ACBUS5		GPIOH5	input		0						0
	//		ACBUS6		GPIOH6	input		0						0
	//		ACBUS7		GPIOH7	input		0						0

	byOutputBuffer[dwNumBytesToSend++] = 0x82;
// Set data bits low-byte of MPSSE port
byOutputBuffer[dwNumBytesToSend++] = 0x00;
// Initial state config above
byOutputBuffer[dwNumBytesToSend++] = 0x00;
// Direction config above

ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
// Send off the high GPIO config commands
if (ftStatus != FT_OK)			// Did the command execute OK?
{
	printf("Error in Set initial states of the MPSSE UINT32erface\n");
	return ERR_MPSSE_INIT;					// Exit with error
}
dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

// Set TCK frequency 
// TCK = 60MHz /((1 + [(1 +0xValueH*256) OR 0xValueL])*2)
printf("ClockDivisor %d\n", clkdiv);
byOutputBuffer[dwNumBytesToSend++] = 0x86;
//Command to set clock divisor
byOutputBuffer[dwNumBytesToSend++] = clkdiv & 0xFF;
//Set 0xValueL of clock divisor
byOutputBuffer[dwNumBytesToSend++] = (clkdiv >> 8) & 0xFF;
//Set 0xValueH of clock divisor
ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
// Send off the clock divisor commands
if (ftStatus != FT_OK)			// Did the command execute OK?
{
	printf("Error in Set TCK frequency\n");
	return ERR_MPSSE_INIT;					// Exit with error
}
dwNumBytesToSend = 0;			// Reset output buffer poUINT32er


// Disable UINT32ernal loop-back

byOutputBuffer[dwNumBytesToSend++] = 0x85;
// Disable loopback

ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
// Send off the loopback command
if (ftStatus != FT_OK)			// Did the command execute OK?
{
	printf("Error in Disable UINT32ernal loop-back\n");
	return ERR_MPSSE_INIT;					// Exit with error
}
dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

return 0;						// Exit with success
}


UINT32 MPSSE_init(UINT32 clkdiv, char channel)
{

	// Does an FTDI device exist?

	printf("Checking for FTDI devices...\n");

	ftStatus = FT_CreateDeviceInfoList(&dwNumDevs);

	printf("Device %d\n", dwNumDevs);
	// Get the number of FTDI devices
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in getting the number of devices\n");
		return ERR_MPSSE_INIT;					// Exit with error
	}

	if (dwNumDevs < 1)				// Exit if we don't see any
	{
		printf("There are no FTDI devices installed\n");
		return ERR_MPSSE_INIT;					// Exist with error
	}

	// Open the port - For this application note, we'll assume the first device is a FT2232H or FT4232H
	// Further checks can be made against the device descriptions, locations, serial numbers, etc. 
	//		before opening the port.

	//printf("\nAssume first device has the MPSSE and open it...\n");

	if (channel == 'A')
	{
		ftStatus = FT_OpenEx("EN673A", FT_OPEN_BY_SERIAL_NUMBER, &ftHandle);
		if (ftStatus != FT_OK) ftStatus = FT_OpenEx("ENXA", FT_OPEN_BY_SERIAL_NUMBER, &ftHandle);
		if (ftStatus != FT_OK)
		{
			printf("Open Failed with error %d\n", ftStatus);
			return ERR_MPSSE_INIT;					// Exit with error
		}
	}
	else if (channel == 'B')
	{
		ftStatus = FT_OpenEx("EN673B", FT_OPEN_BY_SERIAL_NUMBER, &ftHandle);
		if (ftStatus != FT_OK) ftStatus = FT_OpenEx("ENXB", FT_OPEN_BY_SERIAL_NUMBER, &ftHandle);
		if (ftStatus != FT_OK)
		{
			printf("Open Failed with error %d\n", ftStatus);
			return ERR_MPSSE_INIT;					// Exit with error
		}
	}
	else
	{
		printf("Channel Open Failed");
		return ERR_MPSSE_INIT;					// Exit with error
	}

	FT_DEVICE ftDevice;
	DWORD deviceID;
	char SerialNumber[16];
	char Description[64];
	
	ftStatus = FT_GetDeviceInfo(
		ftHandle,
		&ftDevice,
		&deviceID,
		SerialNumber,
		Description,
		NULL
		);
	if (ftStatus == FT_OK) {
		printf(" ID=0x%x\n", deviceID);
		printf(" SerialNumber=%s\n", SerialNumber);
		printf(" Description=%s\n", Description);
	}
	else {
		// FT_GetDeviceType FAILED!
	}

	
	// Configure port parameters

	printf("\nConfiguring port for MPSSE use...\n");

	ftStatus |= FT_ResetDevice(ftHandle);
	//Reset USB device

#if 1
	//Purge USB receive buffer first by reading out all old data from FT2232H receive buffer
	ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
	// Get the number of bytes in the FT2232H receive buffer
	if ((ftStatus == FT_OK) && (dwNumBytesToRead > 0))
		FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
	//Read out the data from FT2232H receive buffer
	ftStatus |= FT_SetUSBParameters(ftHandle, 65536, 65535);
	//Set USB request transfer sizes to 64K
	ftStatus |= FT_SetChars(ftHandle, false, 0, false, 0);
	//Disable event and error characters
	ftStatus |= FT_SetTimeouts(ftHandle, 0, 5000);
	//Sets the read and write timeouts in milliseconds 
	ftStatus |= FT_SetLatencyTimer(ftHandle, 0);
	//Set the latency timer (default is 16mS)
	ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x00);
	//Reset controller
	ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x02);
#endif // 0

#if 0 // FT2232D
	//Purge USB receive buffer first by reading out all old data from FT2232H receive buffer
	ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
	// Get the number of bytes in the FT2232H receive buffer
	if ((ftStatus == FT_OK) && (dwNumBytesToRead > 0))
		FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
	//Read out the data from FT2232H receive buffer
	ftStatus |= FT_SetUSBParameters(ftHandle, 128, 128);
	//Set USB request transfer sizes to 64K
	ftStatus |= FT_SetChars(ftHandle, false, 0, false, 0);
	//Disable event and error characters
	ftStatus |= FT_SetTimeouts(ftHandle, 0, 5000);
	//Sets the read and write timeouts in milliseconds 
	ftStatus |= FT_SetLatencyTimer(ftHandle, 16);
	//Set the latency timer (default is 16mS)
	ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x00);
	//Reset controller
	ftStatus |= FT_SetBitMode(ftHandle, 0x0, 0x02);
	//Enable MPSSE mode  
#endif // 1 // FT2232D

	if (ftStatus != FT_OK)
	{
		printf("Error in initializing the MPSSE %d\n", ftStatus);
		FT_Close(ftHandle);
        return ERR_MPSSE_INIT;					// Exit with error
	}

	Sleep(50); // Wait for all the USB stuff to complete and work		

	/*
	// -----------------------------------------------------------
	// At this poUINT32, the MPSSE is ready for commands
	// -----------------------------------------------------------
	*/

	/*
	// -----------------------------------------------------------
	// Synchronize the MPSSE by sending a bogus opcode (0xAA),
	//		The MPSSE will respond with "Bad Command" (0xFA) followed by
	//		the bogus opcode itself.
	// -----------------------------------------------------------
	*/
	byOutputBuffer[dwNumBytesToSend++] = 0xAA;//'\xAA';
	//Add bogus command ‘xAA?to the queue
	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	// Send off the BAD commands
	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er
	do
	{
		ftStatus = FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
		// Get the number of bytes in the device input buffer
	} while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK));
	//or Timeout

	bool bCommandEchod = false;

	ftStatus = FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
	//Read out the data from input buffer
	for (dwCount = 0; dwCount < dwNumBytesRead - 1; dwCount++)
		//Check if Bad command and echo command received
	{
		if ((byInputBuffer[dwCount] == 0xFA) && (byInputBuffer[dwCount + 1] == 0xAA))
		{
			bCommandEchod = true;
			break;
		}
	}
	if (bCommandEchod == false)
	{
		printf("Error in synchronizing the MPSSE\n");
		FT_Close(ftHandle);
        return ERR_MPSSE_INIT;					// Exit with error
	}

	/*
	// -----------------------------------------------------------
	// Configure the MPSSE settings for JTAG
	//		Multple commands can be sent to the MPSSE with one FT_Write
	// -----------------------------------------------------------
	*/
	dwNumBytesToSend = 0;			// Start with a fresh index


	// Set up the Hi-Speed specific commands for the FTx232H

#if 1  //for the FTx232H
	byOutputBuffer[dwNumBytesToSend++] = 0x8A;
	// Use 60MHz master clock (disable divide by 5)
	byOutputBuffer[dwNumBytesToSend++] = 0x97;
	// Turn off adaptive clocking (may be needed for ARM)
	byOutputBuffer[dwNumBytesToSend++] = 0x8D;
	// Disable three-phase clocking
	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	// Send off the HS-specific commands
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in Set up the Hi-Speed specific commands for the FTx232H \n");
        return ERR_MPSSE_INIT;					// Exit with error
	}
	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er  
#endif // 1  //for the FTx232H


	// Set initial states of the MPSSE UINT32erface - low byte, both pin directions and output values
	//		Pin name	Signal	Direction	Config	Initial State	Config
	//		ADBUS0		TCK		output		1		low				0
	//		ADBUS1		TDI		output		1		low				0
	//		ADBUS2		TDO		input		0						0
	//		ADBUS3		TMS		output		1		high			1
	//		ADBUS4		GPIOL0	input		0						0
	//		ADBUS5		GPIOL1	input		0						0
	//		ADBUS6		GPIOL2	input		0						0
	//		ADBUS7		GPIOL3	input		0						0

	byOutputBuffer[dwNumBytesToSend++] = 0x80;
	// Set data bits low-byte of MPSSE port
	byOutputBuffer[dwNumBytesToSend++] = 0xf8;
	// Initial state config above
	byOutputBuffer[dwNumBytesToSend++] = 0x0B;
	// Direction config above

	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	// Send off the low GPIO config commands
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in Set initial states of the MPSSE UINT32erface fail\n");
        return ERR_MPSSE_INIT;					// Exit with error
	}
	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er


	// Set initial states of the MPSSE UINT32erface - high byte, both pin directions and output values
	//		Pin name	Signal	Direction	Config	Initial State	Config
	//		ACBUS0		GPIOH0	input		0						0
	//		ACBUS1		GPIOH1	input		0						0
	//		ACBUS2		GPIOH2	input		0						0
	//		ACBUS3		GPIOH3	input		0						0
	//		ACBUS4		GPIOH4	input		0						0
	//		ACBUS5		GPIOH5	input		0						0
	//		ACBUS6		GPIOH6	input		0						0
	//		ACBUS7		GPIOH7	input		0						0

	byOutputBuffer[dwNumBytesToSend++] = 0x82;
	// Set data bits low-byte of MPSSE port
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	// Initial state config above
	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	// Direction config above

	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	// Send off the high GPIO config commands
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in Set initial states of the MPSSE UINT32erface\n");
        return ERR_MPSSE_INIT;					// Exit with error
	}
	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

	// Set TCK frequency 
	// TCK = 60MHz /((1 + [(1 +0xValueH*256) OR 0xValueL])*2)
    printf("ClockDivisor %d\n", clkdiv);
	byOutputBuffer[dwNumBytesToSend++] = 0x86;
	//Command to set clock divisor
    byOutputBuffer[dwNumBytesToSend++] = clkdiv & 0xFF;
	//Set 0xValueL of clock divisor
    byOutputBuffer[dwNumBytesToSend++] = (clkdiv >> 8) & 0xFF;
	//Set 0xValueH of clock divisor
	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	// Send off the clock divisor commands
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in Set TCK frequency\n");
        return ERR_MPSSE_INIT;					// Exit with error
	}
	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er


	// Disable UINT32ernal loop-back

	byOutputBuffer[dwNumBytesToSend++] = 0x85;
	// Disable loopback

	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
	// Send off the loopback command
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in Disable UINT32ernal loop-back\n");
        return ERR_MPSSE_INIT;					// Exit with error
	}
	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

	return 0;						// Exit with success
}

UINT32 MPSSE_close(void){

	ftStatus = FT_Close(ftHandle);				// Close the port
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		printf("Error in FT_Close\n");
        return ERR_MPSSE_CLOSE;					// Exit with error
	}
	return 0;
}

UINT32 MPSSE_TMS_bit(UINT8 len, UINT8 bit, UINT8 edge){
	
//	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

	if (edge){
		byOutputBuffer[dwNumBytesToSend++] = 0x4A;	// +VE
	}
	else{
		byOutputBuffer[dwNumBytesToSend++] = 0x4B;	// -VE
	}

	byOutputBuffer[dwNumBytesToSend++] = len - 1;
	byOutputBuffer[dwNumBytesToSend++] = bit;
//	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
//	if (ftStatus != FT_OK)			// Did the command execute OK?
//	{
//		
//        return ERR_MPSSE_TMS_BIT;					// Exit with error
//	}
	return 0;
}

UINT32 MPSSE_TMS_TDO_bit(UINT8 len, UINT8 bit, UINT8 edge){

//	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

	if (edge){
		byOutputBuffer[dwNumBytesToSend++] = 0x6A;	// +VE
	}
	else{
		byOutputBuffer[dwNumBytesToSend++] = 0x6E;	// -VE
	}

	byOutputBuffer[dwNumBytesToSend++] = len - 1;
	byOutputBuffer[dwNumBytesToSend++] = bit;
//	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
//	if (ftStatus != FT_OK)			// Did the command execute OK?
//	{
//		
//        return ERR_MPSSE_TMS_TDO_BIT;					// Exit with error
//	}
	return 0;
}


UINT32 MPSSE_TDI_bit(UINT8 len, UINT8 bit, UINT8 edge){

//	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

	if (edge){
		byOutputBuffer[dwNumBytesToSend++] = 0x1A;	// +VE
	}
	else{
		byOutputBuffer[dwNumBytesToSend++] = 0x1B;	// -VE
	}
	byOutputBuffer[dwNumBytesToSend++] = len - 1;
	byOutputBuffer[dwNumBytesToSend++] = bit;
//	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
//	if (ftStatus != FT_OK)			// Did the command execute OK?
//	{
//		
//        return ERR_MPSSE_TDI_BIT;					// Exit with error
//	}
	return 0;
}



UINT32 MPSSE_TDI_TDO_bit(UINT8 len, UINT8 bit, UINT8 edge){

//	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

	if (edge){
		byOutputBuffer[dwNumBytesToSend++] = 0x3E;	// +VE
	}
	else{
		byOutputBuffer[dwNumBytesToSend++] = 0x3B;	// -VE
	}
	byOutputBuffer[dwNumBytesToSend++] = len - 1;
	byOutputBuffer[dwNumBytesToSend++] = bit;
//	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
//	if (ftStatus != FT_OK)			// Did the command execute OK?
//	{
//		
//        return ERR_MPSSE_TDI_TDO_BIT;					// Exit with error
//	}
	return 0;
}

inline int ftdx_flush()
{
    if (dwNumBytesToSend){
        ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
        dwNumBytesToSend = 0;
    }
    
    if (ftStatus != FT_OK)			// Did the command execute OK?
    {	
        return ERR_MPSSE_TDI_TDO_BIT;					// Exit with error
    } 
    return 0;
}


UINT32 MPSSE_TDO_read_bit(UINT8* data){

    ftdx_flush();

	ftStatus = FT_Read(ftHandle, data, 1, &dwNumBytesRead);
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		
        return ERR_MPSSE_TDO_READ_BIT;					// Exit with error
	}

	return 0;
}

UINT32 MPSSE_TDO_read_byte(UINT8* data, UINT cnt){

    ftdx_flush();

	ftStatus = FT_Read(ftHandle, data, cnt, &dwNumBytesRead);
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		
        return ERR_MPSSE_TDO_READ_BYTE;					// Exit with error
	}

	return 0;
}

UINT32 MPSSE_TDO_write_byte(UINT16 len, UINT8 *data, UINT8 edge){

//	dwNumBytesToSend = 0;			// Reset output buffer poUINT32er
	UINT32 i = 0;
	UINT16 jtag_len = len - 1;
	if (edge){
		byOutputBuffer[dwNumBytesToSend++] = 0x18;	// +VE
	}
	else{
		byOutputBuffer[dwNumBytesToSend++] = 0x19;	// -VE
	}
	byOutputBuffer[dwNumBytesToSend++] = jtag_len & 0xFF;
	byOutputBuffer[dwNumBytesToSend++] = (jtag_len >> 8) & 0xFF;
	for (i = 0; i < len; i++)	byOutputBuffer[dwNumBytesToSend++] = *(data + i);

//	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
//	if (ftStatus != FT_OK)			// Did the command execute OK?
//	{
//		
//        return ERR_MPSSE_TDO_WRITE_BYTE;					// Exit with error
//	}
	return 0;
}

UINT32 MPSSE_TDO_write_read_byte(UINT16 len,UINT8 *outdata ,UINT8 *byte, UINT8 edge){

	
	UINT32 i = 0;
	UINT16 jtag_len = len - 1;
	if (edge){
		byOutputBuffer[dwNumBytesToSend++] = 0x39;	// +VE
	}
	else{
		byOutputBuffer[dwNumBytesToSend++] = 0x3c;	// -VE
	}
	byOutputBuffer[dwNumBytesToSend++] = jtag_len & 0xFF;
	byOutputBuffer[dwNumBytesToSend++] = (jtag_len >> 8) & 0xFF;

	if (outdata == NULL){
		for (i = 0; i < len; i++)	byOutputBuffer[dwNumBytesToSend++] = 0x00;
	}
	else{
		for (i = 0; i < len; i++)	byOutputBuffer[dwNumBytesToSend++] = *(outdata + i);
	}
	ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
    dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

	do{
		ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
	} while ((ftStatus == FT_OK) && (dwNumBytesToRead != len));

	dwNumBytesToRead = len;
	ftStatus = FT_Read(ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
	if (ftStatus != FT_OK)			// Did the command execute OK?
	{
		DEBUG
            return ERR_MPSSE_TDO_WRITE_READ_BYTE;					// Exit with error
	}
	if (byte != NULL)	memcpy(byte, byInputBuffer, len);

	return 0;
}

UINT32 MPSSE_read_write_TDI(UINT len, UCHAR packet, UCHAR *in_bit){
	UCHAR readdata;

	if (MPSSE_TDI_TDO_bit(len, packet, 0))
	{
		DEBUG
            return ERR_MPSSE_READ_WRITE_TDI;	//Err
	}

    ftdx_flush();
	
    do{
		ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
	} while ((ftStatus == FT_OK) && (dwNumBytesToRead == 0));

	if (MPSSE_TDO_read_bit(&readdata)){
		DEBUG
            return ERR_MPSSE_READ_WRITE_TDI;	//Err
	}
	*in_bit = readdata >> (8 - len);

	return 0;
}

UINT32 MPSSE_read_write_TMS(UINT len, UCHAR packet, UCHAR *in_bit){
	UCHAR readdata;

    if (MPSSE_TMS_TDO_bit(len, packet, 0))	return ERR_MPSSE_READ_WRITE_TMS;	//Err

    ftdx_flush();

	do{
		ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
	} while ((ftStatus == FT_OK) && (dwNumBytesToRead == 0));

    if (MPSSE_TDO_read_bit(&readdata))	return ERR_MPSSE_READ_WRITE_TMS;	//Err

	*in_bit = readdata >> (8 - len);

	return 0;
}

UINT32 MPSSE_write_stream(UINT32 *in_data, UINT32 length_bits, UCHAR set_TMS)
{

	UINT i = 0;

	if (set_TMS) length_bits-=1;

	UINT retry = length_bits / 8;
	UINT left = length_bits % 8;
	UCHAR *pos;

	pos = (UCHAR*)in_data;


	for (i = 0; i < retry; i++){
        if (MPSSE_TDI_bit(8, *(pos + i), 0))	return ERR_MPSSE_WRITE_STREAM;	//Err
		//		printf("Indata : %8x\n", *(pos + i));
	}

	if (left){
        if (MPSSE_TDI_bit(left, *(pos + i), 0))	return ERR_MPSSE_WRITE_STREAM;	//Err
	}

	if (set_TMS){
		if ( (*(pos + i))  & (0x01<<left) ){
            if (MPSSE_TMS_bit(1, 0x81, 0)) return ERR_MPSSE_WRITE_STREAM;
		}
		else
		{
            if (MPSSE_TMS_bit(1, 0x01, 0)) return ERR_MPSSE_WRITE_STREAM;
		}
	}
	return 0;
}

UINT32 MPSSE_read_write_stream(UINT32 *in_data, UINT32 *out_data, UINT32 length_bits)
{
	UINT i = 0;
	UINT retry = length_bits / 8;
	UINT left = length_bits % 8;
	UINT readlen;
	UCHAR *pos, *posout;
	UINT8 leftdata;

	pos = (UCHAR*)in_data;
	posout = (UCHAR*)out_data;
	//	printf("retry %d, left %d\n", retry, left);

	readlen = retry + ((left) ? 1 : 0);
	//	printf("readlen %d \n", readlen);

	for (i = 0; i < retry; i++){
        if (MPSSE_TDI_TDO_bit(8, *(pos + i), 0))	return ERR_MPSSE_READ_WRITE_STREAM;	//Err
		//		printf("Indata : %8x\n", *(pos + i));
	}

	if (left){
        if (MPSSE_TDI_TDO_bit(left, *(pos + i), 0))	return ERR_MPSSE_READ_WRITE_STREAM;	//Err
	}

    ftdx_flush();


	do{
		ftStatus |= FT_GetQueueStatus(ftHandle, &dwNumBytesToRead);
	} while ((ftStatus == FT_OK) && (dwNumBytesToRead != readlen));

    if (MPSSE_TDO_read_byte((UINT8 *)out_data, retry))	return ERR_MPSSE_READ_WRITE_STREAM;	//Err

	if (left){
        if (MPSSE_TDO_read_bit(&leftdata))	return ERR_MPSSE_READ_WRITE_STREAM; //Err
		//		printf("read left data %x\n", leftdata);
		*(posout + retry) = leftdata >> (8 - left);
		//		printf("%x\n", *(posout + retry));
	}
	return 0;
}


UINT32 MPSSE_release_port(void){
    // Set initial states of the MPSSE UINT32erface - low byte, both pin directions and output values
    //		Pin name	Signal	Direction	Config	Initial State	Config
    //		ADBUS0		TCK		output		1		low				0
    //		ADBUS1		TDI		output		1		low				0
    //		ADBUS2		TDO		input		0						0
    //		ADBUS3		TMS		output		1		high			1
    //		ADBUS4		GPIOL0	input		0						0
    //		ADBUS5		GPIOL1	input		0						0
    //		ADBUS6		GPIOL2	input		0						0
    //		ADBUS7		GPIOL3	input		0						0

    byOutputBuffer[dwNumBytesToSend++] = 0x80;
    // Set data bits low-byte of MPSSE port
    byOutputBuffer[dwNumBytesToSend++] = 0xf8;
    // Initial state config above
    byOutputBuffer[dwNumBytesToSend++] = 0x00;
    // Direction config above

    ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
    // Send off the low GPIO config commands
    if (ftStatus != FT_OK)			// Did the command execute OK?
    {
        printf("Error in Set initial states of the MPSSE UINT32erface fail\n");
        return ERR_MPSSE_INIT;					// Exit with error
    }
    dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

    return 0;
}

UINT32 MPSSE_init_port(void){
    // Set initial states of the MPSSE UINT32erface - low byte, both pin directions and output values
    //		Pin name	Signal	Direction	Config	Initial State	Config
    //		ADBUS0		TCK		output		1		low				0
    //		ADBUS1		TDI		output		1		low				0
    //		ADBUS2		TDO		input		0						0
    //		ADBUS3		TMS		output		1		high			1
    //		ADBUS4		GPIOL0	input		0						0
    //		ADBUS5		GPIOL1	input		0						0
    //		ADBUS6		GPIOL2	input		0						0
    //		ADBUS7		GPIOL3	input		0						0

    byOutputBuffer[dwNumBytesToSend++] = 0x80;
    // Set data bits low-byte of MPSSE port
    byOutputBuffer[dwNumBytesToSend++] = 0xf8;
    // Initial state config above
    byOutputBuffer[dwNumBytesToSend++] = 0x0B;
    // Direction config above

    ftStatus = FT_Write(ftHandle, byOutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
    // Send off the low GPIO config commands
    if (ftStatus != FT_OK)			// Did the command execute OK?
    {
        printf("Error in Set initial states of the MPSSE UINT32erface fail\n");
        return ERR_MPSSE_INIT;					// Exit with error
    }
    dwNumBytesToSend = 0;			// Reset output buffer poUINT32er

    return 0;
}
