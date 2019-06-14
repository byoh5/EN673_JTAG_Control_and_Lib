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
#include "jtag_chain.h"



// Set up and execute a burst write to a contiguous set of addresses
UINT32 jtag_burst_write(void *data, UINT32 word_size_bytes, UINT32 word_count, unsigned long start_address, UINT module)
{

	unsigned char opcode;
	UINT32 datawords[2] = { 0, 0 };
	UINT32 i;
	UINT32 crc_calc = 0;
	UCHAR crc_match = 0;
	UINT32 word_size_bits;
	unsigned long addr;
	UINT32 bus_error_retries = 0;
	UINT32 successes;
    UINT32 j;
	UINT8 status;
	UINT32 statuswords[2] = { 0, 0 };
	UINT32 first_status_loop = 1;
	UINT32 err = 0;

	//	printf("Doing burst write, word size %d, word count %d, start address 0x%lx\n", word_size_bytes, word_count, start_address);
	word_size_bits = word_size_bytes << 3;

	if (word_count <= 0) {
		printf("Ignoring illegal burst write size (%d)\n", word_count);
		return 0;
	}




	// Select the appropriate opcode
	switch (module) {
	case DC_WISHBONE:
	case DC_WISHBONE1:
	case DC_WISHBONE2:
	case DC_WISHBONE3:
		if (word_size_bytes == 1) opcode = DBG_WB_CMD_BWRITE8;
		else if (word_size_bytes == 2) opcode = DBG_WB_CMD_BWRITE16;
		else if (word_size_bytes == 4) opcode = DBG_WB_CMD_BWRITE32;
		else {
			printf("Tried WB burst write with invalid word size (%0x), defaulting to 4-byte words", word_size_bytes);
			opcode = DBG_WB_CMD_BWRITE32;
		}
		break;
	case DC_CPU0:
	case DC_CPU1:
	case DC_CPU2:

		if (word_size_bytes == 4) opcode = DBG_CPU0_CMD_BWRITE32;
		else {
			printf("Tried CPU0 burst write with invalid word size (%0x), defaulting to 4-byte words", word_size_bytes);
			opcode = DBG_CPU0_CMD_BWRITE32;
		}
		break;

	default:
		printf("ERROR! Illegal debug chain selected while doing burst WRITE!\n");
        return ERR_MPSSE_BURST_WRITE_STREAM;
	}

	//wb_burst_write_retry_full:
	i = 0;
	addr = start_address;
	//wb_burst_write_retry_partial:
	crc_calc = 0xffffffff;
	successes = 0;

	// Send burst command, return to idle state
	if (err |= jtag_burst_command(opcode, addr, (word_count - i), module))  // word_count-i in case of partial retry
		return err;

	// Get us back to shift_dr mode to write a burst
	err |= tap_idle2shift_dr();

	// Write a start bit (a 1) so it knows when to start counting
	//   err |= jtag_write_bit(TDO);// REMOVE OBY



	status = 0;
	j = 0;
	while (!status) {  // Status indicates whether there is a word available to write.  Wait until it returns true.
	//		printf("&");
		if (j > RETRY_NUM){
			printf("No Answer!");
            return ERR_MPSSE_BURST_WRITE_STREAM;
		}
		err |= MPSSE_read_write_TDI(1, 0x0, &status);
		j++;
		//	Sleep(100);
	}

	err |= MPSSE_TDI_bit(1, 0x1, 0);

	err |= MPSSE_TDO_write_read_byte(word_size_bytes*word_count, (BYTE*)data, NULL, 1);

	for (i = 0; i <word_count; i++)
	{
		crc_calc = jtag_compute_crc(crc_calc, ((unsigned long *)data)[i], word_size_bits);
	}

	err |= MPSSE_write_stream(&crc_calc, 32, 0);
	
	status = 0;
	j = 0;
	while (!status) {  // Status indicates whether there is a word available to write.  Wait until it returns true.
	//		printf("!");
		if (j > RETRY_NUM){
			printf("No Answer!");
            return ERR_MPSSE_BURST_WRITE_STREAM;
		}
		err |= MPSSE_read_write_TDI(1, 0x0, &status);
		j++;
		//	Sleep(100);
	}

	err |= MPSSE_read_write_TDI(1, 0x0, &status);

	datawords[0] = 0;
	
	err |= MPSSE_TMS_bit(1, 0x01, 0);
    err |= tap_shift2idle2();

	if (!status) {
		printf("CRC ERROR! match bit after write is %i (computed CRC 0x%x)", crc_match, crc_calc);
		return ERR_MPSSE_CRC;	// 20190103 - CRC error
	}
	else{
		//	printf("CRC OK! match bit after write is %i (computed CRC 0x%x)", crc_match, crc_calc);
	}
	//	else printf("CRC OK!\n");

	//	printf("write CRC Computed 0x%x\n", crc_calc);
	return err;
}



// burst command
//	Name 	| JCMD_SUB (1) | op (JBOP_W=5) | adr (JBADR_W=32) | blen (JBLEN_W=15) |
UINT32 jtag_burst_command(UINT32 op, UINT32 adr, UINT32 blen, UINT8 module) {
    
    UINT32 err = 0;
//	(JCMD_W + JBOP_W + JBADR_W + JBLEN_W)
	unsigned long long int wstream = ((unsigned long long int)op << (JBLEN_W + JBADR_W)) | ((unsigned long long int)adr << JBLEN_W) | (blen);
	err |= jtag_select_module(module);
    err |= tap_idle2shift_dr();
    err |= jtag_chain_write_stream((UINT32*)&wstream, JDAT_W, 1);	// write data, ds_exit1
    err |= tap_shift2idle2();
	return err;
}

#define ADBG_CRC_POLY 0xedb88320

UINT32 jtag_compute_crc(UINT32 crc_in, UINT32 data_in, UINT32 length_bits)
{
	UINT32 i;
	UINT32 d, c;
	UINT32 crc_out = crc_in;

	for (i = 0; i < length_bits; i = i + 1)
	{
		d = ((data_in >> i) & 0x1) ? 0xffffffff : 0;
		c = (crc_out & 0x1) ? 0xffffffff : 0;
		crc_out = crc_out >> 1;
		crc_out = crc_out ^ ((d ^ c) & ADBG_CRC_POLY);
	}
	return crc_out;
}



UINT32 jtag_burst_read(UINT32 word_size_bytes, UINT32 word_count, unsigned long start_address, void *data, UINT module)
{
	unsigned char opcode;
	UINT8 status;
	unsigned long instream;
	UINT32 i, j;
	UINT32 crc_calc = 0;
	UINT32 crc_read = 0;
	unsigned char word_size_bits;
	UINT32 out_data = 0;
	UINT32 in_data = 0;
	unsigned long addr;
	UINT32 bus_error_retries = 0;
	UINT32 err=0;

	// Silence GCC
	(void)in_data;
	(void)out_data;

	//	printf("Doing burst read, word size %d, word count %d, start address 0x%lX\n", word_size_bytes, word_count, start_address);

	if (word_count <= 0) {
		printf("Ignoring illegal read burst length (%d)\n", word_count);
		return 0;
	}

	instream = 0;
	word_size_bits = word_size_bytes << 3;

	// Select the appropriate opcode
	switch (module) {
	case DC_WISHBONE:
	case DC_WISHBONE1:
	case DC_WISHBONE2:
	case DC_WISHBONE3:
		if (word_size_bytes == 1) opcode = DBG_WB_CMD_BREAD8;
		else if (word_size_bytes == 2) opcode = DBG_WB_CMD_BREAD16;
		else if (word_size_bytes == 4) opcode = DBG_WB_CMD_BREAD32;
		else {
			printf("Tried burst read with invalid word size (%0x), defaulting to 4-byte words\n", word_size_bytes);
			opcode = DBG_WB_CMD_BREAD32;
		}
		break;
	case DC_CPU0:
	case DC_CPU1:
	case DC_CPU2:
		if (word_size_bytes == 4) opcode = DBG_CPU0_CMD_BREAD32;
		else {
			printf("Tried burst read with invalid word size (%0x), defaulting to 4-byte words\n", word_size_bytes);
			opcode = DBG_CPU0_CMD_BREAD32;
		}
		break;

	default:
		printf("ERROR! Illegal debug chain selected while doing burst read!\n");
        return ERR_MPSSE_BURST_READ_STREAM;
	}

//wb_burst_read_retry_full:
	i = 0;
	addr = start_address;
//wb_burst_read_retry_partial:
	crc_calc = 0xffffffff;


	// Send the BURST READ command, returns TAP to idle state
    if (err |= jtag_burst_command(opcode, addr, (word_count - i), module)) return err; // word_count-i in case of partial retry 

	// Get us back to shift_dr mode to read a burst
    if (err |= tap_idle2shift_dr()) return err;

	// We do not adjust for the DR length here.  BYPASS regs are loaded with 0,
	// and the debug unit waits for a '1' status bit before beginning to read data.


	status = 0;
	j = 0;
	while (!status) {  // Status indicates whether there is a word available to read.  Wait until it returns true.
		//			printf("*");
        if (j>RETRY_NUM)	return ERR_MPSSE_BURST_READ_STREAM|err;
        if (err |= MPSSE_read_write_TDI(1, 0, &status)) return err;
		j++;
		// If max count exceeded, retry starting with the failure address
	}

	// Repeat for each word: wait until ready = 1, then read word_size_bits bits.


    if (err |= MPSSE_TDO_write_read_byte(word_size_bytes*word_count, NULL, (BYTE*)data, 1)) return err;


	for (i=0; i <(word_size_bytes*word_count)/4; i++)
	{
		crc_calc = jtag_compute_crc(crc_calc, ((unsigned long *)data)[i], word_size_bits);
	}

	// All bus data was read.  Read the data CRC from the debug module.
    if (err |= jtag_chain_read_write_stream(&out_data, &crc_read, 32, 0, 0)) return err;


    if (err |= MPSSE_TMS_bit(1, 0x01, 0)) return err;
    if (err |= tap_shift2idle2()) return err;

	if (crc_calc != crc_read) {
        if (crc_read == 0xffffffff) return ERR_MPSSE_BURST_READ_STREAM; // ASUME that 0xffffffff is not a crc error, this may connection error!
		else						return ERR_MPSSE_CRC;				// 20190103 - CRC error
		printf("CRC ERROR! Computed 0x%x, read CRC 0x%x\n", crc_calc, crc_read);
	}
	else{
		//printf("CRC OK!	   Computed 0x%x, read CRC 0x%x\n", crc_calc, crc_read);
	}
	//	else printf("CRC OK!  0x%x \n", crc_calc);

	//   printf("CRC Computed 0x%x, read CRC 0x%x\n", crc_calc, crc_read);


	// Now, read the error register, and retry/recompute as necessary.
	return err;
}

UINT32 jtag_write32(UINT32 adr, UINT32 data, UINT32 module) {
	UINT32 err=0;
	EnterCriticalSection(&CriticalSection);
	err |= jtag_select_module(module);
	err |= jtag_burst_write((void *)&data, 4, 1, adr, module);
	LeaveCriticalSection(&CriticalSection);
	return err;
}

UINT32 jtag_write_block32(UINT32 adr, UINT32 *data, UINT32 len, UINT32 module) {
	UINT32 err=0;
	EnterCriticalSection(&CriticalSection);
	err |= jtag_select_module(module);
	err |= jtag_burst_write((void *)data, 4, len, adr, module);  // 'len' is words.
	LeaveCriticalSection(&CriticalSection);
	return err;
}

UINT32 jtag_write_block8(UINT32 adr, UINT8 *data, UINT32 len, UINT32 module) {
	UINT32 err = 0;
	EnterCriticalSection(&CriticalSection);
	err |= jtag_select_module(module);
	err |= jtag_burst_write((void *)data, 1, len, adr, module);  // 'len' is words.
	LeaveCriticalSection(&CriticalSection);
	return err;
}


UINT32 jtag_read32(UINT32 adr, UINT32 *data, UINT32 module) {
	UINT32 err=0;
	EnterCriticalSection(&CriticalSection);
	err |= jtag_select_module(module);
	err |= jtag_burst_read(4, 1, adr, (void *)data, module);
	LeaveCriticalSection(&CriticalSection);
	return err;
}

UINT32 jtag_read_block32(UINT32 adr, UINT32 *data, UINT32 len, UINT32 module) {
	UINT32 err=0;
	EnterCriticalSection(&CriticalSection);
	err |= jtag_select_module(module);
	err |= jtag_burst_read(4, len, adr, (void *)data, module);
	LeaveCriticalSection(&CriticalSection);
	return err;
}

UINT32 jtag_read_block8(UINT32 adr, UINT8 *data, UINT32 len, UINT32 module) {
	UINT32 err = 0;
	EnterCriticalSection(&CriticalSection);
	err |= jtag_select_module(module);
	err |= jtag_burst_read(1, len, adr, (void *)data, module);
	LeaveCriticalSection(&CriticalSection);
	return err;
}


UINT32 jtag_wb_write32(UINT32 adr, UINT32 data, UINT32 cpu){
	UINT32 err=0;
	UINT32 wb = cpu - 4;
	if (cpu == WBCOM) wb = cpu;
	err |= jtag_write32(adr, data, wb);
	return err;
}

UINT32 jtag_wb_read32(UINT32 adr, UINT32 *data, UINT32 cpu){
	UINT32 err=0;
	UINT32 wb = cpu - 4;
	if (cpu == WBCOM) wb = cpu;
	err |= jtag_read32(adr, data, wb);
	return err;
}


INT32 jtag_gdb_burst_read32(UINT32* rbuf,INT32 len_words, UINT32 adr, UINT32 module){
//	printf("jtag_gdb_burst_read32: len = %d, adr=0x%x\n", len_words, adr);
	int err = 0;

	if (!len_words) return -1;

	EnterCriticalSection(&CriticalSection);

	err |= jtag_select_module(module);

	for (; len_words >= BLEN_WORD; len_words -= BLEN_WORD, rbuf += BLEN_WORD, adr += BLEN_BYTE)
		err |= jtag_burst_read(4, BLEN_WORD, adr, rbuf, module);
	if (len_words>0)
		err |= jtag_burst_read(4, len_words, adr, rbuf, module);

	LeaveCriticalSection(&CriticalSection);
	return err;
}


INT32 jtag_gdb_burst_write32(UINT32* wbuf, INT32 len_words, UINT32 adr, UINT32 module){
//	DEBUG("jtag_gdb_burst_write32: wbuf[0]=0x%08x, len = %d, adr=0x%08x\n", wbuf[0], len_words, adr);
	int err = 0;

	if (!len_words) return -1;

	EnterCriticalSection(&CriticalSection);;

	err |= jtag_select_module(module);

	for (; len_words >= BLEN_WORD; len_words -= BLEN_WORD, wbuf += BLEN_WORD, adr += BLEN_BYTE)
		err |= jtag_burst_write(wbuf, 4, BLEN_WORD, adr, module);
	if (len_words>0)
		err |= jtag_burst_write(wbuf, 4, len_words, adr, module);

	//jtag_flush();
	LeaveCriticalSection(&CriticalSection);
	return err;
}