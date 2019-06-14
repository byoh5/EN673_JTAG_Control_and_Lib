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

UINT32 tap_reset(void){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err=0;
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// test logic reset
	err |= MPSSE_TMS_bit(3, 0x00, 0);	// Run-Test/Idle
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

UINT32 tap_reset_global(void){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err = 0;
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// global reset
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(3, 0x00, 0);	// Run-Test/Idle
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

UINT32 tap_reset_global0(void){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err = 0;
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// global reset
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(3, 0x00, 0);	// Run-Test/Idle
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

UINT32 tap_reset_global1(void){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err = 0;
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// global reset
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(8, 0xff, 0);	// 
	err |= MPSSE_TMS_bit(3, 0x00, 0);	// Run-Test/Idle
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

UINT32 tap_idle2shift_dr(void){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err=0;
	// 001
	err |= MPSSE_TMS_bit(3, 0x01, 0);	// Shift-DR
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

UINT32 tap_idle2shift_ir(void){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err=0;
	// 0011
	err |= MPSSE_TMS_bit(4, 0x03, 0);	// Shift-IR
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

UINT32 tap_shift2idle(void){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err=0;
	// 011
	err |= MPSSE_TMS_bit(3, 0x03, 0);	// Idle
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

UINT32 tap_shift2idle2(void){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err=0;
	// 01
	err |= MPSSE_TMS_bit(2, 0x01, 0);	// Idle
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

UINT32 tap_set_ir_debug(void){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err=0;

	err |= tap_idle2shift_ir();
	err |= MPSSE_TDI_bit(3, 0x00, 0);	// debug 1000
	err |= MPSSE_TMS_bit(1, 0x81, 0);
	err |= tap_shift2idle2();
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;

}

UINT32 tap_set_ir_idcode(){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err = 0;

	err |= tap_idle2shift_ir();
	err |= MPSSE_TDI_bit(3, 0x02, 0);	// idcode 0010
	err |= MPSSE_TMS_bit(1, 0x01, 0);
	err |= tap_shift2idle2();
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

UINT32 tap_set_ir(UCHAR instruction){
//	EnterCriticalSection(&CriticalSection_tap);
	UINT32 err = 0;

	err |= tap_idle2shift_ir();
	err |= MPSSE_TDI_bit(3, instruction, 0);	// idcode 0010
	if (instruction & 0x8){
		err |= MPSSE_TMS_bit(1, 0x81, 0);
	}
	else
	{
		err |= MPSSE_TMS_bit(1, 0x01, 0);
	}
	err |= tap_shift2idle2();
	err |= ftdx_flush();
//	LeaveCriticalSection(&CriticalSection_tap);
	return err;
}

#define MAX_DEVICES 32
#define ALLOC_SIZE 64

UINT32 tap_get_ir_ID(UINT32 *idcodes, int *num_devices){
	UINT32 err = 0;
	int devindex = 0;
	UINT32 invalid_code = 0x7f;
	const unsigned int done_code = 0x3f;
	UCHAR start_bit = 0;
//	UINT32 *idcodes;
	int reallocs = 0;
	UINT32 temp_manuf_code = 0;
	UINT32 temp_rest_code = 0;
	UINT32 tempID;

	err |= tap_idle2shift_dr();

	while (devindex < MAX_DEVICES) {
		// get 1 bit. 0 = BYPASS, 1 = start of IDCODE
		err |= MPSSE_read_write_TDI(1,invalid_code & 0x01, &start_bit);
//		printf("start : %d\n", start_bit);
		invalid_code >>= 1;

		if (start_bit == 0) {
			idcodes[devindex] = -1;
			devindex++;
		}
		else {
			// get 11 bit manufacturer code
			err |= jtag_chain_read_write_stream(&invalid_code, &temp_manuf_code, 11, 0, 0);
//			printf("manucode:%08x\n", temp_manuf_code);
			invalid_code >>= 11;

			if (temp_manuf_code != done_code) {
				// get 20 more bits, rest of ID
				err |= jtag_chain_read_write_stream(&invalid_code, &temp_rest_code, 20, 0, 0);
//				printf("rest_code:%08x\n", temp_rest_code);
				invalid_code >>= 20;
				tempID = (temp_rest_code << 12) | (temp_manuf_code << 1) | 0x01;
				idcodes[devindex] = tempID;
				devindex++;
			}
			else {
				break;
			}
		}

		if (err)  // Don't try to keep probing if we get a comm. error
			return err;
	}

    if (devindex >= MAX_DEVICES){
        printf("WARNING: maximum supported devices on JTAG chain (%i) exceeded.\n", MAX_DEVICES);
        printf("Please check JTAG connction!\n");
    }
	err |= tap_shift2idle2();
	
//	*id_array = idcodes;
	*num_devices = devindex;
	INT i = 0;
	for (i = 0; i < devindex; i++){
	    printf("JTAG ID[%d]: \t0x%08X\n", i, idcodes[i]);
	}

	return err;
}

