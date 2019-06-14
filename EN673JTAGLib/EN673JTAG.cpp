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

UINT32 gISP_BASE = 0xf4800000;

UINT32 *idcodes = NULL;
int num_devices = 0;

CRITICAL_SECTION	CriticalSection;
CRITICAL_SECTION	CriticalSection_tap;

void EN673_JTAG_SET_ISP_BASE(UINT32 base)
{
	gISP_BASE = base;
}


UINT32 EN673_JTAG_INIT(UINT32 clkdiv, char channel){
	UINT32 err = 0;
	err |= MPSSE_init(clkdiv, channel);
	err |= tap_reset();
	err |= tap_set_ir_debug();
	// Initialize the critical section one time only.
	if (!InitializeCriticalSectionAndSpinCount(&CriticalSection,0x00000400)) return err;
	if (!InitializeCriticalSectionAndSpinCount(&CriticalSection_tap, 0x00000400)) return err;
	return err;
}

UINT32 EN673_JTAG_CLOSE(void){
	UINT32 err = 0;
	err |= MPSSE_close();
	// Release resources used by the critical section object.
	DeleteCriticalSection(&CriticalSection);
	DeleteCriticalSection(&CriticalSection_tap);
	return err;
}

#define IP_OF_START 0x600
#define IP_OF_END   0x6ff

UINT32 EN673_REG_READ(UINT32 adr, UINT32 *data){

	UINT32 err = 0;
	UINT32 module = WB0;
	if (gISP_BASE <= adr && adr <= (gISP_BASE + (0x32ff * 4))){	// Check ISP address!
		if ((gISP_BASE + (IP_OF_START * 4)) <= adr && adr <= (gISP_BASE + (IP_OF_END * 4))){
			module = WB1;
			//			printf("WB1 read!");
		}
		else{
			module = WB0;
			//			printf("WB0 read!");
		}
	}
	else{
		return 1;
	}
	err |= jtag_read32(adr, data, module);

	return err;
}

UINT32 EN673_REG_BURST_READ(UINT32 adr, UINT32 *data, UINT32 len){

	UINT32 err = 0;
	UINT32 module = WB0;
	if (gISP_BASE <= adr && adr <= (gISP_BASE +(0x3281 * 4))){	// Check ISP address!
		if ((gISP_BASE + (IP_OF_START * 4)) <= adr && adr <= (gISP_BASE + (IP_OF_END * 4))){
			module = WB1;
//			printf("WB1 read!");
		}
		else{
			module = WB0;
//			printf("WB0 read!");
		}	
	}else{
		return 1;
	}
	err |= jtag_read_block32(adr, data, len, module);

	return err;
}

UINT32 EN673_REG_WRITE(UINT32 adr, UINT32 data){
	UINT32 err = 0;
	UINT32 module = WB0;
	if (gISP_BASE <= adr && adr <= (gISP_BASE + (0x3281 * 4))){	// Check ISP address!
		if ((gISP_BASE + (IP_OF_START * 4)) <= adr && adr <= (gISP_BASE + (IP_OF_END * 4))){
			module = WB1;
//			printf("WB1 write!");
		}
		else{
			module = WB0;
//			printf("WB0 write!");
		}
	}
	else{
		return 1;
	}
	err |= jtag_write32(adr, data, module);
	return err;
}

UINT32 EN673_REG_BURST_WRITE(UINT32 adr, UINT32* data, UINT32 len){
	UINT32 err = 0;
	UINT32 module = WB0;
	if (gISP_BASE <= adr && adr <= (gISP_BASE + (0x3281 * 4))){	// Check ISP address!
		if ((gISP_BASE + (IP_OF_START * 4)) <= adr && adr <= (gISP_BASE + (IP_OF_END * 4))){
			module = WB1;
			//			printf("WB1 write!");
		}
		else{
			module = WB0;
			//			printf("WB0 write!");
		}
	}
	else{
		return 1;
	}
	err |= jtag_write_block32(adr, data, len, module);
	return err;
}



SHORT EN673_JTAG_ESC_key_checker(void){
	return GetAsyncKeyState(VK_ESCAPE);
}
