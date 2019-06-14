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

//******************************************************************************
// Global variables
//------------------------------------------------------------------------------
int gJtag_ndevices;						// # of JTAG devices on chain
int gTargetID;							// current target ID
int gIrPrefixBits;
int gIrPostfixBits;
int	gDrPrefixBits;
int gDrPostfixBits;


//------------------------------------------------------------------------------
// Get IR size: Reserved!!
int jtag_get_ir_size(int target_id)
{
	return JTAP_IR_W;													
}

// Set Chain ID
// 	Name	| JCMD_TAP 	| JTOP_CID_SET	|CH_ID			|   
//			---------------------------------------------   
//	Bits	| JCMD_W(1)	| JTOP_W(1)		|JCID_W (5)		|   
UINT32 jtag_set_chain_id(int chain_id)
{
    UINT32 err = 0;
	UINT32 wstream = (JCMD_TAP << (JTOP_W + JCID_W)) | (JTOP_CID_SET << JCID_W) | chain_id;

	err |= tap_idle2shift_dr();
    err |= jtag_chain_write_stream(&wstream, JCMD_W + JTOP_W + JCID_W, 1);	// write data, ds_exit1
    err |= tap_shift2idle2();
//	printf("Set Chain ID: %0x\r\n", chain_id);
    return err;
}


UINT32 jtag_config_chain(void)
{
    UINT32 err = 0;
//	printf("Configure Chain: \r\n");

	// save the cur. target
	int	cur_target = gTargetID;
//	printf("cur_target : %d\n", cur_target);
	// Set Chain ID @prefix														// $TST-ygkim-150720: 
	int tid;
	for (tid = cur_target + 1; tid<gJtag_ndevices; tid++) {
		jtag_set_chain_target(tid, gJtag_ndevices);
//		printf("tid : %d\n", tid);
        err |= jtag_set_tap_ir(JTAP_IR_BPDEBUG, JTAP_IR_BYPASS);
        err |= jtag_set_chain_id(tid - cur_target);	// increased from the target ID
//		printf("chain_id : %d\n", tid - cur_target);
	}

	// restore the cur. target
    jtag_set_chain_target(cur_target, gJtag_ndevices);

    return err;
}



// Set IR/DR prefix/postfix bits
//	ID	: 	{	n-1, n-2, ... tid, ..., 1, 0 	}
// 	pre	: 		<--------->	 
//	post:							<--------->
void jtag_set_chain_target(int target_id, int Jtag_ndevices)
{
	gIrPrefixBits = 0;
	gIrPostfixBits = 0;
	if (Jtag_ndevices) gJtag_ndevices = Jtag_ndevices;
	int i;
	for (i = 0; i<target_id; i++)					gIrPostfixBits += jtag_get_ir_size(i);
	for (i = target_id + 1; i<gJtag_ndevices; i++) 	gIrPrefixBits += jtag_get_ir_size(i);

	gDrPrefixBits = gJtag_ndevices - target_id - 1;
	gDrPostfixBits = target_id;
	gTargetID = target_id;

//	printf("Set Chain Target: \r\n");
//	printf("TargetID: %d \n", gTargetID);
//	printf("IrPostfixBits: %d \n", gIrPostfixBits);
//	printf("IrPrefixBits: %d \n", gIrPrefixBits);
//	printf("DrPostfixBits: %d \n", gDrPostfixBits);
//	printf("DrPrefixBits: %d \n\n", gDrPrefixBits);
}




//******************************************************************************
// TAP
//------------------------------------------------------------------------------
// set IR
// 	|	pre			| target (TID)	| post		|
// 	---------------------------------------------
//	| ir_etc		| ir_val		| ir_etc	|
UINT32 jtag_set_tap_ir(UINT32 ir_val, UINT32 ir_etc)  // jtag_set_tap_ir(JTAP_IR_IDCODE,JTAP_IR_BYPASS);
{
    UINT32 err=0;

	int	chain_size = gIrPrefixBits + JTAP_IR_W + gIrPostfixBits;

 //   UINT32 wstream = 0;
    UCHAR wstream[32];
	int ir_postfix_bits = 0;
    int i;

    for (i = 0; i < 32; i++) wstream[i] = 0;
//	printf("Set Tap IR: \r\n");
	
	for (i = 0; i<gTargetID; i++) {
        wstream[i / 2] |= (i % 2) ? (ir_etc << JTAP_IR_W) : ir_etc;
    //    	wstream |= (ir_etc << (jtag_get_ir_size(i)*i));
        ir_postfix_bits += JTAP_IR_W;
	//	printf("%d: %02x, %d\r\n",i,wstream[i/2], ir_postfix_bits);
	}
	for (i = gTargetID + 1; i<gJtag_ndevices; i++) {
        wstream[i / 2] |= (i % 2) ? (ir_etc << JTAP_IR_W) : ir_etc;
    //    wstream |= (ir_etc << (jtag_get_ir_size(i)*i));
	//	printf("%d: %02x\r\n",i,wstream[i/2]);
	}
   
    wstream[gTargetID / 2] |= (gTargetID % 2) ? (ir_val << JTAP_IR_W) : ir_val;
 //   wstream |= (ir_val << ir_postfix_bits);
//	printf("IR = 0x%x, data = 0x%x, chain size = %d\r\n", ir_val, wstream, chain_size);

 //   for (i = 31; i>=0; i--) printf("%02x|", wstream[i]);
 //   printf("\n");

    err |= tap_idle2shift_ir();

//	printf("Chain size %d, IR size %d, pre size %d, post size %d, data 0x%x\r\n", chain_size, JTAP_IR_W, gIrPrefixBits, gIrPostfixBits, wstream);
    err |= MPSSE_write_stream((UINT32*)wstream, chain_size, 1);		 			// write data, is_exit1
    err |= tap_shift2idle2();

    return err;
}


// chain write a stream
UINT32 jtag_chain_write_stream(UINT32* wstream, UINT32 lbits, UCHAR last_tms)
{
    UINT32 err = 0;
	UINT32 zero = 0;

	// Postfix: possible to skip
	//uint wstream = 0;
	//if(DrPostfixBits>0) 	jtag_write_stream(&zero, DrPostfixBits, 0);

	// Data
	UCHAR data_tms = last_tms && (!gDrPrefixBits);		// @data_tms=1: a packet w/o prefix
	err |= MPSSE_write_stream(wstream, lbits, data_tms);	// @data_tms=0: part of a packet / a packet w prefix

	// Prefix
    if (last_tms && gDrPrefixBits>0)	err |= MPSSE_write_stream(&zero, gDrPrefixBits, last_tms);

    return err;
}

int jtag_get_Drprefixbits(void)
{
	return gDrPrefixBits;
}


//------------------------------------------------------------------------------
// chain read/write a stream 
UINT32 jtag_chain_read_write_stream(UINT32* wstream, UINT32* rstream, UINT32 lbits, UCHAR last_tms, UCHAR adjust)
{
    UINT32 err = 0;
    UINT32 zero = 0;

	// Postfix: possible to skip
	if (adjust && gDrPostfixBits>0) 	MPSSE_write_stream(&zero, gDrPostfixBits, 0);

	// Data
	UCHAR data_tms = last_tms & !gDrPrefixBits;					// data_tms=1: a packet w/o prefix
    err |= MPSSE_read_write_stream(wstream, rstream, lbits);	// data_tms=0: part of a packet / a packet w prefix

	// Prefix
    if (last_tms && gDrPrefixBits>0)	err |= MPSSE_write_stream(&zero, gDrPrefixBits, last_tms);

    return err;
}
