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
// JTAG_{MODULE=CPU/WB/JSP} registers
//------------------------------------------------------------------------------
// write module register
// 	Name	| JCMD_SUB	| JBOP_IR_WR 	| rid 					| wdata 				|          
// 			-----------------------------------------------------------------------------          
//	Bits	| JCMD_W(1)	| JBOP_W(5)		| rid_len (J{XX}_IR_AW) | len (J{XX}_{Y}R_DW)	|          
//	e.g.									JIR_AW = 2				JCPU_SR_DW = 2                         
UINT32 jtag_write_module_reg(UINT32 rid, int rid_len, UINT32 wdata, int len)
{
    UINT32 err = 0;
	UINT32 wstream = (JBOP_IR_WR << (rid_len+len)) | (rid << len) | wdata;
	int wlen = JCMD_W+JBOP_W+rid_len+len;
	if (wlen > 32){ printf("STREAM SIZE OVER \n"); }
	err |= tap_idle2shift_dr();
	err |= jtag_chain_write_stream(&wstream, wlen, 1);  	// write data, ds_exit1
    err |= tap_shift2idle2();
    return err;
}

//------------------------------------------------------------------------------
// read module internal register
// : We assume the register is already selected
// : At least 6 more zeros are written to make command=0 and operation=00000b (nop). 
// : But, reading more has no effect on rdata because they are MSBs and 0's.
UINT32 jtag_read_module_reg(UINT32* rdata, int len)
{
    UINT32 err = 0;
	UINT32 wdata[2] = { 0, 0 };
//	ASSERT(len<=64-JCMD_W-JBOP_W);

	err |= tap_idle2shift_dr();

	err |= jtag_chain_read_write_stream(wdata, rdata, len+JCMD_W+JBOP_W,1,1); // write 0 & read data, ds_exit1	
	// $TST-ygkim-150710: 5->6 due to op: 5bits
    err |= tap_shift2idle2();
    return err;
}

//------------------------------------------------------------------------------
// select module internal register
// 	Name	| JCMD_SUB 	| JBOP_IR_SEL 	| rid					|
//			+-----------+---------------+-----------------------+
//	Bits	| JCMD_W	| JBOP_W(5)		| rid_len (J{XX}_IR_AW	|
// 	e.g.									JIR_AW = 2           
UINT32 jtag_sel_module_reg(UINT32 rid, int rid_len)
{   
    UINT32 err = 0;
	UINT32 wstream = (JBOP_IR_SEL << rid_len) | rid;
	int wlen = JCMD_W+JBOP_W+rid_len;

	err |= tap_idle2shift_dr();

    err |= jtag_chain_write_stream(&wstream, wlen, 1); 	// write data, ds_exit1

    err |= tap_shift2idle2();
    
    return err;
}



UINT32 jtag_select_module(UINT8 module_id){
    UINT32 err = 0;
	UINT32 wstream = (JCMD_TAP << (JTOP_W + JMID_W)) | (JTOP_MID_SET << JMID_W) | module_id;

	err |= tap_idle2shift_dr();

    err |= jtag_chain_write_stream(&wstream, JCMD_W + JTOP_W + JMID_W, 1);	// write data, ds_exit1

    err |= tap_shift2idle2();

//	printf("Select module: ID = %x\r\n", module_id);

	return err;
}

UINT32 jtag_stall_cpu(UINT8 cpu){
    UINT32 err = 0;
	err |= jtag_select_module(cpu);
    err |= jtag_write_module_reg(JIR_SR, JIR_AW, JCPU_SR_STALL, JCPU_SR_DW);
	err |= ftdx_flush();
	return err;
}

UINT32 jtag_unstall_cpu(UINT8 cpu){
    UINT32 err = 0;
	err |= jtag_select_module(cpu);
    err |= jtag_write_module_reg(JIR_SR, JIR_AW, JCPU_SR_UNSTALL, JCPU_SR_DW);
	err |= ftdx_flush();
	return err;
}

UINT32 jtag_reset_cpu(UINT8 cpu){
    UINT32 err = 0;
	err |= jtag_select_module(cpu);
    err |= jtag_write_module_reg(JIR_RR, JIR_AW, JCPU_RR_RESET, JCPU_RR_DW);
	err |= ftdx_flush();
	return err;
}

UINT32 jtag_unreset_cpu(UINT8 cpu){
    UINT32 err = 0;
	err |= jtag_select_module(cpu);
	err |= jtag_write_module_reg(JIR_RR, JIR_AW, JCPU_RR_UNRESET, JCPU_RR_DW);
	err |= ftdx_flush();
	return err;
}


UINT32 jtag_check_stalled_cpu(UINT8 cpu, UINT8* stalled){

    UINT32 err = 0;
	UINT32 reg_state;

	err |= jtag_select_module(cpu);

    err |= jtag_sel_module_reg(JIR_SR, JIR_AW);
    err |= jtag_read_module_reg(&reg_state, JCPU_SR_DW);
//	printf("Test stall = %02x\r\n", reg_state);
	err |= ftdx_flush();
	*stalled = reg_state;
	return err;

}

UINT32 jtag_check_pcs(UINT8 cpu, UINT8* pcs){
    UINT32 err = 0;
	UINT32 reg_state;

	err |= jtag_select_module(cpu);

    err |= jtag_sel_module_reg(JIR_SR, JIR_AW);
    err |= jtag_read_module_reg(&reg_state, JCPU_SR_DW);
	err |= ftdx_flush();
//	printf("Test PC stall = %02x\r\n", reg_state);
	*pcs = (reg_state >> 1) & 0x1;
	return err;

}

//------------------------------------------------------------------------------
// Wait for stall CPU: We assume the register is already selected
UINT32 jtag_wait_for_stall(void)
{
    UINT32 err = 0;
	UINT32 reg_state = 0;
	int j = 0;
	while (!reg_state) {
		printf("Test stall = %d\r\n", reg_state);
		err |= jtag_read_module_reg(&reg_state, JCPU_SR_DW);
		reg_state &= 0x1;
//		cpu_stall[ModuleId] = reg_state;
		if (j++>100) { printf("100 tries for waiting for stall"); break; }
	}
    return err;
}

//------------------------------------------------------------------------------
// Wait for PC stall: 															
UINT32 jtag_wait_for_pcstall(void)
{
    UINT32 err = 0;
	UINT32 reg_state = 0;
	int j = 0;
	while (!reg_state) {
		printf("Test PC stall = %d\r\n", reg_state);
		err |= jtag_read_module_reg(&reg_state, JCPU_SR_DW);
		reg_state = (reg_state >> 1) & 0x1;
		if (j++>100) { printf("100 tries for waiting PC stalled"); break; }
	}
    return err;
}
