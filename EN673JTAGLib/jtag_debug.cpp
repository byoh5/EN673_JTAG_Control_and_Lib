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

UINT32 jtag_single_step(UINT8 cpu){
	UINT32 err=0;

	err |= jtag_stall_cpu(cpu);
	err |= jtag_write32(SPR_BIT + DU_GID + (16<<2), (1 << 22) | (1 << 26), cpu);  /* Set step bit and NOP enable */
	return err;
}

UINT32 jtag_continue(UINT8 cpu){
	UINT32 err=0;

	err |= jtag_stall_cpu(cpu);
	err |= jtag_write32(SPR_BIT + DU_GID + (16<<2), 0, cpu);  /* Reset step bit */
		err |= jtag_unstall_cpu(cpu);
	return err;
}

UINT32 jtag_set_trap_pos(UINT32 addr, UINT8 cpu){
	UINT32 err=0;
	UINT32 ins;

	err |= jtag_stall_cpu(cpu);
	err |= jtag_write32(SPR_BIT + DU_GID + (20<<2), 0x2000, cpu);			/* Trap causes stall */
	err |= jtag_wb_write32(addr, 0x21000001, cpu);	// Insert  l.trap 
	err |= jtag_wb_read32(addr, &ins, cpu);
	if (ins == 0x21000001) printf("Trap Success\n");
	else printf("Trap Fail\n");
	err |= jtag_unstall_cpu(cpu);  /* 11x Unstall */
	printf("Trap on addr 0x%.8x cpu%d", addr, cpu - 4);
	return err;
}

UINT32 jtag_read_cpu_register(UINT8 cpu,UINT8* buffer)
{
	UINT32 err = 0;
	UINT32 npc, ppc;

	err |= jtag_stall_cpu(cpu);
	
	err |= jtag_read32(SPR_BIT + SYS_GID + (16<<2), &npc, cpu);  /* Read NPC */
	err |= jtag_read32(SPR_BIT + SYS_GID + (18<<2), &ppc, cpu);  /* Read PPC */
//	printf("\t npc = %.8x ppc = %.8x \n", npc, ppc);
	
	sprintf_s((char*)buffer, 100, "\t npc = %.8x pdf=%1x pdt=%1x ppc = %.8x \n", npc & 0xfffffffc, (npc&0x02)>>1, npc&0x01, ppc & 0xfffffffc);

	return err;
}

#if 1

UINT32 jtag_read_cpu_status(UINT8 cpu, UINT8* buffer)
{	UINT32 err = 0;
	UINT32 npc, ppc;
	UINT32 expc, wbpc;
	UINT32 insn, module;
	UINT32 ex_pd_flag, ex_pd_taken;
	UINT32 ex_nop, ex_flush;
	UINT32 ex_insn;

	UINT32 ex_op, ex_ra, ex_rb, ex_rd_a, ex_rd_b;

	UINT32 ex_branch_op;

	UINT32 ex_fwdat_a, ex_fwdat_b;
	UINT8	ex_fwdat_aS[10] = { 0, 0 };
	UINT8	ex_fwdat_bS[10] = { 0, 0 };

//	err |= jtag_stall_cpu(cpu);

	err |= jtag_read32(SPR_BIT + SYS_GID + (16<<2), &npc, cpu);  /* Read NPC */
	err |= jtag_read32(SPR_BIT + SYS_GID + (18<<2), &ppc, cpu);  /* Read PPC */

	expc		= npc & 0xfffffffc;
	ex_pd_flag  = (npc & 0x00000002) >> 1;
	ex_pd_taken = npc & 0x00000001;

	wbpc		= ppc & 0xfffffffc;
	ex_nop		= (ppc & 0x00000002) >> 1;
	ex_flush	= ppc & 0x00000001;




	if ((expc & 0xff000000) == SDRAM_BASE){
		module = CPU0;
	}
	else
	{
		module = WBCOM;
	}

	err |= jtag_wb_read32(expc, &insn, module);

	if (ex_nop) ex_insn = INOP;
	else        ex_insn = insn;


	ex_op	= (ex_insn & 0xfc000000) >> 26;
	ex_ra	= (ex_insn & 0x001f0000) >> 16;
	ex_rb	= (ex_insn & 0x0000f800) >> 11;
	ex_rd_a = (ex_insn & 0x80000000) >> 31;
	ex_rd_b = (ex_insn & 0x40000000) >> 30;

	switch (ex_op)
	{
		case EXR1K_OP_J:
		case EXR1K_OP_JAL:
		case EXR1K_OP_JR:
		case EXR1K_OP_JALR:
		case EXR1K_OP_BNF:
		case EXR1K_OP_BF:
		case EXR1K_OP_RFE:
			ex_branch_op = 1;
			break;
		default:	
			ex_branch_op = 0;
			break;
	}

	if (ex_rd_a) {
		err |= jtag_read32(DRF_BIT + EXR1K_SYS_RF0 + ex_ra, &ex_fwdat_a, cpu);
		sprintf_s((char*)ex_fwdat_aS, 10, "%.8x", ex_fwdat_a);
	}
	else{
		sprintf_s((char*)ex_fwdat_aS, 10, "%s", "xxxxxxxx");
	}

	if (ex_rd_b) {
		err |= jtag_read32(DRF_BIT + EXR1K_SYS_RF0 + ex_rb, &ex_fwdat_b, cpu);
		sprintf_s((char*)ex_fwdat_bS, 10, "%.8x", ex_fwdat_b);
	}
	else{
		sprintf_s((char*)ex_fwdat_bS, 10, "%s", "xxxxxxxx");
	}

	if (ex_flush){
		sprintf_s((char*)buffer, 100, "@%.8x: %.8x x\n", expc, INOP);
	}
	else{
		sprintf_s((char*)buffer, 100, "@%.8x: %.8x %1x: %s %s\n", expc, ex_insn, ex_branch_op, ex_fwdat_aS, ex_fwdat_bS);

	}

	return err;
}
#endif
UINT32 jtag_read_cpu_all_register(UINT8 cpu)
{
	UINT32 err = 0;
	UINT32 npc, ppc,sr,esr, EPCR,EEAR,DMR1,DSR,DRR ,i;
	UINT32 IWMR, IWSR, IWDTR, IWAR;
	UINT32 DWMR, DWSR, DWDTR, DWAR;
	UINT32 tbcr;
	UINT32 r[32];


	err |= jtag_stall_cpu(cpu);

	err |= jtag_read32(SPR_BIT + SYS_GID + (16<<2), &npc, cpu);  /* Read NPC */
	err |= jtag_read32(SPR_BIT + SYS_GID + (17<<2), &sr , cpu);  /* Read SR  */
	err |= jtag_read32(SPR_BIT + SYS_GID + (18<<2), &ppc, cpu);  /* Read PPC */
	

	err |= jtag_read32(SPR_BIT + SYS_GID + (32 << 2), &EPCR, cpu);  /* Read EPCR */
	err |= jtag_read32(SPR_BIT + SYS_GID + (64 << 2), &esr, cpu);  /* Read ESR */
	err |= jtag_read32(SPR_BIT + SYS_GID + (48 << 2), &EEAR, cpu);  /* Read EEAR */
	

	err |= jtag_read32(SPR_BIT + DU_GID + (16 << 2), &DMR1,cpu);  /* Read DMR1 */
	err |= jtag_read32(SPR_BIT + DU_GID + (20 << 2), &DSR, cpu);  /* Read DSR */
	err |= jtag_read32(SPR_BIT + DU_GID + (21 << 2), &DRR, cpu);  /* Read DRR */
	

	err |= jtag_read32(SPR_BIT + IWB_GID + (16 << 2), &IWMR, cpu);  /* Read IWMR */
	err |= jtag_read32(SPR_BIT + IWB_GID + (17 << 2), &IWSR, cpu);  /* Read IWSR */
	err |= jtag_read32(SPR_BIT + IWB_GID + (18 << 2), &IWDTR, cpu);  /* Read IWDTR */
	err |= jtag_read32(SPR_BIT + IWB_GID + (19 << 2), &IWAR, cpu);  /* Read IWAR */
	

	err |= jtag_read32(SPR_BIT + DWB_GID + (16 << 2), &DWMR, cpu);  /* Read DWMR */
	err |= jtag_read32(SPR_BIT + DWB_GID + (17 << 2), &DWSR, cpu);  /* Read DWSR */
	err |= jtag_read32(SPR_BIT + DWB_GID + (18 << 2), &DWDTR, cpu);  /* Read DWDTR */
	err |= jtag_read32(SPR_BIT + DWB_GID + (19 << 2), &DWAR, cpu);  /* Read DWAR */
	
	err |= jtag_read32(SPR_BIT + TB_GID + TBCR, &tbcr, cpu);  /* Read TBCR */

	printf("Status:     NPC  = %.8x  SR   = %.8x PPC   = %.8x \n", npc & 0xfffffffc, sr, ppc & 0xfffffffc);
	printf("Exception:  EPCR = %.8x  ESR  = %.8x EEAR  = %.8x \n", EPCR & 0xfffffffc, esr, EEAR);
	printf("Debug:      DMR1 = %.8x  DSR  = %.8x DRR   = %.8x \n", DMR1, DSR, DRR);
	printf("IHWP:       IWMR = %.8x  IWSR = %.8x IWDTR = %.8x IWAR = %.8x\n", IWMR, IWSR, IWDTR, IWAR);
	printf("DHWP:       DWMR = %.8x  DWSR = %.8x DWDTR = %.8x DWAR = %.8x\n", DWMR, DWSR, DWDTR, DWAR);
	printf("TB:         TBCR = %.8x\n", tbcr);
	printf("GPR:\n");
	for (i = 0; i < 32; i++){
		err |= jtag_read32(DRF_BIT + (0x400<<2) + (i<<2), &r[i], cpu);
		printf("\t r%.2d = %.8x \n", i, r[i]);
	}
	/*
	for (i = 0; i < 256 * 4 + 1; i++){
		err |= jtag_read32(DTB_BIT + 0x0ff + i, &thbuf[i], cpu);
		printf("\t thbuf%.2d = %.8x \n", i, thbuf[i]);
	}
	*/
//	err |= jtag_unstall_cpu(cpu);
	return err;
}

UINT32 jtag_enable_tb(UINT8 cpu){
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + TB_GID + (7<<10), TBCR_EN_BITS|TBCR_NE_BITS, cpu);
	return err;
}

UINT32 jtag_disable_tb(UINT8 cpu){
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + TB_GID + (7 << 10), 0, cpu);
	return err;
}

#define DUMP 0
#define TB_LEN 256
#define MAX_FRZ_CNT 500
UINT32 jtag_read_tb(UINT8 cpu, UINT8* buffer){
	UINT32 err = 0;
	UINT32 i ,j= 0;
	UINT32 thbuf[TB_LEN * 4 + 1];

	UINT32 tb_ts_prev, tb_adr, tb_pc, tb_insn, tb_rout, tb_ts, tb_pd_flag, tb_pd_taken;
	UINT32 frz_cnt = 0;

	for (i = 0; i < TB_LEN * 4 + 1; i++){
		err |= jtag_read32(DTB_BIT + (0x0ff<<2) + (i<<2), &thbuf[i], cpu);
	//	printf("Adr:%.8x = %.8x \n", DTB_BIT + 0x0ff + i, thbuf[i]);
#if DUMP
		j += sprintf_s((char*)buffer + j, 100*256*4, "@Ad>%.8x: %.8x\n", DTB_BIT + 0x0ff + i, thbuf[i]);
#endif
	}

	
	tb_adr = thbuf[0];
	tb_ts_prev = thbuf[1 + TB_LEN * 3 + tb_adr];
	j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "tb_adr : %8x\n", tb_adr);
    printf("tb_adr : %8x\n", tb_adr);
	for (i = tb_adr; i < TB_LEN; i++){
		tb_pc   = thbuf[1 + TB_LEN * 0 + i];
		tb_insn = thbuf[1 + TB_LEN * 1 + i];
		tb_rout = thbuf[1 + TB_LEN * 2 + i];
		tb_ts   = thbuf[1 + TB_LEN * 3 + i];
		tb_ts_prev = thbuf[1 + TB_LEN * 3 + i - 1];
		tb_pd_flag = (tb_pc & 0x02)>>1;
		tb_pd_taken = tb_pc & 0x01;
		
		j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "| %.8x %1x %1x %.8x %.8x %.8x :", tb_pc & 0xfffffffc, tb_pd_flag, tb_pd_taken, tb_insn, tb_rout, tb_ts);
		frz_cnt = tb_ts - tb_ts_prev;
		j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "%8d\n", frz_cnt);

        printf("| %.8x %1x %1x %.8x %.8x %.8x : %8d\n", tb_pc & 0xfffffffc, tb_pd_flag, tb_pd_taken, tb_insn, tb_rout, tb_ts, frz_cnt);
		if (frz_cnt > MAX_FRZ_CNT) frz_cnt = MAX_FRZ_CNT;
		if (frz_cnt < 0) frz_cnt = 0;
		//for (k = 0; k < frz_cnt; k++) printf(" |\n");
	}

	for (i = 0; i < tb_adr; i++){
		tb_pc   = thbuf[1 + TB_LEN * 0 + i];
		tb_insn = thbuf[1 + TB_LEN * 1 + i];
		tb_rout = thbuf[1 + TB_LEN * 2 + i];
		tb_ts   = thbuf[1 + TB_LEN * 3 + i];
		if (i==0){
			tb_ts_prev = thbuf[1 + TB_LEN * 3 + 0xff];
		}
		else{
			tb_ts_prev = thbuf[1 + TB_LEN * 3 + i - 1];
		}
		tb_pd_flag = (tb_pc & 0x02)>>1;
		tb_pd_taken = tb_pc & 0x01;
		
		j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "| %.8x %1x %1x %.8x %.8x %.8x :", tb_pc & 0xfffffffc, tb_pd_flag, tb_pd_taken, tb_insn, tb_rout, tb_ts);
		frz_cnt = tb_ts - tb_ts_prev;
		
		j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "%8d\n", frz_cnt);

        printf("| %.8x %1x %1x %.8x %.8x %.8x : %8d\n", tb_pc & 0xfffffffc, tb_pd_flag, tb_pd_taken, tb_insn, tb_rout, tb_ts, frz_cnt);
		if (frz_cnt > MAX_FRZ_CNT) frz_cnt = MAX_FRZ_CNT;
		if (frz_cnt < 0) frz_cnt = 0;
		//for (k = 0; k < frz_cnt; k++) printf(" |\n");

	}

	return err;
}

UINT32 jtag_read_tb_new(UINT8 cpu, UINT8* buffer){
	UINT32 err = 0;
	UINT32 i, j = 0;
	UINT32 thbuf[TB_LEN * 4 + 1];

	UINT32 tb_adr, tb_pc, tb_insn, tb_rout, tb_ts;
	UINT32 frz_cnt = 0;
	UINT32 tb_cr;

	err |= jtag_read32(SPR_BIT + TB_GID + (7 << 10), &tb_cr, cpu);
	
	for (i = 0; i < TB_LEN * 4 + 1; i++){
		err |= jtag_read32(DTB_BIT + (0x0ff << 2) + (i << 2), &thbuf[i], cpu);
		//	printf("Adr:%.8x = %.8x \n", DTB_BIT + 0x0ff + i, thbuf[i]);
#if DUMP
		j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "@Ad>%.8x: %.8x\n", DTB_BIT + 0x0ff + i, thbuf[i]);
#endif
	}
	tb_adr = thbuf[0];
	j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "tbcr : %08x\n", tb_cr);
	printf("tbcr : % 08x\n", tb_cr);
	j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "tbar : %02x\n", tb_adr);
	printf("tbar : %02x\n", tb_adr);
	for (i = 0; i < TB_LEN; i++){
		tb_pc	= thbuf[1 + TB_LEN * 0 + i];
		tb_insn = thbuf[1 + TB_LEN * 1 + i];
		tb_rout = thbuf[1 + TB_LEN * 2 + i];
		tb_ts	= thbuf[1 + TB_LEN * 3 + i];
		j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "%02x %.8x %.8x %.8x %.8x \n",i,tb_pc , tb_insn, tb_rout, tb_ts);
		printf("%02x %.8x %.8x %.8x %.8x \n", i, tb_pc, tb_insn, tb_rout, tb_ts);
	}

	return err;
}


UINT32 jtag_set_IWVR(UINT32 value, UINT8 iwbnum, UINT8 cpu)
{
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + IWB_GID + (iwbnum << 2), value, cpu);
	return err;
}
UINT32 jtag_get_IWVR(UINT8 iwbnum, UINT8 cpu)
{
	UINT32 iwb;
	jtag_read32(SPR_BIT + IWB_GID + (iwbnum << 2), &iwb, cpu);
	return iwb;
}
UINT32 jtag_set_DWVR(UINT32 value, UINT8 dwbnum, UINT8 cpu)
{
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + DWB_GID + (dwbnum << 2), value, cpu);
	return err;
}
UINT32 jtag_get_DWVR(UINT8 dwbnum, UINT8 cpu)
{
	UINT32 dwb;
	jtag_read32(SPR_BIT + IWB_GID + (dwbnum << 2), &dwb, cpu);
	return dwb;
}

UINT32 jtag_set_IWCR(UINT8 ct,UINT8 cc, UINT8 iwbnum, UINT8 cpu)
{
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + IWB_GID + ((iwbnum+8)<< 2), (ct<<3)|cc, cpu);
	return err;
}
UINT32 jtag_get_IWCR(UINT8 iwbnum, UINT8 cpu)
{
	UINT32 iwb;
	jtag_read32(SPR_BIT + IWB_GID + ((iwbnum + 8) << 2), &iwb, cpu);
	return iwb;
}

UINT32 jtag_set_DWCR(UINT8 ct, UINT8 cc, UINT8 iwbnum, UINT8 cpu)
{
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + DWB_GID + ((iwbnum + 8) << 2), (ct << 3) | cc, cpu);
	return err;
}
UINT32 jtag_get_DWCR(UINT8 iwbnum, UINT8 cpu)
{
	UINT32 iwb;
	jtag_read32(SPR_BIT + DWB_GID + ((iwbnum + 8) << 2), &iwb, cpu);
	return iwb;
}

UINT32 jtag_set_WMR(UINT32 data, UINT8 cpu)
{
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + DWB_GID + ((16) << 2), data, cpu);
	return err;
}
UINT32 jtag_get_WMR(UINT8 iwbnum, UINT8 cpu)
{
	UINT32 data;
	jtag_read32(SPR_BIT + DWB_GID + ((16) << 2), &data, cpu);
	return data;
}

UINT32 jtag_set_WSR(UINT32 data, UINT8 cpu)
{
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + DWB_GID + ((17) << 2), data, cpu);
	return err;
}
UINT32 jtag_get_WSR(UINT8 iwbnum, UINT8 cpu)
{
	UINT32 data;
	jtag_read32(SPR_BIT + DWB_GID + ((17) << 2), &data, cpu);
	return data;
}
UINT32 jtag_set_WDTR(UINT32 data, UINT8 cpu)
{
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + DWB_GID + ((18) << 2), data, cpu);
	return err;
}
UINT32 jtag_get_WDTR(UINT8 iwbnum, UINT8 cpu)
{
	UINT32 data;
	jtag_read32(SPR_BIT + DWB_GID + ((18) << 2), &data, cpu);
	return data;
}
UINT32 jtag_set_WAR(UINT32 data, UINT8 cpu)
{
	UINT32 err = 0;
	err |= jtag_write32(SPR_BIT + DWB_GID + ((19) << 2), data, cpu);
	return err;
}
UINT32 jtag_get_WAR(UINT8 iwbnum, UINT8 cpu)
{
	UINT32 data;
	jtag_read32(SPR_BIT + DWB_GID + ((19) << 2), &data, cpu);
	return data;
}


// IC Read 

UINT32 jtag_get_ICCFGR(UINT8 cpu)
{
	UINT32 iccfgr;
	jtag_read32(SPR_BIT + SYS_GID + (ICCFGR << 2), &iccfgr, cpu);  
	return iccfgr;
}

UINT32 jtag_get_NCS_on_ICCFGR(UINT cpu)
{
	return ((jtag_get_ICCFGR(cpu)& SPR_ICCFGR_NCS) >> SPR_ICCFGR_NCS_OFF);
}

UINT32 jtag_get_CBS_on_ICCFGR(UINT cpu)
{
	return ((jtag_get_ICCFGR(cpu)& SPR_ICCFGR_CBS) >> SPR_ICCFGR_CBS_OFF);
}


UINT32 jtag_get_TM(UINT cpu)
{
	UINT32 nsets;		
	UINT32 bsize;

	UINT32	bidw=2;		// byte index width        
	UINT32	widw;   	// word index width        
	UINT32	cidw;   	// cache index width       
	UINT32	tidw;   	// tag index width         
	UINT32	caw;		// cache address width     

	UINT32 iccfgr;
	UINT32 ncs;
	UINT32 cbs;

	UINT32 adr;
	UINT32 cadr;

	UINT32 i,j;
	
	UINT32 rtagmem[1024];
	UINT32 ricache;
	UINT32 rvalid;
	UINT32 rtag;


	iccfgr = jtag_get_ICCFGR(cpu);
	ncs = ((iccfgr& SPR_ICCFGR_NCS) >> SPR_ICCFGR_NCS_OFF);
	cbs = ((iccfgr& SPR_ICCFGR_CBS) >> SPR_ICCFGR_CBS_OFF);

	widw = 2 + cbs;
	cidw = ncs;
	caw = cidw + widw + bidw;
	tidw = 32 - caw;

	nsets = (1 << ncs);
	bsize = (16 << cbs);

	for (i = 0; i < nsets; i++){
		adr = 0;
		adr = (i << (widw+bidw));	//cid (10bit)
		adr = adr | (1 << caw);		//Tsel = 1 select tag memory	
		jtag_read32(IC_BIT | adr, &rtagmem[i], cpu);
		rvalid = rtagmem[i] & 0x1;
		rtag = (rtagmem[i] >> 1);

		printf("%04d: v=%1d, tag=%08x: ", i, rvalid, rtag);

		if (rvalid){
			cadr = 0;
			cadr = (i << (widw+bidw));				//cid
			cadr = cadr | (rtag << caw);			//tid
			printf("%08x: ", cadr);
			for (j = 0; j < bsize / 4; j++){
				adr = 0;
				adr = adr | (j << bidw);			//wid
				adr = adr | (i << (widw + bidw));	//cid
				adr = adr | (0 << (caw));			//tsel=0; select data memory
				jtag_read32(IC_BIT | adr, &ricache, cpu);
				printf("%08x ", ricache);
			}
			printf("\n");

		}

	}

	return 0;
}