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




// CPU defines
#define CPU_OP_ADR  0
#define CPU_SEL_ADR 1



UINT32 test_or1k_cpu0(UINT32 cpu)
{
	UINT32 npc, ppc, r1, insn;
	UINT8 stalled,pcs;
	UINT32 result;
	UINT32 i;

	UINT8 buf[128] = { 0, 0 };

	printf("Testing CPU%d (or1k) - writing instructions\n", cpu - 4);
	(jtag_wb_write32(SDRAM_BASE + 0x00, 0xe0000005, cpu));   /* l.xor   r0,r0,r0   */
	(jtag_wb_write32(SDRAM_BASE + 0x04, 0x9c200000, cpu));   /* l.addi  r1,r0,0x0  */
	(jtag_wb_write32(SDRAM_BASE + 0x08, 0x18400000, cpu));   /* l.movhi r2,0x0000  */
	(jtag_wb_write32(SDRAM_BASE + 0x0c, 0xa8420030, cpu));   /* l.ori   r2,r2,0x30 */
	(jtag_wb_write32(SDRAM_BASE + 0x10, 0x9c210001, cpu));   /* l.addi  r1,r1,1    */
	(jtag_wb_write32(SDRAM_BASE + 0x14, 0x9c210001, cpu));   /* l.addi  r1,r1,1    */
	(jtag_wb_write32(SDRAM_BASE + 0x18, 0xd4020800, cpu));   /* l.sw    0(r2),r1   */
	(jtag_wb_write32(SDRAM_BASE + 0x1c, 0x9c210001, cpu));   /* l.addi  r1,r1,1    */
	(jtag_wb_write32(SDRAM_BASE + 0x20, 0x84620000, cpu));   /* l.lwz   r3,0(r2)   */
	(jtag_wb_write32(SDRAM_BASE + 0x24, 0x03fffffb, cpu));   /* l.j     loop2      */
	(jtag_wb_write32(SDRAM_BASE + 0x28, 0xe0211800, cpu));   /* l.add   r1,r1,r3   */

	(jtag_wb_write32(SDRAM_BASE + 0x2c, 0x14000000, cpu));   /* l.nop			   */
	(jtag_wb_write32(SDRAM_BASE + 0x30, 0x14000000, cpu));   /* l.nop			   */
	(jtag_wb_write32(SDRAM_BASE + 0x34, 0x14000000, cpu));   /* l.nop			   */
	(jtag_wb_write32(SDRAM_BASE + 0x38, 0x14000000, cpu));   /* l.nop			   */
	(jtag_wb_write32(SDRAM_BASE + 0x3c, 0x14000000, cpu));   /* l.nop			   */

	printf("Setting up CPU%d\n", cpu - 4);
	(jtag_write32(SPR_BIT+(0 << 11) + 17, 0x01, cpu));  /* Init SR @ any time: disable cache */
	(jtag_write32(SPR_BIT+(0 << 11) + 16, SDRAM_BASE, cpu));  /* Set PC: flushes pipes -> all nop's */
	(jtag_write32(SPR_BIT+(6 << 11) + 20, 0x2000, cpu));  /* Trap causes stall */

	do jtag_check_pcs(cpu, &pcs); while (!(pcs & 0x3));	// check NPC changed b'101   

	(jtag_write32(SPR_BIT + (6 << 11) + 16, (1 << 22), cpu)); /* Set step bit */
//	(jtag_write32((6 << 11) + 16, (1 << 22) | (1 << 26), cpu)); /* Set step bit with NOP enable*/
	printf("Starting CPU%d!\n", cpu - 4);
	for (i = 0; i < 11; i++) {
		jtag_unstall_cpu(cpu);  /* 11x Unstall */
		//printf("Starting CPU, waiting for trap...\n");
		do jtag_check_stalled_cpu(cpu,&stalled); while (!(stalled & 1));

		jtag_read_cpu_status(cpu, buf);
		printf("%s\n",buf);
	}

	(jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, cpu));  /* Read NPC */
	(jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, cpu));  /* Read PPC */
	(jtag_read32(DRF_BIT + 0x401, &r1, cpu));  /* Read R1 */
	printf("Read      npc = %.8x ppc = %.8x r1 = %.8x\n", npc & 0xfffffffc, ppc & 0xfffffffc, r1);
	printf("Expected  npc = %.8x ppc = %.8x r1 = %.8x\n", SDRAM_BASE + 0x00000010, SDRAM_BASE + 0x00000028, 5);
	result = npc + ppc + r1;

	(jtag_write32(SPR_BIT + (6 << 11) + 16, 0, cpu));  // Reset step bit 

	//(jtag_write32((0 << 11) + 17, 0x01, cpu));  /* Init SR @ any time: disable cache */
	(jtag_wb_read32(SDRAM_BASE + 0x28, &insn, cpu));  // Set trap insn in delay slot 
	(jtag_wb_write32(SDRAM_BASE + 0x28, 0x21000001, cpu));
	jtag_unstall_cpu(cpu);  // Unstall 
	do jtag_check_stalled_cpu(cpu, &stalled); while (!(stalled & 1));
	(jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, cpu));  // Read NPC 
	(jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, cpu));  // Read PPC 
	(jtag_read32(DRF_BIT + 0x401, &r1, cpu));  // Read R1 
	(jtag_wb_write32(SDRAM_BASE + 0x28, insn, cpu));  // Set back original insn 
	printf("Read      npc = %.8x ppc = %.8x r1 = %.8x\n", npc & 0xfffffffc, ppc & 0xfffffffc, r1);
	printf("Expected  npc = %.8x ppc = %.8x r1 = %.8x\n", SDRAM_BASE + 0x00000010, SDRAM_BASE + 0x00000028, 8);
	result = npc + ppc + r1 + result;

	(jtag_wb_read32(SDRAM_BASE + 0x24, &insn, cpu));  // Set trap insn in place of branch insn 
	(jtag_wb_write32(SDRAM_BASE + 0x24, 0x21000001, cpu));
	(jtag_write32(SPR_BIT + (0 << 11) + 16, SDRAM_BASE + 0x10, cpu));  // Set PC 
	jtag_unstall_cpu(cpu);  // Unstall 
	do jtag_check_stalled_cpu(cpu, &stalled); while (!(stalled & 1));
	(jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, cpu));  // Read NPC 
	(jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, cpu));  // Read PPC 
	(jtag_read32(DRF_BIT + 0x401, &r1, cpu));  // Read R1 
	(jtag_wb_write32(SDRAM_BASE + 0x24, insn, cpu));  // Set back original insn 
	printf("Read      npc = %.8x ppc = %.8x r1 = %.8x\n", npc & 0xfffffffc, ppc & 0xfffffffc, r1);
	printf("Expected  npc = %.8x ppc = %.8x r1 = %.8x\n", SDRAM_BASE + 0x00000028, SDRAM_BASE + 0x00000024, 11);
	result = npc + ppc + r1 + result;

	(jtag_wb_read32(SDRAM_BASE + 0x20, &insn, cpu));  /* Set trap insn before branch insn */
	(jtag_wb_write32(SDRAM_BASE + 0x20, 0x21000001, cpu));
	(jtag_write32(SPR_BIT + (0 << 11) + 16, SDRAM_BASE + 0x24, cpu));  /* Set PC */
	jtag_unstall_cpu(cpu);  /* Unstall */
	do jtag_check_stalled_cpu(cpu, &stalled); while (!(stalled & 1));
	(jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, cpu));  /* Read NPC */
	(jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, cpu));  /* Read PPC */
	(jtag_read32(DRF_BIT + 0x401, &r1, cpu));  /* Read R1 */
	(jtag_wb_write32(SDRAM_BASE + 0x20, insn, cpu));  /* Set back original insn */
	printf("Read      npc = %.8x ppc = %.8x r1 = %.8x\n", npc & 0xfffffffc, ppc & 0xfffffffc, r1);
	printf("Expected  npc = %.8x ppc = %.8x r1 = %.8x\n", SDRAM_BASE + 0x00000024, SDRAM_BASE + 0x00000020, 24);
	result = npc + ppc + r1 + result;

	(jtag_wb_read32(SDRAM_BASE + 0x1c, &insn, cpu));  /* Set trap insn behind lsu insn */
	(jtag_wb_write32(SDRAM_BASE + 0x1c, 0x21000001, cpu));
	(jtag_write32(SPR_BIT + (0 << 11) + 16, SDRAM_BASE + 0x20, cpu));  /* Set PC */
	jtag_unstall_cpu(cpu);  /* Unstall */
	do jtag_check_stalled_cpu(cpu, &stalled); while (!(stalled & 1));
	(jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, cpu));  /* Read NPC */
	(jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, cpu));  /* Read PPC */
	(jtag_read32(DRF_BIT + 0x401, &r1, cpu));  /* Read R1 */
	(jtag_wb_write32(SDRAM_BASE + 0x1c, insn, cpu));  /* Set back original insn */
	printf("Read      npc = %.8x ppc = %.8x r1 = %.8x\n", npc & 0xfffffffc, ppc & 0xfffffffc, r1);
	printf("Expected  npc = %.8x ppc = %.8x r1 = %.8x\n", SDRAM_BASE + 0x00000020, SDRAM_BASE + 0x0000001c, 49);
	result = npc + ppc + r1 + result;

	(jtag_wb_read32(SDRAM_BASE + 0x20, &insn, cpu));  /* Set trap insn very near previous one */
	(jtag_wb_write32(SDRAM_BASE + 0x20, 0x21000001, cpu));
	(jtag_write32(SPR_BIT + (0 << 11) + 16, SDRAM_BASE + 0x1c, cpu));  /* Set PC */
	jtag_unstall_cpu(cpu);  /* Unstall */
	do jtag_check_stalled_cpu(cpu, &stalled); while (!(stalled & 1));
	(jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, cpu));  /* Read NPC */
	(jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, cpu));  /* Read PPC */
	(jtag_read32(DRF_BIT + 0x401, &r1, cpu));  /* Read R1 */
	(jtag_wb_write32(SDRAM_BASE + 0x20, insn, cpu));  /* Set back original insn */
	printf("Read      npc = %.8x ppc = %.8x r1 = %.8x\n", npc & 0xfffffffc, ppc & 0xfffffffc, r1);
	printf("Expected  npc = %.8x ppc = %.8x r1 = %.8x\n", SDRAM_BASE + 0x00000024, SDRAM_BASE + 0x00000020, 50);
	result = npc + ppc + r1 + result;

	(jtag_wb_read32(SDRAM_BASE + 0x10, &insn, cpu));  /* Set trap insn to the start */
	(jtag_wb_write32(SDRAM_BASE + 0x10, 0x21000001, cpu));
	(jtag_write32(SPR_BIT + (0 << 11) + 16, SDRAM_BASE + 0x20, cpu)  /* Set PC */);
	jtag_unstall_cpu(cpu);  /* Unstall */
	do jtag_check_stalled_cpu(cpu, &stalled); while (!(stalled & 1));
	(jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, cpu));  /* Read NPC */
	(jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, cpu));  /* Read PPC */
	(jtag_read32(DRF_BIT + 0x401, &r1, cpu));  /* Read R1 */
	(jtag_wb_write32(SDRAM_BASE + 0x10, insn, cpu));  /* Set back original insn */
	printf("Read      npc = %.8x ppc = %.8x r1 = %.8x\n", npc & 0xfffffffc, ppc & 0xfffffffc, r1);
	printf("Expected  npc = %.8x ppc = %.8x r1 = %.8x\n", SDRAM_BASE + 0x00000014, SDRAM_BASE + 0x00000010, 99);
	result = npc + ppc + r1 + result;

	(jtag_write32(SPR_BIT + (6 << 11) + 16, 1 << 22, cpu));  /* Set step bit */
	for (i = 0; i < 5; i++) {
		jtag_unstall_cpu(cpu);  /* Unstall */
		//printf("Waiting for trap...");
		do jtag_check_stalled_cpu(cpu, &stalled); while (!(stalled & 1));
		//printf("got trap.\n");
	}
	(jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, cpu));  /* Read NPC */
	(jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, cpu));  /* Read PPC */
	(jtag_read32(DRF_BIT + 0x401, &r1, cpu));  /* Read R1 */
	printf("Read      npc = %.8x ppc = %.8x r1 = %.8x\n", npc & 0xfffffffc, ppc & 0xfffffffc, r1);
	printf("Expected  npc = %.8x ppc = %.8x r1 = %.8x\n", SDRAM_BASE + 0x00000028, SDRAM_BASE + 0x00000024, 101);
	result = npc + ppc + r1 + result;

	(jtag_write32(SPR_BIT + (0 << 11) + 16, SDRAM_BASE + 0x24, cpu));  /* Set PC */
	for (i = 0; i < 2; i++) {
		jtag_unstall_cpu(cpu);  /* Unstall */
		//printf("Waiting for trap...\n");
		do jtag_check_stalled_cpu(cpu, &stalled); while (!(stalled & 1));
		//printf("Got trap.\n");
	}
	(jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, cpu));  /* Read NPC */
	(jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, cpu));  /* Read PPC */
	(jtag_read32(DRF_BIT + 0x401, &r1, cpu));  /* Read R1 */
	printf("Read      npc = %.8x ppc = %.8x r1 = %.8x\n", npc & 0xfffffffc, ppc & 0xfffffffc, r1);
	printf("Expected  npc = %.8x ppc = %.8x r1 = %.8x\n", SDRAM_BASE + 0x00000010, SDRAM_BASE + 0x00000028, 201);
	result = npc + ppc + r1 + result;
	printf("result = %.8x\n", result ^ 0xdeaddae1);

	if ((result ^ 0xdeaddae1) != 0x96adde80)
		return 1;

	return 0;
}



UINT32 test_cpu(UINT8 cpu){
	UINT32 err=0;
	UINT32 i = 0;
//	err |= tap_reset();
//	err |= tap_set_ir_debug();
	err |= jtag_write32(0xf840000c, 0xd7, WBCOM);	// CPU0, CPU1, CPU2 on ***important!!!! 	
	err |= jtag_stall_cpu(cpu);
	if (test_or1k_cpu0(cpu)){
		printf("Test cpu%d fail\n", i - 4);
		err |= 1;
	}
	else printf("Test cpu%d Success!\n", i - 4);
	
	return err;
}

UINT32 test_memory(UINT32 *writedata, UINT32 *readdata, UINT32 adr, UINT32 len, UINT32 module){
	UINT32 err=0;
	printf("Test memory! \n");
	err |= jtag_write_block32(adr, writedata, len, module);
	err |= jtag_read_block32(adr, readdata, len, module);
	return err;
}



UINT32 test_mem(void){
	UINT32 err=0;
	UINT32 data[1024];
	UINT32 readdata[1024];
	UINT32 j, i, len = 1024;
	for (i = 0; i<1024; i++) {
		data[i] = i;
		printf("data_out = %0x\n", data[i]);
	}
//	err |= tap_reset();
//	err |= tap_set_ir_debug();

	for (i = 4; i < 7; i++){
		printf("Test memory on cpu%d ", i - 4);
		err |= jtag_stall_cpu(i);
		err |= test_memory(data, readdata, 0x00000000, len, i - 4);

		for (j = 0; j < len; j++){
			if (data[j] != readdata[j]) printf("data mismatch (%d) value %d\n", len, j);
			else
			{
				printf("data match %d \n ", j);
			}

		}

	}

	err |= test_memory(data, readdata, SDRAM_BASE, len, WBCOM);	//ON  DDR

	for (j = 0; j < len; j++){
		if (data[j] != readdata[j]) printf("data mismatch (%d) value %d\n", len, j);
		else
		{
			printf("data match %d \n ", j);
		}

	}


	printf("Finish mem test \n ");
	return err;
}


UINT32 test_sdram_2(UINT32 adr, UINT32 data, UINT32 module){
	UINT32 err=0;
	UINT32 insn;

//	printf("SDRAM test 2: \n");
	err |= jtag_write32(adr, data, module);
	err |= jtag_read32(adr, &insn, module);
		printf("Addr %08x expected %08x, read %08x E(%d)\n",adr, data, insn, err);
	if (insn != data) return 1;

	return err;
}

/*
Register read write example
USE wishbone3
Uart control
*/
UINT32 example_uart_control(void){
	UINT32 err=0;
	UINT32 data = 0, i = 0;
//	err |= tap_reset();
//	err |= tap_set_ir_debug();

	err |= jtag_stall_cpu(CPU0);
	
	for (i = 48; i < 122; i++){
		data = i << 8;
        err |= jtag_write32(0xf8000004, data, WBCOM);
	}	
	err |= jtag_unstall_cpu(CPU0);
	
	return err;
}

UINT32 example_uart_print(UINT32 cpu,UINT8 *input,UINT32 len){
	UINT32 err = 0;
	UINT32 data,i = 0;
//	err |= tap_reset();
//	err |= tap_set_ir_debug();
	err |= jtag_stall_cpu(cpu);

	for (i =0; i < len; i++){
		data = (UINT32)(*(input + i) << 8);
		err |= jtag_write32(0xf8000004, data, WBCOM);
	}
	
	err |= jtag_unstall_cpu(cpu);

	return err;
}


UINT32 example_wbcom_gpio(void){
	UINT32 err=0;
	UINT32 data = 0, i = 0;
//	err |= tap_reset();
//	err |= tap_set_ir_debug();

	err |= jtag_stall_cpu(CPU0);
	//	jtag_stall_cpu(CPU1);
	//	jtag_stall_cpu(CPU2);

	for (i = 0; i < 10; i++){
		err |= jtag_write32(0xf6000004, 0x00003f00, WB0);
		Sleep(1);
		err |= jtag_write32(0xf6000004, 0x00000000, WB0);
		Sleep(1);
	}

	//	jtag_unstall_cpu(CPU0);
	//	jtag_unstall_cpu(CPU1);
	//	jtag_unstall_cpu(CPU2);
	printf("GPIO on/off finish!\n");
	return err;
}

UINT32 example_read_cpu_register(void)
{
	UINT32 err=0;
	UINT32 npc, ppc, i;
	UINT32 r[32];

//	err |= tap_reset();
//	err |= tap_set_ir_debug();

	err |= jtag_stall_cpu(CPU0);


	err |= jtag_read32(SPR_BIT + (0 << 11) + 16, &npc, CPU0);  /* Read NPC */
	err |= jtag_read32(SPR_BIT + (0 << 11) + 18, &ppc, CPU0);  /* Read PPC */
	printf("\t npc = %.8x ppc = %.8x \n", npc, ppc);
	for (i = 0; i < 32; i++){
		err |= jtag_read32(DRF_BIT + 0x400 + i, &r[i], CPU0);
		//		printf("\t r%.2d = %.8x \n", i,r[i]);
	}
	return err;
}


UINT32 example_write_cpu_register(UINT32 addr,UINT32 cpu)
{
	UINT32 err = 0;
	UINT8 pcs = 0;
    UINT32 retry_cnt = 0;
//	err |= tap_reset();
//	err |= tap_set_ir_debug();
	err |= jtag_stall_cpu(cpu);
	err |= jtag_write32(SPR_BIT + (16<<2), addr, cpu);  /* Write NPC */
    do{ 
        err |= jtag_check_pcs(cpu, &pcs); 
        retry_cnt++;
        if (retry_cnt > 100) return err;

    }while (!(pcs & 0x3));	// check NPC changed b'11 

	return err;
}


UINT32 example_read_register(UINT32 module, UINT32 addr)
{
	UINT32 err = 0;
	UINT32 reg = 0;

	//	err |= tap_reset();
	//	err |= tap_set_ir_debug();
	//	if (module > 3)	err |= jtag_stall_cpu(module);

	err |= jtag_read32(addr, &reg, module);  /* Read reg */
	//	printf(" reg = %.8x \n", reg);
	//	if (err != 0)
	//		return err;

	return reg;
}


UINT32 example_read_register(UINT32 module, UINT32 addr, UINT32 *data)
{
	UINT32 err = 0;
	UINT32 reg=0;

//	err |= tap_reset();
//	err |= tap_set_ir_debug();
//	if (module > 3)	err |= jtag_stall_cpu(module);

	err |= jtag_read32(addr, data, module);  /* Read reg */
//	printf(" reg = %.8x \n", reg);
//	if (err != 0)
//		return err;

	return err;
}

UINT32 test(void){

	printf("TEST\n");
//	tap_reset();
//	tap_set_ir_debug();

	while (1){
		jtag_stall_cpu(CPU0);
		example_read_cpu_register();
		jtag_unstall_cpu(CPU0);
		Sleep(1000);
	}
	return 0;
}




