/*++

Copyright (c) 2015-2025 EYENIX Co. Ltd.

Module Name:

JTAG_DEBUG.h

Abstract:

Environment:

Revision History:

15/05/18    oby     Created.

--*/

#ifndef JTAG_DEBUG_H
#define JTAG_DEBUG_H

#include <windows.h>



#define INOP 0x14000000

#define EXR1K_OP_J     				 0
#define EXR1K_OP_JAL   				1
#define EXR1K_OP_BNF   				3
#define EXR1K_OP_BF    				4
#define EXR1K_OP_NOP   				5
#define EXR1K_OP_MOVHI 				6
#define EXR1K_OP_MACRC 				6
#define EXR1K_OP_XSYNC 				8
#define EXR1K_OP_RFE   				9
// RF: read B = INSN[30], except MACI reading A
#define EXR1K_OP_JR       			17
#define EXR1K_OP_JALR     			18
#define EXR1K_OP_MACI     			19
// RF: read A = INSN[31]    	
#define EXR1K_OP_LWZ      			33
#define EXR1K_OP_LWS      			34
#define EXR1K_OP_LBZ      			35
#define EXR1K_OP_LBS      			36
#define EXR1K_OP_LHZ      			37
#define EXR1K_OP_LHS      			38
#define EXR1K_OP_ADDI     			39
#define EXR1K_OP_ADDIC    			40
#define EXR1K_OP_ANDI     			41
#define EXR1K_OP_ORI      			42
#define EXR1K_OP_XORI     			43
#define EXR1K_OP_MULI     			44
#define EXR1K_OP_MFSPR    			45
#define EXR1K_OP_SHROTI 		  	46
#define EXR1K_OP_SFXXI    			47
// RF: read A & B           	
#define EXR1K_OP_MTSPR    			48
#define EXR1K_OP_MACMSB   			49
#define EXR1K_OP_FLOAT    			50
#define EXR1K_OP_SW       			53
#define EXR1K_OP_SB       			54
#define EXR1K_OP_SH       			55
#define EXR1K_OP_ALU      			56
#define EXR1K_OP_SFXX     			57
#define EXR1K_OP_CUST5    			60


#define EXR1K_SYS_RF0				0x400



#define TBAR	0xff
#define TBCM	0x100
#define TBIM	0x200
#define TBRM	0x300
#define TBTM	0x400
#define TBCR	0x700

#define TBCR_EN_BITS  (1<<0)
#define TBCR_NE_BITS  (1<<1)

UINT32 jtag_single_step(UINT8 cpu);
UINT32 jtag_continue(UINT8 cpu);
UINT32 jtag_set_trap_pos(UINT32 addr, UINT8 cpu);
UINT32 jtag_read_cpu_register(UINT8 cpu, UINT8* buffer);
UINT32 jtag_read_cpu_status(UINT8 cpu, UINT8* buffer);
UINT32 jtag_read_cpu_all_register(UINT8 cpu);
UINT32 jtag_read_tb(UINT8 cpu, UINT8* buffer);
UINT32 jtag_enable_tb(UINT8 cpu);
UINT32 jtag_disable_tb(UINT8 cpu);
UINT32 jtag_get_ICCFGR(UINT8 cpu);
UINT32 jtag_get_TM(UINT cpu);
UINT32 jtag_read_tb_new(UINT8 cpu, UINT8* buffer);
#endif