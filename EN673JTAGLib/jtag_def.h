/*++

Copyright (c) 2015-2025 EYENIX Co. Ltd.

Module Name:

JTAG_DEF.h

Abstract:

Environment:

Revision History:

15/05/18    oby     Created.

--*/

#ifndef JTAG_DEF_H
#define JTAG_DEF_H

#define JTAG_WB0_MODULE_IDX 0
#define JTAG_WB1_MODULE_IDX 1
#define JTAG_COMMON_MODULE_IDX 3
#define JTAG_CPU0_MODULE_IDX 4
#define JTAG_CPU1_MODULE_IDX 5

#define WB0		0x00
#define WB1		0x01
#define WB2		0x02
#define WBCOM	0x03
#define CPU0	0x04
#define CPU1	0x05
#define CPU2	0x06

#define DC_WISHBONE       0
#define DC_WISHBONE1      1
#define DC_WISHBONE2      2
#define DC_WISHBONE3      3
#define DC_CPU0     4
#define DC_CPU1     5
#define DC_CPU2     6
#define DC_JSP      9


#define JTAG_STALL		0x1
#define JTAG_UNSTALL	0x0
#define JTAG_PCS		0x2
#define JTAG_RESET		0x1
#define JTAG_UNRESET	0x0

#define JTAG_RID_ER			0x0
#define JTAG_RID_SR			0x2
#define JTAG_RID_RR			0x3

#define JTAG_INDEX_STALL	0x2
#define JTAG_INDEX_RESET	0x3


#define		NOP			0x00
#define		READ8		0x19
#define		READ16		0x1A
#define		READ32		0x1B
#define		WRITE8		0x1D
#define		WRITE16		0x1E
#define		WRITE32		0x1F
#define		REG_SELECT	0x18
#define		REG_WRITE	0x1C

#define DBG_WB_OPCODE_LEN   5
#define DBG_WB_CMD_NOP      NOP
#define DBG_WB_CMD_BWRITE8  WRITE8
#define DBG_WB_CMD_BWRITE16 WRITE16
#define DBG_WB_CMD_BWRITE32 WRITE32
#define DBG_WB_CMD_BREAD8   READ8
#define DBG_WB_CMD_BREAD16  READ16
#define DBG_WB_CMD_BREAD32  READ32
#define DBG_WB_CMD_IREG_WR  REG_WRITE	// This is both a select and a write
#define DBG_WB_CMD_IREG_SEL REG_SELECT  // There is no 'read', the current register is always read.  Use a NOP to read.

// Opcode definitions for the first CPU module
#define DBG_CPU0_OPCODE_LEN   5
#define DBG_CPU0_CMD_NOP      NOP
#define DBG_CPU0_CMD_BWRITE32 WRITE32
#define DBG_CPU0_CMD_BREAD32  READ32
#define DBG_CPU0_CMD_IREG_WR  REG_WRITE  // This is both a select and a write
#define DBG_CPU0_CMD_IREG_SEL REG_SELECT  // There is no 'read', the current register is always read.  Use a NOP to read.

// Internal register definitions for the CPU1 module
#define DBG_CPU1_REG_SEL_LEN 1
#define DBG_CPU1_REG_STATUS 0
// Opcode definitions for the second CPU module
#define DBG_CPU1_OPCODE_LEN   5
#define DBG_CPU1_CMD_NOP      NOP
#define DBG_CPU1_CMD_BWRITE32 WRITE32
#define DBG_CPU1_CMD_BREAD32  READ32
#define DBG_CPU1_CMD_IREG_WR  REG_WRITE  // This is both a select and a write
#define DBG_CPU1_CMD_IREG_SEL REG_SELECT  // There is no 'read', the current register is always read.  Use a NOP to read.

#define SDRAM_BASE 0x04000000
#if CHANGE_EN674
#define FLASH_BASE 0x2F000000 // EN674
#else
#define FLASH_BASE 0x06000000 // EN673
#endif

#define FAST_MODE 
#define DATA_VERIFY

#define JTAP_IR_EXTEST          0x0
#define JTAP_IR_SAMPLE_PRELOAD  0x1
#define JTAP_IR_IDCODE          0x2
#define JTAP_IR_MBIST         	0x3
#define JTAP_IR_DEBUG           0x8
#define JTAP_IR_BPDEBUG			0x9					// bypassed debug 			// $TST-ygkim-150710: for multi-drop
#define JTAP_IR_BYPASS          0xf           



#define	JTAG_CHECKSTOP		2
#define	JTAG_CHECK			1
#define	JTAG_NOCHECK		0

//------------------------------------------------------------------------------
// Bit
#define JTAG_CRC_POLY 		0xedb88320

#define	JTAG_TDO_BIT		0
#define	JTAG_TMS_BIT		1
#define	JTAG_TCK_BIT		2

#define	JTAG_NONE			0x0
#define	JTAG_TDO_SET		0x1
#define	JTAG_TMS_SET		0x2
#define	JTAG_TCK_SET		0x4

#define	TMS_ON				1
#define	TMS_OFF				0

#define	WAIT_ON				1
#define	WAIT_OFF			0


//******************************************************************************
// Data steram
//------------------------------------------------------------------------------
#define JCMD_W				1			// command width
#define	JBOP_W				5			// burst operation width
#define	JBADR_W				32			// burst address width
#define	JBLEN_W				15			// burst length width
#define JDAT_W	  			(JCMD_W+JBOP_W+JBADR_W+JBLEN_W)		// data width

// Command                                                                  	
#define	JCMD_TAP			1			// 1: tap, 0: submodules
#define	JCMD_SUB			0

//******************************************************************************
// JTAG Tap/Top module
//------------------------------------------------------------------------------
// Tapmodule Opcode			
#define	JTOP_W				1			// Top ID width
#define	JCID_W				5			// Chain ID	width						// $TST-ygkim-150720: max 32+1
#define	JMID_W				4			// Module ID width

#define	JTOP_CID_SET		0
#define	JTOP_MID_SET		1

//******************************************************************************
// JTAG Submodule
//------------------------------------------------------------------------------
// Burst Opcode																	// $TST-ygkim-150710: changed op: 5bits
#define	JBOP_STORE				1
#define	JBOP_LOAD				0

#define	JBOP_BTYPE_W			2												// burst type
#define	JBOP_BYTE				1           		    			
#define	JBOP_HWORD				2                       			
#define	JBOP_WORD				3                       			

#define	JBOP_MTYPE_W			2												// memory type
#define	JBOP_BUF				1												// int. buffer
#define	JBOP_DMA				2												// ext. memory based on DMA
#define	JBOP_ALL				3												// all access

#define	JBOP_SIR				((JBOP_LOAD <<JBOP_BTYPE_W)|0			)		// 000  Select internal reg.
#define JBOP_LDB   				((JBOP_LOAD <<JBOP_BTYPE_W)|JBOP_BYTE 	) 		// 001  Read 8-bit data
#define JBOP_LDH  				((JBOP_LOAD <<JBOP_BTYPE_W)|JBOP_HWORD	)  		// 010  Read 16-bit data
#define JBOP_LDW  				((JBOP_LOAD <<JBOP_BTYPE_W)|JBOP_WORD	) 		// 011  Read 32-bit data
#define	JBOP_WIR				((JBOP_STORE<<JBOP_BTYPE_W)|0			)		// 100  Write internal reg.
#define JBOP_STB  				((JBOP_STORE<<JBOP_BTYPE_W)|JBOP_BYTE 	) 		// 101  Write 8-bit data
#define JBOP_STH 				((JBOP_STORE<<JBOP_BTYPE_W)|JBOP_HWORD	)  		// 110  Write 16-bit data
#define JBOP_STW 				((JBOP_STORE<<JBOP_BTYPE_W)|JBOP_WORD	) 		// 111  Write 32-bit data

#define	JBOP_NOP				0												// 00000  NOP to fast read internal register already selected
// 00001-00111	Reserved
// 01000	Reserved
#define JBOP_BUF_LB 			((JBOP_BUF<<(1+JBOP_BTYPE_W))|JBOP_LDB 	) 		// 01001  Read 8-bit buffer data
#define JBOP_BUF_LH 			((JBOP_BUF<<(1+JBOP_BTYPE_W))|JBOP_LDH	)  		// 01010  Read 16-bit buffer data
#define JBOP_BUF_LW 			((JBOP_BUF<<(1+JBOP_BTYPE_W))|JBOP_LDW	) 		// 01011  Read 32-bit buffer data
// 01100  	Reserved
#define JBOP_BUF_SB 			((JBOP_BUF<<(1+JBOP_BTYPE_W))|JBOP_STB 	) 		// 01101  Write 8-bit buffer data
#define JBOP_BUF_SH 			((JBOP_BUF<<(1+JBOP_BTYPE_W))|JBOP_STH	)  		// 01110  Write 16-bit buffer data
#define JBOP_BUF_SW 			((JBOP_BUF<<(1+JBOP_BTYPE_W))|JBOP_STW	) 		// 01111  Write 32-bit buffer data
// 10000	Reserved
#define JBOP_DMA_LB 			((JBOP_DMA<<(1+JBOP_BTYPE_W))|JBOP_LDB 	) 		// 10001  Read 8-bit from mem to buf
#define JBOP_DMA_LH 			((JBOP_DMA<<(1+JBOP_BTYPE_W))|JBOP_LDH	)  		// 10010  Read 16-bit from mem to buf
#define JBOP_DMA_LW 			((JBOP_DMA<<(1+JBOP_BTYPE_W))|JBOP_LDW	) 		// 10011  Read 32-bit from mem to buf
// 10100  	Reserved
#define JBOP_DMA_SB 			((JBOP_DMA<<(1+JBOP_BTYPE_W))|JBOP_STB 	) 		// 10101  Write 8-bit from buf to mem
#define JBOP_DMA_SH 			((JBOP_DMA<<(1+JBOP_BTYPE_W))|JBOP_STH	)  		// 10110  Write 16-bit from buf to mem
#define JBOP_DMA_SW 			((JBOP_DMA<<(1+JBOP_BTYPE_W))|JBOP_STW	) 		// 10111  Write 32-bit from buf to mem
// 11000	Reserved
#define	JBOP_IR_SEL				((JBOP_ALL<<(1+JBOP_BTYPE_W))|JBOP_SIR	)		// 11000  Select internal reg.
#define JBOP_LB 				((JBOP_ALL<<(1+JBOP_BTYPE_W))|JBOP_LDB 	) 		// 11001  Read 8-bit from mem to buf & jtag
#define JBOP_LH 				((JBOP_ALL<<(1+JBOP_BTYPE_W))|JBOP_LDH	)  		// 11010  Read 16-bit from mem to buf & jtag
#define JBOP_LW 				((JBOP_ALL<<(1+JBOP_BTYPE_W))|JBOP_LDW	) 		// 11011  Read 32-bit from mem to buf & jtag
#define	JBOP_IR_WR				((JBOP_ALL<<(1+JBOP_BTYPE_W))|JBOP_WIR	)		// 11100  Write internal reg.
#define JBOP_SB 				((JBOP_ALL<<(1+JBOP_BTYPE_W))|JBOP_STB 	) 		// 11101  Write 8-bit from mem to buf & jtag
#define JBOP_SH 				((JBOP_ALL<<(1+JBOP_BTYPE_W))|JBOP_STH	)  		// 11110  Write 16-bit from mem to buf & jtag
#define JBOP_SW 				((JBOP_ALL<<(1+JBOP_BTYPE_W))|JBOP_STW	) 		// 11111  Write 32-bit from mem to buf & jtag

//------------------------------------------------------------------------------
// Internal registers
// address
#define JIR_AW					2	// Address width

#define JIR_ER 					0	// Bus error register
#define JIR_SR 					2	// CPU stall register
#define JIR_RR					3	// CPU reset register

// R0: Error register
#define JWB_ER_WDW 				1	// write data width

#define	JWB_ER_RESET			1

// R2: Stall register
#define JCPU_SR_DW 				2

#define	JCPU_SR_STALL			1
#define	JCPU_SR_UNSTALL			0

// R3: Reset register
#define JCPU_RR_DW 				1

#define	JCPU_RR_RESET			1
#define	JCPU_RR_UNRESET			0


#define	DU_TGR(TID,GID,RID)		(((TID)<<18)|((GID)<<13)|((RID)<<2))
#define	DU_TA(TID,ADR)			(((TID)<<18)|((ADR)<<2))


#endif