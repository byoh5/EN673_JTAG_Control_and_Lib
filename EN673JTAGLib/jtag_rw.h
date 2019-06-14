/*++

Copyright (c) 2015-2025 EYENIX Co. Ltd.

Module Name:

JTAG_RW.h

Abstract:

Environment:

Revision History:

15/05/18    oby     Created.

--*/

#ifndef JTAG_RW_H
#define JTAG_RW_H

#include <windows.h>

UINT32 jtag_burst_write(void *data, UINT32 word_size_bytes, UINT32 word_count, unsigned long start_address, UINT module);
UINT32 jtag_burst_read(UINT32 word_size_bytes, UINT32 word_count, unsigned long start_address, void *data, UINT module);
UINT32 jtag_burst_command(UINT32 opcode, UINT32 address, UINT32 length_words, UINT8 module);
UINT32 jtag_compute_crc(UINT32 crc_in, UINT32 data_in, UINT32 length_bits);

UINT32 jtag_write32(UINT32 adr, UINT32 data, UINT32 module);
UINT32 jtag_write_block32(UINT32 adr, UINT32 *data, UINT32 len, UINT32 module);
UINT32 jtag_write_block8(UINT32 adr, UINT8 *data, UINT32 len, UINT32 module);
UINT32 jtag_read32(UINT32 adr, UINT32 *data, UINT32 module);
UINT32 jtag_read_block32(UINT32 adr, UINT32 *data, UINT32 len, UINT32 module);
UINT32 jtag_read_block8(UINT32 adr, UINT8 *data, UINT32 len, UINT32 module);

UINT32 jtag_wb_write32(UINT32 adr, UINT32 data, UINT32 cpu);
UINT32 jtag_wb_read32(UINT32 adr, UINT32 *data, UINT32 cpu);

INT32 jtag_gdb_burst_read32(UINT32* rbuf, INT32 len_words, UINT32 adr, UINT32 module);
INT32 jtag_gdb_burst_write32(UINT32* wbuf, INT32 len_words, UINT32 adr, UINT32 module);


#define RETRY_NUM	1000
#define BLEN_WORD 	2048			// burst length in word
#define	BLEN_HWORD	(BLEN_WORD*2)
#define BLEN_BYTE 	(BLEN_WORD*4)	

#endif