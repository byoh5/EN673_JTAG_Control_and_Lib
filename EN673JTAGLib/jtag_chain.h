/*++

Copyright (c) 2015-2025 EYENIX Co. Ltd.

Module Name:

JTAG_CHAIN.h

Abstract:

Environment:

Revision History:

15/11/26    oby     Created.

--*/

#ifndef JTAG_CHAIN_H
#define JTAG_CHAIN_H

#define	JTAP_IR_W				4

#define MAX_DEVICES				32
#define ALLOC_SIZE				64

extern UINT32 jtag_chain_write_stream(UINT32* wstream, UINT32 lbits, UCHAR last_tms);
extern UINT32 jtag_chain_read_write_stream(UINT32* wstream, UINT32* rstream, UINT32 lbits, UCHAR last_tms, UCHAR adjust);

extern void jtag_set_chain_target(int target_id, int Jtag_ndevices);
extern UINT32 jtag_set_tap_ir(UINT32 ir_val, UINT32 ir_etc);
extern int jtag_get_Drprefixbits(void);
extern UINT32 jtag_config_chain(void);

#endif