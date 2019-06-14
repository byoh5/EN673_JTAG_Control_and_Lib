/*++

Copyright (c) 2015-2025 EYENIX Co. Ltd.

Module Name:

JTAG_BASE.h

Abstract:

Environment:

Revision History:

15/05/18    oby     Created.

--*/

#ifndef JTAG_BASE_H
#define JTAG_BASE_H

#define CHANGE_EN674	0

#include <windows.h>

UINT32 MPSSE_init(UINT32 clkdiv, char channel);
UINT32 MPSSE_init_withDescirption(UINT32 clkdiv, UINT8* desciption);
UINT32 MPSSE_close(void);
//TMS
UINT32 MPSSE_TMS_bit(UINT8 len, UINT8 bit, UINT8 edge);
UINT32 MPSSE_TMS_TDO_bit(UINT8 len, UINT8 bit, UINT8 edge);
//TDI-bit
UINT32 MPSSE_TDI_bit(UINT8 len, UINT8 bit, UINT8 edge);
UINT32 MPSSE_TDI_TDO_bit(UINT8 len, UINT8 bit, UINT8 edge);
//TDI-byte
UINT32 MPSSE_TDO_write_byte(UINT16 len, UINT8 *data, UINT8 edge);
UINT32 MPSSE_TDO_write_read_byte(UINT16 len, UINT8 *outdata, UINT8 *byte, UINT8 edge);
//TDO(read)
UINT32 MPSSE_TDO_read_bit(UINT8* data);
UINT32 MPSSE_TDO_read_byte(UINT8* data, UINT cnt);

UINT32 MPSSE_read_write_TDI(UINT len, UCHAR packet, UCHAR *in_bit);
UINT32 MPSSE_read_write_TMS(UINT len, UCHAR packet, UCHAR *in_bit);
UINT32 MPSSE_write_stream(UINT32 *in_data, UINT32 length_bits, UCHAR set_TMS);
UINT32 MPSSE_read_write_stream(UINT32 *in_data, UINT32 *out_data, UINT32 length_bits);

UINT32 MPSSE_release_port(void);
UINT32 MPSSE_init_port(void);
UINT32 MPSSE_GETListNum(void);

inline int ftdx_flush(void);

#define ERR_MPSSE_INIT                  0x00000001  
#define ERR_MPSSE_CLOSE                 0x00000002  
#define ERR_MPSSE_TMS_BIT               0x00000004  
#define ERR_MPSSE_TMS_TDO_BIT           0x00000008  
#define ERR_MPSSE_TDI_BIT               0x00000010  
#define ERR_MPSSE_TDI_TDO_BIT           0x00000020  
#define ERR_MPSSE_TDO_WRITE_BYTE        0x00000040  
#define ERR_MPSSE_TDO_WRITE_READ_BYTE   0x00000080  
#define ERR_MPSSE_TDO_READ_BIT          0x00000100  
#define ERR_MPSSE_TDO_READ_BYTE         0x00000200  
#define ERR_MPSSE_READ_WRITE_TDI        0x00000400  
#define ERR_MPSSE_READ_WRITE_TMS        0x00000800  
#define ERR_MPSSE_WRITE_STREAM          0x00001000  
#define ERR_MPSSE_READ_WRITE_STREAM     0x00002000  

#define ERR_MPSSE_BURST_WRITE_STREAM    0x00004000 
#define ERR_MPSSE_BURST_READ_STREAM     0x00008000 
#define ERR_MPSSE_FLASH_RETRY_OVER      0x00010000

#define	ERR_MPSSE_CRC					0x00020000	// ksh
#endif