/*++

Copyright (c) 2015-2025 EYENIX Co. Ltd.

Module Name:

JTAG_TAP.h

Abstract:

Environment:

Revision History:

15/05/18    oby     Created.

--*/

#ifndef JTAG_TAP_H
#define JTAG_TAP_H

#include <windows.h>

UINT32 tap_reset(void);
UINT32 tap_reset_global(void);
UINT32 tap_reset_global0(void);
UINT32 tap_reset_global1(void);
UINT32 tap_idle2shift_dr(void);
UINT32 tap_idle2shift_ir(void);
UINT32 tap_shift2idle(void);
UINT32 tap_shift2idle2(void);
UINT32 tap_set_ir_debug(void);
UINT32 tap_get_ir_ID(UINT32 *idcodes, int *num_devices);
UINT32 tap_set_ir_idcode(void);
UINT32 tap_set_ir(UCHAR instruction);

#endif