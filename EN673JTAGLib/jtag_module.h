/*++

Copyright (c) 2015-2025 EYENIX Co. Ltd.

Module Name:

JTAG_MODULE.h

Abstract:

Environment:

Revision History:

15/05/18    oby     Created.

--*/

#ifndef JTAG_MODULE_H
#define JTAG_MODULE_H

#include <windows.h>

UINT32 jtag_select_module(UINT8 module);
UINT32 jtag_stall_cpu(UINT8 cpu);
UINT32 jtag_check_stalled_cpu(UINT8 cpu,UINT8* stalled);
UINT32 jtag_check_pcs(UINT8 cpu, UINT8* pcs);
UINT32 jtag_unstall_cpu(UINT8 cpu);
UINT32 jtag_reset_cpu(UINT8 cpu);
UINT32 jtag_unreset_cpu(UINT8 cpu);




#endif