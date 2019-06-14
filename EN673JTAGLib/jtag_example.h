/*++

Copyright (c) 2015-2025 EYENIX Co. Ltd.

Module Name:

JTAG_EXAMPLE.h

Abstract:

Environment:

Revision History:

15/05/18    oby     Created.

--*/

#ifndef JTAG_EXAMPLE_H
#define JTAG_EXAMPLE_H

#include <windows.h>


UINT32 test_or1k_cpu0(UINT32 cpu);
UINT32 test_cpu(UINT8 cpu);
UINT32 test_memory(UINT32 *writedata, UINT32 *readdata, UINT32 adr, UINT32 len, UINT32 module);
UINT32 test_mem(void);
UINT32 test_sdram_2(UINT32 adr, UINT32 data, UINT32 module);
UINT32 example_uart_control(void);
UINT32 example_uart_print(UINT32 cpu, UINT8 *input, UINT32 len);
UINT32 example_wbcom_gpio(void);
UINT32 example_read_cpu_register(void);
UINT32 example_write_cpu_register(UINT32 addr, UINT32 cpu);
UINT32 example_read_register(UINT32 module, UINT32 addr);
UINT32 example_read_register(UINT32 module, UINT32 addr, UINT32 *data);
UINT32 test(void);



#endif