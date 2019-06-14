/*++

Copyright (c) 2015-2025 EYENIX Co. Ltd.

Module Name:

EN673JTAGLib.h

Abstract:

Environment:

Revision History:

15/05/18    oby     Created.

--*/
#ifndef EN673JTAG_H
#define EN673JTAG_H

#include "stdafx.h"
#include <windows.h>
#include <stdio.h>

extern "C" {
	UINT32 EN673_JTAG_INIT(UINT32 clkdiv, char channel);
	UINT32 EN673_JTAG_CLOSE(void);
	UINT32 EN673_REG_READ(UINT32 adr, UINT32 *data);
	UINT32 EN673_REG_WRITE(UINT32 adr, UINT32 data);
	UINT32 EN673_JTAG_TEST(void);
	void   EN673_JTAG_SET_ISP_BASE(UINT32 base);
}
SHORT  EN673_JTAG_ESC_key_checker(void);

#define DEBUG		printf("line-%.5d, function-%s, file-%s.\n", __LINE__, __FUNCTION__, __FILE__);
#define DEBUGERR	printf("line-%.5d, function-%s, file-%s.,Err: %d \n", __LINE__, __FUNCTION__, __FILE__, err);

extern CRITICAL_SECTION	CriticalSection;
extern CRITICAL_SECTION	CriticalSection_tap;

#endif