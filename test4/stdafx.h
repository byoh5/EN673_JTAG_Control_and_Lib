// stdafx.h : 자주 사용하지만 자주 변경되지는 않는
// 표준 시스템 포함 파일 및 프로젝트 관련 포함 파일이
// 들어 있는 포함 파일입니다.
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

// TODO: 프로그램에 필요한 추가 헤더는 여기에서 참조합니다.
extern int server_state_flag;
extern int lua_socket;
extern int	RemoteLog_flag;
extern unsigned int ERR_State;

#define JTAG_PRINT_SIZE_ADDR	0xF0010014
#define JTAG_PRINT0_START_ADDR	0xF0010018
#define JTAG_PRINT0_POINT_ADDR	0xF001001C
#define JTAG_PRINT1_START_ADDR	0xF0010020
#define JTAG_PRINT1_POINT_ADDR	0xF0010024