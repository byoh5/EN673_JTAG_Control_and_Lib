// stdafx.h : ���� ��������� ���� ��������� �ʴ�
// ǥ�� �ý��� ���� ���� �� ������Ʈ ���� ���� ������
// ��� �ִ� ���� �����Դϴ�.
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

// TODO: ���α׷��� �ʿ��� �߰� ����� ���⿡�� �����մϴ�.
extern int server_state_flag;
extern int lua_socket;
extern int	RemoteLog_flag;
extern unsigned int ERR_State;

#define JTAG_PRINT_SIZE_ADDR	0xF0010014
#define JTAG_PRINT0_START_ADDR	0xF0010018
#define JTAG_PRINT0_POINT_ADDR	0xF001001C
#define JTAG_PRINT1_START_ADDR	0xF0010020
#define JTAG_PRINT1_POINT_ADDR	0xF0010024