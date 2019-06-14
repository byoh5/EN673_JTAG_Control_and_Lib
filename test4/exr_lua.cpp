//******************************************************************************
//	Copyright(c) Young-Geun Kim 2003-
//
//	File Name:		exr_lua.c
//	Description:	
//	Author:			Young-Geun Kim (ygkim@{eyenix.com;rcv.kaist.ac.kr}), Eyenix Co.,Ltd.
//
//	Note:	
//
//	Ver Date		Author		Changes
//	--- ----		------		-------
//	0.1	171106		ygkim		first designed
// -----------------------------------------------------------------------------

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <stdlib.h>
//#include <unistd.h>
#include <string.h>
//#include <sys/time.h>
#include <basetsd.h>
#include "lua.hpp"
#include "jtag_example.h"
#include "jtag_flash.h"

#include "jtag_def.h"
#include "jtag_rw.h"
#include "jtag_debug.h"
#include "time.h"
#include "spr_defs.h"
#include "atlstr.h"
#include "jtag_chain.h"
#include "jtag_tap.h"

#pragma comment(lib, "lua5.1.lib")
#pragma comment(lib, "lua51.lib")
typedef unsigned int UINT32;
int jtag_read_cpu_all_register_for_lua(UINT8 cpu, char *buffer);
UINT32 jtag_read_tb_new_for_lua(UINT8 cpu, UINT8* buffer);
void Eliminate(char *str, char ch);

char* timeToString(struct tm *t);
#define STOP_SCRIPT 0
#define START_SCRIPT 1
#define TB_LEN 256
#define JTAG_PRINT_SIZE_ADDR	0xF0010014
#define JTAG_PRINT0_START_ADDR	0xF0010018
#define JTAG_PRINT0_POINT_ADDR	0xF001001C
#define JTAG_PRINT1_START_ADDR	0xF0010020
#define JTAG_PRINT1_POINT_ADDR	0xF0010024
#define MR_READBUFFER 1024*8
#define MAX_PRINT_BUF 2048




#ifdef _LUA_DEBUG
#define _INS
#else
#define _INS				//__attribute__((no_instrument_function))
#endif



extern int script_state;
extern int lua_flag;

UINT32 _DoCommand(char *cmdline);


UINT32 now_pos;
UINT32 last_pos;
UINT32 jtag_print_size;
UINT32 jtag_print_base;

//******************************************************************************
// for testing Lua
//------------------------------------------------------------------------------


lua_State *pLua;



UINT32 jtag_htonl_(UINT32 n)
{
	return ((n & 0xff) << 24) |
		((n & 0xff00) << 8) |
		((n & 0xff0000UL) >> 8) |
		((n & 0xff000000UL) >> 24);
}

UINT32 ReadJTAGPrintData_(UINT32 last_pos, UINT32 size, UINT8* pbuf, UINT8 on)
{
	UINT8 * memblock = NULL;
	UINT8 * memblock_t = NULL;
	UINT8 * memblock_new_pos = NULL;
	UINT32* memblock32 = NULL;
	UINT32* memblock32_t = NULL;
	UINT32 retry = 0, left = 0;
	UINT32 i = 0;
	UINT32 origsize = 0;
	origsize = size;
	//size = ((size + 4) >> 2) << 2;
	UINT32 offset = 0;
	offset = (last_pos & 0x03);
	size += (offset + 4);
	memblock = (UINT8*)malloc(size + 8);
	memblock_t = (UINT8*)malloc(size + 8);
	retry = (size) / (MR_READBUFFER);
	left = (size) % (MR_READBUFFER);

	for (i = 0; i < retry; i++){
		jtag_read_block32(last_pos + (MR_READBUFFER * i), (UINT32*)(memblock + (MR_READBUFFER * i)), (MR_READBUFFER / 4), 3);
	}
	if (left)
		jtag_read_block32(last_pos + (MR_READBUFFER * i), (UINT32*)(memblock + (MR_READBUFFER * i)), (left / 4) + 1, 3);

	memblock32 = (UINT32*)memblock;
	memblock32_t = (UINT32*)memblock_t;
	for (i = 0; i < (size / 4) + 1; i++){
		*(memblock32_t + i) = jtag_htonl_((UINT32)*(memblock32 + i));
	}
	memblock_t = (UINT8*)memblock32_t;
	memblock_new_pos = memblock_t + offset;
	memblock_new_pos[origsize] = 0;
	//   hexDump("DUMP", memblock_t, origsize);
	if (on == 0) printf("%s", memblock_new_pos);
	if (on) memcpy(pbuf, memblock_new_pos, origsize + 1);

	if (memblock) free(memblock);
	if (memblock_t) free(memblock_t);

	return 0;
}

void exr_set_jtag_print(UINT32 module)
{
	jtag_read_block32(JTAG_PRINT_SIZE_ADDR, &jtag_print_size, 1, JTAG_COMMON_MODULE_IDX);
	if (module == 4)
		jtag_read_block32(JTAG_PRINT0_START_ADDR, &jtag_print_base, 1, JTAG_COMMON_MODULE_IDX);
	if (module == 5)
		jtag_read_block32(JTAG_PRINT1_START_ADDR, &jtag_print_base, 1, JTAG_COMMON_MODULE_IDX);
}



void exr_resetpos(void){
	now_pos = jtag_print_base;
	last_pos = jtag_print_base;
}

UINT32 JTAGPrint_(UINT8* pbuf, UINT8 on, UINT32 module){

	UINT fir_size = 0;
	UINT sec_size = 0;
	UINT32 size = 0;
	INT32 check = 0;

	// get size of jtag_print
	if (module == 4)
		jtag_read_block32(JTAG_PRINT0_POINT_ADDR, &now_pos, 1, JTAG_COMMON_MODULE_IDX);
	if (module == 5)
		jtag_read_block32(JTAG_PRINT1_POINT_ADDR, &now_pos, 1, JTAG_COMMON_MODULE_IDX);

	if ((jtag_print_base <= now_pos) & (now_pos < (jtag_print_base + jtag_print_size))){

		if (now_pos == last_pos){
			return 0;
		}
		check = (now_pos - last_pos);
		if (check > 0)
		{
			size = now_pos - last_pos;
			if (size > MAX_PRINT_BUF) size = MAX_PRINT_BUF;
			ReadJTAGPrintData_(last_pos, size, pbuf, on);
			last_pos += size;
		}
		else if (check < 0){
			fir_size = (jtag_print_base + jtag_print_size) - last_pos;
			ReadJTAGPrintData_(last_pos, fir_size, pbuf, on);

			sec_size = now_pos - (jtag_print_base);
			ReadJTAGPrintData_(jtag_print_base, sec_size, pbuf + fir_size, on);
			last_pos = now_pos;
			size = fir_size + sec_size;
		}
		else{
			return 0;
		}
	}
	return size;
}


int exr_lua_cmd(lua_State* L)
{
	char* name = (char*)luaL_checkstring(L, 1);
	
	_DoCommand(name);

	return 1;
}

int exr_lua_state(lua_State* L)
{
	lua_pushnumber(L, START_SCRIPT);
	if (lua_flag == 0)
	lua_pushnumber(L, STOP_SCRIPT);

	return 1;
}



_INS int exr_lua_read_reg(lua_State* L)
{
	int mid;
	FILE *fp = NULL;
	UINT32 reg=0;
	char * module = (char *)luaL_checkstring(L, 1);
	UINT32 addr = luaL_checknumber(L, 2);

	addr = ((addr >> 2) << 2);

	if (strcmp("wbcpu0", module) == 0) 			mid = 0;
	else if (strcmp("wbcpu1", module) == 0) 	mid = 1;
	else if (strcmp("wbcom", module) == 0)		mid = 3;
	else if (strcmp("cpu0", module) == 0)		mid = 4;
	else if (strcmp("cpu1", module) == 0)		mid = 5;
	else {
		printf("Error : option choose (wbcpu0,wbcpu1,wbcom,cpu0,cpu1)'\n");
		return 0;
	}

	reg = example_read_register(mid, addr );
	lua_pushnumber(L, reg);
	
	return 1;
}


_INS int exr_lua_write_reg(lua_State* L)
{
	int mid;
	char* module = (char*)luaL_checkstring(L, 1);
	UINT32 addr = (UINT32)luaL_checknumber(L, 2);
	UINT32 data = (UINT32)luaL_checknumber(L, 3);
	UINT32 reg = 0;

	if (strcmp("wbcpu0", module) == 0) 			mid = 0;
	else if (strcmp("wbcpu1", module) == 0) 	mid = 1;
	else if (strcmp("wbcom", module) == 0)		mid = 3;
	else if (strcmp("cpu0", module) == 0)		mid = 4;
	else if (strcmp("cpu1", module) == 0)		mid = 5;
	else {
		printf("Error : option choose (wbcpu0,wbcpu1,wbcom,cpu0,cpu1)'\n");
		return 0;
	}


	if ((addr >> 24) == 0x06)
	{
		Sfls_init_quad();
		Sfls_point_write32(data, (addr - FLASH_BASE));
		Sleep(20);
	}
	else
	{
		jtag_write32(addr, data, mid);
	}

	jtag_read32(addr , &reg, mid);
	if (data != reg) printf("write fail\n");

	return 0;
}

/*
_INS int exr_lua_read_mem(lua_State* L)
{
	int mid;
	char *mstr = (char*)luaL_checkstring(L, 1);
	uint addr = luaL_checknumber(L, 2);
	uint size = luaL_checkint(L, 3);


	if (strcmp("wbcpu0", mstr) == 0) 			mid = 0;
	else if (strcmp("wbcpu1", mstr) == 0) 		mid = 1;
	else if (strcmp("wbcom", mstr) == 0)		mid = 3;
	else if (strcmp("cpu0", mstr) == 0)			mid = 4;
	else if (strcmp("cpu1", mstr) == 0)			mid = 5;
	else										mid = -1;


	if (addr&~ADDR_MASK) {	// a multiple of 46
		ERROR("address 0x%0x is not a multiple of 4!\n", addr);
		return CLI_ERROR;
	}

	uint size4 = (size + 3) & ADDR_MASK;

	// read from memory
	char* rbuf = (char*)malloc(size4 + 4);	// big
	char* wbuf = (char*)malloc(size4 + 4);	// little

	//jtag_gdb_burst_read32((uint*)rbuf, size4 / 4, addr); 
	//jtag_gdb_burst_write32_module(&data, 1, addr, mid); 


	jtag_gdb_burst_read32_module((uint*)rbuf, size4 / 4, addr, mid);

	uint* rbuf32 = (uint*)rbuf;
	uint* wbuf32 = (uint*)wbuf;

	int i;
	for (i = 0; i < size4 / 4; i++, rbuf32++, wbuf32++)
	{
		*wbuf32 = htonl(*rbuf32);
	}

	//hex_dump("DUMP(rbuf->char*)", rbuf, 60);
	hex_dump("DUMP(wbuf->uint*)", wbuf, 60);
	//hex_dump("DUMP(wbuf32->uint*(endian change))", wbuf32, 60);

	//void * chan;
	lua_pushlightuserdata(L, (void*)wbuf);//1시도
	//lua_settable(L, LUA_GLOBALSINDEX);
	//chan=lua_touserdata(L, 4);
	printf("wbuf=%x\n", wbuf);
	//printf("chan =%x\n", chan);


	if (rbuf) free(rbuf);
	if (wbuf) free(wbuf);

	return 1;
}

_INS int exr_lua_write_mem(lua_State* L)
{
	//int mid;
	return 0;
}
*/


_INS int debug_file_one_jtag(lua_State* L)
{
	char tbp_title_cpu0[15] = "(tbp_cpu0).htb";
	char dcr_title_cpu0[15] = "(dcr_cpu0).txt";
	char tbp_title_cpu1[15] = "(tbp_cpu1).htb";
	char dcr_title_cpu1[15] = "(dcr_cpu1).txt";
	struct tm *t;
	time_t timer;
	timer = time(NULL);    // 현재 시각을 초 단위로 얻기
	t = localtime(&timer); // 초 단위의 시간을 분리하여 구조체에 넣기
	int mid = 0;
	FILE *stream = NULL;
	FILE *stream1 = NULL;
	
	char s[35] = { 0, };
	TCHAR path[MAX_PATH];
	char cpath[MAX_PATH];
	char* module = (char*)luaL_checkstring(L, 1);

	if (strcmp("isp", module) == 0) 			mid = 4;
	if (strcmp("ip0", module) == 0) 			mid = 5;

	if (mid == 4)
	{
		// dcr제목얻기
		GetCurrentDirectory(MAX_PATH, path);
		CString strDirectory = _T("");
		strDirectory.Format(_T("%s\\htb_files\\"), path);
		CreateDirectory(strDirectory, NULL);
		WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);

		timer = time(NULL);    // 현재 시각을 초 단위로 얻기
		t = localtime(&timer); // 초 단위의 시간을 분리하여 구조체에 넣기
		sprintf(s, "%04d-%02d-%02d,%02d%02d%02d",
			t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
			t->tm_hour, t->tm_min, t->tm_sec
			);
		//printf("3333333\n");

		strcat(s, dcr_title_cpu0);
		strcat(cpath, s);

		if (fopen_s(&stream1, cpath, "w+t") != 0){
			printf("file open fail\n");
			return 0;
		}

		//tbp제목 얻기
		// printf("111111111111\n");

		WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);

		sprintf(s, "%04d-%02d-%02d,%02d%02d%02d",
			t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
			t->tm_hour, t->tm_min, t->tm_sec
			);

		strcat(s, tbp_title_cpu0);
		strcat(cpath, s);

		if (fopen_s(&stream, cpath, "w+t") != 0){
			printf("file open fail\n");
			return 0;
		}
	}
	else if(mid == 5)
	{
		// dcr제목얻기
		GetCurrentDirectory(MAX_PATH, path);
		CString strDirectory = _T("");
		strDirectory.Format(_T("%s\\htb_files\\"), path);
		CreateDirectory(strDirectory, NULL);
		WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);

		timer = time(NULL);    // 현재 시각을 초 단위로 얻기
		t = localtime(&timer); // 초 단위의 시간을 분리하여 구조체에 넣기
		sprintf(s, "%04d-%02d-%02d,%02d%02d%02d",
			t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
			t->tm_hour, t->tm_min, t->tm_sec
			);

		strcat(s, dcr_title_cpu1);
		strcat(cpath, s);
		if (fopen_s(&stream1, cpath, "w+t") != 0){
			printf("file open fail\n");
			return 0;
		}

		//tbp제목 얻기
		// printf("111111111111\n");

		WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);

		sprintf(s, "%04d-%02d-%02d,%02d%02d%02d",
			t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
			t->tm_hour, t->tm_min, t->tm_sec
			);

		strcat(s, tbp_title_cpu1);
		strcat(cpath, s);
		if (fopen_s(&stream, cpath, "w+t") != 0){
			printf("file open fail\n");
			return 0;
		}
	}

	char* memblock;
	memblock = (char*)malloc(100 * 256 * 4);
	jtag_read_tb_new_for_lua(mid, (UINT8*)memblock);
	fwrite(memblock, sizeof(char), strlen(memblock), stream);
	fclose(stream);
	free(memblock);
	
	char *memblock1;
	memblock1 = (char*)malloc(1200);
	jtag_read_cpu_all_register_for_lua(mid,memblock1);
	fwrite(memblock1, sizeof(char), strlen(memblock1), stream1);
	fclose(stream1);
	free(memblock1);

	// TCHAR path[MAX_PATH];
	// GetCurrentDirectory(MAX_PATH, path);
	// char cpath[MAX_PATH];
	//WideCharToMultiByte(CP_ACP, 0, path, MAX_PATH, cpath, MAX_PATH, NULL, NULL);
	// strcat(cpath, "\\");
	 //strcat(cpath, s);
	 lua_pushstring(L, cpath);
	return 1;
}

_INS int debug_file_multi_jtag(lua_State *L)
{

	char tbp_title_cpu0[15] = "(tbp_cpu0).htb";
	char dcr_title_cpu0[15] = "(dcr_cpu0).txt";
	char tbp_title_cpu1[15] = "(tbp_cpu1).htb";
	char dcr_title_cpu1[15] = "(dcr_cpu1).txt";
	struct tm *t;
	time_t timer;
	timer = time(NULL);    // 현재 시각을 초 단위로 얻기
	t = localtime(&timer); // 초 단위의 시간을 분리하여 구조체에 넣기
	int mid = 0;
	FILE *stream = NULL;
	FILE *stream1 = NULL;

	char s[50] = { 0, };
	TCHAR path[MAX_PATH];
	char cpath[MAX_PATH];
	char* module = (char*)luaL_checkstring(L, 1);
	char* target = (char*)luaL_checkstring(L, 2);
	char target_arry[10] = { 0, };
	strcpy(target_arry, target);

	if (strcmp("isp", module) == 0) 			mid = 4;
	if (strcmp("ip0", module) == 0) 			mid = 5;

	if (mid == 4)
	{
		// dcr제목얻기
		GetCurrentDirectory(MAX_PATH, path);
		CString strDirectory = _T("");
		strDirectory.Format(_T("%s\\htb_files\\"), path);
		CreateDirectory(strDirectory, NULL);
		WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);

		timer = time(NULL);    // 현재 시각을 초 단위로 얻기
		t = localtime(&timer); // 초 단위의 시간을 분리하여 구조체에 넣기
		sprintf(s, "%04d-%02d-%02d,%02d%02d%02d",
			t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
			t->tm_hour, t->tm_min, t->tm_sec
			);
		//printf("3333333\n");
		strcat(s, target_arry);
		strcat(s, dcr_title_cpu0);
		strcat(cpath, s);

		if (fopen_s(&stream1, cpath, "w+t") != 0){
			printf("file open fail\n");
			return 0;
		}

		//tbp제목 얻기
		// printf("111111111111\n");

		WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);

		sprintf(s, "%04d-%02d-%02d,%02d%02d%02d",
			t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
			t->tm_hour, t->tm_min, t->tm_sec
			);
		strcat(s, target_arry);
		strcat(s, tbp_title_cpu0);
		strcat(cpath, s);

		if (fopen_s(&stream, cpath, "w+t") != 0){
			printf("file open fail\n");
			return 0;
		}
	}
	else if (mid == 5)
	{
		// dcr제목얻기
		GetCurrentDirectory(MAX_PATH, path);
		CString strDirectory = _T("");
		strDirectory.Format(_T("%s\\htb_files\\"), path);
		CreateDirectory(strDirectory, NULL);
		WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);

		timer = time(NULL);    // 현재 시각을 초 단위로 얻기
		t = localtime(&timer); // 초 단위의 시간을 분리하여 구조체에 넣기
		sprintf(s, "%04d-%02d-%02d,%02d%02d%02d",
			t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
			t->tm_hour, t->tm_min, t->tm_sec
			);
		strcat(s, target_arry);
		strcat(s, dcr_title_cpu1);
		strcat(cpath, s);
		if (fopen_s(&stream1, cpath, "w+t") != 0){
			printf("file open fail\n");
			return 0;
		}

		//tbp제목 얻기
		// printf("111111111111\n");

		WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);

		sprintf(s, "%04d-%02d-%02d,%02d%02d%02d",
			t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
			t->tm_hour, t->tm_min, t->tm_sec
			);
		strcat(s, target_arry);
		strcat(s, tbp_title_cpu1);
		strcat(cpath, s);
		if (fopen_s(&stream, cpath, "w+t") != 0){
			printf("file open fail\n");
			return 0;
		}
	}

	char* memblock;
	memblock = (char*)malloc(100 * 256 * 4);
	jtag_read_tb_new_for_lua(mid, (UINT8*)memblock);
	fwrite(memblock, sizeof(char), strlen(memblock), stream);
	fclose(stream);
	free(memblock);

	char *memblock1;
	memblock1 = (char*)malloc(1200);
	jtag_read_cpu_all_register_for_lua(mid, memblock1);
	fwrite(memblock1, sizeof(char), strlen(memblock1), stream1);
	fclose(stream1);
	free(memblock1);

	// TCHAR path[MAX_PATH];
	// GetCurrentDirectory(MAX_PATH, path);
	// char cpath[MAX_PATH];
	//WideCharToMultiByte(CP_ACP, 0, path, MAX_PATH, cpath, MAX_PATH, NULL, NULL);
	// strcat(cpath, "\\");
	//strcat(cpath, s);
	lua_pushstring(L, cpath);
	return 1;
}

_INS int test1(lua_State* L)
{
	return 0;
}


_INS int  profiler_generator(lua_State* L)
{
	char* module = (char*)luaL_checkstring(L, 1);
	char* htb_path = (char*)luaL_checkstring(L, 2);
	char htb_path_profiler[300];
	//char *htb_path_profiler;
	char* dis_path = (char*)luaL_checkstring(L, 3);
	char* profiler_path = (char*)luaL_checkstring(L, 4);
	char cpu0_prf[15] = "(cpu0).prf";
	char cpu1_prf[15] = "(cpu1).prf";
	char profiler_path_arr[200];
	//char *cpu0_prf = "(cpu0).prf";
	//char *cpu1_prf = "(cpu1).prf";

	char total_path[300];
	int mid = 0;
	if (strcmp("cpu0", module) == 0) 			mid = 4;
	if (strcmp("cpu1", module) == 0) 			mid = 5;


#if 1
	strcpy(htb_path_profiler,htb_path);
	strcpy(profiler_path_arr, profiler_path);
	if (mid == 4)
	{

		char *htb_find_path = strrchr(htb_path_profiler, '\\');
		htb_find_path = strtok(htb_find_path + 1, "(");
		strcat(htb_find_path, cpu0_prf); //////////
		strcat(profiler_path_arr, htb_find_path); 

		sprintf(total_path, "profiler.exe -hz 199800000 -dis %s -htb %s -o %s", dis_path, htb_path, profiler_path_arr);
	
		
		system(total_path);

		//profiler_path=strrchr(profiler_path, '\\');
	
		//strtok(profiler_path , profiler_path+1);
	

		lua_pushnumber(L, 1);
	}
	else if (mid == 5)
	{
		char *htb_find_path = strrchr(htb_path_profiler, '\\');
		htb_find_path = strtok(htb_find_path + 1, "(");
		strcat(htb_find_path, cpu1_prf);
		strcat(profiler_path_arr, htb_find_path); 
		sprintf(total_path, "profiler.exe -hz 199800000 -dis %s -htb %s -o %s", dis_path, htb_path, profiler_path_arr);
		
		system(total_path);

		//profiler_path=strrchr(profiler_path, '\\');

		//strtok(profiler_path , profiler_path+1);
		
		lua_pushnumber(L, 1);
	}
#endif

	return 1;
}

_INS int get_device_num(lua_State* L)
{
	/*	int option = luaL_checkint(L, 1);

	UINT32 idcodes[MAX_DEVICES];
	int num_devices;
	UINT32 target = 0;
	UINT32 mode = 0;



	tap_reset();
	tap_get_ir_ID(idcodes, &num_devices);
	printf("IDCODE : %x, DEVICE NUM : %d ", *idcodes, num_devices);
	tap_reset();


	if (option == 1)
	{
	printf("Check Read JTAG ID only\n");

	}
	else if (option == 2)
	{


	}
	else if (option == 3)
	{
	mode = 1;
	target = atoi(argv[2]);//strtoul(argv[2], NULL, 16);
	printf("Targetnumber : %d\n", target);


	}

	*/


	int num_devices;
	UINT32 idcodes[MAX_DEVICES];


	tap_reset();
	tap_get_ir_ID(idcodes, &num_devices);
	printf("IDCODE : %x, DEVICE NUM : %d \n", *idcodes, num_devices);
	tap_reset();

	jtag_set_tap_ir(JTAP_IR_DEBUG, JTAP_IR_BYPASS);


	lua_pushinteger(L, num_devices);


	return 1;
}


_INS int set_target(lua_State* L)

{
	UINT32 target = luaL_checkinteger(L, 1);
	int num_devices = luaL_checknumber(L, 2);


	printf("set target = %d\n", target);
	jtag_set_chain_target(target, num_devices);
	jtag_set_tap_ir(JTAP_IR_DEBUG, JTAP_IR_BYPASS);

	return 0;
}

_INS int jtag_log_save(lua_State* L)
{
#if 1
	char* module = (char*)luaL_checkstring(L, 1);
	char* file_name = (char*)luaL_checkstring(L, 2);
	FILE *stream = NULL;
	UINT32 size = 0;
	UINT32 option = 1;
	int mid = 4;
	TCHAR path[MAX_PATH];
	char cpath[MAX_PATH];
	char file_name_arry[50] = { 0, };
	if (strcmp("isp", module) == 0)				mid = 4;
	else if (strcmp("ip0", module) == 0)		mid = 5;

	strcpy(file_name_arry, file_name);

	GetCurrentDirectory(MAX_PATH, path);
	CString strDirectory = _T("");
	strDirectory.Format(_T("%s\\jtag_print_files\\"), path);
	CreateDirectory(strDirectory, NULL);
	WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);

	strcat(cpath, file_name_arry);
	//printf("cpath=%s\n", cpath);


	if (fopen_s(&stream, cpath, "w+t") != 0){
		printf("file open fail\n");
		return 0;
	}
	
	exr_set_jtag_print(mid);
	exr_resetpos();
	

	UINT8* memblock = (UINT8*)malloc(jtag_print_size);

	size = JTAGPrint_(memblock, option, mid);

	fwrite(memblock, sizeof(char), size, stream);


	fclose(stream);
	free(memblock);
	
	//lua_pushnumber(L, size);
#endif

#if 0
	char* module = (char*)luaL_checkstring(L, 1);
	char* file_name = (char*)luaL_checkstring(L, 2);

	FILE *stream = NULL;
	UINT32 size = 0;
	UINT32 option = 1;
	int mid = 4;
	if (strcmp("isp", module) == 0)				mid = 4;
	else if (strcmp("ip0", module) == 0)		mid = 5;

	if (fopen_s(&stream, file_name, "w+t") != 0){
		printf("file open fail\n");
		return 0;
	}

	exr_set_jtag_print(mid);
	exr_resetpos();

	UINT8* memblock = (UINT8*)malloc(jtag_print_size);
	
	size = JTAGPrint_(memblock, option, mid);

	fwrite(memblock, sizeof(char), size, stream);



	fclose(stream);
	free(memblock);
	
#endif


	return 0;
}


_INS int exr_lua_init()
{
	// Init Lua library
	pLua = luaL_newstate();
	luaL_openlibs(pLua);
	luaL_dostring(pLua, "print('Lua Version: '.._VERSION)");
	lua_register(pLua, "state", exr_lua_state);
	lua_register(pLua, "debug_file_single_jtag",debug_file_one_jtag);
	lua_register(pLua, "debug_file_multi_jtag", debug_file_multi_jtag);
	lua_register(pLua, "test1", test1);
	// Register Lua glue functions
	lua_register(pLua, "cmd", exr_lua_cmd);
	lua_register(pLua, "profiler_gen", profiler_generator);
	lua_register(pLua, "read_register", exr_lua_read_reg);
	lua_register(pLua, "write_register", exr_lua_write_reg);
	lua_register(pLua,"jtag_log_save",jtag_log_save);
	lua_register(pLua, "get_device_num", get_device_num);
	lua_register(pLua, "set_target", set_target);
	//lua_register(pLua, "cpu_debug_register", exr_cpu_debug_register);
	//lua_register(pLua, "read_memory", exr_lua_read_mem);
	//lua_register(pLua, "write_memory", exr_lua_write_mem);

	return 0;
}

_INS void exr_lua_close()
{
	lua_close(pLua);
}

_INS void exr_lua_run(char* fname)
{
	// Run Lua script
	if (fname == NULL){
		luaL_dofile(pLua, "cmd.lua");
		lua_flag = 0;
	}else{
		luaL_dofile(pLua, fname);
		
		lua_flag = 0;
	}
}


char* timeToString(struct tm *t) 
{
	static char s[30] = { 0, };

	sprintf(s, "%04d-%02d-%02d,%02d:%02d:%02d",
		t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
		t->tm_hour, t->tm_min, t->tm_sec
		);

	return s;
}

int jtag_read_cpu_all_register_for_lua(UINT8 cpu,char * buffer)
{
	UINT32 err = 0;
	UINT32 npc, ppc, sr, esr, EPCR, EEAR, DMR1, DSR, DRR, i;
	UINT32 IWMR, IWSR, IWDTR, IWAR;
	UINT32 DWMR, DWSR, DWDTR, DWAR;
	UINT32 tbcr;
	UINT32 r[32];

	err |= jtag_read32(SPR_BIT + SYS_GID + (16 << 2), &npc, cpu);  /* Read NPC */
	err |= jtag_read32(SPR_BIT + SYS_GID + (17 << 2), &sr, cpu);  /* Read SR  */
	err |= jtag_read32(SPR_BIT + SYS_GID + (18 << 2), &ppc, cpu);  /* Read PPC */


	err |= jtag_read32(SPR_BIT + SYS_GID + (32 << 2), &EPCR, cpu);  /* Read EPCR */
	err |= jtag_read32(SPR_BIT + SYS_GID + (64 << 2), &esr, cpu);  /* Read ESR */
	err |= jtag_read32(SPR_BIT + SYS_GID + (48 << 2), &EEAR, cpu);  /* Read EEAR */


	err |= jtag_read32(SPR_BIT + DU_GID + (16 << 2), &DMR1, cpu);  /* Read DMR1 */
	err |= jtag_read32(SPR_BIT + DU_GID + (20 << 2), &DSR, cpu);  /* Read DSR */
	err |= jtag_read32(SPR_BIT + DU_GID + (21 << 2), &DRR, cpu);  /* Read DRR */


	err |= jtag_read32(SPR_BIT + IWB_GID + (16 << 2), &IWMR, cpu);  /* Read IWMR */
	err |= jtag_read32(SPR_BIT + IWB_GID + (17 << 2), &IWSR, cpu);  /* Read IWSR */
	err |= jtag_read32(SPR_BIT + IWB_GID + (18 << 2), &IWDTR, cpu);  /* Read IWDTR */
	err |= jtag_read32(SPR_BIT + IWB_GID + (19 << 2), &IWAR, cpu);  /* Read IWAR */


	err |= jtag_read32(SPR_BIT + DWB_GID + (16 << 2), &DWMR, cpu);  /* Read DWMR */
	err |= jtag_read32(SPR_BIT + DWB_GID + (17 << 2), &DWSR, cpu);  /* Read DWSR */
	err |= jtag_read32(SPR_BIT + DWB_GID + (18 << 2), &DWDTR, cpu);  /* Read DWDTR */
	err |= jtag_read32(SPR_BIT + DWB_GID + (19 << 2), &DWAR, cpu);  /* Read DWAR */

	err |= jtag_read32(SPR_BIT + TB_GID + TBCR, &tbcr, cpu);  /* Read TBCR */
	/*
	printf("Status:     NPC  = %.8x  SR   = %.8x PPC   = %.8x \n", npc & 0xfffffffc, sr, ppc & 0xfffffffc);
	printf("Exception:  EPCR = %.8x  ESR  = %.8x EEAR  = %.8x \n", EPCR & 0xfffffffc, esr, EEAR);
	printf("Debug:      DMR1 = %.8x  DSR  = %.8x DRR   = %.8x \n", DMR1, DSR, DRR);
	printf("IHWP:       IWMR = %.8x  IWSR = %.8x IWDTR = %.8x IWAR = %.8x\n", IWMR, IWSR, IWDTR, IWAR);
	printf("DHWP:       DWMR = %.8x  DWSR = %.8x DWDTR = %.8x DWAR = %.8x\n", DWMR, DWSR, DWDTR, DWAR);
	printf("TB:         TBCR = %.8x\n", tbcr);
	printf("GPR:\n");
	for (i = 0; i < 32; i++){
		err |= jtag_read32(DRF_BIT + (0x400 << 2) + (i << 2), &r[i], cpu);
		printf("\t r%.2d = %.8x \n", i, r[i]);
	}
	*/

	sprintf(buffer, "Status:     NPC  = %.8x  SR   = %.8x PPC   = %.8x \nException:  EPCR = %.8x  ESR = %.8x EEAR = %.8x \nDebug:      DMR1 = %.8x  DSR = %.8x DRR = %.8x \nIHWP: IWMR = %.8x  IWSR = %.8x IWDTR = %.8x IWAR = %.8x\n DHWP:       DWMR = %.8x  DWSR = %.8x DWDTR = %.8x DWAR = %.8x\nTB:         TBCR = %.8x\nGPR:\n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n\t r%.2d = %.8x \n",
		npc & 0xfffffffc, sr, ppc & 0xfffffffc, EPCR & 0xfffffffc, esr, EEAR, DMR1, DSR, DRR, IWMR, IWSR, IWDTR, IWAR, DWMR, DWSR, DWDTR, DWAR, tbcr, 0,r[0],1, r[1], 2, r[2], 3, r[3], 4, r[4], 5, r[5], 6, r[6], 7, r[7], 8, r[8], 9, r[9], 10, r[10], 11, r[11], 12, r[12], 13, r[13], 14, r[14], 15, r[15], 16, r[16], 17, r[17], 18, r[18], 19, r[19], 20, r[20], 21, r[21], 22, r[22], 23, r[23], 24, r[24], 25, r[25], 26, r[26], 27, r[27], 28, r[28], 29, r[29], 30, r[30], 31, r[31]);
	
	
	return err;
}

UINT32 jtag_read_tb_new_for_lua(UINT8 cpu, UINT8* buffer)
{
	UINT32 err = 0;
	UINT32 i, j = 0;
	UINT32 thbuf[TB_LEN * 4 + 1];

	UINT32 tb_adr, tb_pc, tb_insn, tb_rout, tb_ts;
	UINT32 frz_cnt = 0;
	UINT32 tb_cr;

	jtag_read32(SPR_BIT + TB_GID + (7 << 10), &tb_cr, cpu);

	for (i = 0; i < TB_LEN * 4 + 1; i++){
		err |= jtag_read32(DTB_BIT + (0x0ff << 2) + (i << 2), &thbuf[i], cpu);
		//	printf("Adr:%.8x = %.8x \n", DTB_BIT + 0x0ff + i, thbuf[i]);
#if DUMP
		j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "@Ad>%.8x: %.8x\n", DTB_BIT + 0x0ff + i, thbuf[i]);
#endif
	}

	tb_adr = thbuf[0];
	j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "tbcr : %08x\n", tb_cr);
	//printf("tbcr : % 08x\n", tb_cr);

	j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "tbar : %02x\n", tb_adr);
	//printf("tbar : %02x\n", tb_adr);

	for (i = 0; i < TB_LEN; i++){
		tb_pc = thbuf[1 + TB_LEN * 0 + i];
		tb_insn = thbuf[1 + TB_LEN * 1 + i];
		tb_rout = thbuf[1 + TB_LEN * 2 + i];
		tb_ts = thbuf[1 + TB_LEN * 3 + i];
		j += sprintf_s((char*)buffer + j, 100 * 256 * 4, "%02x %.8x %.8x %.8x %.8x \n", i, tb_pc, tb_insn, tb_rout, tb_ts);
	//	printf("%02x %.8x %.8x %.8x %.8x \n", i, tb_pc, tb_insn, tb_rout, tb_ts);
	}

	return err;
}