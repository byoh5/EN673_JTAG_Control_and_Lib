
#include "stdafx.h"

#include "WaveformImgOperation.h"




#include "EN673JTAG.h"
#include "jtag_debug.h"
#include "jtag_module.h"
#include "jtag_base.h"
#include "jtag_tap.h"
#include "shellcmd.h"
#include "jtag_example.h"
#include <stdio.h>
#include <iostream>
#include <io.h>
#include <fcntl.h>
#include <sys\types.h>
#include <sys\stat.h>
#include <malloc.h>
#include "jtag_rw.h"
#include "jtag_flash.h"
#include "spr_defs.h"
#include <fstream>
#include "jtag_chain.h"
#include "jtag_def.h"
#include <time.h>
#include "exr_lua.h"
#include "remoteControl.h"



#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"


using namespace cv;

#pragma warning(disable:4996)

#define ENX_REMOTE_PRINT
#ifdef ENX_REMOTE_PRINT
#define rprintf(fmt, ...) do { if(RemoteLog_flag) printf(fmt,##__VA_ARGS__); } while(0);
#else
#define rprintf(fmt, ...) do {} while(0);
#endif

/////////////////////////////////////////////////////////////////////////////////

UINT32 _DoCommand(char *cmdline);
void hold_loop(void);
void TAP_Reset(void);
DWORD WINAPI run_lua_script(void *arg);
UINT32 rem_read_mem_(char* cmdline);
UINT32 rem_write_mem_(char* cmdline);
UINT32 SendImage_(char* cmdline);
UINT32 KillSendImage_(char* cmdline);
UINT32 rem_imagecheck(char* cmdline);
UINT32 rem_imageflag(char* cmdline);
UINT32 rem_reset(char* cmdline);
UINT32 rem_reset_cpu(char* cmdline);
UINT32 rem_unreset_cpu(char* cmdline);
UINT32 rem_hold(char* cmdline);
UINT32 rem_release(char* cmdline);
UINT32 rem_stall(char* cmdline);
UINT32 rem_unstall(char* cmdline);
UINT32 rem_checkstall(char* cmdline);
UINT32 rem_memoryset(char* cmdline);
UINT32 rem_ftdx_flush(char* cmdline);
UINT32 rem_lua_read(char* cmdline);
UINT32 rem_lua_scrstate(char *cmdline);
UINT32 rem_lua_script_stop(char* cmdline);
UINT32 rem_tbp(char* cmdline);

UINT32 rem_check(char* cmdline);
int sendMsg(int fd_cli, char* msg);
int image_flag = 0;
int script_state = 0;
int lua_flag = 0;

TCLI_CMD cli_cmds[] =
{
	{ "rem_mr",			rem_read_mem_,			"memory read"				},
	{ "rem_mw",			rem_write_mem_,			"memory wrtie"				},
	{ "sendimg",		SendImage_,				"Send Imgae"				},
	{ "killsendimg",	KillSendImage_,			"Kill Send Imgae"			},
	{ "rem_imagecheck", rem_imagecheck,			"Send Image Check"			},
	{ "rem_imageflag",	rem_imageflag,			"Image flag set"			},
	{ "rem_rst",		rem_reset,				"Reset"						},
	{ "rem_rc",			rem_reset_cpu,			"Reset cpu"					},
	{ "rem_urc",		rem_unreset_cpu,		"Unrset cpu"				},
	{ "rem_hold",		rem_hold,				"Hold JTAG"					},
	{ "rem_release",	rem_release,			"Release JTAG"				},
	{ "rem_st",			rem_stall,				"Stall"						},
	{ "rem_ust",		rem_unstall,			"Unstall"					},
	{ "rem_cst",		rem_checkstall,			"Check stall Cpu"			},
	{ "rem_set",		rem_memoryset,			"set npc address"			},
	{ "rem_ff",			rem_ftdx_flush,			"ftdx_flush()"				},
	{ "rem_lua",        rem_lua_read,			"play lua file"				},
	{ "rem_script_state", rem_lua_scrstate,		"check lua script state"	},
	{ "rem_script_stop", rem_lua_script_stop,	"stop lua script"			},
	{ "rem_check",		rem_check,				"JTAG State Check"			},
	{ "rem_tbp",		rem_tbp,				"tbp"						},
	{ 0, 0, 0 }
};


//#define SHOW_IMG	//for debug image show

//////////////////////////////////////////////////////////////////////////////////

void sendMsg_(int fd_cli, char* msg)
{
	UINT sendmsgsize;
	sendmsgsize = strlen(msg);
	send(fd_cli, msg, sendmsgsize, 0);
}

void hexDump_(char *desc, void *addr, int len) {
	int i;
	unsigned char buff[17];
	unsigned char *pc = (unsigned char*)addr;

	// Output description if given.
	if (desc != NULL)
		rprintf("%s:\n", desc);

	// Process every byte in the data.
	for (i = 0; i < len; i++) {
		// Multiple of 16 means new line (with line offset).

		if ((i % 16) == 0) {
			// Just don't print ASCII for the zeroth line.
			if (i != 0)
				rprintf("  %s\n", buff);

			// Output the offset.
			rprintf("  %04x ", i);
		}

		// Now the hex code for the specific character.
		rprintf(" %02x", pc[i]);

		// And store a printable ASCII character for later.
		if ((pc[i] < 0x20) || (pc[i] > 0x7e))
			buff[i % 16] = '.';
		else
			buff[i % 16] = pc[i];
		buff[(i % 16) + 1] = '\0';
	}

	// Pad out last line if not exactly 16 characters.
	while ((i % 16) != 0) {
		rprintf("   ");
		i++;
	}

	// And print the final ASCII bit.
	rprintf("  %s\n", buff);
}

UINT32 cli_stdin_request_(char* cmdline)
{
//	DEBUG("cli_stdin_request: \"%s\"\n", cmdline);

	// parse a command
	char cmd_buf[CLI_CMD_LEN];
	static char cmdline_old[CLI_CMD_LEN];


	if (cmdline[0] == '_')	cmdline++;
	if (cmdline[0] == '!')	strcpy(cmdline, cmdline_old);

	strcpy(cmd_buf, cmdline);

	if (cmdline[0] != '!')	strcpy(cmdline_old, cmdline);

	// run a command
	TCLI_CMD* pCmd = cli_cmds;
	UINT err = -1;
	for (; pCmd->name; pCmd++) {
		if (!strncmp(pCmd->name, cmdline, strlen(pCmd->name))) {
			if (strstr(cmdline, "-h"))
				err = pCmd->func("-h");
			else
				err = pCmd->func(cmdline);
			break;
		}
	}
	return err;
}



DWORD WINAPI threadReadMem_(void *arg)
{
	rprintf("Start Thread Read Memory\n");
	UINT size;
	UINT addr;
	char fname[128];
	char *cmdline = (char*)arg;
	UINT argc;
	UINT module = 0;
	int cli_fd;
	if ((argc = sscanf(cmdline, "rem_mr %x %d %d %s %d ", &addr, &size, &module, fname, &cli_fd)) == 5) {
		rprintf("cli_fd %d\n", cli_fd);
	}
	else if ((argc = sscanf(cmdline, "rem_mr %x %d %d %d ", &addr, &size, &module, &cli_fd)) == 4) {
		strcpy(fname, "mem.bin");
	}
	else {
		rprintf("%d %x %d\n ", argc, addr, size);
		rprintf("Start Thread Read Memory\n");
		rprintf("\"%s\" not recognized\n", cmdline);
		return -1;
	}

	// Process arguments	
	if (addr&~ADDR_MASK) {	// a multiple of 4
		rprintf("address 0x%0x is not a multiple of 4!\n", addr);
		TAP_Reset();
		return -1;
	}
	UINT size4 = (size + 3) & ADDR_MASK;

	// read from memory
	char* rbuf = (char*)malloc(size4 + 4);	// big
	char* wbuf = (char*)malloc(size4 + 4);	// little

	while (jtag_gdb_burst_read32((UINT*)rbuf, size4 / 4, addr, module)){ TAP_Reset();}

	// change endianness from big to little
	UINT* rbuf32 = (UINT*)rbuf;
	UINT* wbuf32 = (UINT*)wbuf;
	int i;
	for (i = 0; i<size4 / 4; i++, rbuf32++, wbuf32++)
		*wbuf32 = htonl(*rbuf32);


	char sendmsg[128];
	UINT sendmsgsize;
	int a;
	int slen;
	sprintf(sendmsg, "size%d \n", size);
	sendmsgsize = strlen(sendmsg);

	send(cli_fd, ((char*)sendmsg), sendmsgsize, 0);
	slen = 0;

	UINT pos = 0;

	for (a = size; a>1024;){
		slen = send(cli_fd, wbuf + pos, 1024, 0);

		if (slen == -1){
			Sleep(10);
			rprintf("slen = %d\n", slen);
			continue;
		}
		else{
			//rprintf("slen = %d\n",slen);
			a -= slen;
			pos += slen;
		}
	}
	if (a>0){
		slen = send(cli_fd, wbuf + pos, a, 0);
		rprintf("slen = %d\n", a);
	}
	rprintf("sending..\n");


	if (rbuf) free(rbuf);
	if (wbuf) free(wbuf);

	return 0;
}

HANDLE Thread_SendImage_ = NULL;
UINT32 rem_read_mem_(char* cmdline)
{
	DWORD dwThreadID0;
	rprintf("%s", cmdline);
	Thread_SendImage_ = CreateThread(NULL, 0, threadReadMem_, (LPVOID)cmdline, 0, &dwThreadID0);
	return 0;
}


UINT32 rem_write_mem_(char* cmdline)
{
	UINT size;
	UINT addr;
	UINT module;
	SOCKET cli_fd;
	
	if (sscanf(cmdline, "rem_mw %x %d %d %d ", &addr, &size, &module, &cli_fd) == 4) {
		;
	}
	else{
		rprintf("argument err \n");
		return -1;
	}

	sendMsg_(cli_fd, "OK ");
	//Read from Client 

	UINT size4 = (size + 3) & ADDR_MASK;

	char* buf = (char*)malloc(size4);	// little
	char* wbuf = (char*)malloc(size4);	// big
	UINT iResult;
	UINT32 a;
	buf = (char*)malloc(size4);
	for (a = 0; a < size;){
		iResult = recv(cli_fd, buf + a, size - a, 0);
		if (iResult < 0){
			rprintf("Error\n");
		}
		a += iResult;
		rprintf("recv %d\n", iResult);
	}

//	hexDump_("READ", buf, 1024);

	// change the endianness from little to big
	UINT* rbuf32 = (UINT*)buf;
	UINT* wbuf32 = (UINT*)wbuf;
	int i;
	for (i = 0; i<size4 / 4; i++, rbuf32++, wbuf32++)
		*wbuf32 = htonl(*rbuf32);

	// write to memory
	while (jtag_gdb_burst_write32((UINT*)wbuf, size4 / 4, addr, module)){ TAP_Reset();}

	sendMsg_(cli_fd, "OK ");

	if (buf)  free(buf);
	if (wbuf) free(wbuf);

	return 0;
}

UINT32 loopflagbit = 0;

UINT32 EscapeLoop_(UINT32 val){
	//	rprintf("Loop Flag return to %d\n", loopflag);
	return (loopflagbit & val);
}
void EscapeLoopSet_(UINT32 val){
	EnterCriticalSection(&CriticalSection);
	loopflagbit |= val;
	LeaveCriticalSection(&CriticalSection);
}
void EscapeLoopUnSet_(UINT32 val){
	EnterCriticalSection(&CriticalSection);
	loopflagbit &= ~val;
	LeaveCriticalSection(&CriticalSection);
}

void TAP_Reset(void)
{
	EnterCriticalSection(&CriticalSection);
	tap_reset();
	tap_set_ir_debug();
	LeaveCriticalSection(&CriticalSection);
}


#define JPEG_READ_SIZE 1024
int image_cli_fd;
DWORD WINAPI ThreadImageSendFunc_(void* arg){
	EscapeLoopUnSet_(eESC_ImageSend);
	char *cmdline = (char*)arg;
	int cli_fd;
	if (sscanf(cmdline, "sendimg %d ",&cli_fd) == 1) {
		rprintf("cli_fd %d\n", cli_fd);
	}
	else{
		rprintf("Error sendimg arg err!\n");
		return -1;
	}
	image_cli_fd = cli_fd;
	image_flag = 1;

	UINT32 addr;
	UINT32 addrview = JPEG_VIEW_ADDR;
	UINT32 addrsizep = JTAG_VIEW_ADDRSIZEP;
	UINT32 addrflag = JTAG_VIEW_FLAG;
	UINT32 size = 0;
	UINT32 module = 3;
	UINT32 temp = 0;
	UINT32 err = 0;

	TAP_Reset();

	temp = FLAG_INTRO;
	while (jtag_write_block32(addrflag, &temp, 1, JTAG_COMMON_MODULE_IDX)) { TAP_Reset(); Sleep(1); } // SET Flag to 9 for JTAG VIEW MODE
	temp = SIZE_ZERO;
	while (jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX)) { TAP_Reset(); Sleep(1); }


	while (1){
		hold_loop();

		if (EscapeLoop_(eESC_ImageSend)) break;

		TAP_Reset();

		temp = FLAG_INTRO;
		while (jtag_write_block32(addrflag, &temp, 1, JTAG_COMMON_MODULE_IDX)) { TAP_Reset(); Sleep(1); }
		temp = SIZE_ZERO;
		while (jtag_read_block32(addrsizep, &size, 1, JTAG_COMMON_MODULE_IDX)) { TAP_Reset(); Sleep(1); }

	
		rprintf("size:%d\n", size);
		if (size == NULL)
		{
			Sleep(1);
			continue;
		}
		if (size > MAX_JPEG_SIZE){
			rprintf("wrong size %d", size);
			while (jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX)) { TAP_Reset(); Sleep(1); }
			//cvReleaseMat(&matJpg2);
			Sleep(1);
			continue;
		}

		while (jtag_read_block32(addrview, &addr, 1, JTAG_COMMON_MODULE_IDX)) { TAP_Reset(); Sleep(1); }
//		rprintf("Addr %x\n", addr);
		UINT size4 = (size + 3) & ADDR_MASK;

		// read from memory
		char* rbuf = (char*)malloc(size4 + 4);	// big
		char* wbuf = (char*)malloc(size4 + 4);	// little

#if 0
		if (jtag_gdb_burst_read32((UINT*)rbuf, size4 / 4, addr, module)) {continue;}
#else	
		UINT32 retry = (size4) / (JPEG_READ_SIZE);
		UINT32 left = (size4) % (JPEG_READ_SIZE);
		int state = 0;
		int j = 0;
		for (j = 0; j < retry; j++)
		{
			if (jtag_gdb_burst_read32((uint*)rbuf + (JPEG_READ_SIZE * j) / 4, JPEG_READ_SIZE / 4, addr + (JPEG_READ_SIZE * j), module))
			{
				continue;
			}
		}
		if (left)
		{
			if (jtag_gdb_burst_read32((uint*)rbuf + (JPEG_READ_SIZE * j) / 4, left / 4, addr + (JPEG_READ_SIZE * j), module))
			{
				continue;
			}
		}
#endif

		//		rprintf("Read data\n");
		temp = SIZE_ZERO;
		while (jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX)) { TAP_Reset(); Sleep(1); }

		// change endianness from big to little
		UINT* rbuf32 = (UINT*)rbuf;
		UINT* wbuf32 = (UINT*)wbuf;
		int i;
		for (i = 0; i<size4 / 4; i++, rbuf32++, wbuf32++)
			*wbuf32 = htonl(*rbuf32);
	
		char sendmsg[128];
		sprintf(sendmsg, "size%d ", size);
		send(cli_fd, (char*)sendmsg, strlen(sendmsg), 0);
		Sleep(10);
		send(cli_fd, (char*)wbuf, size, 0);

#ifdef SHOW_IMG
		Mat matJpg2(1, 1024*1024, CV_8UC1);
		Mat src;
		memcpy(matJpg2.data, wbuf, size);
		//rprintf("size: %d\n", a);
		src = imdecode(matJpg2, 1);
		imshow("JPEG", src);
		waitKey(1);
#endif
		rprintf("sending..\n");
		Sleep(10);
		if (rbuf) free(rbuf);
		if (wbuf) free(wbuf);

	}

	temp = FLAG_OUTRO;
	while (jtag_write_block32(addrflag, &temp, 1, JTAG_COMMON_MODULE_IDX)){ TAP_Reset(); Sleep(1); }

	EscapeLoopUnSet_(eESC_ImageSend);

#ifdef SHOW_IMG
	destroyAllWindows();
#endif
	image_flag = 0;
	return TRUE;
}

UINT32 SendImage_(char* cmdline)
{
	DWORD dwThreadID0;

	Thread_SendImage_ = CreateThread(NULL, 0, ThreadImageSendFunc_, (LPVOID)cmdline, 0, &dwThreadID0);
	return 0;
}

UINT32 KillSendImage_(char* cmdline)
{
	EscapeLoopSet_(eESC_ImageSend);
	rprintf("Kill\n");
	image_flag = 0;
	return 0;
}

UINT32 rem_imagecheck(char* cmdline)
{
	int cli_fd;
	if (sscanf(cmdline, "rem_imagecheck %d ", &cli_fd) == 1) {
		//rprintf("cli_fd %d\n", cli_fd);
	}

	char sendmsg[10];
	sprintf(sendmsg, "%d ", image_flag);
	send(cli_fd, ((char*)sendmsg), 1, 0);

	return 0;
}


UINT32 rem_imageflag(char* cmdline)
{
	int cli_fd;
	int flag = 0;
	if (sscanf(cmdline, "rem_imagecheck %d %d ", &flag, &cli_fd) == 2) {
		//rprintf("cli_fd %d\n", cli_fd);
	}


	char cmd[100];
	sprintf(cmd, "sendimg %d", cli_fd);
	if (image_flag != flag)
	{
		if (flag)
		{
			SendImage_((char*)cmd);
		}
		else
		{
			KillSendImage_(NULL);
			closesocket(image_cli_fd);
			Sleep(1000);
		}
		
	}

	return 0;
}

UINT32 rem_reset(char* cmdline)
{
	_DoCommand("rst");
	return 0;
}

UINT32 rem_reset_cpu(char* cmdline)
{
	char module[10];
	int cli_fd;
	if (sscanf(cmdline, "rem_rc %s %d ", &module, &cli_fd) == 2) {
		if (strcmp("cpu0", module) == 0)
			_DoCommand("rc cpu0");
		else if (strcmp("cpu1", module) == 0)
			_DoCommand("rc cpu1");
		else
		{
			rprintf("Error sendimg arg err!\n");
			return -1;
		}

	}
	else if (sscanf(cmdline, "rem_rc %d ", &cli_fd) == 1) {
		_DoCommand("rc");
	}
	else
	{
		rprintf("Error sendimg arg err!\n");
		return -1;
	}

	return 0;
}

UINT32 rem_unreset_cpu(char* cmdline)
{
	char module[10];
	int cli_fd;
	if (sscanf(cmdline, "rem_urc %s %d ", &module, &cli_fd) == 2) {
		if (strcmp("cpu0", module) == 0)
			_DoCommand("urc cpu0");
		else if (strcmp("cpu1", module) == 0)
			_DoCommand("urc cpu1");
		else
		{
			rprintf("Error sendimg arg err!\n");
			return -1;
		}

	}
	else if (sscanf(cmdline, "rem_urc %d ", &cli_fd) == 1) {
		_DoCommand("urc");
	}
	else
	{
		rprintf("Error sendimg arg err!\n");
		return -1;
	}

	return 0;
}

UINT32 rem_stall(char* cmdline)
{
	//_DoCommand("st");
	char module[10];
	int cli_fd;
	if (sscanf(cmdline, "rem_st %s %d ", &module, &cli_fd) == 2) {
		if (strcmp("cpu0", module) == 0)
			_DoCommand("st cpu0");
		else if (strcmp("cpu1", module) == 0)
			_DoCommand("st cpu1");
		else
		{
			rprintf("Error sendimg arg err!\n");
			return -1;
		}

	}
	else if (sscanf(cmdline, "rem_st %d ", &cli_fd) == 1) {
		_DoCommand("st");
	}
	else
	{
		rprintf("Error sendimg arg err!\n");
		return -1;
	}


#if 0
	UINT8 stalled = 0;
	UINT8 ncpu;
	for (ncpu = 4; ncpu < 6; ncpu++){
		jtag_stall_cpu(ncpu);
		jtag_check_stalled_cpu(ncpu, &stalled);
		if (stalled){
			rprintf("Success : now stalled\n");
		}
		else{
			rprintf("Fail : now unstalled\n");
		}
	}
#endif
	return 0;
}

UINT32 rem_unstall(char* cmdline)
{
	char module[10];
	int cli_fd;
	if (sscanf(cmdline, "rem_ust %s %d ", &module, &cli_fd) == 2) {
		if (strcmp("cpu0", module) == 0)
			_DoCommand("ust cpu0");
		else if (strcmp("cpu1", module) == 0)
			_DoCommand("ust cpu1");
		else
		{
			rprintf("Error sendimg arg err!\n");
			return -1;
		}

	}
	else if (sscanf(cmdline, "rem_ust %d ", &cli_fd) == 1) {
		_DoCommand("ust");
	}
	else
	{
		rprintf("Error sendimg arg err!\n");
		return -1;
	}
	return 0;
}

UINT8 checkstall(UINT8 cpu)
{
	UINT32 err = 0;
	EnterCriticalSection(&CriticalSection);
	UINT8 stalled = 0;

	err |= jtag_check_stalled_cpu(cpu, &stalled);
	LeaveCriticalSection(&CriticalSection);

	return stalled;
}

UINT32 rem_checkstall(char* cmdline)
{
	char module[10];
	int cli_fd;
	UINT8 stalled = 0;
	if (sscanf(cmdline, "rem_cst %s %d ", &module, &cli_fd) == 2) {
		if (strcmp("cpu0", module) == 0)
			stalled = checkstall(4);
		else if (strcmp("cpu1", module) == 0)
			stalled = checkstall(5);
		else
		{
			rprintf("Error sendimg arg err!\n");
			return -1;
		}
	}
	else
	{
		rprintf("Error sendimg arg err!\n");
		return -1;
	}

	char sendmsg[10];
	sprintf(sendmsg, "%d ", stalled);
	send(cli_fd, ((char*)sendmsg), 1, 0);

	return 0;
}

UINT32 rem_memoryset(char* cmdline)
{
	char module[10];
	UINT32 addr = 0;
	int cli_fd;
	char cmd[48];
	if (sscanf(cmdline, "rem_set %x %s %d ", &addr, &module, &cli_fd) == 3) {
		sprintf(cmd, "set npc %x %s ", addr, &module);
		if (strcmp("cpu0", module) == 0)
		{
			_DoCommand(cmd);
		}
		else if (strcmp("cpu1", module) == 0)
		{
			_DoCommand(cmd);
		}
		else
		{
			rprintf("Error sendimg arg err!\n");
			return -1;
		}

	}
	else if (sscanf(cmdline, "rem_set %x %d ", &addr, &cli_fd) == 2) {
		sprintf(cmd, "set npc %x ", addr);
		_DoCommand(cmd);
	}
	else
	{
		rprintf("Error sendimg arg err!\n");
		return -1;
	}
	return 0;
}

UINT32 rem_ftdx_flush(char* cmdline)
{
	UINT32 err = 0;
	err |= ftdx_flush();
	return err;
}

UINT32 hold_flag = 0;
UINT32 state_flag = 0;
UINT32 state_count1 = 0;
UINT32 state_count2 = 0;
UINT32 state_count3 = 0;
UINT32 state_count4 = 0;
UINT32 state_count5 = 0;
UINT32 rem_check(char* cmdline)
{
	UINT32 err = 0;
	UINT32 reg = 0;

	int cli_fd;
	if (sscanf(cmdline, "rem_check %d ", &cli_fd) == 1) {
		//rprintf("cli_fd %d\n", cli_fd);
	}

	//EnterCriticalSection(&CriticalSection);
	err |= example_read_register(WBCOM, 0xf950002c, &reg);
	ERR_State |= err;

	//EnterCriticalSection(&CriticalSection);
	if (reg != 0x672a0000)
	{
		state_flag = 1;
		hold_flag = 1;
		//rprintf("errA %x\n", err);
		if ((ERR_State & ERR_MPSSE_READ_WRITE_TDI) || (ERR_State & ERR_MPSSE_TDI_TDO_BIT))		// USB connection error
		{
			state_count1++;
			server_state_flag = 1;
			_DoCommand("init all");
		}
		else if (ERR_State == ERR_MPSSE_BURST_READ_STREAM)										// Cable connection error
		{
			state_count2++;
			server_state_flag = 2;
			TAP_Reset();
		}
		else if (ERR_State == ERR_MPSSE_CRC)													// CRC error
		{
			state_count3++;
			server_state_flag = 3;
		}
		else if (ERR_State == ERR_MPSSE_BURST_WRITE_STREAM)										// No Answer!
		{
			state_count4++;
			server_state_flag = 4;
		}
		else																					// ETC..
		{
			state_count5++;
			server_state_flag = 5;
		}
		rprintf("ERR_state = %0.8x\t reg = %0.8x\t flag = %d\t Count = %d %d %d %d %d\n", ERR_State, reg, server_state_flag, state_count1, state_count2, state_count3, state_count4, state_count5);
	}
	else{
		server_state_flag = 0;
		if (state_flag)
		{
			hold_flag = 0;
			state_flag = 0;
		}
		ERR_State = 0;
	}
	//LeaveCriticalSection(&CriticalSection);
	//rprintf("ERR_state = %0.8x\t reg = %0.8x\t flag = %d\t Count = %d %d %d %d %d\n", ERR_State, reg, server_state_flag, state_count1, state_count2, state_count3, state_count4, state_count5);

	char sendmsg[10];
	sprintf(sendmsg, "%d ", server_state_flag);
	send(cli_fd, ((char*)sendmsg), 1, 0);
	
	return 0;
}

void hold_loop(void)
{
	while (hold_flag){
		Sleep(10);
	};
}

UINT32 rem_hold(char* cmdline)
{
	EnterCriticalSection(&CriticalSection);
	hold_flag = 1;
	LeaveCriticalSection(&CriticalSection);
	return 0;
}

UINT32 rem_release(char* cmdline)
{	
	EnterCriticalSection(&CriticalSection);
	hold_flag = 0;
	LeaveCriticalSection(&CriticalSection);
	return 0;
}

int sendMsg(int fd_cli, char* msg)
{
	Sleep(10);
	uint sendmsgsize;
	sendmsgsize = strlen(msg);
	return send(fd_cli, msg, sendmsgsize, 0);
}

uint rem_lua_read(char* cmdline)
{
	const char* help = "\tUsage: command for lua script";

	// Interpret cmdline
	uint argc;
	int size;
	int cli_fd;
	int retval;

	char dir_path[50] = "test";
	//char path[MAX_PATH] = { 0, };
	char file_name[40] = { 0, };
	char state[10] = { 0, };
	char lua_buf_flag[5] = { 0, };
	char file_size[15] = { 0, };
	char *file_content;

	if ((argc = sscanf(cmdline, "rem_lua %d ", &cli_fd)) == 1) {
		;
	}
	lua_socket = cli_fd;

	while (1)
	{
		retval = recv(cli_fd, file_name, sizeof(file_name), 0);
		if (retval > 0)
		{
			recv(cli_fd, file_size, sizeof(file_size), 0);
			if (file_size[0] == 's'&&file_size[1] == 'i'&&file_size[2] == 'z' && file_size[3] == 'e')
			{
				size = atoi(file_size + 5);
			}
			if (size > 0)
			{

				file_content = (char*)malloc(size+1);
				UINT a;
				for (a = 0; a < size;)
				{
					retval = recv(cli_fd, file_content + a, size - a, 0);
					if (retval < 0){
						rprintf("Error~\n");
					}
					a += retval;
				}	
				file_content[a] = '\0';
				if (a == size)
					break;
			}

		}

	}
	
	TCHAR path[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, path);
	CString strDirectory = _T("");
	strDirectory.Format(_T("%s\\lua_files"), path);
	CreateDirectory(strDirectory, NULL);
	char cpath[MAX_PATH];
	WideCharToMultiByte(CP_ACP, 0, strDirectory, MAX_PATH, cpath, MAX_PATH, NULL, NULL);
	strcat(cpath,"\\");
	strcat(cpath, file_name);

	FILE *fp = fopen(cpath, "wb+");
	
	fwrite(file_content, 1, size, fp);

	fclose(fp);

	free(file_content);

	CreateThread(NULL, 0, run_lua_script, (LPVOID)cpath, 0, NULL);

	return 0;
}

DWORD WINAPI run_lua_script(void *arg)
{
	lua_flag = 1;
	exr_lua_run((char*)arg); // dofile
	sendMsg(lua_socket, "size_15");
	Sleep(10);
	sendMsg(lua_socket, "script_finished");
	
	return 0;
}

UINT32 rem_lua_scrstate(char *cmdline)
{
	int cli_fd;
	uint argc;


	if ((argc = sscanf(cmdline, "rem_script_state %d", &cli_fd)) == 1) {
		;
	}

	if (lua_flag == 0)
	{
		sendMsg(cli_fd, "size_11");
		Sleep(10);
		sendMsg(cli_fd, "script_init");
	}
	else
	{
		sendMsg(cli_fd, "size_11");
		Sleep(10);
		sendMsg(cli_fd, "lua_running");
	}

	return 0;
}

UINT32 rem_lua_script_stop(char* cmdline)//rem_lua_ss
{
	int cli_fd;
	uint argc;
	
	if ((argc = sscanf(cmdline, "rem_script_stop %d ", &cli_fd)) == 1) {
		;
	}
	lua_socket = cli_fd;
	lua_flag = 0;

	return 0;
}

UINT32 rem_tbp(char* cmdline)//rem_lua_ss
{
	//_DoCommand("st");
	UINT32 err = 0;
	char cmd_buffer[256];
	char module[10];
	char path[_MAX_PATH];
	int cli_fd;
	if (sscanf(cmdline, "rem_tbp %s %s %d ", &module, &path, &cli_fd) == 3) {
		sprintf(cmd_buffer, "tbp %s %s 1", module, path);
		err |= _DoCommand(cmd_buffer);
	}
	else
	{
		rprintf("Error sendimg arg err!\n");
		return -1;
	}

	return err;
}