/* Copyright (c) 2015-2025 Eyenix Corporation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. The name of Eyenix may not be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an Eyenix
* product.
*
* THIS SOFTWARE IS PROVIDED BY EYENIX "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL EYENIX BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
*/

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

#include "spi_base.h"
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



#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <stdlib.h>
#include <winsock2.h>
#include <ws2tcpip.h>
//#include"jtagcontrol.h"
#pragma comment (lib, "Ws2_32.lib")

#pragma warning(disable:4996)

#define ENX_JTAG_PRINT
#ifdef ENX_JTAG_PRINT
#define jprintf(fmt, ...) do {printf(fmt,##__VA_ARGS__); my_printf(fmt,##__VA_ARGS__); my_printf("\r"); } while(0);
#else
#define jprintf(fmt, ...) do {} while(0);
#endif

#define PRINTFBUF_SIZE 256*1024
int server_port = 5556;

char print_base[PRINTFBUF_SIZE+1024];
UINT u_printf_point;
UINT* print_point = &u_printf_point;


void jtag_printf(char* buf, UINT len)
{
	static UINT base = (UINT)print_base;
	static UINT* pos = (UINT*)print_point;
	UINT fir_size = 0;
	UINT sec_size = 0;
	if (base + len >= ((UINT)print_base + PRINTFBUF_SIZE)){
		sec_size = (base + len) - (((UINT)print_base + PRINTFBUF_SIZE));
		fir_size = len - sec_size;
		if (fir_size>0)memcpy((BYTE*)base, (BYTE*)buf, fir_size);
		base = (UINT)print_base;
		if (sec_size>0)memcpy((BYTE*)base, (BYTE*)buf + fir_size, sec_size);
		base += sec_size;
	}
	else{

		memcpy((BYTE*)base, (BYTE*)buf, len);
		base += len;
	}
	*pos = base;
}

#define MAX_PRINT_BUF 2048
char my_buf[MAX_PRINT_BUF] = { 0 };
UINT my_printf(const char *fmt, ...) {
	
	va_list args;
	va_start(args, fmt);
	UINT len = vsprintf(my_buf, fmt, args);
	jtag_printf(my_buf, len);
	return len;
}

typedef struct term_st {
	UINT port;
	UINT sock_fd;
	UINT last_pos;
	UINT now_pos;
	UINT buf_size;
	UINT base;
	UINT point;
	CHAR name[128];
}term_st;

void TermInitValue(void* arg){
	term_st* term	= (term_st*)arg;
	term->port		= 27015;
	term->now_pos	=(UINT) print_base;
	term->last_pos	=(UINT) print_base;
	term->base		=(UINT) print_base;
	term->point		=(UINT) print_point;
	term->buf_size  =(UINT) PRINTFBUF_SIZE;
	strcpy(term->name, "TermSendCPU0");
}


static inline long myclock()
{
	SYSTEMTIME st;
	GetSystemTime(&st);
	return ((st.wMinute*60*1000)+(st.wSecond * 1000) + (st.wMilliseconds));
}

using namespace cv;
using namespace std;

static char	*gcHistoryBuf;
char		*gcPrompt;
char		*gcCmdBuf;

UINT32		gcpu=4;

UINT32      clkdiv = 0x5;
char		*channel;
#define MAX_TXT_BUFFER 1024*1024

UINT32 _DoCommand(char *cmdline);

//*************************************************************************************************
// Shell functions
//-------------------------------------------------------------------------------------------------
//
UINT32 HelpDisp(int argc, char** argv);
UINT32 ChangeCpu(int argc, char** argv);
UINT32 DispMem(int argc, char** argv);
UINT32 DispCpuReg(int argc, char** argv);
UINT32 SetBreakPoint(int argc, char** argv);
UINT32 DeleteBreakPoint(int argc, char** argv);
UINT32 MemorySet(int argc, char** argv);
UINT32 ContinueExe(int argc, char** argv);
UINT32 SingleStep(int argc, char** argv);
UINT32 JtagTest(int argc, char** argv);
UINT32 StallCpu(int argc, char** argv);
UINT32 UnstallCpu(int argc, char** argv);
UINT32 Checkstall(int argc, char** argv);
UINT32 InitJtag(int argc, char** argv);
UINT32 Uart(int argc, char** argv);
UINT32 UartPrint(int argc, char** argv);
UINT32 ReadRegister(int argc, char** argv);
UINT32 FileSave(int argc, char** argv);
UINT32 FlashRead(int argc, char** argv);
UINT32 MemRead(int argc, char** argv);
UINT32 MemWrite(int argc, char** argv);
UINT32 FlashWrite(int argc, char** argv);
UINT32 VerifyFlash(int argc, char** argv);
UINT32 MakeCFG(int argc, char** argv);
UINT32 RegWrite(int argc, char** argv);
UINT32 Reset(int argc, char** argv);
UINT32 TBprint(int argc, char** argv);
UINT32 ICprint(int argc, char** argv);
UINT32 DCprint(int argc, char** argv);
UINT32 TBenable(int argc, char** argv);
UINT32 TBdisable(int argc, char** argv);
UINT32 RunScript(int argc, char** argv);
UINT32 Test(int argc, char** argv);
UINT32 Exit(int argc, char** argv);
//UINT32 Test2(int argc, char** argv);
UINT32 ReadID(int argc, char** argv);
UINT32 Rsp_Server(int argc, char** argv);
//UINT32 SRAMWrite(int argc, char** argv);
UINT32 SectionWrite(int argc, char** argv);
UINT32 SectionWriteForOJT(int argc, char** argv);
UINT32 Reset_Cpu(int argc, char** argv);
UINT32 UnReset_Cpu(int argc, char** argv);
UINT32 ReadID_only(int argc, char** argv);
UINT32 SpiDownload(int argc, char** argv);
UINT32 JTAGPrintCmd(int argc, char** argv);
UINT32 JTAGPrintOn(int argc, char** argv);
UINT32 Waitms(int argc, char** argv);
UINT32 JTAGShellCmd(int argc, char** argv);
UINT32 GetDevNum(int argc, char** argv);
UINT32 GetReg(int argc, char** argv);
UINT32 NetConSever(int argc, char** argv);
UINT32 NetConClient(int argc, char** argv);

UINT32 Board_Checker(void);

UINT32 SendText(int argc, char** argv);
UINT32 KillSendText(int argc, char** argv);
UINT32 OpenJtag(int argc, char** argv);
UINT32 CloseJtag(int argc, char** argv);

UINT32 RemoteLog(int argc, char** argv);
UINT32 ConsoleClear(int argc, char** argv);

UINT32 FlushTest(int argc, char** argv);


const char *sHelpDisp[]			= { "Shell command list (! : Repeat command)", (char*)0 };
const char *sChangeCpu[]		= { "Change CPU", (char*)0 };
const char *sDispMem[]			= { "Display memory content", (char*)0 };
const char *sDispCpuReg[]		= { "Display Cpu register", (char*)0 };
const char *sSetBreakPoint[]	= { "Set break point", (char*)0 };
const char *sDeleteBreakPoint[] = { "Delete break point", (char*)0 };
const char *sMemorySet[]		= { "set npc address", (char*)0 };
const char *sContinueExe[]		= { "Continue execution", (char*)0 };
const char *sSingleStep[]		= { "Single step", (char*)0 };
const char *sJtagTest[]			= { "Jtag Test", (char*)0 };
const char *sStallCpu[]			= { "Stall Cpu", (char*)0 };
const char *sUnstallCpu[]		= { "Unstall Cpu", (char*)0 };
const char *sCheckstall[]		= { "Check stall Cpu", (char*)0 };
const char *sInitJtag[]			= { "init (all,tab,jtag)", (char*)0 };
const char *sUart[]				= { "Uart test", (char*)0 };
const char *sUartPrint[]		= { "Uart print", (char*)0 };
const char *sReadRegister[]		= { "Read Register : test read", (char*)0 };
const char *sFileSave[]			= { "File Save : ", (char*)0 };
const char *sFlashRead[]		= { "Flash read", (char*)0 };
const char *sMemRead[]			= { "Memory read", (char*)0 };
const char *sMemWrite[]			= { "Memory Write", (char*)0 };
const char *sFlashWrite[]		= { "Flash Write file", (char*)0 };
const char *sVerifyFlash[]		= { "Verify Flash", (char*)0 };
const char *sMakeCFG[]			= { "Make Config file", (char*)0 };
const char *sRegWrite[]			= { "Reg Write : test write (data) at (addr) to (count)", (char*)0 };
const char *sReset[]			= { "Reset All ", (char*)0 };
const char *sTBprint[]			= { "TB print (ex:tbp tbp.txt (option:1))", (char*)0 };
const char *sICprint[]			= { "IC print (ex:icp icp.txt) ", (char*)0 };
const char *sDCprint[]			= { "DC print (ex:dcp dcp.txt) ", (char*)0 };
const char *sTBenable[]			= { "TB enable ", (char*)0 };
const char *sTBdisable[]		= { "TB disable ", (char*)0 };
const char *sRunScript[]		= { "run (use stript.cfg file)", (char*)0 };
const char *sExit[]				= { "EXIT PROGRAM ", (char*)0 };
const char *sTest[]				= { "TEST PROGRAM ", (char*)0 };
const char *sTest2[]			= { "TEST PROGRAM ", (char*)0 };
const char *sReadID[]			= { "Read JTAG ID ", (char*)0 };
const char *sRspServer[]		= { "RSP server ", (char*)0 };
//const char *sSramWrite[]        = { "Sram Write ", (char*)0 };
const char *sSectionWrite[]     = { "Section Write ", (char*)0 };
const char *sSectionWriteForOJT[] = { "Section Write OJT", (char*)0 };
const char *sResetCpu[]         = { "Reset cpu ", (char*)0 };
const char *sUnResetCpu[]       = { "Unrset cpu ", (char*)0 };
const char *sReadID_only[]       = { "RID test ", (char*)0 };
const char *sSpiDownload[]      = { "SPI Download in flash ", (char*)0 };
const char *sJtagPrint[]        = { "JTAG print, jp (option:filename) escape: esc ", (char*)0 };
const char *sJtagPrintOn[]      = { "JTAG print On/Off ", (char*)0 };
const char *sWaitms[]           = { "Wait ms ", (char*)0 };
const char *sJtagShellCmd[]     = { "Jtag Shell Cmd ", (char*)0 };
const char *sDevNum[]			= { "Dev Num Checker", (char*)0 };
const char *sGetReg[]			= { "Get Reg", (char*)0 };
const char *sNetConSever[]		= { "Network Connect Sever", (char*)0 };
const char *sNetConClient[]		= { "Network Connect Client", (char*)0 };


const char *sSendText[]			= { "Send Text", (char*)0 };

const char *sKillSendText[]		= { "Kill Send Text", (char*)0 };

const char *sOpenJtag[]			= { "Open Jtag", (char*)0 };
const char *sCloseJtag[]		= { "Close Jtag", (char*)0 };

const char *sRemoteLog[]		= { "Remote Log On/Off", (char*)0 };
const char *sConsoleClear[]		= { "Console Log Clean", (char*)0 };

const char *sFlushTest[]		= { "Flush Test", (char*)0 };


tMonCmd gCmdList[] =
{
	{ "?", HelpDisp, sHelpDisp },
	{ "init", InitJtag, sInitJtag },
	{ "rst", Reset, sReset },
	{ "cc", ChangeCpu, sChangeCpu },
	{ "dcr", DispCpuReg, sDispCpuReg },
	{ "st", StallCpu, sStallCpu },
	{ "ust", UnstallCpu, sUnstallCpu },
	{ "cst", Checkstall, sCheckstall },
	{ "si", SingleStep, sSingleStep },
	{ "c", ContinueExe, sContinueExe },
	{ "bk", SetBreakPoint, sSetBreakPoint },
	{ "dt", DeleteBreakPoint, sDeleteBreakPoint },
	{ "set", MemorySet, sMemorySet },
	{ "tbp", TBprint, sTBprint },
	{ "icp", ICprint, sICprint },
	{ "dcp", DCprint, sDCprint },
	{ "te", TBenable, sTBenable },
	{ "td", TBdisable, sTBdisable },

//	{ "x", DispMem, sDispMem },

	{ "rr", ReadRegister, sReadRegister },
	{ "rw", RegWrite, sRegWrite },
	{ "fr", FlashRead, sFlashRead },
	{ "fw", FlashWrite, sFlashWrite },
	{ "vf", VerifyFlash, sVerifyFlash },
	{ "fs", FileSave, sFileSave },
	{ "mr", MemRead, sMemRead },
	{ "mw", MemWrite, sMemWrite },
	{ "mkcfg", MakeCFG, sMakeCFG },

	{ "uart", Uart, sUart },
	{ "uartp", UartPrint, sUartPrint },

	{ "run", RunScript, sRunScript },

	{ "up", Test, sTest },
//	{ "tt2", Test2, sTest2 },

	{ "test", JtagTest, sJtagTest },
	{ "quit", Exit, sExit },
	{ "rid", ReadID, sReadID },
	{ "rsp", Rsp_Server, sRspServer },

//  { "sw", SRAMWrite, sSramWrite },
    { "sew", SectionWrite, sSectionWrite },
//    { "sewo", SectionWriteForOJT, sSectionWriteForOJT },
    { "rc", Reset_Cpu, sResetCpu },
    { "urc", UnReset_Cpu, sUnResetCpu },

    { "spi", SpiDownload, sSpiDownload },

    { "jp", JTAGPrintCmd, sJtagPrint },
	{ "jpo", JTAGPrintOn, sJtagPrintOn },
	{ "js", JTAGShellCmd, sJtagShellCmd },
//    { "rido", ReadID_only, sReadID_only },
    { "wms", Waitms, sWaitms },
	{ "getdevnum", GetDevNum, sDevNum },
	{ "getreg", GetReg, sGetReg },
//	{ "ncsv", NetConSever, sNetConSever },
//	{ "nccl", NetConClient, sNetConClient },
	{ "sendtxt", SendText, sSendText },
	{ "killsendtxt", KillSendText, sKillSendText },
	{ "oj", OpenJtag, sKillSendText },
	{ "cj", CloseJtag, sKillSendText },
	{ "log", RemoteLog, sRemoteLog },
	{ "clear", ConsoleClear, sConsoleClear },
	{ "flut", FlushTest, sFlushTest },
	{ 0, 0, 0 }
};

UINT32 HelpDisp(int argc, char** argv)
{
	tMonCmd	*cmdptr;
	UINT8 i = 0;

	cmdptr = gCmdList;

	while (1){
		jprintf("[%02d]:[%-8s] - [%-50s]\n", i, cmdptr->name, *cmdptr->helphelp);
		cmdptr++;
		i++;
		if (cmdptr->name == 0)	return 0;
	}
}

UINT32 ChangeCpu(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - cc cpu0\r\n\
 - cc cpu1\r\n";

	if (argc != 2)
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	     if (strcmp("cpu0", argv[1]) == 0)	gcpu = 4;
	else if (strcmp("cpu1", argv[1]) == 0)	gcpu = 5;
	else									jprintf("Error : option choose cpu0, cpu1'\n");
	
	return 0;
}

UINT32 DispMem(int argc,char** argv){

	return 0;
}

UINT32 DispCpuReg(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - dcr\r\n";

	UINT32 err = 0;
	if (argc != 1)
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	err |= jtag_read_cpu_all_register(gcpu);
	return err;
}

UINT32 SetBreakPoint(int argc, char** argv){
	UINT32 err = 0;
	UINT32 addr;

	if (argc != 2)
	{
		jprintf("error : ex) break 0x100 \n");
		return 0;
	}
	addr = strtol(argv[1], NULL, 16);
	jprintf("addr 0x%x", addr);

	err |= jtag_set_trap_pos(addr, gcpu);
	return err;
}

UINT32 DeleteBreakPoint(int argc, char** argv){

	return 0;
}

UINT32 MemorySet(int argc, char** argv){
#if 0
	char* chCmd = "(ex)\r\n\
 - set npc 0xADDR(hex)\r\n";

	UINT32 addr=0;

	if (argc != 3)
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	addr = strtol(argv[2], NULL, 16);

	example_write_cpu_register(addr, gcpu);

	return 0;
#else
	char* chCmd = "(ex)\r\n\
 - set npc 0xADDR(hex)\r\n\
 - set npc 0xADDR(hex) (cpu0,cpu1)\r\n";

	UINT32 addr = 0;
	UINT32 module = 0;

	UINT32 err = 0;

	if (argc == 3)
	{
		addr = strtol(argv[2], NULL, 16);

		err |= example_write_cpu_register(addr, gcpu);
	}
	else if (argc == 4)
	{
		if (strcmp("cpu0", argv[3]) == 0)	module = 4;
		else if (strcmp("cpu1", argv[3]) == 0)	module = 5;
		else{
			jprintf("Error : Bad or not command!\r\n%s", chCmd);
			return 0;
		}
		addr = strtol(argv[2], NULL, 16);
		err |= example_write_cpu_register(addr, module);
	}
	else
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	return err;
#endif
}

UINT32 ContinueExe(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - c\r\n";

	UINT32 err = 0;

	if (argc != 1)
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	err |= jtag_continue(gcpu);
	jprintf("Continue!\r\n");

	return err;
}

UINT32 SingleStep(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - si\r\n\
 - si NUM(dec)\r\n\
 - si NUM(dec) FILE.TXT\r\n";

	UINT32 i,j = 0;
	UINT32 repeat = 1;
	UINT32 optFlag = 0;
	FILE *stream = NULL;
	UINT8 stalled;
	UINT32 stallchkCnt = 0;
    UINT32 err = 0;
	char buf[128] = { 0, 0 };
	char buffer[128] = { 0, 0 };
	char* optBuf=NULL;

	if (argc > 3) {
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

// Get Count
	if ((argc == 2) || (argc == 3)) {
		repeat = atoi(argv[1]);
		jprintf("repeat %d step\n", repeat);
	}

// Get File Name
	if (argc == 3) {
		optFlag = 1; // save option
		jprintf("Save : %s file\n", argv[2]);
		if (fopen_s(&stream, argv[2], "w+t") != 0){
			jprintf("file open fail\n");
			return 0;
		}
	}

// Single Step
	err |= jtag_single_step(gcpu);

	for (i = 0; i < repeat; i++){
		j = 0;
        err |= jtag_unstall_cpu(gcpu);
		stallchkCnt=0;
		do{
            err |= jtag_check_stalled_cpu(gcpu, &stalled);
			if (stallchkCnt>100) {
				if (optFlag){
					fclose(stream);
				}
				return err;
			}
			stallchkCnt++;
		} while (!(stalled & 1));
#if 0
		err |= jtag_read_cpu_register(gcpu, (UINT8*)buf);
		j |= sprintf(buffer + j, "%s", buf);
		jprintf("%s", buffer);
#endif

		err |= jtag_read_cpu_status(gcpu,(UINT8*)buf);

//		j = sprintf(buffer, "\t[%3d]", i); // for numbering
		j |= sprintf(buffer + j, "%s", buf );
		jprintf("%s", buffer);
//		if ((i % 100) == 0) jprintf("%d\n", i);

		if (optFlag) {
			fwrite(buf, sizeof(char), strlen(buf), stream);
		}
	}

	if (optFlag){
		if (stream != NULL) fclose(stream);
	}

    return err;
}

UINT32 JtagTest(int argc, char** argv)
{
    UINT32 err=0;
    // Init tap
#if !CHANGE_EN674
    reset_sys_wdt();
#endif
    err |=_DoCommand("init tap");
    if (err |= Board_Checker()){
        jprintf("Please check JTAG cable!\n"); 
    }
    else{
        jprintf("JTAG is Connected!\n");
    }
    return err;
}

UINT32 StallCpu(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - st\r\n\
 - st cpu0\r\n\
 - st cpu1\r\n";

	UINT32 err = 0;

#if 0
	if (argc != 1)
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	UINT8 stalled = 0;

	jtag_stall_cpu(gcpu);
	jtag_check_stalled_cpu(gcpu, &stalled);
	if (stalled){
		jprintf("Success : now stalled\n");
	}
	else{
		jprintf("Fail : now unstalled\n");
	}
	return 0;
#else
	EnterCriticalSection(&CriticalSection);
	if (argc == 1)
	{
		UINT8 stalled_CPU0 = 0;
		UINT8 stalled_CPU1 = 0;
		err |= jtag_stall_cpu(4);
		err |= jtag_check_stalled_cpu(4, &stalled_CPU0);

		err |= jtag_stall_cpu(5);
		err |= jtag_check_stalled_cpu(5, &stalled_CPU1);

		if (stalled_CPU0 && stalled_CPU1){
			jprintf("Success : now CPU0, CPU1 stalled\n");
		}
		else{
			if (!stalled_CPU0)
				jprintf("Fail : now CPU0 unstalled\n");
			if (!stalled_CPU1)
				jprintf("Fail : now CPU1 unstalled\n");
		}
	}
	else if (argc == 2)
	{
		UINT8 stalled = 0;
		if (strcmp("cpu0", argv[1]) == 0)
		{
			err |= jtag_stall_cpu(4);
			err |= jtag_check_stalled_cpu(4, &stalled);
			if (stalled){
				jprintf("Success : now CPU0 stalled\n");
			}
			else{
				jprintf("Fail : now CPU0 unstalled\n");
			}
		}
		else if (strcmp("cpu1", argv[1]) == 0)
		{
			err |= jtag_stall_cpu(5);
			err |= jtag_check_stalled_cpu(5, &stalled);
			if (stalled){
				jprintf("Success : now CPU1 stalled\n");
			}
			else{
				jprintf("Fail : now CPU1 unstalled\n");
			}
		}
		else
		{
			jprintf("Error : Bad or not command!\r\n%s", chCmd);
			LeaveCriticalSection(&CriticalSection);
			return 0;
		}
	}
	else
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		LeaveCriticalSection(&CriticalSection);
		return 0;
	}
	LeaveCriticalSection(&CriticalSection);
	return err;
#endif
}
UINT32 UnstallCpu(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - ust\r\n\
 - ust cpu0\r\n\
 - ust cpu1\r\n";

	UINT32 err = 0;
#if 0
	if (argc != 1)
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	UINT8 stalled = 0;

	jtag_unstall_cpu(gcpu);
	jtag_check_stalled_cpu(gcpu, &stalled);
	if (stalled){
		jprintf("Fail : now stalled\n");
	}
	else{
		jprintf("Success : now unstalled\n");
	}
	return 0;
#else
	EnterCriticalSection(&CriticalSection);
	if (argc == 1)
	{
		UINT8 stalled_CPU0 = 0;
		UINT8 stalled_CPU1 = 0;
		err |= jtag_unstall_cpu(4);
		err |= jtag_check_stalled_cpu(4, &stalled_CPU0);

		err |= jtag_unstall_cpu(5);
		err |= jtag_check_stalled_cpu(5, &stalled_CPU1);

		if (stalled_CPU0 && stalled_CPU1){
			if (stalled_CPU0)
				jprintf("Fail : now CPU0 stalled\n");
			if (stalled_CPU1)
				jprintf("Fail : now CPU1 stalled\n");
		}
		else{
			jprintf("Success : now CPU0, CPU1 unstalled\n");
		}
	}
	else if (argc == 2)
	{
		UINT8 stalled = 0;
		if (strcmp("cpu0", argv[1]) == 0)
		{
			err |= jtag_unstall_cpu(4);
			err |= jtag_check_stalled_cpu(4, &stalled);
			if (stalled){
				jprintf("Fail : now CPU0 stalled\n");
			}
			else{
				jprintf("Success : now CPU0 unstalled\n");
			}
		}
		else if (strcmp("cpu1", argv[1]) == 0)
		{
			err |= jtag_unstall_cpu(5);
			err |= jtag_check_stalled_cpu(5, &stalled);
			if (stalled){
				jprintf("Fail : now CPU1 stalled\n");
			}
			else{
				jprintf("Success : now CPU1 unstalled\n");
			}
		}
		else
		{
			jprintf("Error : Bad or not command!\r\n%s", chCmd);
			LeaveCriticalSection(&CriticalSection);
			return 0;
		}
	}
	else
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		LeaveCriticalSection(&CriticalSection);
		return 0;
	}
	LeaveCriticalSection(&CriticalSection);
	return err;
#endif
}

UINT32 Checkstall(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - cst\r\n";

	UINT32 err = 0;

	if (argc != 1)
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}
	EnterCriticalSection(&CriticalSection);
	UINT8 stalled = 0;

	err |= jtag_check_stalled_cpu(gcpu, &stalled);
	if (stalled){
		jprintf("Now stalled\n");
	}
	else{
		jprintf("Now unstalled\n");
	}
	LeaveCriticalSection(&CriticalSection);
	return err;
}

UINT32 InitJtag(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - init\r\n\
 - init all\r\n\
 - init all clkdiv(0 ~ 256)\r\n\
 - init tap\r\n\
 - init jtag\r\n\
 - init g\r\n";

	UINT32 err = 0;
	EnterCriticalSection(&CriticalSection);
	if (argc == 1) {
		jprintf("Tap reset only!\n");
		err |= tap_reset();
		err |= tap_set_ir_debug();
		LeaveCriticalSection(&CriticalSection);
		return err;
	}
	else if (strcmp("all", argv[1]) == 0) {
		if (argc > 3) {
			jprintf("Error : Bad or not command!\r\n%s", chCmd);
			LeaveCriticalSection(&CriticalSection);
			return 0;
		}
		if (argc == 3) clkdiv = atoi(argv[2]);
		err |= MPSSE_close();
		if (server_state_flag)
			spi_ftdx_init(0, 0, 0, 0);
		else
			while (spi_ftdx_init(0, 0, 0, 0))
		spi_ftdx_close();
		if (server_state_flag)
			MPSSE_init(clkdiv, *channel);
		else
			while (MPSSE_init(clkdiv, *channel));
		err |= tap_reset();
		err |= tap_set_ir_debug();
		LeaveCriticalSection(&CriticalSection);
		return err;
	}
	else if (argc == 2) {
		if (strcmp("tap", argv[1]) == 0){
			jprintf("Tap reset only!\n");
			err |= tap_reset();
			err |= tap_set_ir_debug();
		}
		else if (strcmp("jtag", argv[1]) == 0){
			err |= MPSSE_close();
			while (MPSSE_init(clkdiv, *channel));
		}
		else if (strcmp("g", argv[1]) == 0){
			err |= tap_reset_global();
			err |= tap_set_ir_debug();
			//jtag_stall_cpu(gcpu);
		}
		else {
			jprintf("Error : Bad or not command!\r\n%s", chCmd);
			LeaveCriticalSection(&CriticalSection);
			return 0;
		}
	}
	else {
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		LeaveCriticalSection(&CriticalSection);
		return 0;
	}

	LeaveCriticalSection(&CriticalSection);
	return err;
}

UINT32 Uart(int argc, char** argv){
	UINT32 err = 0;
	err |= example_uart_control();
	return err;
}

UINT32 UartPrint(int argc, char** argv){
	UINT32 err = 0;
	UINT32 len=0;

	if (argc != 2)
	{
		jprintf("error : ex) uartp INPUT\n //printf uart");
		return 0;
	}
	len = strlen(argv[1]);
	err |= example_uart_print(gcpu,(UINT8*) argv[1], len);

	return err;
}

UINT32 ReadRegister(int argc, char** argv){
	UINT32 err = 0;
	UINT32 module = 0;
	UINT32 addr = 0;
	UINT32 num, i = 0;
	UINT32 reg = 0;
	FILE*	pFile = NULL;

	UINT32 k = 0;
	char  fileName[1024];

	if (argc == 4){
		jprintf("Use (default:rr.txt) file : \n");
		strcpy(fileName, "rr.txt");
	}
	else if (argc == 5){
		jprintf("Use (%s) file \n", argv[4]);
		strcpy(fileName, argv[4]);
	}
	else{
		jprintf("error : ex) rr (wbcpu0,wbcpu1,wbcom,cpu0,cpu1) 0x00000000 num (rr.txt)\n");
		return 0;
	}

	     if (strcmp("wbcpu0", argv[1]) == 0)	module = 0;
    else if (strcmp("wbcpu1", argv[1]) == 0)	module = 1;
	else if (strcmp("wbcom", argv[1]) == 0)		module = 3;
	else if (strcmp("cpu0",   argv[1]) == 0)	module = 4;
	else if (strcmp("cpu1",   argv[1]) == 0)	module = 5;
	else {
		jprintf("Error : option choose (wbcpu0,wbcpu1,wbcom,cpu0,cpu1)'\n");
		jprintf("Error : ex) rr (wbcpu0,wbcpu1,wbcom,cpu0,cpu1) 0x00000000 num (rr.txt)\n");
		return 0;
	}

	addr = strtoul(argv[2], NULL, 16);

	char* memblock;
	memblock = (char*)malloc(MAX_TXT_BUFFER);
	if (memblock == NULL){
		jprintf("Malloc fail !\n");
		return 0;
	}
	num = atoi(argv[3]);
	addr = ((addr >> 2) << 2);			// 4의 배수 단위로 Addr 맞춤 
//	jprintf("addr:0x%x\n", addr);
	for (i = 0; i < num; i++){
		err = example_read_register(module, addr + (i * 4), &reg);
		jprintf("0x%.8x :   reg = %.8x \n", addr + (i * 4), reg);
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "0x%.8x :   reg = %.8x \n", addr + (i * 4), reg);
	}

	pFile = fopen(fileName, "w+b");
	fwrite(memblock, sizeof(char), strlen(memblock), pFile);
	fclose(pFile);

	if (memblock) free(memblock);

	return err;
}

void hexDump(char *desc, void *addr, int len) {
	int i;
	unsigned char buff[17];
	unsigned char *pc = (unsigned char*)addr;

	// Output description if given.
	if (desc != NULL)
		jprintf("%s:\n", desc);

	// Process every byte in the data.
	for (i = 0; i < len; i++) {
		// Multiple of 16 means new line (with line offset).

		if ((i % 16) == 0) {
			// Just don't print ASCII for the zeroth line.
			if (i != 0)
				jprintf("  %s\n", buff);

			// Output the offset.
			jprintf("  %04x ", i);
		}

		// Now the hex code for the specific character.
		jprintf(" %02x", pc[i]);

		// And store a printable ASCII character for later.
		if ((pc[i] < 0x20) || (pc[i] > 0x7e))
			buff[i % 16] = '.';
		else
			buff[i % 16] = pc[i];
		buff[(i % 16) + 1] = '\0';
	}

	// Pad out last line if not exactly 16 characters.
	while ((i % 16) != 0) {
		jprintf("   ");
		i++;
	}

	// And print the final ASCII bit.
	jprintf("  %s\n", buff);
}

UINT32 FileSave(int argc, char** argv){
	
	UINT32 err = 0;
	streampos length;
	UINT size;
	char* memblock;

	ifstream file("or1200-mul.bin", ios::in | ios::binary | ios::ate);
	if (file.is_open())
	{
		length = file.tellg();
		size = (UINT32)length;
		memblock = new char[size];
		file.seekg(0, ios::beg);
		file.read(memblock, size);
		file.close();

		cout << "the entire file content is in memory";
		
//		hexDump("DUMP", memblock, size);
	
	
		err |= jtag_write_block8(SDRAM_BASE, (UINT8*)memblock, (UINT32)size, 0);

		delete[] memblock;
	}
	else cout << "Unable to open file";

	return err;
}

UINT32 jtag_htonl(UINT32 n)
{
	return ((n & 0xff) << 24) |
		((n & 0xff00) << 8) |
		((n & 0xff0000UL) >> 8) |
		((n & 0xff000000UL) >> 24);
}


#define READBUFFER 1024*2
#define FAST_READ 
UINT32 FlashRead(int argc, char** argv){
	UINT32 err = 0;
	UINT32 i,reg = 0;
	UINT32 pSfls = FLASH_BASE;
	UINT32 size = 0;
	FILE * pFile;
	UINT8 * memblock=NULL;
	UINT8 * memcompblock = NULL;
	UINT8 * Nmemblock = NULL;
	UINT32 retry=0, left = 0;

	UINT32* memblock32 = NULL;
	UINT32* Nmemblock32 = NULL;

	UINT32 ret = 0;

    char  fileName[1024];
    if (argc == 1){
        jprintf("Use (default:flash.bin) file : \n");
        strcpy(fileName, "flash.bin");
    }
    else if (argc == 2){
        jprintf("Use (%s) file \n", argv[1]);
        strcpy(fileName, argv[1]);
    }

	cout << "\t----Flash Info----\n";
	for (i = 0; i < 4; i++){
		jtag_read32(pSfls, &reg, 3);
		pSfls += reg;
		size += reg;
		cout <<"["<<i<<"]"<<"size :"<< reg<<"\n";
	}
	cout << "total size :" <<size<<"\n";

	

	memblock = (UINT8*)malloc(size+4);
	memcompblock = (UINT8*)malloc(size + 4);
	Nmemblock = (UINT8*)malloc(size + 4);
	retry = (size) / (READBUFFER);
	left = (size) % (READBUFFER);
	for (i = 0; i < retry; i++){
		
#ifdef FAST_READ
		do{
			if (ret)jprintf("Read again : %d\n", i);
			err |= jtag_read_block32(FLASH_BASE + (READBUFFER * i), (UINT32*)(memblock + (READBUFFER * i)), (READBUFFER / 4), JTAG_COMMON_MODULE_IDX);
			err |= jtag_read_block32(FLASH_BASE + (READBUFFER * i), (UINT32*)(memcompblock + (READBUFFER * i)), (READBUFFER / 4), JTAG_COMMON_MODULE_IDX);
		} while (ret = memcmp((memblock + (READBUFFER * i)), (memcompblock + (READBUFFER * i)), READBUFFER));
		
#else
		jtag_read_block8(FLASH_BASE + (READBUFFER * i), (UINT8*)memblock + (READBUFFER * i), (READBUFFER), JTAG_COMMON_MODULE_IDX);
#endif
		cout << "*";
	}
	
#ifdef FAST_READ
	if (left){
		do{
			if (ret)jprintf("Read again : %d\n", i);
			err |= jtag_read_block32(FLASH_BASE + (READBUFFER * i), (UINT32*)(memblock + (READBUFFER * i)), (left / 4) + 1, JTAG_COMMON_MODULE_IDX);
			err |= jtag_read_block32(FLASH_BASE + (READBUFFER * i), (UINT32*)(memcompblock + (READBUFFER * i)), (left / 4), JTAG_COMMON_MODULE_IDX);
		} while (ret = memcmp((memblock + (READBUFFER * i)), (memcompblock + (READBUFFER * i)), left));
	}
#else
	if(left) jtag_read_block8(FLASH_BASE + (READBUFFER * i), (UINT8*)memblock + (READBUFFER * i), left, JTAG_COMMON_MODULE_IDX);
#endif
	cout << "END";
//	hexDump("DUMP", memblock, size);




#ifdef FAST_READ
	// change endian fr
	memblock32 = (UINT32*)memblock;
	Nmemblock32 = (UINT32*)Nmemblock;
	for (i = 0; i < (size / 4) + 1; i++){
		*(Nmemblock32 + i) = jtag_htonl((UINT32)*(memblock32 + i));
	}
    pFile = fopen(fileName, "w+b");
	fwrite(Nmemblock, sizeof(char), size, pFile);
	fclose(pFile);
#else
	pFile = fopen("flash.bin", "w+b");
	fwrite(memblock, sizeof(char), size, pFile);
	fclose(pFile);
#endif	
	if(memblock) free(memblock);
	if (memcompblock) free(memcompblock);
	if (Nmemblock) free(Nmemblock);
	return err;
}

//#define MEASURE_TIME
#define MR_READBUFFER 1024*8
UINT32 MemRead(int argc, char** argv){
	UINT32 err = 0;
	UINT32 i, reg = 0;
	UINT32 size = 0;
	UINT32 size_adj = 0;
	FILE * pFile;
	UINT8 * memblock = NULL;
	UINT8 * memblock_t = NULL;
	UINT32* memblock32 = NULL;
	UINT32* memblock32_t = NULL;
	UINT32 retry = 0, left = 0;

	char  fileName[1024];

	cout << "\t----Memory Info----\n";

	UINT32 addr;
	UINT32 module;

	if (argc == 3){
		jprintf("Use (default:mem.bin) file : \n");
		strcpy(fileName, "mem.bin");
		module = 3;
	}
	else if (argc == 4){
		strcpy(fileName, argv[3]);
		jprintf("Use (%s) file \n", fileName);
		module = 3;
	}
	else if (argc == 5){
		strcpy(fileName, argv[3]);
		jprintf("Use (%s) file \n", fileName);

		if (strcmp("wbcpu0", argv[4]) == 0){
			jprintf("wbcpu0 selected!\n");
			module = 0;
		}
		else if (strcmp("wbcpu1", argv[4]) == 0){
			jprintf("wbcpu1 selected!\n");
			module = 1;
		}
		else if (strcmp("wbcom", argv[4]) == 0){
			module = 3;
		}
		else{
			jprintf("Error Command!\n");
			return 0;
		}

	}
	else{
		jprintf("error : ex) mr addr size (optional=file) (optional=wbcpu0,wbcpu1,wbcom) \n");
		return 0;
	}

	addr = strtoul(argv[1], NULL, 16);

	size = atoi(argv[2]);
	size_adj = ((size + 4) >> 2) << 2;

//	addr = ((addr >> 2) << 2);			// 4의 배수 단위로 Addr 맞춤 
	jprintf("addr:0x%x\n", addr);

	if ((memblock = (UINT8*)malloc(size_adj + 4)) == NULL){
		jprintf("Malloc fail \n");
		return 0;
	}
	if ((memblock_t = (UINT8*)malloc(size_adj + 4)) == NULL){
		jprintf("Malloc fail \n");
		return 0;
	}

	retry = (size_adj) / (MR_READBUFFER);
	left = (size_adj) % (MR_READBUFFER);

#ifdef MEASURE_TIME
	ULONG64 measure_time = 0;
	LONG64 diff_time = 0;
	measure_time = myclock();
#endif

	for (i = 0; i < retry; i++){
        if (EN673_JTAG_ESC_key_checker()){
            if (memblock) free(memblock);
            if (memblock_t) free(memblock_t);
			return 0;
        }
		err |= jtag_read_block32(addr + (MR_READBUFFER * i), (UINT32*)(memblock + (MR_READBUFFER * i)), (MR_READBUFFER / 4), module);
		cout << "*";
	}
	if (left)
		err |= jtag_read_block32(addr + (MR_READBUFFER * i), (UINT32*)(memblock + (MR_READBUFFER * i)), (left / 4) + 1, module);
	cout << "END";


#ifdef MEASURE_TIME
	diff_time = myclock() - measure_time;
    if (diff_time <= 0)diff_time = 1;
	jprintf("\n       Time : %10ld ms \n", (ULONG64)diff_time);
	jprintf("       Size : %10ld bytes\n", (ULONG64)size_adj);
	jprintf("       Speed: %10ld byte/s\n", ((ULONG64)size_adj * 1000) / (ULONG64)diff_time);
#endif

	memblock32 = (UINT32*)memblock;
	memblock32_t = (UINT32*)memblock_t;
	for (i = 0; i < (size_adj / 4) + 1; i++){
		*(memblock32_t + i) = jtag_htonl((UINT32)*(memblock32 + i));
	}
	memblock_t = (UINT8*)memblock32_t;
//	hexDump("DUMP", memblock_t, size);

	pFile = fopen(fileName, "w+b");
	fwrite(memblock_t, sizeof(char), size, pFile);
	fclose(pFile);

	if (memblock) free(memblock);
	if (memblock_t) free(memblock_t);

	return err;
}


#define MW_MAX_RETRY_NUM 100
#define MEM_WRITE_BUFFER_SIZE 1024*4
UINT32 MemWrite(int argc, char** argv){
	UINT32 err = 0;
	streampos size;
	UINT32 size_adj;
	char * memblock = NULL;
	char * Nmemblock = NULL;
	UINT32* memblock32 = NULL;
	UINT32* Nmemblock32 = NULL;
	UINT32 addr;
	char  fileName[1024];

	UINT32 module;

	if (argc == 2){
		jprintf("Use (default:mw.bin) file : \n");
		strcpy(fileName, "mw.bin");
		module = 3;
	}
	else if (argc == 3){
		jprintf("Use (%s) file \n", argv[2]);
		strcpy(fileName, argv[2]);
		module = 3;
	}
	else if (argc == 4)
	{
		jprintf("Use (%s) file \n", argv[2]);
		strcpy(fileName, argv[2]);

		if (strcmp("wbcpu0", argv[3]) == 0){
			jprintf("wbcpu0 selected!\n");
			module = 0;
		}
		else if (strcmp("wbcpu1", argv[3]) == 0){
			jprintf("wbcpu1 selected!\n");
			module = 1;
		}
		else if (strcmp("wbcom", argv[3]) == 0){
			module = 3;
		}
		else{
			jprintf("Error Command!\n");
			return 0;
		}
	}
	else{
		jprintf("error : ex) mw addr (optional=file) (optional=wbcpu0,wbcpu1,wbcom) \n");
		return 0;
	}

	addr = strtoul(argv[1], NULL, 16);

	ifstream file(fileName, ios::in | ios::binary | ios::ate);
	if (file.is_open())
	{
		file.seekg(0, ios::end);
		size = file.tellg();

		size_adj =(UINT32) size;
		//size_adj = ((size_adj + 4) >> 2) << 2;

		memblock = new char[size_adj];
		Nmemblock = new char[size_adj];
		file.seekg(0, ios::beg);
		file.read(memblock, size);
		file.close();
		
		cout << "MEM WRITE SIZE:" << size_adj << "bytes\n";
		jprintf("Addr : %x\n", addr);
		UINT i;

		memblock32 = (UINT32*)memblock;
		Nmemblock32 = (UINT32*)Nmemblock;
		for (i = 0; i < (size_adj / 4) + 1; i++){
			*(Nmemblock32 + i) = jtag_htonl((UINT32)*(memblock32 + i));
		}
				//hexDump("DUMP", memblock, size);
	
		UINT32 remaining_len = size_adj;
		UINT32 re_remaining_len = remaining_len % 16;
		UINT32 pos=0;

#ifdef MEASURE_TIME
		ULONG64 measure_time = 0;
		LONG64 diff_time = 0;
		measure_time = myclock();
#endif
		for (remaining_len = size_adj; remaining_len >= MEM_WRITE_BUFFER_SIZE; remaining_len -= MEM_WRITE_BUFFER_SIZE){
			
			err |= jtag_write_block32(addr + pos, (UINT32*)(Nmemblock + pos), MEM_WRITE_BUFFER_SIZE / 4, module);
			
			pos += MEM_WRITE_BUFFER_SIZE;
            if (EN673_JTAG_ESC_key_checker()){
                delete[] memblock;
                delete[] Nmemblock;
				return 0;
            }
			jprintf("*");
			//Sleep(100);z
		}

		if (remaining_len){
			err |= jtag_write_block32(addr + pos, (UINT32*)(Nmemblock + pos), (remaining_len - re_remaining_len / 4), module);
			pos = pos + remaining_len - re_remaining_len;
		}

		if (re_remaining_len) {
			err |= jtag_write_block8(addr + pos, (UINT8*)(Nmemblock + pos), (re_remaining_len / 4), module);
			pos = pos - re_remaining_len;
		}

#ifdef MEASURE_TIME
		diff_time = myclock() - measure_time;
        if (diff_time <= 0)diff_time = 1;
		jprintf("\n       Time : %10ld ms \n", (ULONG64)diff_time);
		jprintf("       Size : %10ld bytes\n", (ULONG64)size_adj);
		jprintf("       Speed: %10ld byte/s\n", ((ULONG64)size_adj*1000) / (ULONG64)diff_time);
#endif



#ifdef DATA_VERIFY // for Verify
		//Verify
		//Compare 4kbytes 
		//If different write again
		UINT32 remainlen = 0;
		char * veribuf;
		char retry = 0;
        veribuf = new char[MEM_WRITE_BUFFER_SIZE];
		i = 0;
		for (remainlen = size_adj; remainlen > MEM_WRITE_BUFFER_SIZE; remainlen -= (UINT32)(MEM_WRITE_BUFFER_SIZE)){
            if (EN673_JTAG_ESC_key_checker()){
                delete[] memblock;
                delete[] Nmemblock;
                delete[] veribuf;
				return 0;
            }
			retry = 0;
			err |= jtag_read_block32(addr + i, (UINT32*)veribuf, MEM_WRITE_BUFFER_SIZE / 4, module);
			//		hexDump("veribuf", veribuf, 1024 * 4);
			//		hexDump("memblock", Nmemblock + i, 1024 * 4);
			jprintf("v");
            while (memcmp(veribuf, Nmemblock + i, MEM_WRITE_BUFFER_SIZE)){
                if (EN673_JTAG_ESC_key_checker()){
                    delete[] memblock;
                    delete[] Nmemblock;
                    delete[] veribuf;
					return 0;
                }
				//Erase 4K 
				//	hexDump("veribuf", veribuf, 1024 );
				//	hexDump("memblock", Nmemblock + i, 1024);
				err |= jtag_write_block32(addr + i, (UINT32*)(Nmemblock + i), MEM_WRITE_BUFFER_SIZE / 4, module);
				retry++;
				if (retry>MW_MAX_RETRY_NUM){
					jprintf("Over Max Retry for write!");
					break;
				}
				jtag_read_block32(addr + i, (UINT32*)veribuf, MEM_WRITE_BUFFER_SIZE / 4, module);
			}
            i += MEM_WRITE_BUFFER_SIZE;
		}

		if (remainlen){
			retry = 0;
			err |= jtag_read_block32(addr + i, (UINT32*)veribuf, (remainlen / 4), module);
			//hexDump("veribuf", veribuf, 1024 );
			while (memcmp(veribuf, Nmemblock + i, remainlen)){
				jtag_write_block32(addr + i, (UINT32*)(Nmemblock + i), MEM_WRITE_BUFFER_SIZE / 4, module);
				retry++;
				if (retry>MW_MAX_RETRY_NUM){
					jprintf("Over Max Retry for write flash!");
					break;
				}
				err |= jtag_read_block32(addr + i, (UINT32*)veribuf, (remainlen / 4), module);
			}
		}
		cout << "END";
        delete[] veribuf;
#endif

		delete[] memblock;
		delete[] Nmemblock;
	}
	else cout << "Unable to open file";

	return err;

}


typedef enum {
    CS_LMA = 0,
    CS_RESET_LMA = 1, CS_TEXT_LMA, CS_ISPM_TEXT_LMA, CS_ISPM_DATA_LMA, CS_RODATA_LMA, CS_DATA_LMA,
    CS_RESET = 7, CS_TEXT, CS_ISPM_TEXT, CS_ISPM_DATA, CS_RODATA, CS_DATA,  		// : Code Sections
    CS_BSS, CS_STACK
} eCodeSectionIndex;

#define	CODE_SECTION_IDX(NAME) 		CS_ ## NAME
#define	CODE_SECTION_LMA_IDX(NAME)	CS_ ## NAME ##_LMA
#define	CS_NSECTIONS				(CS_RESET-CS_RESET_LMA)
#define	VMA_TO_LMA(VMA_ID)			((VMA_ID)-CS_NSECTIONS)

typedef union {
    UINT32 info[2];
    struct {
        UINT32 s, e;
    };
} tCodeSectionInfo;

typedef union {
    tCodeSectionInfo section[1 + 2 * CS_NSECTIONS];
    struct {
        tCodeSectionInfo	lma;
        tCodeSectionInfo 	reset_lma, text_lma, ispm_text_lma, ispm_data_lma, rodata_lma, data_lma;
        tCodeSectionInfo	reset, text, ispm_text, ispm_data, rodata, data;
        tCodeSectionInfo	bss, stack;
    };
} tCodeMemInfo;

UINT32 SectionWrite(int argc, char** argv){

    UINT32 err = 0;
    streampos size;
    UINT32 size_adj;
    char * memblock = NULL;
    char * Nmemblock = NULL;
    UINT32* memblock32 = NULL;
    UINT32* Nmemblock32 = NULL;
    UINT32 addr;
    UINT32 localaddr;
    char  fileName[1024];
    UINT32 id = 0;
    UINT32 cpu = 0;
    UINT32 module_idx;
    UINT32 start_of_addr;
    if (argc == 1){
        jprintf("Use (default:sew.bin) file : \n");
        strcpy(fileName, "sew.bin");
    }
    else if (argc == 2){
        jprintf("Use (%s) file \n", argv[1]);
        strcpy(fileName, argv[1]);
        id = 0;
        cpu = 0;
    }
    else if (argc == 4){
        jprintf("Use (%s) file \n", argv[1]);
        strcpy(fileName, argv[1]);
        id = atoi(argv[2]);
        jprintf("ID %d \n", id);
        cpu = atoi(argv[3]);
        jprintf("CPU %d \n", cpu);
    }
    else{
        jprintf("error : ex) sew file (cpunum) \n");
        return 0;
    }

    ifstream file(fileName, ios::in | ios::binary | ios::ate);
    if (file.is_open())
    {
        size = file.tellg();

        size_adj = (UINT32)size;
        size_adj = ((size_adj + 4) >> 2) << 2;

        memblock = new char[size_adj];
        Nmemblock = new char[size_adj+0x1024];
        file.seekg(0, ios::beg);
        file.read(memblock, size);
        file.close();


        cout << "MEM WRITE SIZE:" << size_adj << "bytes\n";
       
        UINT i,j;

        memblock32 = (UINT32*)memblock;
        Nmemblock32 = (UINT32*)Nmemblock;
        for (i = 0; i < (size_adj / 4) + 1; i++){
            *(Nmemblock32 + i) = jtag_htonl((UINT32)*(memblock32 + i));
        }
        //		hexDump("DUMP", memblock, size);
// Find its FMA
        for (i = 0; i < id; i++) Nmemblock32 += (*Nmemblock32 >> 2);
        jprintf("Address of Membuf %p\n", Nmemblock32);
// Get Section Information
        tCodeMemInfo *pCodeMemInfo;
        pCodeMemInfo = (tCodeMemInfo *)Nmemblock32;
        UINT32 fma_base =(UINT32) Nmemblock32;
        start_of_addr = pCodeMemInfo->section[VMA_TO_LMA(CODE_SECTION_IDX(RESET))].s;

#ifdef MEASURE_TIME
        ULONG64 measure_time = 0;
        LONG64 diff_time = 0;
        measure_time = myclock();
#endif
 // Code and data copy
        for (j = CODE_SECTION_IDX(RESET); j <= CODE_SECTION_IDX(DATA); j++) {			// $CMT-ygkim-150424: copy sections from flash to its corresponding VMA
            UINT32 vma_s = pCodeMemInfo->section[j].s;
            UINT32 vma_e = pCodeMemInfo->section[j].e;
            UINT32 lma_s = pCodeMemInfo->section[VMA_TO_LMA(j)].s;
            UINT32 len = vma_e - vma_s;

            jprintf("%02d vS(0x%08X) vE(0x%08X) lS(0x%08X) size(%d) startAddr(0x%08x) - ", j, vma_s, vma_e, lma_s, len, start_of_addr);

            // Setting Addr 
            addr = (UINT32)vma_s;
            size_adj =(((len+3)>>2)<<2); 
            localaddr = (UINT32)(fma_base + (lma_s - start_of_addr));
           
			if ((addr & 0x0f000000) == SDRAM_BASE)	module_idx = cpu;
            else									module_idx = JTAG_COMMON_MODULE_IDX;
            jprintf("\nID:%d Addr:%x size:%x localAddr:%x module:%d \n", id, addr, size_adj, localaddr, module_idx);
            UINT32 remaining_len;
            UINT32 pos = 0;
            
            if (len > 0){

                ftdx_flush();   //#important!!(for flush all write data) 
                for (remaining_len = size_adj; remaining_len >= MEM_WRITE_BUFFER_SIZE; remaining_len -= MEM_WRITE_BUFFER_SIZE){

                    if (err |= jtag_write_block32( addr + pos , (UINT32*)(localaddr + pos), MEM_WRITE_BUFFER_SIZE/4, module_idx)){
                        delete[] memblock;
                        delete[] Nmemblock;
                        return err;
                    }
                    pos += MEM_WRITE_BUFFER_SIZE;
                    if (EN673_JTAG_ESC_key_checker()){
                        delete[] memblock;
                        delete[] Nmemblock;
                        return 0;
                    }
                    jprintf("*");
                    //Sleep(100);
                }

                if (remaining_len){
                    if (err |= jtag_write_block32(addr + pos, (UINT32*)(localaddr + pos), (remaining_len/4)+1, module_idx)){
                        delete[] memblock;
                        delete[] Nmemblock;
                        return err;
                    }
                    jprintf("#\n");
                }

#if 1 // for Verify
                //Verify
                //Compare 4kbytes 
                //If different write again
                UINT32 remainlen = 0;
                char retry = 0; 
                char veribuf[MEM_WRITE_BUFFER_SIZE + 1024];
                i = 0;
                for (remainlen = len; remainlen > MEM_WRITE_BUFFER_SIZE; remainlen -= (MEM_WRITE_BUFFER_SIZE)){
                    if (EN673_JTAG_ESC_key_checker()){
                        delete[] memblock;
                        delete[] Nmemblock;
                        return 0;
                    }
                    retry = 0;
                    if (err |= jtag_read_block32(addr + i, (UINT32*)veribuf, MEM_WRITE_BUFFER_SIZE / 4, module_idx)) return err;
                  //  hexDump("veribuf", veribuf, MEM_WRITE_BUFFER_SIZE);
                  //  hexDump("memblock", Nmemblock + i, MEM_WRITE_BUFFER_SIZE);
                    jprintf("v");
                    while (memcmp(veribuf, (char*)localaddr + i, MEM_WRITE_BUFFER_SIZE)){
                        if (EN673_JTAG_ESC_key_checker()){
                            delete[] memblock;
                            delete[] Nmemblock;
                            return 0;
                        }
                  //      hexDump("veribuf", veribuf, MEM_WRITE_BUFFER_SIZE);
                  //      hexDump("memblock", Nmemblock + i, MEM_WRITE_BUFFER_SIZE);
                        if (err |= jtag_write_block32(addr + i, (UINT32*)(localaddr + i), MEM_WRITE_BUFFER_SIZE / 4, module_idx)) return err;
                        retry++;
                        if (retry > MW_MAX_RETRY_NUM){
                            jprintf("Over Max Retry for write!");
                            break;
                        }
                        if (err |= jtag_read_block32(addr + i, (UINT32*)veribuf, MEM_WRITE_BUFFER_SIZE / 4, module_idx)) return err;
                    }
                    i += MEM_WRITE_BUFFER_SIZE;
                }

                if (remainlen){
                    retry = 0;
                    if (err |= jtag_read_block32(addr + i, (UINT32*)veribuf, remainlen / 4, module_idx)) return err;
                //    hexDump("veribuf", veribuf, remainlen);
                //    hexDump("memblock", Nmemblock + i, remainlen);
                    jprintf("v");
                    while (memcmp(veribuf, (char*)localaddr + i, remainlen)){
                        if (err |= jtag_write_block32(addr + i, (UINT32*)(localaddr + i), MEM_WRITE_BUFFER_SIZE / 4, module_idx)) return err;
                        retry++;
                        if (retry > MW_MAX_RETRY_NUM){
                            jprintf("Over Max Retry for write!");
                            break;
                        }
                        if (err |= jtag_read_block32(addr + i, (UINT32*)veribuf, remainlen / 4, module_idx)) return err;
                    }
                }
                jprintf("\n");
#endif
            }
        }
        delete[] memblock;
        delete[] Nmemblock;
#ifdef MEASURE_TIME
        diff_time = myclock() - measure_time;
        if (diff_time <= 0)diff_time = 1;
        jprintf("\n       Time : %10ld ms \n", (ULONG64)diff_time);
        jprintf("       Size : %10ld bytes\n", (ULONG64)size_adj);
        jprintf("       Speed: %10ld byte/s\n", ((ULONG64)size_adj * 1000) / (ULONG64)diff_time);
#endif
    }
    else cout << "Unable to open file";

    return err;

}

UINT32 SectionWriteForOJT(int argc, char** argv){

    UINT32 err = 0;
    streampos size;
    UINT32 size_adj;
    char * memblock = NULL;
    char * Nmemblock = NULL;
    UINT32* memblock32 = NULL;
    UINT32* Nmemblock32 = NULL;
    UINT32 addr;
    UINT32 localaddr;
    char  fileName[1024];
    UINT32 id = 0;


    if (argc == 1){
        jprintf("Use (default:mw.bin) file : \n");
        strcpy(fileName, "sew.bin");
    }
    else if (argc == 3){
        jprintf("Use (%s) file \n", argv[1]);
        strcpy(fileName, argv[1]);
        id = atoi(argv[2]);
    }
    else{
        jprintf("error : ex) sew file \n");
        return 0;
    }

    ifstream file(fileName, ios::in | ios::binary | ios::ate);
    if (file.is_open())
    {
        size = file.tellg();

        size_adj = (UINT32)size;
        size_adj = ((size_adj + 4) >> 2) << 2;

        memblock = new char[size_adj];
        Nmemblock = new char[size_adj + 0x1024];
        file.seekg(0, ios::beg);
        file.read(memblock, size);
        file.close();


        cout << "MEM WRITE SIZE:" << size_adj << "bytes\n";

        UINT i;

        memblock32 = (UINT32*)memblock;
        Nmemblock32 = (UINT32*)Nmemblock;
        for (i = 0; i < (size_adj / 4) + 1; i++){
            *(Nmemblock32 + i) = jtag_htonl((UINT32)*(memblock32 + i));
        }
        //		hexDump("DUMP", memblock, size);

        // Get Section Information
        tCodeMemInfo *pCodeMemInfo;
        pCodeMemInfo = (tCodeMemInfo *)Nmemblock32;
        UINT32 fma_base = (UINT32)Nmemblock32;
        // Code and data copy
        for (i = CODE_SECTION_IDX(RESET); i <= CODE_SECTION_IDX(DATA); i++) {			// $CMT-ygkim-150424: copy sections from flash to its corresponding VMA
            UINT32 vma_s = pCodeMemInfo->section[i].s;
            UINT32 vma_e = pCodeMemInfo->section[i].e;
            UINT32 lma_s = pCodeMemInfo->section[VMA_TO_LMA(i)].s;
            UINT32 len = vma_e - vma_s;

            jprintf("%02d vS(0x%08X) vE(0x%08X) lS(0x%08X) size(%d) - ", i, vma_s, vma_e, lma_s, len);

            // Setting Addr 
            addr = (UINT32)vma_s;
            size_adj = ((len >> 2) << 2);
			localaddr = (UINT32)(fma_base + (lma_s - SDRAM_BASE));
            jprintf("\n Addr:%x size:%x localAddr:%p \n", addr, size_adj, Nmemblock);
            UINT32 remaining_len;
            UINT32 pos = 0;
            if (len > 0){
#ifdef MEASURE_TIME
                ULONG64 measure_time = 0;
                LONG64 diff_time = 0;
                measure_time = myclock();
#endif
                for (remaining_len = size_adj; remaining_len >= MEM_WRITE_BUFFER_SIZE; remaining_len -= MEM_WRITE_BUFFER_SIZE){

                    if (err |= jtag_write_block32(addr + pos, (UINT32*)(localaddr + pos), MEM_WRITE_BUFFER_SIZE / 4, id)){
                        delete[] memblock;
                        delete[] Nmemblock;
                        return err;
                    }
                    pos += MEM_WRITE_BUFFER_SIZE;
                    if (EN673_JTAG_ESC_key_checker()){
                        delete[] memblock;
                        delete[] Nmemblock;
                        return 0;
                    }
                    jprintf("*");
                    //Sleep(100);
                }

                if (remaining_len){
                    if (err |= jtag_write_block32(addr + pos, (UINT32*)(localaddr + pos), (remaining_len / 4) + 1, id)){
                        delete[] memblock;
                        delete[] Nmemblock;
                        return err;
                    }
                }

#ifdef MEASURE_TIME
                diff_time = myclock() - measure_time;
                if (diff_time <= 0)diff_time = 1;
                jprintf("\n       Time : %10ld ms \n", (ULONG64)diff_time);
                jprintf("       Size : %10ld bytes\n", (ULONG64)size_adj);
                jprintf("       Speed: %10ld byte/s\n", ((ULONG64)size_adj * 1000) / (ULONG64)diff_time);
#endif



#if 0 // for Verify
                //Verify
                //Compare 4kbytes 
                //If different write again
                UINT32 remainlen = 0;
                char * veribuf;
                char retry = 0;
                veribuf = new char[MEM_WRITE_BUFFER_SIZE];
                i = 0;
                for (remainlen = size; remainlen > MEM_WRITE_BUFFER_SIZE; remainlen -= (MEM_WRITE_BUFFER_SIZE)){
                    if (EN673_JTAG_ESC_key_checker()) return 0;
                    retry = 0;
                    if (err |= jtag_read_block32(addr + i, (UINT32*)veribuf, MEM_WRITE_BUFFER_SIZE / 4, id)) return err;
                    //		hexDump("veribuf", veribuf, 1024 * 4);
                    //		hexDump("memblock", Nmemblock + i, 1024 * 4);
                    jprintf("v");
                    while (memcmp(veribuf,(char*) localaddr + i, MEM_WRITE_BUFFER_SIZE)){
                        if (EN673_JTAG_ESC_key_checker()) return 0;
                        //Erase 4K 
                        //	hexDump("veribuf", veribuf, 1024 );
                        //	hexDump("memblock", Nmemblock + i, 1024);
                        if (err |= jtag_write_block32(addr + i, (UINT32*)(localaddr + i), MEM_WRITE_BUFFER_SIZE / 4, id)) return err;
                        retry++;
                        if (retry > MW_MAX_RETRY_NUM){
                            jprintf("Over Max Retry for write !");
                            break;
                        }
                        if (err |= jtag_read_block32(addr + i, (UINT32*)veribuf, MEM_WRITE_BUFFER_SIZE / 4, id)) return err;
                    }
                    i += MEM_WRITE_BUFFER_SIZE;
                }

                if (remainlen){
                    retry = 0;
                    if (err |= jtag_read_block32(addr + i, (UINT32*)veribuf, remainlen / 4, id)) return err;
                    while (memcmp(veribuf, (char*)localaddr + i, remainlen)){
                        if (err |= jtag_write_block32(addr + i, (UINT32*)(localaddr + i), MEM_WRITE_BUFFER_SIZE / 4, id)) return err;
                        retry++;
                        if (retry > MW_MAX_RETRY_NUM){
                            jprintf("Over Max Retry for write flash!");
                            break;
                        }
                        if (err |= jtag_read_block32(addr + i, (UINT32*)veribuf, remainlen / 4, id)) return err;
                    }
                }

#endif
            }
        }
        delete[] memblock;
        delete[] Nmemblock;
    }
    else cout << "Unable to open file";

    return err;

}

#define SECTOR_SIZE 1024*4
#define MAX_RETRY_NUM 100

UINT32 FlashWrite(int argc, char** argv){

	streampos length;
	UINT32 size;
	char linkname[1024];
	char * memblock = NULL;
	char * Nmemblock = NULL;
	UINT32* memblock32 = NULL;
	UINT32* Nmemblock32 = NULL;
    UINT32 err = 0;

	if (argc == 1){
		jprintf("Use (default:link.cfg) file : \n");
		strcpy(linkname, "link.cfg");

	}else if (argc == 2){
		jprintf("Use (%s) file \n", argv[1]);
		strcpy(linkname, argv[1]);

	}else{
		jprintf("error : ex) fw (file)\n");
		return 0;
	}

	ifstream linkfile(linkname, ios::in | ios::ate);
	if (linkfile.is_open())
	{
		length = linkfile.tellg();
		size = (UINT32)length;
		if (size > 1024 ){
			cout << "file path name is over buffer size!";
			return 0;
		}
		linkfile.seekg(0, ios::beg);
		linkfile.read(linkname, size);
		linkname[size] = '\0';
		linkfile.close();
		cout << linkname << "\n";
	}
	else
	{
		cout << "Unable to open link.cfg file";
		return 0;
	}

	ifstream file(linkname, ios::in | ios::binary | ios::ate);
	if (file.is_open())
	{
		length = file.tellg();
		size = (UINT32)length;
		memblock = new char[size];
		Nmemblock = new char[size];
		file.seekg(0, ios::beg);
		file.read(memblock, size);
		file.close();

		UINT i;

		memblock32 = (UINT32*)memblock;
		Nmemblock32 = (UINT32*)Nmemblock;
		for (i = 0; i < (size / 4) + 1; i++){
			*(Nmemblock32 + i) = jtag_htonl((UINT32)*(memblock32 + i));
		}

		//		hexDump("DUMP", memblock, size);

		//jtag_write_block8(SDRAM_BASE, (UINT8*)memblock, size, 0);
		//Close all CPU
		UINT8 stalled=0;
		UINT8 ncpu;
//	for (ncpu = 4; ncpu < 7; ncpu++){
        //Reset CPU
        err |= _DoCommand("rst");

        jprintf("Flash Reg Base: %x \n",SFLS_REG_BASE);
        for (ncpu = 4; ncpu < 6; ncpu++){
			err |= jtag_stall_cpu(ncpu);
			err |= jtag_check_stalled_cpu(ncpu, &stalled);
			if (stalled){
				jprintf("Success : now stalled\n");
			}
			else
			{
				jprintf("Fail : now unstalled\n");
                delete[] memblock;
                delete[] Nmemblock;
				return 0;
			}
            err |= jtag_reset_cpu(ncpu);
		}
#if !CHANGE_EN674
        err |= reset_sys_wdt();    // Add for Reset Watchdog!!!! Flash write will fail when watchdog enabled!
		err |= Sfls_writeProtect_Disable_all();
#endif

        if (err |= Sfls_init_quad()) return err;
		cout << "ERASE:" << size << "bytes\n";
        if (err |= Sfls_erase(0x0, size)) return err;
		Sleep(100);
		cout << "\nWRITE:\n";


#ifdef MEASURE_TIME
		ULONG64 measure_time = 0;
		LONG64 diff_time = 0;
		measure_time = myclock();
#endif
        if (err |= Sfls_write32((UINT32*)Nmemblock, (size / 4) + 1, 0x0)) return err;
#ifdef MEASURE_TIME
		diff_time = myclock() - measure_time;
        if (diff_time <= 0)diff_time = 1;
		jprintf("\n       Time : %10ld ms \n", (ULONG64)diff_time);
		jprintf("       Size : %10ld bytes\n", (ULONG64)size);
		jprintf("       Speed: %10ld byte/s\n", ((ULONG64)size * 1000) / (ULONG64)diff_time);
#endif
// for debug 
//		UINT32 hello0 = 0x55555555;
//		jtag_write_block32(FLASH_BASE + 0x10100, &hello0, 1, 3);
//		UINT32 hello1 = 0x55555555;
//		jtag_write_block32(FLASH_BASE + 0x20200, &hello1, 1, 3);
//		UINT32 hello2 = 0x55555555;
//		jtag_write_block32(FLASH_BASE + 0x30300, &hello2, 1, 3);
//		UINT32 hello3 = 0x55555555;
//		jtag_write_block32(FLASH_BASE + 0x40500, &hello3, 1, 3);
//
#ifdef DATA_VERIFY // for Verify
		//Verify
		//Compare 4kbytes 
		//If different write again
		UINT32 remainlen = 0;
		char * veribuf;
		char retry = 0;
		veribuf = new char[SECTOR_SIZE];
		i = 0;
		for (remainlen = size; remainlen > SECTOR_SIZE; remainlen -= (SECTOR_SIZE)){
            if (EN673_JTAG_ESC_key_checker()| err){
                delete[] memblock;
                delete[] Nmemblock;
                delete[] veribuf;
                return err;
            }
			retry = 0;
            if (err |= jtag_read_block32(FLASH_BASE + i, (UINT32*)veribuf, SECTOR_SIZE / 4, JTAG_COMMON_MODULE_IDX)) return err;
	//		hexDump("veribuf", veribuf, 1024 * 4);
	//		hexDump("memblock", Nmemblock + i, 1024 * 4);
			jprintf("v");
			while (memcmp(veribuf, Nmemblock + i, SECTOR_SIZE)){
                if (EN673_JTAG_ESC_key_checker()|err){
                    delete[] memblock;
                    delete[] Nmemblock;
                    delete[] veribuf;
                    return err;
                }
				//Erase 4K 
	//			hexDump("veribuf", veribuf, 1024 );
	//			hexDump("memblock", Nmemblock + i, 1024);
                if (err |= Sfls_erase_sect(i)) return err;
				jprintf("e");
                if (err |= Sfls_write32((UINT32*)(Nmemblock + i), SECTOR_SIZE / 4, i)) return err;
				retry++;
				if (retry>MAX_RETRY_NUM){
					jprintf("Over Max Retry for write flash!");
					break;
				}
                if (err |= jtag_read_block32(FLASH_BASE + i, (UINT32*)veribuf, SECTOR_SIZE / 4, JTAG_COMMON_MODULE_IDX)) return err;
			}
			i += SECTOR_SIZE;
		}

		if (remainlen){
			retry = 0;
            if (err |= jtag_read_block32(FLASH_BASE + i, (UINT32*)veribuf, remainlen / 4, JTAG_COMMON_MODULE_IDX)) return err;
			while (memcmp(veribuf, Nmemblock + i, remainlen)){
				//Erase 4K 
                if (err |= Sfls_erase_sect(i)) return err;
                if (err |= Sfls_write32((UINT32*)(Nmemblock + i), remainlen / 4, i)) return err;
				retry++;
				if (retry>MAX_RETRY_NUM){
					jprintf("Over Max Retry for write flash!");
					break;
				}
                if (err |= jtag_read_block32(FLASH_BASE + i, (UINT32*)veribuf, remainlen / 4, JTAG_COMMON_MODULE_IDX)) return err;
			}
		}
        delete[] veribuf;
#endif
		delete[] memblock;
		delete[] Nmemblock;
	}
	else cout << "Unable to open file";


	err |= tap_reset_global0();  // Reset All

	err |= ReadID(0, NULL);

	return err;
}


UINT32 VerifyFlash(int argc, char** argv){
	UINT32 err = 0;
	streampos length;
	UINT32 size;
	char linkname[1024];
	char * memblock = NULL;
	char * Nmemblock = NULL;
	UINT32* memblock32 = NULL;
	UINT32* Nmemblock32 = NULL;

	ifstream linkfile("link.cfg", ios::in | ios::ate);
	if (linkfile.is_open())
	{
		length = linkfile.tellg();
		size = (UINT32)length;
		if (size > 1024){
			cout << "file path name is over buffer size!";
			return 0;
		}
		linkfile.seekg(0, ios::beg);
		linkfile.read(linkname, size);
		linkname[size] = '\0';
		linkfile.close();
		cout << linkname << "\n";
	}
	else
	{
		cout << "Unable to open link.cfg file";
		return 0;
	}

	ifstream file(linkname, ios::in | ios::binary | ios::ate);
	if (file.is_open())
	{
		length = file.tellg();
		size = (UINT32)length;
		memblock = new char[size];
		Nmemblock = new char[size];
		file.seekg(0, ios::beg);
		file.read(memblock, size);
		file.close();

		UINT i;

		memblock32 = (UINT32*)memblock;
		Nmemblock32 = (UINT32*)Nmemblock;
		for (i = 0; i < (size / 4) + 1; i++){
			*(Nmemblock32 + i) = jtag_htonl((UINT32)*(memblock32 + i));
		}

		err |= Sfls_init_quad();
	
#if 1 // for Verify
		//Verify
		//Compare 4kbytes 
		//If different write again
		UINT32 remainlen = 0;
		char * veribuf;
		char retry = 0;
		veribuf = new char[SECTOR_SIZE];
		i = 0;
		for (remainlen = size; remainlen > SECTOR_SIZE; remainlen -= (SECTOR_SIZE)){
			retry = 0;
			err |= jtag_read_block32(FLASH_BASE + i, (UINT32*)veribuf, SECTOR_SIZE / 4, JTAG_COMMON_MODULE_IDX);
			jprintf("v");
			if (memcmp(veribuf, Nmemblock + i, SECTOR_SIZE)){	
				jprintf("\nVerify fail at %08x\n", FLASH_BASE + i);
			}
			i += SECTOR_SIZE;
		}

		if (remainlen){
			retry = 0;
			err |= jtag_read_block32(FLASH_BASE + i, (UINT32*)veribuf, remainlen / 4, JTAG_COMMON_MODULE_IDX);
			if (memcmp(veribuf, Nmemblock + i, remainlen)){
				jprintf("\nVerify fail at %08x\n", FLASH_BASE + i);
			}
		}
        delete[] veribuf;
#endif
		delete[] memblock;
		delete[] Nmemblock;
	}
	else cout << "Unable to open file";
	return err;
}

UINT32 MakeCFG(int argc, char** argv){
	UINT32 len=0;
	CHAR linkname[1024];

	if (argc == 2){
		jprintf("Use (default:link.cfg) file : \n");
		strcpy(linkname, "link.cfg");

	}
	else if (argc == 3){
		jprintf("Use (%s) file \n", argv[2]);
		strcpy(linkname, argv[2]);

	}
	else{
		jprintf("error : ex) mkcfg Z:/Source/DEV_2015_EN673/2015-07/portrc113_001/en672_ref_dev_rc113_ZA20S10_fullcode/_Prj/u2.bin (file)\n");
		return 0;
	}

	len = strlen(argv[1]);

	FILE * pFile;

	pFile = fopen(linkname, "w+b");
	fwrite(argv[1], sizeof(char), len, pFile);
	fclose(pFile);
	return 0;
}

UINT32 RegWrite(int argc, char** argv){
	UINT32 err = 0;
	UINT32 addr, data, module, count;
	UINT32 reg = 0;

	if (argc < 4)
	{
		jprintf("error : ex) rw (module) (addr) (data) (count) \n");
		return 0;
	}

	if (strcmp("wbcpu0", argv[1]) == 0)			module = 0;
	else if (strcmp("wbcpu1", argv[1]) == 0)	module = 1;
	else if (strcmp("wbcom", argv[1]) == 0)		module = 3;
	else if (strcmp("cpu0", argv[1]) == 0)		module = 4;
	else if (strcmp("cpu1", argv[1]) == 0)		module = 5;
	else {
		jprintf("Error : option choose (wbcpu0,wbcpu1,wbcom,cpu0,cpu1)'\n");
		return 0;
	}

	UINT32 i = 0;
	if (argc != 4){
		count = atoi(argv[4]);
		jprintf("count %d", count);
	}
	else{
		count = 1;
	}
	addr = strtoul(argv[2], NULL, 16);
	jprintf("addr: 0x%x\n", addr);
	data = strtoul(argv[3], NULL, 16);
	jprintf("data: 0x%x ", data);

	for (i = 0; i < count; i++){
		
		if ((addr>>24) == 0x06){
			//Flash
			jprintf("Flash Write\n");
			Sfls_init_quad();
			Sfls_point_write32(data, (addr - FLASH_BASE) + (i * 4));
			Sleep(20);
		}else{
			err |= jtag_write32(addr + (i * 4), data, module);
		}
		
		err |= jtag_read32(addr + (i * 4), &reg, module);
		jprintf("[%03d] read: 0x%x\n", i,reg);

		if (data != reg) 	jprintf("[%d] write fail\n",i);
	}

	return err;
}


UINT32 Reset(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - rst\r\n\
 - rst cpu0\r\n\
 - rst cpu1\r\n";

	UINT32 err = 0;
	UINT8 cpu = 0;
	EnterCriticalSection(&CriticalSection);
	if (argc == 1){
		jprintf("Reset!\n");
		err |= tap_reset_global0();
        Sleep(100);
		err |= ReadID(0, NULL);
		jprintf("\r\n");
	}
    else if (argc == 2){
        if (strcmp("cpu0", argv[1]) == 0){
			jprintf("Reset CPU0!\n");
			err |= jtag_reset_cpu(4);
			err |= (4);
        }
		else if (strcmp("cpu1", argv[1]) == 0){
			jprintf("Reset CPU1!\n");
			err |= jtag_reset_cpu(5);
			err |= jtag_unreset_cpu(5);
        }
        else if (strcmp("all", argv[1]) == 0){
			err |= system_reset();
			err |= ftdx_flush();
        }
        else{
            jprintf("Error : Bad or not command!\r\n%s", chCmd);
			return 0;
        }
	}
	else {
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}
	LeaveCriticalSection(&CriticalSection);
	return err;
}




UINT32 Reset_Cpu(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - rc\r\n\
 - rc cpu0\r\n\
 - rc cpu1\r\n";

	UINT32 err = 0;

	EnterCriticalSection(&CriticalSection);
    if (argc == 1){
		err |= jtag_reset_cpu(4);
		err |= jtag_reset_cpu(5);
		jprintf("CPU0, CPU1 Reset!\r\n");
    }
    else if (argc == 2){
        if (strcmp("cpu0", argv[1]) == 0){
			err |= jtag_reset_cpu(4);
			jprintf("CPU0 Reset!\r\n");
        }
        else if (strcmp("cpu1", argv[1]) == 0){
			err |= jtag_reset_cpu(5);
			jprintf("CPU1 Reset!\r\n");
        }
		else {
			jprintf("Error : Bad or not command!\r\n%s", chCmd);
			LeaveCriticalSection(&CriticalSection);
			return 0;
		}
    }
	else {
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		LeaveCriticalSection(&CriticalSection);
		return 0;
	}
	LeaveCriticalSection(&CriticalSection);
    return err;
}

UINT32 UnReset_Cpu(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - urc\r\n\
 - urc cpu0\r\n\
 - urc cpu1\r\n";

	UINT32 err = 0;

	EnterCriticalSection(&CriticalSection);
    if (argc == 1){
		err |= jtag_unreset_cpu(4);
		err |= jtag_unreset_cpu(5);
		jprintf("CPU0, CPU1 Unreset!\r\n");
    }
    else if (argc == 2){
        if (strcmp("cpu0", argv[1]) == 0){
			err |= jtag_unreset_cpu(4);
			jprintf("CPU0 Unreset!\r\n");
        }
        else if (strcmp("cpu1", argv[1]) == 0){
			err |= jtag_unreset_cpu(5);
			jprintf("CPU1 Unreset!\r\n");
        }
		else {
			jprintf("Error : Bad or not command!\r\n%s", chCmd);
			LeaveCriticalSection(&CriticalSection);
			return 0;
		}
    }
	else {
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		LeaveCriticalSection(&CriticalSection);
		return 0;
	}
	LeaveCriticalSection(&CriticalSection);
    return err;
}



UINT32 ICprint_old(int argc, char** argv){
	UINT32 nsets;
	UINT32 bsize;

	UINT32	bidw = 2;		// byte index width        
	UINT32	widw;   	// word index width        
	UINT32	cidw;   	// cache index width       
	UINT32	tidw;   	// tag index width         
	UINT32	caw;		// cache address width     

	UINT32 iccfgr;
	UINT32 ncs;
	UINT32 cbs;

	UINT32 adr;
	UINT32 cadr;

	UINT32 i, j, k=0;

	UINT32 rtagmem;
	UINT32 ricache;
	UINT32 rvalid;
	UINT32 rtag;


	UINT8 stalled = 0;

	FILE *stream = NULL;
	if (argc < 2)
	{
		jprintf("error : ex) icp (txt) \n");
		return 0;
	}


	if (fopen_s(&stream, argv[1], "w+t") != 0){
		jprintf("file open fail\n");
	}


	char* memblock;
	memblock = (char*)malloc(MAX_TXT_BUFFER);


	jtag_stall_cpu(gcpu);
	jtag_check_stalled_cpu(gcpu, &stalled);
	if (!stalled){
		jprintf("Fail : stall fail!! Please retry!\n");
        free(memblock);
		return 0;
	}
	Sleep(1000);	//Wait 

	iccfgr = jtag_get_ICCFGR(gcpu);
	ncs = ((iccfgr& SPR_ICCFGR_NCS) >> SPR_ICCFGR_NCS_OFF);
	cbs = ((iccfgr& SPR_ICCFGR_CBS) >> SPR_ICCFGR_CBS_OFF);

	widw = 2 + cbs;
	cidw = ncs;
	caw = cidw + widw + bidw;
	tidw = 32 - caw;

	nsets = (1 << ncs);
	bsize = (16 << cbs);

	for (i = 0; i < nsets; i++){
		adr = 0;
		adr = (i << (widw + bidw));	//cid (10bit)
		adr = adr | (1 << caw);		//Tsel = 1 select tag memory	
		adr = (adr << 2);			//Add BID 2 0bit
		jtag_read32(IC_BIT | adr, &rtagmem, gcpu);
		rvalid = rtagmem & 0x1;
		rtag = (rtagmem >> 1);

		jprintf("%04d: v=%1d, tag=%08x: ", i, rvalid, rtag);
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%04d: v=%1d, tag=%08x: ", i, rvalid, rtag);
		if (rvalid){
			cadr = 0;
			cadr = (i << (widw + bidw));				//cid
			cadr = cadr | (rtag << caw);			//tid
			jprintf("%08x: ", cadr);
			k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x: ", cadr);
			for (j = 0; j < bsize / 4; j++){
				adr = 0;
				adr = adr | (j << bidw);			//wid
				adr = adr | (i << (widw + bidw));	//cid
				adr = adr | (0 << (caw));			//tsel=0; select data memory
				adr = (adr << 2);
				jtag_read32(IC_BIT | adr, &ricache, gcpu);
				jprintf("%08x ", ricache);
				k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x ", ricache);
			}
			

		}
		jprintf("\n");
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "\n");
	}

	fwrite(memblock, sizeof(char), strlen(memblock), stream);

	fclose(stream);
	free(memblock);


	return 0;

}

#define RAM0_BASE 0x04000000
UINT32 Cacheprint(int argc, char** argv,UINT cache){
	UINT32 nsets;
	UINT32 bsize;

	UINT32	bidw = 2;	// byte index width        
	UINT32	widw;   	// word index width        
	UINT32	cidw;   	// cache index width       
	UINT32	tidw;   	// tag index width         
	UINT32	caw;		// cache address width     

	UINT32 iccfgr;
	UINT32 ncs;
	UINT32 cbs;

	UINT32 adr;
	UINT32 cadr;

	UINT32 i, j, k = 0;

	UINT32 rtagmem;
	UINT32 ricache[8];
	UINT32 readmem[8];
	UINT32 rvalid;
	UINT32 rtag;

	UINT32 readTarget = 0;

	UINT8 stalled = 0;

	FILE *stream = NULL;
	if (argc < 2)
	{
		jprintf("error : ex) icp (txt) \n");
		return 0;
	}


	if (fopen_s(&stream, argv[1], "w+t") != 0){
		jprintf("file open fail\n");
	}


	char* memblock;
	memblock = (char*)malloc(MAX_TXT_BUFFER);


	jtag_stall_cpu(gcpu);
	jtag_check_stalled_cpu(gcpu, &stalled);
	if (!stalled){
		jprintf("Fail : stall fail!! Please retry!\n");
        free(memblock);
		return 0;
	}
	Sleep(1000);	//Wait 

	iccfgr = jtag_get_ICCFGR(gcpu);
	ncs = ((iccfgr& SPR_ICCFGR_NCS) >> SPR_ICCFGR_NCS_OFF);
	cbs = ((iccfgr& SPR_ICCFGR_CBS) >> SPR_ICCFGR_CBS_OFF);

	widw = 2 + cbs;
	cidw = ncs;
	caw = cidw + widw + bidw;
	tidw = 32 - caw;

	nsets = (1 << ncs);
	bsize = (16 << cbs);

	for (i = 0; i < nsets; i++){
		adr = 0;
		adr = (i << widw);	//cid (10bit)
		adr = adr | (1 << 14);		//Tsel = 1 select tag memory	
		adr = (adr << 2);			//Add BID 2 0bit
		jtag_read32(IC_BIT | adr, &rtagmem, gcpu);
		rvalid = rtagmem & 0x1;
		rtag = (rtagmem >> 1);

		jprintf("%04d: v=%1d, tag=%08x: ", i, rvalid, rtag);
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%04d: v=%1d, tag=%08x: ", i, rvalid, rtag);
		if (rvalid){
			cadr = 0;
			cadr = (i << (widw + bidw));			//cid
			cadr = cadr | (rtag << caw);			//tid
			jprintf("%08x: ", cadr);
			k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x: ", cadr);

			adr = 0;
			adr = adr | (0);			//wid
			adr = adr | (i << widw);	//cid
			adr = adr | (0 << 14);			//tsel=0; select data memory
			adr = (adr << 2);
			jtag_read_block32(IC_BIT | adr, ricache, bsize / 4, gcpu);
			for (j = 0; j < bsize / 4; j++){
				jprintf("%08x ", ricache[j]);
				k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x ", ricache[j]);
			}
			jprintf(" | ");
			k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, " | ");
			if ((cadr & 0xff000000) == (RAM0_BASE & 0xff000000)){
				readTarget = gcpu - 4;
			}
			else
			{
				readTarget = 3;
			}
			jtag_read_block32(cadr, readmem, bsize / 4, readTarget);
			for (j = 0; j < bsize / 4; j++){
				jprintf("%08x ", readmem[j]);
				k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x ", readmem[j]);
			}
			for (j = 0; j < bsize / 4; j++){
				if (ricache[j] != readmem[j]){
					jprintf(" | X ");
					k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, " | X ");
				}
			}


		}
		jprintf("\n");
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "\n");
	}

	fwrite(memblock, sizeof(char), strlen(memblock), stream);

	fclose(stream);
	free(memblock);
}



UINT32 ICprint(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - icp FILE.TXT\r\n";

	UINT32 err = 0;

	UINT32 nsets;
	UINT32 bsize;

	UINT32	bidw = 2;		// byte index width        
	UINT32	widw;   	// word index width        
	UINT32	cidw;   	// cache index width       
	UINT32	tidw;   	// tag index width         
	UINT32	caw;		// cache address width     

	UINT32 iccfgr;
	UINT32 ncs;
	UINT32 cbs;

	UINT32 adr;
	UINT32 cadr;

	UINT32 i, j, k = 0;

	UINT32 rtagmem;
	UINT32 ricache[8];
	UINT32 readmem[8];
	UINT32 rvalid;
	UINT32 rtag;

	UINT32 readTarget = 0;

	UINT8 stalled = 0;

	FILE *stream = NULL;

	if (argc != 2)
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	if (fopen_s(&stream, argv[1], "w+t") != 0){
		jprintf("file open fail\r\n");
		return 0;
	}

	char* memblock;
	memblock = (char*)malloc(MAX_TXT_BUFFER);

	jtag_stall_cpu(gcpu);
	jtag_check_stalled_cpu(gcpu, &stalled);
	if (!stalled){
		jprintf("Fail : stall fail!! Please retry!\n");
        free(memblock);
		return 0;
	}
	Sleep(1000);	//Wait 

	iccfgr = jtag_get_ICCFGR(gcpu);
	ncs = ((iccfgr& SPR_ICCFGR_NCS) >> SPR_ICCFGR_NCS_OFF);
	cbs = ((iccfgr& SPR_ICCFGR_CBS) >> SPR_ICCFGR_CBS_OFF);

	widw = 2 + cbs;
	cidw = ncs;
	caw = cidw + widw + bidw;
	tidw = 32 - caw;

	nsets = (1 << ncs);
	bsize = (16 << cbs);

	for (i = 0; i < nsets; i++){
		adr = 0;
		adr = (i << widw);	//cid (10bit)
		adr = adr | (1 << 14);		//Tsel = 1 select tag memory	
		adr = (adr << 2);			//Add BID 2 0bit
		err |= jtag_read32(IC_BIT | adr, &rtagmem, gcpu);
		rvalid = rtagmem & 0x1;
		rtag = (rtagmem >> 1);

		jprintf("%04d: v=%1d, tag=%08x: ", i, rvalid, rtag);
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%04d: v=%1d, tag=%08x: ", i, rvalid, rtag);
		if (rvalid){
			cadr = 0;
			cadr = (i << (widw + bidw));			//cid
			cadr = cadr | (rtag << caw);			//tid
			jprintf("%08x: ", cadr);
			k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x: ", cadr);

			adr = 0;
			adr = adr | (0);			//wid
			adr = adr | (i << widw);	//cid
			adr = adr | (0 << 14);			//tsel=0; select data memory
			adr = (adr << 2);
			err |= jtag_read_block32(IC_BIT | adr, ricache, bsize / 4, gcpu);
			for (j = 0; j < bsize / 4; j++){
				jprintf("%08x ", ricache[j]);
				k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x ", ricache[j]);
			}
			jprintf(" | ");
			k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, " | ");
			if ((cadr & 0xff000000) == (RAM0_BASE & 0xff000000)){
				readTarget = gcpu-4;
			}
			else
			{
				readTarget = 3;
			}
			err |= jtag_read_block32(cadr, readmem, bsize / 4, readTarget);
			for (j = 0; j < bsize / 4; j++){
				jprintf("%08x ", readmem[j]);
				k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x ", readmem[j]);
			}
			for (j = 0; j < bsize / 4; j++){
				if (ricache[j] != readmem[j]){
					jprintf(" | X ");
					k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, " | X ");
				}
			}
		}
		jprintf("\n");
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "\n");
	}

	fwrite(memblock, sizeof(char), strlen(memblock), stream);

	fclose(stream);
	free(memblock);

	return err;
}

UINT32 DCprint(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - dcp FILE.TXT\r\n";

	UINT32 err = 0;

	UINT32 nsets;
	UINT32 bsize;

	UINT32	bidw = 2;		// byte index width        
	UINT32	widw;   	// word index width        
	UINT32	cidw;   	// cache index width       
	UINT32	tidw;   	// tag index width         
	UINT32	caw;		// cache address width     

	UINT32 iccfgr;
	UINT32 ncs;
	UINT32 cbs;

	UINT32 adr;
	UINT32 cadr;

	UINT32 i, j, k = 0;

	UINT32 rtagmem;
	UINT32 ricache[8];
	UINT32 readmem[8];
	UINT32 rvalid;
	UINT32 rtag;

	UINT32 readTarget = 0;
	UINT8 stalled = 0;

	FILE *stream = NULL;

	if (argc != 2)
	{
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	if (fopen_s(&stream, argv[1], "w+t") != 0){
		jprintf("file open fail\r\n");
		return 0;
	}

	char* memblock;
	memblock = (char*)malloc(MAX_TXT_BUFFER);

	err |= jtag_stall_cpu(gcpu);
	err |= jtag_check_stalled_cpu(gcpu, &stalled);
	if (!stalled){
		jprintf("Fail : stall fail!! Please retry!\n");
        free(memblock);
		return 0;
	}
	Sleep(1000);	//Wait 

	iccfgr = jtag_get_ICCFGR(gcpu);
	ncs = ((iccfgr& SPR_ICCFGR_NCS) >> SPR_ICCFGR_NCS_OFF);
	cbs = ((iccfgr& SPR_ICCFGR_CBS) >> SPR_ICCFGR_CBS_OFF);

	widw = 2 + cbs;
	cidw = ncs;
	caw = cidw + widw + bidw;
	tidw = 32 - caw;

	nsets = (1 << ncs);
	bsize = (16 << cbs);

	for (i = 0; i < nsets; i++){
		adr = 0;
		adr = (i << widw);	//cid (10bit)
		adr = adr | (3 << 14);		//Tsel = 1 select tag memory	
		adr = (adr << 2);			//Add BID 2 0bit
		err |= jtag_read32(IC_BIT | adr, &rtagmem, gcpu);
		rvalid = rtagmem & 0x1;
		rtag = (rtagmem >> 1);

		jprintf("%04d: v=%1d, tag=%08x: ", i, rvalid, rtag);
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%04d: v=%1d, tag=%08x: ", i, rvalid, rtag);
		if (rvalid){
			cadr = 0;
			cadr = (i << (widw + bidw));			//cid
			cadr = cadr | (rtag << caw);			//tid
			jprintf("%08x: ", cadr);
			k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x: ", cadr);

			adr = 0;
			adr = adr | (0);			//wid
			adr = adr | (i << widw);	//cid
			adr = adr | (2 << 14);			//tsel=0; select data memory
			adr = (adr << 2);
			err |= jtag_read_block32(IC_BIT | adr, ricache, bsize / 4, gcpu);
			for (j = 0; j < bsize / 4; j++){
				jprintf("%08x ", ricache[j]);
				k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x ", ricache[j]);
			}

			jprintf(" | ");
			k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, " | ");

			if ((cadr & 0xff000000) == (RAM0_BASE & 0xff000000)){
				readTarget = gcpu-4;
			}
			else
			{
				readTarget = 3;
			}
			err |= jtag_read_block32(cadr, readmem, bsize / 4, readTarget);
			for (j = 0; j < bsize / 4; j++){
				jprintf("%08x ", readmem[j]);
				k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "%08x ", readmem[j]);
			}
			for (j = 0; j < bsize / 4; j++){
				if (ricache[j] != readmem[j]){
					jprintf(" | X ");
					k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, " | X ");
				}
			}
		}
		jprintf("\n");
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "\n");
	}

	fwrite(memblock, sizeof(char), strlen(memblock), stream);

	fclose(stream);
	free(memblock);

	return err;
}

UINT32 TBprint(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - tbp FILE.TXT\r\n";

	UINT32 err = 0;

	FILE *stream = NULL;
	UINT32 optionNum = 0;
	UINT8 module = 4;

	if (argc < 3)
	{
		jprintf("error : ex) tbp (cpu0,cpu1) (file) (option=0,1,2...) \n");
		return 0;
	}

	if (strcmp("cpu0", argv[1]) == 0)	module = 4;
	else if (strcmp("cpu1", argv[1]) == 0)	module = 5;
	else {
		jprintf("Error : option choose (cpu0,cpu1)'\n");
		jprintf("Error : ex) te(cpu0,cpu1) \n");
	}

	if (fopen_s(&stream, argv[2], "w+t") != 0){
		jprintf("file open fail\n");
		return 0;
	}
	if (argc == 3){
		optionNum = atoi(argv[2]);
		jprintf("option number:%d\n", optionNum);
	}
	else
		optionNum = atoi(argv[3]);

	char* memblock;
	memblock = (char*)malloc(100 * 256 * 4);

	if (optionNum)	err |= jtag_read_tb_new(module, (UINT8*)memblock);
	else			err |= jtag_read_tb(module, (UINT8*)memblock);

	fwrite(memblock, sizeof(char), strlen(memblock), stream);

	fclose(stream);
	free(memblock);
	return 0;
}

UINT32 TBenable(int argc, char** argv){
	UINT32 err = 0;
	UINT8 module=4;
	if (strcmp("cpu0", argv[1]) == 0)	module = 4;
	else if (strcmp("cpu1", argv[1]) == 0)	module = 5;
	else {
		jprintf("Error : option choose (cpu0,cpu1)'\n");
		jprintf("Error : ex) te(cpu0,cpu1) \n");
		return 0;
	}
	err |= jtag_enable_tb(module);
	return err;
}

UINT32 TBdisable(int argc, char** argv){
	UINT32 err = 0;
	UINT8 module = 4;
	if (strcmp("cpu0", argv[1]) == 0)	module = 4;
	else if (strcmp("cpu1", argv[1]) == 0)	module = 5;
	else {
		jprintf("Error : option choose (cpu0,cpu1)'\n");
		jprintf("Error : ex) te(cpu0,cpu1) \n");
		return 0;
	}
	err |= jtag_disable_tb(module);
	return err;
}

UINT32 RunScript(int argc, char** argv){
	streampos size;
	UINT32 ret;
	
	char cmd[1024];
	char fileName[512];
	char *token = NULL;
	char *p = NULL;
	
    UINT32 test_round = 1; //default is 1 time
    UINT32 err_count=0;

	if (argc == 1){
		jprintf("Use (default:script.ctg) file : \n");
		strcpy(fileName, "script.cfg");
	}
	else if (argc == 2){
		jprintf("Use (%s) file \n",argv[1]);
		strcpy(fileName, argv[1]);
	}
    else if (argc == 3){
        jprintf("Test mode \n", argv[1]);
        if (strcmp("test", argv[1]) == 0){
            strcpy(fileName, "script.cfg");
            test_round = atoi(argv[2]);
        }
        else{
            jprintf("this is for test mode ex) run test 100\n");
			return 0;
        }
    }
	else{
		jprintf("error : ex) run (default==script.cfg)\n");
		return 0;
	}

    UINT32 i = 0;
	ifstream linkfile(fileName);
	if (linkfile.is_open())
	{
        jprintf("test_round %d\n", test_round);
        for (i=0;i<test_round;i++){
            jprintf("Round %d, Errcnt %d \n", i, err_count);
            linkfile.seekg(0, ifstream::beg);
		while (!linkfile.eof()){
			linkfile.getline(cmd, 1024);
//			cout << cmd << endl;

			if ((cmd[0] == '#') || ((cmd[0] == '/') && (cmd[1] == '/')) || ((cmd[0] == 'r') && (cmd[1] == 'u') && (cmd[2] == 'n'))){
			//	cout <<"skip cmd"<< endl;
			}
			else{

				strtok_s(cmd, "#", &token);
				p=strstr(cmd, "//");
				if (p != NULL){
					*p = 0;
					p = NULL;
				}
				p = strstr(cmd, "run");
				if (p != NULL){
					*p = 0;
					p = NULL;
				}
                    cout << "cmd>" << cmd << endl;
//				cout << strlen(cmd) << endl;
				if (strlen(cmd) != 0){
					ret = _DoCommand((char*)cmd);
                        if (ret != 0){
                            err_count++;
                            jprintf("Err:%08x\n", ret);
                        }
                    }
				}
			}
		}
		linkfile.close();
	}
	else
	{
		cout << "Unable to open link.cfg file";
		return 0;
	}
	return 0;
}

UINT32 Exit(int argc, char** argv){

	EN673_JTAG_CLOSE();
	exit(0);
	return 0;
}


UINT32 JtagUartRead(UINT32 addr, UINT32 size, char* fileName, char* fileMode, UINT32 module){
	UINT32 err = 0;
	FILE * pFile;
	UINT8 * memblock = NULL;
	UINT8 * memblock_t = NULL;
	UINT32* memblock32 = NULL;
	UINT32* memblock32_t = NULL;
	UINT32 i;
	UINT32 size_adj = 0;
	UINT32 retry = 0, left = 0;

	size_adj = ((size + 4) >> 2) << 2;
	memblock = (UINT8*)malloc(size_adj + 4);
	memblock_t = (UINT8*)malloc(size_adj + 4);
	retry = (size_adj) / (READBUFFER);
	left = (size_adj) % (READBUFFER);
	for (i = 0; i < retry; i++){
		err |= jtag_read_block32(addr + (READBUFFER * i), (UINT32*)(memblock + (READBUFFER * i)), (READBUFFER / 4), module);
		//		cout << "*";
	}
	if (left)
		err |= jtag_read_block32(addr + (READBUFFER * i), (UINT32*)(memblock + (READBUFFER * i)), (left / 4) + 1, module);
	//	cout << "END";

	memblock32 = (UINT32*)memblock;
	memblock32_t = (UINT32*)memblock_t;
	for (i = 0; i < (size_adj / 4) + 1; i++){
		*(memblock32_t + i) = jtag_htonl((UINT32)*(memblock32 + i));
	}
	memblock_t = (UINT8*)memblock32_t;
	*(memblock_t + size) = NULL; //terminate string
	//	hexDump("DUMP", memblock_t, size);
	cout << memblock_t;


	pFile = fopen(fileName, fileMode);
	fwrite(memblock_t, sizeof(char), size, pFile);
	fclose(pFile);

	if (memblock) free(memblock);
	if (memblock_t) free(memblock_t);

	return err;
}

#define SIZEOFBUF (1024*1024 )
#define OFFSET 0x0
UINT32 buf[SIZEOFBUF];
UINT32 rbuf[SIZEOFBUF];
#define JTAG_UART_BUF_BASE_ISP 0x0f800000
#define JTAG_UART_BUF_BASE_IP0 0x0f900000
#define JTAG_UART_BUF_BASE_IP1 0x0fa00000
#define JTAG_UART_BUF_POSI_ISP 0xf83000c0
#define JTAG_UART_BUF_POSI_IP0 0xf83000c4
#define JTAG_UART_BUF_POSI_IP1 0xf83000c8

UINT32 JtagUartPrint(UINT32 first,UINT32 uart_buf_base,UINT32 uart_buf_posi){
	UINT32 err = 0;
	UINT32 reg=0;
	UINT32 readsize = 0;
	static UINT32 readpos;
	UINT32 size = 0;
	//Read *address 
	//reg = example_read_register(3, uart_buf_posi);
	err = example_read_register(3, uart_buf_posi, &reg);

	reg = reg & 0xfffffffc; 
	
//	jprintf("0x%.8x :   reg = %.8x \n", JTAG_UART_BUF_POSI, reg);
	//Calc read pos 
	if (reg == 0){
		cout << "DATA IS NOT PREPARED\n";
		return 1;
	}
	else{
		if (first == 1) readpos = reg;
	}
	
	if (readpos == reg){
//		cout << "NODATA" << "\n";
		return 0;
	}else if (readpos > reg){	//Ring Buffer Over
		readsize = (uart_buf_base + 0x100000) - readpos;
        err |= JtagUartRead(readpos, readsize, "uart.txt", "ab", JTAG_COMMON_MODULE_IDX);
		readpos = uart_buf_base;
		readsize = reg - readpos;
		err |= JtagUartRead(readpos, readsize, "uart.txt", "ab", JTAG_COMMON_MODULE_IDX);
		readpos = reg;
	}
	else{
		readsize = reg - readpos;
		err |= JtagUartRead(readpos, readsize, "uart.txt", "ab", JTAG_COMMON_MODULE_IDX);
		readpos = reg;
	}

	return err;
}

#define ToRadian( degree )  ( (degree) * (PI / 180.0f) )
#define ToDegree( radian )  ( (radian) * (180.0f / PI) )





const string COLOR_NAME[VECTORSCOPE_COLOR_POINT] = { "Mg", "R", "Y", "G", "Cy", "B" };
Mat drawBackgroundVectorscope(Mat img){
    Scalar color(0, 128, 255); //RGB(255, 128, 0)
    Scalar color2(0x3a, 0x5f, 0xa6); //RGB(255, 128, 0)
    int thickness = 1;
    line(img, Point(VECTORSCOPE_INTERVAL_WIDTH, VECTORSCOPE_CENTER_Y), Point(VECTORSCOPE_CENTER_X - 15, VECTORSCOPE_CENTER_Y), color, thickness);
    line(img, Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_INTERVAL_HEIGHT), Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_CENTER_Y - 15), color, thickness);
    line(img, Point(VECTORSCOPE_CENTER_X + 15, VECTORSCOPE_CENTER_Y), Point(VECTORSCOPE_BACKGROUND_WIDTH - VECTORSCOPE_INTERVAL_WIDTH, VECTORSCOPE_CENTER_Y), color, thickness);
    line(img, Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_CENTER_Y + 15), Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_BACKGROUND_HEIGHT - VECTORSCOPE_INTERVAL_HEIGHT), color, thickness);
    line(img, Point(VECTORSCOPE_CENTER_X - 5, VECTORSCOPE_CENTER_Y), Point(VECTORSCOPE_CENTER_X + 5, VECTORSCOPE_CENTER_Y), color, thickness);
    line(img, Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_CENTER_Y - 5), Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_CENTER_Y + 5), color, thickness);
   
    circle(img, Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_CENTER_Y), 20, color, thickness, CV_AA);
    circle(img, Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_CENTER_Y), 40, color, thickness, CV_AA);
    circle(img, Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_CENTER_Y), 60, color, thickness, CV_AA);
    circle(img, Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_CENTER_Y), 80, color, thickness, CV_AA);
    circle(img, Point(VECTORSCOPE_CENTER_X, VECTORSCOPE_CENTER_Y), 100, color, thickness, CV_AA);

    line(img, Point(VECTORSCOPE_CENTER_X + (20 * cos(ToRadian(30))), VECTORSCOPE_CENTER_Y - (20 * sin(ToRadian(30)))), Point(VECTORSCOPE_CENTER_X + (100 * cos(ToRadian(30))), VECTORSCOPE_CENTER_Y - (100 * sin(ToRadian(30)))), color2, thickness, CV_AA);
    line(img, Point(VECTORSCOPE_CENTER_X - (20 * cos(ToRadian(30))), VECTORSCOPE_CENTER_Y - (20 * sin(ToRadian(30)))), Point(VECTORSCOPE_CENTER_X - (100 * cos(ToRadian(30))), VECTORSCOPE_CENTER_Y - (100 * sin(ToRadian(30)))), color2, thickness, CV_AA);
    line(img, Point(VECTORSCOPE_CENTER_X + (20 * cos(ToRadian(60))), VECTORSCOPE_CENTER_Y - (20 * sin(ToRadian(60)))), Point(VECTORSCOPE_CENTER_X + (100 * cos(ToRadian(60))), VECTORSCOPE_CENTER_Y - (100 * sin(ToRadian(60)))), color2, thickness, CV_AA);
    line(img, Point(VECTORSCOPE_CENTER_X - (20 * cos(ToRadian(60))), VECTORSCOPE_CENTER_Y - (20 * sin(ToRadian(60)))), Point(VECTORSCOPE_CENTER_X - (100 * cos(ToRadian(60))), VECTORSCOPE_CENTER_Y - (100 * sin(ToRadian(60)))), color2, thickness, CV_AA);
    line(img, Point(VECTORSCOPE_CENTER_X + (20 * cos(ToRadian(30))), VECTORSCOPE_CENTER_Y + (20 * sin(ToRadian(30)))), Point(VECTORSCOPE_CENTER_X + (100 * cos(ToRadian(30))), VECTORSCOPE_CENTER_Y + (100 * sin(ToRadian(30)))), color2, thickness, CV_AA);
    line(img, Point(VECTORSCOPE_CENTER_X - (20 * cos(ToRadian(30))), VECTORSCOPE_CENTER_Y + (20 * sin(ToRadian(30)))), Point(VECTORSCOPE_CENTER_X - (100 * cos(ToRadian(30))), VECTORSCOPE_CENTER_Y + (100 * sin(ToRadian(30)))), color2, thickness, CV_AA);
    line(img, Point(VECTORSCOPE_CENTER_X + (20 * cos(ToRadian(60))), VECTORSCOPE_CENTER_Y + (20 * sin(ToRadian(60)))), Point(VECTORSCOPE_CENTER_X + (100 * cos(ToRadian(60))), VECTORSCOPE_CENTER_Y + (100 * sin(ToRadian(60)))), color2, thickness, CV_AA);
    line(img, Point(VECTORSCOPE_CENTER_X - (20 * cos(ToRadian(60))), VECTORSCOPE_CENTER_Y + (20 * sin(ToRadian(60)))), Point(VECTORSCOPE_CENTER_X - (100 * cos(ToRadian(60))), VECTORSCOPE_CENTER_Y + (100 * sin(ToRadian(60)))), color2, thickness, CV_AA);

    float dRadian;
    int x, y;
    for (int i = 0; i < VECTORSCOPE_COLOR_POINT; i++) {
        dRadian = (float)ToRadian(-COLOR_DEGREE[i]);
        x = (int)(COLOR_SCALE[i] * (BIT_FREQUENCY / 2) * cos(dRadian));
        y = (int)(COLOR_SCALE[i] * (BIT_FREQUENCY / 2) * sin(dRadian));

        // 평행이동
        x += VECTORSCOPE_CENTER_X;
        y += VECTORSCOPE_CENTER_Y;

        // 사각형
        Rect rc(x-3, y-3, 6, 6);
        rectangle(img, rc, color, thickness);
       
        if (cos(dRadian) < 0)	putText(img, COLOR_NAME[i], Point(x - 6, y - 6), FONT_HERSHEY_SIMPLEX, 0.3, color, thickness, CV_AA);
        else					putText(img, COLOR_NAME[i], Point(x + 6, y + 3), FONT_HERSHEY_SIMPLEX, 0.3, color, thickness, CV_AA);
    }
    return img;
}

Mat drawBackgroundVectorscope(Mat img, int width, int height){
	Scalar color(0, 128, 255); //RGB(255, 128, 0)
	Scalar color2(0x3a, 0x5f, 0xa6); //RGB(255, 128, 0)

	int w2 = width / 2;
	int h2 = height / 2;
	int thickness = 1;
	float c1 = ((float)20  / 256)*width; // Circle1
	float c2 = ((float)40  / 256)*width; // Circle2
	float c3 = ((float)60  / 256)*width; // Circle3
	float c4 = ((float)80  / 256)*width; // Circle4
	float c5 = ((float)100 / 256)*width; // Circle5

	// Horizontal
	line(img, Point(VECTORSCOPE_INTERVAL, h2), Point(w2 - 15, h2), color, thickness);
	line(img, Point(w2 + 15, h2), Point(width - VECTORSCOPE_INTERVAL, h2), color, thickness);
	line(img, Point(w2 - 5, h2), Point(w2 + 5, h2), color, thickness);
	// Vertical
	line(img, Point(w2, VECTORSCOPE_INTERVAL), Point(w2, h2 - 15), color, thickness);
	line(img, Point(w2, h2 + 15), Point(w2, height - VECTORSCOPE_INTERVAL), color, thickness);
	line(img, Point(w2, h2 - 5), Point(w2, h2 + 5), color, thickness);

	// Circle
	circle(img, Point(w2, h2), c1, color, thickness, CV_AA);
	circle(img, Point(w2, h2), c2, color, thickness, CV_AA);
	circle(img, Point(w2, h2), c3, color, thickness, CV_AA);
	circle(img, Point(w2, h2), c4, color, thickness, CV_AA);
	circle(img, Point(w2, h2), c5, color, thickness, CV_AA);

	double cos30 = cos(ToRadian(30));
	double cos60 = cos(ToRadian(60));
	double sin30 = sin(ToRadian(30));
	double sin60 = sin(ToRadian(60));

	line(img, Point(w2 + (c1 * cos30), h2 - (c1 * sin30)), Point(w2 + (c5 * cos30), h2 - (c5 * sin30)), color2, thickness, CV_AA);
	line(img, Point(w2 - (c1 * cos30), h2 - (c1 * sin30)), Point(w2 - (c5 * cos30), h2 - (c5 * sin30)), color2, thickness, CV_AA);
	line(img, Point(w2 + (c1 * cos60), h2 - (c1 * sin60)), Point(w2 + (c5 * cos60), h2 - (c5 * sin60)), color2, thickness, CV_AA);
	line(img, Point(w2 - (c1 * cos60), h2 - (c1 * sin60)), Point(w2 - (c5 * cos60), h2 - (c5 * sin60)), color2, thickness, CV_AA);
	line(img, Point(w2 + (c1 * cos30), h2 + (c1 * sin30)), Point(w2 + (c5 * cos30), h2 + (c5 * sin30)), color2, thickness, CV_AA);
	line(img, Point(w2 - (c1 * cos30), h2 + (c1 * sin30)), Point(w2 - (c5 * cos30), h2 + (c5 * sin30)), color2, thickness, CV_AA);
	line(img, Point(w2 + (c1 * cos60), h2 + (c1 * sin60)), Point(w2 + (c5 * cos60), h2 + (c5 * sin60)), color2, thickness, CV_AA);
	line(img, Point(w2 - (c1 * cos60), h2 + (c1 * sin60)), Point(w2 - (c5 * cos60), h2 + (c5 * sin60)), color2, thickness, CV_AA);

	float dRadian;
	int x, y;
	for (int i = 0; i < VECTORSCOPE_COLOR_POINT; i++) {
		dRadian = ToRadian(-COLOR_DEGREE[i]);
		x = (int)(COLOR_SCALE[i] * w2 * cos(dRadian));
		y = (int)(COLOR_SCALE[i] * h2 * sin(dRadian));

		// 평행이동
		x += w2;
		y += h2;

		// 사각형
		Rect rc(x - 3, y - 3, 6, 6);
		rectangle(img, rc, color, thickness);

		if (cos(dRadian) < 0)	putText(img, COLOR_NAME[i], Point(x - 6, y - 6), FONT_HERSHEY_SIMPLEX, 0.3, color, thickness, CV_AA);
		else					putText(img, COLOR_NAME[i], Point(x + 6, y + 3), FONT_HERSHEY_SIMPLEX, 0.3, color, thickness, CV_AA);
	}

	return img;
}

const string WAVE_GRAD[WAVE_GRADATION_POINT] = {"225", "200", "175", "150", "125", "100" , "75", "50" , "25"};
#define L_DEPTH 25
Mat drawBackgroundWaveform(Mat img){
    Scalar color(0, 128, 255); //RGB(255, 128, 0)
    int thickness = 1;
    for (int i = 0; i<WAVE_GRADATION_POINT; i++){
        // line
        line(img, Point(WAVEFORM_INTERVAL_WIDTH, 30 + L_DEPTH * i), Point(WAVEFORM_BACKGROUND_WIDTH - WAVEFORM_INTERVAL_WIDTH, 30 + L_DEPTH * i), color, thickness);
        // Text
        if (i < 6)	putText(img, WAVE_GRAD[i], Point(1, 30 + L_DEPTH * i - 6), FONT_HERSHEY_SIMPLEX, 0.3, color, thickness);
        else		putText(img, WAVE_GRAD[i], Point(7, 30 + L_DEPTH * i - 6), FONT_HERSHEY_SIMPLEX, 0.3, color, thickness);
        putText(img, WAVE_GRAD[i], Point(WAVEFORM_BACKGROUND_WIDTH - WAVEFORM_INTERVAL_WIDTH + 1, 30 + L_DEPTH * i - 6), FONT_HERSHEY_SIMPLEX, 0.3, color, thickness);
    }
    return img;
}

#define NOMAL_WIDTH 320
#define NOMAL_RATE  ((float)1920/(float)1080)

UINT getMacbPos_X(Mat img,UINT x){
	UINT width = img.cols;
	UINT height = img.rows;
	UINT center_x = width / 2;
	UINT block_len = (center_x / 2) / 3;
	UINT half_block_len = block_len / 2;
	UINT pos_x;
	float offset;
	float rate;
	rate = ((float)width / (float)height);
	offset = rate / NOMAL_RATE;
	UINT start_x = center_x - ((float)(center_x / 2)*offset);
	pos_x = (UINT)((start_x + (block_len*offset*x)) + half_block_len); // x

	return pos_x;
}

UINT getMacbPos_Y(Mat img, UINT y){
	UINT width = img.cols;
	UINT height = img.rows;
	UINT center_x = width / 2;
	UINT center_y = height / 2;
	UINT block_len = (center_x / 2) / 3;
	UINT half_block_len = block_len / 2;
	UINT pos_y;
	UINT start_y = center_y - (2 * block_len);
	pos_y = (UINT)((start_y + (block_len*y)) + half_block_len); // y

	return pos_y;
}

Mat drawMacbethChartPosition(Mat img){
	Scalar color(0, 128, 255);
	UINT thickness = 1;
	UINT radius = (img.cols) / NOMAL_WIDTH;

	int i = 0;
	int j = 0;
	for (i = 0; i < 6; i++){
		for (j = 0; j < 4; j++){
			circle(img, Point(getMacbPos_X(img, i), getMacbPos_Y(img, j)), radius * 3, color, thickness, CV_AA);
		}
	}
	return img;
}

#define BLUE    0
#define GREEN   1
#define RED     2
//(img ,bgr, UINT x, UINT y, color )
//      (0,0) (0,1) (0,2) (0,3) (0,4) (0,5) 
//      (1,0) (1,1) (1,2) (1,3) (1,4) (1,5)    
//      (2,0) (2,1) (2,2) (2,3) (2,4) (2,5)
//      (3,0) (3,1) (3,2) (3,3) (3,4) (3,5)
UINT getRGB(Mat img, Mat* bgr, UINT x, UINT y, UINT color)
{
	return  bgr[color].data[(img.cols*getMacbPos_Y(img, y)) + getMacbPos_X(img, x)];;
}

#define ISP_BYGAINN		0xE803		
#define ISP_BYGAINP		0xE802		
#define ISP_RYGAINN		0xE801		
#define ISP_RYGAINP		0xE800		

#define ISP_BYHUEN      0xE903	
#define ISP_BYHUEP		0xE902		
#define ISP_RYHUEN		0xE901	
#define ISP_RYHUEP		0xE900		


BYTE get_control(UINT reg){
	UINT read;
	UINT regi = (reg & 0xff00) >> 6;
	UINT offs = (reg & 0xf) * 8;
	jtag_read32(ISP_REG_BASE + regi, &read, JTAG_WB0_MODULE_IDX);
	return (read >> offs) & 0xff;
}

void set_control(UINT reg, BYTE data){
	UINT read;
	UINT write;
	UINT regi = (reg & 0xff00) >> 6;
	UINT offs = (reg & 0xf) * 8;
	jtag_read32(ISP_REG_BASE + regi, &read, JTAG_WB0_MODULE_IDX);
	write = (read & ~(0xff << offs)) | (data << offs);
	jtag_write32(ISP_REG_BASE + regi, write, JTAG_WB0_MODULE_IDX);
}

//#define JPEG_VIEW_ADDR       0xF0010a00
//#define JTAG_VIEW_ADDRSIZEP  0xF0010a04
//#define JTAG_VIEW_FLAG       0xF0010a08

#define JPEG_VIEW_ADDR       0xF0010000
#define JTAG_VIEW_ADDRSIZEP  0xF0010004
#define JTAG_VIEW_FLAG       0xF0010008
#define FLAG_INTRO  9
#define FLAG_OUTRO  0
#define SIZE_ZERO   0
//#define JPEG_VIEW_ADDR       0x0a000004
//#define JTAG_VIEW_ADDRSIZEP  0x0a000000

#define READ_JTAG_BUF        (1024*8)
#define READ_JTAG_SIZE        (1024*1024)
const char* source_window = "Source image";
const char* warp_window = "Warp";
const char* warp_rotate_window = "Warp + Rotate";

UINT32 GetJpegImageSize(void)
{
	UINT32 size		= 0;
	UINT32 size_adj = 0;
	UINT32 addrsizep = JTAG_VIEW_ADDRSIZEP;
	UINT32 temp		= 0;
	UINT32 err		= 0;
		temp = SIZE_ZERO;
		err |= jtag_read_block32(addrsizep, &size, 1, JTAG_COMMON_MODULE_IDX);
		if (err) {
			jprintf("Read Fail!");
		}
		jprintf("size:%d\n", size);
	
		if (size > READ_JTAG_SIZE){
			jprintf("wrong size %d", size);
			err |= jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX);
		}
		size_adj = ((size + 4) >> 2) << 2;
	return size;
}

UINT32 GetJpegImage(void* buf){
	UINT8 * memblock = NULL;
	UINT32* memblock32 = NULL;
	UINT32 i;
	UINT32 size_adj = 0;
	UINT32 retry = 0, left = 0;
	UINT32 addr;
	UINT32 addrview = JPEG_VIEW_ADDR;
	UINT32 addrsizep = JTAG_VIEW_ADDRSIZEP;
	UINT32 addrflag = JTAG_VIEW_FLAG;
	UINT32 size = 0;
	UINT32 module = 3;
	UINT32 temp = 0;
	UINT32 err = 0;

	temp = FLAG_INTRO;
	err |= jtag_write_block32(addrflag, &temp, 1, JTAG_COMMON_MODULE_IDX); // SET Flag to 9 for JTAG VIEW MODE
	temp = SIZE_ZERO;
	err |= jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX);
	
	

	

		temp = SIZE_ZERO;
		err |= jtag_read_block32(addrsizep, &size, 1, JTAG_COMMON_MODULE_IDX);
		if (err) {
			jprintf("Read Fail!");
			
		}
		//      jprintf("size:%d\n", size);
		if (size == NULL) return 0;
		if (size > READ_JTAG_SIZE){
			jprintf("wrong size %d", size);
			err |= jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX);
			//cvReleaseMat(&matJpg2);
		
		}

		err |= jtag_read_block32(addrview, &addr, 1, JTAG_COMMON_MODULE_IDX);


		size_adj = ((size + 4) >> 2) << 2;
		memblock = (UINT8*)malloc(size_adj + 4);

		retry = (size_adj) / (READ_JTAG_BUF);
		left = (size_adj) % (READ_JTAG_BUF);

		for (i = 0; i < retry; i++)
			err |= jtag_read_block32(addr + (READ_JTAG_BUF * i), (UINT32*)(memblock + (READ_JTAG_BUF * i)), (READ_JTAG_BUF / 4), module);

		if (left)
			err |= jtag_read_block32(addr + (READ_JTAG_BUF * i), (UINT32*)(memblock + (READ_JTAG_BUF * i)), (left / 4) + 1, module);

		err |= jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX);

		if (err){
			if (err == ERR_MPSSE_BURST_READ_STREAM) return 0;

			jprintf("JTAG Error! (%x) \n", err);
			if (memblock) free(memblock); memblock = NULL;
			return 0;
		}
		memblock32 = (UINT32*)memblock;

		for (i = 0; i < (size_adj / 4) + 1; i++){
			*(((UINT32*)buf) + i) = htonl((UINT32)*(memblock32 + i));
		}
		if (memblock) free(memblock); memblock = NULL;
		Mat matJpg2(1, READ_JTAG_SIZE, CV_8UC1);
		Mat src;
		src = imdecode(matJpg2, 1);
		imshow("Image", src);

	return size_adj;
}

UINT32 loopflag = 0;

UINT32 EscapeLoop(){
//	printf("Loop Flag return to %d\n", loopflag);
	return loopflag;
}
void EscapeLoopSet(UINT32 set){
//	printf("Loop Flag Set to %d\n", set);
	loopflag = set;
}


UINT32 Test(int argc, char** argv){

    UINT8 * memblock = NULL;
    UINT32* memblock32 = NULL;
    UINT32 i;
    UINT32 size_adj = 0;
    UINT32 retry = 0, left = 0;
    UINT32 addr;
    UINT32 addrview = JPEG_VIEW_ADDR;
    UINT32 addrsizep = JTAG_VIEW_ADDRSIZEP;
    UINT32 addrflag = JTAG_VIEW_FLAG;
    UINT32 size = 0;
    UINT32 module = 3;
    UINT32 temp = 0;
    UINT32 err = 0;

//    CvMat *matJpg2 = cvCreateMat(1, 500*1024, CV_8UC1);
    Mat matJpg2(1, READ_JTAG_SIZE, CV_8UC1);
    Mat src;
    Mat yuv;
    Mat ch_yuv[3];
    Mat ch_bgr[3];
    CWaveformImgOperation vec;
    Mat img(BIT_FREQUENCY, BIT_FREQUENCY, CV_8UC3);
    Mat img_filp;
    Mat wimg_r(WAVEFORM_HEIGHT, WAVEFORM_WIDTH, CV_8UC4);
    Mat img_filp_r;
    Mat wimg_g(WAVEFORM_HEIGHT, WAVEFORM_WIDTH, CV_8UC4);
    Mat img_filp_g;
    Mat wimg_b(WAVEFORM_HEIGHT, WAVEFORM_WIDTH, CV_8UC4);
    Mat img_filp_b;

    if (argc == 2){
        addrsizep = strtoul(argv[1], NULL, 16);
        addr = addrsizep + 4;
    }

	EnterCriticalSection(&CriticalSection);
	err |= tap_reset();
	err |= tap_set_ir_debug();
	LeaveCriticalSection(&CriticalSection);

    temp = FLAG_INTRO;
    err |= jtag_write_block32(addrflag, &temp, 1, JTAG_COMMON_MODULE_IDX); // SET Flag to 9 for JTAG VIEW MODE
	
	temp = SIZE_ZERO;
    err |= jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX);
	
    Sleep(500);

    while (1){
		EnterCriticalSection(&CriticalSection);
		err |= tap_reset();
		err |= tap_set_ir_debug();
		LeaveCriticalSection(&CriticalSection);
#ifdef MEASURE_TIME
        ULONG64 measure_time = 0;
        LONG64 diff_time = 0;
        measure_time = myclock();
#endif
		if (memblock) free(memblock); memblock = NULL;

        if (EN673_JTAG_ESC_key_checker()) break;

        temp = SIZE_ZERO;
		/*
		uint flag;
		err |= jtag_read_block32(addrflag, &flag, 1, JTAG_COMMON_MODULE_IDX);
		if (err) {
			jprintf("Read Fail!");
			break;
		}
		//      jprintf("size:%d\n", size);
		if (flag == NULL) continue;
		*/



        err |= jtag_read_block32(addrsizep, &size, 1, JTAG_COMMON_MODULE_IDX);
		if (err) {
			jprintf("Read Fail!");
			break;
		}
  //      jprintf("size:%d\n", size);
        if (size == NULL) continue;
        if (size > READ_JTAG_SIZE){
            jprintf("wrong size %d", size);
			err |= jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX);
            //cvReleaseMat(&matJpg2);
            break;
        }

        err |= jtag_read_block32(addrview, &addr, 1, JTAG_COMMON_MODULE_IDX);


        size_adj = ((size + 4) >> 2) << 2;
        memblock = (UINT8*)malloc(size_adj + 4);
 
        retry = (size_adj) / (READ_JTAG_BUF);
        left = (size_adj) % (READ_JTAG_BUF);

        for (i = 0; i < retry; i++)
            err |= jtag_read_block32(addr + (READ_JTAG_BUF * i), (UINT32*)(memblock + (READ_JTAG_BUF * i)), (READ_JTAG_BUF / 4), module);

        if (left)
            err |= jtag_read_block32(addr + (READ_JTAG_BUF * i), (UINT32*)(memblock + (READ_JTAG_BUF * i)), (left / 4) + 1, module);
       
        err |= jtag_write_block32(addrsizep, &temp, 1, JTAG_COMMON_MODULE_IDX);

        if (err){
            if (err == ERR_MPSSE_BURST_READ_STREAM) continue;

            jprintf("JTAG Error! (%x) \n", err);
			if (memblock) free(memblock); memblock = NULL;
            break;
        }
        memblock32 = (UINT32*)memblock;

        for (i = 0; i < (size_adj / 4) + 1; i++){
            *((UINT32*)(matJpg2.data) + i) = htonl((UINT32)*(memblock32 + i));
        }
		
        src = imdecode(matJpg2, 1);
		if (src.data == NULL) continue;

//        jprintf("size %d %d\n", src.cols, src.rows );
/////////////////////////////
#define VECTORSCOPE
#ifdef VECTORSCOPE
        cvtColor(src, yuv, COLOR_BGR2YCrCb);
        split(yuv, ch_yuv);
        split(src, ch_bgr);
		/*
		jprintf("R-%d G-%d B-%d\n", getRGB(src, ch_bgr, 0, 0, RED), getRGB(src, ch_bgr, 0, 0, GREEN), getRGB(src, ch_bgr, 0, 0, BLUE));
		jprintf("%02x %02x %02x %02x\n", get_control(ISP_BYGAINN), get_control(ISP_BYGAINP), get_control(ISP_RYGAINN), get_control(ISP_RYGAINP));
		jprintf("%02x %02x %02x %02x\n", get_control(ISP_BYHUEN), get_control(ISP_BYHUEP), get_control(ISP_RYHUEN), get_control(ISP_RYHUEP));
		
		set_control(ISP_BYGAINN, get_control(ISP_BYGAINN) + 1);
		set_control(ISP_BYGAINP, get_control(ISP_BYGAINP) + 1);
		set_control(ISP_RYGAINN, get_control(ISP_RYGAINN) + 1);
		set_control(ISP_RYGAINP, get_control(ISP_RYGAINP) + 1);
		set_control(ISP_BYHUEN, get_control(ISP_BYHUEN) + 1);
		set_control(ISP_BYHUEP, get_control(ISP_BYHUEP) + 1);
		set_control(ISP_RYHUEN, get_control(ISP_RYHUEN) + 1);
		set_control(ISP_RYHUEP, get_control(ISP_RYHUEP) + 1);
		*/
        vec.InitWaveformDraw(src.cols, src.rows);
		if (ch_yuv[2].data == NULL) continue;
		if (ch_yuv[1].data == NULL) continue;
        BYTE *pVectorImage = vec.GetVectorScope(ch_yuv[2].data, ch_yuv[1].data, src.cols, src.rows, 1);
        
        img.data = pVectorImage;
        
        flip(img, img_filp, 0);// vertical flip
		Mat dst;
		resize(img_filp, dst, Size(VECTORSCOPE_BACKGROUND_WIDTH * (1.5), VECTORSCOPE_BACKGROUND_HEIGHT * (1.5)));
		//drawBackgroundVectorscope(dst);
		drawBackgroundVectorscope(dst, VECTORSCOPE_BACKGROUND_WIDTH * (1.5), VECTORSCOPE_BACKGROUND_WIDTH * (1.5));
        imshow("Vectorscope", img_filp);

		if (ch_bgr[2].data == NULL) continue;
        BYTE *pWaveformImage_r = vec.GetWaveform(ch_bgr[2].data, src.cols, src.rows, 1, 2);
        wimg_r.data = pWaveformImage_r; 
        flip(wimg_r, img_filp_r, 0);
        drawBackgroundWaveform(img_filp_r);
        imshow("Red", img_filp_r);

		if (ch_bgr[1].data == NULL) continue;
        BYTE *pWaveformImage_g = vec.GetWaveform(ch_bgr[1].data, src.cols, src.rows, 1, 1);
        wimg_g.data = pWaveformImage_g;    
        flip(wimg_g, img_filp_g, 0);
        drawBackgroundWaveform(img_filp_g);
        imshow("Green", img_filp_g);

		if (ch_bgr[0].data == NULL) continue;
        BYTE *pWaveformImage_b = vec.GetWaveform(ch_bgr[0].data, src.cols, src.rows, 1, 0);
        wimg_b.data = pWaveformImage_b;   
        flip(wimg_b, img_filp_b, 0);
        drawBackgroundWaveform(img_filp_b);
        imshow("Blue", img_filp_b);

  
#endif

//#define FINDCHESSBOARD
#ifdef FINDCHESSBOARD
        Mat viewGray;
        Size boardSize;
        boardSize.height = 6;
        boardSize.width = 9;
        vector<Point2f> pointbuf;
        cvtColor(src, viewGray, COLOR_BGR2GRAY);
          
        bool found;
        found = findChessboardCorners(src, boardSize, pointbuf,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

        if (found){
            cornerSubPix(viewGray, pointbuf, Size(11, 11), Size(-1, -1),
                TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            drawChessboardCorners(src, boardSize, Mat(pointbuf), found);
        }
       
       
#endif
//#define TRANSFORM
#ifdef TRANSFORM
        Point2f srcTri[3];
        Point2f dstTri[3];
        Mat rot_mat(2, 3, CV_32FC1);
        Mat warp_mat(2, 3, CV_32FC1);
        Mat warp_dst, warp_rotate_dst;
      
        /// Set the dst image the same type and size as src
        warp_dst = Mat::zeros(src.rows, src.cols, src.type());

        /// Set your 3 points to calculate the  Affine Transform
        srcTri[0] = Point2f(0, 0);
        srcTri[1] = Point2f(src.cols - 1.f, 0);
        srcTri[2] = Point2f(0, src.rows - 1.f);

        dstTri[0] = Point2f(src.cols*0.0f, src.rows*0.33f);
        dstTri[1] = Point2f(src.cols*0.85f, src.rows*0.25f);
        dstTri[2] = Point2f(src.cols*0.15f, src.rows*0.7f);
        /// Get the Affine Transform
        warp_mat = getAffineTransform(srcTri, dstTri);

        /// Apply the Affine Transform just found to the src image
        warpAffine(src, warp_dst, warp_mat, warp_dst.size());

        /** Rotating the image after Warp */

        /// Compute a rotation matrix with respect to the center of the image
        Point center = Point(warp_dst.cols / 2, warp_dst.rows / 2);
        static double angle;
        angle -= 1;
        double scale = 0.6;

        /// Get the rotation matrix with the specifications above
        rot_mat = getRotationMatrix2D(center, angle, scale);

        /// Rotate the warped image
        warpAffine(src, warp_rotate_dst, rot_mat, warp_dst.size());

        namedWindow(warp_window, WINDOW_AUTOSIZE);
        imshow(warp_window, warp_dst);

        namedWindow(warp_rotate_window, WINDOW_AUTOSIZE);
        imshow(warp_rotate_window, warp_rotate_dst);
#endif
////////////////////////////
		if (ch_yuv[2].data == NULL) continue;
		namedWindow(source_window, WINDOW_AUTOSIZE);
        imshow(source_window, src);
        waitKey(1);
		if (memblock) free(memblock); memblock = NULL;

#ifdef MEASURE_TIME
        diff_time = myclock() - measure_time;
        if (diff_time <= 0)diff_time = 1;
       jprintf("\n       Time : %10ld ms \n", (ULONG64)diff_time);
//        jprintf("       Size : %10ld bytes\n", (ULONG64)size);
        jprintf("       Speed: %10ld byte/s\n", ((ULONG64)size * 1000) / (ULONG64)diff_time);
#endif

    }

    temp = FLAG_OUTRO;
    err |= jtag_write_block32(addrflag, &temp, 1, JTAG_COMMON_MODULE_IDX); // SET Flag to 0 for NORMAL MODE
	
    destroyAllWindows();
    return 0;
}

//#define JTAG_PRINT_BASE 0x0a000000
//#define JTAG_PRINT_SIZE 1024*10
//#define JTAG_PRINT_POINT (JTAG_PRINT_BASE+JTAG_PRINT_SIZE)
UINT8 jtagPrintOn = 0;

UINT32 ReadJTAGPrintData(UINT32 last_pos, UINT32 size, UINT8* pbuf, UINT8 on)
{
	UINT32 err = 0;
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
    size += (offset+4);
    memblock = (UINT8*)malloc(size + 8);
    memblock_t = (UINT8*)malloc(size + 8);
    retry = (size) / (MR_READBUFFER);
    left = (size) % (MR_READBUFFER);

    for (i = 0; i < retry; i++){
		err |= jtag_read_block32(last_pos + (MR_READBUFFER * i), (UINT32*)(memblock + (MR_READBUFFER * i)), (MR_READBUFFER / 4), 3);
    }
    if (left)
		err |= jtag_read_block32(last_pos + (MR_READBUFFER * i), (UINT32*)(memblock + (MR_READBUFFER * i)), (left / 4) + 1, 3);

    memblock32 = (UINT32*)memblock;
    memblock32_t = (UINT32*)memblock_t;
    for (i = 0; i < (size / 4) + 1; i++){
        *(memblock32_t + i) = jtag_htonl((UINT32)*(memblock32 + i));
    }
    memblock_t = (UINT8*)memblock32_t;
    memblock_new_pos = memblock_t + offset;
    memblock_new_pos[origsize] = 0;
 //   hexDump("DUMP", memblock_t, origsize);
    if(on==0) jprintf("%s", memblock_new_pos);
    if (on) memcpy(pbuf, memblock_new_pos, origsize + 1);

    if (memblock) free(memblock);
    if (memblock_t) free(memblock_t);

    return err;
}


extern UINT32 now_pos;
extern UINT32 last_pos;
extern UINT32 jtag_print_size;
extern UINT32 jtag_print_base;

#define JTAG_PRINT_SIZE_ADDR	0xF0010014
#define JTAG_PRINT0_START_ADDR	0xF0010018
#define JTAG_PRINT0_POINT_ADDR	0xF001001C
#define JTAG_PRINT1_START_ADDR	0xF0010020
#define JTAG_PRINT1_POINT_ADDR	0xF0010024

UINT32 set_jtag_print(UINT32 module)
 {
	 UINT32 err = 0;
	 err |= jtag_read_block32(JTAG_PRINT_SIZE_ADDR, &jtag_print_size, 1, JTAG_COMMON_MODULE_IDX);
	 if (module == 4)
		 err |= jtag_read_block32(JTAG_PRINT0_START_ADDR, &jtag_print_base, 1, JTAG_COMMON_MODULE_IDX);
	 if (module == 5)
		 err |= jtag_read_block32(JTAG_PRINT1_START_ADDR, &jtag_print_base, 1, JTAG_COMMON_MODULE_IDX);
	 return err;
 }

void resetpos(void){
     now_pos = jtag_print_base;
     last_pos = jtag_print_base;
 }

UINT32 JTAGPrint(UINT8* pbuf, UINT8 on, UINT32 module){
	UINT32 err = 0;
    UINT fir_size = 0;
    UINT sec_size = 0;
    UINT32 size = 0;
    INT32 check = 0;
   
    // get size of jtag_print
	if (module == 4)
		err |= jtag_read_block32(JTAG_PRINT0_POINT_ADDR, &now_pos, 1, JTAG_COMMON_MODULE_IDX);
	if (module == 5)
		err |= jtag_read_block32(JTAG_PRINT1_POINT_ADDR, &now_pos, 1, JTAG_COMMON_MODULE_IDX);

	if ((jtag_print_base <= now_pos) & (now_pos < (jtag_print_base + jtag_print_size))){

        if (now_pos == last_pos){
            return 0;
        }
        check = (now_pos - last_pos);
        if (check > 0)
        {
            size = now_pos - last_pos;
			if (size > MAX_PRINT_BUF) size = MAX_PRINT_BUF;
			err |= ReadJTAGPrintData(last_pos, size, pbuf, on);
            last_pos += size;
        }
        else if (check < 0){
			fir_size = (jtag_print_base + jtag_print_size) - last_pos;
			err |= ReadJTAGPrintData(last_pos, fir_size, pbuf, on);

            sec_size = now_pos - (jtag_print_base);
			err |= ReadJTAGPrintData(jtag_print_base, sec_size, pbuf + fir_size, on);
            last_pos = now_pos;
            size = fir_size + sec_size;
        }
        else{
            return 0;
        }
    }
    return size;   
}

UINT32 JTAGPrintCmd(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - jp\r\n\
 - jp (cpu0,cpu1) FILENAME\r\n\
 - jp 0xADDR(hex) SIZE(dec)\r\n";

	UINT32 err = 0;
	UINT32 module = gcpu;
    FILE *stream = NULL;
    UINT32 size=0;
    UINT32 option = 0;

    if (argc == 1){
        option = 0;
    }
    else if (argc == 2){
		option = 0;
		if (strcmp("cpu0", argv[1]) == 0)		module = 4;
		else if (strcmp("cpu1", argv[1]) == 0)	module = 5;
		else {
			jprintf("Bad or not command!\r\n%s", chCmd);
			return 0;
		}
    }
    else if (argc == 3){
		option = 1;

		if (strcmp("cpu0", argv[1]) == 0)		module = 4;
		else if (strcmp("cpu1", argv[1]) == 0)	module = 5;
		else {
			jprintf("Bad or not command!\r\n%s", chCmd);
			return 0;
		}

		if (fopen_s(&stream, argv[2], "w+t") != 0){
			jprintf("file open fail\n");
			return 0;
		}
    }
	else{
		jprintf("Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	err |= set_jtag_print(module);
	resetpos();

	UINT8* memblock = (UINT8*)malloc(jtag_print_size);

#if 0
	jprintf("Please ESC key if you want to stop.\r\n");
	
    while (1){
        if (EN673_JTAG_ESC_key_checker()) break;
		size = JTAGPrint(memblock, option, module);
        if (option){
            fwrite(memblock, sizeof(char), size, stream);
        }
//        jprintf("size of string (%d)\n", size);
    }
#else
	size = JTAGPrint(memblock, option, module);
	if (option){
		fwrite(memblock, sizeof(char), size, stream);
	}
#endif

    if (option){
        fclose(stream);
	}

	free(memblock);

    return err;

}

UINT32 JTAGPrintOn(int argc, char** argv){
    jtagPrintOn = atoi(argv[1]);
    return 0;
}

#define JTAG_SHELL_ADDR         0xf94000cc
#define JTAG_SHELL_NOTYFY       0x00001000
#define JTAG_SHELL_NOTYFY_ADDR  0xf94000c4
#define JTAG_CPU_IRQ1           0x00000002
#define JTAG_CPU_IRQ0           0x00000001    
#define JTAG_CPU_IRQ_ADDR       0xf9400000
UINT32 JTAGShellCmd(int argc, char** argv){

	UINT32 addr_jtag_shell = JTAG_SHELL_ADDR;
	UINT32 temp[64];
	UINT32 n_temp[64];
	UINT32 shell_noty;
	UINT32 irq;
	UINT32 addr;
	UINT32 err;
	jtagPrintOn = atoi(argv[1]);
	UINT32 i = 0;

	memset(temp, 0, sizeof(temp));
	for (i = 1; i < argc; i++){
		strcat((char*)temp, argv[i]);
		strcat((char*)temp, " ");
	}
	jprintf((char*)temp);

	for (i = 0; i < (64 / 4); i++){
		*(n_temp + i) = jtag_htonl((UINT32)*(temp + i));
	}

	err |= jtag_read_block32(addr_jtag_shell, &addr, 1, JTAG_COMMON_MODULE_IDX);

	err |= jtag_write_block32(addr, (UINT32*)n_temp, 64, JTAG_COMMON_MODULE_IDX);
	shell_noty = JTAG_SHELL_NOTYFY;
	err |= jtag_write_block32(JTAG_SHELL_NOTYFY_ADDR, &shell_noty, 1, JTAG_COMMON_MODULE_IDX);
	irq = JTAG_CPU_IRQ1;
	err |= jtag_write_block32(JTAG_CPU_IRQ_ADDR, &irq, 1, JTAG_COMMON_MODULE_IDX);

	return err;
}

//#define MULTI_ACCESS
//#define SINGLE_ACCESS

UINT32 ReadID(int argc, char** argv){
	char* chCmd = "(ex)\r\n\
 - rid\r\n\
 - rid m\r\n\
 - rid s TARGETNUMBER(dec)\r\n";

	UINT32 err = 0;
	UINT32 idcodes[MAX_DEVICES];
	int num_devices = 0;
	UINT32 target=0;
	UINT32 mode = 0;

    resetpos(); // for reset 

#if 1
	if (argc == 1){
		jprintf("Check Read JTAG ID only\n");
	}
	else if (argc == 2){
		if (strcmp("m", argv[1]) == 0){
			mode = 0;
		}
		else{
            jprintf("Error or not command!\r\n%s", chCmd);
			return 0;
		}
	}else if (argc == 3){
		if (strcmp("s", argv[1]) != 0){
			jprintf("Error or not command!\r\n%s", chCmd);
			return 0;
		}
		mode = 1;
        target = atoi(argv[2]);//strtoul(argv[2], NULL, 16);
		jprintf("Targetnumber : %d\n", target);
	}
	
#endif // 0
	do{
		err |= tap_reset();
//	tap_set_ir_idcode();
		err |= tap_get_ir_ID(idcodes, &num_devices);
		Sleep(1000);
	} while (*idcodes  == 0xffffffff);
	jprintf("IDCODE : %x, DEVICE NUM : %d ", *idcodes, num_devices);
	err |= tap_reset();
	
	if(num_devices == 1){
		jtag_set_chain_target(0, num_devices);
		err |= jtag_set_tap_ir(JTAP_IR_DEBUG, JTAP_IR_BYPASS);
	}else{
		if (mode == 1){
			jtag_set_chain_target(target, num_devices);
			err |= jtag_set_tap_ir(JTAP_IR_DEBUG, JTAP_IR_BYPASS);
		}
		else{
			jtag_set_chain_target(0, num_devices);
			err |= jtag_config_chain(); // For multi access
			err |= jtag_set_tap_ir(JTAP_IR_DEBUG, JTAP_IR_BPDEBUG);
		}
	}
 
	return err;
}
#define MAX_ERR_CNT 2
UINT32 SpiDownload(int argc, char** argv){
    UINT32 err = 0;

    jprintf("USB Serial Flash Downloader for Linux Ver1.0\n");
    jprintf("Copyright (c) Eyenix Co., Ltd. All Rights Reserved.\n\n\n");
    if (argc<3) {
        ERROR_("Input file name is not correct!\n");
        jprintf("Usage: UsbWrite [target.bin] [Start Address] [debug] [bench]\n");
        return FALSE;
    }

    uint adr = atoi(argv[2]);
    if (argc >= 4) debug_spi = atoi(argv[3]);
    if (argc >= 5) debug_bench = atoi(argv[4]);

    uint clkdiv = 0;
    uint rd_timeout = 0;
    uint wr_timeout = 0;
    uint latency = 0;

    UINT32 errCnt = 0;
    UINT32 error = 0;
    while (error = spi_ftdx_init(clkdiv, rd_timeout, wr_timeout, latency) | spi_flash_download(argv[1], adr)){
        spi_ftdx_close();
        Sleep(10);
        jprintf("$");
        errCnt++;
        if (errCnt > MAX_ERR_CNT) return -1;
    }
   
    err |= spi_ftdx_close();
    if (err) {
        ERROR_("Close FTD\n");
        return -1;
    }

    return err;
}

UINT32 Rsp_Server(int argc, char** argv){

//	handle_rsp();
	return 0;
}

UINT32 Board_Checker(void){
	UINT32 err = 0;
	UINT32 reg=0;
	err = example_read_register(WBCOM, 0xf950002c, &reg);
    if (reg != 0x672a0000){
        jprintf("Please check JTAG cable!\n");
		return err;
		//return 0xf0f00000;
    }
    else{
		return 0;
    }
}

//-------------------------------------------------------------------------------------------------
// Take the incoming string and create an argv[] array from that.
// The incoming string is assumed to be writeable.
// The argv[] array is simple a set of pointers into that string, where the whitespace delimited character sets are each NULL terminated.

UINT32 _tokenize(char *string, char *argv[])
{
	UINT32	argc, done;

	for(argc=0; argc<ARGCNT; argc++)	argv[argc] = (char *)0;

	argc = 0;

	while(1){
		while( (*string==' ') || (*string=='\t') )		string++;

		if (*string==0)			break;

		argv[argc] = string;

		while( (*string!=' ') && (*string!='\t') ){
			if( (*string=='\\') && (*(string+1)=='"') ){
				strcpy(string,string+1);
			}
			else if(*string == '"'){
				strcpy(string,string+1);
				while(*string != '"'){
					if( (*string=='\\') && (*(string+1)=='"') )		strcpy(string,string+1);
					if(*string==0)		return(-1);
					string++;
				}

				strcpy(string,string+1);
				continue;
			}

			if(*string==0)		break;
			string++;
		}

		if(*string == 0){	done = 1;					}
		else			{	done = 0;	*string++ = 0;	}

		argc++;
		if(done)			break;

		if(argc >= ARGCNT){
			argc = -1;
			break;
		}
	}
	return(argc);
}


//-------------------------------------------------------------------------------------------------
//
UINT32 _DoCommand(char *cmdline)
{
	UINT32		ret, argc;
	tMonCmd	*cmdptr;
	char		*argv[ARGCNT], cmdcpy[CMDLINESIZE];


	if(cmdline[0]=='_')		cmdline++;
	if(cmdline[0]=='!')		strcpy(cmdline, gcHistoryBuf);

	strcpy(cmdcpy, cmdline);

//	if(cmdline[0]!='!')		strcpy(gcHistoryBuf, cmdline);

	
	argc = _tokenize(cmdcpy, argv);

	if( (argc==0) && (argc<0) )		return(CMD_FAILURE);

	
	cmdptr = gCmdList;

	if(argv[0]==NULL)				return(CMD_SUCCESS);

	while(cmdptr->name){
		if(strcmp(argv[0], cmdptr->name) == 0)		break;
		cmdptr++;

		if(cmdptr->name == 0)		return (CMD_LINE_ERROR);
	}

//	if(strcmp("md", cmdptr->name) != 0)		jprintf("%s\n", *cmdptr->helphelp);		// for fast memory display

	ret = cmdptr->func(argc,argv);

	jprintf("\n");


	return(ret);
}

HANDLE ghMutex;

DWORD WINAPI ThreadCliFunc(void *arg)
{
    UINT32 ret = 0;
    UINT32 err = 0;
 
    char instr[MAX_IN_STRING] = { 0, 0 };

    gcHistoryBuf = (char*)malloc(MAX_IN_STRING);
    gcPrompt = (char*)malloc(MAX_IN_STRING);
    gcCmdBuf = (char*)malloc(MAX_IN_STRING);

    HANDLE stdinHandle;
    stdinHandle = GetStdHandle(STD_INPUT_HANDLE);
  
    while (1){
        if (WaitForSingleObject(ghMutex, INFINITE) == WAIT_OBJECT_0)
        {
            instr[0] = 0;
            //if (WaitForSingleObject(stdinHandle, 100) == WAIT_OBJECT_0){
                if (gcpu == 4){
                    cout << "cpu0>";
                }
                else if (gcpu == 5){
                    cout << "cpu1>";
                }
                cin.getline(instr, MAX_IN_STRING);
            //}

            //  if (!Board_Checker()){
            if (1){
                ret = _DoCommand((char*)instr);
                if (ret == CMD_LINE_ERROR)
				{
					jprintf("Bad or not command!\n");
					ERR_State = 0;
				}
                if (ret != 0){
					jprintf("Err :%08x\n", ret);
					ERR_State = ret;
                }
            }
            else{
                _DoCommand("init all");
                if (strcmp(instr, "rst") == 0){
                    _DoCommand("rst");
                    _DoCommand("rc");               // For stop cpu
                }
            }
            ReleaseMutex(ghMutex);
        }
        Sleep(1);
    }
    EN673_JTAG_CLOSE();

    free(gcHistoryBuf);
    free(gcPrompt);
    free(gcCmdBuf);
	cout << "return";
    return 0;
}

UINT32 cli_stdin_request_(char* cmdline);

#define CLI_CMD_LEN 		256
DWORD WINAPI Thread_Read(void *arg)
{

	SOCKET* pcli_fd;
	SOCKET cli_fd;
	pcli_fd = (SOCKET*)arg;
	cli_fd = *pcli_fd;
	int ret;
	char cmd[CLI_CMD_LEN + 10];
	char testbuf[100] = { 0, };
	while (1)
	{
		Sleep(10);
		ret = recv(cli_fd, cmd, CLI_CMD_LEN, 0);
		
		if ((cmd[0] == 'E' && cmd[1] == 'X' && cmd[2] == 'I' && cmd[3] == 'T') || \
			(cmd[0] == 'e' && cmd[1] == 'x' && cmd[2] == 'i' && cmd[3] == 't')){		// For escape terminal	EX) exit EXIT				
			closesocket(cli_fd);
			printf("Socket closed(%d)!\n", cli_fd);
			break;
		}

		if (ret > 2){
			cmd[ret] = '\0';
			sprintf(cmd, "%s%d ", cmd, cli_fd);
			if (RemoteLog_flag)
			{
				if (strstr(cmd, "rem_check") == NULL)
					printf("%s", cmd);

			}
			ret = cli_stdin_request_(cmd);
			if (ret) WARNING("Bad or not command!\n");
		}
		
	}

	return 0;

}


DWORD WINAPI ThreadAFunc(void *arg)
{
	WSADATA wsa;
	SOCKET master, new_socket;
	struct sockaddr_in server, address;
	int addrlen;
	

	jprintf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		jprintf("Failed. Error Code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	jprintf("Initialised.\n");

	//Create a socket
	if ((master = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
	{
		jprintf("Could not create socket : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	jprintf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(server_port);

	//Bind
	if (bind(master, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
	{
		jprintf("Bind failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	puts("Bind done");

	//Listen to incoming connections
	listen(master, 3);

	//Accept and incoming connection
	puts("Waiting for incoming connections...");

	addrlen = sizeof(struct sockaddr_in);

	while (TRUE)
	{
	
			if ((new_socket = accept(master, (struct sockaddr *)&address, (int *)&addrlen))<0)
			{
				perror("accept");
				exit(EXIT_FAILURE);
			}

			//inform user of socket number - used in send and receive commands
			jprintf("New connection , socket fd is %d , ip is : %s , port : %d \n", new_socket, inet_ntoa(address.sin_addr), ntohs(address.sin_port));

		
			DWORD dwThreadID0;
			HANDLE CliThread = CreateThread(NULL, 0, Thread_Read, (LPVOID)&new_socket, 0, &dwThreadID0);
			
	}
	
	closesocket(master);
	WSACleanup();

    return TRUE;
}

DWORD WINAPI ThreadBFunc(void *arg)
{
    while (1){
        if (WaitForSingleObject(ghMutex, INFINITE) == WAIT_OBJECT_0)
        {
            jprintf("B");
            _DoCommand("si");
            ReleaseMutex(ghMutex);
        }
        Sleep(100);
    }

    return TRUE;
}

int _tmain(int argc, wchar_t* argv[])
{
	UINT32 err = 0;

	for (int i = 0; i < argc; i++){
		printf("argv %S \n", argv[i]);
	}

    DWORD dwThreadID0;
	DWORD dwThreadID1;
	
#if 0
	// Command 실행
	if (argc >= 2) {
		int i = 0;
	//	for (i = 0; i < argc;i++)
	//	jprintf("argc %d argv[%s]\n",i, argv[i]);


		i = 0;
		
		UINT32 ret = 0;
		tMonCmd	*cmdptr = gCmdList;
		UINT8 devName[512] = { 0, 0 };
		WideCharToMultiByte(CP_ACP, 0, argv[1], -1,(char*) devName, 512, 0, 0);
		while (MPSSE_init_withDescirption(clkdiv, devName));
		tap_reset();
		tap_set_ir_debug();

		char** chArgv = NULL;
		chArgv = new char*[argc-2];
		for(i = 0; i < argc-2; i++) {
			chArgv[i] = new char[512];
			WideCharToMultiByte(CP_ACP, 0, argv[i+2], -1, chArgv[i], 512, 0, 0);
		}

		while (cmdptr->name){
			if (strcmp(chArgv[0], cmdptr->name) == 0)	break;
			cmdptr++;

			if (cmdptr->name == 0)	{
				jprintf("\r\nCommand error!\r\n");
				return FALSE;
			}
		}

		ret = cmdptr->func(argc-2, chArgv);

		for (i = 0; i < argc-2; i++)
			delete chArgv[i];
		delete chArgv;
		MPSSE_close();

		return ret;
	}
#else
	if (argc >= 3) {
		server_port = _wtoi(argv[1]);
		clkdiv = _wtoi(argv[2]);
		channel = (char*)argv[3];
	}
	else
	{
		server_port = 5556;
		clkdiv = 0x5;
		channel = "B";
	}
#endif

//	while (spi_ftdx_init(0, 0, 0, 0));
//	spi_ftdx_close();
	while (err |= EN673_JTAG_INIT(clkdiv, *channel));

	// LUA init
	exr_lua_init();

    ghMutex = CreateMutex(NULL, FALSE, NULL);

    if (ghMutex == NULL)
    {
        jprintf("CreateMutex error:%d\n", GetLastError());
        return 1;
    }

    Sleep(1000);

    HANDLE CliThread = CreateThread(NULL, 0, ThreadCliFunc, NULL, 0, &dwThreadID0);
    HANDLE NetThread = CreateThread(NULL, 0, ThreadAFunc, NULL, 0, &dwThreadID1);
 //   HANDLE BThread = CreateThread(NULL, 0, ThreadBFunc, NULL, 0, &dwThreadID2);

    while (1){
        Sleep(1);
    }
    return 0;
}


UINT32 Waitms(int argc, char** argv)
{
	char* chCmd = "(ex)\r\n\
				   - wms 100 : 100ms wait\r\n";

	if (argc != 2){
		jprintf("Error : Bad or not command!\r\n%s", chCmd);
		return 0;
	}

	int ms = atoi(argv[1]);

	Sleep(ms);

	return 0;
}

UINT32 GetDevNum(int argc, char** argv)
{
	return MPSSE_GETListNum();
}

UINT32 GetReg(int argc, char** argv){
	UINT32 err = 0;
	UINT32 module = 0;
	UINT32 addr = 0;
	UINT32 num, i = 0;
	UINT32 reg = 0;
	FILE*	pFile = NULL;

	UINT32 k = 0;
	char  fileName[1024];

	if (argc == 4){
		jprintf("Use (default:rr.txt) file : \n");
		strcpy(fileName, "rr.txt");
	}
	else if (argc == 5){
		jprintf("Use (%s) file \n", argv[4]);
		strcpy(fileName, argv[4]);
	}
	else{
		jprintf("error : ex) rr (wbcpu0,wbcpu1,wbcom,cpu0,cpu1) 0x00000000 num (rr.txt)\n");
		return 0;
	}

	if (strcmp("wbcpu0", argv[1]) == 0)			module = 0;
	else if (strcmp("wbcpu1", argv[1]) == 0)	module = 1;
	else if (strcmp("wbcom", argv[1]) == 0)		module = 3;
	else if (strcmp("cpu0", argv[1]) == 0)		module = 4;
	else if (strcmp("cpu1", argv[1]) == 0)		module = 5;
	else {
		jprintf("Error : option choose (wbcpu0,wbcpu1,wbcom,cpu0,cpu1)'\n");
		jprintf("Error : ex) rr (wbcpu0,wbcpu1,wbcom,cpu0,cpu1) 0x00000000 num (rr.txt)\n");
	}

	addr = strtoul(argv[2], NULL, 16);

	char* memblock;
	memblock = (char*)malloc(MAX_TXT_BUFFER);
	if (memblock == NULL){
		jprintf("Malloc fail !\n");
		return 0;
	}
	num = 1;
	addr = ((addr >> 2) << 2);			// 4의 배수 단위로 Addr 맞춤 
	//	jprintf("addr:0x%x\n", addr);
	for (i = 0; i < num; i++){
		//reg = example_read_register(module, addr + (i * 4));
		err |= example_read_register(module, addr + (i * 4), &reg);
		jprintf("0x%.8x :   reg = %.8x \n", addr + (i * 4), reg);
		k += sprintf_s((char*)memblock + k, MAX_TXT_BUFFER, "0x%.8x :   reg = %.8x \n", addr + (i * 4), reg);
	}

//	pFile = fopen(fileName, "w+b");
//	fwrite(memblock, sizeof(char), strlen(memblock), pFile);
//	fclose(pFile);

	if (memblock) free(memblock);

	return err;
}

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"


void TermTcpSend(void* arg)
{
	term_st* term = (term_st*)arg;
	UINT size = 0;
	INT  check = 0;
	UINT* pos = (UINT*)term->point;
	term->now_pos = *pos;
	if ((term->base <= term->now_pos) )
	{

		if (term->now_pos == term->last_pos){
			return;
		}
		check = (term->now_pos - term->last_pos);
		if (check > 0){
			size = term->now_pos - term->last_pos;
			send(term->sock_fd, (char*)term->last_pos, size, 0);
			term->last_pos += size;
		}
		else if (check < 0){
			size = term->point - term->last_pos;
			send(term->sock_fd, (char*)term->last_pos, size, 0);
			term->last_pos = term->base;
		}
		else{
			return;
		}
	}
	return;
}

HANDLE Thread_SendImage = NULL;
HANDLE Thread_SendText = NULL;
UINT sock_fd = 0;
term_st term;


DWORD WINAPI ThreadSendFunc(void *arg)
{
	EscapeLoopSet(0);
	while (1){
		if (EscapeLoop()) break;
		TermTcpSend(arg);
		Sleep(1);
	}
	return 0;
}


UINT32 SendText(int argc, char** argv)
{
	DWORD dwThreadID0;
	TermInitValue((void*)&term);
	if (argc == 2){
		jprintf("fd - %d \n", argv[1]);
		term.sock_fd = atoi(argv[1]);
	}
	else{
		term.sock_fd = sock_fd;
	}
	Thread_SendText = CreateThread(NULL, 0, ThreadSendFunc, (LPVOID)&term, 0, &dwThreadID0);
	return 0;
}
UINT32 KillSendText(int argc, char** argv)
{
	EscapeLoopSet(1);

	return 0;
}
UINT32 NetConSever(int argc, char** argv){

//	term_st term;
//	TermInitValue((void*)&term);
//	HANDLE CliThread=NULL;

	WSADATA wsa;
	SOCKET master, new_socket, client_socket[30], s;
	struct sockaddr_in server, address;
	int max_clients = 30, activity, addrlen, i, valread;
	char *message = "ECHO Daemon v1.0 \r\n";

	//size of our receive buffer, this is string length.
	int MAXRECV = 1024;
	//set of socket descriptors
	fd_set readfds;
	//1 extra for null character, string termination
	char *buffer;
	buffer = (char*)malloc((MAXRECV + 1) * sizeof(char));

	for (i = 0; i < 30; i++)
	{
		client_socket[i] = 0;
	}

	jprintf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		jprintf("Failed. Error Code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	jprintf("Initialised.\n");

	//Create a socket
	if ((master = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
	{
		jprintf("Could not create socket : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	jprintf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(server_port);

	//Bind
	if (bind(master, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
	{
		jprintf("Bind failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	puts("Bind done");

	//Listen to incoming connections
	listen(master, 3);

	//Accept and incoming connection
	puts("Waiting for incoming connections...");

	addrlen = sizeof(struct sockaddr_in);

	while (TRUE)
	{
		Sleep(1);
		//clear the socket fd set
		FD_ZERO(&readfds);

		//add master socket to fd set
		FD_SET(master, &readfds);

		//add child sockets to fd set
		for (i = 0; i < max_clients; i++)
		{
			s = client_socket[i];
			if (s > 0)
			{
				FD_SET(s, &readfds);
			}
		}

		//wait for an activity on any of the sockets, timeout is NULL , so wait indefinitely
		activity = select(0, &readfds, NULL, NULL, NULL);

		if (activity == SOCKET_ERROR)
		{
			jprintf("select call failed with error code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}

		//If something happened on the master socket , then its an incoming connection
		if (FD_ISSET(master, &readfds))
		{
			if ((new_socket = accept(master, (struct sockaddr *)&address, (int *)&addrlen))<0)
			{
				perror("accept");
				exit(EXIT_FAILURE);
			}

			//inform user of socket number - used in send and receive commands
			jprintf("New connection , socket fd is %d , ip is : %s , port : %d \n", new_socket, inet_ntoa(address.sin_addr), ntohs(address.sin_port));

			//send new connection greeting message
			if (send(new_socket, message, strlen(message), 0) != strlen(message))
			{
				perror("send failed");
			}
//			DWORD dwThreadID0;
			sock_fd = new_socket;
//			CliThread = CreateThread(NULL, 0, ThreadSendFunc, (LPVOID)&term, 0, &dwThreadID0);
//			CliThread = CreateThread(NULL, 0, ThreadImageSendFunc, (LPVOID)&term, 0, &dwThreadID0);
			puts("Welcome message sent successfully");
			//add new socket to array of sockets
			for (i = 0; i < max_clients; i++)
			{
				if (client_socket[i] == 0)
				{
					client_socket[i] = new_socket;
					jprintf("Adding to list of sockets at index %d \n", i);
					break;
				}
			}
		}

		//else its some IO operation on some other socket :)
		for (i = 0; i < max_clients; i++)
		{
			s = client_socket[i];
			//if client presend in read sockets             
			if (FD_ISSET(s, &readfds))
			{
				//get details of the client
				getpeername(s, (struct sockaddr*)&address, (int*)&addrlen);

				//Check if it was for closing , and also read the incoming message
				//recv does not place a null terminator at the end of the string (whilst printf %s assumes there is one).
				valread = recv(s, buffer, MAXRECV, 0);

				if (valread == SOCKET_ERROR)
				{
					int error_code = WSAGetLastError();
					if (error_code == WSAECONNRESET)
					{
						//Somebody disconnected , get his details and print
						jprintf("Host disconnected unexpectedly , ip %s , port %d \n", inet_ntoa(address.sin_addr), ntohs(address.sin_port));

						//Close the socket and mark as 0 in list for reuse
						closesocket(s);
						client_socket[i] = 0;
					}
					else
					{
						jprintf("recv failed with error code : %d", error_code);
					}
				}
				if (valread == 0)
				{
					//Somebody disconnected , get his details and print
					jprintf("Host disconnected , ip %s , port %d \n", inet_ntoa(address.sin_addr), ntohs(address.sin_port));

					//Close the socket and mark as 0 in list for reuse
					closesocket(s);
					client_socket[i] = 0;
				}

				//Echo back the message that came in
				else
				{
					//add null character, if you want to use with printf/puts or other string handling functions
					//buffer[valread] = '\0';
					jprintf(" %s:%d>%s \n", inet_ntoa(address.sin_addr), ntohs(address.sin_port), buffer);
					if ((buffer[0] == 'E' && buffer[1] == 'X' && buffer[2] == 'I' && buffer[3] == 'T') || \
						(buffer[0] == 'e' && buffer[1] == 'x' && buffer[2] == 'i' && buffer[3] == 't')){		// For escape terminal  EX) exit EXIT
//						TerminateThread(CliThread, NULL);
						closesocket(s);
						WSACleanup();

						return 0;
					}
					if (buffer[0] == '\r' && buffer[1] == '\0'){
						buffer[0] = ' ';
						buffer[1] = '\0';
					}
					buffer[valread - 2] = ' '; // for change "CR" to "SPACE"
					buffer[valread - 1] = '\0';

					UINT32 ret = _DoCommand((char*)buffer);
					if (ret == CMD_LINE_ERROR)
					{
						jprintf("Bad or not command!\n");
						ERR_State = 0;
					}
					if (ret != 0){
						jprintf("Err :%08x\n", ret);
						ERR_State = ret;
					}
					//send(s, buffer, valread, 0);
				}
			}
		}
	}
//	TerminateThread(CliThread,NULL);
	closesocket(s);
	WSACleanup();

	return 0;
}


SOCKET ConnectSocket = INVALID_SOCKET;
#define MAX_IN_STRING		256

DWORD WINAPI ThreadRecvFunc(void *arg)
{
	//	SOCKET ConnectSocket = INVALID_SOCKET;
	char recvbuf[DEFAULT_BUFLEN];
	int recvbuflen = DEFAULT_BUFLEN;
	int iResult;

	while (1){
		do {

			iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);

			if (iResult > 0){
				recvbuf[iResult + 1] = 0;
				jprintf("%s\n", recvbuf);

			}
			else if (iResult == 0){
				jprintf("Connection closed\n");

			}
			else
				jprintf("recv failed with error: %d\n", WSAGetLastError());
		} while (iResult > 0);
		Sleep(1);
	}
	return TRUE;
}


UINT32 NetConClient(int argc, char** argv)
{
	WSADATA wsaData;
	//SOCKET ConnectSocket = INVALID_SOCKET;
	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;
	char *sendbuf = "this is a test ";
	//char recvbuf[DEFAULT_BUFLEN];
	int iResult;
	int recvbuflen = DEFAULT_BUFLEN;

	// Validate the parameters
	if (argc != 2) {
		jprintf("usage: %s server-name\n", argv[0]);
		//	return 1;
	}

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		jprintf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("192.168.1.50", DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		jprintf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			jprintf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 1;
		}
		/*
		u_long iMode = 1;
		iResult = ioctlsocket(ConnectSocket, FIONBIO, &iMode);
		if (iResult != NO_ERROR)
		jprintf("ioctlsocket failed with error: %ld\n", iResult);
		*/
		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		jprintf("Unable to connect to server!\n");
		WSACleanup();
		return 1;
	}
	char instr[MAX_IN_STRING] = { 0, 0 };
	DWORD dwThreadID0;
	HANDLE CliThread = CreateThread(NULL, 0, ThreadRecvFunc, NULL, 0, &dwThreadID0);

	while (1){

		cin.getline(instr, MAX_IN_STRING);
		// Send an initial buffer
		iResult = send(ConnectSocket, instr, (int)strlen(instr) + 1, 0);
		if (iResult == SOCKET_ERROR) {
			jprintf("send failed with error: %d\n", WSAGetLastError());
			closesocket(ConnectSocket);
			WSACleanup();
			return 1;
		}
		// jprintf("Bytes Sent: %ld\n", iResult);
	}
	// cleanup
	closesocket(ConnectSocket);
	WSACleanup();

	return 0;
}



UINT32 OpenJtag(int argc, char** argv)
{
	int clkdiv = 0;
	if (argc == 2){
		clkdiv = atoi(argv[1]);
		MPSSE_init(clkdiv, *channel);
	}
	else if (argc == 3){
		clkdiv = atoi(argv[1]);
		MPSSE_init_withDescirption(clkdiv,(UINT8*) argv[2]);
	}
	return 0;
}

UINT32 CloseJtag(int argc, char** argv)
{
	MPSSE_close();
	return 0;
}

UINT32 RemoteLog(int argc, char** argv)
{
	if (argc == 2)
	{
		if (strcmp("1", argv[1]) == 0)
		{
			RemoteLog_flag = 1;
			printf("ECM Remote Log On\n");
		}
		else if (strcmp("0", argv[1]) == 0)
		{
			RemoteLog_flag = 0;
			printf("ECM Remote Log Off\n");
		}
		else
		{
			jprintf("error : ex) log (0, 1)\n");
			return 0;
		}
	}
	else
	{
		jprintf("error : ex) log (0, 1)\n");
	}
	return 0;
}


UINT32 ConsoleClear(int argc, char** argv)
{
	system("cls");

	return 0;
}


#define SRC_ADDR_REG	0x08000000
#define DST_ADDR_REG	0x08000004
#define COMPSIZE_REG	0x08000008
#define ACTIVATE_REG	0x0800000c

#define ACTIVATE		1
#define DEACTIVATE		0

#define SIZE_OF_DATA	1024*1024	
#define SRC_ADDR_START	0x08000000
#define DST_ADDR_START	0x08000000



UINT32 FlushTest(int argc, char** argv)
{
	//This is Flush Test Code for EN675 cache flush.
	UINT32 src = 0;
	UINT32 dst = 0;
	UINT32 size = 0;
	UINT32 read;
	UINT32 err;
	UINT32 offset = 0x1000;


	// loop start
	UINT32 i;

	UINT8 * memblock_src = NULL;
	UINT8 * memblock_dst = NULL;

	UINT32 mismatch = 0;


	// 초기화 
	src = SRC_ADDR_START;
	dst = DST_ADDR_START;
	size = SIZE_OF_DATA;

	for (i = 0; i < 100; i++){


		if ((memblock_src = (UINT8*)malloc(size + 4)) == NULL){
			jprintf("Malloc fail \n");
			return 0;
		}
		if ((memblock_dst = (UINT8*)malloc(size + 4)) == NULL){
			jprintf("Malloc fail \n");
			return 0;
		}


		// SRC address write
		err |= jtag_write32(SRC_ADDR_REG, src, JTAG_COMMON_MODULE_IDX);
		// dst address write
		err |= jtag_write32(DST_ADDR_REG, dst, JTAG_COMMON_MODULE_IDX);
		// size write
		err |= jtag_write32(COMPSIZE_REG, size, JTAG_COMMON_MODULE_IDX);
		// activate reg write to 1
		err |= jtag_write32(ACTIVATE_REG, ACTIVATE, JTAG_COMMON_MODULE_IDX);
		// activate check until 0 

		do{
			err |= jtag_read32(ACTIVATE_REG, &read, JTAG_COMMON_MODULE_IDX);
			if (err) return err;
			//	if (((clock() - start) / CLOCKS_PER_SEC) > FLASH_TIME_OUT_SECOND) return ERR_MPSSE_FLASH_RETRY_OVER;

		} while (read);

		// compare src and dst

		UINT32 size_adj;
		UINT32 retry;
		UINT32 left;
		UINT32 addr;
		UINT32 i = 0;

		size_adj = ((size + 4) >> 2) << 2;
		retry = (size_adj) / (MR_READBUFFER);
		left = (size_adj) % (MR_READBUFFER);

		for (i = 0; i < retry; i++){
			if (EN673_JTAG_ESC_key_checker()){
				if (memblock_src) free(memblock_src);
				if (memblock_dst) free(memblock_dst);
				return 0;
			}
			err |= jtag_read_block32(src + (MR_READBUFFER * i), (UINT32*)(memblock_src + (MR_READBUFFER * i)), (MR_READBUFFER / 4), JTAG_COMMON_MODULE_IDX);
			cout << "*";
		}
		if (left)
			err |= jtag_read_block32(src + (MR_READBUFFER * i), (UINT32*)(memblock_src + (MR_READBUFFER * i)), (left / 4) + 1, JTAG_COMMON_MODULE_IDX);
		cout << "END";

		for (i = 0; i < retry; i++){
			if (EN673_JTAG_ESC_key_checker()){
				if (memblock_src) free(memblock_src);
				if (memblock_dst) free(memblock_dst);
				return 0;
			}
			err |= jtag_read_block32(dst + (MR_READBUFFER * i), (UINT32*)(memblock_dst + (MR_READBUFFER * i)), (MR_READBUFFER / 4), JTAG_COMMON_MODULE_IDX);
			cout << "*";
		}
		if (left)
			err |= jtag_read_block32(dst + (MR_READBUFFER * i), (UINT32*)(memblock_dst + (MR_READBUFFER * i)), (left / 4) + 1, JTAG_COMMON_MODULE_IDX);
		cout << "END";

		if (memcmp(memblock_src, memblock_dst, size)){
			// Error
			mismatch++;
			printf("Error[%d] %d\n", i, mismatch);
		}
		else{
			// Pass
			printf("Pass %d\n", i);
		}
		// 주소업데이트 
		src += offset;
		dst += offset;

		free(memblock_src);
		free(memblock_dst);

	}


	


	return 0;
}