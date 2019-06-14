#ifndef JTAG_FLASH_H
#define JTAG_FLASH_H

#include <windows.h>


#define ISP_REG_BASE  0xf1100000
#if CHANGE_EN674
#define SFLS_REG_BASE 0xf0a00000 // EN674
#else
#define SFLS_REG_BASE 0xf9200000 // EN673
#endif
#define SYS_REG_BASE  0xf9500000
#define CPU_REG_BASE  0xf9400000
#define SYS_WDT_CNT    0x20
#define SYS_WDT_LMT    0x24
#define SYS_WDT_CGF    0x28

#define CPU0_WDT_CGF    0x10
#define CPU1_WDT_CGF    0x14
#define CPU0_WDT_LTM	0x28
#define CPU1_WDT_LTM	0x2c

#define _SFLS_DATIN			0x0000
#define SFLS_DATIN		24

#define _SFLS_DATOUT		0x0004
#define SFLS_DATOUT		24



#define _SFLS_WBCMD			0x0008
#define WB_RDCMD		8
#define WB_WRCMD		0

#define _SFLS_WBCONT		0x000c
#define REG_CMD			24
#define WB_ADR_EXT		23
#define WB_WR_EN		22
#define WB_WAIT_EN		21
#define WB_GAP_EN		20
#define WB_GAP_LEN		18
#define WB_RDCMD_IOM	16
#define WB_RDADR_IOM	14
#define WB_RDDAT_IOM	12
#define WB_WREN_IOM		10
#define WB_WRCMD_IOM	8
#define WB_WRADR_IOM	6
#define WB_WRDAT_IOM	4
#define WB_WAITCMD_IOM	2
#define WB_WAITDAT_IOM	0

#define _SFLS_CMDADR		0x0010
//#define REG_CMD			24
#define REG_ADR			0

#define _SFLS_CMD			0x0014
#define CLKDIV			28
#define REG_REQ			17		
#define WB_REQ			16
#define REG_ADR_EXT		15
#define REG_ADR_EN		14
#define REG_GAP_EN		13
#define REG_DAT_EN		12
#define REG_WAIT_EN		11
#define REG_DAT_RW		10
#define REG_CMD_IOM		8
#define REG_ADR_IOM		6
#define REG_DAT_IOM		4
#define REG_GAP_LEN		2
#define REG_DAT_LEN		0

#define FLASH_TIME_OUT_SECOND 3
UINT32 sfls_reset(void);
UINT32 sfls_wait_for_reg_request(void);
UINT32 sfls_write_en(void);
UINT32 Sfls_erase_sect(UINT32 Adr);
UINT32 Sfls_erase_block(UINT32 Adr);
UINT32 Sfls_erase_all(void);
UINT32 Sfls_write_reg(UCHAR dat);
UINT32 Sfls_read_reg(BYTE* dat);

UINT32 Sfls_init_quad(void);
UINT32 Sfls_erase(UINT32 addr, UINT32 len);
UINT32 Sfls_write32(UINT32* buf, UINT32 len, UINT32 offset);
UINT32 Sfls_point_write32(UINT32 data, UINT32 byteoffset);
UINT32 Sfls_point_write8(UINT8 data, UINT32 byteoffset);
UINT32 Get_device_ID(UINT32* ID);
UINT32 Get_device_SIZE(UINT32* SIZE);

UINT32 Sfls_writeProtect_Disable_all(void);
UINT32 Sfls_writeProtect_Enable_all(void);

#if! CHANGE_EN674
UINT32 reset_sys_wdt(void);
#endif
UINT32 system_reset(void);

#endif