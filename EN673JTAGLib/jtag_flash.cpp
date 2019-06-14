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
#include <windows.h>
#include <stdio.h>
#include "ftd2xx.h"
#include "spr_defs.h"
#include "EN673JTAG.h"
#include "jtag_base.h"
#include "jtag_tap.h"
#include "jtag_module.h"
#include "jtag_rw.h"
#include "jtag_example.h"
#include "jtag_debug.h"
#include "jtag_def.h"
#include "jtag_flash.h"
#include <time.h>

UINT32 sfls_wait_for_reg_request(void)
{
	UINT32 read = 0;
    UINT32 err = 0;
    clock_t start;
    start = clock();
    do{
        err |= jtag_read32(SFLS_REG_BASE + _SFLS_CMD, &read, JTAG_COMMON_MODULE_IDX);
        if (err) return err;
        if (((clock() - start) / CLOCKS_PER_SEC) > FLASH_TIME_OUT_SECOND) return ERR_MPSSE_FLASH_RETRY_OVER;
    } while (read & (1 << REG_REQ));

    return 0;
}

UINT32 sfls_reset(void)
{
    UINT32 read = 0;
    UINT32 err = 0;

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
    read = read & 0x00ffffff;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0x66 << REG_CMD), JTAG_COMMON_MODULE_IDX);
    err |= sfls_wait_for_reg_request();

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
    read = read & 0x00ffffff;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0x99 << REG_CMD), JTAG_COMMON_MODULE_IDX);
    err |= sfls_wait_for_reg_request();
    return err;
}


UINT32 sfls_write_en(void)
{
	UINT32 read = 0;
	UINT32 rd_com;
    UINT32 clk_div;
    UINT32 err = 0;

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	read = read & 0x00ffffff;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0x06 << REG_CMD), JTAG_COMMON_MODULE_IDX);

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);

	rd_com = (read & 0x00030000) >> WB_RDCMD_IOM;
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_CMD, &read, JTAG_COMMON_MODULE_IDX);
    clk_div = (read & 0xf0000000) >> CLKDIV;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x00020000 | (rd_com << REG_CMD_IOM) | (clk_div << CLKDIV), JTAG_COMMON_MODULE_IDX);

    err |= sfls_wait_for_reg_request();
    return err;
}

UINT32 Sfls_erase_sect(UINT32 Adr)
{
	UINT32 read = 0;
	UINT32 rd_com;
    UINT32 clk_div;
    UINT32 err = 0;
    err |= sfls_write_en();

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	read = read & 0x00ffffff;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0x20 << REG_CMD), JTAG_COMMON_MODULE_IDX);

    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMDADR, ((Adr & 0x00ffffff) << REG_ADR), JTAG_COMMON_MODULE_IDX);
	
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	rd_com = (read & 0x00030000) >> WB_RDCMD_IOM;
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_CMD, &read, JTAG_COMMON_MODULE_IDX);
    clk_div = (read & 0xf0000000) >> CLKDIV;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x00024800 | (rd_com << REG_CMD_IOM) | (rd_com << REG_ADR_IOM) | (clk_div << CLKDIV), JTAG_COMMON_MODULE_IDX);
	
    err |= sfls_wait_for_reg_request();
    return err;
}

UINT32 Sfls_erase_block(UINT32 Adr)
{
	UINT32 read = 0;
	UINT32 rd_com;
    UINT32 clk_div;
    UINT32 err = 0;
	err |= sfls_write_en();

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	read = read & 0x00ffffff;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0xD8 << REG_CMD), JTAG_COMMON_MODULE_IDX);
	
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMDADR, ((Adr & 0x00ffffff) << REG_ADR), JTAG_COMMON_MODULE_IDX);
	
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	rd_com = (read & 0x00030000) >> WB_RDCMD_IOM;
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_CMD, &read, JTAG_COMMON_MODULE_IDX);
    clk_div = (read & 0xf0000000) >> CLKDIV;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x00024800 | (rd_com << REG_CMD_IOM) | (rd_com << REG_ADR_IOM) | (clk_div << CLKDIV), JTAG_COMMON_MODULE_IDX);
	
    err |= sfls_wait_for_reg_request();
    return err;
}

UINT32 Sfls_writeProtect_Disable_all(void)
{
	printf("func : %s \t line : %d\n", __FUNCTION__, __LINE__);
	UINT32 read = 0;
	UINT32 rd_com;
	UINT32 clk_div;
	UINT32 err = 0;

	err |= sfls_write_en();

	err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	read = read & 0x00ffffff;
	err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0x01 << REG_CMD), JTAG_COMMON_MODULE_IDX);

	err |= jtag_write32(SFLS_REG_BASE + _SFLS_DATIN, (0x40 << SFLS_DATIN), JTAG_COMMON_MODULE_IDX);

	err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	rd_com = (read & 0x00030000) >> WB_RDCMD_IOM;
	err |= jtag_read32(SFLS_REG_BASE + _SFLS_CMD, &read, JTAG_COMMON_MODULE_IDX);
	clk_div = (read & 0xf0000000) >> CLKDIV;
	err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x20021c01 | (rd_com << REG_CMD_IOM) | (rd_com << REG_DAT_IOM) | (clk_div << CLKDIV), JTAG_COMMON_MODULE_IDX);

	err |= sfls_wait_for_reg_request();
	return err;
}


UINT32 Sfls_writeProtect_Enable_all(void)
{
	printf("func : %s \t line : %d\n", __FUNCTION__, __LINE__);
	UINT32 read = 0;
	UINT32 rd_com;
	UINT32 clk_div;
	UINT32 err = 0;

	err |= sfls_write_en();

	err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	read = read & 0x00ffffff;
	err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0x01 << REG_CMD), JTAG_COMMON_MODULE_IDX);

	err |= jtag_write32(SFLS_REG_BASE + _SFLS_DATIN, (0x7e << SFLS_DATIN), JTAG_COMMON_MODULE_IDX);

	err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	rd_com = (read & 0x00030000) >> WB_RDCMD_IOM;
	err |= jtag_read32(SFLS_REG_BASE + _SFLS_CMD, &read, JTAG_COMMON_MODULE_IDX);
	clk_div = (read & 0xf0000000) >> CLKDIV;
	err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x20021c01 | (rd_com << REG_CMD_IOM) | (rd_com << REG_DAT_IOM) | (clk_div << CLKDIV), JTAG_COMMON_MODULE_IDX);

	err |= sfls_wait_for_reg_request();
	return err;
}

UINT32 Sfls_erase_all(void)
{
	UINT32 read = 0;
	UINT32 rd_com;
    UINT32 clk_div;
    UINT32 err = 0;
	
	err |= sfls_write_en();

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	read = read & 0x00ffffff;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0xC7 << REG_CMD), JTAG_COMMON_MODULE_IDX);
		
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	rd_com = (read & 0x00030000) >> WB_RDCMD_IOM;
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_CMD, &read, JTAG_COMMON_MODULE_IDX);
    clk_div = (read & 0xf0000000) >> CLKDIV;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x00020800 | (rd_com << REG_CMD_IOM) | (clk_div << CLKDIV), JTAG_COMMON_MODULE_IDX);
	
    err |= sfls_wait_for_reg_request();
    return err;
}

UINT32 Sfls_write_reg(UCHAR dat)
{
	UINT32 read = 0;
	UINT32 rd_com;
    UINT32 clk_div;
    UINT32 err = 0;
	
	err |= sfls_write_en();

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	read = read & 0x00ffffff;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0x01 << REG_CMD), JTAG_COMMON_MODULE_IDX);

    err |= jtag_write32(SFLS_REG_BASE + _SFLS_DATIN, (dat << SFLS_DATIN), JTAG_COMMON_MODULE_IDX);
	
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	rd_com = (read & 0x00030000) >> WB_RDCMD_IOM;
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_CMD, &read, JTAG_COMMON_MODULE_IDX);
    clk_div = (read & 0xf0000000) >> CLKDIV;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x00021c00 | (rd_com << REG_CMD_IOM) | (rd_com << REG_DAT_IOM) | (clk_div << CLKDIV), JTAG_COMMON_MODULE_IDX);
	
    err |= sfls_wait_for_reg_request();
    return err;
}

UINT32 Sfls_read_reg(BYTE* datout)
{
	UINT32 read = 0;
	UINT32 rd_com;
    UINT32 clk_div;
    UINT32 err = 0; 

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	read = read & 0x00ffffff;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0x05 << REG_CMD), JTAG_COMMON_MODULE_IDX);

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
	rd_com = (read & 0x00030000) >> WB_RDCMD_IOM;
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_CMD, &read, JTAG_COMMON_MODULE_IDX);
    clk_div = (read & 0xf0000000) >> CLKDIV;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x00021000 | (rd_com << REG_CMD_IOM) | (rd_com << REG_DAT_IOM) | (clk_div << CLKDIV), JTAG_COMMON_MODULE_IDX);

    err |= sfls_wait_for_reg_request();
	
    err |= jtag_read32(SFLS_REG_BASE + _SFLS_DATOUT, &read, JTAG_COMMON_MODULE_IDX);
	*datout = (read & 0xff000000) >> SFLS_DATOUT;
 
    return err;
}

UINT32 Sfls_init_quad(void)
{
	UINT32 read = 0;
	UINT32 mask = 0;
	BYTE manu = 0;
    BYTE type = 0;
    BYTE capa = 0;
    UINT32 err = 0;
		
    UINT32 id = 0;
    
    err |= Get_device_ID(&id);

    manu = (id & 0xFF000000) >> 24;
    type = (id & 0x00FF0000) >> 16;
    capa = (id & 0x0000FF00) >> 8;
	printf("Manufacture:%02x Type:%02x Capacity:%02x\n", manu, type, capa);
    
  
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x30010000, JTAG_COMMON_MODULE_IDX);
    err |= sfls_wait_for_reg_request();
#if 0
	switch (manu){
		case 0x1C:
		// EON QUAD

 
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCMD, (0xEB << WB_RDCMD) | (0x02 << WB_WRCMD), JTAG_COMMON_MODULE_IDX);
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, 0x387EAAAA, JTAG_COMMON_MODULE_IDX);
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x10030000, JTAG_COMMON_MODULE_IDX);
			
			/*
		jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, 3);
		read = read | 0x00600000 ;
		jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read, 3);
		*/
	

            err |= sfls_wait_for_reg_request();
		break;
		case 0xc8:
		// 
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCMD, (0xEB << WB_RDCMD) | (0x02 << WB_WRCMD), JTAG_COMMON_MODULE_IDX);

            err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, 0x007CAAAA, JTAG_COMMON_MODULE_IDX);

		//jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x20030000, JTAG_COMMON_MODULE_IDX);

            err |= sfls_wait_for_reg_request();
		break;
        case 0x9d:
        case 0xef:
        case 0xc2:
        // ISSI QUAD
            /*
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMDADR, (0x05 << REG_CMD), JTAG_COMMON_MODULE_IDX);
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x10021000, JTAG_COMMON_MODULE_IDX);
            err |= sfls_wait_for_reg_request();
            jtag_read32(SFLS_REG_BASE + _SFLS_DATOUT, &read, JTAG_COMMON_MODULE_IDX);
            if ((read & 0x40000000) != 0x40000000){
                err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMDADR, (0x06 << REG_CMD), JTAG_COMMON_MODULE_IDX);
                err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x10020000, JTAG_COMMON_MODULE_IDX);
                err |= sfls_wait_for_reg_request();
                err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMDADR, (0x01 << REG_CMD), JTAG_COMMON_MODULE_IDX);
                err |= jtag_write32(SFLS_REG_BASE + _SFLS_DATIN, 0x40000000, JTAG_COMMON_MODULE_IDX);
                err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x10021C00, JTAG_COMMON_MODULE_IDX);
                err |= sfls_wait_for_reg_request();
            }
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCMD, (0xEB << WB_RDCMD) | (0x38 << WB_WRCMD), JTAG_COMMON_MODULE_IDX);
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, 0x007CA020, JTAG_COMMON_MODULE_IDX);
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x30010000, JTAG_COMMON_MODULE_IDX);
            err |= sfls_wait_for_reg_request();
            */
        //Single
         
          
        break;
	default:
            err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0x30010000, JTAG_COMMON_MODULE_IDX);
            err |= sfls_wait_for_reg_request();
		printf("Unknown Sflash!\n");
	}
#endif
	
    return err;
}

#define BLOCK_SIZE 1024*64
#define SECTOR_SIZE 1024*4
#define FAST_ERASE
UINT32 Sfls_erase(UINT32 addr, UINT32 len)
{
	UINT32 i = 0;
	UINT32 remaining_len = 0;
	UINT32 ad;
    UINT32 err = 0;

	ad = addr;

	for (remaining_len = len; remaining_len >= BLOCK_SIZE; remaining_len -= BLOCK_SIZE){
		if (EN673_JTAG_ESC_key_checker()| err) return err;
        if (err |= Sfls_erase_block(ad)) return err;
		ad += BLOCK_SIZE;
		printf("e");
	}
#ifdef FAST_ERASE
	if (remaining_len>0){
        if (err |= Sfls_erase_block(ad)) return err;
		ad += BLOCK_SIZE;
		printf("e");
		remaining_len = 0;
	}
#endif


	for (; remaining_len >= SECTOR_SIZE; remaining_len -= SECTOR_SIZE){
        if (err |= Sfls_erase_sect(ad)) return err;
		ad += SECTOR_SIZE;
		printf("e");
	}

    if (remaining_len>0)	err |= Sfls_erase_sect(ad);

    return err;
}


//#define WRITE_BUF_SIZE 256   // 256 is Flash buffer unit (word)
#define WRITE_BUF_SIZE 256
UINT32 Sfls_write32(UINT32* buf, UINT32 len, UINT32 offset)
{

	UINT32 remaining_len;
	UINT32 pos;
    UINT32 err=0;

	pos = 0;
	for (remaining_len = len; remaining_len >= WRITE_BUF_SIZE; remaining_len -= WRITE_BUF_SIZE){
		if (EN673_JTAG_ESC_key_checker()| err ) return err;
		if (err |= jtag_write_block32(FLASH_BASE + offset + (pos * 4), (UINT32*)buf + pos, WRITE_BUF_SIZE, JTAG_COMMON_MODULE_IDX)) return err;
		pos += WRITE_BUF_SIZE;
		printf("*");
		//Sleep(100);
	}

	if (remaining_len){
		if (err |= jtag_write_block32(FLASH_BASE + offset + (pos * 4), (UINT32*)buf + pos, remaining_len, JTAG_COMMON_MODULE_IDX)) return err;
	}

    return err;
}


#define SECTOR_BYTE_SIZE 1024*4
UINT32 Sfls_point_write32(UINT32 data, UINT32 byteoffset){

    UINT8 memblock[SECTOR_BYTE_SIZE];
	UINT32 adj_offset = 0;
    UINT32 err = 0;

	adj_offset = (byteoffset >> 12) << 12;
	printf("%x-%x", byteoffset, adj_offset);

	err |= jtag_read_block32(FLASH_BASE + adj_offset, (UINT32*)memblock, (SECTOR_BYTE_SIZE / 4), JTAG_COMMON_MODULE_IDX);

	*(memblock + ((byteoffset-adj_offset) / 4)) = data;

    err |= Sfls_erase_sect(adj_offset);

	err |= jtag_write_block32(FLASH_BASE + adj_offset, (UINT32*)memblock, (SECTOR_BYTE_SIZE / 4), JTAG_COMMON_MODULE_IDX);

    return err;
}

UINT32 Sfls_point_write8(UINT8 data, UINT32 byteoffset){
	
    UINT8 memblock[SECTOR_BYTE_SIZE];
	UINT32 adj_offset = 0;
    UINT32 err = 0;

	adj_offset = (byteoffset >> 12) << 12;
	printf("%x-%x", byteoffset, adj_offset);

	err |= jtag_read_block32(FLASH_BASE + adj_offset, (UINT32*)memblock, (SECTOR_BYTE_SIZE / 4), JTAG_COMMON_MODULE_IDX);

	*(memblock + byteoffset) = data;

    err |= Sfls_erase_sect(adj_offset);

	err |= jtag_write_block32(FLASH_BASE + adj_offset, (UINT32*)memblock, (SECTOR_BYTE_SIZE / 4), JTAG_COMMON_MODULE_IDX);

    return err;
}

UINT32 Get_device_ID(UINT32* ID)
{
    UINT32 read = 0;
    UINT32 err = 0;

    sfls_reset();
    Sleep(100);
// exit quad
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCMD, (0x03 << WB_RDCMD) | (0x02 << WB_WRCMD), JTAG_COMMON_MODULE_IDX);
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT,0xff600000, JTAG_COMMON_MODULE_IDX);
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0xf0030200, JTAG_COMMON_MODULE_IDX);
    err |= sfls_wait_for_reg_request();

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_WBCONT, &read, JTAG_COMMON_MODULE_IDX);
    read = read & 0x00ffffff;
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_WBCONT, read | (0x9f << REG_CMD), JTAG_COMMON_MODULE_IDX);
    err |= jtag_write32(SFLS_REG_BASE + _SFLS_CMD, 0xf0021002, JTAG_COMMON_MODULE_IDX);
    err |= sfls_wait_for_reg_request();

    err |= jtag_read32(SFLS_REG_BASE + _SFLS_DATOUT, &read, JTAG_COMMON_MODULE_IDX);
    *ID = read;

    return err;
}

UINT32 Get_device_SIZE(UINT32* byte)
{
    UINT32 err = 0;
    UINT32 id=0;
    err |= Get_device_ID(&id);

    BYTE size;

    size = (id >> 8) & 0xff;

    switch (size)
    {
        case 0x18: *byte = 128; break;
        case 0x17: *byte = 64; break;
        case 0x16: *byte = 32; break;
        default: return err|-1;
    }

    return err;

}

// Only EN672A 
#if !CHANGE_EN674
UINT32 reset_sys_wdt(void){
UINT32 err = 0;
    err |= jtag_write32(SYS_REG_BASE + SYS_WDT_CGF, 2, JTAG_COMMON_MODULE_IDX);
    err |= jtag_write32(CPU_REG_BASE + CPU0_WDT_CGF, 0, JTAG_COMMON_MODULE_IDX);
    err |= jtag_write32(CPU_REG_BASE + CPU1_WDT_CGF, 0, JTAG_COMMON_MODULE_IDX);

    return err;
}
#endif

UINT32 system_reset(void){
    UINT32 err = 0;
    err |= jtag_write32(SYS_REG_BASE + SYS_WDT_LMT, 0x1, JTAG_COMMON_MODULE_IDX);
    err |= jtag_write32(SYS_REG_BASE + SYS_WDT_CGF, 3, JTAG_COMMON_MODULE_IDX);
    return err;
}