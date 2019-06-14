#ifndef _SPI_FLASH_H_
#define _SPI_FLASH_H_

#include "spi_common.h"

// ftdx functions
int spi_ftdx_init(uint clkdiv, uint rd_timeout, uint wr_timeout, uint latency);
int spi_ftdx_reset_target(void);
int spi_ftdx_toggle_cs(int cs);
int spi_ftdx_write_bytes(uchar* wbuf, ushort len);
int spi_ftdx_read_bytes(uchar* rbuf, ushort len);
int spi_ftdx_release_target(void);
int spi_ftdx_release_port(void);
int spi_ftdx_close(void);

// flash low-level functions
int spi_flash_reset(void);
int spi_flash_write_en(void);
int spi_flash_write_wrsr(uchar wdat);
int spi_flash_read_id(uint* id);
int spi_flash_wait_for_busy(void);

// flash mid-level functions
int spi_flash_remove_protect(void);
int spi_flash_erase_block(uint adr);
int spi_flash_erase_sector(uint adr);
int spi_flash_write_page(uchar* wbuf, uint adr, uint len);
int spi_flash_read(uchar* rbuf, uint adr, uint len);

// flash top-level functions
int spi_flash_read_bin(char* fname, uchar** buf, int* filesize);
int spi_flash_erase(uint adr, uint len);
int spi_flash_write(uchar* wbuf, uint adr, uint len);
int spi_flash_verify(uchar* wbuf, uint adr, uint len);

int spi_flash_download(char* fname, uint adr);

// register top-level functions
int spi_reg_read(ushort adr, uint* rdat);
int spi_reg_write(ushort adr, uint wdat);

extern int debug_spi;


#define MAX_RETRY_CNT   10000

#endif//_SPI_FLASH_H_