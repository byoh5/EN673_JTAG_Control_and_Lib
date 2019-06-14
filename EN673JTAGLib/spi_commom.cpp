#include <stdio.h>
#include <string.h>
#include "spi_common.h"

//******************************************************************************
// Utilities
//------------------------------------------------------------------------------
// String to map hex digits to chars
const char hexchars[] = "0123456789abcdef";

// the value of a HEX char
/*
inline int HEX(int c)
{
return  (c>='a') ? 	c-'a'+10:
(c>='A') ? 	c-'A'+10:
c-'0'	;
//return  ((c >= 'a') && (c <= 'f')) ?	c - 'a' + 10 	:
//      	((c >= '0') && (c <= '9')) ? 	c - '0' 		:
//      	((c >= 'A') && (c <= 'F')) ? 	c - 'A' + 10 	:
//      									-1				;
}
*/

// Convert a hex digit string to a register value
uint hex2reg(char *buf)
{
    int   n;
    uint  val = 0;

    for (n = 0; n<8; n++) {
#ifdef WORDSBIGENDIAN
        int nyb_shift = n * 4;
#else
        int nyb_shift = 28 - (n * 4);
#endif
        val |= HEX(buf[n]) << nyb_shift;
    }

    return val;
}


// Convert a register to a hex digit string
void reg2hex(uint  val, char *buf)
{
    int  n;
    for (n = 0; n<8; n++) {
#ifdef WORDSBIGENDIAN
        int  nyb_shift = n * 4;
#else
        int  nyb_shift = 28 - (n * 4);
#endif
        buf[n] = hexchars[(val >> nyb_shift) & 0xf];
    }

    buf[8] = 0;			// Useful to terminate as string
}


// Convert an ASCII character string to pairs of hex digits
void ascii2hex(char *dst, char *src)
{
    int  i;
    for (i = 0; src[i] != '\0'; i++)	{ // Step through converting the source string
        char  ch = src[i];

        dst[i * 2] = hexchars[ch >> 4 & 0xf];
        dst[i * 2 + 1] = hexchars[ch & 0xf];
    }

    dst[i * 2] = '\0';

}


// Convert pairs of hex digits to an ASCII character string
void hex2ascii(char *dst, char *src)
{
    int  i;

    for (i = 0; src[i * 2] != '\0' && src[i * 2 + 1] != '\0'; i++) {
        dst[i] = ((HEX(src[i * 2]) & 0xf) << 4) | (HEX(src[i * 2 + 1]) & 0xf);
    }

    dst[i] = '\0';

}


//******************************************************************************
// BENCHMARK
//------------------------------------------------------------------------------
#include <time.h>
#include <windows.h>

int bench_cnt = 0;
int debug_bench = 0;

 uint bench_tic(void) // Millisecond
{
    SYSTEMTIME st;
    GetSystemTime(&st);
    return ((st.wMinute * 60 * 1000) + (st.wSecond * 1000) + (st.wMilliseconds));
}

#include <stdarg.h>

inline char* STR(char *format, ...)
{
    static char str[256];

    va_list args;
    va_start(args, format);
    vsprintf(str, format, args);
    va_end(args);

    return str;
}
