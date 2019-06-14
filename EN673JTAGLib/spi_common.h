#ifndef _EXR_COMMON_H_
#define _EXR_COMMON_H_

#include <stdint.h>

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned long ulong;
typedef unsigned int uint;

typedef unsigned long long		ullong;

#define	WARNING(...)	{ fprintf(stderr, "\tWARNING! "); fprintf(stderr, __VA_ARGS__); }
#define ERROR_(...)		{ fprintf(stderr, "\tERROR! "); fprintf(stderr, __VA_ARGS__); }

#ifdef _DEBUG
#define ASSERT(v) 		if(!(v)) { ERROR("@%s : %d\n", __FILE__, __LINE__); exit(1); }
#else
#define	ASSERT(v)		
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef min
#define min(X,Y) 		((X) < (Y) ? (X) : (Y))
#endif

#define HEX(c)  		((c>='a') ? c-'a'+10 : (c>='A') ? c-'A'+10 : c-'0')
//int HEX(int c);

extern const char hexchars[];

uint hex2reg(char *buf);
void reg2hex(uint val, char *buf);

void ascii2hex(char *dst, char *src);
void hex2ascii(char *dst, char *src);


//******************************************************************************
// BENCHMARK
//------------------------------------------------------------------------------
uint bench_tic(void);
extern int debug_bench;
//#define _BENCHMARK
#ifdef _BENCHMARK
#define BENCHMARK_COUNT 1
char* STR(char* format, ...);
extern int bench_cnt;

#define BENCHMARK(str, func)		if(debug_bench)						\
{	bench_cnt++;														\
    uint bench_start_tick = bench_tic();								\
    int bench_i;														\
for (bench_i = 0; bench_i<BENCHMARK_COUNT; bench_i++) { func; }		\
    uint bench_end_tick = bench_tic();									\
    uint bench_us = (bench_end_tick - bench_start_tick) / BENCHMARK_COUNT;	\
for (bench_i = 0; bench_i<bench_cnt; bench_i++) fprintf(stdout, "  "); 	\
    fprintf(stdout, "BENCHMARK ( %s ): %d us %c\n", str, bench_us, \
    (bench_us>1000 ? 'X' : bench_us>100 ? '?' : ' '));			\
    fflush(stdout);														\
    bench_cnt--;														\
}																		
 
#else
#define BENCHMARK(str, func)	func
#endif

#endif//_EXR_COMMON_H_
