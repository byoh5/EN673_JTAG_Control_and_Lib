


#define CLI_CMD_LEN 		256

// Command structure


typedef struct {
	char *name;
	UINT(*func)(char*);
	const char	*help;
} TCLI_CMD;

#define	ADDR_MASK		0xFFFFFFFC

#define JPEG_VIEW_ADDR       0xF0010000
#define JTAG_VIEW_ADDRSIZEP  0xF0010004
#define JTAG_VIEW_FLAG       0xF0010008
#define FLAG_INTRO  9
#define FLAG_OUTRO  0
#define SIZE_ZERO   0

#define MAX_JPEG_SIZE 1024*1024
#define READ_JTAG_BUF        (1024*8)


enum {
	eESC_ImageSend		= 0x00000001,
	eESC_DageSend		= 0x00000002
};