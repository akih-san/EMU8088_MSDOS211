/*
 * UART, disk I/O and monitor firmware for EMU8088
 *
 * Based on main.c by Tetsuya Suzuki and emuz80_z80ram.c by Satoshi Okue
 * Base source code by @hanyazou https://twitter.com/hanyazou
 * Modified by Akihito Honda
 */
/*!
 * PIC18F47Q43/PIC18F47Q83/PIC18F47Q84 ROM image uploader and UART emulation firmware
 * This single source file contains all code
 *
 * Target: EMUZ80 with Z80+RAM
 * Compiler: MPLAB XC8 v2.40
 *
 * Modified by Satoshi Okue https://twitter.com/S_Okue
 * Version 0.1 2022/11/15
 */

/*
    PIC18F57Q43 ROM RAM and UART emulation firmware
    This single source file contains all code
    Original source code for PIC18F47Q43 ROM RAM and UART emulation firmware
    Designed by @hanyazou https://twitter.com/hanyazou

    Target: EMU8088 - The computer with only 8088/V20 and PIC18F57Q43
    Written by Akihito Honda
*/

#ifndef __SUPERMEZ80_H__
#define __SUPERMEZ80_H__

#include "../src/picconfig.h"
#include <xc.h>
#include <stdint.h>
#include "../fatfs/ff.h"

//
// Configlations
//

#define P64 15.625

#define ENABLE_DISK_DEBUG

#define NUM_FILES        6
#define SECTOR_SIZE      512
#define SECTOR_SZPH      SECTOR_SIZE >> 4

#define TMP_BUF_SIZE     256

#define MEM_CHECK_UNIT	TMP_BUF_SIZE * 16	// 4 KB
#define MAX_MEM_SIZE	0x00100000			// 1 MB
#define bioreq_buffadr	0xff500				// bios(unimon request IO header address)

//#define TIMER0_INITC	0x86e8	//Theoretically value
//#define TIMER0_INITCH	0x86
//#define TIMER0_INITCL	0xe8
#define TIMER0_INITC	0x87e1	//Actual value
#define TIMER0_INITCH	0x87
#define TIMER0_INITCL	0xe1
//
// Constant value definitions
//

#define UART_DREG			0x00	// 00h Data REG
#define UART_CREG			0x02	// 00h Control REG
#define DISK_REG_DRIVE		0x10	// 10h fdc-port: # of drive
#define DISK_REG_TRACK		0x12	// 12h fdc-port: # of track
#define DISK_REG_SECTOR		0x14	// 14h fdc-port: # of sector(0-7bit)
#define DISK_REG_SECTORH	0x15	// 15h fdc-port: # of sector high(8-15bit)

#define DISK_REG_FDCOP	0x20	// 20h fdc-port: command
#define DISK_REG_FDCST	0x22	// 22h fdc-port: status

#define DISK_REG_DMALL	0x24	// 24h dma-port: dma address 0-7bit
#define DISK_REG_DMALH	0x25	// 25h dma-port: dma address 8-15bit
#define DISK_REG_DMAHL	0x26	// 26h dma-port: dma address 16-23bit
#define DISK_REG_DMAHH	0x27	// 27h dma-port: dma address 24-31bit

#define DISK_OP_READ	0
#define DISK_OP_WRITE	1

#define NMI_SIG_OFF		0x30	// 30h NMI signal off (LED OFF)

#define CTL_Q 0x11

// Status register
// bit 76543210
//     WR....EB
//
// bit 7 : 1=Write operation
// bit 6 : 1=Read operation
// bit 1 : 1=Error
// bit 0 : 1=Busy, 0=Ready

#define DISK_ST_READY		0x00
#define DISK_ST_BUSY		0x01
#define DISK_ST_ERROR		0x02
#define DISK_ST_READ		0x40
#define DISK_ST_WRITE		0x80

#define IOSYS_OFF        0x400

//
// Type definitions
//

// Address Bus
union address_bus_u {
    uint32_t w;             // 32 bits Address
    struct {
        uint8_t ll;        // Address L low
        uint8_t lh;        // Address L high
        uint8_t hl;        // Address H low
        uint8_t hh;        // Address H high
    };
};

union io_address {
	uint16_t adr;
	struct {
		uint8_t l8;
		uint8_t h8;
	};
};

typedef struct {
    unsigned int sectors;
    FIL *filep;
} drive_t;

typedef struct {
    uint8_t disk;
    uint8_t disk_read;
    uint8_t disk_write;
    uint8_t disk_verbose;
    uint16_t disk_mask;
} debug_t;

typedef struct {
    uint8_t *addr;
    uint16_t offs;
    unsigned int len;
} mem_region_t;

typedef struct {
	uint8_t  cmd_len;		// LENGTH OF THIS COMMAND
	uint8_t  unit;			// SUB UNIT SPECIFIER
	uint8_t  cmd;			// COMMAND CODE
	uint16_t status;		// STATUS
	uint8_t  reserve[8];	// RESERVE
	uint8_t  media;			// MEDIA DESCRIPTOR
	uint16_t trans_off;		// TRANSFER OFFSET
	uint16_t trans_seg;		// TRANSFER SEG
	uint16_t count;			// COUNT OF BLOCKS OR CHARACTERS
	uint16_t start;			// FIRST BLOCK TO TRANSFER
} iodat;

typedef struct {
	uint8_t  cmd_len;		// LENGTH OF THIS COMMAND
	uint8_t  unit;			// SUB UNIT SPECIFIER
	uint8_t  cmd;			// COMMAND CODE
	uint16_t status;		// STATUS
	uint8_t  reserve[8];	// RESERVE
	uint8_t  bpb1;			// number of support drives.
	uint16_t bpb2_off;		// DWORD transfer address.
	uint16_t bpb2_seg;
	uint16_t bpb3_off;		// DWORD pointer to BPB
	uint16_t bpb3_seg;
	uint8_t  bdev_no;		// block device No.
} CMDP;

typedef struct {
	uint8_t  UREQ_COM;		// unimon CONIN/CONOUT request command
	uint8_t  UNI_CHR;		// charcter (CONIN/CONOUT) or number of strings
	uint16_t STR_off;		// unimon string offset
	uint16_t STR_SEG;		// unimon string segment
	uint8_t  DREQ_COM;		// device request command
	uint8_t  DEV_RES;		// reserve
	uint16_t PTRSAV_off;	// request header offset
	uint16_t PTRSAV_SEG;	// request header segment
} PTRSAV;

typedef struct {
	uint8_t  jmp_ner[3];	// Jmp Near xxxx  for boot.
	uint8_t  mane_var[8];	// Name / Version of OS.
} DPB_HEAD;

typedef struct {
	DPB_HEAD reserve;
//-------  Start of Drive Parameter Block.
	uint16_t sec_size;		// Sector size in bytes.                  (dpb)
	uint8_t  alloc;			// Number of sectors per alloc. block.    (dpb)
	uint16_t res_sec;		// Reserved sectors.                      (dpb)
	uint8_t  fats;			// Number of FAT's.                       (dpb)
	uint16_t max_dir;		// Number of root directory entries.      (dpb)
	uint16_t sectors;		// Number of sectors per diskette.        (dpb)
	uint8_t  media_id;		// Media byte ID.                         (dpb)
	uint16_t fat_sec;		// Number of FAT Sectors.                 (dpb)
//-------  End of Drive Parameter Block.
	uint16_t sec_trk;		// Number of Sectors per track.
} DPB;

typedef struct {
	uint8_t  cmd_len;		// LENGTH OF THIS COMMAND
	uint8_t  unit;			// SUB UNIT SPECIFIER
	uint8_t  cmd;			// COMMAND CODE
	uint16_t status;		// STATUS
	uint8_t  reserve[8];	// RESERVE
	uint8_t  medias1;		//Media byte.
	uint8_t  medias2;		//Media status byte flag.
} MEDIAS;

typedef struct {
	uint8_t  cmd_len;		// LENGTH OF THIS COMMAND
	uint8_t  unit;			// SUB UNIT SPECIFIER
	uint8_t  cmd;			// COMMAND CODE
	uint16_t status;		// STATUS
	uint8_t  reserve[8];	// RESERVE
	uint8_t  media;			// MEDIA DESCRIPTOR
	uint16_t bpb2_off;		// DWORD transfer address.
	uint16_t bpb2_seg;
	uint16_t bpb3_off;		// DWORD pointer to BPB
	uint16_t bpb3_seg;
} BPB;

typedef struct {
	uint16_t TIM_DAYS;		//Number of days since 1-01-1980.
	uint8_t  TIM_MINS;		//Minutes.
	uint8_t  TIM_HRS;		//Hours.
	uint8_t  TIM_SECS;		//Seconds.
	uint8_t  TIM_HSEC;		//Hundreths of a second.
} TPB;

#define TIM20240101	16071	// 16071days from 1980

//
// Global variables and function prototypes
//

extern uint8_t tmp_buf[2][TMP_BUF_SIZE];
extern debug_t debug;
extern int nmi_flg;

extern void reset_ioreq(void);
extern void io_init(void);
extern int io_stat(void);
extern int getch(void);
extern drive_t drives[];
extern const int num_drives;

extern void mem_init(void);

	//TIMER0 seconds counter
//extern union secCnt;
//extern union adjCnt;

// board

extern void board_init(void);
extern void (*board_sys_init_hook)(void);
#define board_sys_init() (*board_sys_init_hook)()
extern void (*board_start_i88_hook)(void);
#define board_start_i88() (*board_start_i88_hook)()

extern void write_sram(uint32_t addr, uint8_t *buf, unsigned int len);
extern void read_sram(uint32_t addr, uint8_t *buf, unsigned int len);
extern void board_event_loop(void);
extern void bus_master_operation(void);

// RD    read only
extern __bit (*board_rd_pin_hook)(void);
#define rd_pin() (board_rd_pin_hook?(*board_rd_pin_hook)():1)
extern __bit (*board_wr_pin_hook)(void);
#define wr_pin() (board_wr_pin_hook?(*board_wr_pin_hook)():1)

//NMI
extern void nmi_sig_off(void);
extern void nmi_sig_on(void);

//
// debug macros
//
#ifdef ENABLE_DISK_DEBUG
#define DEBUG_DISK (debug.disk || debug.disk_read || debug.disk_write || debug.disk_verbose)
#define DEBUG_DISK_READ (debug.disk_read)
#define DEBUG_DISK_WRITE (debug.disk_write)
#define DEBUG_DISK_VERBOSE (debug.disk_verbose)
#else
#define DEBUG_DISK 0
#define DEBUG_READ 0
#define DEBUG_WRITE 0
#define DEBUG_DISK_VERBOSE 0
#endif

#endif  // __SUPERMEZ80_H__
