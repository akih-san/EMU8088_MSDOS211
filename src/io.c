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

#include "../src/emu88.h"
#include <stdio.h>
//#include <stdlib.h>
#include <assert.h>

#include "../fatfs/ff.h"
#include "../drivers/utils.h"

//
// 8088/V20 Memory address/offset definition for disk device driver
//
#define DRVMAX 4
#define INITTAB			3	// INITTAB offset
#define INITTAB_SIZE	2	// 2 bytes(word)
#define INITTAB_SEG		0x40

#define BPB_drive0_off INITTAB + (INITTAB_SIZE * DRVMAX)
#define BPB_drive1_off BPB_drive0_off + sizeof(DPB)
#define BPB_drive2_off BPB_drive1_off + sizeof(DPB)
#define BPB_drive3_off BPB_drive2_off + sizeof(DPB)

#define drive0	0
#define drive1	1
#define drive2	2
#define drive3	3

//
// Define Disk Parameter Block
//
#define d144_tsec 36
#define d10m_tsec 252
#define d144_media 0xf0
#define d10m_media 0xf8

DPB	dsk1440 = {{{0,0,0},{0,0,0,0,0,0,0,0}},512,1,1,2,224,2880,d144_media,9,d144_tsec};
DPB	dsk10m  = {{{0,0,0},{0,0,0,0,0,0,0,0}},512,8,1,2,512,20160,d10m_media,8,d10m_tsec};

//
// Drive definition
//
drive_t drives[] = {
    { d144_tsec },		// 1.44MB drive
    { d144_tsec },		// 1.44MB drive
    { d10m_tsec },		// 10MB drive
    { d10m_tsec },		// 10MB drive
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 16484 },
};
//const int num_drives = (sizeof(drives)/sizeof(*drives));
const int num_drives = DRVMAX;
int nmi_flg;

	//TIMER0 seconds counter
union {
    unsigned int w; //16 bits Address
    struct {
        unsigned char l; //Address low
        unsigned char h; //Address high
    };
} adjCnt;

TPB tim_pb;			// TIME device parameter block

// console input buffers
#define U3B_SIZE 256

// from unimon
#define CONIN_REQ	0x01
#define CONOUT_REQ	0x02
#define CONST_REQ	0x03
#define STROUT_REQ	0x04
#define MNI_LEDOFF	0x05
// from IO.SYS
#define AUX_INT		0x10
#define PRN_INT		0x20
#define TIM_INT		0x30
#define CON_INT		0x40
#define DSK_INT		0x50

static unsigned char rx_buf[U3B_SIZE];	//UART Rx ring buffer
static unsigned int rx_wp, rx_rp, rx_cnt;
static uint8_t disk_drive;
//static uint8_t disk_track;
static uint16_t disk_sector;
static uint16_t disk_dmal;
static uint16_t disk_dmah;
static uint8_t disk_buf[SECTOR_SIZE];
static uint8_t verify_buf[SECTOR_SIZE];

void io_init(void) {
	rx_wp = 0;
	rx_rp = 0;
	rx_cnt = 0;
    disk_drive = 0;
//    disk_track = 0;
//    disk_sector = 0;
    disk_dmal = 0;
    disk_dmah = 0;
	nmi_flg = 0;

	//initialize TIMER0 & TIM device parameter block

	adjCnt.w = TIMER0_INITC;	// set initial adjust timer counter
	tim_pb.TIM_DAYS = TIM20240101;
	tim_pb.TIM_MINS = 0;
	tim_pb.TIM_HRS = 0;
	tim_pb.TIM_SECS = 0;
	tim_pb.TIM_HSEC = 0;
}

//
// define interrupt
//
// Never called, logically
void __interrupt(irq(default),base(8)) Default_ISR(){}

////////////// TIMER0 vector interrupt ////////////////////////////
//TIMER0 interrupt
/////////////////////////////////////////////////////////////////
void __interrupt(irq(TMR0),base(8)) TIMER0_ISR(){

	union {
    	unsigned int w; //16 bits Address
    	struct {
        	unsigned char l; //Address low
        	unsigned char h; //Address high
    	};
	} tmpCnt;

	TMR0IF =0; // Clear timer0 interrupt flag

	// adjust timer conter
	tmpCnt.l = TMR0L;
	tmpCnt.h = TMR0H;
	tmpCnt.w = tmpCnt.w + adjCnt.w;

	TMR0L = tmpCnt.l;
	TMR0H = tmpCnt.h;

	if( ++tim_pb.TIM_SECS == 60 ) {
		tim_pb.TIM_SECS = 0;
		if ( ++tim_pb.TIM_MINS == 60 ) {
			tim_pb.TIM_MINS = 0;
			if ( ++tim_pb.TIM_HRS == 24 ) {
				tim_pb.TIM_HRS = 0;
				tim_pb.TIM_DAYS++;
			}
		}
	}
	tim_pb.TIM_HSEC = 0;
}

////////////// UART3 Receive interrupt ////////////////////////////
// UART3 Rx interrupt
// PIR9 (bit0:U3RXIF bit1:U3TXIF)
/////////////////////////////////////////////////////////////////
void __interrupt(irq(U3RX),base(8)) URT3Rx_ISR(){

	unsigned char rx_data;

	rx_data = U3RXB;			// get rx data

	if (rx_data == CTL_Q) {
		nmi_flg = 1;
	}
	else {
		if (rx_cnt < U3B_SIZE) {
			rx_buf[rx_wp] = rx_data;
			rx_wp = (rx_wp + 1) & (U3B_SIZE - 1);
			rx_cnt++;
		}
	}
}

// UART3 Transmit
void putch(char c) {
    while(!U3TXIF);             // Wait or Tx interrupt flag set
    U3TXB = c;                  // Write data
}

// UART3 Recive
int getch(void) {
	char c;

	while(!rx_cnt);             // Wait for Rx interrupt flag set
	GIE = 0;                // Disable interrupt
	c = rx_buf[rx_rp];
	rx_rp = (rx_rp + 1) & ( U3B_SIZE - 1);
	rx_cnt--;
	GIE = 1;                // enable interrupt
    return c;               // Read data
}

uint32_t get_physical_addr(uint16_t ah, uint16_t al)
{
// real 32 bit address
//	return (uint32_t)ah*0x1000 + (uint32_t)al;

// 8086 : segment:offset
	return (uint32_t)ah*0x10 + (uint32_t)al;
}

/*////////////////////////////////////////////////////////
;
; Universal monitor I/F
;
;	CONIN_REQ	0x01
;	CONOUT_REQ	0x02
;	CONST_REQ	0x03
;	STROUT_REQ	0x04
;	MNI_LEDOFF	0x05

////////////////////////////////////////////////////////*/

void unimon_console(PTRSAV *u_buff) {

	uint8_t *buf;
	uint16_t cnt;

	switch (u_buff->UREQ_COM) {
		// CONIN
		case CONIN_REQ:
		u_buff->UNI_CHR = (uint8_t)getch();
			break;
		// CONOUT
		case CONOUT_REQ:
			putch((char)u_buff->UNI_CHR);		// Write data
			break;
		// CONST
		case CONST_REQ:
			u_buff->UNI_CHR = (uint8_t)(rx_cnt !=0);
			break;
		case STROUT_REQ:
			buf = tmp_buf[0];
			cnt = (uint16_t)u_buff->UNI_CHR;
			// get string
			read_sram(get_physical_addr(u_buff->STR_SEG, u_buff->STR_off), buf, cnt);
			while( cnt ) {
				putch( *buf++);
				cnt--;
			}
			break;
		case MNI_LEDOFF:
			nmi_flg = 0;
			nmi_sig_off();
	}
	u_buff->UREQ_COM = 0;	// clear unimon request
}

#define CON_CERR	3	// Reserved. (Currently returns error)
#define CON_READ	4	// Character read. (Destructive)
#define CON_RDND	5	// Character read. (Non-destructive)
#define CON_FLSH	7	// Character write.
#define CON_WRIT1	8	// Character write.
#define CON_WRIT2	9	// Character write.

void dsk_sec_err(iodat *req_h) {
	req_h->status = (uint16_t)0x8108;			//set error code & done bits
}

void dsk_drv_err(iodat *req_h) {
	req_h->status = (uint16_t)0x8101;			//set error code & done bits
}

void dsk_rd_err(iodat *req_h) {
	req_h->status = (uint16_t)0x810b;			//set error code & done bits
}

void dsk_wr_err(iodat *req_h) {
	req_h->status = (uint16_t)0x810a;			//set error code & done bits
}

void dsk_crc_err(iodat *req_h) {
	req_h->status = (uint16_t)0x8104;			//set error code & done bits
}

void dsk_bpb_err(iodat *req_h) {
	req_h->status = (uint16_t)0x8107;			//set error code & done bits
}

//void dsk_media_err(iodat *req_h) {
//	req_h->status = (uint16_t)0x8102;			//set error code & done bits
//}

void command_error(iodat *req_h) {
	req_h->status = (uint16_t)0x8103;			//set error code & done bits
}

void busy_exit(iodat *req_h) {
	req_h->status = (uint16_t)0x0300;			//set busy code & done bits
}

void dev_exit(iodat *req_h) {
	req_h->status = (uint16_t)0x0100;			//set done bits
}

void dev_con(iodat *req_h) {

	uint8_t	*buf;
	uint32_t trans_adr;

	buf = tmp_buf[1];
	trans_adr = get_physical_addr(req_h->trans_seg, req_h->trans_off);
	
	switch(req_h->cmd) {
		case CON_CERR:
			command_error(req_h);
			break;

		case CON_READ:
			*buf = (uint8_t)getch();
			write_sram( trans_adr, buf, 1 );
			dev_exit(req_h);
/*
			cnt1 = req_h->count;
			trans_adr = get_physical_addr(req_h->trans_seg, req_h->trans_off);
			cnt2 = 0;
			do {
				*buf++ = (uint8_t)getch();
				cnt1--;
				cnt2++;
				if (cnt2 == TMP_BUF_SIZE) {	// flush buffer
					write_sram( trans_adr, buf, TMP_BUF_SIZE );
					trans_adr += TMP_BUF_SIZE;
					buf = tmp_buf[1];
					cnt2 = 0;
				}
			} while( cnt1 );	// flush remain data
			if (cnt2) write_sram( trans_adr, buf, cnt2 );
			dev_exit(req_h);
*/
			break;

		case CON_RDND:
			if (rx_cnt !=0) {
				req_h->media = rx_buf[rx_rp];
				dev_exit(req_h);
			}
			else busy_exit(req_h);
			break;

		case CON_FLSH:
			GIE = 0;                // Disable interrupt
			rx_wp = 0;
			rx_rp = 0;
			rx_cnt = 0;
			GIE = 1;                // Eable interrupt
			dev_exit(req_h);
			break;

		case CON_WRIT1:
		case CON_WRIT2:
			read_sram( trans_adr, buf, 1 );
			putch(*buf);
/*
			cnt1 = req_h->count;
			trans_adr = get_physical_addr(req_h->trans_seg, req_h->trans_off);

			while ( cnt1 ) {
				if (cnt1 >= TMP_BUF_SIZE) {
					read_sram( trans_adr, buf, TMP_BUF_SIZE );
					cnt2 = TMP_BUF_SIZE;
					cnt1 -= TMP_BUF_SIZE;
					trans_adr += TMP_BUF_SIZE;
				}
				else {
					read_sram( trans_adr, buf, cnt1 );
					cnt2 = cnt1;
					trans_adr += cnt1;
					cnt1 = 0;
				}
				while( cnt2 ) {
					putch(*buf++);
					cnt2--;
				}
			}
*/
		default:
			dev_exit(req_h);
	}
};



#define TIM_ERR		3			//Reserved. (Currently returns an error)
#define TIM_RED		4			//Character read. (Destructive)
#define TIM_BUSY	5			//(Not used, returns busy flag.)
#define TIM_WRT1	8			//Character write.
#define TIM_WRT2	9			//Character write with verify.

void dev_tim(iodat *req_h) {

	uint32_t trans_adr;

	trans_adr = get_physical_addr(req_h->trans_seg, req_h->trans_off);

	switch(req_h->cmd) {
		case TIM_ERR:
			command_error(req_h);
			break;

		case TIM_RED:
			TMR0IE = 0;			// disable timer0 interrupt
			write_sram( trans_adr, (uint8_t *)&tim_pb, sizeof(tim_pb) );
			TMR0IE = 1;			// Enable timer0 interrupt
			dev_exit(req_h);
			break;

		case TIM_BUSY:
			busy_exit(req_h);
			break;

		case TIM_WRT1:
		case TIM_WRT2:
			TMR0IE = 0;			// disable timer0 interrupt
			read_sram( trans_adr, (uint8_t *)&tim_pb, sizeof(tim_pb) );
			TMR0IE = 1;			// Enable timer0 interrupt
			dev_exit(req_h);
			break;

		default:
			dev_exit(req_h);
	}
};



void dev_aux(iodat *req_h) {
	dev_exit(req_h);
};

void dev_prn(iodat *req_h) {
	dev_exit(req_h);
};

#define DSK_INIT	0		// Initialize Driver.
#define MEDIAC		1		// Return current media code.
#define GET_BPB		2		// Get Bios Parameter Block.
#define CMDERR		3		// Reserved. (currently returns error)
#define DSK_RED		4		// Block read.
#define CMDBSY		5		// (Not used, return busy flag)
#define DSK_WRT		8		// Block write.
#define DSK_WRV		9		// Block write with verify.

void set_dskinit_bpb(CMDP *req_h) {

	uint32_t addr;
	
	/* copy Disk Parameter Block to physical memory area */
	addr = get_physical_addr(INITTAB_SEG, BPB_drive0_off);

	write_sram(addr, (uint8_t *)&dsk1440, (unsigned int)sizeof(DPB));		/* set drive A DPB */
	addr += (uint32_t)sizeof(DPB);
	write_sram(addr, (uint8_t *)&dsk1440, (unsigned int)sizeof(DPB));		/* set drive B DPB */
	addr += (uint32_t)sizeof(DPB);

	write_sram(addr, (uint8_t *)&dsk10m, (unsigned int)sizeof(DPB));		/* set drive C DPB */
	addr += (uint32_t)sizeof(DPB);
	write_sram(addr, (uint8_t *)&dsk10m, (unsigned int)sizeof(DPB));		/* set drive D DPB */

	req_h->bpb1 = DRVMAX;				/* max drive */
	req_h->bpb3_off = INITTAB;
	req_h->bpb3_seg = INITTAB_SEG;
}

void dsk_media_check(MEDIAS *req_h) {
	req_h->medias2 = 1;			// No replacement
}

static int set_bpb(BPB *req_h) {
	switch ( req_h->unit ) {
		case drive0:
			req_h->bpb3_off = BPB_drive0_off + sizeof(DPB_HEAD);
			req_h->media = d144_media;
			break;
		case drive1:
			req_h->bpb3_off = BPB_drive1_off + sizeof(DPB_HEAD);
			req_h->media = d144_media;
			break;
		case drive2:
			req_h->bpb3_off = BPB_drive2_off + sizeof(DPB_HEAD);
			req_h->media = d10m_media;
			break;
		case drive3:
			req_h->bpb3_off = BPB_drive3_off + sizeof(DPB_HEAD);
			req_h->media = d10m_media;
			break;
		default:
		return(-1);
	}
	req_h->bpb3_seg = INITTAB_SEG;
	return(0);
}
int set_drive( iodat *req_h ) {
	uint8_t u;
	
	u = req_h->unit;
	if ( u >= DRVMAX ) return( -1 );
	disk_drive = u;
	return(0);
}

/*
int set_tr_sec( iodat *req_h ) {
	DPB *dp;
	div_t ans;
	
	if (disk_drive < 2 ) dp = &dsk1440;
	else dp = &dsk10m;
	if ( req_h->start >= dp->sectors ) return( -1 );
	ans = div((int)req_h->start, (int)dp->sec_trk);
	disk_track = (uint8_t)ans.quot;
	disk_sector = (uint16_t)ans.rem;
	return(0);
}
*/

int setup_drive(iodat *req_h) {
	if ( set_drive( req_h ) ) {
		dsk_drv_err( req_h );
		return( -1 );
	}
	disk_dmal = req_h->trans_off;
	disk_dmah = req_h->trans_seg;

	return( 0 );
}

int seek_disk(iodat *req_h) {
	unsigned int n;
	FRESULT fres;
	FIL *filep = drives[disk_drive].filep;

	if (drives[disk_drive].filep == NULL) return(-1);
	if ((fres = f_lseek(filep, (uint32_t)disk_sector * SECTOR_SIZE)) != FR_OK) {
		printf("f_lseek(): ERROR %d\n\r", fres);
		return(-1);
	}
	return(0);
}

int read_sector(iodat *req_h, uint8_t *buf, int flg) {
	unsigned int n;
	FRESULT fres;
	FIL *filep = drives[disk_drive].filep;
	
	if (seek_disk(req_h)) return(-1);

	// read from the DISK
	if ((fres = f_read(filep, buf, SECTOR_SIZE, &n)) != FR_OK || n != SECTOR_SIZE) {
		printf("f_read(): ERROR res=%d, n=%d\n\r", fres, n);
		return(-1);
	}
	else if (DEBUG_DISK_READ && DEBUG_DISK_VERBOSE && !(debug.disk_mask & (1 << disk_drive))) {
				util_hexdump_sum("buf: ", buf, SECTOR_SIZE);
	}
	else {
		if (flg) {
			// transfer read data to SRAM
			write_sram(get_physical_addr( disk_dmah, disk_dmal ), buf, SECTOR_SIZE);

			#ifdef MEM_DEBUG
			uint32_t addr = get_physical_addr( disk_dmah, disk_dmal );
			printf("f_read(): SRAM address(%08lx),disk_dmah(%04x),disk_dmal(%04x)\n\r", addr, disk_dmah, disk_dmal);
			read_sram(addr, buf, SECTOR_SIZE);
			util_hexdump_sum("RAM: ", buf, SECTOR_SIZE);
			#endif  // MEM_DEBUG
		}
	}
	return(0);
}

int read_disk(iodat *req_h) {
	uint16_t cnt;

	disk_sector = req_h->start;			//set logical sector No from MSDOS.SYS
	cnt = req_h->count;

	while( cnt ) {
		if (read_sector(req_h, disk_buf, 1)) {
			req_h->count -= cnt;		// set number read sectors
			return(-1);
		}
		--cnt;
		disk_dmah += SECTOR_SZPH;		//paragraph of SECTOR_SIZE
		disk_sector++;
	}
	return(0);
}

int write_sector(iodat *req_h) {
	unsigned int n;
	FRESULT fres;
	FIL *filep = drives[disk_drive].filep;
	
	if (seek_disk(req_h)) return(-1);

	// transfer write data from SRAM to the buffer
	read_sram(get_physical_addr( disk_dmah, disk_dmal ), disk_buf, SECTOR_SIZE);

	if (DEBUG_DISK_WRITE && DEBUG_DISK_VERBOSE && !(debug.disk_mask & (1 << disk_drive))) {
		util_hexdump_sum("buf: ", disk_buf, SECTOR_SIZE);
	}

	// write buffer to the DISK
	if ((fres = f_write(filep, disk_buf, SECTOR_SIZE, &n)) != FR_OK || n != SECTOR_SIZE) {
		printf("f_write(): ERROR res=%d, n=%d\n\r", fres, n);
		return(-1);
	}
	else if ((fres = f_sync(filep)) != FR_OK) {
		printf("f_sync(): ERROR %d\n\r", fres);
		return(-1);
	}
	return(0);
}

int write_disk(iodat *req_h) {
	uint16_t cnt;

	disk_sector = req_h->start;
	cnt = req_h->count;
	while( cnt ) {
		if (write_sector(req_h)) {
			req_h->count -= cnt;		// set number read sectors
			return(-1);
		}
		--cnt;
		disk_dmah += SECTOR_SZPH;		//paragraph of SECTOR_SIZE
		disk_sector++;
	}
	return(0);
}

int write_verify(iodat *req_h) {

	uint16_t cnt, i;

	disk_sector = req_h->start;
	cnt = req_h->count;
	while( cnt ) {
		if (write_sector(req_h)) {
			req_h->count -= cnt;		// set number read sectors
			return(-1);
		}
		if (read_sector(req_h, verify_buf, 0)) {
			req_h->count -= cnt;		// set number read sectors
			return(-1);
		}
		for(i=0; i != SECTOR_SIZE; i++) {
			if (disk_buf[i] != verify_buf[i]) {
				req_h->count -= cnt;		// set number read sectors
				return(-1);
			}
		}
		--cnt;
		disk_dmah += SECTOR_SZPH;		//paragraph of SECTOR_SIZE
		disk_sector++;
	}
	return(0);
}

void dev_dsk(iodat *req_h) {
	uint8_t	*buf;
	uint16_t cnt1, cnt2;
	uint32_t trans_adr;

	buf = (uint8_t *)tmp_buf[1];
	switch(req_h->cmd) {
		case DSK_INIT:		// Initialize Driver.
			set_dskinit_bpb((CMDP *)req_h);
			dev_exit(req_h);
			break;
		case MEDIAC:		// Return current media code.
			dsk_media_check( (MEDIAS *)req_h );
			dev_exit(req_h);
			break;
		case GET_BPB:		// Get Bios Parameter Block.
			if (set_bpb((BPB *)req_h)) dsk_bpb_err(req_h);
			else dev_exit(req_h);
			break;
		case CMDERR:		// Reserved. (currently returns error)
			command_error(req_h);
			break;
		case DSK_RED:		// Block read.
			if ( setup_drive(req_h) ) break;
			if ( read_disk(req_h) ) dsk_rd_err(req_h);
			else dev_exit(req_h);
			break;
		case CMDBSY:		// (Not used, return busy flag)
			busy_exit(req_h);
			break;
		case DSK_WRT:		// Block write.
			if ( setup_drive(req_h) ) break;
			if ( write_disk(req_h) ) dsk_wr_err(req_h);
			else dev_exit(req_h);
			break;
		case DSK_WRV:		// Block write with verify.
			if ( setup_drive(req_h) ) break;
			if ( write_verify(req_h) ) {
				dsk_crc_err(req_h);
				break;
			}
		default:
			dev_exit(req_h);
	}
}

//
// bus master handling
// this fanction is invoked at main() after HOLDA = 1
//
// bioreq_buffadr = top address of unimon
//
//;  ---- unimon request
//; UREQ_COM = 1 ; unimon request CONIN  : return char in UNI_CHR
//;          = 2 ; unimon request CONOUT : UNI_CHR = output char
//;          = 3 ; unimon request CONST  : return status in UNI_CHR
//;                                      : ( 0: no key, 1 : key exist )
//;			 = 4 ; unimon request STROUT
//;          = 0 ; unimon request is done
//;
//;  ---- IO.SYS request
//; UREQ_COM = 10h ; AUX INT
//;	   = 20h ; PRN INT
//;	   = 30h ; TIM INT
//;	   = 40h ; CON INT
//;	   = 50h ; DSK INT
//;
void bus_master_operation(void) {

	uint32_t addr;
	PTRSAV u_buff;
	iodat *req_h;

	// read request from 8088/V20
	read_sram(bioreq_buffadr, (uint8_t *)&u_buff, (unsigned int)sizeof(PTRSAV));

	if ( u_buff.UREQ_COM ) {
		unimon_console(&u_buff);
	}
	else {
		// get DOS request header from SRAM into PIC buffer
		req_h = (iodat *)tmp_buf[0];
		addr = (uint32_t)(u_buff.PTRSAV_SEG)*0x10 + (uint32_t)u_buff.PTRSAV_off;
		read_sram(addr, (uint8_t *)req_h, (unsigned int)sizeof(iodat));
		
		switch ( u_buff.DREQ_COM ) {
			case AUX_INT:
				dev_aux(req_h);
				break;

			case PRN_INT:
				dev_prn(req_h);
				break;

			case TIM_INT:
				dev_tim(req_h);
				break;

			case CON_INT:
				dev_con(req_h);
				break;

			case DSK_INT:
				dev_dsk(req_h);
				break;

			default:
				req_h -> status = (uint16_t)0x8103;		//set error code & done bits
				printf("UNKNOWN DEVICE : CMD(%02x)\r\n", u_buff.UREQ_COM);
		}
		write_sram(addr, (uint8_t *)req_h, (unsigned int)sizeof(iodat));	// save request header
	}
	// write end request to SRAM for 8088/V20
	write_sram(bioreq_buffadr, (uint8_t *)&u_buff, 2);	// 2bytes( UREQ_COM & UNI_CHR )
}
