/*
 * Copyright 2012 Luke Dashjr
 * Copyright 2012 Xiangfu <xiangfu@openmobilefree.com>
 * Copyright 2012 Andrew Smith
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

/*
 * Those code should be works fine with V2 and V3 bitstream of Icarus.
 * Operation:
 *   No detection implement.
 *   Input: 64B = 32B midstate + 20B fill bytes + last 12 bytes of block head.
 *   Return: send back 32bits immediately when Icarus found a valid nonce.
 *           no query protocol implemented here, if no data send back in ~11.3
 *           seconds (full cover time on 32bit nonce range by 380MH/s speed)
 *           just send another work.
 * Notice:
 *   1. Icarus will start calculate when you push a work to them, even they
 *      are busy.
 *   2. The 2 FPGAs on Icarus will distribute the job, one will calculate the
 *      0 ~ 7FFFFFFF, another one will cover the 80000000 ~ FFFFFFFF.
 *   3. It's possible for 2 FPGAs both find valid nonce in the meantime, the 2
 *      valid nonce will all be send back.
 *   4. Icarus will stop work when: a valid nonce has been found or 32 bits
 *      nonce range is completely calculated.
 */

#include "config.h"
#include "miner.h"

#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#ifndef WIN32
  #include <termios.h>
  #include <sys/stat.h>
  #include <fcntl.h>
  #ifndef O_CLOEXEC
    #define O_CLOEXEC 0
  #endif
#else
  #include <windows.h>
  #include <io.h>
#endif
#include <math.h>

#include "elist.h"
#include "fpgautils.h"
#include "compat.h"


#define CHIP_GEN1_CORES 8

#define CHIP_GEN 1
#define CHIP_CORES CHIP_GEN1_CORES

extern bool opt_ltc_debug;
extern int opt_chips_count;
extern int opt_chip_clk;
extern bool opt_ltc_nocheck_golden;


int opt_chips_count_max=1; 


// The serial I/O speed - Linux uses a define 'B115200' in bits/termios.h
#define ICARUS_IO_SPEED 115200

// The size of a successful nonce read
#define ICARUS_READ_SIZE 4

// Ensure the sizes are correct for the Serial read
#if (ICARUS_READ_SIZE != 4)
#error ICARUS_READ_SIZE must be 4
#endif
#define ASSERT1(condition) __maybe_unused static char sizeof_uint32_t_must_be_4[(condition)?1:-1]
ASSERT1(sizeof(uint32_t) == 4);


// Fraction of a second, USB timeout is measured in
// i.e. 10 means 1/10 of a second
#define TIME_FACTOR 10
// It's 10 per second, thus value = 10/TIME_FACTOR =
#define ICARUS_READ_FAULT_DECISECONDS 1



struct ICARUS_INFO {

	uint32_t read_count;
	uint64_t golden_speed_percore;// speed pre core per sec


	int check_num;
	int baud;
	int cores_perchip;
	int chips_count_max;
	int chips_count;
	int chip_clk;
	uint32_t clk_header;
	int chips_bit_num;//log2(chips_count_max)

	char core_hash[10];
	char chip_hash[10];
	char board_hash[10];

};


// One for each possible device
static struct ICARUS_INFO **icarus_info;

// Looking for options in --icarus-timing and --icarus-options:
//
// Code increments this each time we start to look at a device
// However, this means that if other devices are checked by
// the Icarus code (e.g. BFL) they will count in the option offset
//
// This, however, is deterministic so that's OK
//
// If we were to increment after successfully finding an Icarus
// that would be random since an Icarus may fail and thus we'd
// not be able to predict the option order
//
// This also assumes that serial_detect() checks them sequentially
// and in the order specified on the command line
//
static int option_offset = -1;

struct device_drv icarus_drv;


uint8_t flush_buf[400];

void flush_uart(int fd)
{
#ifdef WIN32

//read(fd, flush_buf, 100);

	const HANDLE fh = (HANDLE)_get_osfhandle(fd);
	
	PurgeComm(fh, PURGE_RXCLEAR);
#else
	tcflush(fd, TCIFLUSH);
	//read(fd, flush_buf, 100);

#endif

}
	


int log_2(int value)   //非递归判断一个数是2的多少次方  
{  
    int x=0;  
    while(value>1)  
    {  
        value>>=1;  
        x++;  
    }  
    return x;  
}  


uint32_t get_revindex(uint32_t value,int bit_num)
{

	uint32_t newvalue;
	int i;

	
#if CHIP_GEN==1
	value = (value&0x1ff80000)>>(29-bit_num);
#else
#error
#endif

	newvalue=0;

	for(i=0;i<bit_num;i++){
		newvalue = newvalue<<1;
		newvalue += value&0x01;
		value = value>>1;
	}

	return newvalue;

}



static void rev(unsigned char *s, size_t l)
{
	size_t i, j;
	unsigned char t;

	for (i = 0, j = l - 1; i < j; i++, j--) {
		t = s[i];
		s[i] = s[j];
		s[j] = t;
	}
}

#define icarus_open2(devpath, baud, purge)  serial_open(devpath, baud, ICARUS_READ_FAULT_DECISECONDS, purge)
#define icarus_open(devpath, baud)  icarus_open2(devpath, baud, false)

#define ICA_GETS_ERROR -1
#define ICA_GETS_OK 0
#define ICA_GETS_RESTART 1
#define ICA_GETS_TIMEOUT 2

static int icarus_gets(unsigned char *buf, int fd, struct timeval *tv_finish, struct thr_info *thr, int read_count,uint32_t *elapsed_count)
{
	ssize_t ret = 0;
	int rc = 0;
	int read_amount = ICARUS_READ_SIZE;
	bool first = true;

	*elapsed_count = 0;
	// Read reply 1 byte at a time to get earliest tv_finish
	while (true) {
		ret = read(fd, buf, 1);

		if (ret < 0)
			return ICA_GETS_ERROR;

		if (first)
			cgtime(tv_finish);

		if (ret >= read_amount)
			return ICA_GETS_OK;

		if (ret > 0) {
			buf += ret;
			read_amount -= ret;
			first = false;
			continue;
		}
			
		rc++;
		*elapsed_count=rc;
		if (rc >= read_count) {
			if (opt_debug) {
				applog(LOG_DEBUG,
					"Icarus Read: No data in %.2f seconds",
					(float)rc/(float)TIME_FACTOR);
			}
			return ICA_GETS_TIMEOUT;
		}

		if (thr && thr->work_restart) {
			if (opt_debug) {
				applog(LOG_DEBUG,
					"Icarus Read: Work restart at %.2f seconds",
					(float)(rc)/(float)TIME_FACTOR);
			}
			return ICA_GETS_RESTART;
		}
	}
}

static int icarus_write(int fd, const void *buf, size_t bufLen)
{
	size_t ret;

#if 0
	char *hexstr;
	hexstr = bin2hex(buf, bufLen);
	applog(LOG_WARNING, "icarus_write %s", hexstr);
	free(hexstr);
#endif

	ret = write(fd, buf, bufLen);
	if (unlikely(ret != bufLen))
		return 1;

	return 0;
}

#define icarus_close(fd) close(fd)

static void do_icarus_close(struct thr_info *thr)
{
	struct cgpu_info *icarus = thr->cgpu;
	icarus_close(icarus->device_fd);
	icarus->device_fd = -1;
}


extern void suffix_string(uint64_t val, char *buf, int sigdigits);
int update_num(int chips_count)
{
	int i;
	for (i=1;i<1024;i=i*2){
		if (chips_count<=i){
			return i;
		}
	}
	return 1024;
}

static bool icarus_detect_one(const char *devpath)
{
	int this_option_offset = ++option_offset;

	struct ICARUS_INFO *info;
	int fd;
	struct timeval tv_start, tv_finish;
	struct timeval golden_tv;
	
	int numbytes = 84;
	
	int baud, cores_perchip, chips_count_max,chips_count;
	uint32_t clk_reg;
	uint32_t clk_reg_init;
	uint64_t golden_speed_percore;
	
#if 1	
	if(opt_chip_clk>(0xff*3/2)){
		opt_chip_clk = 0xff*3/2;
	}
	else if(opt_chip_clk<2){
		opt_chip_clk = 2;
	}

	clk_reg= (uint32_t)(opt_chip_clk*2/3);
#endif
		
	char clk_header_str[10];
		
	

#if 1
	char golden_ob[] =
		"55aa0001"
		"00038000063b0b1b028f32535e900609c15dc49a42b1d8492a6dd4f8f15295c989a1decf584a6aa93be26066d3185f55ef635b5865a7a79b7fa74121a6bb819da416328a9bd2f8cef72794bf02000000";

	char golden_ob2[] =
		"55aa00ff"
		"c00278894532091be6f16a5381ad33619dacb9e6a4a6e79956aac97b51112bfb93dc450b8fc765181a344b6244d42d78625f5c39463bbfdc10405ff711dc1222dd065b015ac9c2c66e28da7202000000";

	const char golden_nonce[] = "00038d26";
	const uint32_t golden_nonce_val = 0x00038d26;// 0xd26= 3366
#endif




	unsigned char ob_bin[84], nonce_bin[ICARUS_READ_SIZE];
	char *nonce_hex;


	baud = ICARUS_IO_SPEED;
	cores_perchip = CHIP_CORES;
	chips_count = opt_chips_count;
	if(chips_count>opt_chips_count_max){
		opt_chips_count_max = update_num(chips_count);
	}
	chips_count_max = opt_chips_count_max;


		applog(LOG_DEBUG, "Icarus Detect: Attempting to open %s", devpath);
		
		fd = icarus_open2(devpath, baud, true);
		if (unlikely(fd == -1)) {
			applog(LOG_ERR, "Icarus Detect: Failed to open %s", devpath);
			return false;
		}
		
	uint32_t clk_header;



	//from 150M step to the high or low speed. we need to add delay and resend to init chip


	if(clk_reg>(150*2/3)){
		clk_reg_init = 165*2/3;
	}
	else {
		clk_reg_init = 139*2/3;
	}

	
	flush_uart(fd);
		
	clk_header = (clk_reg_init<<24)+((0xff-clk_reg_init)<<16);
	sprintf(clk_header_str,"%08x",clk_header+0x01);
	memcpy(golden_ob2,clk_header_str,8);
		
	hex2bin(ob_bin, golden_ob2, numbytes);
	icarus_write(fd, ob_bin, numbytes);
	sleep(1);
	flush_uart(fd);
	icarus_write(fd, ob_bin, numbytes);
	sleep(1);
	flush_uart(fd);
	icarus_write(fd, ob_bin, numbytes);
	read(fd, flush_buf, 400);


	clk_header = (clk_reg<<24)+((0xff-clk_reg)<<16);
	sprintf(clk_header_str,"%08x",clk_header+0x01);
	memcpy(golden_ob2,clk_header_str,8);
		
	hex2bin(ob_bin, golden_ob2, numbytes);
	icarus_write(fd, ob_bin, numbytes);
	sleep(1);
	flush_uart(fd);
	icarus_write(fd, ob_bin, numbytes);
	sleep(1);
	flush_uart(fd);



	clk_header = (clk_reg<<24)+((0xff-clk_reg)<<16);
	sprintf(clk_header_str,"%08x",clk_header+1);
	memcpy(golden_ob,clk_header_str,8);


	if (opt_ltc_nocheck_golden==false){
		
		read(fd, flush_buf, 400);
		hex2bin(ob_bin, golden_ob, numbytes);	
		icarus_write(fd, ob_bin, numbytes);
		cgtime(&tv_start);

		memset(nonce_bin, 0, sizeof(nonce_bin));
		
		uint32_t elapsed_count;
		icarus_gets(nonce_bin, fd, &tv_finish, NULL, 50,&elapsed_count);

		icarus_close(fd);

		nonce_hex = bin2hex(nonce_bin, sizeof(nonce_bin));
		if (strncmp(nonce_hex, golden_nonce, 8)) {
			applog(LOG_ERR,
				"Icarus Detect: "
				"Test failed at %s: get %s, should: %s",
				devpath, nonce_hex, golden_nonce);
			free(nonce_hex);
			return false;
		}

		timersub(&tv_finish, &tv_start, &golden_tv);

		golden_speed_percore = (uint64_t)(((double)0xd26)/((double)(golden_tv.tv_sec) + ((double)(golden_tv.tv_usec))/((double)1000000)));

		if(opt_ltc_debug){
			applog(LOG_ERR,
				"[Test succeeded] at %s: got %s.",
					devpath, nonce_hex);
		}
		
		free(nonce_hex);
	}
	else{
		
		icarus_close(fd);
		golden_speed_percore = (((opt_chip_clk*2)/3)*1024)/8;
	}

	/* We have a real Icarus! */
	struct cgpu_info *icarus;
	icarus = calloc(1, sizeof(struct cgpu_info));
	icarus->drv = &icarus_drv;
	icarus->device_path = strdup(devpath);
	icarus->device_fd = -1;
	icarus->threads = 1;
	add_cgpu(icarus);
	icarus_info = realloc(icarus_info, sizeof(struct ICARUS_INFO *) * (total_devices + 1));

	applog(LOG_INFO, "Found Icarus at %s, mark as %d",
		devpath, icarus->device_id);

	applog(LOG_DEBUG, "Icarus: Init: %d baud=%d cores_perchip=%d chips_count=%d",
		icarus->device_id, baud, cores_perchip, chips_count);

	// Since we are adding a new device on the end it needs to always be allocated
	icarus_info[icarus->device_id] = (struct ICARUS_INFO *)malloc(sizeof(struct ICARUS_INFO));
	if (unlikely(!(icarus_info[icarus->device_id])))
		quit(1, "Failed to malloc ICARUS_INFO");

	info = icarus_info[icarus->device_id];

	// Initialise everything to zero for a new device
	memset(info, 0, sizeof(struct ICARUS_INFO));

	info->check_num = 0x1234;
	info->baud = baud;
	info->cores_perchip = cores_perchip;
	info->chips_count = chips_count;
	info->chips_count_max= chips_count_max;
	if ((chips_count_max &(chips_count_max-1))!=0){
		quit(1, "chips_count_max  must be 2^n");
	}	
	info->chips_bit_num = log_2(chips_count_max);
	info->golden_speed_percore = golden_speed_percore;


	info->read_count = (uint32_t)((4294967296*10)/(cores_perchip*chips_count_max*golden_speed_percore*2));

	info->read_count = info->read_count*3/4;


	info->chip_clk=opt_chip_clk;
	
	info->clk_header=clk_header;
	
	suffix_string(golden_speed_percore, info->core_hash, 0);
	suffix_string(golden_speed_percore*cores_perchip, info->chip_hash, 0);
	suffix_string(golden_speed_percore*cores_perchip*chips_count, info->board_hash, 0);

	if(opt_ltc_debug){
		applog(LOG_ERR,
			"[Speed] %dMhz core|chip|board: [%s/s], [%s/s], [%s/s], readcount:%d,bitnum:%d ",
				info->chip_clk,info->core_hash,info->chip_hash,info->board_hash,info->read_count,info->chips_bit_num);

	}

	return true;
}

static void icarus_detect()
{
	serial_detect(&icarus_drv, icarus_detect_one);
}

static bool icarus_prepare(struct thr_info *thr)
{
	struct cgpu_info *icarus = thr->cgpu;

	struct timeval now;

	icarus->device_fd = -1;

	int fd = icarus_open(icarus->device_path, icarus_info[icarus->device_id]->baud);
	if (unlikely(-1 == fd)) {
		applog(LOG_ERR, "Failed to open Icarus on %s",
		       icarus->device_path);
		return false;
	}

	icarus->device_fd = fd;

	applog(LOG_INFO, "Opened Icarus on %s", icarus->device_path);
	cgtime(&now);
	get_datestamp(icarus->init, &now);

	return true;
}

void update_chip_stat(struct ICARUS_INFO *info,uint32_t nonce);


static int64_t icarus_scanhash(struct thr_info *thr, struct work *work,
				__maybe_unused int64_t max_nonce)
{
	struct cgpu_info *icarus;
	int fd;
	int ret;

	struct ICARUS_INFO *info;

	int numbytes = 84;				// KRAMBLE 76 byte protocol

	
	unsigned char ob_bin[84], nonce_bin[ICARUS_READ_SIZE];
	char *ob_hex;
	uint32_t nonce;
	int64_t hash_count;
	uint32_t mask;
	struct timeval tv_start, tv_finish, elapsed;
	int curr_hw_errors, i;
	bool was_hw_error;


	int64_t estimate_hashes;

	elapsed.tv_sec = elapsed.tv_usec = 0;

	icarus = thr->cgpu;
	if (icarus->device_fd == -1)
		if (!icarus_prepare(thr)) {
			applog(LOG_ERR, "%s%i: Comms error", icarus->drv->name, icarus->device_id);
			dev_error(icarus, REASON_DEV_COMMS_ERROR);

			// fail the device if the reopen attempt fails
			return -1;
		}

	fd = icarus->device_fd;


	info = icarus_info[icarus->device_id];

	uint32_t clock = info->clk_header;


	int diff = floor(work->device_diff);

	if(diff<info->chips_count){
		diff=info->chips_count;
	}
	
	
	uint32_t target_me = 0xffff/diff;

	uint32_t header = clock+target_me;
	
#if !defined (__BIG_ENDIAN__) && !defined(MIPSEB)
		header = header;

#else
		header = swab32(header);
#endif

	memcpy(ob_bin,(uint8_t *)&header,4);
	memcpy(&ob_bin[4], work->data, 80);	
	rev(ob_bin, 4);
	rev(ob_bin+4, 80);


	if (opt_ltc_debug&1) {
		//ob_hex = bin2hex(ob_bin, sizeof(ob_bin));
		ob_hex = bin2hex(ob_bin, 8);
		applog(LOG_ERR, "Icarus %d nounce2 = %s readcount = %d try sent: %s",
			icarus->device_id,work->nonce2,info->read_count, ob_hex);
		free(ob_hex);
	}


	read(fd, flush_buf, 400);
	ret = icarus_write(fd, ob_bin, 84); 
	if (ret) {
		do_icarus_close(thr);
		applog(LOG_ERR, "%s%i: Comms error", icarus->drv->name, icarus->device_id);
		dev_error(icarus, REASON_DEV_COMMS_ERROR);
		return 0;	/* This should never happen */
	}


	cgtime(&tv_start);


	/* Icarus will return 4 bytes (ICARUS_READ_SIZE) nonces or nothing */
	memset(nonce_bin, 0, sizeof(nonce_bin));


	if (opt_ltc_debug&0) {
		applog(LOG_ERR, "diff is %d",diff);
	}

	uint32_t elapsed_count;
	uint32_t read_count = info->read_count;
	while(1){
		
		ret = icarus_gets(nonce_bin, fd, &tv_finish, thr, read_count,&elapsed_count);
		if (ret == ICA_GETS_ERROR) {
			do_icarus_close(thr);
			applog(LOG_ERR, "%s%i: Comms error", icarus->drv->name, icarus->device_id);
			dev_error(icarus, REASON_DEV_COMMS_ERROR);
			return 0;
		}
#ifndef WIN32
//openwrt
		flush_uart(fd);
#endif

		work->blk.nonce = 0xffffffff;

		// aborted before becoming idle, get new work
		if (ret == ICA_GETS_TIMEOUT || ret == ICA_GETS_RESTART) {

			if (opt_ltc_debug&1) {
				applog(LOG_ERR, "1restart or 2timeout:%d ",ret);
			}


			timersub(&tv_finish, &tv_start, &elapsed);

			estimate_hashes = ((double)(elapsed.tv_sec) + ((double)(elapsed.tv_usec))/((double)1000000))
								* info->golden_speed_percore*info->chips_count*info->cores_perchip;

			if (unlikely(estimate_hashes > 0xffffffff))
				estimate_hashes = 0xffffffff;


			return estimate_hashes;
		}

		if(read_count>elapsed_count){
			read_count -= elapsed_count;
		}
		else {
			read_count=0;
		}

		memcpy((char *)&nonce, nonce_bin, sizeof(nonce_bin));
		//rev(nonce_bin,4);
#if !defined (__BIG_ENDIAN__) && !defined(MIPSEB)
		nonce = swab32(nonce);
#endif


		curr_hw_errors = icarus->hw_errors;

		submit_nonce(thr, work, nonce);
		
		was_hw_error = (curr_hw_errors < icarus->hw_errors);

		if (was_hw_error){
			
			flush_uart(fd);
			if (opt_ltc_debug&&1) {
				applog(LOG_ERR, "ERR nonce:%08x ",nonce);
			}
		}
		else {
			
			if (opt_ltc_debug&&0) {
#if CHIP_GEN==1
				uint32_t chip_index=get_revindex(nonce,info->chips_bit_num);
				uint32_t core_index=(nonce&0xe0000000)>>29;
#else
#error
#endif
				applog(LOG_ERR, "nonce:%08x,chip_index:%d ",nonce,chip_index);
			}

			

		}

	}

}



static struct api_data *icarus_api_stats(struct cgpu_info *cgpu)
{
	struct api_data *root = NULL;
	struct ICARUS_INFO *info = icarus_info[cgpu->device_id];

	// Warning, access to these is not locked - but we don't really
	// care since hashing performance is way more important than
	// locking access to displaying API debug 'stats'
	// If locking becomes an issue for any of them, use copy_data=true also
	root = api_add_string(root, "golden_speed_chip", info->chip_hash, false);
	root = api_add_int(root, "chipclk", &(info->chip_clk), false);
	root = api_add_int(root, "chips_count", &(info->chips_count), false);
	root = api_add_int(root, "chips_count_max", &(info->chips_count_max), false);
	root = api_add_uint32(root, "readcount", &(info->read_count), false);


	return root;
}

static void icarus_shutdown(struct thr_info *thr)
{
	do_icarus_close(thr);
}

struct device_drv icarus_drv = {
	.drv_id = DRIVER_ICARUS,
	.dname = "Icarus",
	.name = "ICA",
	.max_diff = 32768,
	.drv_detect = icarus_detect,
	.get_api_stats = icarus_api_stats,
	.thread_prepare = icarus_prepare,
	.scanhash = icarus_scanhash,
	.thread_shutdown = icarus_shutdown,
};
