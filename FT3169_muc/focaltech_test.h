/************************************************************************
* Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
*
* File Name: focaltech_test.h
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-01
*
* Abstract: test entry for all IC
*
************************************************************************/
#ifndef _FOCALTECH_TEST_H
#define _FOCALTECH_TEST_H

/*****************************************************************************
* Included header files
*****************************************************************************/

#include "focaltech_core.h"


/*****************************************************************************
* Macro definitions using #define
*****************************************************************************/


#define WEAKSHORT_TEST_EN           1
#define RAWDATA_TEST_EN             1
#define NOISE_TEST_EN               1
#define SCAP_CB_TEST_EN             1
#define SCAP_RAWDATA_TEST_EN        1
#define MCAP_CMB_TEST_EN            1


#define BYTES_PER_TIME                          (128)  /* max:128 */

/*-----------------------------------------------------------
Test Status
-----------------------------------------------------------*/
#define RESULT_NULL                             0
#define RESULT_PASS                             1
#define RESULT_NG                               2

#define TX_NUM_MAX                              60
#define RX_NUM_MAX                              100

/*
 * factory test registers
 */
#define ENTER_WORK_FACTORY_RETRIES              5
#define READ_NOISE_DATA_RETRIES                 8

#define FACTORY_TEST_RETRY                      100
#define FACTORY_TEST_DELAY                      50
#define FACTORY_TEST_RETRY_DELAY                100

#define DEVIDE_MODE_ADDR                        0x00
#define REG_FW_VERSION                          0xA6

/* ms_sc */
#define FACTORY_REG_LINE_ADDR                   0x01
#define FACTROY_REG_SHORT_TEST_EN               0x07
#define FACTORY_REG_SCAN_ADDR2                  0x08
#define FACTORY_REG_CHX_NUM                     0x02
#define FACTORY_REG_CHY_NUM                     0x03

#define FACTORY_REG_SC_CB_ADDR_OFF              0x33
#define FACTORY_REG_SC_CB_ADDR                  0x39
#define FACTORY_REG_RAWDATA_SADDR_SC            0x34
#define FACTORY_REG_RAWDATA_ADDR_MC_SC          0x36
#define FACTORY_REG_RESETPIN_ADDR_SC            0x88
#define FACTORY_REG_INT_ADDR_SC                 0x20

#define FACTORY_REG_MCB_OFF                     0x26
#define FACTORY_REG_MCB_ADDR                    0x3A
#define FACTORY_REG_MC_SC_MODE                  0x44
#define FACTORY_REG_MC_SC_CB_ADDR_OFF           0x45
#define FACTORY_REG_MC_SC_CB_H_ADDR_OFF         0x49

#define FACTORY_REG_MC_SC_CB_ADDR               0x4E

/*****************************************************************************
* enumerations, structures and unions
*****************************************************************************/
enum wp_type {
    WATER_PROOF_OFF = 0,
    WATER_PROOF_ON = 1,
    HIGH_SENSITIVITY = 2,
    HOV = 3,
    WATER_PROOF_ON_TX = 100,
    WATER_PROOF_ON_RX,
    WATER_PROOF_OFF_TX,
    WATER_PROOF_OFF_RX,
};

/* sc */

struct sc_threshold {

    int rawdata_min;
    int rawdata_max;
    int scap_cb_min;
    int scap_cb_max;
    int noise_max;
    int scap_rawdata_min;
    int scap_rawdata_max;
    int short_min;
    int mcap_cmb_min;
    int mcap_cmb_max;
};


struct fts_test_node {
    int channel_num;
    int tx_num;
    int rx_num;
    int node_num;
    int key_num;
};

struct fts_test {
    struct fts_ts_data *ts_data;
    struct fts_test_node node;
    uint8_t fw_ver;
    struct sc_threshold thr;

    int result;

};



/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/


extern struct fts_test *fts_ftest;

void sys_delay(int ms);
int fts_abs(int value);
void print_buffer(int *buffer, int length, int line_num);
int fts_test_read_reg(uint8_t addr, uint8_t *val);
int fts_test_write_reg(uint8_t addr, uint8_t val);
int fts_test_read(uint8_t addr, uint8_t *readbuf, int readlen);
int fts_test_write(uint8_t addr, uint8_t *writebuf, int writelen);
int enter_work_mode(void);
int enter_factory_mode(void);



int start_scan(void);
bool compare_array(int *data, int min, int max, bool key);
void show_data(int *data);
void show_noise_data(int *data, int retries);


void *fts_malloc(size_t size);
void fts_free_proc(void *p);

int fts_test_entry(void);

#define fts_malloc_r(p, size) do {\
    if (NULL == p) {\
        p = fts_malloc(size);\
        if (NULL == p) {\
            return -1;\
        }\
    }\
} while(0)

#define fts_free(p) do {\
    if (p) {\
        fts_free_proc(p);\
        p = NULL;\
    }\
} while(0)

#define CSV_SUPPORT             1
#define TXT_SUPPORT             1


#endif
