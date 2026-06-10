/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/************************************************************************
*
* File Name: focaltech_test.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-01
*
* Modify:
*
* Abstract: create char device and proc node for  the comm between APK and TP
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_test.h"


/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define TX_NUM 14
#define RX_NUM 6
#define MAX_NODE_NUM                 (TX_NUM * RX_NUM)
#define MAX_CHANNEL_NUM              (TX_NUM + RX_NUM)

#define NODE_VALID                                       {1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,\
                                                          1,1,1,1,1,1}

#define CHANNEL_VALID                                    {1,1,1,1,1,1,\
                                                          1,1,1,1,1,1,1,1,1,1,1,1,1,1}

#define RAW_MIN                                          {2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160,\
                                                          2160,2160,2160,2160,2160,2160}

#define RAW_MAX                                          {8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000,\
                                                          8000,8000,8000,8000,8000,8000}

#define SCAP_CB_MIN                                      {0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,}

#define SCAP_CB_MAX                                      {4095,4095,4095,4095,4095,4095,\
                                                          4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095,4095}

#define MCAP_CMB_MIN                                      {0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0,\
                                                          0,0,0,0,0,0}

#define MCAP_CMB_MAX                                      {127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127,\
                                                          127,127,127,127,127,127}

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
static struct fts_test _fts_ftest = {
    .thr.rawdata_min = 2160,
    .thr.rawdata_max = 8000,
    .thr.scap_cb_min = 0,
    .thr.scap_cb_max = 4095,
    .thr.noise_max = 200,
    .thr.scap_rawdata_min = 3000,
    .thr.scap_rawdata_max = 15000,
    .thr.short_min = 1200,
    .thr.mcap_cmb_min = 0,
    .thr.mcap_cmb_max = 127,
};

int node_valid[] = NODE_VALID;
int channel_valid[] = CHANNEL_VALID;
struct fts_test *fts_ftest = &_fts_ftest;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

/*****************************************************************************
* functions body
*****************************************************************************/
void sys_delay(int msec)
{
    fts_msleep(msec);
}

int fts_abs(int value)
{
    if (value < 0)
        value = 0 - value;

    return value;
}

void *fts_malloc(size_t size)
{
    return malloc(size);
}

void fts_free_proc(void *p)
{
    free(p);
}

#define MAX_LOG_BUFFER 2048
void print_buffer(int *buffer, int length, int line_num)
{
    int i = 0;
    int j = 0;
    char tmpbuf[MAX_LOG_BUFFER] = {0};
    int tmplen = MAX_LOG_BUFFER;
    int cnt = 0;
    struct fts_test *tdata = fts_ftest;

    if (tdata && tdata->ts_data ) {
        return;
    }

    if ((NULL == buffer) || (length <= 0)) {
        FTS_DEBUG("buffer/length(%d) fail", length);
        return;
    }

    for (i = 0; i < length; i = i + line_num) {
        cnt = 0;
        for (j = 0; j < line_num; j++) {
            cnt += snprintf(tmpbuf + cnt, tmplen - cnt, "%5d ", buffer[i + j]);
            if ((cnt >= tmplen) || ((i + j + 1) >= length))
                break;
        }
        FTS_DEBUG("%s", tmpbuf);
    }

}

/********************************************************************
 * test read/write interface
 *******************************************************************/
static int fts_test_bus_read(
    uint8_t cmd, uint8_t *data, int datalen)
{
    int ret = 0;

    ret = fts_read(cmd, data, datalen);
    if (ret < 0)
        return ret;
    else
        return 0;
}

static int fts_test_bus_write(uint8_t *writebuf, int writelen)
{
    int ret = 0;

    ret = fts_write(writebuf[0], &writebuf[1], writelen);
    if (ret < 0)
        return ret;
    else
        return 0;
}

int fts_test_read_reg(uint8_t addr, uint8_t *val)
{
    return fts_test_bus_read(addr, val, 1);
}

int fts_test_write_reg(uint8_t addr, uint8_t val)
{
    int ret;
    uint8_t cmd[2] = {0};

    cmd[0] = addr;
    cmd[1] = val;
    ret = fts_test_bus_write(cmd, 1);

    return ret;
}

int fts_test_read(uint8_t addr, uint8_t *readbuf, int readlen)
{
    int ret = 0;
    int i = 0;
    int packet_length = 0;
    int packet_num = 0;
    int packet_remainder = 0;
    int offset = 0;
    int byte_num = readlen;

    packet_num = byte_num / BYTES_PER_TIME;
    packet_remainder = byte_num % BYTES_PER_TIME;
    if (packet_remainder)
        packet_num++;

    if (byte_num < BYTES_PER_TIME) {
        packet_length = byte_num;
    } else {
        packet_length = BYTES_PER_TIME;
    }
    /* FTS_DEBUG("packet num:%d, remainder:%d", packet_num, packet_remainder); */

    ret = fts_test_bus_read(addr, &readbuf[offset], packet_length);
    if (ret < 0) {
        FTS_ERROR("read buffer fail");
        return ret;
    }
    for (i = 1; i < packet_num; i++) {
        offset += packet_length;
        if ((i == (packet_num - 1)) && packet_remainder) {
            packet_length = packet_remainder;
        }


        ret = fts_test_bus_read(addr, &readbuf[offset],
                                packet_length);

        if (ret < 0) {
            FTS_ERROR("read buffer fail");
            return ret;
        }
    }

    return 0;
}

int fts_test_write(uint8_t addr, uint8_t *writebuf, int writelen)
{
    int ret = 0;
    int i = 0;
    uint8_t data[128] = {0};
    int packet_length = 0;
    int packet_num = 0;
    int packet_remainder = 0;
    int offset = 0;
    int byte_num = writelen;

    packet_num = byte_num / BYTES_PER_TIME;
    packet_remainder = byte_num % BYTES_PER_TIME;
    if (packet_remainder)
        packet_num++;

    if (byte_num < BYTES_PER_TIME) {
        packet_length = byte_num;
    } else {
        packet_length = BYTES_PER_TIME;
    }
    /* FTS_DEBUG("packet num:%d, remainder:%d", packet_num, packet_remainder); */

    data[0] = addr;
    for (i = 0; i < packet_num; i++) {
        if (i != 0) {
            data[0] = addr + 1;
        }
        if ((i == (packet_num - 1)) && packet_remainder) {
            packet_length = packet_remainder;
        }
        memcpy(&data[1], &writebuf[offset], packet_length);

        ret = fts_test_bus_write(data, packet_length + 1);
        if (ret < 0) {
            FTS_ERROR("write buffer fail");

            return ret;
        }
        offset += packet_length;
    }

    return 0;
}

/********************************************************************
 * test global function enter work/factory mode
 *******************************************************************/
int enter_work_mode(void)
{
    int ret = 0;
    uint8_t mode = 0;
    int i = 0;
    int j = 0;



    ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);
    if ((ret >= 0) && (0x00 == mode))
        return 0;

    for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
        ret = fts_test_write_reg(DEVIDE_MODE_ADDR, 0x00);
        if (ret >= 0) {
            sys_delay(FACTORY_TEST_DELAY);
            for (j = 0; j < 20; j++) {
                ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);
                if ((ret >= 0) && (0x00 == mode)) {
                    FTS_INFO("enter work mode success");
                    return 0;
                } else
                    sys_delay(FACTORY_TEST_DELAY);
            }
        }

        sys_delay(50);
    }

    if (i >= ENTER_WORK_FACTORY_RETRIES) {
        FTS_ERROR("Enter work mode fail");
        return -1;
    }


    return 0;
}

int enter_factory_mode(void)
{
    int ret = 0;
    uint8_t mode = 0;
    int i = 0;
    int j = 0;

    ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);
    if ((ret >= 0) && (0x40 == mode))
        return 0;

    for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
        ret = fts_test_write_reg(DEVIDE_MODE_ADDR, 0x40);
        if (ret >= 0) {
            sys_delay(FACTORY_TEST_DELAY);
            for (j = 0; j < 20; j++) {
                ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);
                if ((ret >= 0) && (0x40 == mode)) {
                    FTS_INFO("enter factory mode success");
                    sys_delay(200);
                    return 0;
                } else
                    sys_delay(FACTORY_TEST_DELAY);
            }
        }

        sys_delay(50);
    }

    if (i >= ENTER_WORK_FACTORY_RETRIES) {
        FTS_ERROR("Enter factory mode fail");
        return -1;
    }

    return 0;
}


int get_cb_mc_sc(uint8_t wp, int byte_num, int *cb_buf)
{
    int ret = 0;
    int i = 0;
    uint8_t cb[MAX_CHANNEL_NUM * 2] = {0};

    /* start scanning */
    ret = start_scan();
    if (ret < 0) {
        FTS_ERROR("scan fail\n");
        return -1;
    }

    /* 1:waterproof 0:non-waterproof */
    ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, wp);
    if (ret < 0) {
        FTS_ERROR("set mc_sc mode fail\n");
        return ret;
    }

    ret = fts_test_write_reg(FACTORY_REG_MC_SC_CB_ADDR_OFF, 0x00);
    if (ret < 0) {
        FTS_ERROR("set mc_sc mode fail\n");
        return ret;
    }


    /* read cb */
    ret = fts_test_read(FACTORY_REG_MC_SC_CB_ADDR, cb, byte_num);
    if (ret < 0) {
        FTS_ERROR("read cb data fail\n");
        return -1;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        cb_buf[i >> 1] = (int)(((int)(cb[i]) << 8) + cb[i + 1]);
    }

    return 0;
}


/*
 * start_scan - start to scan a frame
 */
int start_scan(void)
{
    int ret = 0;
    uint8_t addr = 0;
    uint8_t val = 0;
    uint8_t finish_val = 0;
    int times = 0;
    struct fts_test *tdata = fts_ftest;

    if (NULL == tdata ) {
        FTS_ERROR("test/func is null\n");
        return -2;
    }

    /* sc ic */
    addr = 0x00;
    val = 0xC0;
    finish_val = 0x40;

    /* write register to start scan */
    ret = fts_test_write_reg(addr, val);
    if (ret < 0) {
        FTS_ERROR("write start scan mode fail\n");
        return ret;
    }

    /* Wait for the scan to complete */
    while (times++ < FACTORY_TEST_RETRY) {
        sys_delay(FACTORY_TEST_DELAY);

        ret = fts_test_read_reg(addr, &val);
        if ((ret >= 0) && (val == finish_val)) {
            break;
        } else
            FTS_DEBUG("reg%x=%x,retry:%d", addr, val, times);
    }

    if (times >= FACTORY_TEST_RETRY) {
        FTS_ERROR("scan timeout\n");
        return -1;
    }

    return 0;
}


bool compare_array(int *data, int min, int max, bool key)
{
    int i = 0;
    bool result = true;
    struct fts_test *tdata = fts_ftest;
    int rx = tdata->node.rx_num;
    int node_num = tdata->node.node_num;

    if (!key) {
        node_num -= tdata->node.key_num;
    }
    for (i = 0; i < node_num; i++) {

        if ((data[i] < min) || (data[i] > max)) {
            FTS_ERROR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
                      i / rx + 1, i % rx + 1, data[i], min, max);
            result = false;
        }
    }

    return result;
}

/*
 * show_data - show and save test data to testresult.txt
 */
void show_data(int *data)
{
    struct fts_test *tdata = fts_ftest;
    int node_num = tdata->node.node_num;
    int rx_num = tdata->node.rx_num;

    print_buffer(data, node_num, rx_num);


}

static int get_tx_rx_num(uint8_t tx_rx_reg, uint8_t *ch_num, uint8_t ch_num_max)
{
    int ret = 0;
    int i = 0;

    for (i = 0; i < 3; i++) {
        ret = fts_test_read_reg(tx_rx_reg, ch_num);
        if ((ret < 0) || (*ch_num > ch_num_max)) {
            sys_delay(50);
        } else
            break;
    }

    if (i >= 3) {
        FTS_ERROR("get channel num fail");
        return -1;
    }

    return 0;
}
static int get_channel_num(struct fts_test *tdata)
{
    int ret = 0;
    uint8_t tx_num = 0;
    uint8_t rx_num = 0;

    /* node structure */
    ret = get_tx_rx_num(FACTORY_REG_CHX_NUM, &tx_num, TX_NUM_MAX);
    if (ret < 0) {
        FTS_ERROR("get tx_num fail");
        return ret;
    }

    ret = get_tx_rx_num(FACTORY_REG_CHY_NUM, &rx_num, RX_NUM_MAX);
    if (ret < 0) {
        FTS_ERROR("get rx_num fail");
        return ret;
    }
    FTS_INFO("====tx_num = %d,rx_num = %d", tx_num, rx_num);
    tdata->node.tx_num = tx_num;
    tdata->node.rx_num = rx_num;
    tdata->node.channel_num = tx_num + rx_num;
    tdata->node.node_num = tx_num * rx_num;

    /* key */
    tdata->node.key_num = 0;

    if (tdata->node.tx_num > TX_NUM_MAX) {
        FTS_ERROR("tx num(%d) fail", tdata->node.tx_num);
        return -1;
    }

    if (tdata->node.rx_num > RX_NUM_MAX) {
        FTS_ERROR("rx num(%d) fail", tdata->node.rx_num);
        return -1;
    }

    FTS_INFO("node_num:%d, tx:%d, rx:%d, key:%d",
             tdata->node.node_num, tdata->node.tx_num,
             tdata->node.rx_num, tdata->node.key_num);
    return 0;
}

static int fts_test_init_basicinfo(struct fts_test *tdata)
{
    int ret = 0;
    uint8_t val = 0;

    if ((NULL == tdata) ) {
        FTS_ERROR("tdata/func is NULL\n");
        return -2;
    }

    fts_test_read_reg(REG_FW_VERSION, &val);
    tdata->fw_ver = val;

    /* enter factory mode */
    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fail\n");
        return ret;
    }

    /* enter into factory mode and read tx/rx num */
    ret = get_channel_num(tdata);
    if (ret < 0) {
        FTS_ERROR("get channel number fail\n");
        return ret;
    }

    return ret;
}

static int fts_test_main_init(void)
{
    int ret = 0;
    struct fts_test *tdata = fts_ftest;

    /* get basic information: tx/rx num ... */
    ret = fts_test_init_basicinfo(tdata);
    if (ret < 0) {
        FTS_ERROR("test init basicinfo fail");
        return ret;
    }

    return ret;
}

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static int ft3169_scap_cb_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    bool tmp_result1 = false;
    int ScapCBMin[MAX_CHANNEL_NUM] = SCAP_CB_MIN;
    int ScapCBMax[MAX_CHANNEL_NUM] = SCAP_CB_MAX;
    uint8_t sc_mode = 0;
    uint8_t reg45 = 0;
    uint8_t reg49 = 0;
    int byte_num = 0;
    int scb_tmp[MAX_CHANNEL_NUM] = {0};
    int i = 0;

    FTS_INFO("\n============ Test Item: CB Test\n");

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_MC_SC_MODE, &sc_mode);
    if (ret < 0) {
        FTS_ERROR("read sc_mode fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_MC_SC_CB_ADDR_OFF, &reg45);
    if (ret < 0) {
        FTS_ERROR("read sc_mode fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_MC_SC_CB_H_ADDR_OFF, &reg49);
    if (ret < 0) {
        FTS_ERROR("read sc_mode fail,ret=%d\n", ret);
        goto test_err;
    }

    byte_num = tdata->node.channel_num * 2;
    /* read cbdata */
    /* 1:waterproof 0:non-waterproof */
    ret = get_cb_mc_sc(WATER_PROOF_ON, byte_num, scb_tmp);
    if (ret < 0) {
        FTS_ERROR("read sc_cb fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* compare */
    //tmp_result = compare_array(scb_tmp, thr->scap_cb_min, thr->scap_cb_max, key_check);
    tmp_result = true;
    for (i = 0; i < tdata->node.channel_num; i++) {
        if (0 == channel_valid[i])
            continue;

        if ((scb_tmp[i] < ScapCBMin[i]) || (scb_tmp[i] > ScapCBMax[i])) {
            FTS_ERROR("test fail,ch:%d, scap_cb=%5d,range=(%5d,%5d)\n",
                      i, scb_tmp[i], ScapCBMin[i], ScapCBMax[i]);
            tmp_result = false;
        }
    }
    FTS_INFO("scap_cb in waterproof on mode :");
    print_buffer(scb_tmp, tdata->node.rx_num, tdata->node.rx_num);
    print_buffer((scb_tmp + tdata->node.rx_num), tdata->node.tx_num, tdata->node.tx_num);


    /* 1:waterproof 0:non-waterproof */
    ret = get_cb_mc_sc(WATER_PROOF_OFF, byte_num, scb_tmp);
    if (ret < 0) {
        FTS_ERROR("read sc_cb fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* compare */
    //tmp_result1 = compare_array(scb_tmp, thr->scap_cb_min, thr->scap_cb_max, key_check);
    tmp_result1 = true;
    for (i = 0; i < tdata->node.channel_num; i++) {
        if (0 == channel_valid[i])
            continue;

        if ((scb_tmp[i] < ScapCBMin[i]) || (scb_tmp[i] > ScapCBMax[i])) {
            FTS_ERROR("test fail,ch:%d,scap_cb=%5d,range=(%5d,%5d)\n",
                      i, scb_tmp[i], ScapCBMin[i], ScapCBMax[i]);
            tmp_result1 = false;
        }
    }

    /* save */
    FTS_INFO("scap_cb in waterproof off mode :");
    print_buffer(scb_tmp, tdata->node.rx_num, tdata->node.rx_num);
    print_buffer((scb_tmp + tdata->node.rx_num), tdata->node.tx_num, tdata->node.tx_num);


restore_reg:
    ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, sc_mode);
    if (ret < 0) {
        FTS_ERROR("reset 0x44 value fail,ret = %d\n", ret);
        goto test_err;
    }

    ret = fts_test_write_reg(FACTORY_REG_MC_SC_CB_ADDR_OFF, reg45);
    if (ret < 0) {
        FTS_ERROR("reset 0x45 value fail,ret = %d\n", ret);
        goto test_err;
    }

    ret = fts_test_write_reg(FACTORY_REG_MC_SC_CB_H_ADDR_OFF, reg49);
    if (ret < 0) {
        FTS_ERROR("reset 0x49 value fail,ret = %d\n", ret);
        goto test_err;
    }

test_err:
    if ((tmp_result) && (tmp_result1)) {
        *test_result = true;
        FTS_INFO("------ CB Test PASS\n");
    } else {
        *test_result = false;
        FTS_INFO("------ CB Test NG\n");
    }

    return ret;
}

static int ft3169_scap_rawdata_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    bool tmp_result1 = false;
    uint8_t tmp_data[200] = {0};
    int scap_rawdata[MAX_CHANNEL_NUM] = {0};
    uint8_t rawdata_addr = 0;
    int byte_num = 0;
    int i = 0;
    uint8_t reg01 = 0;
    struct sc_threshold *thr = &tdata->thr;

    FTS_INFO("\n============ Test Item: Scap RawData Test\n");

    if (!thr->scap_rawdata_min || !thr->scap_rawdata_max) {
        FTS_ERROR("rawdata_min/max test_result is null\n");
        ret = -1;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    /* start scanning */
    ret = start_scan();
    if (ret < 0) {
        FTS_ERROR("scan fail\n");
        goto restore_reg;
    }

    ret = fts_test_read_reg(0x01, &reg01);
    if (ret < 0) {
        FTS_ERROR("read 0x01 fail,ret=%d\n", ret);
        goto test_err;
    }

    /* read rawdata */
    rawdata_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;

    byte_num = tdata->node.channel_num * 2;
    ret = fts_test_write_reg(0x01, 0xAC);
    if (ret < 0) {
        FTS_ERROR("set 0x01 fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read(rawdata_addr, tmp_data, byte_num);
    if (ret < 0) {
        FTS_ERROR("read 0x01 fail,ret=%d\n", ret);
        goto test_err;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        scap_rawdata[i >> 1] = (int)(short)((tmp_data[i] << 8) + tmp_data[i + 1]);
    }

    /* compare */
    tmp_result = true;
    for (i = 0; i < tdata->node.channel_num; i++) {
        if (0 == channel_valid[i])
            continue;

        if ((scap_rawdata[i] < thr->scap_rawdata_min) || (scap_rawdata[i] > thr->scap_rawdata_max)) {
            FTS_ERROR("test fail,ch:%d,scap_rawdata=%5d,range=(%5d,%5d)\n",
                      i, scap_rawdata[i], thr->scap_rawdata_min, thr->scap_rawdata_max);
            tmp_result = false;
        }
    }

    /* show data */
    FTS_INFO("scap_rawdata in waterproof on mode:");
    //show_data(scap_rawdata);
    print_buffer(scap_rawdata, tdata->node.rx_num, tdata->node.rx_num);
    print_buffer((scap_rawdata + tdata->node.rx_num), tdata->node.tx_num, tdata->node.tx_num);

    ret = fts_test_write_reg(0x01, 0xAB);
    if (ret < 0) {
        FTS_ERROR("set 0x01 fail,ret=%d\n", ret);
        goto restore_reg;
    }
    ret = fts_test_read(rawdata_addr, tmp_data, byte_num);
    if (ret < 0) {
        FTS_ERROR("read 0x01 fail,ret=%d\n", ret);
        goto test_err;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        scap_rawdata[i >> 1] = (int)(short)((tmp_data[i] << 8) + tmp_data[i + 1]);
    }


    tmp_result1 = true;
    for (i = 0; i < tdata->node.channel_num; i++) {
        if (0 == channel_valid[i])
            continue;

        if ((scap_rawdata[i] < thr->scap_rawdata_min) || (scap_rawdata[i] > thr->scap_rawdata_max)) {
            FTS_ERROR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
                      i / tdata->node.rx_num + 1, i % tdata->node.rx_num + 1, scap_rawdata[i], thr->scap_rawdata_min, thr->scap_rawdata_max);
            tmp_result1 = false;
        }
    }

    /* show data */
    FTS_INFO("scap_rawdata in waterproof off mode:");
    //show_data(scap_rawdata);
    print_buffer(scap_rawdata, tdata->node.rx_num, tdata->node.rx_num);
    print_buffer((scap_rawdata + tdata->node.rx_num), tdata->node.tx_num, tdata->node.tx_num);

restore_reg:
    ret = fts_test_write_reg(0x01, reg01);
    if (ret < 0) {
        FTS_ERROR("set 0x01 fail,ret=%d\n", ret);
    }

test_err:
    if ((tmp_result) && (tmp_result1)) {
        *test_result = true;
        FTS_INFO("------ Scap RawData Test PASS\n");
    } else {
        *test_result = false;
        FTS_INFO("------ Scap RawData Test NG\n");
    }


    return ret;
}

static int ft3169_mcap_cmb_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    uint8_t tmp_data[200] = {0};
    int McapCMBMin[MAX_NODE_NUM] = MCAP_CMB_MIN;
    int McapCMBMax[MAX_NODE_NUM] = MCAP_CMB_MAX;
    int mcap_cmb_data[MAX_NODE_NUM] = {0};
    uint8_t mcapcmbdata_addr = 0;
    int byte_num = 0;
    int i = 0;

    FTS_INFO("\n============ Test Item: MCAP CMB Test\n");

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_write_reg(0x24, 0x00);
    if (ret < 0) {
        FTS_ERROR("write 0x24 fail\n");
        goto test_err;
    }

    ret = fts_test_write_reg(FACTORY_REG_MCB_OFF, 0x00);
    if (ret < 0) {
        FTS_ERROR("write 0x26 mode fail\n");
        goto test_err;
    }


    /* read rawdata */
    mcapcmbdata_addr = FACTORY_REG_MCB_ADDR;

    byte_num = tdata->node.node_num;

    ret = fts_test_read(mcapcmbdata_addr, tmp_data, byte_num);
    if (ret < 0) {
        FTS_ERROR("read mcap_cmb_data fail\n");
        goto test_err;
    }

    for (i = 0; i < tdata->node.node_num; i++) {
        mcap_cmb_data[i] = (int)tmp_data[i];
    }

    /* compare */
    tmp_result = true;
    for ( i = 0; i < tdata->node.node_num; i++) {
        if (0 == node_valid[i])
            continue;

        if ((mcap_cmb_data[i] < McapCMBMin[i]) || (mcap_cmb_data[i] > McapCMBMax[i])) {
            FTS_ERROR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n", i / tdata->node.rx_num + 1, i % tdata->node.rx_num + 1,
                      mcap_cmb_data[i], McapCMBMin[i], McapCMBMax[i]);
            tmp_result = false;
        }
    }

    /* show data */
    FTS_INFO("mcap cmb data:");
    show_data(mcap_cmb_data);


test_err:
    if (tmp_result) {
        *test_result = true;
        FTS_INFO("------ MCAP CMB Test PASS\n");
    } else {
        *test_result = false;
        FTS_INFO("------ MCAP CMB Test NG\n");
    }

    return ret;
}



static int ft3169_rawdata_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    uint16_t RawMin[MAX_NODE_NUM] = RAW_MIN;
    uint16_t RawMax[MAX_NODE_NUM] = RAW_MAX;
    int rawdata[MAX_NODE_NUM] = {0};
    uint8_t tmp_data[100] = {0};
    uint16_t *min;
    uint16_t *max;
    int i = 0;
    uint8_t addr = 0;
    uint8_t rawdata_addr = 0;
    int byte_num = 0;
    uint8_t val = 0;
    uint8_t fre = 0;

    FTS_INFO("\n============ Test Item: RawData Test\n");

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }
    ret = fts_test_read_reg(0x0A, &fre);
    if (ret < 0) {
        FTS_ERROR("read 0x0A fail\n");
        goto test_err;
    }

    ret = fts_test_write_reg(0x0A, 0x81);
    if (ret < 0) {
        FTS_ERROR("enter mode0 fail\n");
        goto restore_reg;
    }

    ret = fts_test_write_reg(0x06, 0x00);
    if (ret < 0) {
        FTS_ERROR("enter mode0 fail\n");
        goto restore_reg;
    }

    for (i = 0; i < 3; i++) {
        /* start scanning */
        ret = start_scan();
        if (ret < 0) {
            FTS_ERROR("scan fail\n");
            goto restore_reg;
        }
    }

    /* read rawdata */
    val = 0xAA;
    addr = FACTORY_REG_LINE_ADDR;
    rawdata_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;


    byte_num = tdata->node.node_num * 2;

    /* set line addr or rawdata start addr */
    ret = fts_test_write_reg(addr, val);
    if (ret < 0) {
        FTS_ERROR("wirte line/start addr fail\n");
        goto restore_reg;
    }

    ret = fts_test_read(rawdata_addr, tmp_data, byte_num);
    if (ret < 0) {
        FTS_ERROR("read 0x01 fail,ret=%d\n", ret);
        goto test_err;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        rawdata[i >> 1] = (uint16_t)(short)((tmp_data[i] << 8) + tmp_data[i + 1]);
    }

    min = RawMin;
    max = RawMax;
    tmp_result = true;
    /* compare */
    for ( i = 0; i < tdata->node.node_num; i++) {
        if (0 == node_valid[i])
            continue;

        if ((rawdata[i] < min[i]) || (rawdata[i] > max[i])) {
            FTS_ERROR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
                      i / tdata->node.rx_num + 1, i % tdata->node.rx_num + 1, rawdata[i], min[i], max[i]);
            tmp_result = false;
        }
    }

    /* show data */
    FTS_INFO("rawdata:");
    show_data(rawdata);

restore_reg:
    ret = fts_test_write_reg(0x0A, fre);
    if (ret < 0) {
        FTS_ERROR("restore fre mode reg fail\n");
    }

    ret = fts_test_write_reg(0x06, 0x00);
    if (ret < 0) {
        FTS_ERROR("restore rawdata mode reg fail\n");
    }

test_err:
    if (tmp_result) {
        *test_result = true;
        FTS_INFO("------ RawData Test PASS\n");
    } else {
        *test_result = false;
        FTS_INFO("------ RawData Test NG\n");
    }


    return ret;
}

static int ft3169_noise_test(struct fts_test *tdata, bool *test_result)
{
    int i = 0;
    //int j = 0;
    int ret = 0;
    uint8_t val = 0;
    bool tmp_result = false;
    uint8_t tmp_data[200] = {0};
    uint8_t addr = 0;
    uint8_t noise_addr = 0;
    int byte_num = 0;
    int noise_data[MAX_NODE_NUM] = {0};
    uint8_t reg06 = 0;
    uint8_t reg1a = 0;
    uint8_t fre = 0;
    uint8_t reg1c = 0;
    struct sc_threshold *thr = &tdata->thr;


    if (!thr->noise_max) {
        FTS_ERROR("noise_max test_result is null\n");
        ret = -1;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(0x06, &reg06);
    if (ret < 0) {
        FTS_ERROR("read 0x0A fail\n");
        goto test_err;
    }

    ret = fts_test_read_reg(0x0A, &fre);
    if (ret < 0) {
        FTS_ERROR("read 0x0A fail\n");
        goto test_err;
    }
    ret = fts_test_read_reg(0x1a, &reg1a);
    if (ret < 0) {
        FTS_ERROR("read 0x1A fail\n");
        goto test_err;
    }


    ret = fts_test_write_reg(0x06, 0x01);
    if (ret < 0) {
        FTS_ERROR("read 0x06 fail\n");
        goto test_err;
    }

    ret = fts_test_write_reg(0x1a, 0x01);
    if (ret < 0) {
        FTS_ERROR("read 0x1a fail\n");
        goto restore_reg06;
    }

    ret = fts_test_write_reg(0x0a, 0x00);
    if (ret < 0) {
        FTS_ERROR("read 0x0a fail\n");
        goto restore_reg1a;
    }

    ret = fts_test_write_reg(0x1d, 0x00);
    if (ret < 0) {
        FTS_ERROR("read 0x1d fail\n");
        goto restore_reg0a;
    }

    ret = fts_test_write_reg(0x1c, 0xc8);
    if (ret < 0) {
        FTS_ERROR("read 0x1c fail\n");
        goto restore_reg0a;
    }

    ret = fts_test_write_reg(0x1b, 0x01);
    if (ret < 0) {
        FTS_ERROR("read 0x1b fail\n");
        goto restore_reg1c;
    }

    /* start scanning */
    ret = start_scan();
    if (ret < 0) {
        FTS_ERROR("scan fail\n");
        goto restore_reg;
    }
    sys_delay(10);
    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* read rawdata */
    val = 0xAA;
    addr = FACTORY_REG_LINE_ADDR;
    noise_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;

    byte_num = tdata->node.node_num * 2;

    /* set line addr or rawdata start addr */
    ret = fts_test_write_reg(addr, val);
    if (ret < 0) {
        FTS_ERROR("wirte line/start addr fail\n");
        goto restore_reg;
    }
    sys_delay(10);

    ret = fts_test_read(noise_addr, tmp_data, byte_num);
    if (ret < 0) {
        FTS_ERROR("read rawdata fail\n");
        goto restore_reg;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        noise_data[i >> 1] = (int)(short)((tmp_data[i] << 8) + tmp_data[i + 1]);
    }


    /* compare */
    tmp_result = true;
    for (i = 0; i < tdata->node.node_num; i++) {
        if (0 == node_valid[i])
            continue;

        if ((noise_data[i] < 0) || (noise_data[i] > thr->noise_max)) {
            FTS_ERROR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
                      i / tdata->node.rx_num + 1, i % tdata->node.rx_num + 1, noise_data[i], 0, thr->noise_max);
            tmp_result = false;
        }
    }

    /* show data */
    FTS_INFO("noise data:");
    show_data(noise_data);

restore_reg:
    ret = fts_test_write_reg(0x1b, 0x00);
    if (ret < 0) {
        FTS_ERROR("restore 0x1b reg fail\n");
    }

restore_reg1c:
    ret = fts_test_write_reg(0x1c, reg1c);
    if (ret < 0) {
        FTS_ERROR("restore 0x1c reg fail\n");
    }

restore_reg0a:
    ret = fts_test_write_reg(0x0a, fre);
    if (ret < 0) {
        FTS_ERROR("restore 0x0a reg fail\n");
    }
restore_reg1a:
    ret = fts_test_write_reg(0x1a, reg1a);
    if (ret < 0) {
        FTS_ERROR("restore 0x1a reg fail\n");
    }

restore_reg06:
    ret = fts_test_write_reg(0x06, reg06);
    if (ret < 0) {
        FTS_ERROR("restore 0x06 reg fail\n");
    }

test_err:
    if (tmp_result) {
        *test_result = true;
        FTS_INFO("------ Noise Test PASS\n");
    } else {
        *test_result = false;
        FTS_INFO("------ Noise Test NG\n");
    }


    return ret;
}


static int ft3169_weak_short_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    uint8_t val = 0;
    bool tmp_result = false;
    uint8_t tmp_data[200] = {0};
    int shortdata[MAX_CHANNEL_NUM] = {0};
    uint8_t shortdata_addr = 0;
    int byte_num = 0;
    uint8_t old_mode = 0;
    int i = 0;
    struct sc_threshold *thr = &tdata->thr;

    FTS_INFO("\n============ Test Item: Weak Short Test\n");

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(0xC0, &old_mode);
    if (ret < 0) {
        FTS_ERROR("write weak short test mode fail\n");
        goto test_err;
    }


    for (i = 0; i < FACTORY_TEST_RETRY; i++) {
        sys_delay(FACTORY_TEST_DELAY);
        ret = fts_test_read_reg(0xC3, &val);
        if (ret < 0) {
            FTS_ERROR("read weak short test idle fail\n");
            goto restore_reg;
        }
        if (val == 0x03) {
            break;
        }

    }

    if (i >= FACTORY_TEST_RETRY) {
        FTS_ERROR(" weak short test idle time out\n");
        goto restore_reg;
    }


    ret = fts_test_write_reg(0xC0, 0x01);
    if (ret < 0) {
        FTS_ERROR("start weak short test mode fail\n");
        goto restore_reg;
    }
    sys_delay(50);


    /* start scanning */
    for (i = 0; i < FACTORY_TEST_RETRY; i++) {
        ret = fts_test_read_reg(0xC3, &val);
        if (ret < 0) {
            FTS_ERROR("read weak short test idle fail\n");
            goto restore_reg;
        }
        if (val == 0xAA) {
            break;
        }

        sys_delay(FACTORY_TEST_DELAY);

    }

    if (i >= FACTORY_TEST_RETRY) {
        FTS_ERROR(" weak short test scan time out\n");
        goto restore_reg;
    }

    ret = fts_test_write_reg(0xC3, 0x03);
    if (ret < 0) {
        FTS_ERROR("write 0xC3 fail\n");
        goto restore_reg;
    }

    /* read rawdata */
    shortdata_addr = 0xC4;

    byte_num = tdata->node.channel_num * 2;

    ret = fts_test_read(shortdata_addr, tmp_data, byte_num);
    if (ret < 0) {
        FTS_ERROR("read 0x01 fail,ret=%d\n", ret);
        goto test_err;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        shortdata[i >> 1] = (int)(short)((tmp_data[i] << 8) + tmp_data[i + 1]);
    }


    /* compare */
    tmp_result = true;
    for (i = 0; i < tdata->node.channel_num; i++) {
        if (0 == channel_valid[i])
            continue;

        if ((shortdata[i] < thr->short_min) || (shortdata[i] > 65535)) {
            FTS_ERROR("test fail,ch:%d,shortdata=%5d,range=(%5d,%5d)\n",
                      i, shortdata[i], thr->short_min, 65535);
            tmp_result = false;
        }
    }

    /* show data */
    FTS_INFO("weak short data:");
    print_buffer(shortdata, tdata->node.rx_num, tdata->node.rx_num);
    print_buffer((shortdata + tdata->node.rx_num), tdata->node.tx_num, tdata->node.tx_num);


restore_reg:
    ret = fts_test_write_reg(0xC0, old_mode);
    if (ret < 0) {
        FTS_ERROR("restore short test mode fail\n");
    }

test_err:
    if (tmp_result) {
        *test_result = true;
        FTS_INFO("------ Weak Short Test PASS\n");
    } else {
        *test_result = false;
        FTS_INFO("------ Weak Short Test NG\n");
    }

    return ret;
}


int start_test_ft3169(void)
{
    int ret = 0;
    struct fts_test *tdata = fts_ftest;
    bool temp_result = false;
    bool test_result = true;


    /* rawdata test */
    if (RAWDATA_TEST_EN) {
        ret = ft3169_rawdata_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    /* noise test */
    if (NOISE_TEST_EN) {
        ret = ft3169_noise_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }
    /* scap_cb test */
    if (SCAP_CB_TEST_EN) {
        ret = ft3169_scap_cb_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    /* mcap cmd test */
    if (MCAP_CMB_TEST_EN) {
        ret = ft3169_mcap_cmb_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    /* scap_rawdata test */
    if (SCAP_RAWDATA_TEST_EN) {
        ret = ft3169_scap_rawdata_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    /* short test */
    if (WEAKSHORT_TEST_EN) {
        ret = ft3169_weak_short_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    /* delta cb test */

    return test_result;
}

/*
 * fts_test_entry - test main entry
 *
 * warning - need disable irq & esdcheck before call this function
 *
 */
int fts_test_entry(void)
{
    int ret = 0;

    /* test initialize */
    ret = fts_test_main_init();
    if (ret < 0) {
        FTS_ERROR("fts_test_main_init fail");
        goto test_err;
    }

    /* Start testing according to the test configuration */
    if (true == start_test_ft3169()) {
        FTS_INFO("\n\n=======Tp test pass.\n");
        fts_ftest->result = true;
    } else {
        FTS_INFO("\n\n=======Tp test failure.\n");
        fts_ftest->result = false;

    }

    ret = 0;
test_err:

    enter_work_mode();
    return ret;
}

