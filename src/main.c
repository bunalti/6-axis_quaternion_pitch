/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <irq.h>
#include <logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <device.h>
#include <devicetree.h>
#include <zephyr/types.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include <soc.h>
#include <settings/settings.h>
#include <stdio.h>

#include "bmx160.h"


#include "quaternion.h"
#include "sensor_processing_lib.h"
#include "vector_3d.h"


#include <storage/disk_access.h>
#include <fs/fs.h>
#include <ff.h>


/**
 * @brief 
 * SD Card Logging Section
 * 
 **/

static const char *disk_mount_pt = "/SD:";
char filename[255];
struct fs_file_t file;

static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		printk("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printk("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR ] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);

	return res;
}

static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};


/***********SD CARD LOGGING***************/


#define SENSOR_READ_MS_THREAD   0.625 //ms = 1600 HZ
LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

static bool ready = true;
static struct esb_payload rx_payload;
static struct esb_payload tx_payload;

#define _RADIO_SHORTS_COMMON                                                   \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |         \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                                  \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

void event_handler(struct esb_evt const *event)
{
	ready = true;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		while (esb_read_rx_payload(&rx_payload) == 0) {
			LOG_DBG("Packet received, len %d : "
				"0x%02x, 0x%02x, 0x%02x, 0x%02x, "
				"0x%02x, 0x%02x, 0x%02x, 0x%02x",
				rx_payload.length, rx_payload.data[0],
				rx_payload.data[1], rx_payload.data[2],
				rx_payload.data[3], rx_payload.data[4],
				rx_payload.data[5], rx_payload.data[6],
				rx_payload.data[7]);
		}
		break;
	}
}

int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.event_handler = event_handler;
	config.mode = ESB_MODE_PTX;
	config.selective_auto_ack = true;


	err = esb_init(&config);

	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	return 0;
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

/*****BMX160 IMU SPI*****/
bmx160_spi  bmxIMU_spi = {
	.spi_cfg = {
		.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
		.frequency = 1000000,
		.slave = 0,
		},
	.spi_dev = NULL,
	.spi_cs_pin = 11,
	.gpio0_dev = NULL,
};

bmx160_sensor_data bmx160_data;

float delta,wx,wy,wz;
euler_angles angles;
vector_ijk fused_vector;
Quaternion q_acc; 
float pitch_angle;



/*****BMX160 IMU SPI*****/

  

void main(void)
{
	int err;
	int len;
	char* data;
	data = (char*)malloc(64 * sizeof(char));

	err = clocks_start();
 	if (err) {
		return;
	}



	if (device_get_binding("SPI_1") == NULL) {
		printk("Could not get %s device\n", "SPI_1");
		return;
	}
	
	/* raw disk i/o */
	do {
		static const char *disk_pdrv = "SD";
		uint64_t memory_size_mb;
		uint32_t block_count;
		uint32_t block_size;

		if (disk_access_init(disk_pdrv) != 0) {
			LOG_ERR("Storage init ERROR!");
			break;
		}

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
			LOG_ERR("Unable to get sector count");
			break;
		}
		LOG_INF("Block count %u", block_count);

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			LOG_ERR("Unable to get sector size");
			break;
		}
		printk("Sector size %u\n", block_size);

		memory_size_mb = (uint64_t)block_count * block_size;
		printk("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));
	} while (0);	

	mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

	if (res == FR_OK) {
		printk("Disk mounted.\n");
		lsdir(disk_mount_pt);
	} else {
		printk("Error mounting disk.\n");
	}

	fs_file_t_init(&file);

	snprintf(filename, sizeof(filename), "%s/test.txt", mp.mnt_point);




	/**
	 * @brief
	 * Initialize Sensor Object and communication port through SPI
	 **/
	
	bmxIMU_spi.gpio0_dev= device_get_binding("GPIO_0");
	gpio_pin_configure(bmxIMU_spi.gpio0_dev, bmxIMU_spi.spi_cs_pin, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	const char* const spiName0 = "SPI_0";
	bmxIMU_spi.spi_dev = device_get_binding(spiName0);

	if (bmxIMU_spi.spi_dev == NULL) {
		printk("Could not get %s device\n", spiName0);
		return;
	}
	else
		printk("Got SPI device\n");
	begin_bmx160(&bmxIMU_spi);
	k_msleep(1000);
	/**
	 * @brief
	 * Initial conditions for algorithm.
	 **/
	//fused_vector = vector_3d_initialize(0.0,0.0,-1.0);
	//q_acc = quaternion_initialize(1.0,0.0,0.0,0.0);


	/**
	 * @brief
	 * Onboard led for sync with video
	 **/
	gpio_pin_configure(device_get_binding("GPIO_0"), 7, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	gpio_pin_set(device_get_binding("GPIO_0"), 7, 1);
	k_msleep(1000);



	
	for (;;) {
    // Main entry point after IMU sensors and BLE init 

    // Read filtered BMX160 IMU data at 200MHZ and store it to the struct
	err = getData_bmx160(&bmxIMU_spi, &bmx160_data);
	if (bmx160_data.ax == 0)
	{
		gpio_pin_set(device_get_binding("GPIO_0"), 7, 0);
	}
	

	delta = 0.001f * SENSOR_READ_MS_THREAD;
	//  Serial.print("Time Delta: \t");
	//  Serial.print(delta,16);
	//  Serial.println();

	fused_vector = update_fused_vector(fused_vector,
									   bmx160_data.ax,bmx160_data.ay,bmx160_data.az,
									   bmx160_data.gx,bmx160_data.gy,bmx160_data.gz,
									   delta);

	q_acc = quaternion_from_accelerometer(fused_vector.a,fused_vector.b,fused_vector.c);
	angles = quaternion_to_euler_angles(q_acc);

	if(angles.yaw > 0 && angles.pitch > 0)
		pitch_angle = angles.pitch;
	else if(angles.yaw < 0 && angles.pitch > 0)
		pitch_angle = 180 - angles.pitch;
	else if(angles.yaw > 0 && angles.pitch < 0)
		pitch_angle = 180 - angles.pitch;    
	else if(angles.yaw < 0 && angles.pitch < 0)
		pitch_angle = 360 + angles.pitch;
	
	len = sprintf(data,"%f\n",pitch_angle);

	// len = sprintf(data, "G_B: %.3f, %.3f, %.3f\nA_B: %.3f, %.3f, %.3f", 
	// bmx160_data.gx, bmx160_data.gy, bmx160_data.gz,
	// bmx160_data.ax, bmx160_data.ay, bmx160_data.az);


	if(len > 0){
		res = fs_open(&file, filename, FS_O_CREATE | FS_O_RDWR);

		if (res < 0) {
			printk("FAIL: open %s: %d", filename, res);
			goto out;
		}

		res = fs_seek(&file, 0, FS_SEEK_END);
		if (res < 0) {
			printk("FAIL: seek %s: %d", filename, res);
			goto out;
		}	
		


		res = fs_write(&file, data, len);
		if (res < 0) {
			printk("FAIL: write %s: %d", filename, res);
			goto out;
		}
 out:
		res = fs_close(&file);
		if (res < 0) {
			printk("FAIL: close %s: %d", filename, res);
		}
	}

		
    // Sleep and wait for next cycle
	k_sleep(K_MSEC(6.25));
	}
}
