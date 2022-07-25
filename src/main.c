/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <logging/log.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "bmx160.h"

/*****BMX160 IMU SPI*****/
bmx160_spi  bmxIMU_spi = {
	.spi_cfg = {
		.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
		.frequency = 4000000,
		.slave = 0,
		},
	.spi_dev = NULL,
	.spi_cs_pin = 11,
	.gpio0_dev = NULL,
};

bmx160_sensor_data bmx160_data;

#define SENSOR_READ_MS_THREAD   0.625 //ms = 1600 HZ


LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

void main(void)
{

	printk("Hello World! %s\n", CONFIG_BOARD);

	bmxIMU_spi.gpio0_dev= device_get_binding("GPIO_0");

	
	LOG_ERR("This is a error message!");
	LOG_WRN("This is a warning message!");
	LOG_INF("This is a information message!");
	LOG_DBG("This is a debugging message!");

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

	for (;;) {
    // Main entry point after IMU sensors and BLE init 

    // Read filtered BMX160 IMU data at 200MHZ and store it to the struct
	getData_bmx160(&bmxIMU_spi, &bmx160_data);

    // Sleep and wait for next cycle
	k_sleep(K_MSEC(SENSOR_READ_MS_THREAD));
	}
}
