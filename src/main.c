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

#include "quaternion.h"
#include "sensor_processing_lib.h"
#include "vector_3d.h"

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

float delta,wx,wy,wz;
euler_angles angles;
vector_ijk fused_vector;
Quaternion q_acc; 
float pitch_angle;


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


	fused_vector = vector_3d_initialize(0.0,0.0,-1.0);
	q_acc = quaternion_initialize(1.0,0.0,0.0,0.0);

	
	for (;;) {
    // Main entry point after IMU sensors and BLE init 

    // Read filtered BMX160 IMU data at 200MHZ and store it to the struct
	getData_bmx160(&bmxIMU_spi, &bmx160_data);

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
	else;


    // Sleep and wait for next cycle
	k_sleep(K_MSEC(SENSOR_READ_MS_THREAD));
	}
}
