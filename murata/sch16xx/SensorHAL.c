//
// This software is released under the BSD license as follows.
// Copyright (c) 2024, Murata Electronics Oy.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following 
// conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.
//    3. Neither the name of Murata Electronics Oy nor the names of its    
//       contributors may be used to endorse or promote products derived    
//       from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <stddef.h>
#include <string.h>
#include <hardware/sensors.h>
#include <pthread.h>
#include <utils/Log.h>

#include <errno.h>
#include "sensor_device.h"

struct sensor_data;
struct device_data;


static int close_device(struct hw_device_t* device)
{
	struct sensor_module_data *data = (struct sensor_module_data *)device->module->dso;

	// one should stop all the data acquisition before closing the device
	close(data->read_pipe_fd);
	close(data->write_pipe_fd);

	return 0;
}

static int open_sensors_module(const struct hw_module_t *module,
			       const char __attribute__((unused))*id,
			       struct hw_device_t **device)
{
	int pipefd[2];
	int ret;

	struct sensor_module_data *data;

	data = (struct sensor_module_data *)calloc(1, sizeof(struct sensor_module_data));

	if (strcmp(id, SENSORS_HARDWARE_POLL))
			return 0;

	data->poll_device.common.tag = HARDWARE_DEVICE_TAG;
	data->poll_device.common.version = SENSORS_DEVICE_API_VERSION_1_3;
	data->poll_device.common.module = (struct hw_module_t*) module;
	data->poll_device.common.close = close_device;
	data->poll_device.common.module->dso = data;

	data->poll_device.activate = device_activate;
	data->poll_device.setDelay = device_set_delay;
	data->poll_device.poll = device_poll;
	data->poll_device.batch = device_batch;
	data->poll_device.flush = device_flush;
	data->poll_device.inject_sensor_data = device_inject_sensor_data;

	*device = &data->poll_device.common;

	ret = pipe(pipefd);
	if (ret) {
		ALOGE("Error creating sensors event pipe.");
		return -ENOMEM;
	}

	data->read_pipe_fd = pipefd[0];
	data->write_pipe_fd = pipefd[1];

	pthread_mutex_init(&data->pipe_mutex, NULL);

	ALOGI("Initializing Murata sensors HAL module\n");

	find_sensors(data);

	return 0;
}

/**
  *  Place the module in a specific mode. The following modes are defined
  *
  *  0 - Normal operation. Default state of the module.
  *  1 - Loopback mode. Data is injected for the supported
  *      sensors by the sensor service in this mode.
  * return 0 on success
  *         -EINVAL if requested mode is not supported
  *         -EPERM if operation is not allowed
  */
static int set_operation_mode(unsigned int mode)
{
	if (mode == 0)
		return 0;

	return -EINVAL;
}

/**
   * Enumerate all available sensors. The list is returned in "list".
   * return number of sensors in the list
   */
static int get_sensors_list(struct sensors_module_t *module,
				   struct sensor_t const **list)
{
	struct sensor_module_data *data = (struct sensor_module_data *)module->common.dso;

	*list = data->sensors_list;
	return data->sensor_count;
}

static struct hw_module_methods_t sensors_module_methods = {
	.open = open_sensors_module
};

/*
 * struct sensors_module_t - Hardware module info
 * see Android documentation.
 */
struct sensors_module_t HAL_MODULE_INFO_SYM = {
	.common = {
		.tag = HARDWARE_MODULE_TAG,
		.module_api_version = SENSORS_MODULE_API_VERSION_0_1,
		.hal_api_version = 0,
		.id = SENSORS_HARDWARE_MODULE_ID,
		.name = "Murata Sensors Module",
		.author = "Murata",
		.methods = &sensors_module_methods,
	},
	.get_sensors_list = get_sensors_list,
	.set_operation_mode = set_operation_mode,
};
