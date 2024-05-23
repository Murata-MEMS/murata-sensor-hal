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

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <endian.h>
#include <sys/stat.h>

#include <hardware/sensors.h>
#include <utils/Log.h>

#include <unistd.h>
#include <stdbool.h>
#include <endian.h>

#include "utils.h"

#include "sensor_device.h"

#define MAX_DEVICES (10)
#define MAX_SENSORS (MAX_DEVICES * 2)
#define MAX_CHANNELS 20
static const char *MURATA_VENDOR_STRING = "Murata";

#define SCH16XX_HAL_MIN_FREQ_HZ 10
#define SCH16XX_HAL_MAX_FREQ_HZ (1000)

static int supported_sample_frequencies[] = { 10, 40, 100, 200, 400, 1000 };

static int64_t sign_extend(int64_t value, int bits)
{
	value <<= (64 - bits);
	value >>= (64 - bits);
	return value;
}

static int64_t mask_upper_bits(int64_t value, int bits)
{
	return value & ~(((int64_t)1 << bits) - 1);
}

static double convert_data(uint8_t *src, struct channel_format *format, int64_t *raw_out)
{
	// let the data be in the network endianness
	int64_t raw = 0;
	bool swap = false;
	int storage_bytes = format->storage_bits / 8;

	if (swap) {
		int b;
		for (b = 0; b < storage_bytes; b++) {
			raw |= *(src + storage_bytes - b - 1);
			raw <<= 8;
		}
	} else {
		memcpy(&raw, src, storage_bytes);
	//	raw >>= (64 - format->storage_bits);
	}

	if (format->shift)
		raw >>= format->shift;

	if (format->is_signed)
		raw = sign_extend(raw, format->bits);
	else
		raw = mask_upper_bits(raw, format->bits);

	if (raw_out != NULL)
		*raw_out = raw;

	//ALOGI("%s: %02x %02x %02x %02x -> %016lx -> %lf", __FUNCTION__,
	//		*src, *(src+1), *(src+2), *(src+3), raw, raw * format->scale);

	return raw * format->scale;
}

static int fill_event_data(int32_t handle, int32_t sensor_type, struct device_data *device, uint8_t *buf, struct sensors_event_t *event)
{
	int scan_index;

	event->sensor = handle;
	event->type = sensor_type;
	event->version = sizeof(*event);

	for (scan_index = 0; scan_index <= device->max_scan_index; scan_index++) {
		double value;

		if (!device->format[scan_index].enabled) {
		//	ALOGI("scan %d: not enabled", scan_index);
			continue;
		}

		if (device->format[scan_index].is_timestamp) {
			convert_data(buf, &device->format[scan_index], &event->timestamp);
		}

		if (device->format[scan_index].sensor_type == sensor_type) {

			value = convert_data(buf, &device->format[scan_index], NULL);
			event->data[device->format[scan_index].event_data_index] = value;

		//	ALOGI("scan %d: data[%d] <- %lf", scan_index, device->format[scan_index].event_data_index, value);
		} else {
		//	ALOGI("scan %d: type %d not %d", scan_index, device->format[scan_index].sensor_type, sensor_type);
		}

		buf += device->format[scan_index].storage_bits / 8;
	}

	return 0;
}

static void *device_worker(void *arg)
{
	struct device_data *device = (struct device_data *)arg;
	int device_fd;
	char device_file[200];
	int ret;

	sprintf(device_file, "/dev/iio:device%d", device->iio_device_num);

	device_fd = open(device_file, O_RDONLY);
	if (device_fd < 0) {
		ALOGE("Failed to open %s. errno %d %s", device_file, errno, strerror(errno));
		return (void *)-1;
	}

	while(!device->must_stop) {
		int data_size = device->scan_size;
		uint8_t buf[100];
		sensors_event_t event[2] = { 0 };
		int event_count = 0;

		ret = read(device_fd, buf, data_size);

		if (device->must_stop)
			break;

		if (ret <= 0) {
			ALOGE("read returned %d. %s", ret, strerror(errno));
			break;
		}

		if (device->gyro_active) {
			fill_event_data(device->gyro_sensor_num, SENSOR_TYPE_GYROSCOPE, device, buf, &event[event_count]);
			event_count++;
		}

		if (device->accel_active) {
			fill_event_data(device->accel_sensor_num, SENSOR_TYPE_ACCELEROMETER, device, buf, &event[event_count]);
			event_count++;
		}

		ret = pthread_mutex_lock(&device->module_data->pipe_mutex);
		if (ret != 0) {
			// error
		}
		write(device->module_data->write_pipe_fd, event, sizeof(struct sensors_event_t) * event_count);

		pthread_mutex_unlock(&device->module_data->pipe_mutex);
	}
	close(device_fd);
	return (void *)0;
}

/**
 * Write an array of sensor_event_t to data. The size of the
 * available buffer is specified by count. Returns number of
 * valid sensor_event_t.
 *
 * This function should block if there is no sensor event
 * available when being called. Thus, return value should always be
 * positive.
 */
int device_poll(struct sensors_poll_device_t *dev,
			       sensors_event_t* events, int count)
{
	struct sensor_module_data *data = (struct sensor_module_data *)dev;

	int ret = read(data->read_pipe_fd, events, sizeof(sensors_event_t));

	if (ret != sizeof(sensors_event_t)) return -1;

	return 1;
}

#define min(a, b) (a > b ? b : a)
#define max(a, b) (a > b ? a : b)

static int update_sampling_frequency(struct device_data *device)
{
	char sampling_frequency_path[PATH_MAX];
	int sampling_frequency_hz;
	int ret;

	struct sensor_data *accel_sensor = &device->module_data->sensor_data[device->accel_sensor_num];
	struct sensor_data *gyro_sensor = &device->module_data->sensor_data[device->gyro_sensor_num];

	sprintf(sampling_frequency_path, "/sys/bus/iio/devices/trigger%d/sampling_frequency", device->iio_trigger_device_num);

	sampling_frequency_hz = 1;
	if (accel_sensor->enabled)
		sampling_frequency_hz = max(sampling_frequency_hz, accel_sensor->sampling_frequency_hz);

	if (gyro_sensor->enabled)
		sampling_frequency_hz = max(sampling_frequency_hz, gyro_sensor->sampling_frequency_hz);

	ret = sysfs_write_int(sampling_frequency_path, sampling_frequency_hz);

	if (ret < 0)
		return ret;

	return 0;
}

static int period_ns_to_frequency(int64_t period_ns)
{
	int frequency;

	if (period_ns == 0) {
		frequency = SCH16XX_HAL_MAX_FREQ_HZ;

	} else {
		frequency = 1000000000LL / period_ns;

		if (frequency > SCH16XX_HAL_MAX_FREQ_HZ)
			frequency = SCH16XX_HAL_MAX_FREQ_HZ;
		else if (frequency < SCH16XX_HAL_MIN_FREQ_HZ)
			frequency = SCH16XX_HAL_MIN_FREQ_HZ;
	}

	return frequency;
}

static int set_filter_frequency(struct device_data *device, struct sensor_data *sensor, int filter_frequency_hz)
{
	char filter_frequency_path[PATH_MAX];
	
	sprintf(filter_frequency_path, "/sys/bus/iio/devices/iio:device%d/in_%s_filter_low_pass_3db_frequency",
		sensor->device_num, sensor->is_accel ? "accel" : "anglvel");

	if (sysfs_write_int(filter_frequency_path, filter_frequency_hz) < 0)
		return -1;

	return 0;
}

static bool is_supported_frequency(int freq)
{
	int i = 0;
	for (i = 0; i < sizeof(supported_sample_frequencies) / sizeof(supported_sample_frequencies[0]); i++) {
		if (freq == supported_sample_frequencies[i])
			return true;
	}
	return false;
}

/**
 * Find a filter frequency for the given sample rate based on Nyquist theorem
 * 
 * Select the highest frequency below sample rate / 2, or the lowest 
 * available filter frequecy, if no better match is found.
*/
static int find_best_filter_freq(struct sensor_data *sensor, int sample_rate)
{
	int best_i = -1;
	int best_freq = -1;
	int lowest_freq = -1;
	int i;
	for (i = 0; i < sensor->filter_freq_count; i++) {
		if (lowest_freq == -1 || lowest_freq > sensor->filter_freq_available[i])
			lowest_freq = sensor->filter_freq_available[i];

		if (sensor->filter_freq_available[i] > best_freq && sensor->filter_freq_available[i] <= sample_rate / 2) {
			best_i = i;
			best_freq = sensor->filter_freq_available[i];
		}
	}
	if (best_i == -1)
		return lowest_freq;
	
	return sensor->filter_freq_available[best_i];
}

/**
 * Set the events's period in nanoseconds for a given sensor.
 * If sampling_period_ns > max_delay it will be truncated to
 * max_delay and if sampling_period_ns < min_delay it will be
 * replaced by min_delay.
 */
int device_set_delay(struct sensors_poll_device_t *dev,
				    int handle, int64_t sampling_period_ns)
{
	struct sensor_module_data *data = (struct sensor_module_data *)dev;
	struct sensor_data *sensor = &data->sensor_data[handle];
	struct device_data *device = &data->device_data[sensor->device_num];

	int frequency = period_ns_to_frequency(sampling_period_ns);
	int filter_frequency;

	if (!is_supported_frequency(frequency)) {
		ALOGE("Sampling period %lld ns (frequency %d Hz) is not supported.", sampling_period_ns, frequency);
		return -1;
	}

	sensor->sampling_frequency_hz = frequency;

	filter_frequency = find_best_filter_freq(sensor, frequency);

	if (set_filter_frequency(device, sensor, filter_frequency) < 0) {
		ALOGE("Error while setting filter frequency for '%s'.", sensor->sensor_name);
		return -1;
	}

	if (update_sampling_frequency(device) < 0) {
		ALOGE("Error while setting sampling frequency for '%s'.", sensor->sensor_name);
		return -1;
	}
	return 0;
}


/*
 * Sets a sensor's parameters, including sampling frequency and maximum
 * report latency. This function can be called while the sensor is
 * activated, in which case it must not cause any sensor measurements to
 * be lost: transitioning from one sampling rate to the other cannot cause
 * lost events, nor can transitioning from a high maximum report latency to
 * a low maximum report latency.
 * See the Batching sensor results page for details:
 * http://source.android.com/devices/sensors/batching.html
 */
int device_batch (struct sensors_poll_device_1* dev,
		  int sensor_handle, __attribute__((unused)) int flags,
		  int64_t sampling_period_ns,
		  __attribute__((unused)) int64_t max_report_latency_ns)
{
	return device_set_delay((struct sensors_poll_device_t*)dev,
		sensor_handle, sampling_period_ns);

}

/*
  * Flush adds a META_DATA_FLUSH_COMPLETE event (sensors_event_meta_data_t)
  * to the end of the "batch mode" FIFO for the specified sensor and flushes
  * the FIFO.
  * If the FIFO is empty or if the sensor doesn't support batching (FIFO size zero),
  * it should return SUCCESS along with a trivial META_DATA_FLUSH_COMPLETE event added to the
  * event stream. This applies to all sensors other than one-shot sensors.
  * If the sensor is a one-shot sensor, flush must return -EINVAL and not generate
  * any flush complete metadata.
  * If the sensor is not active at the time flush() is called, flush() should return
  * -EINVAL.
  */
int device_flush (struct sensors_poll_device_1* dev,
		  int handle)
{
	int ret;
	sensors_event_t event;
	struct sensor_module_data *data = (struct sensor_module_data *)dev;
	struct sensor_data *sensor = &data->sensor_data[handle];
	struct device_data *device = &data->device_data[sensor->device_num];

	event.sensor = handle;
	event.version = META_DATA_VERSION;
	event.type = SENSOR_TYPE_META_DATA;
	event.meta_data.what = META_DATA_FLUSH_COMPLETE;
	event.meta_data.sensor = handle;
	// Other Sensors HAL seem to set flush event timestamp to zero.
	event.timestamp = 0;

	ret = pthread_mutex_lock(&device->module_data->pipe_mutex);
	if (ret != 0) {
		// error
	}
	write(device->module_data->write_pipe_fd, &event, sizeof(event));

	pthread_mutex_unlock(&device->module_data->pipe_mutex);

	return 0;
}

/*
 * Inject a single sensor sample to be to this device.
 * data points to the sensor event to be injected
 * return 0 on success
 *         -EPERM if operation is not allowed
 *         -EINVAL if sensor event cannot be injected
 */
int device_inject_sensor_data(struct sensors_poll_device_1 *dev, const sensors_event_t *data)
{
	(void)dev;
	(void)data;
	return -EINVAL;
}


static int enable_channel(struct device_data *device, const char *channel, int enable)
{
	char enable_file[PATH_MAX];
	snprintf(enable_file, sizeof(enable_file), "/sys/bus/iio/devices/iio:device%d/scan_elements/in_%s_en",
			device->iio_device_num, channel);
	return sysfs_write_str(enable_file, enable ? "1" : "0");
}

/*
 * [be|le]:[s|u]bits/storagebits[Xrepeat][>>shift] .
 */
static int parse_channel_params(struct device_data *device, int channel_index, float scale, const char *channel_type,
		int32_t sensor_type, int event_data_index, bool is_timestamp)
{
	struct channel_format *format = &device->format[channel_index];
	char endian, sign;
	int ret;
	if (strchr(channel_type, 'X')) {
		ret = sscanf(channel_type, "%ce:%c%u/%uX%u>>%u",
			&endian, &sign,
			&format->bits, &format->storage_bits,
			&format->repeat, &format->shift);
		if (ret != 6)
			return -1;
		if (format->repeat == 0)
			format->repeat = 1;
	} else {
		format->repeat = 1;
		ret = sscanf(channel_type, "%ce:%c%u/%u>>%u",
				&endian, &sign,
				&format->bits, &format->storage_bits,
				&format->shift);
		if (ret != 5)
			return -1;
	}
	format->is_signed = (sign == 's' || sign == 'S');
	format->is_big_endian = endian == 'b';

	format->scale = scale;

	format->enabled = 1;
	format->sensor_type = sensor_type;
	format->event_data_index = event_data_index;
	format->is_timestamp = is_timestamp;

	return 0;
}

static int read_channel_params(struct device_data *device, const char *channel, int32_t sensor_type, int event_data_index, bool is_timestamp)
{
	char file_path[PATH_MAX];
	int index;
	float scale;
	int ret;
	char type_buf[100];

	sprintf(file_path, "/sys/bus/iio/devices/iio:device%d/scan_elements/in_%s_index",
			device->iio_device_num, channel);
	ret = sysfs_read_int(file_path, &index);
	if (ret < 0)
		return ret;

	sprintf(file_path, "/sys/bus/iio/devices/iio:device%d/scan_elements/in_%s_type",
				device->iio_device_num, channel);
	ret = sysfs_read_str(file_path, type_buf, sizeof(type_buf));
	if (ret < 0)
		return ret;

	sprintf(file_path, "/sys/bus/iio/devices/iio:device%d/in_%s_scale",
					device->iio_device_num, channel);
	if (access(file_path, R_OK) == 0) {
		ret = sysfs_read_float(file_path, &scale);

		if (ret < 0)
			return ret;
	} else {
		scale = 1;
	}

	ret = parse_channel_params(device, index, scale, type_buf, sensor_type, event_data_index, is_timestamp);
	if (ret < 0)
		return ret;

	if (index > device->max_scan_index)
		device->max_scan_index = index;

	return 0;
}

static int prepare_channel(struct device_data *device, const char *channel, int32_t sensor_type, int event_data_index, bool is_timestamp)
{
	int ret = read_channel_params(device, channel, sensor_type, event_data_index, is_timestamp);
	if (ret < 0)
		return ret;

	return enable_channel(device, channel, 1);
}

static int update_scan_size(struct device_data *device)
{
	int i;
	int scan_size = 0;

	for (i = 0; i <= device->max_scan_index; i++) {

		if (device->format[i].enabled)
			scan_size += device->format[i].storage_bits / 8;
	}
	device->scan_size = scan_size;

	return 0;
}

static int start_capture(struct device_data *device)
{
	ALOGI(__FUNCTION__);

	sysfs_write_str_format_path("0", "/sys/bus/iio/devices/iio:device%d/buffer/enable",
			device->iio_device_num);

	prepare_channel(device, "accel_x", SENSOR_TYPE_ACCELEROMETER, 0, false);
	prepare_channel(device, "accel_y", SENSOR_TYPE_ACCELEROMETER, 1, false);
	prepare_channel(device, "accel_z", SENSOR_TYPE_ACCELEROMETER, 2, false);
	prepare_channel(device, "anglvel_x", SENSOR_TYPE_GYROSCOPE, 0, false);
	prepare_channel(device, "anglvel_y", SENSOR_TYPE_GYROSCOPE, 1, false);
	prepare_channel(device, "anglvel_z", SENSOR_TYPE_GYROSCOPE, 2, false);
	prepare_channel(device, "timestamp", -1, -1, true);

	update_scan_size(device);

	update_sampling_frequency(device);

	sysfs_write_str_format_path("1", "/sys/bus/iio/devices/iio:device%d/buffer/enable",
				device->iio_device_num);

	device->must_stop = 0;

	pthread_create(&device->worker_thread,
			NULL, device_worker, device);

	return 0;
}

static int stop_capture(struct device_data *device)
{
	int ret;
	void *return_value;
	intptr_t return_int;
	device->must_stop = 1;

	ret = pthread_join(device->worker_thread, &return_value);
	(void)ret;
	return_int = (intptr_t)return_value;

	sysfs_write_str_format_path("0", "/sys/bus/iio/devices/iio:device%d/buffer/enable",
				device->iio_device_num);

	return (int)return_int;
}

/* Activate/de-activate one sensor.
 *
 * sensor_handle is the handle of the sensor to change.
 * enabled set to 1 to enable, or 0 to disable the sensor.
 *
 * Before sensor activation, existing sensor events that have not
 * been picked up by poll() should be abandoned so that application
 * upon new activation request will not get stale events.
 * (events that are generated during latter activation or during
 * data injection mode after sensor deactivation)
 *
 * Return 0 on success, negative errno code otherwise.
 */
int device_activate(struct sensors_poll_device_t *dev,
				   int handle, int enabled)
{
	struct sensor_module_data *data = (struct sensor_module_data *)dev;
	struct sensor_data *sensor = &data->sensor_data[handle];
	struct device_data *device = &data->device_data[sensor->device_num];
	int is_running = device->accel_active | device->gyro_active;

	if (sensor->enabled == enabled)
		return 0;

	sensor->enabled = enabled;

	if (sensor->sensor_type == SENSOR_TYPE_ACCELEROMETER)
		device->accel_active = enabled;

	else if(sensor->sensor_type == SENSOR_TYPE_GYROSCOPE)
		device->gyro_active = enabled;
	else
		ALOGW("%s: unsupported sensor type %d", __FUNCTION__, sensor->sensor_type);

	update_sampling_frequency(device);

	if (device->accel_active || device->gyro_active)
		if (!is_running)
			start_capture(device);

	if (!device->accel_active && !device->gyro_active)
		stop_capture(device);

	return 0;
}

static int find_trigger(const char *trigger_name)
{
	ALOGI(__FUNCTION__);

	int iio_num;
	for (iio_num = 0; iio_num < MAX_DEVICES; iio_num++) {

		char iio_name_path[PATH_MAX];
		char iio_name[200];
		snprintf(iio_name_path, sizeof(iio_name_path), "/sys/bus/iio/devices/trigger%d/name", iio_num);

		if (access(iio_name_path, F_OK) != 0)
			// the file does not exist, assuming there are no more iio devices
			break;

		sysfs_read_str(iio_name_path, iio_name, sizeof(iio_name));

		if (strcmp(trigger_name, iio_name) == 0) {

			return iio_num;
		}
	}

	return -1;
}

static int create_trigger(const char *trigger_name)
{
	struct stat dir_status;
	char trigger_path[100];
#define CONFIGFS_TRIGGER_PATH "/sys/kernel/config/iio/triggers"
	/* Get parent dir status */
	if (stat(CONFIGFS_TRIGGER_PATH, &dir_status)) {
		ALOGE("stat for '%s' failed.", CONFIGFS_TRIGGER_PATH);
		return -1;
	}
	snprintf(trigger_path, sizeof(trigger_path), "%s/hrtimer/%s", CONFIGFS_TRIGGER_PATH, trigger_name);

	/* Create hrtimer with the same access rights as it's parent */
	if (mkdir(trigger_path, dir_status.st_mode)) {
		if (errno != EEXIST) {
			ALOGE("Creating trigger '%s' failed (mkdir %s).", trigger_name, trigger_path);
			return -1;
		}
	}

	return 0;
}

static int setup_trigger_for_device(struct device_data *device)
{
	int trigger_device_num;

	char trigger_name[100];
	char current_trigger_path[PATH_MAX];

	snprintf(trigger_name, sizeof(trigger_name), "iio:device%d-trigger", device->iio_device_num);

	trigger_device_num = find_trigger(trigger_name);
	if (trigger_device_num < 0)
		create_trigger(trigger_name);

	trigger_device_num = find_trigger(trigger_name);
	if (trigger_device_num < 0) {
		ALOGE("Cannot find trigger named '%s'", trigger_name);
		return -1;
	}
	snprintf(current_trigger_path, sizeof(current_trigger_path),
			"/sys/bus/iio/devices/iio:device%d/trigger/current_trigger", device->iio_device_num);

	sysfs_write_str(current_trigger_path, trigger_name);

	device->iio_trigger_device_num = trigger_device_num;

	return 0;
}

#define DEFAULT_SAMPLING_FREQUENCY_HZ (100)

static float iio_device_read_max_float(int iio_device_num, const char *attr1, const char *attr2, const char *attr3)
{
	float max = 0;
	const char *attrs[] = { attr1, attr2, attr3 };
	int i;
	for (i = 0; i < 3; i++) {
		float value = NAN;
		if (sysfs_read_float_format_path(&value, "/sys/bus/iio/devices/iio:device%d/%s", iio_device_num, attrs[i]) == 0) {
			if (value > max)
				max = value;
		}
	}
	return max;
}

static int add_sensor(struct sensor_module_data *data, int iio_device_num, int sensor_type, const char *attr_prefix, bool is_accel)
{
	char scale_attr_x[100], scale_attr_y[100], scale_attr_z[100];

	snprintf(scale_attr_x, sizeof(scale_attr_x), "in_%s_x_scale", attr_prefix);
	snprintf(scale_attr_y, sizeof(scale_attr_y), "in_%s_y_scale", attr_prefix);
	snprintf(scale_attr_z, sizeof(scale_attr_z), "in_%s_z_scale", attr_prefix);

	// TODO: Sensor handle is greater than 0 and less than INT32_MAX.
	int sensor_handle = data->sensor_count;

	struct sensor_t *sensor = &(data->sensors_list[sensor_handle]);
	struct sensor_data *sensor_data = &(data->sensor_data[sensor_handle]);
	data->sensor_count++;

	sprintf(sensor_data->sensor_name, "iio%d", iio_device_num);

	sensor->name = sensor_data->sensor_name;
	sensor->vendor = MURATA_VENDOR_STRING;
	sensor->version = 1;
	sensor->handle = sensor_handle;
	sensor->type = sensor_type;
	sensor->resolution = iio_device_read_max_float(iio_device_num, scale_attr_x, scale_attr_y, scale_attr_z);
	sensor->minDelay = 1000000L / SCH16XX_HAL_MAX_FREQ_HZ;
	sensor->maxDelay = 1000000L / SCH16XX_HAL_MIN_FREQ_HZ;
	sensor->requiredPermission = "";
	sensor->fifoReservedEventCount = 0;
	sensor->fifoMaxEventCount = 1;

	sysfs_read_float_format_path(&sensor->maxRange, "/sys/bus/iio/devices/iio:device%d/%s_range",
			iio_device_num, attr_prefix);
	sensor->maxRange = sensor->maxRange * M_PI / 180.0;

	sensor_data->sensor_type = sensor->type;
	sensor_data->is_accel = is_accel;
	sensor_data->sampling_frequency_hz = DEFAULT_SAMPLING_FREQUENCY_HZ;

	sysfs_read_int_array_format_path(sensor_data->filter_freq_available, &sensor_data->filter_freq_count, MAX_FILTER_FREQ_AVAILABLE,
		"/sys/bus/iio/devices/iio:device%d/in_%s_filter_low_pass_3db_frequency_available", iio_device_num, attr_prefix);

	for(int i = 0; i < sensor_data->filter_freq_count; i++) {
		ALOGI("%s %d: %d", attr_prefix, i, sensor_data->filter_freq_available[i]);
	}

	return sensor_handle;
}

static int add_iio_device(struct sensor_module_data *data, int iio_device_num)
{
	int device_handle = data->device_count;
	data->device_count++;

	struct device_data *device = &data->device_data[device_handle];

	device->accel_sensor_num = add_sensor(data, iio_device_num, SENSOR_TYPE_ACCELEROMETER, "accel", true);
	device->gyro_sensor_num = add_sensor(data, iio_device_num, SENSOR_TYPE_GYROSCOPE, "anglvel", false);

	device->module_data = data;
	device->iio_device_num = iio_device_num;

	setup_trigger_for_device(device);

	return 0;
}

int find_sensors(struct sensor_module_data *data)
{
	ALOGI(__FUNCTION__);

	int iio_num;
	for (iio_num = 0; iio_num < MAX_DEVICES; iio_num++) {

		char iio_name_path[PATH_MAX];
		char iio_name[200];
		snprintf(iio_name_path, sizeof(iio_name_path), "/sys/bus/iio/devices/iio:device%d/name", iio_num);

		if (access(iio_name_path, F_OK) != 0)
			// the file does not exist, assuming there are no more iio devices
			break;

		sysfs_read_str(iio_name_path, iio_name, sizeof(iio_name));

		if (strcmp("sch16xx", iio_name) == 0) {

			snprintf(data->device_data[data->device_count].device_name, 
				sizeof(data->device_data[data->device_count].device_name), "%s", iio_name);
			add_iio_device(data, iio_num);
		}
	}

	return 0;
}


