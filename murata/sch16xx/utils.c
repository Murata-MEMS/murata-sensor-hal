/*
// Copyright (c) 2015 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <stdlib.h>
#include <fcntl.h>
//#include <sys/ioctl.h>
#include <utils/Log.h>
//#include <hardware/sensors.h>
//#include <utils/Atomic.h>
//#include <linux/android_alarm.h>

#include "utils.h"

#include <errno.h>
#include <stdarg.h>

/* Note:
 *
 * Some of these calls are going to fail, because not all sensors expose all
 * possible sysfs attributes. As an optimization we may want to cache which
 * ones are valid and immediately return in error for inexistent entries.
 */
int sysfs_read(const char path[PATH_MAX], void *buf, int buf_len)
{
	int fd, len;

	if (!path[0] || !buf || buf_len < 1)
		return -1;

	fd = open(path, O_RDONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = read(fd, buf, buf_len);

	close(fd);

	if (len == -1)
		ALOGW("Cannot read from %s (%s)\n", path, strerror(errno));
	else
		ALOGV("Read %d bytes from %s\n", len, path);

	return len;
}


int sysfs_write(const char path[PATH_MAX], const void *buf, const int buf_len)
{
	int fd, len;

	if (!path[0] || !buf || buf_len < 1)
		return -1;

	fd = open(path, O_WRONLY);

	if (fd == -1) {
		ALOGV("Cannot open %s (%s)\n", path, strerror(errno));
		return -1;
	}

	len = write(fd, buf, buf_len);

	close(fd);

	if (len == -1)
		ALOGW("Cannot write to %s (%s)\n", path, strerror(errno));
	else if (len != buf_len)
		ALOGW("Cannot write %d bytes to %s (%d)\n", buf_len, path, len);
	else
		ALOGV("Wrote %d bytes to %s \"%s\"\n", buf_len, path, (char *)buf);

	return len;
}


static void str2int(const char* buf, void *v, int index)
{
	((int*)v)[index] = atoi(buf);
}


static void str2float(const char* buf, void *v, int index)
{
	((float*)v)[index] = strtof(buf, NULL);
}


static void str2uint64(const char* buf, void *v, int index)
{
	((uint64_t*)v)[index] = atoll(buf);
}

int sysfs_read_num_array(const char path[PATH_MAX], void *values, int *size, int max_size,
		void (*str2num)(const char* buf, void *v, int index))
{
	char buf[100];
	int index;
	char *saveprt;
	char *token;
	const char *delimiters = " \t,";

	int len = sysfs_read_str(path, buf, sizeof(buf));
	
	if (len <= 0) {
		ALOGW("Cannot read number from %s (%s)\n", path,
		      strerror(errno));
		return -1;
	}

	index = 0;
	
	token = strtok_r(buf, delimiters, &saveprt);
	while (token != NULL) {
		str2num(token, values, index);

		if (index++ >= max_size)
			break;
		
		token = strtok_r(NULL, delimiters, &saveprt);
	}
	*size = index;

	return 0;
}

int sysfs_read_int_array(const char path[PATH_MAX], int *values, int *size, int max_size)
{
	return sysfs_read_num_array(path, values, size, max_size, str2int);
}


int sysfs_read_num(const char path[PATH_MAX], void *v,
		void (*str2num)(const char* buf, void *v, int index))
{
	char buf[20];
	int len = sysfs_read_str(path, buf, sizeof(buf));

	if (len <= 0) {
		ALOGW("Cannot read number from %s (%s)\n", path,
		      strerror(errno));
		return -1;
	}

	str2num(buf, v, 0);
	return 0;
}

int sysfs_read_int(const char path[PATH_MAX], int *value)
{
	return sysfs_read_num(path, value, str2int);
}


int sysfs_read_float(const char path[PATH_MAX], float *value)
{
	return sysfs_read_num(path, value, str2float);
}


int sysfs_read_uint64(const char path[PATH_MAX], uint64_t *value)
{
	return sysfs_read_num(path, value, str2uint64);
}


int sysfs_write_int(const char path[PATH_MAX], int value)
{
	char buf[20];
	int len = snprintf(buf, sizeof(buf), "%d", value);

	if (len <= 0) {
		ALOGE("Unexpected condition in sysfs_write_int\n");
		return -1;
	}

	return sysfs_write(path, buf, len);
}

int sysfs_write_float(const char path[PATH_MAX], float value)
{
	char buf[20];
	int len = snprintf(buf, sizeof(buf), "%g", value);

	if (len <= 0) {
		ALOGE("Unexpected condition in sysfs_write_float\n");
		return -1;
	}

	return sysfs_write(path, buf, len);
}

int sysfs_write_str(const char path[PATH_MAX], const char *str)
{
	if (!str || !str[0])
		return -1;

	return sysfs_write(path, str, strlen(str));
}

int sysfs_write_str_format_path(const char *str, const char *path_fmt, ...)
{
	char path[PATH_MAX];
	va_list args;
	va_start(args, path_fmt);
	vsnprintf(path, sizeof(path), path_fmt, args);
	va_end(args);
	return sysfs_write_str(path, str);
}

int sysfs_read_float_format_path(float *value, const char *path_fmt, ...)
{
	char path[PATH_MAX];
	va_list args;
	va_start(args, path_fmt);
	vsnprintf(path, sizeof(path), path_fmt, args);
	va_end(args);
	return sysfs_read_float(path, value);
}

int sysfs_read_int_array_format_path(int *array, int *size, int max_size, const char *path_fmt, ...)
{
	char path[PATH_MAX];
	va_list args;
	va_start(args, path_fmt);
	vsnprintf(path, sizeof(path), path_fmt, args);
	va_end(args);
	return sysfs_read_int_array(path, array, size, max_size);
}

int sysfs_read_str(const char path[PATH_MAX], char *buf, int buf_len)
{
	int len;

	if (!buf || buf_len < 1)
		return -1;

	len = sysfs_read(path, buf, buf_len);

	if (len == -1) {
		ALOGW("Cannot read string from %s (%s)\n", path,
		      strerror(errno));
		return -1;
	}

	buf[len == 0 ? 0 : len - 1] = '\0';

	ALOGV("Read %s from %s\n", buf, path);

	return len;
}


int64_t get_timestamp (clockid_t clock_id)
{
	struct timespec ts = {0};

	if (!clock_gettime(clock_id, &ts))
		return 1000000000LL * ts.tv_sec + ts.tv_nsec;
	else	/* in this case errno is set appropriately */
		return -1;
}

int64_t get_timestamp_realtime (void)
{
	return get_timestamp(CLOCK_REALTIME);
}

int64_t get_timestamp_boot (void)
{
	return get_timestamp(CLOCK_BOOTTIME);
}

int64_t get_timestamp_thread (void)
{
	return get_timestamp(CLOCK_THREAD_CPUTIME_ID);
}

int64_t get_timestamp_monotonic (void)
{
	return get_timestamp(CLOCK_MONOTONIC);
}

void set_timestamp (struct timespec *out, int64_t target_ns)
{
	out->tv_sec  = target_ns / 1000000000LL;
	out->tv_nsec = target_ns % 1000000000LL;
}

