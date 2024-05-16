
#include <stdbool.h>
#include <hardware/sensors.h>

#define MAX_DEVICES (10)
#define MAX_SENSORS (MAX_DEVICES * 2)
#define MAX_CHANNELS 20
#define MAX_FILTER_FREQ_AVAILABLE 20

struct sensor_data;

struct channel_format {
	float scale;
	bool is_big_endian;
	bool is_signed;
	int bits;
	int storage_bits;
	int shift;
	int repeat;
	int enabled;
	int32_t sensor_type;
	int event_data_index;
	bool is_timestamp;
};

struct device_data {
	char device_name[100];
	pthread_t worker_thread;
	struct sensor_module_data *module_data;
	int iio_device_num;
	int iio_trigger_device_num;
	struct channel_format format[MAX_CHANNELS];
	int max_scan_index;
	int scan_size;
	int accel_active;
	int gyro_active;
	int accel_sensor_num;
	int gyro_sensor_num;
	int must_stop;
};

struct sensor_data {
	int sensor_type;
	char sensor_name[100];
	int device_num;
	int enabled;
	bool is_accel;
	int sampling_frequency_hz;
	int filter_freq_available[MAX_FILTER_FREQ_AVAILABLE];
	int filter_freq_count;
};

struct sensor_module_data {
	struct sensors_poll_device_1 poll_device;

	struct sensor_t sensors_list[MAX_SENSORS];

	struct sensor_data sensor_data[MAX_SENSORS];
	int sensor_count;

	struct device_data device_data[MAX_DEVICES];
	int device_count;

	int read_pipe_fd;
	int write_pipe_fd;
	pthread_mutex_t pipe_mutex;
};

int device_activate(struct sensors_poll_device_t *dev,
				   int handle, int enabled);

int device_set_delay(struct sensors_poll_device_t *dev,
				    int handle, int64_t sampling_period_ns);


int device_batch (__attribute__((unused)) struct sensors_poll_device_1* dev,
		  int sensor_handle, __attribute__((unused)) int flags,
		  int64_t sampling_period_ns,
		  __attribute__((unused)) int64_t max_report_latency_ns);

int device_flush (__attribute__((unused)) struct sensors_poll_device_1* dev,
		  int handle);

int device_poll(struct sensors_poll_device_t *dev,
			       sensors_event_t* events, int count);

int find_sensors(struct sensor_module_data *data);

int device_inject_sensor_data(struct sensors_poll_device_1 *dev, const sensors_event_t *data);

