
#include <asm/types.h>
#include <stdio.h>
#include <linux/limits.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <dlfcn.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <sys/wait.h>
#include <stdbool.h>
#include <hardware/sensors.h>
#include <stdarg.h>
#include <math.h>


#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define log(...) printf(__VA_ARGS__)
#define error(...) { fprintf(stderr, KRED); fprintf(stderr, __VA_ARGS__); fprintf(stderr, KNRM); }

static struct sensors_module_t *hmi;
static struct hw_device_t *dev;
struct sensors_poll_device_t *poll_dev;

#define MAX_SENSORS 10

static int64_t timespec_to_timestamp(struct timespec *t)
{
	return t->tv_nsec + t->tv_sec * 1000000000LL;
}

static int64_t event_to_timestamp(struct sensors_event_t *event)
{
	return event->timestamp;
}

static int64_t timestamp()
{
	struct timespec t;
	clock_gettime(CLOCK_REALTIME, &t);
	return timespec_to_timestamp(&t);
}

static void print_record(const char *operation, int sensor_handle, const char *event_type, double x, double y, double z, int64_t event_timestamp)
{
	int64_t record_timestamp = timestamp();

	printf("%lld\t%lld\t%s\t%d\t%s\t%lf\t%lf\t%lf\n",
			record_timestamp, event_timestamp, operation, sensor_handle, event_type, x, y, z);
}

static void dump_event(struct sensors_event_t *e)
{
	switch (e->type) {
	case SENSOR_TYPE_ACCELEROMETER:
		print_record("event", e->sensor, "ACCELEROMETER", e->acceleration.x, e->acceleration.y, e->acceleration.z, event_to_timestamp(e));
		break;
	case SENSOR_TYPE_GYROSCOPE:
		print_record("event", e->sensor, "GYROSCOPE", e->gyro.x, e->gyro.y, e->gyro.z, event_to_timestamp(e));
		break;
	case SENSOR_TYPE_META_DATA:
		if (e->meta_data.what == META_DATA_FLUSH_COMPLETE)
			print_record("event", e->sensor, "FLUSH_COMPLETE", NAN, NAN, NAN, event_to_timestamp(e));
		break;
	default:
	{
		char s[100];
		snprintf(s, sizeof(s), "Unknownw event type %d", e->type);
		print_record("event", e->sensor, s, NAN, NAN, NAN, event_to_timestamp(e));
	}
		break;
	}
}

struct event_thread_data {
	sensors_poll_device_1_t *device;
	FILE *out;
};

static void *event_thread(void *arg)
{
	struct event_thread_data *data = (struct event_thread_data *)arg;
	sensors_poll_device_1_t *device = data->device;
	sensors_event_t event;
	int ret;

	do {

		ret = device->poll(&device->v0, &event, 1);
		if (ret < 0) {
			printf("data poll error %d!\n", ret);
		} else {
			dump_event(&event);
		}
	} while (ret >= 0);

	pthread_exit((void *)ret);
}

static void dump_sensor(const struct sensor_t *s)
{
	log("Name %s\n", s->name);
	log("\tVendor %s\n", s->vendor);
	log("\tHandle %d\n", s->handle);
	log("\tType %d\n", s->type);
	log("\tVersion %d\n", s->version);
	log("\tMax Range %f\n", s->maxRange);
	log("\tResolution %f\n", s->resolution);
	log("\tPower %f\n", s->power);
	log("\tMin Delay %d\n", s->minDelay);
	log("\tFIFO Reserved Event %d\n", s->fifoReservedEventCount);
	log("\tFIFO Max Event %d\n", s->fifoMaxEventCount);
}

static struct sensors_poll_device_1 *open_hal(char *path)
{
	int err;
	void *hal;

	hal = dlopen(path, RTLD_NOW);
	if (!hal) {
		error("ERROR: unable to load HAL %s: %s\n", path,
			dlerror());
		return NULL;
	}

	hmi = (struct sensors_module_t *)dlsym(hal, HAL_MODULE_INFO_SYM_AS_STR);

	if (!hmi) {
		error("ERROR: unable to find %s entry point in HAL\n",
			HAL_MODULE_INFO_SYM_AS_STR);
		return NULL;
	}

	log("HAL loaded:\n\tname %s\n\tvendor %s\n\tversion %d.%d\n\tid %s\n",
	       hmi->common.name, hmi->common.author,
	       hmi->common.version_major, hmi->common.version_minor,
	       hmi->common.id);

	err = hmi->common.methods->open((struct hw_module_t *)hmi,
					SENSORS_HARDWARE_POLL, &dev);
	if (err) {
		error("ERROR: failed to initialize HAL: %d\n", err);
		exit(1);
	}

	return (struct sensors_poll_device_1 *)dev;
}

static void activate(struct sensors_poll_device_1 *dev, int handle, int enabled)
{
	dev->activate(&dev->v0, handle, enabled);
	print_record(__FUNCTION__, handle, enabled ? "enabled" : "disabled", NAN, NAN, NAN, -1);
}

static void set_sampling_period(struct sensors_poll_device_1 *dev, int handle, int64_t period_ns)
{
	char s[100];
	snprintf(s, sizeof(s), "%d", period_ns);
	dev->batch(dev, handle, 0, period_ns, 0);
	print_record(__FUNCTION__, handle, s, NAN, NAN, NAN, -1);
}

static void flush(struct sensors_poll_device_1 *dev, int handle)
{
	dev->flush(dev, handle);
	print_record(__FUNCTION__, handle, "-", NAN, NAN, NAN, -1);
}

int main(int argc, char **argv)
{
	int sindex = 0;
	int num_sample = 10;
	int ret;
	int handle;
	struct sensor_t *sensor = NULL;
	int test_sensor_type[] = {
		SENSOR_TYPE_ACCELEROMETER,
		SENSOR_TYPE_GYROSCOPE,
		-1,
	};
	struct sensors_poll_device_1 *poll_dev;
	int sensor_count;
	struct sensor_t const *sensor_list;
	int i;
	pthread_t tid;
	struct timespec t;
	struct event_thread_data data;

	poll_dev = open_hal(argv[1]);
	if (poll_dev == NULL) {
		error("ERROR: unable to open SensorHAL\n");
		exit(1);
	}

	sensor_count = hmi->get_sensors_list(hmi, &sensor_list);
	if (sensor_count == 0) {
		error("ERROR: no sensors available\n");
		exit(1);
	}

	// Dump sensor list
	for (i = 0; i < sensor_count; i++) {
		dump_sensor(&sensor_list[i]);
	}


	// Start event thread
	data.device = poll_dev;
	data.out = stdout;
	ret = pthread_create(&tid, NULL, event_thread, &data);
	if (ret != 0) {
		error("thread create error [%s]\n", strerror(-ret));
		exit(1);
	}

	// Test sampling frequency when one sensor is activated at a time
	for (int i = 0; i < 2; i++) {
		int64_t period_ns[] = { 100000000L, 50000000L };
		int count[] = { 10, 10 };
		int parts = 2;

		log("Testing sampling frequency changes for %s (handle %d, type %d).\n",
				sensor_list[i].name, sensor_list[i].handle, sensor_list[i].type);

		handle = sensor_list[i].handle;

		for (int j = 0; j < parts; j++) {
			set_sampling_period(poll_dev, handle, period_ns[j]);

			activate(poll_dev, handle, 1);

			usleep(period_ns[j] * (count[j] - 1)/ 1000);
			usleep(period_ns[j] / 2 / 1000);
		}
		activate(poll_dev, handle, 0);

	}

	usleep(1000000);

	// Test sampling frequency when both sensors are activated
	{
		int64_t period_ns[] = { 100000000L, 50000000L };
		int count[] = { 10, 10 };
		int parts = 2;

		log("Testing sampling frequency changes for %s (handle %d, type %d) and %s (handle %d, type %d).\n",
				sensor_list[0].name, sensor_list[0].handle, sensor_list[0].type,
				sensor_list[1].name, sensor_list[1].handle, sensor_list[1].type);

		int handle1 = sensor_list[0].handle;
		int handle2 = sensor_list[1].handle;

		for (int j = 0; j < parts; j++) {
			set_sampling_period(poll_dev, handle1, period_ns[j]);
			set_sampling_period(poll_dev, handle2, period_ns[j]);

			activate(poll_dev, handle1, 1);
			activate(poll_dev, handle2, 1);

			usleep(period_ns[j] * (count[j] - 1) / 1000);
			usleep(period_ns[j] / 2 / 1000);
		}

		activate(poll_dev, handle1, 0);
		activate(poll_dev, handle2, 0);
	}

	usleep(1000000);

	// Test flush event
	{
		int64_t period_ns = 100000000L;

		log("Testing flush event for %s (handle %d, type %d) and %s (handle %d, type %d).\n",
				sensor_list[0].name, sensor_list[0].handle, sensor_list[0].type,
				sensor_list[1].name, sensor_list[1].handle, sensor_list[1].type);

		int handle1 = sensor_list[0].handle;
		int handle2 = sensor_list[1].handle;


		set_sampling_period(poll_dev, handle1, period_ns);
		set_sampling_period(poll_dev, handle2, period_ns);

		activate(poll_dev, handle1, 1);
		activate(poll_dev, handle2, 1);

		usleep(period_ns * 10 / 1000);

		flush(poll_dev, handle1);
		flush(poll_dev, handle2);

		usleep(period_ns / 1000);

		flush(poll_dev, handle1);

		usleep(period_ns * 10 / 1000);

		activate(poll_dev, handle1, 0);
		activate(poll_dev, handle2, 0);

	}

	return 0;
}
