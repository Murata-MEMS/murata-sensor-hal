
CC := $(CROSS_COMPILE)gcc
INCLUDE := -Iandroid/include

CFLAGS += $(INCLUDE) -DLOG_TAG=\"SensorHAL\" -DPLTF_LINUX_ENABLED -Wall -fPIC -Werror \
		-Wno-unused-variable -Wno-unused-parameter -Wno-unused-value -Wno-unused-function -Wno-unused-label

LDLIBS = -lpthread

SRC_FILES := \
		SensorHAL.c \
		sensor_device.c \
		log.c \
		utils.c

OBJS=$(subst .c,.o,$(SRC_FILES))

all: SensorHAL

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

SensorHAL: $(OBJS)
	$(CXX) $(LDFLAGS) -shared $(OBJS) $(LDLIBS) -o SensorHAL.so

clean:
	rm -f $(OBJS) SensorHAL.so

