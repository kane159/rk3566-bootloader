//#ifndef _GNU_SOURCE
//#define _GNU_SOURCE
//#endif
 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <sys/ioctl.h>
#include <sys/time.h>
//new code
#include <pthread.h>
#include <linux/ff.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "ukey.h"


#if 0
#include <android/log.h>

#define TAG "rxiovjoy"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG,__VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG,__VA_ARGS__)
#else
// #define LOGI printf
// #define LOGD printf
// #define LOGE printf
#define LOGI(fmt...)
#define LOGD(fmt...)
#define LOGE(fmt...)
#endif

#define RXIO_NAME "Microsoft X-Box 360 pad"
#define RXIO_VID (0x045e)
#define RXIO_PID (0x028e)
#define RXIO_VER (0x0114)

static struct uinput_setup s_setup[TRIMUI_MAX_XPAD];
static const char *s_name[TRIMUI_MAX_XPAD] =
{
#if 0
	"Microsoft X-Box 360 pad",
	"Microsoft X-Box 360 pad",
	"Microsoft X-Box 360 pad",
	"Microsoft X-Box 360 pad",
#else
	"MIYOO Player1",
	"MIYOO Player2",
	"MIYOO Player3",
	"MIYOO Player4",
#endif
};
static int s_fd[TRIMUI_MAX_XPAD] = {-1, -1, -1, -1};

//new code
static pthread_t ff_thread[TRIMUI_MAX_XPAD];
static int ff_thread_running[TRIMUI_MAX_XPAD] = {0};

static void setup_abs(int fd, unsigned chan, int min, int max)
{
	if (ioctl(fd, UI_SET_ABSBIT, chan))
		perror("UI_SET_ABSBIT");

	struct uinput_abs_setup s =
	{
		.code = chan,
		.absinfo = {.minimum = min, .maximum = max},
	};

	if (ioctl(fd, UI_ABS_SETUP, &s))
		perror("UI_ABS_SETUP");
}

int trimui_setup_xpad(int index)
{
	if (index < 0 || index >= TRIMUI_MAX_XPAD)
		return -1;

	int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if (fd < 0) {
		perror("open /dev/uinput");
		return 1;
	}

	ioctl(fd, UI_SET_EVBIT, EV_KEY);
	ioctl(fd, UI_SET_EVBIT, EV_ABS);
	ioctl(fd, UI_SET_EVBIT, EV_SW);
    ioctl(fd, UI_SET_EVBIT, EV_FF); // 支援震動
    ioctl(fd, UI_SET_FFBIT, FF_RUMBLE);


	ioctl(fd, UI_SET_KEYBIT, BTN_A);
	ioctl(fd, UI_SET_KEYBIT, BTN_B);
	ioctl(fd, UI_SET_KEYBIT, BTN_X);
	ioctl(fd, UI_SET_KEYBIT, BTN_Y);
	ioctl(fd, UI_SET_KEYBIT, BTN_TL);
	ioctl(fd, UI_SET_KEYBIT, BTN_TR);
	ioctl(fd, UI_SET_KEYBIT, BTN_START);
	ioctl(fd, UI_SET_KEYBIT, BTN_SELECT);
	ioctl(fd, UI_SET_KEYBIT, BTN_MODE);
	ioctl(fd, UI_SET_KEYBIT, BTN_THUMBL);
	ioctl(fd, UI_SET_KEYBIT, BTN_THUMBR);
	ioctl(fd, UI_SET_SWBIT, SW_TABLET_MODE);

	setup_abs(fd, ABS_HAT0X, -1, 1);
	setup_abs(fd, ABS_HAT0Y, -1, 1);
	setup_abs(fd, ABS_X, -0x7FFF, 0x7FFF);
	setup_abs(fd, ABS_Y, -0x7FFF, 0x7FFF);
	setup_abs(fd, ABS_Z, 0, 0xFF);
	setup_abs(fd, ABS_RX, -0x7FFF, 0x7FFF);
	setup_abs(fd, ABS_RY, -0x7FFF, 0x7FFF);
	setup_abs(fd, ABS_RZ, 0, 0xFF);

	strcpy(s_setup[index].name, s_name[index]);
	s_setup[index].id.bustype = BUS_USB;
	s_setup[index].id.vendor = RXIO_VID;
	s_setup[index].id.product = RXIO_PID;
	s_setup[index].id.version = RXIO_VER;

	

	if (ioctl(fd, UI_DEV_SETUP, &(s_setup[index]))) {
		perror("UI_DEV_SETUP");
		return 1;
	}

	if (ioctl(fd, UI_DEV_CREATE)) {
		perror("UI_DEV_CREATE");
		return 1;
	}
    
//new code
    ff_thread_running[index] = 1;
    pthread_create(&ff_thread[index], NULL, ff_handler_thread, (void*)(intptr_t)index);
	LOGD("register:%s\n", s_setup[index].name);
	s_fd[index] = fd;
	return 0;
}

int trimui_close_xpad(int index)
{
	if (index < 0 || index >= TRIMUI_MAX_XPAD || s_fd[index] <= 0)
		return -1;

	if (ioctl(s_fd[index], UI_DEV_DESTROY))
	{
		printf("UI_DEV_DESTROY");
		return 1;
	}
    //new code
    ff_thread_running[index] = 0;
    pthread_join(ff_thread[index], NULL);
	close(s_fd[index]);
	s_fd[index] = -1;
    return 0;
}

void trimui_report_event(int uinput_fd, unsigned int type, unsigned int keycode, unsigned int value)
{
	struct input_event key_event;
	int ret;

	memset(&key_event, 0, sizeof(struct input_event));

	gettimeofday(&key_event.time, NULL);
	key_event.type = type;
	key_event.code = keycode;
	key_event.value = value;
	ret = write(uinput_fd, &key_event, sizeof(struct input_event));
	if (ret < 0)
	{
		LOGE("%s:%d\n", __func__, __LINE__);
		return; // ret;//error process.
	}

	gettimeofday(&key_event.time, NULL);
	key_event.type = EV_SYN;
	key_event.code = SYN_REPORT;
	key_event.value = 0; // event status sync
	ret = write(uinput_fd, &key_event, sizeof(struct input_event));
	if (ret < 0)
	{
		LOGE("%s:%d\n", __func__, __LINE__);
		return; // ret;//error process.
	}

	// return 0;
}

void trimui_vkey(int port, int keycode, int value)
{
    LOGE("trimui_vkey[%d]:%d=%d\n", port, keycode, value);
	trimui_report_event(s_fd[port], EV_KEY, keycode, value);
}

void trimui_vaxis(int port, int axis, int value)
{
    LOGE("trimui_vaxis[%d]:%d=%d\n", port, axis, value);
	trimui_report_event(s_fd[port], EV_ABS, axis, value);
}
void trimui_vsw(int port, int sw, int value)
{
    LOGE("trimui_vsw[%d]:%d=%d\n", port, sw, value);
	trimui_report_event(s_fd[port], EV_SW, sw, value);	
}

static void* ff_handler_thread(void* arg) {
	int index = (intptr_t)arg;
	char dev_path[32];
	snprintf(dev_path, sizeof(dev_path), "/dev/input/event%d", index); // 根據你的系統可能要調整

	int fd = open(dev_path, O_RDONLY);
	if (fd < 0) {
		perror("open event device for FF");
		return NULL;
	}

	struct input_event ie;
	while (ff_thread_running[index]) {
		if (read(fd, &ie, sizeof(ie)) != sizeof(ie))
			continue;

		if (ie.type == EV_FF) {
			struct ff_effect effect;
			if (ioctl(fd, EVIOCGFF, &effect) < 0)
				continue;

			if (effect.type == FF_RUMBLE) {
				LOGI("FF_RUMBLE detected: strong=%u, duration=%ums\n",
					effect.u.rumble.strong_magnitude,
					effect.replay.length);

				// 震動開始
				int gpio = open("/sys/class/gpio/gpio20/value", O_WRONLY);
				if (gpio >= 0) {
					write(gpio, "1", 1);
					usleep(effect.replay.length * 1000); // duration is in ms
					write(gpio, "0", 1);
					close(gpio);
				}
			}
		}
	}
	close(fd);
	return NULL;
}
