#ifndef TRIMUI_UKEY_INCLUDE
#define TRIMUI_UKEY_INCLUDE

#define TRIMUI_MAX_XPAD   (4)
#define ARRAY_SIZE(x)    sizeof(x) / sizeof(x[0])

int trimui_setup_xpad(int index);
int trimui_close_xpad(int index);
void trimui_report_event(int uinput_fd, unsigned int type, unsigned int keycode, unsigned int value);
void trimui_vkey(int port, int keycode, int value);
void trimui_vaxis(int port, int axis, int value);
void trimui_vsw(int port, int sw, int value);

#endif //TRIMUI_UKEY_INCLUDE
