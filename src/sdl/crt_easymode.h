/* Software crt-easymode pass for the Falcon's SC1224 RGB/TV presentation. */
#ifndef HATARI_CRT_EASYMODE_H
#define HATARI_CRT_EASYMODE_H

#include <stdint.h>

void CRT_EasyMode_Process(uint32_t *dst, int dst_width, int dst_height,
                          const uint32_t *src, int src_width, int src_height,
                          int src_pitch, int zoom_x, int zoom_y,
                          int scans_per_line);

#endif
