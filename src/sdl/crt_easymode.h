/* Software crt-easymode pass for the Falcon's SC1224 RGB/TV presentation. */
#ifndef HATARI_CRT_EASYMODE_H
#define HATARI_CRT_EASYMODE_H

#include <stdint.h>

void CRT_EasyMode_Process(uint32_t *dst, const uint32_t *src, int width,
                          int height, int src_pitch);

#endif
