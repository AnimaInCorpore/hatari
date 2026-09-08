/* Software crt-hyllian pass for the Falcon's CRT presentation. */
#ifndef HATARI_CRT_HYLLIAN_H
#define HATARI_CRT_HYLLIAN_H

#include <stdint.h>

void CRT_Hyllian_Process(uint32_t *dst, int dst_width, int dst_height,
                         const uint32_t *src, int src_width, int src_height,
                         int src_pitch);

#endif
