/*
 * Software port of crt-easymode for Hatari's portable SDL renderer.
 *
 * The reference shader is CRT Shader by EasyMode, distributed under the GPL
 * in libretro/slang-shaders.  This implementation keeps its useful low-res
 * CRT stages: input gamma/dilation, sharp horizontal reconstruction, adaptive
 * scanline beam, RGB aperture grille, output gamma and brightness boost.
 *
 * Hatari uploads a complete logical Videl raster to SDL, so this pass runs on
 * that raster before SDL scales it.  The status bar is copied by screen.c
 * after this function returns and is therefore never part of the shader.
 */
#include <math.h>

#include "crt_easymode.h"

#define EASYMODE_GAMMA_INPUT        2.0f
#define EASYMODE_GAMMA_OUTPUT       1.8f
#define EASYMODE_DILATION           1.0f
#define EASYMODE_SHARPNESS_H        0.5f
#define EASYMODE_SHARPNESS_V        1.0f
#define EASYMODE_MASK_STRENGTH     0.30f
#define EASYMODE_MASK_DOT_WIDTH     1
#define EASYMODE_MASK_DOT_HEIGHT    1
#define EASYMODE_MASK_STAGGER        0
#define EASYMODE_SCANLINE_STRENGTH  1.0f
#define EASYMODE_BEAM_WIDTH_MIN     1.5f
#define EASYMODE_BEAM_WIDTH_MAX     1.5f
#define EASYMODE_BRIGHT_MIN          0.35f
#define EASYMODE_BRIGHT_MAX          0.65f
#define EASYMODE_BRIGHT_BOOST        1.2f
#define EASYMODE_SCANLINE_CUTOFF      400

static float clamp01(float value)
{
	if (value < 0.0f)
		return 0.0f;
	if (value > 1.0f)
		return 1.0f;
	return value;
}

static float channel(uint32_t pixel, int shift)
{
	return (float)((pixel >> shift) & 255) / 255.0f;
}

static uint32_t rgb(float red, float green, float blue)
{
	return ((unsigned)(clamp01(red) * 255.0f + 0.5f) << 16) |
	       ((unsigned)(clamp01(green) * 255.0f + 0.5f) << 8) |
	       (unsigned)(clamp01(blue) * 255.0f + 0.5f);
}

static uint32_t pixel_at(const uint32_t *src, int width, int height,
	                       int pitch, int x, int y)
{
	const uint32_t *line;

	if (x < 0)
		x = 0;
	else if (x >= width)
		x = width - 1;
	if (y < 0)
		y = 0;
	else if (y >= height)
		y = height - 1;

	line = (const uint32_t *)((const unsigned char *)src + y * pitch);
	return line[x];
}

static float dilate(float value)
{
	/* EasyMode's dilation is col * mix(1, col, DILATION). */
	return value * (1.0f + (value - 1.0f) * EASYMODE_DILATION);
}

static float input_gamma(float value)
{
	return powf(clamp01(value),
	            EASYMODE_GAMMA_INPUT / (EASYMODE_DILATION + 1.0f));
}

static float filtered_channel(const uint32_t *src, int width, int height,
	                            int pitch, int x, int y, int shift)
{
	float center = input_gamma(dilate(channel(pixel_at(src, width, height,
	                                                   pitch, x, y), shift)));
	float left = input_gamma(dilate(channel(pixel_at(src, width, height,
	                                                 pitch, x - 1, y), shift)));
	float right = input_gamma(dilate(channel(pixel_at(src, width, height,
	                                                  pitch, x + 1, y), shift)));
	float vertical;
	float blur = 0.16f * (1.0f - EASYMODE_SHARPNESS_H);

	/* A small three-tap reconstruction is the inexpensive equivalent of the
	 * EasyMode Lanczos path at a source-resolution output. */
	center = center * (1.0f - 2.0f * blur) + (left + right) * blur;

	if (EASYMODE_SHARPNESS_V < 1.0f) {
		float above = input_gamma(dilate(channel(pixel_at(src, width, height,
										 pitch, x, y - 1), shift)));
		float below = input_gamma(dilate(channel(pixel_at(src, width, height,
										 pitch, x, y + 1), shift)));
		vertical = 0.04f * (1.0f - EASYMODE_SHARPNESS_V);
		center = center * (1.0f - 2.0f * vertical) +
		         (above + below) * vertical;
	}

	return clamp01(center);
}

void CRT_EasyMode_Process(uint32_t *dst, const uint32_t *src, int width,
	                      int height, int src_pitch)
{
	int x, y;

	for (y = 0; y < height; ++y) {
		float scan_phase = 0.5f + 0.5f * cosf((float)y * 3.141592653589f);
		float scan_beam = EASYMODE_BEAM_WIDTH_MIN;

		if (scan_beam > EASYMODE_BEAM_WIDTH_MAX)
			scan_beam = EASYMODE_BEAM_WIDTH_MAX;

		for (x = 0; x < width; ++x) {
			float red = filtered_channel(src, width, height, src_pitch, x, y, 16);
			float green = filtered_channel(src, width, height, src_pitch, x, y, 8);
			float blue = filtered_channel(src, width, height, src_pitch, x, y, 0);
			float luma = red * 0.2126f + green * 0.7152f + blue * 0.0722f;
			float bright = fmaxf(red, fmaxf(green, blue));
			float scan_bright, scan_weight, mask;
			int dot;

			bright = (bright + luma) * 0.5f;
			scan_bright = bright;
			if (scan_bright < EASYMODE_BRIGHT_MIN)
				scan_bright = EASYMODE_BRIGHT_MIN;
			else if (scan_bright > EASYMODE_BRIGHT_MAX)
				scan_bright = EASYMODE_BRIGHT_MAX;

			scan_weight = 1.0f -
			              powf(scan_phase, scan_beam) *
			              EASYMODE_SCANLINE_STRENGTH;
			if (height >= EASYMODE_SCANLINE_CUTOFF)
				scan_weight = 1.0f;
			red = red * scan_weight + red * scan_bright * (1.0f - scan_weight);
			green = green * scan_weight + green * scan_bright * (1.0f - scan_weight);
			blue = blue * scan_weight + blue * scan_bright * (1.0f - scan_weight);

			dot = (x / EASYMODE_MASK_DOT_WIDTH +
			       ((y / EASYMODE_MASK_DOT_HEIGHT) & 1) *
			       EASYMODE_MASK_STAGGER) % 3;
			mask = 1.0f - EASYMODE_MASK_STRENGTH;
			if (dot == 0) {
				green *= mask;
				blue *= mask;
			} else if (dot == 1) {
				red *= mask;
				blue *= mask;
			} else {
				red *= mask;
				green *= mask;
			}

			/* EasyMode's output transfer and SC1224-friendly brightness boost. */
			red = powf(clamp01(red), 1.0f / EASYMODE_GAMMA_OUTPUT) *
			      EASYMODE_BRIGHT_BOOST;
			green = powf(clamp01(green), 1.0f / EASYMODE_GAMMA_OUTPUT) *
			        EASYMODE_BRIGHT_BOOST;
			blue = powf(clamp01(blue), 1.0f / EASYMODE_GAMMA_OUTPUT) *
			       EASYMODE_BRIGHT_BOOST;
			dst[y * width + x] = rgb(red, green, blue);
		}
	}
}
