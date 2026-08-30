/*
 * Software crt-hyllian pass for Hatari's portable SDL renderer.
 *
 * The reference shader is Hyllian's CRT shader, distributed by
 * libretro/glsl-shaders.  This source-resolution implementation keeps the
 * characteristic stages that matter for Falcon RGB/TV and VGA presentation:
 * gamma-linearized cubic reconstruction with anti-ringing, an adaptive beam
 * profile, phosphor mask, and output gamma/colour boost.  It deliberately
 * leaves the Videl raster dimensions unchanged; overscan geometry is owned by
 * videl.c and the status bar is owned by screen.c.
 */
#include <math.h>
#include <stdbool.h>

#include "crt_hyllian.h"

#define HYLLIAN_INPUT_GAMMA         2.4f
#define HYLLIAN_OUTPUT_GAMMA        2.2f
#define HYLLIAN_BEAM_MIN_WIDTH      0.72f
#define HYLLIAN_BEAM_MAX_WIDTH      1.00f
#define HYLLIAN_SCANLINE_STRENGTH   0.58f
#define HYLLIAN_COLOR_BOOST         1.25f
#define HYLLIAN_MASK_INTENSITY      0.38f
#define HYLLIAN_SCANLINE_CUTOFF     400
#define HYLLIAN_OUTPUT_LUT_SIZE     4096

static float input_gamma_lut[256];
static float output_gamma_lut[HYLLIAN_OUTPUT_LUT_SIZE];
static bool gamma_luts_ready;

static void init_gamma_luts(void)
{

	int i;

	if (gamma_luts_ready)
		return;

	for (i = 0; i < 256; ++i)
		input_gamma_lut[i] = powf((float)i / 255.0f, HYLLIAN_INPUT_GAMMA);
	for (i = 0; i < HYLLIAN_OUTPUT_LUT_SIZE; ++i)
		output_gamma_lut[i] = powf((float)i /
		                           (HYLLIAN_OUTPUT_LUT_SIZE - 1),
		                           1.0f / HYLLIAN_OUTPUT_GAMMA);

	gamma_luts_ready = true;
}

static float clamp01(float value)
{
	if (value < 0.0f)
		return 0.0f;
	if (value > 1.0f)
		return 1.0f;
	return value;
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

static float gamma_in(uint32_t pixel, int shift)
{
	return input_gamma_lut[(pixel >> shift) & 255];
}

static float gamma_out(float value)
{
	int index = (int)(clamp01(value) * (HYLLIAN_OUTPUT_LUT_SIZE - 1) + 0.5f);

	return output_gamma_lut[index];
}

static float cubic_channel(const uint32_t *src, int width, int height,
	                         int pitch, int x, int y, int shift)
{
	float p0 = gamma_in(pixel_at(src, width, height, pitch, x - 1, y), shift);
	float p1 = gamma_in(pixel_at(src, width, height, pitch, x, y), shift);
	float p2 = gamma_in(pixel_at(src, width, height, pitch, x + 1, y), shift);
	float p3 = gamma_in(pixel_at(src, width, height, pitch, x + 2, y), shift);
	float t = 0.5f;
	float value;
	float lo = fminf(p1, p2);
	float hi = fmaxf(p1, p2);

	/* Catmull-Rom cubic reconstruction, equivalent to Hyllian's default
	 * B=0/C=0.5 Mitchell-Netravali coefficients at this output resolution. */
	value = ((-0.5f * p0 + 1.5f * p1 - 1.5f * p2 + 0.5f * p3) * t +
	         (p0 - 2.5f * p1 + 2.0f * p2 - 0.5f * p3)) * t +
	        (-0.5f * p0 + 0.5f * p2);
	value = value * t + p1;

	/* Anti-ringing clamp to the two central samples. */
	if (value < lo)
		value = lo;
	else if (value > hi)
		value = hi;
	return value;
}

static void cubic_rgb(const uint32_t *src, int width, int height, int pitch,
	                    int x, int y, float *red, float *green, float *blue)
{
	*red = cubic_channel(src, width, height, pitch, x, y, 16);
	*green = cubic_channel(src, width, height, pitch, x, y, 8);
	*blue = cubic_channel(src, width, height, pitch, x, y, 0);
}

void CRT_Hyllian_Process(uint32_t *dst, const uint32_t *src, int width,
	                     int height, int src_pitch)
{
	int x, y;
	const bool scanlines = height < HYLLIAN_SCANLINE_CUTOFF;

	init_gamma_luts();

	for (y = 0; y < height; ++y) {
		for (x = 0; x < width; ++x) {
			float red0, green0, blue0;
			float red1, green1, blue1;
			float pos0, pos1, red, green, blue;
			float lum0, lum1, d0, d1;
			float mask, color_boost;
			int dot;

			cubic_rgb(src, width, height, src_pitch, x, y,
			          &red0, &green0, &blue0);

			if (scanlines) {
				cubic_rgb(src, width, height, src_pitch, x, y + 1,
				          &red1, &green1, &blue1);
				/* Two alternating beam positions approximate the physical
				 * line spread while keeping one source row per Videl line. */
				pos0 = (y & 1) ? 0.75f : 0.25f;
				pos1 = 1.0f - pos0;
				lum0 = HYLLIAN_BEAM_MIN_WIDTH +
				       (HYLLIAN_BEAM_MAX_WIDTH - HYLLIAN_BEAM_MIN_WIDTH) *
				       fmaxf(red0, fmaxf(green0, blue0));
				lum1 = HYLLIAN_BEAM_MIN_WIDTH +
				       (HYLLIAN_BEAM_MAX_WIDTH - HYLLIAN_BEAM_MIN_WIDTH) *
				       fmaxf(red1, fmaxf(green1, blue1));
				d0 = 4.0f * HYLLIAN_SCANLINE_STRENGTH * pos0 /
				     (lum0 + 0.0000001f);
				d1 = 4.0f * HYLLIAN_SCANLINE_STRENGTH * pos1 /
				     (lum1 + 0.0000001f);
				d0 = expf(-d0 * d0);
				d1 = expf(-d1 * d1);
				red = red0 * d0 + red1 * d1;
				green = green0 * d0 + green1 * d1;
				blue = blue0 * d0 + blue1 * d1;
				color_boost = HYLLIAN_COLOR_BOOST;
			} else {
				red = red0;
				green = green0;
				blue = blue0;
				color_boost = 1.0f;
			}

			/* Hyllian's default layout is an aperture grille.  The
			 * Falcon target uses RGB order, matching the SC1224 input. */
			dot = x % 3;
			mask = 1.0f - HYLLIAN_MASK_INTENSITY;
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

			red = gamma_out(red * color_boost);
			green = gamma_out(green * color_boost);
			blue = gamma_out(blue * color_boost);
			dst[y * width + x] = rgb(red, green, blue);
		}
	}
}
