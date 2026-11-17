/*
 * Software port of crt-easymode for Hatari's portable SDL renderer.
 *
 * The reference shader is CRT Shader by EasyMode, distributed under the GPL
 * in libretro/slang-shaders.  This implementation keeps its useful low-res
 * CRT stages: input gamma/dilation, sharp horizontal reconstruction, adaptive
 * scanline beam, RGB aperture grille, output gamma and brightness boost.
 *
 * Like the Hyllian pass it runs at the output resolution: the scanlines are
 * gaps between the video lines and the mask is one output pixel wide.  The
 * emulated raster is read in video pixels and lines (a raster the converter
 * zoomed with nScreenZoomX/Y is read one zoomed pixel per video pixel), and
 * the caller says how the monitor shows a line (one or two scanlines, or
 * interlaced without gaps).  The status bar is drawn by screen.c.
 *
 * Gamma and dilation are tables, the horizontal reconstruction runs once per
 * video line, and the scanline profile once per output row.
 */
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "crt_easymode.h"

#define EASYMODE_GAMMA_INPUT        2.0f
#define EASYMODE_GAMMA_OUTPUT       1.8f
#define EASYMODE_DILATION           1.0f
#define EASYMODE_SHARPNESS_H        0.5f
#define EASYMODE_MASK_STRENGTH      0.30f
#define EASYMODE_SCANLINE_STRENGTH  1.0f
#define EASYMODE_BEAM_WIDTH         1.5f
#define EASYMODE_BRIGHT_MIN         0.35f
#define EASYMODE_BRIGHT_MAX         0.65f
#define EASYMODE_BRIGHT_BOOST       1.2f
#define EASYMODE_OUTPUT_LUT_SIZE    4096

static float input_lut[256];
static uint8_t output_lut[EASYMODE_OUTPUT_LUT_SIZE];
static bool luts_ready;

/* Geometry, rebuilt when a dimension changes. */
static struct {
	int dst_width, dst_height;
	int src_width, src_height;
	int zoom_x, zoom_y;
	int scans_per_line;
	int columns, lines;
	int *taps;          /* left source column per output column */
	float *frac;        /* sharpened position between it and the next */
	int *rows;          /* video line per output row */
	float *row_weight;  /* scanline weight per output row */
	float *line_rgb;    /* reconstructed lines: lines x dst_width x 3 */
	float *filtered;    /* one filtered video line: columns x 3 */
} geo;

static void init_luts(void)
{
	int i;

	if (luts_ready)
		return;

	/* EasyMode's dilation col * mix(1, col, DILATION), then its input
	 * gamma, which the dilation reduces. */
	for (i = 0; i < 256; ++i) {
		const float value = (float)i / 255.0f;
		const float dilated = value * (1.0f + (value - 1.0f) * EASYMODE_DILATION);

		input_lut[i] = powf(dilated, EASYMODE_GAMMA_INPUT /
		                    (EASYMODE_DILATION + 1.0f));
	}
	/* Output transfer and the SC1224-friendly brightness boost. */
	for (i = 0; i < EASYMODE_OUTPUT_LUT_SIZE; ++i) {
		float value = powf((float)i / (EASYMODE_OUTPUT_LUT_SIZE - 1),
		                   1.0f / EASYMODE_GAMMA_OUTPUT) * EASYMODE_BRIGHT_BOOST;

		output_lut[i] = (uint8_t)((value > 1.0f ? 1.0f : value) * 255.0f + 0.5f);
	}

	luts_ready = true;
}

static inline unsigned output(float value)
{
	int index = (int)(value * (EASYMODE_OUTPUT_LUT_SIZE - 1) + 0.5f);

	if (index < 0)
		index = 0;
	else if (index >= EASYMODE_OUTPUT_LUT_SIZE)
		index = EASYMODE_OUTPUT_LUT_SIZE - 1;
	return output_lut[index];
}

static int clampi(int value, int low, int high)
{
	return value < low ? low : value > high ? high : value;
}

static void free_geometry(void)
{
	free(geo.taps);
	free(geo.frac);
	free(geo.rows);
	free(geo.row_weight);
	free(geo.line_rgb);
	free(geo.filtered);
	geo.taps = NULL;
	geo.frac = NULL;
	geo.rows = NULL;
	geo.row_weight = NULL;
	geo.line_rgb = NULL;
	geo.filtered = NULL;
	geo.dst_width = 0;
}

static bool update_geometry(int dst_width, int dst_height, int src_width,
	                        int src_height, int zoom_x, int zoom_y,
	                        int scans_per_line)
{
	const int repeat = scans_per_line > 0 ? scans_per_line : 1;
	int scans, x, y;

	if (geo.taps && geo.dst_width == dst_width && geo.dst_height == dst_height &&
	    geo.src_width == src_width && geo.src_height == src_height &&
	    geo.zoom_x == zoom_x && geo.zoom_y == zoom_y &&
	    geo.scans_per_line == scans_per_line)
		return true;

	free_geometry();
	geo.columns = src_width / zoom_x;
	geo.lines = src_height / zoom_y;
	if (geo.columns < 1 || geo.lines < 1)
		return false;

	geo.taps = malloc(sizeof(int) * dst_width);
	geo.frac = malloc(sizeof(float) * dst_width);
	geo.rows = malloc(sizeof(int) * dst_height);
	geo.row_weight = malloc(sizeof(float) * dst_height);
	geo.line_rgb = malloc(sizeof(float) * 3 * dst_width * geo.lines);
	geo.filtered = malloc(sizeof(float) * 3 * geo.columns);
	if (!geo.taps || !geo.frac || !geo.rows || !geo.row_weight ||
	    !geo.line_rgb || !geo.filtered) {
		free_geometry();
		return false;
	}

	/* Sharp interpolation between video pixel centres: the transition
	 * is narrowed by the horizontal sharpness. */
	for (x = 0; x < dst_width; ++x) {
		float fx = ((float)x + 0.5f) * geo.columns / dst_width - 0.5f;
		float base = floorf(fx);
		float t = ((fx - base) - 0.5f) / (1.0f - EASYMODE_SHARPNESS_H) + 0.5f;

		geo.taps[x] = clampi((int)base, 0, geo.columns - 1);
		if (geo.taps[x] >= geo.columns - 1)
			t = 0.0f;
		else if ((int)base < 0)
			t = 0.0f;
		geo.frac[x] = t < 0.0f ? 0.0f : t > 1.0f ? 1.0f : t;
	}

	/* EasyMode's scanline: 1 - ((cos(2 pi y) + 1) / 2)^beam * strength,
	 * y in scanlines, dark between scanlines and full at their centres.
	 * Interlaced lines leave no gaps. */
	scans = geo.lines * repeat;
	for (y = 0; y < dst_height; ++y) {
		const float fy = ((float)y + 0.5f) * scans / dst_height;
		const int scan = clampi((int)fy, 0, scans - 1);
		const float phase = 0.5f + 0.5f * cosf(2.0f * 3.14159265f * (fy - 0.5f));

		geo.rows[y] = scan / repeat;
		geo.row_weight[y] = scans_per_line > 0
			? 1.0f - powf(1.0f - phase, EASYMODE_BEAM_WIDTH) * EASYMODE_SCANLINE_STRENGTH
			: 1.0f;
	}

	geo.dst_width = dst_width;
	geo.dst_height = dst_height;
	geo.src_width = src_width;
	geo.src_height = src_height;
	geo.zoom_x = zoom_x;
	geo.zoom_y = zoom_y;
	geo.scans_per_line = scans_per_line;
	return true;
}

/* One video line: the small three-tap blur of the horizontal sharpness in
 * linear light, then interpolated to every output column. */
static void reconstruct_line(float *out, const uint32_t *line, int zoom_x)
{
	const float blur = 0.16f * (1.0f - EASYMODE_SHARPNESS_H);
	float *filtered = geo.filtered;
	int x, c, shift;

	for (x = 0; x < geo.columns; ++x) {
		const uint32_t left = line[(x > 0 ? x - 1 : 0) * zoom_x];
		const uint32_t center = line[x * zoom_x];
		const uint32_t right = line[(x < geo.columns - 1 ? x + 1 : x) * zoom_x];

		for (c = 0, shift = 16; c < 3; ++c, shift -= 8) {
			const float value = input_lut[(center >> shift) & 255] * (1.0f - 2.0f * blur) +
			                    (input_lut[(left >> shift) & 255] +
			                     input_lut[(right >> shift) & 255]) * blur;

			filtered[x * 3 + c] = value;
		}
	}

	for (x = 0; x < geo.dst_width; ++x) {
		const float *a = filtered + geo.taps[x] * 3;
		const float *b = a + (geo.taps[x] < geo.columns - 1 ? 3 : 0);
		const float t = geo.frac[x];

		*out++ = a[0] + (b[0] - a[0]) * t;
		*out++ = a[1] + (b[1] - a[1]) * t;
		*out++ = a[2] + (b[2] - a[2]) * t;
	}
}

void CRT_EasyMode_Process(uint32_t *dst, int dst_width, int dst_height,
	                      const uint32_t *src, int src_width, int src_height,
	                      int src_pitch, int zoom_x, int zoom_y,
	                      int scans_per_line)
{
	/* RGB aperture grille: the lit channel at full strength. */
	const float dim = 1.0f - EASYMODE_MASK_STRENGTH;
	const float mask[3][3] = {
		{ 1.0f, dim, dim }, { dim, 1.0f, dim }, { dim, dim, 1.0f }
	};
	int x, y, line;

	if (dst_width < 1 || dst_height < 1)
		return;
	if (zoom_x < 1)
		zoom_x = 1;
	if (zoom_y < 1)
		zoom_y = 1;

	init_luts();
	if (!update_geometry(dst_width, dst_height, src_width, src_height,
	                     zoom_x, zoom_y, scans_per_line))
		return;

	for (line = 0; line < geo.lines; ++line) {
		const uint32_t *row = (const uint32_t *)((const unsigned char *)src +
		                      (size_t)line * zoom_y * src_pitch);

		reconstruct_line(geo.line_rgb + (size_t)line * dst_width * 3, row, zoom_x);
	}

	for (y = 0; y < dst_height; ++y) {
		const float *rgb = geo.line_rgb + (size_t)geo.rows[y] * dst_width * 3;
		const float weight = geo.row_weight[y];
		uint32_t *out = dst + (size_t)y * dst_width;
		int dot = 0;

		for (x = 0; x < dst_width; ++x, rgb += 3) {
			const float *m = mask[dot];
			float scale = 1.0f;

			if (weight < 1.0f) {
				/* Bright pixels fill more of the gap. */
				const float luma = rgb[0] * 0.2126f + rgb[1] * 0.7152f +
				                   rgb[2] * 0.0722f;
				float bright = rgb[0] > rgb[1] ? rgb[0] : rgb[1];

				if (rgb[2] > bright)
					bright = rgb[2];
				bright = (bright + luma) * 0.5f;
				if (bright < EASYMODE_BRIGHT_MIN)
					bright = EASYMODE_BRIGHT_MIN;
				else if (bright > EASYMODE_BRIGHT_MAX)
					bright = EASYMODE_BRIGHT_MAX;
				scale = weight + bright * (1.0f - weight);
			}

			out[x] = output(rgb[0] * scale * m[0]) << 16 |
			         output(rgb[1] * scale * m[1]) << 8 |
			         output(rgb[2] * scale * m[2]);
			if (++dot == 3)
				dot = 0;
		}
	}
}
