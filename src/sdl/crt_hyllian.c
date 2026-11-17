/*
 * Software crt-hyllian pass for Hatari's portable SDL renderer.
 *
 * The reference shader is Hyllian's CRT shader, distributed by
 * libretro/glsl-shaders.  This implementation keeps the characteristic
 * stages that matter for Falcon RGB/TV and VGA presentation:
 * gamma-linearized cubic reconstruction with anti-ringing, an adaptive beam
 * profile, phosphor mask, and output gamma/colour boost.  It runs at the
 * output resolution and samples the emulated raster in video pixels and
 * lines: a raster that the converter zoomed (nScreenZoomX/Y) is read one
 * zoomed pixel per video pixel.  The caller says how the monitor shows a
 * line (one or two scanlines, or interlaced without gaps), and the beam
 * profile is laid out on those scanlines.  Overscan geometry is owned by
 * videl.c and the status bar by screen.c.
 *
 * Per frame the horizontal reconstruction runs once per Videl line, the beam
 * profile comes from per-row tables, and the output transfer from an 8-bit
 * table, so the per-pixel work is a few multiply-adds.
 */
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "crt_hyllian.h"

#define HYLLIAN_INPUT_GAMMA         2.4f
#define HYLLIAN_OUTPUT_GAMMA        2.2f
#define HYLLIAN_BEAM_MIN_WIDTH      0.72f
#define HYLLIAN_BEAM_MAX_WIDTH      1.00f
#define HYLLIAN_SCANLINE_STRENGTH   0.58f
#define HYLLIAN_COLOR_BOOST         1.25f
#define HYLLIAN_MASK_INTENSITY      0.38f
#define HYLLIAN_OUTPUT_LUT_SIZE     4096
/* Beam weight table resolution over the brightest channel (0..1). */
#define HYLLIAN_BEAM_LUT_SIZE       256

static float input_gamma_lut[256];
static uint8_t output_gamma_lut[HYLLIAN_OUTPUT_LUT_SIZE];
static bool gamma_luts_ready;

/* Geometry, rebuilt when a dimension changes. */
static struct {
	int dst_width, dst_height;
	int src_width, src_height;
	int zoom_x, zoom_y;
	int scans_per_line;
	int lines;
	int *taps;          /* 4 source columns per output column */
	float *weights;     /* 4 cubic weights per output column */
	int *rows;          /* 2 Videl lines per output row */
	float *row_pos;     /* beam distance of the first line per output row */
	float *line_rgb;    /* reconstructed lines: lines x dst_width x 3 */
} geo;

static void init_gamma_luts(void)
{
	int i;

	if (gamma_luts_ready)
		return;

	for (i = 0; i < 256; ++i)
		input_gamma_lut[i] = powf((float)i / 255.0f, HYLLIAN_INPUT_GAMMA);
	for (i = 0; i < HYLLIAN_OUTPUT_LUT_SIZE; ++i)
		output_gamma_lut[i] = (uint8_t)(powf((float)i /
		                                     (HYLLIAN_OUTPUT_LUT_SIZE - 1),
		                                     1.0f / HYLLIAN_OUTPUT_GAMMA) *
		                                255.0f + 0.5f);

	gamma_luts_ready = true;
}

static inline unsigned gamma_out(float value)
{
	int index = (int)(value * (HYLLIAN_OUTPUT_LUT_SIZE - 1) + 0.5f);

	if (index < 0)
		index = 0;
	else if (index >= HYLLIAN_OUTPUT_LUT_SIZE)
		index = HYLLIAN_OUTPUT_LUT_SIZE - 1;
	return output_gamma_lut[index];
}

static int clampi(int value, int low, int high)
{
	return value < low ? low : value > high ? high : value;
}

static void free_geometry(void)
{
	free(geo.taps);
	free(geo.weights);
	free(geo.rows);
	free(geo.row_pos);
	free(geo.line_rgb);
	geo.taps = NULL;
	geo.weights = NULL;
	geo.rows = NULL;
	geo.row_pos = NULL;
	geo.line_rgb = NULL;
	geo.dst_width = 0;
}

static bool update_geometry(int dst_width, int dst_height, int src_width,
	                        int src_height, int zoom_x, int zoom_y,
	                        int scans_per_line)
{
	int columns, scans, x, y;

	if (geo.taps && geo.dst_width == dst_width && geo.dst_height == dst_height &&
	    geo.src_width == src_width && geo.src_height == src_height &&
	    geo.zoom_x == zoom_x && geo.zoom_y == zoom_y &&
	    geo.scans_per_line == scans_per_line)
		return true;

	free_geometry();
	columns = src_width / zoom_x;
	geo.lines = src_height / zoom_y;
	if (columns < 1 || geo.lines < 1)
		return false;

	geo.taps = malloc(sizeof(int) * 4 * dst_width);
	geo.weights = malloc(sizeof(float) * 4 * dst_width);
	geo.rows = malloc(sizeof(int) * 2 * dst_height);
	geo.row_pos = malloc(sizeof(float) * dst_height);
	geo.line_rgb = malloc(sizeof(float) * 3 * dst_width * geo.lines);
	if (!geo.taps || !geo.weights || !geo.rows || !geo.row_pos || !geo.line_rgb) {
		free_geometry();
		return false;
	}

	/* Output pixel centres mapped onto Videl pixel centres. */
	for (x = 0; x < dst_width; ++x) {
		float fx = ((float)x + 0.5f) * columns / dst_width - 0.5f;
		float base = floorf(fx);
		int ix = (int)base;
		float t = fx - ix, t2 = t * t, t3 = t2 * t;
		int k;

		for (k = 0; k < 4; ++k)
			geo.taps[x * 4 + k] = clampi(ix - 1 + k, 0, columns - 1) * zoom_x;
		/* Catmull-Rom, Hyllian's default B=0/C=0.5 Mitchell-Netravali. */
		geo.weights[x * 4 + 0] = -0.5f * t3 + t2 - 0.5f * t;
		geo.weights[x * 4 + 1] = 1.5f * t3 - 2.5f * t2 + 1.0f;
		geo.weights[x * 4 + 2] = -1.5f * t3 + 2.0f * t2 + 0.5f * t;
		geo.weights[x * 4 + 3] = 0.5f * t3 - 0.5f * t2;
	}

	/* Each output row lies between two scanlines; the beam profile of
	 * both is evaluated at its distance from their centres, and each
	 * scanline shows the video line it repeats.  Interlaced lines have no
	 * gaps between them and are laid out one per line. */
	scans = geo.lines * (scans_per_line > 0 ? scans_per_line : 1);
	for (y = 0; y < dst_height; ++y) {
		float fy = ((float)y + 0.5f) * scans / dst_height - 0.5f;
		float base = floorf(fy);
		int iy = (int)base;
		int repeat = scans_per_line > 0 ? scans_per_line : 1;

		geo.rows[y * 2] = clampi(iy, 0, scans - 1) / repeat;
		geo.rows[y * 2 + 1] = clampi(iy + 1, 0, scans - 1) / repeat;
		geo.row_pos[y] = fy - iy;
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

/* Linear-light cubic reconstruction of one Videl line at every output
 * column, clamped to the two central samples (anti-ringing). */
static void reconstruct_line(float *out, const uint32_t *line, int dst_width)
{
	const int *taps = geo.taps;
	const float *weights = geo.weights;
	int x, shift, c;

	for (x = 0; x < dst_width; ++x, taps += 4, weights += 4) {
		const uint32_t p0 = line[taps[0]], p1 = line[taps[1]];
		const uint32_t p2 = line[taps[2]], p3 = line[taps[3]];

		for (c = 0, shift = 16; c < 3; ++c, shift -= 8) {
			const float s0 = input_gamma_lut[(p0 >> shift) & 255];
			const float s1 = input_gamma_lut[(p1 >> shift) & 255];
			const float s2 = input_gamma_lut[(p2 >> shift) & 255];
			const float s3 = input_gamma_lut[(p3 >> shift) & 255];
			float value = s0 * weights[0] + s1 * weights[1] +
			              s2 * weights[2] + s3 * weights[3];
			const float lo = s1 < s2 ? s1 : s2;
			const float hi = s1 < s2 ? s2 : s1;

			*out++ = value < lo ? lo : value > hi ? hi : value;
		}
	}
}

/* Beam weight exp(-(4 * strength * pos / width)^2) for every brightness,
 * the width growing with the line's brightest channel. */
static void beam_table(float *table, float pos)
{
	int i;

	for (i = 0; i < HYLLIAN_BEAM_LUT_SIZE; ++i) {
		const float bright = (float)i / (HYLLIAN_BEAM_LUT_SIZE - 1);
		const float width = HYLLIAN_BEAM_MIN_WIDTH +
		                    (HYLLIAN_BEAM_MAX_WIDTH - HYLLIAN_BEAM_MIN_WIDTH) * bright;
		const float d = 4.0f * HYLLIAN_SCANLINE_STRENGTH * pos / width;

		table[i] = expf(-d * d);
	}
}

static inline int beam_index(const float *rgb)
{
	float bright = rgb[0] > rgb[1] ? rgb[0] : rgb[1];

	if (rgb[2] > bright)
		bright = rgb[2];
	return (int)(bright * (HYLLIAN_BEAM_LUT_SIZE - 1) + 0.5f);
}

void CRT_Hyllian_Process(uint32_t *dst, int dst_width, int dst_height,
	                     const uint32_t *src, int src_width, int src_height,
	                     int src_pitch, int zoom_x, int zoom_y,
	                     int scans_per_line)
{
	/* Aperture grille in RGB order (the SC1224 layout): the lit channel
	 * keeps full strength, the other two are dimmed. */
	const float dim = 1.0f - HYLLIAN_MASK_INTENSITY;
	float mask[3][3] = {
		{ 1.0f, dim, dim }, { dim, 1.0f, dim }, { dim, dim, 1.0f }
	};
	float beam0[HYLLIAN_BEAM_LUT_SIZE], beam1[HYLLIAN_BEAM_LUT_SIZE];
	bool scanlines;
	int x, y, line, c;

	if (dst_width < 1 || dst_height < 1)
		return;
	if (zoom_x < 1)
		zoom_x = 1;
	if (zoom_y < 1)
		zoom_y = 1;

	init_gamma_luts();
	if (!update_geometry(dst_width, dst_height, src_width, src_height,
	                     zoom_x, zoom_y, scans_per_line))
		return;

	scanlines = scans_per_line > 0;
	for (x = 0; x < 3; ++x) {
		for (c = 0; c < 3; ++c)
			mask[x][c] *= scanlines ? HYLLIAN_COLOR_BOOST : 1.0f;
	}

	for (line = 0; line < geo.lines; ++line) {
		const uint32_t *row = (const uint32_t *)((const unsigned char *)src +
		                      (size_t)line * zoom_y * src_pitch);

		reconstruct_line(geo.line_rgb + (size_t)line * dst_width * 3, row,
		                 dst_width);
	}

	for (y = 0; y < dst_height; ++y) {
		const int line0 = geo.rows[y * 2], line1 = geo.rows[y * 2 + 1];
		const float pos = geo.row_pos[y];
		const float *rgb0 = geo.line_rgb + (size_t)line0 * dst_width * 3;
		const float *rgb1 = geo.line_rgb + (size_t)line1 * dst_width * 3;
		uint32_t *out = dst + (size_t)y * dst_width;
		int dot = 0;

		if (scanlines) {
			beam_table(beam0, pos);
			beam_table(beam1, 1.0f - pos);
		} else if (pos >= 0.5f) {
			/* Interlaced: the nearest line, no gaps to draw. */
			rgb0 = rgb1;
		}

		for (x = 0; x < dst_width; ++x, rgb0 += 3, rgb1 += 3) {
			const float *m = mask[dot];
			float red, green, blue;

			if (scanlines) {
				const float d0 = beam0[beam_index(rgb0)];
				const float d1 = beam1[beam_index(rgb1)];

				red = rgb0[0] * d0 + rgb1[0] * d1;
				green = rgb0[1] * d0 + rgb1[1] * d1;
				blue = rgb0[2] * d0 + rgb1[2] * d1;
			} else {
				red = rgb0[0];
				green = rgb0[1];
				blue = rgb0[2];
			}

			out[x] = gamma_out(red * m[0]) << 16 |
			         gamma_out(green * m[1]) << 8 |
			         gamma_out(blue * m[2]);
			if (++dot == 3)
				dot = 0;
		}
	}
}
