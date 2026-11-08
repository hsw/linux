/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The NEON rows of rk3026_ebc_blit_neon.c (CONFIG_DRM_RK3026_EBC_NEON).
 * Plain C types only: that unit includes <arm_neon.h> and no kernel header.
 */
#ifndef _RK3026_EBC_BLIT_NEON_H
#define _RK3026_EBC_BLIT_NEON_H

/*
 * The DRM formats the NEON rows take, coded as <drm/drm_fourcc.h> codes
 * them: that header cannot be included in the NEON unit.
 * rk3026_ebc_blit.c checks each against its DRM_FORMAT_ name.
 */
#define EBC_NEON_FOURCC(a, b, c, d)					\
	((unsigned int)(a) | (unsigned int)(b) << 8 |			\
	 (unsigned int)(c) << 16 | (unsigned int)(d) << 24)
#define EBC_NEON_XRGB8888	EBC_NEON_FOURCC('X', 'R', '2', '4')
#define EBC_NEON_ARGB8888	EBC_NEON_FOURCC('A', 'R', '2', '4')
#define EBC_NEON_XBGR8888	EBC_NEON_FOURCC('X', 'B', '2', '4')
#define EBC_NEON_ABGR8888	EBC_NEON_FOURCC('A', 'B', '2', '4')
#define EBC_NEON_RGB565	EBC_NEON_FOURCC('R', 'G', '1', '6')
#define EBC_NEON_BGR565	EBC_NEON_FOURCC('B', 'G', '1', '6')
#define EBC_NEON_R8		EBC_NEON_FOURCC('R', '8', ' ', ' ')

/*
 * The luma of a 16-bit pixel (RGB565, BGR565) from its 5-, 6- and 5-bit
 * fields, for the rows of both units: (EBC_565_WR R + EBC_565_WG G +
 * EBC_565_WB B) >> EBC_565_SHIFT, the BT.601 weights 77, 150 and 29 / 256
 * scaled by 255 / 31, 255 / 63 and 255 / 31. Of the sets with every
 * weight below 256, so that NEON multiplies the fields as bytes, this one
 * gives the fewest nibbles that differ from the nibble of the colour
 * expanded to 8 bits by bit replication and converted as XRGB8888: 852 of
 * the 65536 values, each by one level. White sums to 16334 < 2^16, so the
 * scalar rows weigh two pixels per 32-bit word, one in each half.
 */
#define EBC_565_WR	159
#define EBC_565_WG	152
#define EBC_565_WB	59
#define EBC_565_SHIFT	6

void rk3026_ebc_blit_rows_neon(unsigned char *dst, const void *src,
			       unsigned int src_pitch, unsigned int format,
			       int x1, int x2, int y1, int y2,
			       unsigned int width);
void rk3026_ebc_blit_rows_dither_neon(unsigned char *dst, const void *src,
				      unsigned int src_pitch,
				      unsigned int format, int x1, int x2,
				      int y1, int y2, unsigned int width,
				      const unsigned char *thr,
				      unsigned int steps);

#endif /* _RK3026_EBC_BLIT_NEON_H */
