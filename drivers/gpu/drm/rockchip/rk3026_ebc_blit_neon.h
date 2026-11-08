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
#define EBC_NEON_R8		EBC_NEON_FOURCC('R', '8', ' ', ' ')

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
