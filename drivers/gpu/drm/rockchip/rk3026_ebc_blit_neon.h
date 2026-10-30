/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The NEON rows of rk3026_ebc_blit_neon.c (CONFIG_DRM_RK3026_EBC_NEON).
 * Plain C types only: that unit includes <arm_neon.h> and no kernel header.
 */
#ifndef _RK3026_EBC_BLIT_NEON_H
#define _RK3026_EBC_BLIT_NEON_H

void rk3026_ebc_blit_rows_neon(unsigned char *dst, const void *src,
			       unsigned int src_pitch, int x1, int x2,
			       int y1, int y2, unsigned int width);
void rk3026_ebc_blit_grey_rows_neon(unsigned char *dst, const void *src,
				    unsigned int src_pitch, int x1, int x2,
				    int y1, int y2, unsigned int width);

#endif /* _RK3026_EBC_BLIT_NEON_H */
