/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Reference implementations for the RK3026 EBC KUnit tests: the snapshot
 * and the blits as per-pixel loops. Test only.
 */

#ifndef _RK3026_EBC_KUNIT_REF_H
#define _RK3026_EBC_KUNIT_REF_H

#include <linux/types.h>

struct drm_rect;
struct rk3026_ebc_shadows;

bool rk3026_ebc_ref_snapshot(const struct rk3026_ebc_shadows *sh,
			     const u8 *next, const struct drm_rect *box,
			     u32 *mode);
void rk3026_ebc_ref_blit_rows(u8 *dst, const void *src,
			      unsigned int src_pitch, int x1, int x2,
			      int y1, int y2, unsigned int width);
void rk3026_ebc_ref_grey_rows(u8 *dst, const void *src,
			      unsigned int src_pitch, int x1, int x2,
			      int y1, int y2, unsigned int width,
			      const u8 *levels);

#endif /* _RK3026_EBC_KUNIT_REF_H */
