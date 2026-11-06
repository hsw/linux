// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro JPEG decoder driver
 *
 * Copyright (C) STMicroelectronics SA 2024
 * Authors: Hugues Fruchet <hugues.fruchet@foss.st.com>
 *          for STMicroelectronics.
 *
 * This code is inspired from coda-jpeg.c
 *
 */

#include <kunit/visibility.h>
#include <linux/bitfield.h>
#include <linux/unaligned.h>
#include <media/v4l2-jpeg.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-rect.h>
#include "hantro.h"
#include "hantro_g1_regs.h"
#include "hantro_hw.h"
#include "hantro_jpeg.h"

/* JPEG_MODE values: the sampling the decoder reads and writes. */
#define G1_JPEG_MODE_GREY	0
#define G1_JPEG_MODE_420	2
#define G1_JPEG_MODE_422	3

VISIBLE_IF_KUNIT int hantro_g1_jpeg_dec_mode(int subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
		return G1_JPEG_MODE_GREY;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		return G1_JPEG_MODE_420;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return G1_JPEG_MODE_422;
	default:
		return -EINVAL;
	}
}

VISIBLE_IF_KUNIT u32 hantro_g1_jpeg_dec_fourcc(int subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
		return V4L2_PIX_FMT_GREY;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		return V4L2_PIX_FMT_NV12;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return V4L2_PIX_FMT_NV16;
	default:
		return 0;
	}
}

/*
 * The 16 code counts (BITS) of the two AC and the two DC Huffman tables;
 * table 1 is the one the first component uses, table 2 the other one.
 */
static void vlc_code_counts(const u8 *ac1, const u8 *ac2,
			    const u8 *dc1, const u8 *dc2, u32 *vlc)
{
	vlc[0] = G1_REG_DEC_JPEG_VLC_AC1_CODE1_CNT(ac1[0]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE2_CNT(ac1[1]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE3_CNT(ac1[2]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE4_CNT(ac1[3]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE5_CNT(ac1[4]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE6_CNT(ac1[5]);
	vlc[1] = G1_REG_DEC_JPEG_VLC_AC1_CODE7_CNT(ac1[6]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE8_CNT(ac1[7]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE9_CNT(ac1[8]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE10_CNT(ac1[9]);
	vlc[2] = G1_REG_DEC_JPEG_VLC_AC1_CODE11_CNT(ac1[10]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE12_CNT(ac1[11]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE13_CNT(ac1[12]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE14_CNT(ac1[13]);
	vlc[3] = G1_REG_DEC_JPEG_VLC_AC1_CODE15_CNT(ac1[14]) |
		 G1_REG_DEC_JPEG_VLC_AC1_CODE16_CNT(ac1[15]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE1_CNT(ac2[0]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE2_CNT(ac2[1]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE3_CNT(ac2[2]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE4_CNT(ac2[3]);
	vlc[4] = G1_REG_DEC_JPEG_VLC_AC2_CODE5_CNT(ac2[4]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE6_CNT(ac2[5]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE7_CNT(ac2[6]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE8_CNT(ac2[7]);
	vlc[5] = G1_REG_DEC_JPEG_VLC_AC2_CODE9_CNT(ac2[8]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE10_CNT(ac2[9]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE11_CNT(ac2[10]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE12_CNT(ac2[11]);
	vlc[6] = G1_REG_DEC_JPEG_VLC_AC2_CODE13_CNT(ac2[12]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE14_CNT(ac2[13]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE15_CNT(ac2[14]) |
		 G1_REG_DEC_JPEG_VLC_AC2_CODE16_CNT(ac2[15]);
	vlc[7] = G1_REG_DEC_JPEG_VLC_DC1_CODE1_CNT(dc1[0]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE2_CNT(dc1[1]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE3_CNT(dc1[2]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE4_CNT(dc1[3]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE5_CNT(dc1[4]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE6_CNT(dc1[5]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE7_CNT(dc1[6]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE8_CNT(dc1[7]);
	vlc[8] = G1_REG_DEC_JPEG_VLC_DC1_CODE9_CNT(dc1[8]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE10_CNT(dc1[9]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE11_CNT(dc1[10]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE12_CNT(dc1[11]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE13_CNT(dc1[12]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE14_CNT(dc1[13]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE15_CNT(dc1[14]) |
		 G1_REG_DEC_JPEG_VLC_DC1_CODE16_CNT(dc1[15]);
	vlc[9] = G1_REG_DEC_JPEG_VLC_DC2_CODE1_CNT(dc2[0]) |
		 G1_REG_DEC_JPEG_VLC_DC2_CODE2_CNT(dc2[1]) |
		 G1_REG_DEC_JPEG_VLC_DC2_CODE3_CNT(dc2[2]) |
		 G1_REG_DEC_JPEG_VLC_DC2_CODE4_CNT(dc2[3]) |
		 G1_REG_DEC_JPEG_VLC_DC2_CODE5_CNT(dc2[4]) |
		 G1_REG_DEC_JPEG_VLC_DC2_CODE6_CNT(dc2[5]) |
		 G1_REG_DEC_JPEG_VLC_DC2_CODE7_CNT(dc2[6]) |
		 G1_REG_DEC_JPEG_VLC_DC2_CODE8_CNT(dc2[7]);
	vlc[10] = G1_REG_DEC_JPEG_VLC_DC2_CODE9_CNT(dc2[8]) |
		  G1_REG_DEC_JPEG_VLC_DC2_CODE10_CNT(dc2[9]) |
		  G1_REG_DEC_JPEG_VLC_DC2_CODE11_CNT(dc2[10]) |
		  G1_REG_DEC_JPEG_VLC_DC2_CODE12_CNT(dc2[11]) |
		  G1_REG_DEC_JPEG_VLC_DC2_CODE13_CNT(dc2[12]) |
		  G1_REG_DEC_JPEG_VLC_DC2_CODE14_CNT(dc2[13]) |
		  G1_REG_DEC_JPEG_VLC_DC2_CODE15_CNT(dc2[14]) |
		  G1_REG_DEC_JPEG_VLC_DC2_CODE16_CNT(dc2[15]);
}

/* The last macroblock column/row holds 1 to 8 picture columns/rows. */
static bool fill_needed(unsigned int size)
{
	unsigned int rem = size % MB_DIM;

	return rem && rem <= MB_DIM / 2;
}

/* G1_REG_PP_CONTROL input formats: the decoder's output */
#define G1_PP_IN_YUV420		1
#define G1_PP_IN_YUV400		3
#define G1_PP_IN_YUV422		4

/*
 * The CAPTURE buffers the post-processor writes into: up to its widest
 * stride, and as high as the decoder's highest picture.
 */
static const struct v4l2_frmsize_stepwise pp_buf_sizes = {
	.min_width = HANTRO_G1_PP_MIN_SIZE,
	.max_width = HANTRO_G1_PP_MAX_STRIDE,
	.step_width = HANTRO_G1_PP_STEP_WIDTH,
	.min_height = HANTRO_G1_PP_MIN_SIZE,
	.max_height = HANTRO_G1_JPEG_DEC_MAX_SIZE,
	.step_height = HANTRO_G1_PP_STEP_HEIGHT,
};

/* A format the decoder writes itself, for one sampling or another. */
static bool dec_fmt(u32 fourcc)
{
	return fourcc == V4L2_PIX_FMT_NV12 || fourcc == V4L2_PIX_FMT_NV16 ||
	       fourcc == V4L2_PIX_FMT_GREY;
}

/* A format the post-processor writes. */
static bool pp_fmt(const struct hantro_g1_jpeg_dec_out *out)
{
	return out->pp_max_width && hantro_g1_pp_writes(out->fourcc);
}

/* The decoder's own buffer: its format, the picture in macroblocks. */
static bool is_dec_buf(const struct hantro_g1_jpeg_dec_out *out)
{
	return dec_fmt(out->fourcc) &&
	       out->width == ALIGN(out->src_width, MB_DIM) &&
	       out->height == ALIGN(out->src_height, MB_DIM);
}

/*
 * A buffer of width x height holds the whole picture: each direction,
 * rounded up to 8 as the OUTPUT format is, at least the picture's. A
 * client that asks for the size in the JPEG header gets the picture, not a
 * smaller buffer it would be scaled into.
 */
static bool holds_picture(const struct hantro_g1_jpeg_dec_out *out,
			  u32 width, u32 height)
{
	return ALIGN(width, 8) >= out->src_width &&
	       ALIGN(height, 8) >= out->src_height;
}

static bool is_crop(const struct hantro_g1_jpeg_dec_out *out,
		    const struct v4l2_rect *r)
{
	return !r->left && !r->top && r->width == out->src_width &&
	       r->height == out->src_height;
}

/* A buffer the post-processor writes into. */
static bool is_pp_buf(const struct hantro_g1_jpeg_dec_out *out)
{
	const struct v4l2_frmsize_stepwise *s = &pp_buf_sizes;

	return pp_fmt(out) &&
	       out->width >= s->min_width && out->width <= s->max_width &&
	       IS_ALIGNED(out->width, s->step_width) &&
	       out->height >= s->min_height && out->height <= s->max_height &&
	       IS_ALIGNED(out->height, s->step_height);
}

/*
 * The picture scaled down, its aspect ratio kept, to fit width x height and
 * the post-processor's largest output.
 */
static void pp_fit(const struct hantro_g1_jpeg_dec_out *out,
		   u32 width, u32 height, u32 *w, u32 *h)
{
	u32 src_w = out->src_width, src_h = out->src_height;

	width = min(width, out->pp_max_width);
	height = min_t(u32, height, HANTRO_G1_PP_MAX_HEIGHT);
	if (src_w <= width && src_h <= height) {
		*w = src_w;
		*h = src_h;
	} else if ((u64)src_w * height >= (u64)src_h * width) {
		*w = width;
		*h = div_u64((u64)src_h * width, src_w);
	} else {
		*w = div_u64((u64)src_w * height, src_h);
		*h = height;
	}
	*w = max_t(u32, ALIGN_DOWN(*w, HANTRO_G1_PP_STEP_WIDTH),
		   HANTRO_G1_PP_MIN_SIZE);
	*h = max_t(u32, ALIGN_DOWN(*h, HANTRO_G1_PP_STEP_HEIGHT),
		   HANTRO_G1_PP_MIN_SIZE);
}

/*
 * One direction of the post-processor's input and output. At the
 * picture's own size, with room for it in the buffer, it reads and writes
 * whole macroblocks, as the decoder writes them; otherwise it reads the
 * picture as the OUTPUT format has it, a multiple of 8, and scales it to
 * the compose size.
 */
static void pp_axis(u32 src, u32 pos, u32 size, u32 buf, u32 max,
		    u32 *in, u32 *out)
{
	u32 mb = ALIGN(src, MB_DIM);

	if (size == src && pos + mb <= buf && mb <= max) {
		*in = mb;
		*out = mb;
	} else {
		*in = src;
		*out = size;
	}
}

/**
 * hantro_g1_jpeg_dec_cap_size() - adjust a CAPTURE size
 * @out:	the context, with the CAPTURE format to adjust
 * @width:	CAPTURE width asked for; the one to use
 * @height:	CAPTURE height asked for; likewise
 *
 * NV16 and GREY are the decoder's: the picture rounded up to macroblocks.
 * So is NV12 at 0x0 or at any size that holds the whole picture, each
 * direction rounded up to 8 as the OUTPUT format is: a client that asks
 * for the picture's size, also one in the header that is not a multiple
 * of 8, gets all of it, as from the decoder alone. NV12 smaller than that
 * in either direction is a buffer the post-processor writes into, up to
 * its widest stride. Its other formats
 * are such buffers at any size; 0 asks for the whole picture as large as
 * the post-processor writes it.
 */
VISIBLE_IF_KUNIT void hantro_g1_jpeg_dec_cap_size(const struct hantro_g1_jpeg_dec_out *out,
						  u32 *width, u32 *height)
{
	if (!dec_fmt(out->fourcc)) {
		if (!*width || !*height)
			pp_fit(out, out->pp_max_width, HANTRO_G1_PP_MAX_HEIGHT,
			       width, height);
		v4l2_apply_frmsize_constraints(width, height, &pp_buf_sizes);
		return;
	}

	if (pp_fmt(out) && *width && *height &&
	    !holds_picture(out, *width, *height)) {
		v4l2_apply_frmsize_constraints(width, height, &pp_buf_sizes);
		/* Not rounded up to the picture's size by the steps. */
		if (!holds_picture(out, *width, *height))
			return;
	}

	*width = ALIGN(out->src_width, MB_DIM);
	*height = ALIGN(out->src_height, MB_DIM);
}

/**
 * hantro_g1_jpeg_dec_cap_frmsize() - the CAPTURE sizes of a format
 * @out:	the context, with the CAPTURE format
 * @fsize:	the sizes
 *
 * The decoder's own buffer: the one size of NV16 and GREY, and the one
 * NV12 size that holds the whole picture. The smaller NV12 buffers the
 * post-processor writes into are not listed: no stepwise range gives the
 * sizes below the picture in either direction without sizes that S_FMT
 * replaces by the decoder's. The post-processor's other formats: the
 * buffers it writes into, stepwise.
 *
 * Return: 0, or -EINVAL for an index other than 0.
 */
VISIBLE_IF_KUNIT int hantro_g1_jpeg_dec_cap_frmsize(const struct hantro_g1_jpeg_dec_out *out,
						    struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;

	if (!dec_fmt(out->fourcc)) {
		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise = pp_buf_sizes;
		return 0;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ALIGN(out->src_width, MB_DIM);
	fsize->discrete.height = ALIGN(out->src_height, MB_DIM);

	return 0;
}

/**
 * hantro_g1_jpeg_dec_compose_default() - the default compose rectangle
 * @out:	the context
 * @r:		the rectangle
 *
 * The crop rectangle, the whole picture, in the decoder's own buffer. In a
 * buffer the post-processor writes, the crop rectangle too where the buffer
 * holds it and the post-processor writes it that large; otherwise, the one
 * case where it cannot be the crop rectangle, the picture at (0, 0) scaled
 * down, its aspect ratio kept, as far as it needs to fit the buffer and the
 * post-processor's largest output.
 */
VISIBLE_IF_KUNIT void hantro_g1_jpeg_dec_compose_default(const struct hantro_g1_jpeg_dec_out *out,
							 struct v4l2_rect *r)
{
	r->left = 0;
	r->top = 0;
	if (is_dec_buf(out) || !is_pp_buf(out)) {
		r->width = min(out->src_width, out->width);
		r->height = min(out->src_height, out->height);
		return;
	}

	pp_fit(out, out->width, out->height, &r->width, &r->height);
}

/**
 * hantro_g1_jpeg_dec_compose_adjust() - adjust a compose rectangle
 * @out:	the context
 * @r:		the rectangle asked for; the one to use
 *
 * In a buffer the post-processor writes into, the rectangle is the size it
 * writes, inside the buffer and no larger than the picture: it scales down
 * only. Its width goes in steps of 8 pixels, its height in steps of 2, and
 * it starts at a multiple of 8 bytes in an even line. In any other buffer
 * it is the default.
 */
VISIBLE_IF_KUNIT void hantro_g1_jpeg_dec_compose_adjust(const struct hantro_g1_jpeg_dec_out *out,
							struct v4l2_rect *r)
{
	u32 max_w, max_h, w, h, x_step;

	if (!is_pp_buf(out)) {
		hantro_g1_jpeg_dec_compose_default(out, r);
		return;
	}

	max_w = min3(out->src_width, out->pp_max_width, out->width);
	max_h = min3(out->src_height, (u32)HANTRO_G1_PP_MAX_HEIGHT, out->height);
	w = clamp_t(u32, ALIGN_DOWN(r->width, HANTRO_G1_PP_STEP_WIDTH),
		    HANTRO_G1_PP_MIN_SIZE,
		    ALIGN_DOWN(max_w, HANTRO_G1_PP_STEP_WIDTH));
	h = clamp_t(u32, ALIGN_DOWN(r->height, HANTRO_G1_PP_STEP_HEIGHT),
		    HANTRO_G1_PP_MIN_SIZE,
		    ALIGN_DOWN(max_h, HANTRO_G1_PP_STEP_HEIGHT));
	x_step = 8 / v4l2_format_info(out->fourcc)->bpp[0];

	r->left = ALIGN_DOWN(clamp_t(s32, r->left, 0, out->width - w), x_step);
	r->top = ALIGN_DOWN(clamp_t(s32, r->top, 0, out->height - h),
			    HANTRO_G1_PP_STEP_HEIGHT);
	r->width = w;
	r->height = h;
}

/**
 * hantro_g1_jpeg_dec_compose_padded() - what the hardware writes
 * @out:	the context
 * @r:		the rectangle
 *
 * The compose rectangle, but whole macroblocks in a direction the picture
 * keeps its size in and has room for them: the decoder writes them in its
 * own buffer, the post-processor where it is not scaling.
 */
VISIBLE_IF_KUNIT void hantro_g1_jpeg_dec_compose_padded(const struct hantro_g1_jpeg_dec_out *out,
							struct v4l2_rect *r)
{
	bool dec = is_dec_buf(out) && is_crop(out, &out->compose);
	u32 in;

	*r = out->compose;
	pp_axis(out->src_width, r->left, r->width, out->width,
		dec ? U32_MAX : out->pp_max_width, &in, &r->width);
	pp_axis(out->src_height, r->top, r->height, out->height,
		dec ? U32_MAX : HANTRO_G1_PP_MAX_HEIGHT, &in, &r->height);
}

/**
 * hantro_g1_jpeg_dec_job_out() - how a job writes the CAPTURE buffer
 * @out:		the context
 * @subsampling:	the picture's sampling
 * @pp:			the post-processor registers, when it writes it
 * @use_pp:		whether it does
 *
 * The decoder writes its own buffer, in the format of the picture's
 * sampling, with the whole picture at (0, 0). The post-processor, in
 * pipeline with it, writes anything else it can: the picture scaled to the
 * compose rectangle, in its format.
 *
 * Return: 0, or -EINVAL if neither can.
 */
VISIBLE_IF_KUNIT int hantro_g1_jpeg_dec_job_out(const struct hantro_g1_jpeg_dec_out *out,
						int subsampling,
						struct hantro_g1_pp_regs *pp,
						bool *use_pp)
{
	const struct v4l2_rect *c = &out->compose;
	struct hantro_g1_pp_cfg cfg = {
		.orig_mb_width = MB_WIDTH(out->src_width),
		.fourcc = out->fourcc,
		.stride = out->width,
		.buf_height = out->height,
		.left = c->left,
		.top = c->top,
		/* JFIF: BT.601 at full range. */
		.csc = &hantro_g1_pp_csc_bt601_full,
	};
	struct v4l2_rect r = *c;

	*use_pp = false;
	if (is_dec_buf(out) && is_crop(out, c) &&
	    out->fourcc == hantro_g1_jpeg_dec_fourcc(subsampling))
		return 0;

	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		cfg.in_fmt = G1_PP_IN_YUV420;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		cfg.in_fmt = G1_PP_IN_YUV422;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
		cfg.in_fmt = G1_PP_IN_YUV400;
		break;
	default:
		return -EINVAL;
	}

	/* A compose rectangle the post-processor writes stays as it is. */
	hantro_g1_jpeg_dec_compose_adjust(out, &r);
	if (!is_pp_buf(out) || memcmp(&r, c, sizeof(r))) {
		vpu_debug(1, "%ux%u, subsampling %d: no output %p4cc %ux%u, (%d,%d) %ux%u\n",
			  out->src_width, out->src_height, subsampling,
			  &out->fourcc, out->width, out->height,
			  c->left, c->top, c->width, c->height);
		return -EINVAL;
	}

	pp_axis(out->src_width, c->left, c->width, out->width,
		out->pp_max_width, &cfg.in_width, &cfg.width);
	pp_axis(out->src_height, c->top, c->height, out->height,
		HANTRO_G1_PP_MAX_HEIGHT, &cfg.in_height, &cfg.height);

	*use_pp = true;
	return hantro_g1_pp_regs(&cfg, pp);
}

/**
 * hantro_g1_jpeg_dec_regs() - compute the registers of a decoding job
 * @hdr:	parsed header that passed hantro_g1_jpeg_dec_check()
 * @src:	bus address of the OUTPUT buffer
 * @payload:	bytes used in the OUTPUT buffer, from its start
 * @regs:	the register values
 */
VISIBLE_IF_KUNIT void hantro_g1_jpeg_dec_regs(const struct v4l2_jpeg_header *hdr,
					      dma_addr_t src, size_t payload,
					      struct hantro_g1_jpeg_dec_regs *regs)
{
	const struct v4l2_jpeg_frame_header *frame = &hdr->frame;
	const struct v4l2_jpeg_scan_header *scan = hdr->scan;
	const struct v4l2_jpeg_reference *ht = hdr->huffman_tables;
	static const u8 no_codes[16];
	bool grey = scan->num_components == 1;
	u32 mb_width = MB_WIDTH(frame->width);
	u32 mb_height = MB_HEIGHT(frame->height);
	u8 ta = scan->component[0].ac_entropy_coding_table_selector;
	u8 td = scan->component[0].dc_entropy_coding_table_selector;
	dma_addr_t ecs = src + hdr->ecs_offset;

	memset(regs, 0, sizeof(*regs));

	/* Decoder mode 3: JPEG. */
	regs->ctrl0 = G1_REG_DEC_CTRL0_DEC_MODE(3) |
		      G1_REG_DEC_CTRL0_FILTERING_DIS;

	regs->ctrl1 = G1_REG_DEC_CTRL1_PIC_MB_WIDTH(mb_width) |
		      G1_REG_DEC_CTRL1_PIC_MB_HEIGHT_P(mb_height) |
		      G1_REG_DEC_CTRL1_PIC_MB_W_EXT(mb_width >> 9) |
		      G1_REG_DEC_CTRL1_PIC_MB_H_EXT(mb_height >> 8);

	/*
	 * The stream starts at an 8-byte aligned address; the bit offset of
	 * the entropy-coded data in that first double word goes to
	 * STRM_START_BIT, and STREAM_LEN counts from the aligned address.
	 */
	regs->stream = ALIGN_DOWN(ecs, 8);
	regs->ctrl2 = G1_REG_DEC_CTRL2_STRM_START_BIT(8 * (ecs - regs->stream)) |
		      G1_REG_DEC_CTRL2_JPEG_MODE(hantro_g1_jpeg_dec_mode(frame->subsampling)) |
		      G1_REG_DEC_CTRL2_JPEG_QTABLES(scan->num_components) |
		      G1_REG_DEC_CTRL2_JPEG_STREAM_ALL;
	regs->ctrl3 = G1_REG_DEC_CTRL3_STREAM_LEN(payload - (regs->stream - src));

	/*
	 * The hardware holds the Huffman tables of the first component as
	 * table 1 and the other ones as table 2: a chroma component selects
	 * table 2 when its table is not the first component's.
	 */
	if (scan->num_components > 2) {
		if (scan->component[1].ac_entropy_coding_table_selector != ta)
			regs->ctrl2 |= G1_REG_DEC_CTRL2_CB_AC_VLCTABLE;
		if (scan->component[1].dc_entropy_coding_table_selector != td)
			regs->ctrl2 |= G1_REG_DEC_CTRL2_CB_DC_VLCTABLE;
		if (scan->component[2].ac_entropy_coding_table_selector != ta)
			regs->ctrl2 |= G1_REG_DEC_CTRL2_CR_AC_VLCTABLE;
		if (scan->component[2].dc_entropy_coding_table_selector != td)
			regs->ctrl2 |= G1_REG_DEC_CTRL2_CR_DC_VLCTABLE;
	}

	if (hdr->restart_interval) {
		regs->ctrl2 |= G1_REG_DEC_CTRL2_SYNC_MARKER_E;
		regs->ctrl5 = G1_REG_DEC_CTRL5_PJPEG_REST_FREQ(hdr->restart_interval);
	}

	/*
	 * A 4:2:2 MCU is 8 rows high, a greyscale one 8x8: when the last
	 * macroblock row (column) holds 8 picture rows (columns) or fewer,
	 * the stream has no MCUs for its lower (right) half and the decoder
	 * must fill it in.
	 */
	if ((grey || frame->subsampling == V4L2_JPEG_CHROMA_SUBSAMPLING_422) &&
	    fill_needed(frame->height))
		regs->ctrl4 |= G1_REG_DEC_CTRL4_PJPEG_FILDOWN_E;
	if (grey && fill_needed(frame->width))
		regs->ctrl2 |= G1_REG_DEC_CTRL2_JPEG_FILRIGHT_E;

	/* No slice mode: the whole picture in one job. */
	regs->jpeg_ctrl = G1_REG_JPEG_CTRL_SLICE_H(0);

	/* A greyscale picture has no second table. */
	vlc_code_counts(ht[2 + ta].start, grey ? no_codes : ht[2 + !ta].start,
			ht[td].start, grey ? no_codes : ht[!td].start, regs->vlc);
}

/*
 * The decoder takes three components as YCbCr. libjpeg, without a JFIF
 * marker, reads them as RGB when an Adobe APP14 segment says so (transform
 * 0), or when there is none and their identifiers are 'R', 'G', 'B'. The
 * parser does not report a JFIF marker, so the rare picture that has both
 * is refused too.
 */
static bool jpeg_is_rgb(const struct v4l2_jpeg_header *hdr)
{
	const struct v4l2_jpeg_frame_component_spec *c = hdr->frame.component;

	if (hdr->frame.num_components != 3)
		return false;

	if (hdr->app14_tf != V4L2_JPEG_APP14_TF_UNKNOWN)
		return hdr->app14_tf == V4L2_JPEG_APP14_TF_CMYK_RGB;

	return c[0].component_identifier == 'R' &&
	       c[1].component_identifier == 'G' &&
	       c[2].component_identifier == 'B';
}

/* The decoder needs at least three macroblocks per side. */
#define G1_JPEG_MIN_MBS		3

static int check_huffman_table(const struct v4l2_jpeg_reference *table,
			       unsigned int max_values)
{
	unsigned int i, codes = 0;

	if (!table->start || table->length < 16 + 1 ||
	    table->length > 16 + max_values)
		return -EINVAL;

	/*
	 * The parser counted the values from the code counts in the OUTPUT
	 * buffer, which userspace can change before they are copied: the
	 * copied counts must still add up to them.
	 */
	for (i = 0; i < 16; i++)
		codes += table->start[i];
	if (codes != table->length - 16)
		return -EINVAL;

	/*
	 * A valid table has at most 2^L - 1 codes of length L. The register
	 * fields of the lengths 1 to 7 are narrower than a byte and hold such
	 * counts; from length 8 on an AC field is a byte wide, and a DC
	 * table of at most 12 values fits its four-bit fields.
	 */
	for (i = 0; i < 7; i++)
		if (table->start[i] >= BIT(i + 1))
			return -EINVAL;

	return 0;
}

/**
 * hantro_g1_jpeg_dec_check() - check a parsed header against the hardware
 * @hdr:	parsed header, with the default Huffman tables filled in
 * @payload:	bytes used in the OUTPUT buffer, from its start
 * @src_width:	OUTPUT width
 * @src_height:	OUTPUT height
 *
 * The picture must have the size of the OUTPUT format, in macroblocks: the
 * CAPTURE format and the compose rectangle are set up for it.
 *
 * Return: 0 if the decoder can decode the picture, -EINVAL otherwise.
 */
VISIBLE_IF_KUNIT int hantro_g1_jpeg_dec_check(const struct v4l2_jpeg_header *hdr,
					      size_t payload, u32 src_width,
					      u32 src_height)
{
	const struct v4l2_jpeg_frame_header *frame = &hdr->frame;
	const struct v4l2_jpeg_scan_header *scan = hdr->scan;
	unsigned int i;

	if (frame->precision != 8) {
		vpu_debug(1, "unsupported precision %u\n", frame->precision);
		return -EINVAL;
	}

	if (MB_WIDTH(frame->width) < G1_JPEG_MIN_MBS ||
	    MB_HEIGHT(frame->height) < G1_JPEG_MIN_MBS ||
	    frame->width > HANTRO_G1_JPEG_DEC_MAX_SIZE ||
	    frame->height > HANTRO_G1_JPEG_DEC_MAX_SIZE ||
	    frame->width * frame->height > HANTRO_G1_JPEG_DEC_MAX_PIXELS) {
		vpu_debug(1, "unsupported size %ux%u\n",
			  frame->width, frame->height);
		return -EINVAL;
	}

	if (MB_WIDTH(frame->width) != MB_WIDTH(src_width) ||
	    MB_HEIGHT(frame->height) != MB_HEIGHT(src_height)) {
		vpu_debug(1, "size %ux%u does not match the OUTPUT %ux%u\n",
			  frame->width, frame->height, src_width, src_height);
		return -EINVAL;
	}

	if (frame->num_components !=
	    (frame->subsampling == V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY ? 1 : 3) ||
	    !hantro_g1_jpeg_dec_fourcc(frame->subsampling)) {
		vpu_debug(1, "%u components, subsampling %d\n",
			  frame->num_components, frame->subsampling);
		return -EINVAL;
	}

	/* One interleaved scan with all the components, in frame order. */
	if (scan->num_components != frame->num_components) {
		vpu_debug(1, "non-interleaved scan\n");
		return -EINVAL;
	}

	/*
	 * In a one-component scan the decoder does not count the restart
	 * interval: a restart marker it meets where it expects a block gives
	 * an empty block and moves the rest of the picture on by one. Only a
	 * marker at the end of a block row is taken without loss, so every
	 * restart must fall there.
	 */
	if (scan->num_components == 1 &&
	    hdr->restart_interval % DIV_ROUND_UP(frame->width, 8)) {
		vpu_debug(1, "one component, restart interval %u: not whole rows\n",
			  hdr->restart_interval);
		return -EINVAL;
	}

	/*
	 * A one-component picture that needs FILRIGHT but not FILDOWN comes
	 * out with its last block row written as zero, without an error: on
	 * the RK3026, 200 and 216 pixels wide and 400 high, with and without
	 * restart markers. With FILDOWN set as well the picture is right.
	 */
	if (scan->num_components == 1 && fill_needed(frame->width) &&
	    !fill_needed(frame->height)) {
		vpu_debug(1, "one component, %ux%u: fill right without fill down\n",
			  frame->width, frame->height);
		return -EINVAL;
	}

	for (i = 0; i < frame->num_components; i++) {
		const struct v4l2_jpeg_frame_component_spec *fc = &frame->component[i];
		const struct v4l2_jpeg_scan_component_spec *sc = &scan->component[i];
		u8 tq = fc->quantization_table_selector;
		u8 td = sc->dc_entropy_coding_table_selector;
		u8 ta = sc->ac_entropy_coding_table_selector;

		if (sc->component_selector != fc->component_identifier) {
			vpu_debug(1, "scan component %u out of order\n", i);
			return -EINVAL;
		}

		if (tq >= V4L2_JPEG_MAX_TABLES ||
		    !hdr->quantization_tables[tq].start ||
		    hdr->quantization_tables[tq].length != JPEG_QUANT_SIZE) {
			vpu_debug(1, "component %u: no 8-bit quantization table %u\n",
				  i, tq);
			return -EINVAL;
		}

		if (td > 1 || ta > 1 ||
		    check_huffman_table(&hdr->huffman_tables[td],
					HANTRO_JPEG_DEC_DC_SIZE) ||
		    check_huffman_table(&hdr->huffman_tables[2 + ta],
					HANTRO_JPEG_DEC_AC_SIZE)) {
			vpu_debug(1, "component %u: bad Huffman table DC%u/AC%u\n",
				  i, td, ta);
			return -EINVAL;
		}
	}

	if (jpeg_is_rgb(hdr)) {
		vpu_debug(1, "RGB components\n");
		return -EINVAL;
	}

	if (hdr->ecs_offset >= payload ||
	    payload - ALIGN_DOWN(hdr->ecs_offset, 8) > HANTRO_G1_JPEG_DEC_MAX_STREAM) {
		vpu_debug(1, "entropy-coded data at %zu of %zu bytes\n",
			  hdr->ecs_offset, payload);
		return -EINVAL;
	}

	return 0;
}

static void jpeg_dec_out(const struct hantro_ctx *ctx, u32 fourcc,
			 struct hantro_g1_jpeg_dec_out *out)
{
	out->src_width = ctx->src_fmt.width;
	out->src_height = ctx->src_fmt.height;
	out->fourcc = fourcc;
	out->width = ctx->dst_fmt.width;
	out->height = ctx->dst_fmt.height;
	out->compose = ctx->compose;
	out->pp_max_width = ctx->dev->variant->pp_max_width;
}

/* hantro_codec_ops.cap_try_fmt */
int hantro_g1_jpeg_dec_try_cap(const struct hantro_ctx *ctx, u32 fourcc,
			       u32 *width, u32 *height)
{
	struct hantro_g1_jpeg_dec_out out;

	jpeg_dec_out(ctx, fourcc, &out);
	if (!dec_fmt(fourcc) && !pp_fmt(&out))
		return -EINVAL;

	hantro_g1_jpeg_dec_cap_size(&out, width, height);
	return 0;
}

/* hantro_codec_ops.cap_enum_framesizes */
int hantro_g1_jpeg_dec_enum_cap(const struct hantro_ctx *ctx,
				struct v4l2_frmsizeenum *fsize)
{
	struct hantro_g1_jpeg_dec_out out;

	jpeg_dec_out(ctx, fsize->pixel_format, &out);
	if (!dec_fmt(out.fourcc) && !pp_fmt(&out))
		return -EINVAL;

	return hantro_g1_jpeg_dec_cap_frmsize(&out, fsize);
}

/*
 * hantro_codec_ops.cap_selection: the crop rectangle is the picture, as
 * the OUTPUT format has it; the compose rectangle can be set, in a buffer
 * the post-processor writes into, and is adjusted to what it writes:
 * -ERANGE if that breaks V4L2_SEL_FLAG_LE or V4L2_SEL_FLAG_GE.
 */
int hantro_g1_jpeg_dec_cap_selection(const struct hantro_ctx *ctx,
				     struct v4l2_selection *sel, bool set)
{
	struct hantro_g1_jpeg_dec_out out;

	jpeg_dec_out(ctx, ctx->dst_fmt.pixelformat, &out);

	if (set) {
		struct v4l2_rect want = sel->r;

		if (sel->target != V4L2_SEL_TGT_COMPOSE)
			return -EINVAL;
		hantro_g1_jpeg_dec_compose_adjust(&out, &sel->r);
		if ((sel->flags & V4L2_SEL_FLAG_LE &&
		     !v4l2_rect_enclosed(&sel->r, &want)) ||
		    (sel->flags & V4L2_SEL_FLAG_GE &&
		     !v4l2_rect_enclosed(&want, &sel->r)))
			return -ERANGE;
		return 0;
	}

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = out.src_width;
		sel->r.height = out.src_height;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		sel->r = out.compose;
		break;
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		hantro_g1_jpeg_dec_compose_default(&out, &sel->r);
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = out.width;
		sel->r.height = out.height;
		break;
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		hantro_g1_jpeg_dec_compose_padded(&out, &sel->r);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int hantro_g1_jpeg_dec_start(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct v4l2_jpeg_scan_header scan_header = { };
	struct v4l2_jpeg_reference quantization_tables[V4L2_JPEG_MAX_TABLES] = { };
	struct v4l2_jpeg_reference huffman_tables[V4L2_JPEG_MAX_TABLES] = { };
	struct v4l2_jpeg_header header = {
		.scan = &scan_header,
		.quantization_tables = quantization_tables,
		.huffman_tables = huffman_tables,
	};
	struct hantro_g1_jpeg_dec_out out;
	struct hantro_g1_jpeg_dec_regs regs;
	struct hantro_g1_pp_regs pp;
	dma_addr_t src_dma, dst_dma;
	unsigned int offset, i;
	size_t payload;
	bool use_pp;
	void *vaddr;
	int ret;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	/*
	 * Everything that can fail is checked before the job is prepared:
	 * a failing ->run() must not leave the watchdog armed.
	 *
	 * The header is read through the kernel mapping of the OUTPUT
	 * buffer. An imported DMABUF is not mapped: its exporter may refuse.
	 */
	if (src_buf->vb2_buf.memory != VB2_MEMORY_MMAP) {
		vpu_debug(1, "the OUTPUT buffer is not an MMAP buffer\n");
		return -EINVAL;
	}

	vaddr = vb2_plane_vaddr(&src_buf->vb2_buf, 0);
	if (!vaddr)
		return -EFAULT;

	/*
	 * The JPEG data starts data_offset bytes into the plane, which vb2
	 * keeps below the payload. The stream position counts from the start
	 * of the buffer, as the payload does.
	 */
	offset = src_buf->vb2_buf.planes[0].data_offset;
	payload = vb2_get_plane_payload(&src_buf->vb2_buf, 0);

	ret = v4l2_jpeg_parse_header(vaddr + offset, payload - offset, &header);
	if (ret < 0) {
		vpu_debug(1, "cannot parse the JPEG header: %d\n", ret);
		return ret;
	}
	header.ecs_offset += offset;

	/*
	 * Userspace can still write to the OUTPUT buffer: the checks and the
	 * hardware tables must see the same Huffman tables, so copy them.
	 */
	for (i = 0; i < V4L2_JPEG_MAX_TABLES; i++) {
		struct v4l2_jpeg_reference *ht = &header.huffman_tables[i];

		if (!ht->start)
			continue;
		if (ht->length > sizeof(ctx->jpeg_dec.huffman[i])) {
			vpu_debug(1, "Huffman table %u: %u bytes\n", i,
				  ht->length);
			return -EINVAL;
		}
		memcpy(ctx->jpeg_dec.huffman[i], ht->start, ht->length);
		ht->start = ctx->jpeg_dec.huffman[i];
	}

	hantro_jpeg_get_default_huffman_tables(header.huffman_tables);

	ret = hantro_g1_jpeg_dec_check(&header, payload, ctx->src_fmt.width,
				       ctx->src_fmt.height);
	if (ret)
		return ret;

	/*
	 * The formats and the compose rectangle are read without the ioctl
	 * lock: S_FMT and S_SELECTION leave them as they are while the
	 * CAPTURE queue has buffers, and each of them holds the CAPTURE
	 * format (hantro_queue_setup() and hantro_buf_prepare() check).
	 */
	jpeg_dec_out(ctx, ctx->dst_fmt.pixelformat, &out);
	ret = hantro_g1_jpeg_dec_job_out(&out, header.frame.subsampling, &pp,
					 &use_pp);
	if (ret)
		return ret;

	ret = hantro_jpeg_prepare_vlc_hw_table(&header, ctx->jpeg_dec.priv.cpu);
	if (ret) {
		vpu_debug(1, "cannot build the hardware tables: %d\n", ret);
		return ret;
	}

	src_dma = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	dst_dma = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	hantro_g1_jpeg_dec_regs(&header, src_dma, payload, &regs);

	hantro_start_prepare_run(ctx);

	/*
	 * The post-processor, in pipeline mode, takes the decoded picture
	 * and writes the CAPTURE buffer; the decoder's own output is off, so
	 * it needs no buffer of the picture's size.
	 */
	if (use_pp) {
		hantro_g1_pp_write(vpu, &pp, dst_dma);
		regs.ctrl0 |= G1_REG_DEC_CTRL0_DEC_OUT_DIS;
	}

	vdpu_write_relaxed(vpu, regs.ctrl0, G1_REG_DEC_CTRL0);
	vdpu_write_relaxed(vpu, regs.ctrl1, G1_REG_DEC_CTRL1);
	vdpu_write_relaxed(vpu, regs.ctrl2, G1_REG_DEC_CTRL2);
	vdpu_write_relaxed(vpu, regs.ctrl3, G1_REG_DEC_CTRL3);
	vdpu_write_relaxed(vpu, regs.ctrl4, G1_REG_DEC_CTRL4);
	vdpu_write_relaxed(vpu, regs.ctrl5, G1_REG_DEC_CTRL5);
	vdpu_write_relaxed(vpu, regs.jpeg_ctrl, G1_REG_JPEG_CTRL);
	for (i = 0; i < ARRAY_SIZE(regs.vlc); i++)
		vdpu_write_relaxed(vpu, regs.vlc[i],
				   G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_A + 4 * i);

	vdpu_write_relaxed(vpu, regs.stream, G1_REG_ADDR_STR);
	if (use_pp) {
		vdpu_write_relaxed(vpu, 0, G1_REG_ADDR_DST);
		vdpu_write_relaxed(vpu, 0, G1_REG_ADDR_DST_CHROMA);
	} else {
		vdpu_write_relaxed(vpu, dst_dma, G1_REG_ADDR_DST);
		/* A greyscale picture has no chroma plane. */
		if (header.frame.num_components > 1)
			vdpu_write_relaxed(vpu, dst_dma +
					   ctx->dst_fmt.plane_fmt[0].bytesperline *
					   ctx->dst_fmt.height,
					   G1_REG_ADDR_DST_CHROMA);
	}
	vdpu_write_relaxed(vpu, ctx->jpeg_dec.priv.dma, G1_REG_ADDR_QTABLE);

	hantro_end_prepare_run(ctx);

	/* Start decoding! */
	vdpu_write_relaxed(vpu,
			   G1_REG_CONFIG_DEC_AXI_RD_ID(0xffu) |
			   G1_REG_CONFIG_DEC_TIMEOUT_E |
			   G1_REG_CONFIG_DEC_OUT_ENDIAN |
			   G1_REG_CONFIG_DEC_STRENDIAN_E |
			   G1_REG_CONFIG_DEC_MAX_BURST(16) |
			   G1_REG_CONFIG_DEC_OUTSWAP32_E |
			   G1_REG_CONFIG_DEC_INSWAP32_E |
			   G1_REG_CONFIG_DEC_STRSWAP32_E |
			   G1_REG_CONFIG_DEC_CLK_GATE_E,
			   G1_REG_CONFIG);
	vdpu_write(vpu, G1_REG_INTERRUPT_DEC_E, G1_REG_INTERRUPT);

	return 0;
}

int hantro_g1_jpeg_dec_run(struct hantro_ctx *ctx)
{
	struct vb2_v4l2_buffer *src_buf = hantro_get_src_buf(ctx);
	int ret;

	ret = hantro_g1_jpeg_dec_start(ctx);
	/*
	 * A refused job ends before hantro_end_prepare_run(), which completes
	 * the request of the OUTPUT buffer: complete it here.
	 */
	if (ret)
		v4l2_ctrl_request_complete(src_buf->vb2_buf.req_obj.req,
					   &ctx->ctrl_handler);

	return ret;
}

/*
 * The decoder's interrupt ends a job with the post-processor in pipeline
 * mode too: leave pipeline mode. After a job that fails, the next job of
 * any context sets the post-processor up, or turns pipeline mode off, for
 * itself.
 */
void hantro_g1_jpeg_dec_done(struct hantro_ctx *ctx)
{
	vdpu_write(ctx->dev, 0, G1_REG_PP_INTERRUPT);
}
