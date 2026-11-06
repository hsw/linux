// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro G1 post-processor support
 *
 * Copyright (C) 2019 Collabora, Ltd.
 */

#include <kunit/visibility.h>
#include <linux/dma-mapping.h>
#include <linux/math64.h>
#include <linux/types.h>

#include "hantro.h"
#include "hantro_hw.h"
#include "hantro_g1_regs.h"
#include "hantro_g2_regs.h"
#include "hantro_v4l2.h"

#define VPU_PP_IN_YUYV			0x0
#define VPU_PP_IN_NV12			0x1
#define VPU_PP_IN_YUV420		0x2
#define VPU_PP_IN_YUV240_TILED		0x5
#define VPU_PP_OUT_RGB			0x0
#define VPU_PP_OUT_YUYV			0x3
#define VPU_PP_OUT_NV12			0x5

/* G1_REG_PP_SCALING1 scaling mode */
#define VPU_PP_SCALE_DOWN		2

/**
 * struct hantro_g1_pp_fmt - an output format of the G1 post-processor
 * @fourcc:	V4L2 pixel format
 * @out_fmt:	G1_REG_PP_CONTROL_OUT_FMT() value
 * @cpp:	bytes per pixel of the first plane
 */
struct hantro_g1_pp_fmt {
	u32 fourcc;
	u32 out_fmt;
	u8 cpp;
};

static const struct hantro_g1_pp_fmt hantro_g1_pp_fmts[] = {
	{
		/* CbCr after the luma plane, at the same stride */
		.fourcc = V4L2_PIX_FMT_NV12,
		.out_fmt = VPU_PP_OUT_NV12,
		.cpp = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.out_fmt = VPU_PP_OUT_YUYV,
		.cpp = 2,
	},
};

static const struct hantro_g1_pp_fmt *hantro_g1_pp_find(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hantro_g1_pp_fmts); i++)
		if (hantro_g1_pp_fmts[i].fourcc == fourcc)
			return &hantro_g1_pp_fmts[i];

	return NULL;
}

/* An inverse scaling ratio, out / in with 16 fractional bits, below 1. */
static u32 hantro_g1_pp_inv_ratio(u32 out, u32 in)
{
	return div_u64((u64)out << 16, in);
}

/**
 * hantro_g1_pp_regs() - compute the G1 post-processor registers of a
 *			 picture
 * @cfg:	the decoded picture and the output
 * @regs:	the register values
 *
 * The post-processor reads @cfg->in_width x @cfg->in_height pixels of the
 * picture the decoder writes, scales them down to the output size in each
 * direction where that is smaller, and stores them in the output format at
 * @cfg->left, @cfg->top of the buffer. The device configuration is the one
 * the video post-processing has always used: OUT_ENDIAN and OUT_SWAP32_E
 * set, which store YUYV in its memory order, and bursts of 16.
 *
 * The caller keeps the sizes and the position within the post-processor's
 * limits (HANTRO_G1_PP_*): the output no larger than the input, the input
 * a multiple of 8, the output width and the stride in steps of 8 pixels,
 * @cfg->left at a multiple of 8 bytes, the output height and @cfg->top
 * even.
 *
 * Return: 0, or -EINVAL for a format the post-processor does not write.
 */
int hantro_g1_pp_regs(const struct hantro_g1_pp_cfg *cfg,
		      struct hantro_g1_pp_regs *regs)
{
	const struct hantro_g1_pp_fmt *fmt = hantro_g1_pp_find(cfg->fourcc);
	u32 in_mb_width = DIV_ROUND_UP(cfg->in_width, MB_DIM);
	u32 in_mb_height = DIV_ROUND_UP(cfg->in_height, MB_DIM);

	memset(regs, 0, sizeof(*regs));
	if (!fmt)
		return -EINVAL;

	regs->dev_config = G1_REG_PP_OUT_ENDIAN(1) |
			   G1_REG_PP_OUTSWAP32_E(1) |
			   G1_REG_PP_MAX_BURST(16);
	regs->input_size = G1_REG_PP_INPUT_SIZE_WIDTH(in_mb_width) |
			   G1_REG_PP_INPUT_SIZE_HEIGHT(in_mb_height);
	regs->control = G1_REG_PP_CONTROL_IN_FMT(cfg->in_fmt) |
			G1_REG_PP_CONTROL_OUT_FMT(fmt->out_fmt) |
			G1_REG_PP_CONTROL_OUT_HEIGHT(cfg->height) |
			G1_REG_PP_CONTROL_OUT_WIDTH(cfg->width);
	/*
	 * The input fields count macroblocks, the last one included where
	 * the input ends in its middle; a crop bit drops the last 8 columns
	 * (rows) of that macroblock from what is read.
	 */
	if (cfg->in_width % MB_DIM)
		regs->control |= G1_REG_PP_CONTROL_CROP8_R_E;
	if (cfg->in_height % MB_DIM)
		regs->control |= G1_REG_PP_CONTROL_CROP8_D_E;
	regs->orig_width = G1_REG_PP_ORIG_WIDTH(cfg->orig_mb_width);
	regs->display_width = G1_REG_PP_DISPLAY_WIDTH(cfg->stride) |
			      G1_REG_PP_IN_W_EXT(in_mb_width >> 9) |
			      G1_REG_PP_IN_H_EXT(in_mb_height >> 8);

	/*
	 * A direction scaled down takes the inverse ratio, out / in. The
	 * vertical one is programmed a unit above the truncated quotient,
	 * the horizontal one is not; why the vertical scaler needs it is
	 * not documented.
	 */
	if (cfg->width < cfg->in_width) {
		regs->scaling1 |= G1_REG_PP_HOR_SCALE_MODE(VPU_PP_SCALE_DOWN);
		regs->scaling_inv |=
			G1_REG_PP_WSCALE_INVRA(hantro_g1_pp_inv_ratio(cfg->width,
								      cfg->in_width));
	}
	if (cfg->height < cfg->in_height) {
		regs->scaling1 |= G1_REG_PP_VER_SCALE_MODE(VPU_PP_SCALE_DOWN);
		regs->scaling_inv |=
			G1_REG_PP_HSCALE_INVRA(hantro_g1_pp_inv_ratio(cfg->height,
								      cfg->in_height) + 1);
	}

	regs->luma_offset = (cfg->top * cfg->stride + cfg->left) * fmt->cpp;
	/* NV12: two CbCr bytes for every two pixels of every second line. */
	if (fmt->out_fmt == VPU_PP_OUT_NV12)
		regs->chroma_offset = cfg->stride * cfg->buf_height +
				      cfg->top / 2 * cfg->stride + cfg->left;

	return 0;
}

/**
 * hantro_g1_pp_write() - set the G1 post-processor up for the next job
 * @vpu:	the device
 * @regs:	registers from hantro_g1_pp_regs()
 * @dst:	bus address of the output buffer
 *
 * Pipeline mode goes on first, with the post-processor's own interrupt
 * masked: the decoder's interrupt ends the job. The other registers are
 * written whole, those the output does not use as 0, so nothing is left
 * over from a job with another set-up.
 */
void hantro_g1_pp_write(struct hantro_dev *vpu,
			const struct hantro_g1_pp_regs *regs, dma_addr_t dst)
{
	vdpu_write(vpu, G1_REG_PP_PIPELINE_EN | G1_REG_PP_IRQ_DIS,
		   G1_REG_PP_INTERRUPT);
	vdpu_write_relaxed(vpu, regs->dev_config, G1_REG_PP_DEV_CONFIG);
	vdpu_write_relaxed(vpu, dst + regs->luma_offset,
			   G1_REG_PP_OUT_LUMA_BASE);
	vdpu_write_relaxed(vpu, regs->chroma_offset ?
			   dst + regs->chroma_offset : 0,
			   G1_REG_PP_OUT_CHROMA_BASE);
	/* No contrast, colour conversion or masks. */
	vdpu_write_relaxed(vpu, 0, G1_REG_PP_CONTRAST_ADJUST);
	vdpu_write_relaxed(vpu, 0, G1_REG_PP_COLOR_CONVERSION);
	vdpu_write_relaxed(vpu, 0, G1_REG_PP_COLOR_CONVERSION0);
	vdpu_write_relaxed(vpu, 0, G1_REG_PP_COLOR_CONVERSION1);
	vdpu_write_relaxed(vpu, regs->input_size, G1_REG_PP_INPUT_SIZE);
	vdpu_write_relaxed(vpu, 0, G1_REG_PP_SCALING0);
	vdpu_write_relaxed(vpu, regs->scaling1, G1_REG_PP_SCALING1);
	vdpu_write_relaxed(vpu, regs->scaling_inv, G1_REG_PP_SCALING_INV);
	vdpu_write_relaxed(vpu, 0, G1_REG_PP_MASK_R);
	vdpu_write_relaxed(vpu, 0, G1_REG_PP_MASK_G);
	vdpu_write_relaxed(vpu, 0, G1_REG_PP_MASK_B);
	vdpu_write_relaxed(vpu, regs->control, G1_REG_PP_CONTROL);
	vdpu_write_relaxed(vpu, regs->orig_width, G1_REG_PP_MASK1_ORIG_WIDTH);
	vdpu_write_relaxed(vpu, regs->display_width,
			   G1_REG_PP_DISPLAY_WIDTH_IN_EXT);
}

bool hantro_needs_postproc(const struct hantro_ctx *ctx,
			   const struct hantro_fmt *fmt)
{
	if (ctx->is_encoder)
		return false;

	if (ctx->need_postproc)
		return true;

	return fmt->postprocessed;
}

/*
 * A video decoder's picture at its own size, whole macroblocks, as the
 * CAPTURE format: the post-processor converts NV12 to it.
 */
VISIBLE_IF_KUNIT int hantro_g1_pp_video_regs(u32 fourcc, u32 width, u32 height,
					     struct hantro_g1_pp_regs *regs)
{
	struct hantro_g1_pp_cfg cfg = {
		.in_fmt = VPU_PP_IN_NV12,
		.in_width = MB_WIDTH(width) * MB_DIM,
		.in_height = MB_HEIGHT(height) * MB_DIM,
		.orig_mb_width = MB_WIDTH(width),
		.fourcc = fourcc,
		.width = width,
		.height = height,
		.stride = width,
		.buf_height = height,
	};

	return hantro_g1_pp_regs(&cfg, regs);
}

static void hantro_postproc_g1_enable(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct vb2_v4l2_buffer *dst_buf;
	struct hantro_g1_pp_regs regs;
	dma_addr_t dst_dma;

	if (hantro_g1_pp_video_regs(ctx->vpu_dst_fmt->fourcc,
				    ctx->dst_fmt.width, ctx->dst_fmt.height,
				    &regs)) {
		WARN(1, "output format %p4cc not supported by the post-processor, this wasn't expected.",
		     &ctx->vpu_dst_fmt->fourcc);
		return;
	}

	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	dst_dma = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	hantro_g1_pp_write(vpu, &regs, dst_dma);
}

static int down_scale_factor(struct hantro_ctx *ctx)
{
	if (ctx->src_fmt.width <= ctx->dst_fmt.width)
		return 0;

	return DIV_ROUND_CLOSEST(ctx->src_fmt.width, ctx->dst_fmt.width);
}

static void hantro_postproc_g2_enable(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct vb2_v4l2_buffer *dst_buf;
	int down_scale = down_scale_factor(ctx);
	int out_depth;
	size_t chroma_offset;
	dma_addr_t dst_dma;

	dst_buf = hantro_get_dst_buf(ctx);
	dst_dma = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	chroma_offset = ctx->dst_fmt.plane_fmt[0].bytesperline *
			ctx->dst_fmt.height;

	if (down_scale) {
		hantro_reg_write(vpu, &g2_down_scale_e, 1);
		hantro_reg_write(vpu, &g2_down_scale_y, down_scale >> 2);
		hantro_reg_write(vpu, &g2_down_scale_x, down_scale >> 2);
		hantro_write_addr(vpu, G2_DS_DST, dst_dma);
		hantro_write_addr(vpu, G2_DS_DST_CHR, dst_dma + (chroma_offset >> down_scale));
	} else {
		hantro_write_addr(vpu, G2_RS_OUT_LUMA_ADDR, dst_dma);
		hantro_write_addr(vpu, G2_RS_OUT_CHROMA_ADDR, dst_dma + chroma_offset);
	}

	out_depth = hantro_get_format_depth(ctx->dst_fmt.pixelformat);
	if (ctx->dev->variant->legacy_regs) {
		u8 pp_shift = 0;

		if (out_depth > 8)
			pp_shift = 16 - out_depth;

		hantro_reg_write(ctx->dev, &g2_rs_out_bit_depth, out_depth);
		hantro_reg_write(ctx->dev, &g2_pp_pix_shift, pp_shift);
	} else {
		hantro_reg_write(vpu, &g2_output_8_bits, out_depth > 8 ? 0 : 1);
		hantro_reg_write(vpu, &g2_output_format, out_depth > 8 ? 1 : 0);
	}
	hantro_reg_write(vpu, &g2_out_rs_e, 1);
}

static int hantro_postproc_g2_enum_framesizes(struct hantro_ctx *ctx,
					      struct v4l2_frmsizeenum *fsize)
{
	/**
	 * G2 scaler can scale down by 0, 2, 4 or 8
	 * use fsize->index has power of 2 diviser
	 **/
	if (fsize->index > 3)
		return -EINVAL;

	if (!ctx->src_fmt.width || !ctx->src_fmt.height)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ctx->src_fmt.width >> fsize->index;
	fsize->discrete.height = ctx->src_fmt.height >> fsize->index;

	return 0;
}

void hantro_postproc_free(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_queue *queue = &m2m_ctx->cap_q_ctx.q;
	unsigned int i;

	for (i = 0; i < queue->max_num_buffers; ++i) {
		struct hantro_aux_buf *priv = &ctx->postproc.dec_q[i];

		if (priv->cpu) {
			dma_free_attrs(vpu->dev, priv->size, priv->cpu,
				       priv->dma, priv->attrs);
			priv->cpu = NULL;
		}
	}
}

static unsigned int hantro_postproc_buffer_size(struct hantro_ctx *ctx)
{
	unsigned int buf_size;

	buf_size = ctx->ref_fmt.plane_fmt[0].sizeimage;
	if (ctx->vpu_src_fmt->fourcc == V4L2_PIX_FMT_H264_SLICE)
		buf_size += hantro_h264_mv_size(ctx->ref_fmt.width,
						ctx->ref_fmt.height);
	else if (ctx->vpu_src_fmt->fourcc == V4L2_PIX_FMT_VP9_FRAME)
		buf_size += hantro_vp9_mv_size(ctx->ref_fmt.width,
					       ctx->ref_fmt.height);
	else if (ctx->vpu_src_fmt->fourcc == V4L2_PIX_FMT_HEVC_SLICE) {
		buf_size += hantro_hevc_mv_size(ctx->ref_fmt.width,
						ctx->ref_fmt.height);
		if (ctx->hevc_dec.use_compression)
			buf_size += hantro_hevc_compressed_size(ctx->ref_fmt.width,
								ctx->ref_fmt.height);
	}
	else if (ctx->vpu_src_fmt->fourcc == V4L2_PIX_FMT_AV1_FRAME)
		buf_size += hantro_av1_mv_size(ctx->ref_fmt.width,
					       ctx->ref_fmt.height);

	return buf_size;
}

static int hantro_postproc_alloc(struct hantro_ctx *ctx, int index)
{
	struct hantro_dev *vpu = ctx->dev;
	struct hantro_aux_buf *priv = &ctx->postproc.dec_q[index];
	unsigned int buf_size = hantro_postproc_buffer_size(ctx);

	if (!buf_size)
		return -EINVAL;

	/*
	 * The buffers on this queue are meant as intermediate
	 * buffers for the decoder, so no mapping is needed.
	 */
	priv->attrs = DMA_ATTR_NO_KERNEL_MAPPING;
	priv->cpu = dma_alloc_attrs(vpu->dev, buf_size, &priv->dma,
				    GFP_KERNEL, priv->attrs);
	if (!priv->cpu)
		return -ENOMEM;
	priv->size = buf_size;

	return 0;
}

int hantro_postproc_init(struct hantro_ctx *ctx)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_queue *cap_queue = &m2m_ctx->cap_q_ctx.q;
	unsigned int num_buffers = vb2_get_num_buffers(cap_queue);
	unsigned int i;
	int ret;

	for (i = 0; i < num_buffers; i++) {
		ret = hantro_postproc_alloc(ctx, i);
		if (ret) {
			hantro_postproc_free(ctx);
			return ret;
		}
	}

	return 0;
}

dma_addr_t
hantro_postproc_get_dec_buf_addr(struct hantro_ctx *ctx, int index)
{
	struct hantro_aux_buf *priv = &ctx->postproc.dec_q[index];
	unsigned int buf_size = hantro_postproc_buffer_size(ctx);
	struct hantro_dev *vpu = ctx->dev;
	int ret;

	if (priv->size < buf_size && priv->cpu) {
		/* buffer is too small, release it */
		dma_free_attrs(vpu->dev, priv->size, priv->cpu,
			       priv->dma, priv->attrs);
		priv->cpu = NULL;
	}

	if (!priv->cpu) {
		/* buffer not already allocated, try getting a new one */
		ret = hantro_postproc_alloc(ctx, index);
		if (ret)
			return 0;
	}

	if (!priv->cpu)
		return 0;

	return priv->dma;
}

static void hantro_postproc_g1_disable(struct hantro_ctx *ctx)
{
	static const struct hantro_reg pipeline_en = {
		G1_REG_PP_INTERRUPT, 1, 0x1
	};

	hantro_reg_write(ctx->dev, &pipeline_en, 0);
}

static void hantro_postproc_g2_disable(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;

	hantro_reg_write(vpu, &g2_out_rs_e, 0);
}

void hantro_postproc_disable(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;

	if (vpu->variant->postproc_ops && vpu->variant->postproc_ops->disable)
		vpu->variant->postproc_ops->disable(ctx);
}

void hantro_postproc_enable(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;

	if (vpu->variant->postproc_ops && vpu->variant->postproc_ops->enable)
		vpu->variant->postproc_ops->enable(ctx);
}

int hanto_postproc_enum_framesizes(struct hantro_ctx *ctx,
				   struct v4l2_frmsizeenum *fsize)
{
	struct hantro_dev *vpu = ctx->dev;

	if (vpu->variant->postproc_ops && vpu->variant->postproc_ops->enum_framesizes)
		return vpu->variant->postproc_ops->enum_framesizes(ctx, fsize);

	return -EINVAL;
}

const struct hantro_postproc_ops hantro_g1_postproc_ops = {
	.enable = hantro_postproc_g1_enable,
	.disable = hantro_postproc_g1_disable,
};

const struct hantro_postproc_ops hantro_g2_postproc_ops = {
	.enable = hantro_postproc_g2_enable,
	.disable = hantro_postproc_g2_disable,
	.enum_framesizes = hantro_postproc_g2_enum_framesizes,
};
