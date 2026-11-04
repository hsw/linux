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
#include "hantro.h"
#include "hantro_g1_regs.h"
#include "hantro_hw.h"
#include "hantro_jpeg.h"

VISIBLE_IF_KUNIT int hantro_g1_jpeg_dec_mode(int subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		return 2;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return 3;
	default:
		return -EINVAL;
	}
}

VISIBLE_IF_KUNIT u32 hantro_g1_jpeg_dec_fourcc(int subsampling)
{
	switch (subsampling) {
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

/**
 * hantro_g1_jpeg_dec_regs() - compute the registers of a decoding job
 * @hdr:	parsed header that passed hantro_g1_jpeg_dec_check()
 * @src:	bus address of the OUTPUT buffer
 * @payload:	bytes of JPEG data in the OUTPUT buffer
 * @regs:	the register values
 */
VISIBLE_IF_KUNIT void hantro_g1_jpeg_dec_regs(const struct v4l2_jpeg_header *hdr,
					      dma_addr_t src, size_t payload,
					      struct hantro_g1_jpeg_dec_regs *regs)
{
	const struct v4l2_jpeg_frame_header *frame = &hdr->frame;
	const struct v4l2_jpeg_scan_header *scan = hdr->scan;
	const struct v4l2_jpeg_reference *ht = hdr->huffman_tables;
	u32 mb_width = MB_WIDTH(frame->width);
	u32 mb_height = MB_HEIGHT(frame->height);
	u8 ta = scan->component[0].ac_entropy_coding_table_selector;
	u8 td = scan->component[0].dc_entropy_coding_table_selector;
	dma_addr_t ecs = src + hdr->ecs_offset;

	memset(regs, 0, sizeof(*regs));

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

	if (hdr->restart_interval)
		regs->ctrl2 |= G1_REG_DEC_CTRL2_SYNC_MARKER_E;
	regs->ctrl5 = G1_REG_DEC_CTRL5_PJPEG_REST_FREQ(hdr->restart_interval);

	/*
	 * A 4:2:2 MCU is 8 rows high: when the last macroblock row holds 8
	 * picture rows or fewer, the stream has no MCUs for its lower half
	 * and the decoder must fill it in.
	 */
	if (frame->subsampling == V4L2_JPEG_CHROMA_SUBSAMPLING_422 &&
	    fill_needed(frame->height))
		regs->ctrl4 |= G1_REG_DEC_CTRL4_PJPEG_FILDOWN_E;

	/* No slice mode: the whole picture in one job. */
	regs->jpeg_ctrl = G1_REG_JPEG_CTRL_SLICE_H(0);

	vlc_code_counts(ht[2 + ta].start, ht[2 + !ta].start,
			ht[td].start, ht[!td].start, regs->vlc);
}

/* The decoder needs at least three macroblocks per side. */
#define G1_JPEG_MIN_MBS		3
/*
 * Largest picture the decoder takes in one job, without slice mode: at most
 * 8176 pixels per side and 4672x3504 pixels in total.
 */
#define G1_JPEG_MAX_DIM		8176
#define G1_JPEG_MAX_PIXELS	(4672 * 3504)
/* STREAM_LEN is 24 bits wide. */
#define G1_JPEG_MAX_STREAM_LEN	0xffffff

static int check_huffman_table(const struct v4l2_jpeg_reference *table,
			       unsigned int max_values)
{
	unsigned int i;

	if (!table->start || table->length < 16 + 1 ||
	    table->length > 16 + max_values)
		return -EINVAL;

	/*
	 * The code counts go to register fields as wide as a valid table
	 * needs: at most 2^L - 1 codes of length L.
	 */
	for (i = 0; i < 7; i++)
		if (table->start[i] >= BIT(i + 1))
			return -EINVAL;

	return 0;
}

/**
 * hantro_g1_jpeg_dec_check() - check a parsed header against the hardware
 * @hdr:	parsed header, with the default Huffman tables filled in
 * @payload:	bytes of JPEG data in the OUTPUT buffer
 * @fourcc:	CAPTURE pixel format
 * @width:	CAPTURE width
 * @height:	CAPTURE height
 *
 * Return: 0 if the decoder can decode the picture into the CAPTURE format,
 * -EINVAL otherwise.
 */
VISIBLE_IF_KUNIT int hantro_g1_jpeg_dec_check(const struct v4l2_jpeg_header *hdr,
					      size_t payload, u32 fourcc,
					      u32 width, u32 height)
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
	    frame->width > G1_JPEG_MAX_DIM || frame->height > G1_JPEG_MAX_DIM ||
	    frame->width * frame->height > G1_JPEG_MAX_PIXELS) {
		vpu_debug(1, "unsupported size %ux%u\n",
			  frame->width, frame->height);
		return -EINVAL;
	}

	/*
	 * The decoder writes whole macroblocks: the CAPTURE size is the
	 * picture size rounded up to them.
	 */
	if (!IS_ALIGNED(width, MB_DIM) || !IS_ALIGNED(height, MB_DIM) ||
	    MB_WIDTH(frame->width) != MB_WIDTH(width) ||
	    MB_HEIGHT(frame->height) != MB_HEIGHT(height)) {
		vpu_debug(1, "size %ux%u does not match the CAPTURE %ux%u\n",
			  frame->width, frame->height, width, height);
		return -EINVAL;
	}

	if (frame->num_components != 3 ||
	    hantro_g1_jpeg_dec_fourcc(frame->subsampling) != fourcc) {
		vpu_debug(1, "%u components, subsampling %d: not %p4cc\n",
			  frame->num_components, frame->subsampling, &fourcc);
		return -EINVAL;
	}

	/* One interleaved scan with all the components, in frame order. */
	if (scan->num_components != frame->num_components) {
		vpu_debug(1, "non-interleaved scan\n");
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

	if (hdr->ecs_offset >= payload ||
	    payload - ALIGN_DOWN(hdr->ecs_offset, 8) > G1_JPEG_MAX_STREAM_LEN) {
		vpu_debug(1, "entropy-coded data at %zu of %zu bytes\n",
			  hdr->ecs_offset, payload);
		return -EINVAL;
	}

	return 0;
}

int hantro_g1_jpeg_dec_run(struct hantro_ctx *ctx)
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
	struct hantro_g1_jpeg_dec_regs regs;
	dma_addr_t src_dma, dst_dma;
	size_t payload;
	unsigned int i;
	void *vaddr;
	int ret;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	/*
	 * Everything that can fail is checked before the job is prepared:
	 * a failing ->run() must not leave the watchdog armed.
	 */
	vaddr = vb2_plane_vaddr(&src_buf->vb2_buf, 0);
	if (!vaddr)
		return -EFAULT;

	payload = vb2_get_plane_payload(&src_buf->vb2_buf, 0);

	ret = v4l2_jpeg_parse_header(vaddr, payload, &header);
	if (ret < 0) {
		vpu_debug(1, "cannot parse the JPEG header: %d\n", ret);
		return ret;
	}

	hantro_jpeg_get_default_huffman_tables(header.huffman_tables);

	ret = hantro_g1_jpeg_dec_check(&header, payload,
				       ctx->dst_fmt.pixelformat,
				       ctx->dst_fmt.width, ctx->dst_fmt.height);
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
	vdpu_write_relaxed(vpu, dst_dma, G1_REG_ADDR_DST);
	vdpu_write_relaxed(vpu, dst_dma + ctx->dst_fmt.plane_fmt[0].bytesperline *
				ctx->dst_fmt.height, G1_REG_ADDR_DST_CHROMA);
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
