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

static void set_buffers(struct hantro_dev *vpu, struct hantro_ctx *ctx,
			struct vb2_buffer *src_buf,
			struct vb2_buffer *dst_buf,
			struct v4l2_jpeg_header *header)
{
	dma_addr_t src_dma, dst_dma;

	/* Source (stream) buffer. */
	src_dma = vb2_dma_contig_plane_dma_addr(src_buf, 0);

	/* Skip header */
	src_dma += header->ecs_offset;

	vdpu_write_relaxed(vpu, src_dma, G1_REG_ADDR_STR);

	/* Destination (decoded frame) buffer. */
	dst_dma = hantro_get_dec_buf_addr(ctx, dst_buf);
	vdpu_write_relaxed(vpu, dst_dma, G1_REG_ADDR_DST);
	vdpu_write_relaxed(vpu, dst_dma +
				ctx->dst_fmt.height * ctx->dst_fmt.width,
			   G1_REG_ADDR_DST_CHROMA);

	/* Auxiliary buffer prepared in hantro_jpeg_prepare_vlc_hw_table(). */
	vdpu_write_relaxed(vpu, ctx->jpeg_dec.priv.dma, G1_REG_ADDR_QTABLE);
}

static int to_jpeg_mode(int subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return 3;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		return 2;
	/*
	 * Currently unsupported:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_411:
	 */
	default:
		return 2;
	}
}

VISIBLE_IF_KUNIT u32 hantro_g1_jpeg_dec_fourcc(int subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return V4L2_PIX_FMT_NV16;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		return V4L2_PIX_FMT_NV12;
	/*
	 * Currently unsupported:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
	 * case V4L2_JPEG_CHROMA_SUBSAMPLING_411:
	 */
	default:
		return 0;
	}
}

static int write_vlc_code_lengths(struct hantro_ctx *ctx,
				  struct v4l2_jpeg_header *header)
{
	struct hantro_dev *vpu = ctx->dev;
	u32 reg;
	u8 *bits1;
	u8 *bits2;

	if (header->huffman_tables[0].length < 16 ||
	    header->huffman_tables[1].length < 16 ||
	    header->huffman_tables[2].length < 16 ||
	    header->huffman_tables[3].length < 16) {
		dev_err(vpu->dev, "Invalid huffman table size: at least 16 bytes expected");
		return -EINVAL;
	}

	/* First 16 bits are BITS code lengths */
	bits1 = (u8 *)header->huffman_tables[2].start;/* AC luma */
	bits2 = (u8 *)header->huffman_tables[3].start;/* AC chroma  */

	/* Hardware requires that AC1 registers contains luma table */
	if (header->scan->component[0].ac_entropy_coding_table_selector == 1) /* Ta[0] == 1 */
		swap(bits1, bits2);

	/* AC1 table code lengths (luma) */
	reg = G1_REG_DEC_JPEG_VLC_AC1_CODE1_CNT(bits1[0]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE2_CNT(bits1[1]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE3_CNT(bits1[2]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE4_CNT(bits1[3]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE5_CNT(bits1[4]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE6_CNT(bits1[5]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_A);

	reg = G1_REG_DEC_JPEG_VLC_AC1_CODE7_CNT(bits1[6]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE8_CNT(bits1[7]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE9_CNT(bits1[8]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE10_CNT(bits1[9]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_B);

	reg = G1_REG_DEC_JPEG_VLC_AC1_CODE11_CNT(bits1[10]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE12_CNT(bits1[11]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE13_CNT(bits1[12]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE14_CNT(bits1[13]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_C);

	reg = G1_REG_DEC_JPEG_VLC_AC1_CODE15_CNT(bits1[14]) |
	      G1_REG_DEC_JPEG_VLC_AC1_CODE16_CNT(bits1[15]) |

	/* AC2 table code lengths (the not-luma) */
	      G1_REG_DEC_JPEG_VLC_AC2_CODE1_CNT(bits2[0]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE2_CNT(bits2[1]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE3_CNT(bits2[2]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE4_CNT(bits2[3]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_D);

	reg = G1_REG_DEC_JPEG_VLC_AC2_CODE5_CNT(bits2[4]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE6_CNT(bits2[5]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE7_CNT(bits2[6]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE8_CNT(bits2[7]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_E);

	reg = G1_REG_DEC_JPEG_VLC_AC2_CODE9_CNT(bits2[8]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE10_CNT(bits2[9]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE11_CNT(bits2[10]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE12_CNT(bits2[11]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_F);

	reg = G1_REG_DEC_JPEG_VLC_AC2_CODE13_CNT(bits2[12]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE14_CNT(bits2[13]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE15_CNT(bits2[14]) |
	      G1_REG_DEC_JPEG_VLC_AC2_CODE16_CNT(bits2[15]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_G);

	bits1 = (u8 *)header->huffman_tables[0].start;/* DC luma */
	bits2 = (u8 *)header->huffman_tables[1].start;/* DC chroma  */

	/* Hardware requires that DC1 registers contains luma table */
	if (header->scan->component[0].dc_entropy_coding_table_selector == 1) /* Td[0] == 1 */
		swap(bits1, bits2);

	/* DC1 table code lengths (luma) */
	reg = G1_REG_DEC_JPEG_VLC_DC1_CODE1_CNT(bits1[0]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE2_CNT(bits1[1]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE3_CNT(bits1[2]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE4_CNT(bits1[3]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE5_CNT(bits1[4]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE6_CNT(bits1[5]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE7_CNT(bits1[6]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE8_CNT(bits1[7]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_DC_CODE_LENGTHS_A);

	reg = G1_REG_DEC_JPEG_VLC_DC1_CODE9_CNT(bits1[8]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE10_CNT(bits1[9]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE11_CNT(bits1[10]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE12_CNT(bits1[11]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE13_CNT(bits1[12]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE14_CNT(bits1[13]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE15_CNT(bits1[14]) |
	      G1_REG_DEC_JPEG_VLC_DC1_CODE16_CNT(bits1[15]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_DC_CODE_LENGTHS_B);

	/* DC2 table code lengths (not-luma) */
	reg = G1_REG_DEC_JPEG_VLC_DC2_CODE1_CNT(bits2[0]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE2_CNT(bits2[1]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE3_CNT(bits2[2]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE4_CNT(bits2[3]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE5_CNT(bits2[4]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE6_CNT(bits2[5]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE7_CNT(bits2[6]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE8_CNT(bits2[7]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_DC_CODE_LENGTHS_C);

	reg = G1_REG_DEC_JPEG_VLC_DC2_CODE9_CNT(bits2[8]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE10_CNT(bits2[9]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE11_CNT(bits2[10]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE12_CNT(bits2[11]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE13_CNT(bits2[12]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE14_CNT(bits2[13]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE15_CNT(bits2[14]) |
	      G1_REG_DEC_JPEG_VLC_DC2_CODE16_CNT(bits2[15]);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_JPEG_VLC_DC_CODE_LENGTHS_D);

	return 0;
}

static int stream_bit_offset(struct v4l2_jpeg_header *header)
{
	int page_ptr, word_ptr, bit_ptr;
	u32 ecs_offset = header->ecs_offset;

	page_ptr = ecs_offset / 256;
	word_ptr = (ecs_offset % 256) / 4;
	if (page_ptr & 1)
		word_ptr += 64;
	bit_ptr = (ecs_offset % 4) * 8;
	if (word_ptr & 1)
		bit_ptr += 32;

	return bit_ptr;
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
	    frame->width > HANTRO_G1_JPEG_DEC_MAX_SIZE ||
	    frame->height > HANTRO_G1_JPEG_DEC_MAX_SIZE ||
	    frame->width * frame->height > HANTRO_G1_JPEG_DEC_MAX_PIXELS) {
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

static int hantro_g1_jpeg_dec_start(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	size_t height = ctx->dst_fmt.height;
	size_t width = ctx->dst_fmt.width;
	struct v4l2_jpeg_scan_header scan_header = { };
	struct v4l2_jpeg_reference quantization_tables[V4L2_JPEG_MAX_TABLES] = { };
	struct v4l2_jpeg_reference huffman_tables[V4L2_JPEG_MAX_TABLES] = { };
	struct v4l2_jpeg_header header = {
		.scan = &scan_header,
		.quantization_tables = quantization_tables,
		.huffman_tables = huffman_tables,
	};
	u32 mb_width, mb_height;
	u32 jpeg_size;
	unsigned int i;
	void *vaddr;
	u32 reg;
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

	jpeg_size = vb2_get_plane_payload(&src_buf->vb2_buf, 0);

	ret = v4l2_jpeg_parse_header(vaddr, jpeg_size, &header);
	if (ret < 0) {
		vpu_debug(1, "cannot parse the JPEG header: %d\n", ret);
		return ret;
	}

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

	/*
	 * The CAPTURE format is read without the ioctl lock: S_FMT leaves it
	 * as it is while the CAPTURE queue has buffers, and each of them
	 * holds it (hantro_queue_setup() and hantro_buf_prepare() check).
	 */
	ret = hantro_g1_jpeg_dec_check(&header, jpeg_size,
				       ctx->dst_fmt.pixelformat, width, height);
	if (ret)
		return ret;

	/* Write VLC table code lengths */
	ret = write_vlc_code_lengths(ctx, &header);
	if (ret)
		return ret;

	/* Prepare VLC QP/AC/DC hardware tables */
	ret = hantro_jpeg_prepare_vlc_hw_table(&header, ctx->jpeg_dec.priv.cpu);
	if (ret) {
		vpu_debug(1, "cannot build the hardware tables: %d\n", ret);
		return ret;
	}

	hantro_start_prepare_run(ctx);

	reg = G1_REG_DEC_CTRL0_DEC_MODE(3) |
	      G1_REG_DEC_CTRL0_FILTERING_DIS;
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL0);

	/* Frame dimensions */
	mb_width = MB_WIDTH(width);
	mb_height = MB_HEIGHT(height);
	reg = G1_REG_DEC_CTRL1_PIC_MB_WIDTH(mb_width) |
	      G1_REG_DEC_CTRL1_PIC_MB_HEIGHT_P(mb_height) |
	      G1_REG_DEC_CTRL1_PIC_MB_W_EXT(mb_width >> 9) |
	      G1_REG_DEC_CTRL1_PIC_MB_H_EXT(mb_height >> 8);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL1);

	reg = G1_REG_DEC_CTRL2_JPEG_MODE(to_jpeg_mode(header.frame.subsampling)) |
	      G1_REG_DEC_CTRL2_JPEG_STREAM_ALL;

	/*
	 * For luminance the first table is always used
	 * For chrominance: check jpeg header
	 */
	reg |= G1_REG_DEC_CTRL2_JPEG_QTABLES(header.scan->num_components);

	if (header.scan->num_components > 0) {
		if (header.scan->component[1].dc_entropy_coding_table_selector == 1)/* Td[1]==1 */
			reg |= G1_REG_DEC_CTRL2_CB_DC_VLCTABLE;

		if (header.scan->component[1].ac_entropy_coding_table_selector == 1)/* Ta[1]==1 */
			reg |= G1_REG_DEC_CTRL2_CB_AC_VLCTABLE;
	}
	if (header.scan->num_components > 1) {
		if (header.scan->component[2].dc_entropy_coding_table_selector == 1)/* Td[2]==1 */
			reg |= G1_REG_DEC_CTRL2_CR_DC_VLCTABLE;

		if (header.scan->component[2].ac_entropy_coding_table_selector == 1)/* Ta[2]==1 */
			reg |= G1_REG_DEC_CTRL2_CR_AC_VLCTABLE;
	}

	reg |= G1_REG_DEC_CTRL2_STRM_START_BIT(stream_bit_offset(&header));

	if (header.restart_interval) {
		reg |= G1_REG_DEC_CTRL2_SYNC_MARKER_E;
		vdpu_write_relaxed(vpu,
				   G1_REG_DEC_CTRL5_PJPEG_REST_FREQ(header.restart_interval),
				   G1_REG_DEC_CTRL5);
	}

	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL2);

	reg = G1_REG_DEC_CTRL3_STREAM_LEN(jpeg_size);
	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL3);

	/* Vertical alignment */
	reg = IS_ALIGNED(header.frame.height, MB_DIM) ?
		0 : G1_REG_DEC_CTRL4_PJPEG_FILDOWN_E;

	vdpu_write_relaxed(vpu, reg, G1_REG_DEC_CTRL4);

	/* Disable slice mode */
	vdpu_write_relaxed(vpu, G1_REG_JPEG_CTRL_SLICE_H(0), G1_REG_JPEG_CTRL);

	set_buffers(vpu, ctx, &src_buf->vb2_buf, &dst_buf->vb2_buf, &header);

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
