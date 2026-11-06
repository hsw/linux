// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) Collabora, Ltd.
 *
 * Based on GSPCA and CODA drivers:
 * Copyright (C) Jean-Francois Moine (http://moinejf.free.fr)
 * Copyright (C) 2014 Philipp Zabel, Pengutronix
 * Copyright (C) STMicroelectronics SA 2024
 *	Hugues Fruchet <hugues.fruchet@foss.st.com>
 */

#include <linux/align.h>
#include <linux/build_bug.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <media/v4l2-jpeg.h>
#include "hantro_jpeg.h"
#include "hantro.h"

#define LUMA_QUANT_OFF		25
#define CHROMA_QUANT_OFF	90
#define HEIGHT_OFF		159
#define WIDTH_OFF		161

#define HUFF_LUMA_DC_OFF	178
#define HUFF_LUMA_AC_OFF	211
#define HUFF_CHROMA_DC_OFF	394
#define HUFF_CHROMA_AC_OFF	427

static const u32 hw_reorder[] = {
	 0,  8, 16, 24,  1,  9, 17, 25,
	32, 40, 48, 56, 33, 41, 49, 57,
	 2, 10, 18, 26,  3, 11, 19, 27,
	34, 42, 50, 58, 35, 43, 51, 59,
	 4, 12, 20, 28,  5, 13, 21, 29,
	36, 44, 52, 60, 37, 45, 53, 61,
	 6, 14, 22, 30,  7, 15, 23, 31,
	38, 46, 54, 62, 39, 47, 55, 63
};

/* For simplicity, we keep a pre-formatted JPEG header,
 * and we'll use fixed offsets to change the width, height
 * quantization tables, etc.
 */
static const unsigned char hantro_jpeg_header[] = {
	/* SOI */
	0xff, 0xd8,

	/* JFIF-APP0 */
	0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46,
	0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01,
	0x00, 0x00,

	/* DQT */
	0xff, 0xdb, 0x00, 0x84,

	0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* SOF */
	0xff, 0xc0, 0x00, 0x11, 0x08, 0x00, 0xf0, 0x01,
	0x40, 0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01,
	0x03, 0x11, 0x01,

	/* DHT */
	0xff, 0xc4, 0x00, 0x1f, 0x00,

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,

	/* DHT */
	0xff, 0xc4, 0x00, 0xb5, 0x10,

	0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* DHT */
	0xff, 0xc4, 0x00, 0x1f, 0x01,

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,

	/* DHT */
	0xff, 0xc4, 0x00, 0xb5, 0x11,

	0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	/* COM */
	0xff, 0xfe, 0x00, 0x03, 0x00,

	/* SOS */
	0xff, 0xda, 0x00, 0x0c, 0x03, 0x01, 0x00, 0x02,
	0x11, 0x03, 0x11, 0x00, 0x3f, 0x00,
};

/*
 * JPEG_HEADER_SIZE is used in other parts of the driver in lieu of
 * "sizeof(hantro_jpeg_header)". The two must be equal.
 */
static_assert(sizeof(hantro_jpeg_header) == JPEG_HEADER_SIZE);

/*
 * hantro_jpeg_header is padded with a COM segment, so that the payload
 * of the SOS segment (the entropy-encoded image scan), which should
 * trail the whole header, is 8-byte aligned for the hardware to write
 * to directly.
 */
static_assert(IS_ALIGNED(sizeof(hantro_jpeg_header), 8),
	      "Hantro JPEG header size needs to be 8-byte aligned.");

static unsigned char jpeg_scale_qp(const unsigned char qp, int scale)
{
	unsigned int temp;

	temp = DIV_ROUND_CLOSEST((unsigned int)qp * scale, 100);
	if (temp <= 0)
		temp = 1;
	if (temp > 255)
		temp = 255;

	return (unsigned char)temp;
}

static void
jpeg_scale_quant_table(unsigned char *file_q_tab,
		       unsigned char *reordered_q_tab,
		       const unsigned char *tab, int scale)
{
	int i;

	BUILD_BUG_ON(ARRAY_SIZE(v4l2_jpeg_zigzag_scan_index) != JPEG_QUANT_SIZE);
	BUILD_BUG_ON(ARRAY_SIZE(hw_reorder) != JPEG_QUANT_SIZE);

	for (i = 0; i < JPEG_QUANT_SIZE; i++) {
		file_q_tab[i] = jpeg_scale_qp(tab[v4l2_jpeg_zigzag_scan_index[i]], scale);
		reordered_q_tab[i] = jpeg_scale_qp(tab[hw_reorder[i]], scale);
	}
}

static void jpeg_set_quality(struct hantro_jpeg_ctx *ctx)
{
	int scale;
	/*
	 * Non-linear scaling factor:
	 * [5,50] -> [1000..100], [51,100] -> [98..0]
	 */
	if (ctx->quality < 50)
		scale = 5000 / ctx->quality;
	else
		scale = 200 - 2 * ctx->quality;

	BUILD_BUG_ON(ARRAY_SIZE(v4l2_jpeg_ref_table_luma_qt) != JPEG_QUANT_SIZE);
	BUILD_BUG_ON(ARRAY_SIZE(v4l2_jpeg_ref_table_chroma_qt) != JPEG_QUANT_SIZE);
	BUILD_BUG_ON(ARRAY_SIZE(ctx->hw_luma_qtable) != JPEG_QUANT_SIZE);
	BUILD_BUG_ON(ARRAY_SIZE(ctx->hw_chroma_qtable) != JPEG_QUANT_SIZE);

	jpeg_scale_quant_table(ctx->buffer + LUMA_QUANT_OFF,
			       ctx->hw_luma_qtable,
			       (const unsigned char *)v4l2_jpeg_ref_table_luma_qt, scale);
	jpeg_scale_quant_table(ctx->buffer + CHROMA_QUANT_OFF,
			       ctx->hw_chroma_qtable,
			       (const unsigned char *)v4l2_jpeg_ref_table_chroma_qt, scale);
}

void hantro_jpeg_header_assemble(struct hantro_jpeg_ctx *ctx)
{
	char *buf = ctx->buffer;

	memcpy(buf, hantro_jpeg_header,
	       sizeof(hantro_jpeg_header));

	buf[HEIGHT_OFF + 0] = ctx->height >> 8;
	buf[HEIGHT_OFF + 1] = ctx->height;
	buf[WIDTH_OFF + 0] = ctx->width >> 8;
	buf[WIDTH_OFF + 1] = ctx->width;

	memcpy(buf + HUFF_LUMA_DC_OFF, v4l2_jpeg_ref_table_luma_dc_ht, V4L2_JPEG_REF_HT_DC_LEN);
	memcpy(buf + HUFF_LUMA_AC_OFF, v4l2_jpeg_ref_table_luma_ac_ht, V4L2_JPEG_REF_HT_AC_LEN);
	memcpy(buf + HUFF_CHROMA_DC_OFF, v4l2_jpeg_ref_table_chroma_dc_ht, V4L2_JPEG_REF_HT_DC_LEN);
	memcpy(buf + HUFF_CHROMA_AC_OFF, v4l2_jpeg_ref_table_chroma_ac_ht, V4L2_JPEG_REF_HT_AC_LEN);

	jpeg_set_quality(ctx);
}

/**
 * hantro_jpeg_get_default_huffman_tables() - fill in missing Huffman tables
 * @huffman_tables: the DC0, DC1, AC0, AC1 tables of a parsed header
 *
 * A JPEG stream may omit Huffman tables (Motion JPEG frames usually do);
 * the tables of ITU-T T.81 Annex K.3 then apply, the luminance ones to
 * table 0 and the chrominance ones to table 1. Fill every table the stream
 * did not define.
 */
void hantro_jpeg_get_default_huffman_tables(struct v4l2_jpeg_reference *huffman_tables)
{
	static const struct {
		const u8 *table;
		size_t length;
	} defaults[V4L2_JPEG_MAX_TABLES] = {
		{ v4l2_jpeg_ref_table_luma_dc_ht, V4L2_JPEG_REF_HT_DC_LEN },
		{ v4l2_jpeg_ref_table_chroma_dc_ht, V4L2_JPEG_REF_HT_DC_LEN },
		{ v4l2_jpeg_ref_table_luma_ac_ht, V4L2_JPEG_REF_HT_AC_LEN },
		{ v4l2_jpeg_ref_table_chroma_ac_ht, V4L2_JPEG_REF_HT_AC_LEN },
	};
	unsigned int i;

	for (i = 0; i < V4L2_JPEG_MAX_TABLES; i++) {
		if (huffman_tables[i].start)
			continue;

		/* Read only: v4l2_jpeg_reference has no const variant. */
		huffman_tables[i].start = (u8 *)defaults[i].table;
		huffman_tables[i].length = defaults[i].length;
	}
}

/* HUFFVAL follows the 16 BITS code counts in a DHT table. */
#define JPEG_HUFFVAL_OFF	16

static int copy_huffman_values(u8 *dst, const struct v4l2_jpeg_reference *table,
			       unsigned int max_values)
{
	if (!table->start || table->length < JPEG_HUFFVAL_OFF ||
	    table->length - JPEG_HUFFVAL_OFF > max_values)
		return -EINVAL;

	memcpy(dst, table->start + JPEG_HUFFVAL_OFF,
	       table->length - JPEG_HUFFVAL_OFF);

	return 0;
}

/**
 * hantro_jpeg_prepare_vlc_hw_table() - build the decoder's table buffer
 * @header: parsed header, with the default Huffman tables filled in
 * @vlc_hw_table: HANTRO_JPEG_DEC_TABLE_SIZE bytes, 32-bit aligned
 *
 * The buffer is a byte sequence the hardware reads as little-endian 32-bit
 * words, the first byte of each word in bits 31:24:
 *
 * - the quantization table each scan component selects, in natural (not
 *   zigzag) order: one table for a one-component scan, three otherwise;
 * - the AC Huffman values (HUFFVAL) of the table the first component uses,
 *   then of the other table, each zero-padded to 162 bytes;
 * - the DC Huffman values in the same order, each zero-padded to 12 bytes.
 *
 * The other table's values stay zero for a one-component scan. The code
 * counts (BITS) go to registers, see hantro_g1_jpeg_dec_regs().
 *
 * Return: 0, or -EINVAL if a table is missing or does not fit.
 */
int hantro_jpeg_prepare_vlc_hw_table(const struct v4l2_jpeg_header *header,
				     u8 *vlc_hw_table)
{
	const struct v4l2_jpeg_scan_header *scan = header->scan;
	const struct v4l2_jpeg_reference *ht = header->huffman_tables;
	unsigned int ns = scan->num_components;
	u8 *p = vlc_hw_table;
	unsigned int i, j;
	u8 ta, td;
	int ret;

	memset(vlc_hw_table, 0, HANTRO_JPEG_DEC_TABLE_SIZE);

	if (ns != 1 && ns != 3)
		return -EINVAL;

	for (j = 0; j < ns; j++) {
		u8 tq = header->frame.component[j].quantization_table_selector;
		const struct v4l2_jpeg_reference *qt;

		if (tq >= V4L2_JPEG_MAX_TABLES)
			return -EINVAL;

		qt = &header->quantization_tables[tq];
		if (!qt->start || qt->length != JPEG_QUANT_SIZE)
			return -EINVAL;

		for (i = 0; i < JPEG_QUANT_SIZE; i++)
			p[v4l2_jpeg_zigzag_scan_index[i]] = qt->start[i];
		p += JPEG_QUANT_SIZE;
	}

	/* The first component's tables come first, whatever their number. */
	ta = scan->component[0].ac_entropy_coding_table_selector;
	td = scan->component[0].dc_entropy_coding_table_selector;
	if (ta > 1 || td > 1)
		return -EINVAL;

	ret = copy_huffman_values(p, &ht[2 + ta], HANTRO_JPEG_DEC_AC_SIZE);
	if (ret)
		return ret;
	p += HANTRO_JPEG_DEC_AC_SIZE;

	if (ns > 1) {
		ret = copy_huffman_values(p, &ht[2 + !ta], HANTRO_JPEG_DEC_AC_SIZE);
		if (ret)
			return ret;
	}
	p += HANTRO_JPEG_DEC_AC_SIZE;

	ret = copy_huffman_values(p, &ht[td], HANTRO_JPEG_DEC_DC_SIZE);
	if (ret)
		return ret;
	p += HANTRO_JPEG_DEC_DC_SIZE;

	if (ns > 1) {
		ret = copy_huffman_values(p, &ht[!td], HANTRO_JPEG_DEC_DC_SIZE);
		if (ret)
			return ret;
	}

	/* The decoder reads little-endian words, whatever the CPU. */
	for (i = 0; i < HANTRO_JPEG_DEC_TABLE_SIZE; i += 4)
		put_unaligned_le32(get_unaligned_be32(vlc_hw_table + i),
				   vlc_hw_table + i);

	return 0;
}

void hantro_jpeg_dec_exit(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct hantro_jpeg_dec_hw_ctx *jpeg_dec = &ctx->jpeg_dec;
	struct hantro_aux_buf *priv = &jpeg_dec->priv;

	dma_free_coherent(vpu->dev, priv->size, priv->cpu, priv->dma);
}

int hantro_jpeg_dec_init(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct hantro_jpeg_dec_hw_ctx *jpeg_dec = &ctx->jpeg_dec;
	struct hantro_aux_buf *priv = &jpeg_dec->priv;

	priv->cpu = dma_alloc_coherent(vpu->dev, HANTRO_JPEG_DEC_TABLE_SIZE,
				       &priv->dma, GFP_KERNEL);
	if (!priv->cpu)
		return -ENOMEM;

	priv->size = HANTRO_JPEG_DEC_TABLE_SIZE;

	return 0;
}
