// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the Hantro JPEG decoder helpers: the header check, the
 * defaults the decoder fills in, the table buffer, the G1 registers of a
 * job and the variants that enable the decoder. No hardware is touched.
 */

#include <kunit/test.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/videodev2.h>
#include <media/v4l2-jpeg.h>

#include "hantro.h"
#include "hantro_g1_regs.h"
#include "hantro_jpeg.h"

struct test_jpeg {
	struct v4l2_jpeg_header hdr;
	struct v4l2_jpeg_scan_header scan;
	struct v4l2_jpeg_reference qt[V4L2_JPEG_MAX_TABLES];
	struct v4l2_jpeg_reference ht[V4L2_JPEG_MAX_TABLES];
	size_t payload;
};

/*
 * A baseline picture as v4l2_jpeg_parse_header() returns it for a stream
 * with the ITU-T T.81 Annex K tables: components 1, 2, 3 in one scan, luma
 * on quantization and Huffman tables 0, chroma on tables 1.
 */
static void make_jpeg(struct test_jpeg *j,
		      enum v4l2_jpeg_chroma_subsampling subsampling,
		      u16 width, u16 height)
{
	struct v4l2_jpeg_frame_header *frame = &j->hdr.frame;
	unsigned int i, nf;
	u8 h = 1, v = 1;

	memset(j, 0, sizeof(*j));
	j->hdr.scan = &j->scan;
	j->hdr.quantization_tables = j->qt;
	j->hdr.huffman_tables = j->ht;

	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		h = 2;
		v = 2;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		h = 2;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_411:
		h = 4;
		break;
	default:
		break;
	}
	nf = subsampling == V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY ? 1 : 3;

	frame->precision = 8;
	frame->width = width;
	frame->height = height;
	frame->num_components = nf;
	frame->subsampling = subsampling;
	j->scan.num_components = nf;
	for (i = 0; i < nf; i++) {
		frame->component[i].component_identifier = i + 1;
		frame->component[i].horizontal_sampling_factor = i ? 1 : h;
		frame->component[i].vertical_sampling_factor = i ? 1 : v;
		frame->component[i].quantization_table_selector = i ? 1 : 0;
		j->scan.component[i].component_selector = i + 1;
		j->scan.component[i].dc_entropy_coding_table_selector = i ? 1 : 0;
		j->scan.component[i].ac_entropy_coding_table_selector = i ? 1 : 0;
	}

	j->qt[0].start = (u8 *)v4l2_jpeg_ref_table_luma_qt;
	j->qt[0].length = JPEG_QUANT_SIZE;
	if (nf > 1) {
		j->qt[1].start = (u8 *)v4l2_jpeg_ref_table_chroma_qt;
		j->qt[1].length = JPEG_QUANT_SIZE;
	}
	hantro_jpeg_get_default_huffman_tables(j->ht);

	j->hdr.ecs_offset = 623;
	j->payload = 4096;
}

static int check(const struct test_jpeg *j, u32 fourcc, u32 width, u32 height)
{
	return hantro_g1_jpeg_dec_check(&j->hdr, j->payload, fourcc,
					width, height);
}

static int check_size(u16 w, u16 h, u32 cap_w, u32 cap_h)
{
	struct test_jpeg j;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, w, h);
	j.payload = 64 * 1024;
	return check(&j, V4L2_PIX_FMT_NV12, cap_w, cap_h);
}

static void jpeg_dec_check_sampling(struct kunit *test)
{
	struct test_jpeg j;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1072, 1448);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), 0);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV16, 1072, 1456), -EINVAL);
	/* A post-processed CAPTURE format is not decoded to. */
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_YUYV, 1072, 1456), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_422, 1072, 1448);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV16, 1072, 1456), 0);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1072, 1448);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), 0);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), -EINVAL);
	/* Any sampling factors: a one-component MCU is one block. */
	j.hdr.frame.component[0].horizontal_sampling_factor = 2;
	j.hdr.frame.component[0].vertical_sampling_factor = 2;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), 0);
	/* Greyscale needs only the tables its one component uses. */
	j.ht[1].start = NULL;
	j.ht[3].start = NULL;
	j.qt[1].start = NULL;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), 0);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1072, 1448);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_444, 1072, 1448);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), -EINVAL);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV24, 1072, 1456), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_411, 1072, 1448);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), -EINVAL);

	/* Four components (CMYK), and two with a 4:2:0 luma. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_444, 1072, 1448);
	j.hdr.frame.num_components = 4;
	j.scan.num_components = 4;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1072, 1448);
	j.hdr.frame.num_components = 2;
	j.scan.num_components = 2;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), -EINVAL);
}

static void jpeg_dec_check_frame(struct kunit *test)
{
	struct test_jpeg j;

	/* Extended sequential, 12 bits. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.hdr.frame.precision = 12;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);

	/* 16-bit quantization table. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.qt[0].length = 2 * JPEG_QUANT_SIZE;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);

	/* A component selects a quantization table the stream lacks. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.hdr.frame.component[2].quantization_table_selector = 2;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);
	j.qt[2] = j.qt[1];
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), 0);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.qt[1].start = NULL;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);

	/* Non-interleaved: the first scan holds the luma only. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.scan.num_components = 1;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);

	/* Scan components in another order than the frame's. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.scan.component[1].component_selector = 3;
	j.scan.component[2].component_selector = 2;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);

	/* Huffman table 2 or 3 (extended sequential only). */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.scan.component[1].ac_entropy_coding_table_selector = 2;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.scan.component[2].dc_entropy_coding_table_selector = 3;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);
}

static void jpeg_dec_check_size(struct kunit *test)
{
	/* At most 8176 pixels per side. */
	KUNIT_EXPECT_EQ(test, check_size(8176, 48, 8176, 48), 0);
	KUNIT_EXPECT_EQ(test, check_size(8177, 48, 8192, 48), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(48, 8176, 48, 8176), 0);
	KUNIT_EXPECT_EQ(test, check_size(48, 8177, 48, 8192), -EINVAL);

	/* At most 4672x3504 pixels in total. */
	KUNIT_EXPECT_EQ(test, check_size(4672, 3504, 4672, 3504), 0);
	KUNIT_EXPECT_EQ(test, check_size(4673, 3504, 4688, 3504), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(4672, 3505, 4672, 3520), -EINVAL);

	/* At least three macroblocks per side. */
	KUNIT_EXPECT_EQ(test, check_size(33, 33, 48, 48), 0);
	KUNIT_EXPECT_EQ(test, check_size(32, 33, 48, 48), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(33, 32, 48, 48), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(32, 32, 32, 32), -EINVAL);

	/* The CAPTURE is the picture rounded up to macroblocks. */
	KUNIT_EXPECT_EQ(test, check_size(1000, 1448, 1008, 1456), 0);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1448, 1000, 1448), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1448, 1024, 1456), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1448, 992, 1456), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1457, 1008, 1456), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1441, 1008, 1456), 0);
}

static void jpeg_dec_check_huffman(struct kunit *test)
{
	/* 13 DC values; L = 2: 3 codes, L = 3: 7, L = 4: 3. */
	static const u8 dc13[16 + 13] = { 0, 3, 7, 3 };
	/* 163 AC values, all of length 9. */
	static const u8 ac163[16 + 163] = { [8] = 163 };
	/* 162 AC values, all of length 9. */
	static const u8 ac162[16 + 162] = { [8] = 162 };
	/* Two codes of length 1. */
	static const u8 dc_long1[16 + 2] = { 2 };
	/* No codes at all. */
	static const u8 empty[16] = { };
	struct test_jpeg j;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.ht[1].start = (u8 *)dc13;
	j.ht[1].length = sizeof(dc13);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.ht[3].start = (u8 *)ac162;
	j.ht[3].length = sizeof(ac162);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), 0);
	j.ht[3].start = (u8 *)ac163;
	j.ht[3].length = sizeof(ac163);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.ht[0].start = (u8 *)dc_long1;
	j.ht[0].length = sizeof(dc_long1);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.ht[2].start = (u8 *)empty;
	j.ht[2].length = sizeof(empty);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);
}

static void jpeg_dec_check_stream(struct kunit *test)
{
	struct test_jpeg j;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);

	/* No entropy-coded data after the scan header. */
	j.payload = j.hdr.ecs_offset;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);
	j.payload = j.hdr.ecs_offset + 1;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), 0);

	/* STREAM_LEN counts from the 8-byte aligned start: 24 bits. */
	j.payload = 0xffffff + ALIGN_DOWN(j.hdr.ecs_offset, 8);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), 0);
	j.payload++;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);
}

static void expect_table(struct kunit *test,
			 const struct v4l2_jpeg_reference *table,
			 const u8 *start, size_t length)
{
	KUNIT_EXPECT_PTR_EQ(test, (const u8 *)table->start, start);
	KUNIT_EXPECT_EQ(test, table->length, length);
}

static void jpeg_dec_default_huffman(struct kunit *test)
{
	static const u8 ac_custom[16 + 2] = { 0, 2, [16] = 0x00, 0x01 };
	struct v4l2_jpeg_reference ht[V4L2_JPEG_MAX_TABLES] = { };

	/* No DHT segment: all four tables from Annex K. */
	hantro_jpeg_get_default_huffman_tables(ht);
	expect_table(test, &ht[0], v4l2_jpeg_ref_table_luma_dc_ht,
		     V4L2_JPEG_REF_HT_DC_LEN);
	expect_table(test, &ht[1], v4l2_jpeg_ref_table_chroma_dc_ht,
		     V4L2_JPEG_REF_HT_DC_LEN);
	expect_table(test, &ht[2], v4l2_jpeg_ref_table_luma_ac_ht,
		     V4L2_JPEG_REF_HT_AC_LEN);
	expect_table(test, &ht[3], v4l2_jpeg_ref_table_chroma_ac_ht,
		     V4L2_JPEG_REF_HT_AC_LEN);

	/* A table the stream defines is kept. */
	memset(ht, 0, sizeof(ht));
	ht[2].start = (u8 *)ac_custom;
	ht[2].length = sizeof(ac_custom);
	hantro_jpeg_get_default_huffman_tables(ht);
	expect_table(test, &ht[2], ac_custom, sizeof(ac_custom));
	expect_table(test, &ht[3], v4l2_jpeg_ref_table_chroma_ac_ht,
		     V4L2_JPEG_REF_HT_AC_LEN);
}

static void jpeg_dec_fourcc(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_420),
			V4L2_PIX_FMT_NV12);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_422),
			V4L2_PIX_FMT_NV16);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY),
			V4L2_PIX_FMT_GREY);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_444), 0);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_411), 0);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_410), 0);
}

/* The table buffer as bytes, in the order the hardware reads them. */
static void table_bytes(struct kunit *test, const u8 *buf, u8 *bytes)
{
	unsigned int i;

	for (i = 0; i < HANTRO_JPEG_DEC_TABLE_SIZE / 4; i++)
		put_unaligned_be32(((const u32 *)buf)[i], bytes + 4 * i);
}

static void natural_order(const u8 *zigzag, u8 *natural)
{
	unsigned int i;

	for (i = 0; i < JPEG_QUANT_SIZE; i++)
		natural[v4l2_jpeg_zigzag_scan_index[i]] = zigzag[i];
}

/* Expected AC (162 + 162) and DC (12 + 12) value lists after the q tables */
static void expect_huffman(struct kunit *test, const u8 *bytes, unsigned int nq,
			   const struct v4l2_jpeg_reference *ac1,
			   const struct v4l2_jpeg_reference *ac2,
			   const struct v4l2_jpeg_reference *dc1,
			   const struct v4l2_jpeg_reference *dc2)
{
	const struct v4l2_jpeg_reference *lists[] = { ac1, ac2, dc1, dc2 };
	const unsigned int sizes[] = {
		HANTRO_JPEG_DEC_AC_SIZE, HANTRO_JPEG_DEC_AC_SIZE,
		HANTRO_JPEG_DEC_DC_SIZE, HANTRO_JPEG_DEC_DC_SIZE,
	};
	u8 expected[HANTRO_JPEG_DEC_AC_SIZE];
	const u8 *p = bytes + nq * JPEG_QUANT_SIZE;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(lists); i++) {
		memset(expected, 0, sizeof(expected));
		if (lists[i])
			memcpy(expected, lists[i]->start + 16, lists[i]->length - 16);
		KUNIT_EXPECT_MEMEQ_MSG(test, p, expected, sizes[i], "list %u", i);
		p += sizes[i];
	}
	/* The rest of the buffer stays zero. */
	memset(expected, 0, sizeof(expected));
	KUNIT_EXPECT_MEMEQ(test, p, expected, bytes + HANTRO_JPEG_DEC_TABLE_SIZE - p);
}

static void jpeg_dec_table_420(struct kunit *test)
{
	u8 *buf = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	u8 *bytes = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	u8 luma[JPEG_QUANT_SIZE], chroma[JPEG_QUANT_SIZE];
	struct test_jpeg j;

	KUNIT_ASSERT_NOT_NULL(test, buf);
	KUNIT_ASSERT_NOT_NULL(test, bytes);
	natural_order(v4l2_jpeg_ref_table_luma_qt, luma);
	natural_order(v4l2_jpeg_ref_table_chroma_qt, chroma);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	KUNIT_ASSERT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), 0);
	table_bytes(test, buf, bytes);

	/* The first word: the first four luma quantizers, first in 31:24. */
	KUNIT_EXPECT_EQ(test, ((u32 *)buf)[0],
			(u32)luma[0] << 24 | luma[1] << 16 | luma[2] << 8 | luma[3]);
	KUNIT_EXPECT_MEMEQ(test, bytes, luma, JPEG_QUANT_SIZE);
	KUNIT_EXPECT_MEMEQ(test, bytes + 64, chroma, JPEG_QUANT_SIZE);
	KUNIT_EXPECT_MEMEQ(test, bytes + 128, chroma, JPEG_QUANT_SIZE);
	expect_huffman(test, bytes, 3, &j.ht[2], &j.ht[3], &j.ht[0], &j.ht[1]);
}

static void jpeg_dec_table_selectors(struct kunit *test)
{
	u8 *buf = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	u8 *bytes = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	u8 luma[JPEG_QUANT_SIZE], chroma[JPEG_QUANT_SIZE];
	struct test_jpeg j;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, buf);
	KUNIT_ASSERT_NOT_NULL(test, bytes);
	natural_order(v4l2_jpeg_ref_table_luma_qt, luma);
	natural_order(v4l2_jpeg_ref_table_chroma_qt, chroma);

	/* Luma on Huffman tables 1, chroma on tables 0: luma's come first. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	for (i = 0; i < 3; i++) {
		j.scan.component[i].dc_entropy_coding_table_selector = !i;
		j.scan.component[i].ac_entropy_coding_table_selector = !i;
	}
	KUNIT_ASSERT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), 0);
	table_bytes(test, buf, bytes);
	expect_huffman(test, bytes, 3, &j.ht[3], &j.ht[2], &j.ht[1], &j.ht[0]);

	/* Each component gets the quantization table it selects. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.hdr.frame.component[0].quantization_table_selector = 1;
	j.hdr.frame.component[1].quantization_table_selector = 0;
	j.hdr.frame.component[2].quantization_table_selector = 3;
	j.qt[3] = j.qt[0];
	KUNIT_ASSERT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), 0);
	table_bytes(test, buf, bytes);
	KUNIT_EXPECT_MEMEQ(test, bytes, chroma, JPEG_QUANT_SIZE);
	KUNIT_EXPECT_MEMEQ(test, bytes + 64, luma, JPEG_QUANT_SIZE);
	KUNIT_EXPECT_MEMEQ(test, bytes + 128, luma, JPEG_QUANT_SIZE);

	/* A missing table, and a 16-bit one. */
	j.qt[3].start = NULL;
	KUNIT_EXPECT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), -EINVAL);
	j.qt[3] = j.qt[0];
	j.qt[3].length = 128;
	KUNIT_EXPECT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), -EINVAL);
}

static void jpeg_dec_table_lengths(struct kunit *test)
{
	/* An optimised table: two codes of length 2. */
	static const u8 ac_short[16 + 2] = { 0, 2, [16] = 0x01, 0x11 };
	static const u8 ac163[16 + 163] = { [8] = 163 };
	static const u8 dc13[16 + 13] = { 0, 3, 7, 3 };
	static const u8 dc12[16 + 12] = { 0, 3, 7, 2, [16] = 1, 2, 3, 4, 5, 6,
					  7, 8, 9, 10, 11, 12 };
	u8 *buf = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	u8 *bytes = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	struct test_jpeg j;

	KUNIT_ASSERT_NOT_NULL(test, buf);
	KUNIT_ASSERT_NOT_NULL(test, bytes);

	/* Short value lists are padded with zeros, 12 values fill DC. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.ht[2].start = (u8 *)ac_short;
	j.ht[2].length = sizeof(ac_short);
	j.ht[1].start = (u8 *)dc12;
	j.ht[1].length = sizeof(dc12);
	memset(buf, 0xff, HANTRO_JPEG_DEC_TABLE_SIZE);
	KUNIT_ASSERT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), 0);
	table_bytes(test, buf, bytes);
	expect_huffman(test, bytes, 3, &j.ht[2], &j.ht[3], &j.ht[0], &j.ht[1]);

	/* Value lists longer than the buffer holds. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.ht[3].start = (u8 *)ac163;
	j.ht[3].length = sizeof(ac163);
	KUNIT_EXPECT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.ht[0].start = (u8 *)dc13;
	j.ht[0].length = sizeof(dc13);
	KUNIT_EXPECT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), -EINVAL);
}

/*
 * The code count fields of registers 0x040-0x068, written out from the
 * register map rather than from the driver's macros: { word, shift } of
 * count L = 1..16 of the AC1, AC2, DC1 and DC2 tables.
 */
static const u8 vlc_field[4][16][2] = {
	{ { 0, 0 }, { 0, 3 }, { 0, 7 }, { 0, 11 }, { 0, 16 }, { 0, 24 },
	  { 1, 0 }, { 1, 8 }, { 1, 16 }, { 1, 24 },
	  { 2, 0 }, { 2, 8 }, { 2, 16 }, { 2, 24 }, { 3, 0 }, { 3, 8 } },
	{ { 3, 16 }, { 3, 19 }, { 3, 23 }, { 3, 27 },
	  { 4, 0 }, { 4, 8 }, { 4, 16 }, { 4, 24 },
	  { 5, 0 }, { 5, 8 }, { 5, 16 }, { 5, 24 },
	  { 6, 0 }, { 6, 8 }, { 6, 16 }, { 6, 24 } },
	{ { 7, 0 }, { 7, 4 }, { 7, 8 }, { 7, 12 }, { 7, 16 }, { 7, 20 },
	  { 7, 24 }, { 7, 28 }, { 8, 0 }, { 8, 4 }, { 8, 8 }, { 8, 12 },
	  { 8, 16 }, { 8, 20 }, { 8, 24 }, { 8, 28 } },
	{ { 9, 0 }, { 9, 4 }, { 9, 8 }, { 9, 12 }, { 9, 16 }, { 9, 20 },
	  { 9, 24 }, { 9, 28 }, { 10, 0 }, { 10, 4 }, { 10, 8 }, { 10, 12 },
	  { 10, 16 }, { 10, 20 }, { 10, 24 }, { 10, 28 } },
};

static void expect_vlc(struct kunit *test, const u32 *vlc,
		       const u8 *ac1, const u8 *ac2, const u8 *dc1, const u8 *dc2)
{
	const u8 *bits[4] = { ac1, ac2, dc1, dc2 };
	u32 expected[HANTRO_G1_JPEG_DEC_VLC_REGS] = { };
	unsigned int t, l;

	for (t = 0; t < 4; t++)
		for (l = 0; l < 16; l++)
			if (bits[t])
				expected[vlc_field[t][l][0]] |=
					(u32)bits[t][l] << vlc_field[t][l][1];

	KUNIT_EXPECT_MEMEQ(test, vlc, expected, sizeof(expected));
}

static void jpeg_dec_regs_vlc(struct kunit *test)
{
	struct hantro_g1_jpeg_dec_regs regs;
	struct test_jpeg j;
	unsigned int i;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);
	expect_vlc(test, regs.vlc, v4l2_jpeg_ref_table_luma_ac_ht,
		   v4l2_jpeg_ref_table_chroma_ac_ht,
		   v4l2_jpeg_ref_table_luma_dc_ht,
		   v4l2_jpeg_ref_table_chroma_dc_ht);
	/* Spot checks: luma AC counts 0, 2, 1, 3, ..., DC chroma 0, 3, 1. */
	KUNIT_EXPECT_EQ(test, regs.vlc[0],
			0u << 0 | 2u << 3 | 1u << 7 | 3u << 11 | 3u << 16 | 2u << 24);
	KUNIT_EXPECT_EQ(test, regs.vlc[9] & 0xfff, 0u << 0 | 3u << 4 | 1u << 8);

	/* Luma on tables 1: its counts go to the AC1/DC1 fields. */
	for (i = 0; i < 3; i++) {
		j.scan.component[i].dc_entropy_coding_table_selector = !i;
		j.scan.component[i].ac_entropy_coding_table_selector = !i;
	}
	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);
	expect_vlc(test, regs.vlc, v4l2_jpeg_ref_table_chroma_ac_ht,
		   v4l2_jpeg_ref_table_luma_ac_ht,
		   v4l2_jpeg_ref_table_chroma_dc_ht,
		   v4l2_jpeg_ref_table_luma_dc_ht);
}

static u32 ctrl2_selectors(u32 ctrl2)
{
	return ctrl2 & (G1_REG_DEC_CTRL2_CB_AC_VLCTABLE |
			G1_REG_DEC_CTRL2_CB_DC_VLCTABLE |
			G1_REG_DEC_CTRL2_CR_AC_VLCTABLE |
			G1_REG_DEC_CTRL2_CR_DC_VLCTABLE |
			G1_REG_DEC_CTRL2_CB_DC_VLCTABLE3 |
			G1_REG_DEC_CTRL2_CR_DC_VLCTABLE3);
}

static u32 selectors(u8 y_dc, u8 y_ac, u8 cb_dc, u8 cb_ac, u8 cr_dc, u8 cr_ac)
{
	struct hantro_g1_jpeg_dec_regs regs;
	struct test_jpeg j;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.scan.component[0].dc_entropy_coding_table_selector = y_dc;
	j.scan.component[0].ac_entropy_coding_table_selector = y_ac;
	j.scan.component[1].dc_entropy_coding_table_selector = cb_dc;
	j.scan.component[1].ac_entropy_coding_table_selector = cb_ac;
	j.scan.component[2].dc_entropy_coding_table_selector = cr_dc;
	j.scan.component[2].ac_entropy_coding_table_selector = cr_ac;
	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);

	return ctrl2_selectors(regs.ctrl2);
}

static void jpeg_dec_regs_selectors(struct kunit *test)
{
	const u32 all = G1_REG_DEC_CTRL2_CB_AC_VLCTABLE |
			G1_REG_DEC_CTRL2_CB_DC_VLCTABLE |
			G1_REG_DEC_CTRL2_CR_AC_VLCTABLE |
			G1_REG_DEC_CTRL2_CR_DC_VLCTABLE;

	/* Luma on tables 0. */
	KUNIT_EXPECT_EQ(test, selectors(0, 0, 1, 1, 1, 1), all);
	KUNIT_EXPECT_EQ(test, selectors(0, 0, 0, 0, 0, 0), 0);
	KUNIT_EXPECT_EQ(test, selectors(0, 0, 0, 1, 1, 0),
			G1_REG_DEC_CTRL2_CB_AC_VLCTABLE |
			G1_REG_DEC_CTRL2_CR_DC_VLCTABLE);
	/* Luma on tables 1: the selector says "not the luma's table". */
	KUNIT_EXPECT_EQ(test, selectors(1, 1, 0, 0, 0, 0), all);
	KUNIT_EXPECT_EQ(test, selectors(1, 1, 1, 1, 1, 1), 0);
	KUNIT_EXPECT_EQ(test, selectors(1, 0, 1, 1, 0, 0),
			G1_REG_DEC_CTRL2_CB_AC_VLCTABLE |
			G1_REG_DEC_CTRL2_CR_DC_VLCTABLE);
}

static void jpeg_dec_regs_stream(struct kunit *test)
{
	static const size_t offsets[] = {
		2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 623,
	};
	const dma_addr_t src = 0x10000000;
	struct hantro_g1_jpeg_dec_regs regs;
	struct test_jpeg j;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		size_t ecs = offsets[i];

		make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
		j.hdr.ecs_offset = ecs;
		j.payload = 100000;
		hantro_g1_jpeg_dec_regs(&j.hdr, src, j.payload, &regs);

		KUNIT_EXPECT_EQ_MSG(test, regs.stream, src + (ecs & ~7),
				    "ecs %zu", ecs);
		KUNIT_EXPECT_EQ_MSG(test, (regs.ctrl2 >> 26) & 0x3f, 8 * (ecs & 7),
				    "ecs %zu", ecs);
		KUNIT_EXPECT_EQ_MSG(test, regs.ctrl3 & 0xffffff,
				    j.payload - (ecs & ~7), "ecs %zu", ecs);
		KUNIT_EXPECT_TRUE(test, regs.ctrl2 & G1_REG_DEC_CTRL2_JPEG_STREAM_ALL);
	}
}

static void jpeg_dec_regs_restart(struct kunit *test)
{
	/* Ri from a DRI segment: none, every MCU, odd ones, the largest. */
	static const u16 intervals[] = { 0, 1, 7, 67, 0xffff };
	static const enum v4l2_jpeg_chroma_subsampling sampling[] = {
		V4L2_JPEG_CHROMA_SUBSAMPLING_420,
		V4L2_JPEG_CHROMA_SUBSAMPLING_422,
	};
	struct hantro_g1_jpeg_dec_regs regs, plain;
	struct test_jpeg j;
	unsigned int i, s;

	for (s = 0; s < ARRAY_SIZE(sampling); s++) {
		make_jpeg(&j, sampling[s], 1072, 1448);
		hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &plain);

		for (i = 0; i < ARRAY_SIZE(intervals); i++) {
			u16 ri = intervals[i];

			j.hdr.restart_interval = ri;
			hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload,
						&regs);
			/*
			 * SYNC_MARKER_E is CTRL2 bit 25, PJPEG_REST_FREQ is
			 * CTRL5 bits 15:0; nothing else changes.
			 */
			KUNIT_EXPECT_EQ_MSG(test, regs.ctrl2,
					    plain.ctrl2 | (ri ? BIT(25) : 0),
					    "Ri %u", ri);
			KUNIT_EXPECT_EQ_MSG(test, regs.ctrl5, (u32)ri,
					    "Ri %u", ri);
			KUNIT_EXPECT_EQ_MSG(test, regs.ctrl4, plain.ctrl4,
					    "Ri %u", ri);
		}
	}
}

static u32 fill_bits(enum v4l2_jpeg_chroma_subsampling subsampling,
		     u16 width, u16 height)
{
	struct hantro_g1_jpeg_dec_regs regs;
	struct test_jpeg j;
	u32 bits = 0;

	make_jpeg(&j, subsampling, width, height);
	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);
	if (regs.ctrl2 & G1_REG_DEC_CTRL2_JPEG_FILRIGHT_E)
		bits |= 1;
	if (regs.ctrl4 & G1_REG_DEC_CTRL4_PJPEG_FILDOWN_E)
		bits |= 2;
	/* Nothing else in CTRL4 for a baseline picture. */
	if (regs.ctrl4 & ~G1_REG_DEC_CTRL4_PJPEG_FILDOWN_E)
		bits |= 4;

	return bits;
}

static void jpeg_dec_regs_fill(struct kunit *test)
{
	/* 1 = FILRIGHT, 2 = FILDOWN */
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1920, 1080), 0);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1000, 1081), 0);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_422, 1920, 1080), 2);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_422, 1920, 1073), 2);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_422, 1920, 1081), 0);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_422, 1000, 1440), 0);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1000, 1448), 3);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1072, 1448), 2);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1000, 1440), 1);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1001, 1449), 0);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1440, 1440), 0);
}

static void jpeg_dec_regs_frame(struct kunit *test)
{
	struct hantro_g1_jpeg_dec_regs regs;
	struct test_jpeg j;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1072, 1448);
	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);
	/* DEC_MODE 3 (JPEG) in 31:28, FILTERING_DIS (bit 14). */
	KUNIT_EXPECT_EQ(test, regs.ctrl0, 3u << 28 | BIT(14));
	/* 67 x 91 macroblocks: width in 31:23, height in 18:11, no ext. */
	KUNIT_EXPECT_EQ(test, regs.ctrl1, 67u << 23 | 91u << 11);
	/* JPEG_MODE 2 in 10:8, three quantization tables in 12:11. */
	KUNIT_EXPECT_EQ(test, (regs.ctrl2 >> 8) & 0x7, 2);
	KUNIT_EXPECT_EQ(test, (regs.ctrl2 >> 11) & 0x3, 3);
	KUNIT_EXPECT_EQ(test, regs.jpeg_ctrl, 0);

	/* 511 macroblock rows: 8 bits in HEIGHT_P, the ninth in H_EXT. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_422, 48, 8176);
	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);
	KUNIT_EXPECT_EQ(test, regs.ctrl1, 3u << 23 | 0xffu << 11 | 1u << 0);
	KUNIT_EXPECT_EQ(test, (regs.ctrl2 >> 8) & 0x7, 3);

	/* 511 macroblock columns fit the 9-bit width field. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 8176, 48);
	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);
	KUNIT_EXPECT_EQ(test, regs.ctrl1, 511u << 23 | 3u << 11);
}

static void jpeg_dec_mode(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_mode(V4L2_JPEG_CHROMA_SUBSAMPLING_420), 2);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_mode(V4L2_JPEG_CHROMA_SUBSAMPLING_422), 3);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_mode(V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY), 0);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_mode(V4L2_JPEG_CHROMA_SUBSAMPLING_444),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_mode(V4L2_JPEG_CHROMA_SUBSAMPLING_411),
			-EINVAL);
}

static void jpeg_dec_grey(struct kunit *test)
{
	u8 *buf = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	u8 *bytes = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	struct hantro_g1_jpeg_dec_regs regs;
	u8 luma[JPEG_QUANT_SIZE];
	struct test_jpeg j;

	KUNIT_ASSERT_NOT_NULL(test, buf);
	KUNIT_ASSERT_NOT_NULL(test, bytes);
	natural_order(v4l2_jpeg_ref_table_luma_qt, luma);

	/* Only table 0 of each kind in the stream. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1072, 1448);
	j.ht[1].start = NULL;
	j.ht[3].start = NULL;

	/* One quantization table, AC from offset 64, no second lists. */
	memset(buf, 0xff, HANTRO_JPEG_DEC_TABLE_SIZE);
	KUNIT_ASSERT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), 0);
	table_bytes(test, buf, bytes);
	KUNIT_EXPECT_MEMEQ(test, bytes, luma, JPEG_QUANT_SIZE);
	expect_huffman(test, bytes, 1, &j.ht[2], NULL, &j.ht[0], NULL);

	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);
	/* JPEG_MODE 0, one quantization table, no chroma table selects. */
	KUNIT_EXPECT_EQ(test, (regs.ctrl2 >> 8) & 0x7, 0);
	KUNIT_EXPECT_EQ(test, (regs.ctrl2 >> 11) & 0x3, 1);
	KUNIT_EXPECT_EQ(test, ctrl2_selectors(regs.ctrl2), 0);
	expect_vlc(test, regs.vlc, v4l2_jpeg_ref_table_luma_ac_ht, NULL,
		   v4l2_jpeg_ref_table_luma_dc_ht, NULL);

	/* The same with the luma on tables 1. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1072, 1448);
	j.scan.component[0].dc_entropy_coding_table_selector = 1;
	j.scan.component[0].ac_entropy_coding_table_selector = 1;
	j.ht[0].start = NULL;
	j.ht[2].start = NULL;
	KUNIT_ASSERT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), 0);
	KUNIT_ASSERT_EQ(test, hantro_jpeg_prepare_vlc_hw_table(&j.hdr, buf), 0);
	table_bytes(test, buf, bytes);
	expect_huffman(test, bytes, 1, &j.ht[3], NULL, &j.ht[1], NULL);
	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);
	KUNIT_EXPECT_EQ(test, ctrl2_selectors(regs.ctrl2), 0);
	expect_vlc(test, regs.vlc, v4l2_jpeg_ref_table_chroma_ac_ht, NULL,
		   v4l2_jpeg_ref_table_chroma_dc_ht, NULL);

	/* A greyscale header claiming three components. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1072, 1448);
	j.hdr.frame.subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), -EINVAL);
}

static const struct hantro_fmt *find_fmt(const struct hantro_variant *variant,
					 u32 fourcc, unsigned int *count)
{
	const struct hantro_fmt *found = NULL;
	unsigned int i;

	*count = 0;
	for (i = 0; i < variant->num_dec_fmts; i++) {
		if (variant->dec_fmts[i].fourcc != fourcc)
			continue;
		found = &variant->dec_fmts[i];
		(*count)++;
	}

	return found;
}

static void jpeg_dec_rockchip_fmts(struct kunit *test)
{
	static const u32 jpeg_only[] = { V4L2_PIX_FMT_NV16, V4L2_PIX_FMT_GREY };
	const struct hantro_variant *rk3026 = &rk3026_vpu_variant;
	const struct hantro_fmt *fmt;
	unsigned int count, i;

	if (!IS_ENABLED(CONFIG_VIDEO_HANTRO_ROCKCHIP))
		kunit_skip(test, "no Rockchip variants");

	/* RK3036, RK3066 and RK3188 do not decode JPEG. */
	find_fmt(&rk3036_vpu_variant, V4L2_PIX_FMT_JPEG, &count);
	KUNIT_EXPECT_EQ(test, count, 0);
	KUNIT_EXPECT_FALSE(test, rk3036_vpu_variant.codec & HANTRO_JPEG_DECODER);
	find_fmt(&rk3066_vpu_variant, V4L2_PIX_FMT_JPEG, &count);
	KUNIT_EXPECT_EQ(test, count, 0);
	KUNIT_EXPECT_FALSE(test, rk3066_vpu_variant.codec & HANTRO_JPEG_DECODER);
	KUNIT_EXPECT_FALSE(test, rk3066_vpu_variant.src_needs_kmap);
	for (i = 0; i < ARRAY_SIZE(jpeg_only); i++) {
		find_fmt(&rk3066_vpu_variant, jpeg_only[i], &count);
		KUNIT_EXPECT_EQ(test, count, 0);
	}

	/* The RK3026 does, with one JPEG entry and the decoder's limits. */
	fmt = find_fmt(rk3026, V4L2_PIX_FMT_JPEG, &count);
	KUNIT_ASSERT_EQ(test, count, 1);
	KUNIT_EXPECT_EQ(test, fmt->codec_mode, HANTRO_MODE_JPEG_DEC);
	KUNIT_EXPECT_EQ(test, fmt->frmsize.max_width, 8176);
	KUNIT_EXPECT_EQ(test, fmt->frmsize.max_height, 8176);
	KUNIT_EXPECT_TRUE(test, rk3026->codec & HANTRO_JPEG_DECODER);
	KUNIT_EXPECT_TRUE(test, rk3026->src_needs_kmap);
	KUNIT_EXPECT_TRUE(test, rk3026->codec_ops[HANTRO_MODE_JPEG_DEC].run ==
			  hantro_g1_jpeg_dec_run);
	KUNIT_EXPECT_TRUE(test, rk3026->codec_ops[HANTRO_MODE_JPEG_DEC].init ==
			  hantro_jpeg_dec_init);

	/* NV12 serves every codec, NV16 and GREY only JPEG. */
	fmt = find_fmt(rk3026, V4L2_PIX_FMT_NV12, &count);
	KUNIT_ASSERT_EQ(test, count, 1);
	KUNIT_EXPECT_EQ(test, fmt->codec_modes, 0);
	KUNIT_EXPECT_EQ(test, fmt->frmsize.max_width, 8176);
	for (i = 0; i < ARRAY_SIZE(jpeg_only); i++) {
		fmt = find_fmt(rk3026, jpeg_only[i], &count);
		KUNIT_ASSERT_EQ(test, count, 1);
		KUNIT_EXPECT_EQ(test, fmt->codec_modes, BIT(HANTRO_MODE_JPEG_DEC));
	}

	/* The video formats keep the RK3066 limits. */
	fmt = find_fmt(rk3026, V4L2_PIX_FMT_H264_SLICE, &count);
	KUNIT_ASSERT_EQ(test, count, 1);
	KUNIT_EXPECT_EQ(test, fmt->frmsize.max_width, FMT_FHD_WIDTH);
	KUNIT_EXPECT_EQ(test, fmt->frmsize.max_height, FMT_FHD_HEIGHT);
}

static struct kunit_case hantro_jpeg_dec_cases[] = {
	KUNIT_CASE(jpeg_dec_check_sampling),
	KUNIT_CASE(jpeg_dec_check_frame),
	KUNIT_CASE(jpeg_dec_check_size),
	KUNIT_CASE(jpeg_dec_check_huffman),
	KUNIT_CASE(jpeg_dec_check_stream),
	KUNIT_CASE(jpeg_dec_default_huffman),
	KUNIT_CASE(jpeg_dec_fourcc),
	KUNIT_CASE(jpeg_dec_mode),
	KUNIT_CASE(jpeg_dec_table_420),
	KUNIT_CASE(jpeg_dec_table_selectors),
	KUNIT_CASE(jpeg_dec_table_lengths),
	KUNIT_CASE(jpeg_dec_regs_vlc),
	KUNIT_CASE(jpeg_dec_regs_selectors),
	KUNIT_CASE(jpeg_dec_regs_stream),
	KUNIT_CASE(jpeg_dec_regs_restart),
	KUNIT_CASE(jpeg_dec_regs_fill),
	KUNIT_CASE(jpeg_dec_regs_frame),
	KUNIT_CASE(jpeg_dec_grey),
	KUNIT_CASE(jpeg_dec_rockchip_fmts),
	{ }
};

static struct kunit_suite hantro_jpeg_dec_suite = {
	.name = "hantro-jpeg-dec",
	.test_cases = hantro_jpeg_dec_cases,
};

kunit_test_suite(hantro_jpeg_dec_suite);
