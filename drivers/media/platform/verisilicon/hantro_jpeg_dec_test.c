// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the Hantro JPEG decoder helpers: the header check, and
 * the defaults the decoder fills in. No hardware is touched.
 */

#include <kunit/test.h>
#include <linux/string.h>
#include <linux/videodev2.h>
#include <media/v4l2-jpeg.h>

#include "hantro.h"
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
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), -EINVAL);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), -EINVAL);

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
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY), 0);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_444), 0);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_411), 0);
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_fourcc(V4L2_JPEG_CHROMA_SUBSAMPLING_410), 0);
}

static struct kunit_case hantro_jpeg_dec_cases[] = {
	KUNIT_CASE(jpeg_dec_check_sampling),
	KUNIT_CASE(jpeg_dec_check_frame),
	KUNIT_CASE(jpeg_dec_check_size),
	KUNIT_CASE(jpeg_dec_check_huffman),
	KUNIT_CASE(jpeg_dec_check_stream),
	KUNIT_CASE(jpeg_dec_default_huffman),
	KUNIT_CASE(jpeg_dec_fourcc),
	{ }
};

static struct kunit_suite hantro_jpeg_dec_suite = {
	.name = "hantro-jpeg-dec",
	.test_cases = hantro_jpeg_dec_cases,
};

kunit_test_suite(hantro_jpeg_dec_suite);
