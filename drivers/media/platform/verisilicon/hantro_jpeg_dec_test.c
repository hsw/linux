// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the Hantro JPEG decoder helpers: the header check, the
 * defaults the decoder fills in, the table buffer, the G1 registers of a
 * job and the variants that enable the decoder; and for the G1
 * post-processor's register builder. No hardware is touched.
 */

#include <kunit/test.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/videodev2.h>
#include <media/v4l2-jpeg.h>
#include <media/v4l2-mem2mem.h>

#include "hantro.h"
#include "hantro_g1_regs.h"
#include "hantro_hw.h"
#include "hantro_jpeg.h"
#include "hantro_v4l2.h"

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
	j->hdr.app14_tf = V4L2_JPEG_APP14_TF_UNKNOWN;
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

/* The RK3026's widest post-processor output. */
#define TEST_PP_MAX_WIDTH	1920

/*
 * A context with an OUTPUT format of src_w x src_h and a CAPTURE format of
 * fourcc at width x height, the compose rectangle at its default.
 */
static void make_out(struct hantro_g1_jpeg_dec_out *out, u32 src_w, u32 src_h,
		     u32 fourcc, u32 width, u32 height)
{
	memset(out, 0, sizeof(*out));
	out->src_width = src_w;
	out->src_height = src_h;
	out->fourcc = fourcc;
	out->width = width;
	out->height = height;
	out->pp_max_width = TEST_PP_MAX_WIDTH;
	hantro_g1_jpeg_dec_compose_default(out, &out->compose);
}

/* The checks of a job, and how it writes the CAPTURE buffer. */
static int check_out(const struct test_jpeg *j,
		     const struct hantro_g1_jpeg_dec_out *out,
		     struct hantro_g1_pp_regs *pp, bool *use_pp)
{
	int ret;

	ret = hantro_g1_jpeg_dec_check(&j->hdr, j->payload, out->src_width,
				       out->src_height);
	if (ret)
		return ret;

	return hantro_g1_jpeg_dec_job_out(out, j->hdr.frame.subsampling, pp,
					  use_pp);
}

/*
 * A job with the OUTPUT format of the picture's size, rounded up to 8 as
 * TRY_FMT does, and the CAPTURE format and size given.
 */
static int check(const struct test_jpeg *j, u32 fourcc, u32 width, u32 height)
{
	struct hantro_g1_jpeg_dec_out out;
	struct hantro_g1_pp_regs pp;
	bool use_pp;

	make_out(&out, ALIGN(j->hdr.frame.width, 8),
		 ALIGN(j->hdr.frame.height, 8), fourcc, width, height);
	return check_out(j, &out, &pp, &use_pp);
}

/*
 * A 4:2:0 picture of w x h with the OUTPUT format at src_w x src_h, into
 * the decoder's own NV12 buffer.
 */
static int check_size(u16 w, u16 h, u32 src_w, u32 src_h)
{
	struct hantro_g1_jpeg_dec_out out;
	struct hantro_g1_pp_regs pp;
	struct test_jpeg j;
	bool use_pp;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, w, h);
	j.payload = 64 * 1024;
	make_out(&out, src_w, src_h, V4L2_PIX_FMT_NV12, ALIGN(src_w, 16),
		 ALIGN(src_h, 16));
	return check_out(&j, &out, &pp, &use_pp);
}

static void jpeg_dec_check_sampling(struct kunit *test)
{
	struct test_jpeg j;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1072, 1448);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), 0);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV16, 1072, 1456), -EINVAL);
	/* The post-processor's formats, at a size it writes. */
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_YUYV, 1072, 1456), 0);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_XBGR32, 1072, 1448), 0);
	/* A format neither the decoder nor the post-processor writes. */
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV21, 1072, 1456), -EINVAL);

	/* NV12 from 4:2:2 or greyscale is the post-processor's. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_422, 1072, 1448);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV16, 1072, 1456), 0);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), 0);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), -EINVAL);

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1072, 1448);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), 0);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1072, 1456), 0);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV16, 1072, 1456), -EINVAL);
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
	unsigned int i;

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

	/* RGB: an Adobe transform 0, or 'R', 'G', 'B' without an APP14. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	j.hdr.app14_tf = V4L2_JPEG_APP14_TF_CMYK_RGB;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);
	j.hdr.app14_tf = V4L2_JPEG_APP14_TF_YCBCR;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), 0);
	for (i = 0; i < 3; i++) {
		j.hdr.frame.component[i].component_identifier = "RGB"[i];
		j.scan.component[i].component_selector = "RGB"[i];
	}
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), 0);
	j.hdr.app14_tf = V4L2_JPEG_APP14_TF_UNKNOWN;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);
}

static void jpeg_dec_check_size(struct kunit *test)
{
	/* At most 8176 pixels per side. */
	KUNIT_EXPECT_EQ(test, check_size(8176, 48, 8176, 48), 0);
	KUNIT_EXPECT_EQ(test, check_size(8177, 48, 8176, 48), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(48, 8176, 48, 8176), 0);
	KUNIT_EXPECT_EQ(test, check_size(48, 8177, 48, 8176), -EINVAL);

	/* At most 4672x3504 pixels in total. */
	KUNIT_EXPECT_EQ(test, check_size(4672, 3504, 4672, 3504), 0);
	KUNIT_EXPECT_EQ(test, check_size(4673, 3504, 4680, 3504), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(4672, 3505, 4672, 3512), -EINVAL);

	/* At least three macroblocks per side. */
	KUNIT_EXPECT_EQ(test, check_size(33, 33, 48, 48), 0);
	KUNIT_EXPECT_EQ(test, check_size(32, 33, 48, 48), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(33, 32, 48, 48), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(32, 32, 48, 48), -EINVAL);

	/* The picture has the OUTPUT format's size in macroblocks. */
	KUNIT_EXPECT_EQ(test, check_size(1000, 1448, 1000, 1448), 0);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1448, 1008, 1456), 0);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1448, 1024, 1456), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1448, 992, 1456), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1457, 1000, 1456), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(1000, 1441, 1000, 1448), 0);
	/*
	 * An OUTPUT format left at another picture's size, or at the
	 * smallest one, is refused: the CAPTURE buffer and the compose
	 * rectangle are set up for it.
	 */
	KUNIT_EXPECT_EQ(test, check_size(333, 251, 336, 256), 0);
	KUNIT_EXPECT_EQ(test, check_size(333, 251, 1072, 1448), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(333, 251, 336, 1448), -EINVAL);
	KUNIT_EXPECT_EQ(test, check_size(640, 480, 48, 48), -EINVAL);
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
	u8 dc[V4L2_JPEG_REF_HT_DC_LEN];
	u8 ac[16 + 128];
	struct test_jpeg j;
	unsigned int l;

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

	/* A count field holds 2^L - 1 codes of length L, no more. */
	for (l = 2; l <= 7; l++) {
		memset(ac, 0, sizeof(ac));
		make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
		j.ht[2].start = ac;
		ac[l - 1] = BIT(l) - 1;
		j.ht[2].length = 16 + ac[l - 1];
		KUNIT_EXPECT_EQ_MSG(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480),
				    0, "L = %u", l);
		ac[l - 1] = BIT(l);
		j.ht[2].length = 16 + ac[l - 1];
		KUNIT_EXPECT_EQ_MSG(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480),
				    -EINVAL, "L = %u", l);
	}

	/* The code counts add up to the number of values, 12 here. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	memcpy(dc, v4l2_jpeg_ref_table_luma_dc_ht, sizeof(dc));
	j.ht[0].start = dc;
	j.ht[0].length = sizeof(dc);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), 0);
	dc[8]++;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 640, 480), -EINVAL);
	dc[8] -= 2;
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

/*
 * The table buffer as bytes, in the order the hardware reads them: each
 * little-endian word from bits 31:24 down.
 */
static void table_bytes(struct kunit *test, const u8 *buf, u8 *bytes)
{
	unsigned int i;

	for (i = 0; i < HANTRO_JPEG_DEC_TABLE_SIZE; i += 4)
		put_unaligned_be32(get_unaligned_le32(buf + i), bytes + i);
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

	/*
	 * The first word: the first four luma quantizers, first in 31:24,
	 * stored little-endian on any CPU.
	 */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(buf),
			(u32)luma[0] << 24 | luma[1] << 16 | luma[2] << 8 | luma[3]);
	KUNIT_EXPECT_EQ(test, buf[0], luma[3]);
	KUNIT_EXPECT_EQ(test, buf[3], luma[0]);
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
	/* 4:2:0 MCUs are whole macroblocks: never a fill, at any size. */
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1000, 750), 0);
	KUNIT_EXPECT_EQ(test, fill_bits(V4L2_JPEG_CHROMA_SUBSAMPLING_420, 333, 251), 0);
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
	/* Ri from a DRI segment: every MCU, two, odd ones, a row, the largest. */
	static const u16 intervals[] = { 1, 2, 67, 125, 0xffff };
	static const struct {
		u16 ri;
		int ret;
	} restarts[] = {
		{ 0, 0 }, { 125, 0 }, { 250, 0 }, { 125 * 524, 0 },
		{ 1, -EINVAL }, { 2, -EINVAL }, { 67, -EINVAL },
		{ 124, -EINVAL }, { 126, -EINVAL }, { 0xffff, -EINVAL },
	};
	/* Widths: 192 whole, 200 and 216 FILRIGHT, 204 two block columns. */
	static const struct {
		u16 w, h;
		int ret;
	} fills[] = {
		{ 200, 400, -EINVAL }, { 216, 400, -EINVAL },
		{ 200, 410, -EINVAL }, { 200, 415, -EINVAL },
		{ 1000, 1440, -EINVAL },
		{ 200, 408, 0 }, { 200, 392, 0 }, { 200, 401, 0 },
		{ 192, 400, 0 }, { 204, 400, 0 }, { 1000, 1448, 0 },
	};
	u8 *buf = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	u8 *bytes = kunit_kzalloc(test, HANTRO_JPEG_DEC_TABLE_SIZE, GFP_KERNEL);
	struct hantro_g1_jpeg_dec_regs regs, plain;
	unsigned int i;
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

	/*
	 * A restart interval: SYNC_MARKER_E (CTRL2 bit 25) and Ri in
	 * PJPEG_REST_FREQ (CTRL5 bits 15:0), as for three components.
	 */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1000, 1448);
	hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &plain);
	KUNIT_EXPECT_EQ(test, plain.ctrl2 & BIT(25), 0);
	KUNIT_EXPECT_EQ(test, plain.ctrl5, 0);
	for (i = 0; i < ARRAY_SIZE(intervals); i++) {
		u16 ri = intervals[i];

		j.hdr.restart_interval = ri;
		hantro_g1_jpeg_dec_regs(&j.hdr, 0x10000000, j.payload, &regs);
		KUNIT_EXPECT_EQ_MSG(test, regs.ctrl2, plain.ctrl2 | BIT(25),
				    "Ri %u", ri);
		KUNIT_EXPECT_EQ_MSG(test, regs.ctrl5, (u32)ri, "Ri %u", ri);
		KUNIT_EXPECT_EQ_MSG(test, regs.ctrl4, plain.ctrl4, "Ri %u", ri);
	}

	/* Restarts only at the end of a block row: 125 blocks per row. */
	for (i = 0; i < ARRAY_SIZE(restarts); i++) {
		j.hdr.restart_interval = restarts[i].ri;
		KUNIT_EXPECT_EQ_MSG(test, check(&j, V4L2_PIX_FMT_GREY, 1008, 1456),
				    restarts[i].ret, "Ri %u", restarts[i].ri);
	}
	/* 134 blocks per row. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 1072, 1448);
	j.hdr.restart_interval = 134;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), 0);
	j.hdr.restart_interval = 125;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 1072, 1456), -EINVAL);
	/* Three components take any interval. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1000, 1448);
	j.hdr.restart_interval = 67;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 1008, 1456), 0);

	/* FILRIGHT only together with FILDOWN. */
	for (i = 0; i < ARRAY_SIZE(fills); i++) {
		make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, fills[i].w,
			  fills[i].h);
		KUNIT_EXPECT_EQ_MSG(test, check(&j, V4L2_PIX_FMT_GREY,
						ALIGN(fills[i].w, 16),
						ALIGN(fills[i].h, 16)),
				    fills[i].ret, "%ux%u", fills[i].w, fills[i].h);
	}
	/* With restarts at whole block rows too; three components decode. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 200, 400);
	j.hdr.restart_interval = 50;
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_GREY, 208, 400), -EINVAL);
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 200, 400);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV12, 208, 400), 0);
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_422, 200, 400);
	KUNIT_EXPECT_EQ(test, check(&j, V4L2_PIX_FMT_NV16, 208, 400), 0);

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
	const struct hantro_variant *rk3026 = &rk3026_vpu_variant;
	const struct hantro_fmt *fmt;
	unsigned int count;

	if (!IS_ENABLED(CONFIG_VIDEO_HANTRO_ROCKCHIP))
		kunit_skip(test, "no Rockchip variants");

	/* RK3036, RK3066 and RK3188 do not decode JPEG. */
	find_fmt(&rk3036_vpu_variant, V4L2_PIX_FMT_JPEG, &count);
	KUNIT_EXPECT_EQ(test, count, 0);
	KUNIT_EXPECT_FALSE(test, rk3036_vpu_variant.codec & HANTRO_JPEG_DECODER);
	find_fmt(&rk3066_vpu_variant, V4L2_PIX_FMT_JPEG, &count);
	KUNIT_EXPECT_EQ(test, count, 0);
	KUNIT_EXPECT_FALSE(test, rk3066_vpu_variant.codec & HANTRO_JPEG_DECODER);

	/* The RK3026 does, with one JPEG entry. */
	fmt = find_fmt(rk3026, V4L2_PIX_FMT_JPEG, &count);
	KUNIT_ASSERT_EQ(test, count, 1);
	KUNIT_EXPECT_EQ(test, fmt->codec_mode, HANTRO_MODE_JPEG_DEC);
	KUNIT_EXPECT_TRUE(test, rk3026->codec & HANTRO_JPEG_DECODER);
	KUNIT_EXPECT_TRUE(test, rk3026->codec_ops[HANTRO_MODE_JPEG_DEC].run ==
			  hantro_g1_jpeg_dec_run);
}

/*
 * Which RK3026 CAPTURE formats a decoder context offers for its OUTPUT
 * format: NV16, GREY and RGB only for JPEG, the post-processed YUYV for
 * every coded format.
 */
static void jpeg_dec_raw_fmt_allowed(struct kunit *test)
{
	static const u32 raw[] = {
		V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV16, V4L2_PIX_FMT_GREY,
		V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_XBGR32, V4L2_PIX_FMT_XRGB32,
	};
	static const struct {
		u32 src;
		bool encoder;
		/* NV12, NV16, GREY, RGB565, XBGR32, XRGB32, YUYV */
		bool allowed[7];
	} cases[] = {
		{ V4L2_PIX_FMT_JPEG, false,
		  { true, true, true, true, true, true, true } },
		{ V4L2_PIX_FMT_H264_SLICE, false,
		  { true, false, false, false, false, false, true } },
		{ V4L2_PIX_FMT_VP8_FRAME, false,
		  { true, false, false, false, false, false, true } },
		/* OUTPUT not set yet, or set to a raw format: no filter. */
		{ 0, false, { true, true, true, true, true, true, true } },
		{ V4L2_PIX_FMT_NV12, false,
		  { true, true, true, true, true, true, true } },
		/* An encoder's raw formats are its OUTPUT formats. */
		{ V4L2_PIX_FMT_JPEG, true,
		  { true, true, true, true, true, true, true } },
	};
	const struct hantro_variant *rk3026 = &rk3026_vpu_variant;
	const struct hantro_fmt *fmts[7];
	struct hantro_ctx *ctx;
	struct hantro_dev *dev;
	unsigned int c, i, count;

	if (!IS_ENABLED(CONFIG_VIDEO_HANTRO_ROCKCHIP))
		kunit_skip(test, "no Rockchip variants");

	dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	dev->variant = rk3026;
	ctx->dev = dev;

	for (i = 0; i < ARRAY_SIZE(raw); i++) {
		fmts[i] = find_fmt(rk3026, raw[i], &count);
		KUNIT_ASSERT_EQ(test, count, 1);
	}
	KUNIT_ASSERT_EQ(test, rk3026->num_postproc_fmts, 1);
	fmts[6] = &rk3026->postproc_fmts[0];
	KUNIT_ASSERT_EQ(test, fmts[6]->fourcc, V4L2_PIX_FMT_YUYV);

	for (c = 0; c < ARRAY_SIZE(cases); c++) {
		ctx->is_encoder = cases[c].encoder;
		ctx->vpu_src_fmt = NULL;
		if (cases[c].src) {
			ctx->vpu_src_fmt = find_fmt(rk3026, cases[c].src, &count);
			KUNIT_ASSERT_EQ(test, count, 1);
		}
		for (i = 0; i < ARRAY_SIZE(fmts); i++)
			KUNIT_EXPECT_EQ_MSG(test,
					    hantro_raw_fmt_allowed(ctx, fmts[i]),
					    cases[c].allowed[i],
					    "case %u, %p4cc", c, &fmts[i]->fourcc);
	}
}

/* NV12 and the decoder's formats: the buffer sizes TRY_FMT gives. */
static void jpeg_dec_cap_size(struct kunit *test)
{
	static const struct {
		u32 src_w, src_h;	/* the picture */
		u32 fourcc;
		u32 w, h;		/* asked for */
		u32 want_w, want_h;
	} cases[] = {
		/* The decoder's formats: its own buffer, whatever the size. */
		{ 1000, 744, V4L2_PIX_FMT_NV16, 0, 0, 1008, 752 },
		{ 1000, 744, V4L2_PIX_FMT_NV16, 640, 480, 1008, 752 },
		{ 1000, 744, V4L2_PIX_FMT_GREY, 4000, 4000, 1008, 752 },
		/* NV12: its own buffer for 0 or a size that holds the picture. */
		{ 1000, 744, V4L2_PIX_FMT_NV12, 0, 0, 1008, 752 },
		{ 1000, 744, V4L2_PIX_FMT_NV12, 0, 480, 1008, 752 },
		{ 1000, 744, V4L2_PIX_FMT_NV12, 1008, 752, 1008, 752 },
		{ 1000, 744, V4L2_PIX_FMT_NV12, 1000, 744, 1008, 752 },
		{ 1000, 744, V4L2_PIX_FMT_NV12, 1072, 1448, 1008, 752 },
		{ 4000, 3000, V4L2_PIX_FMT_NV12, 4000, 3000, 4000, 3008 },
		{ 4000, 3000, V4L2_PIX_FMT_NV12, 5000, 3000, 4000, 3008 },
		{ 5000, 3000, V4L2_PIX_FMT_NV12, 5000, 3000, 5008, 3008 },
		{ 8176, 48, V4L2_PIX_FMT_NV12, 8176, 48, 8176, 48 },
		/* ... each direction rounded up to 8: the size in the header. */
		{ 1000, 744, V4L2_PIX_FMT_NV12, 993, 737, 1008, 752 },
		{ 336, 256, V4L2_PIX_FMT_NV12, 333, 251, 336, 256 },
		{ 1000, 752, V4L2_PIX_FMT_NV12, 1000, 750, 1008, 752 },
		{ 208, 400, V4L2_PIX_FMT_NV12, 203, 397, 208, 400 },
		{ 1008, 752, V4L2_PIX_FMT_NV12, 1004, 750, 1008, 752 },
		/* ... also once the steps round it up to the picture's size. */
		{ 32, 32, V4L2_PIX_FMT_NV12, 16, 16, 32, 32 },
		/* Smaller in either direction: the post-processor's buffer. */
		{ 1000, 744, V4L2_PIX_FMT_NV12, 992, 744, 992, 744 },
		{ 1000, 744, V4L2_PIX_FMT_NV12, 1000, 736, 1000, 736 },
		{ 1000, 744, V4L2_PIX_FMT_NV12, 1072, 736, 1072, 736 },
		{ 1000, 752, V4L2_PIX_FMT_NV12, 1000, 744, 1000, 744 },
		{ 1000, 744, V4L2_PIX_FMT_NV12, 641, 481, 648, 482 },
		{ 1000, 744, V4L2_PIX_FMT_NV12, 20, 20, 48, 48 },
		{ 4000, 3000, V4L2_PIX_FMT_NV12, 1072, 1448, 1072, 1448 },
		{ 4000, 3000, V4L2_PIX_FMT_NV12, 4000, 2992, 4000, 2992 },
		{ 5000, 3000, V4L2_PIX_FMT_NV12, 4992, 3000, 4088, 3000 },
		{ 8176, 48, V4L2_PIX_FMT_NV12, 8000, 48, 4088, 48 },
	};
	struct hantro_g1_jpeg_dec_out out;
	unsigned int i;
	u32 w, h;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		make_out(&out, cases[i].src_w, cases[i].src_h, cases[i].fourcc,
			 0, 0);
		w = cases[i].w;
		h = cases[i].h;
		hantro_g1_jpeg_dec_cap_size(&out, &w, &h);
		KUNIT_EXPECT_EQ_MSG(test, w, cases[i].want_w, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, h, cases[i].want_h, "case %u", i);
		/* Applied again, it keeps the size. */
		hantro_g1_jpeg_dec_cap_size(&out, &w, &h);
		KUNIT_EXPECT_EQ_MSG(test, w, cases[i].want_w, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, h, cases[i].want_h, "case %u", i);
	}

	/* Without a post-processor, NV12 is the decoder's alone. */
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV12, 0, 0);
	out.pp_max_width = 0;
	w = 640;
	h = 480;
	hantro_g1_jpeg_dec_cap_size(&out, &w, &h);
	KUNIT_EXPECT_EQ(test, w, 1008);
	KUNIT_EXPECT_EQ(test, h, 752);
}

/*
 * ENUM_FRAMESIZES of NV12, NV16 and GREY: the decoder's own buffer, taken
 * by TRY_FMT as it is, also where it is wider than the widest stride.
 */
static void jpeg_dec_cap_frmsize(struct kunit *test)
{
	static const u32 fourccs[] = {
		V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV16, V4L2_PIX_FMT_GREY,
	};
	static const u32 pics[][2] = {
		{ 1000, 744 }, { 4000, 3000 }, { 8176, 48 }, { 48, 8176 },
	};
	struct v4l2_frmsizeenum fsize = { };
	struct hantro_g1_jpeg_dec_out out;
	unsigned int f, i;
	u32 w, h;

	for (i = 0; i < ARRAY_SIZE(pics); i++) {
		for (f = 0; f < ARRAY_SIZE(fourccs); f++) {
			make_out(&out, pics[i][0], pics[i][1], fourccs[f], 0, 0);
			fsize.index = 0;
			KUNIT_ASSERT_EQ(test,
					hantro_g1_jpeg_dec_cap_frmsize(&out, &fsize),
					0);
			KUNIT_EXPECT_EQ(test, fsize.type,
					V4L2_FRMSIZE_TYPE_DISCRETE);
			KUNIT_EXPECT_EQ(test, fsize.discrete.width,
					ALIGN(pics[i][0], 16));
			KUNIT_EXPECT_EQ(test, fsize.discrete.height,
					ALIGN(pics[i][1], 16));
			w = fsize.discrete.width;
			h = fsize.discrete.height;
			hantro_g1_jpeg_dec_cap_size(&out, &w, &h);
			KUNIT_EXPECT_EQ_MSG(test, w, fsize.discrete.width,
					    "%ux%u %p4cc", pics[i][0], pics[i][1],
					    &fourccs[f]);
			KUNIT_EXPECT_EQ_MSG(test, h, fsize.discrete.height,
					    "%ux%u %p4cc", pics[i][0], pics[i][1],
					    &fourccs[f]);
			fsize.index = 1;
			KUNIT_EXPECT_EQ(test,
					hantro_g1_jpeg_dec_cap_frmsize(&out, &fsize),
					-EINVAL);
		}
	}

	/* NV12 without a post-processor. */
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV12, 0, 0);
	out.pp_max_width = 0;
	fsize.index = 0;
	KUNIT_ASSERT_EQ(test, hantro_g1_jpeg_dec_cap_frmsize(&out, &fsize), 0);
	KUNIT_EXPECT_EQ(test, fsize.type, V4L2_FRMSIZE_TYPE_DISCRETE);
	KUNIT_EXPECT_EQ(test, fsize.discrete.width, 1008);
	KUNIT_EXPECT_EQ(test, fsize.discrete.height, 752);
}

static void expect_rect(struct kunit *test, const struct v4l2_rect *r,
			s32 left, s32 top, u32 width, u32 height,
			const char *what)
{
	KUNIT_EXPECT_EQ_MSG(test, r->left, left, "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, r->top, top, "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, r->width, width, "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, r->height, height, "%s", what);
}

/* The default compose rectangle and the one S_SELECTION gives. */
static void jpeg_dec_compose(struct kunit *test)
{
	struct hantro_g1_jpeg_dec_out out;
	struct v4l2_rect r;

	/* The decoder's own buffer: the whole picture. */
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV12, 1008, 752);
	expect_rect(test, &out.compose, 0, 0, 1000, 744, "dec");

	/* A smaller buffer: the picture fitted, aspect ratio kept. */
	make_out(&out, 4000, 3000, V4L2_PIX_FMT_NV12, 1072, 1448);
	expect_rect(test, &out.compose, 0, 0, 1072, 804, "fit width");
	make_out(&out, 3000, 4000, V4L2_PIX_FMT_NV12, 1072, 1072);
	expect_rect(test, &out.compose, 0, 0, 800, 1072, "fit height");
	/* ... and the widest output. */
	make_out(&out, 4000, 3000, V4L2_PIX_FMT_NV12, 4088, 2048);
	expect_rect(test, &out.compose, 0, 0, 1920, 1440, "fit 1920");

	/*
	 * Set: inside the buffer, no larger than the picture, the width in
	 * steps of 8, the height in steps of 2, at a multiple of 8 bytes in
	 * an even line.
	 */
	make_out(&out, 4000, 3000, V4L2_PIX_FMT_NV12, 1072, 1448);
	r = (struct v4l2_rect){ 13, 7, 536, 403 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	expect_rect(test, &r, 8, 6, 536, 402, "steps");
	r = (struct v4l2_rect){ 100, 51, 2000, 2000 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	expect_rect(test, &r, 0, 0, 1072, 1448, "buffer");
	r = (struct v4l2_rect){ -8, -2, 10, 10 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	expect_rect(test, &r, 0, 0, 48, 48, "smallest");
	r = (struct v4l2_rect){ 1072, 1448, 536, 400 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	expect_rect(test, &r, 536, 1048, 536, 400, "bottom right");
	make_out(&out, 640, 480, V4L2_PIX_FMT_NV12, 1072, 472);
	r = (struct v4l2_rect){ 216, 484, 1000, 1000 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	expect_rect(test, &r, 216, 0, 640, 472, "no up-scaling");
	make_out(&out, 4000, 3000, V4L2_PIX_FMT_NV12, 4088, 2048);
	r = (struct v4l2_rect){ 0, 0, 4000, 3000 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	expect_rect(test, &r, 0, 0, 1920, 2046, "widest");

	/* The decoder's formats, and buffers wider than the stride: fixed. */
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV16, 1008, 752);
	r = (struct v4l2_rect){ 0, 0, 536, 400 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	expect_rect(test, &r, 0, 0, 1000, 744, "NV16");
	make_out(&out, 8176, 48, V4L2_PIX_FMT_NV12, 8176, 48);
	r = (struct v4l2_rect){ 0, 0, 48, 48 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	expect_rect(test, &r, 0, 0, 8176, 48, "8176 wide");

	/* What the hardware writes: whole macroblocks where it is 1:1. */
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV12, 1008, 752);
	hantro_g1_jpeg_dec_compose_padded(&out, &r);
	expect_rect(test, &r, 0, 0, 1008, 752, "padded dec");
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV12, 1072, 736);
	out.compose = (struct v4l2_rect){ 8, 0, 1000, 736 };
	hantro_g1_jpeg_dec_compose_padded(&out, &r);
	expect_rect(test, &r, 8, 0, 1008, 736, "padded 1:1");
	out.compose = (struct v4l2_rect){ 72, 0, 1000, 736 };
	hantro_g1_jpeg_dec_compose_padded(&out, &r);
	expect_rect(test, &r, 72, 0, 1000, 736, "padded no room");
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV12, 992, 1448);
	out.compose = (struct v4l2_rect){ 0, 0, 536, 744 };
	hantro_g1_jpeg_dec_compose_padded(&out, &r);
	expect_rect(test, &r, 0, 0, 536, 752, "padded scaled");
}

/* G1_REG_PP_DEV_CONFIG: OUT_ENDIAN 6, OUT_SWAP32_E 5, burst length 16. */
#define PP_DEV_CONFIG_YUV	(BIT(6) | BIT(5) | 16)

struct pp_image {
	u32 input_size;		/* swreg72 */
	u32 scaling1;		/* swreg80 */
	u32 scaling_inv;	/* swreg81 */
	u32 control;		/* swreg85 */
	u32 orig_width;		/* swreg88 */
	u32 display_width;	/* swreg92 */
	u32 luma_offset;
	u32 chroma_offset;
};

static void expect_pp(struct kunit *test, const struct hantro_g1_pp_regs *pp,
		      const struct pp_image *want, const char *what)
{
	KUNIT_EXPECT_EQ_MSG(test, pp->dev_config, PP_DEV_CONFIG_YUV, "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, pp->input_size, want->input_size, "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, pp->scaling1, want->scaling1, "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, pp->scaling_inv, want->scaling_inv, "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, pp->control, want->control, "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, pp->orig_width, want->orig_width, "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, pp->display_width, want->display_width,
			    "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, pp->luma_offset, want->luma_offset,
			    "%s", what);
	KUNIT_EXPECT_EQ_MSG(test, pp->chroma_offset, want->chroma_offset,
			    "%s", what);
}

/*
 * Register fields: INPUT_SIZE height 16:9, width 8:0 (macroblocks);
 * SCALING1 HOR_SCALE_MODE 26:25, VER_SCALE_MODE 24:23 (2: down);
 * SCALING_INV WSCALE_INVRA 31:16, HSCALE_INVRA 15:0; CONTROL IN_FMT 31:29
 * (1: 4:2:0, 4: 4:2:2, 3: 4:0:0), OUT_FMT 28:26 (5: NV12), OUT_HEIGHT
 * 25:15, OUT_WIDTH 14:4, CROP8_R_E 1, CROP8_D_E 0; ORIG_WIDTH 31:23;
 * DISPLAY_WIDTH 11:0, IN_W_EXT 28:26, IN_H_EXT 31:29.
 */
#define PP_MODES_DOWN		(2u << 25 | 2u << 23)
#define PP_CONTROL(in, out, w, h) \
	((u32)(in) << 29 | (u32)(out) << 26 | (u32)(h) << 15 | (u32)(w) << 4)

/* How a job writes the CAPTURE buffer: the decoder, or the PP's registers. */
static void jpeg_dec_job_out(struct kunit *test)
{
	struct hantro_g1_jpeg_dec_out out;
	struct hantro_g1_pp_regs pp;
	struct test_jpeg j;
	bool use_pp;

	/* The decoder's own buffer and format, the whole picture. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1000, 744);
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV12, 1008, 752);
	KUNIT_ASSERT_EQ(test, check_out(&j, &out, &pp, &use_pp), 0);
	KUNIT_EXPECT_FALSE(test, use_pp);

	/*
	 * Scaled down into a wider, lower buffer at (8, 2): the picture
	 * rounded up to 8, 63x47 macroblocks less the 8-pixel crops of the
	 * last ones, both directions down; 536 / 1000 and 400 / 744 (the
	 * vertical one a unit up).
	 */
	out = (struct hantro_g1_jpeg_dec_out){
		.src_width = 1000, .src_height = 744,
		.fourcc = V4L2_PIX_FMT_NV12, .width = 1072, .height = 736,
		.compose = { 8, 2, 536, 400 }, .pp_max_width = 1920,
	};
	KUNIT_ASSERT_EQ(test, check_out(&j, &out, &pp, &use_pp), 0);
	KUNIT_EXPECT_TRUE(test, use_pp);
	expect_pp(test, &pp, &(struct pp_image){
		.input_size = 47 << 9 | 63,
		.scaling1 = PP_MODES_DOWN,
		.scaling_inv = (536u << 16) / 1000 << 16 | ((400u << 16) / 744 + 1),
		.control = PP_CONTROL(1, 5, 536, 400) | BIT(1) | BIT(0),
		.orig_width = 63u << 23,
		.display_width = 1072,
		.luma_offset = 2 * 1072 + 8,
		.chroma_offset = 1072 * 736 + 1072 + 8,
	}, "420 scaled");

	/*
	 * NV12 of a 4:2:2 or greyscale picture in the decoder's buffer: the
	 * post-processor reads and writes whole macroblocks, 1:1.
	 */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_422, 1000, 744);
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV12, 1008, 752);
	KUNIT_ASSERT_EQ(test, check_out(&j, &out, &pp, &use_pp), 0);
	KUNIT_EXPECT_TRUE(test, use_pp);
	expect_pp(test, &pp, &(struct pp_image){
		.input_size = 47 << 9 | 63,
		.control = PP_CONTROL(4, 5, 1008, 752),
		.orig_width = 63u << 23,
		.display_width = 1008,
		.chroma_offset = 1008 * 752,
	}, "422 1:1");
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 216, 408);
	make_out(&out, 216, 408, V4L2_PIX_FMT_NV12, 224, 416);
	KUNIT_ASSERT_EQ(test, check_out(&j, &out, &pp, &use_pp), 0);
	expect_pp(test, &pp, &(struct pp_image){
		.input_size = 26 << 9 | 14,
		.control = PP_CONTROL(3, 5, 224, 416),
		.orig_width = 14u << 23,
		.display_width = 224,
		.chroma_offset = 224 * 416,
	}, "grey 1:1");
	/* Scaled in one direction: the other one reads whole macroblocks. */
	out.width = 224;
	out.height = 208;
	hantro_g1_jpeg_dec_compose_default(&out, &out.compose);
	expect_rect(test, &out.compose, 0, 0, 104, 208, "grey fit");
	out.compose.width = 216;
	KUNIT_ASSERT_EQ(test, check_out(&j, &out, &pp, &use_pp), 0);
	expect_pp(test, &pp, &(struct pp_image){
		.input_size = 26 << 9 | 14,
		.scaling1 = 2u << 23,
		.scaling_inv = (208u << 16) / 408 + 1,
		.control = PP_CONTROL(3, 5, 224, 208) | BIT(0),
		.orig_width = 14u << 23,
		.display_width = 224,
		.chroma_offset = 224 * 208,
	}, "grey height only");

	/* 8176 to 48 (about 1/170), and 8176 lines (IN_H_EXT) to 2046. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 8176, 48);
	make_out(&out, 8176, 48, V4L2_PIX_FMT_NV12, 48, 48);
	KUNIT_ASSERT_EQ(test, check_out(&j, &out, &pp, &use_pp), 0);
	expect_pp(test, &pp, &(struct pp_image){
		.input_size = 3 << 9 | (511 & 0x1ff),
		.scaling1 = 2u << 25,
		.scaling_inv = 384u << 16,
		.control = PP_CONTROL(1, 5, 48, 48),
		.orig_width = 511u << 23,
		.display_width = 48,
		.chroma_offset = 48 * 48,
	}, "8176 wide");
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 48, 8176);
	make_out(&out, 48, 8176, V4L2_PIX_FMT_NV12, 48, 2046);
	KUNIT_ASSERT_EQ(test, check_out(&j, &out, &pp, &use_pp), 0);
	expect_pp(test, &pp, &(struct pp_image){
		.input_size = (511 & 0xff) << 9 | 3,
		.scaling1 = 2u << 23,
		.scaling_inv = (2046u << 16) / 8176 + 1,
		.control = PP_CONTROL(1, 5, 48, 2046),
		.orig_width = 3u << 23,
		.display_width = 48 | 1u << 29,
		.chroma_offset = 48 * 2046,
	}, "8176 high");

	/* Refused: no format or compose rectangle that either writes. */
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_422, 4000, 3000);
	make_out(&out, 4000, 3000, V4L2_PIX_FMT_NV12, 4000, 3008);
	KUNIT_EXPECT_EQ(test, check_out(&j, &out, &pp, &use_pp), -EINVAL);
	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1000, 744);
	make_out(&out, 1000, 744, V4L2_PIX_FMT_NV12, 1072, 736);
	out.compose.width = 532;
	KUNIT_EXPECT_EQ(test, check_out(&j, &out, &pp, &use_pp), -EINVAL);
	out.compose = (struct v4l2_rect){ 4, 0, 536, 400 };
	KUNIT_EXPECT_EQ(test, check_out(&j, &out, &pp, &use_pp), -EINVAL);
	out.compose = (struct v4l2_rect){ 536, 0, 544, 400 };
	KUNIT_EXPECT_EQ(test, check_out(&j, &out, &pp, &use_pp), -EINVAL);
	out.compose = (struct v4l2_rect){ 0, 0, 536, 400 };
	out.pp_max_width = 0;
	KUNIT_EXPECT_EQ(test, check_out(&j, &out, &pp, &use_pp), -EINVAL);
}

/* The JPEG decoder's selection targets, through the codec ops. */
static void jpeg_dec_selection(struct kunit *test)
{
	const struct hantro_variant *rk3026 = &rk3026_vpu_variant;
	const struct hantro_codec_ops *ops;
	struct v4l2_selection sel = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
	struct hantro_ctx *ctx;
	struct hantro_dev *dev;
	u32 w = 0, h = 0;

	if (!IS_ENABLED(CONFIG_VIDEO_HANTRO_ROCKCHIP))
		kunit_skip(test, "no Rockchip variants");

	dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	dev->variant = rk3026;
	ctx->dev = dev;
	ops = &rk3026->codec_ops[HANTRO_MODE_JPEG_DEC];
	KUNIT_ASSERT_TRUE(test, ops->cap_try_fmt && ops->cap_enum_framesizes &&
			  ops->cap_selection);

	ctx->src_fmt.width = 4000;
	ctx->src_fmt.height = 3000;
	ctx->dst_fmt.pixelformat = V4L2_PIX_FMT_NV12;
	ctx->dst_fmt.width = 1072;
	ctx->dst_fmt.height = 1448;
	ctx->compose = (struct v4l2_rect){ 0, 322, 1072, 804 };

	sel.target = V4L2_SEL_TGT_CROP;
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, false), 0);
	expect_rect(test, &sel.r, 0, 0, 4000, 3000, "crop");
	sel.target = V4L2_SEL_TGT_CROP_BOUNDS;
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, false), 0);
	expect_rect(test, &sel.r, 0, 0, 4000, 3000, "crop bounds");
	sel.target = V4L2_SEL_TGT_COMPOSE;
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, false), 0);
	expect_rect(test, &sel.r, 0, 322, 1072, 804, "compose");
	sel.target = V4L2_SEL_TGT_COMPOSE_DEFAULT;
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, false), 0);
	expect_rect(test, &sel.r, 0, 0, 1072, 804, "compose default");
	sel.target = V4L2_SEL_TGT_COMPOSE_BOUNDS;
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, false), 0);
	expect_rect(test, &sel.r, 0, 0, 1072, 1448, "compose bounds");
	sel.target = V4L2_SEL_TGT_COMPOSE_PADDED;
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, false), 0);
	expect_rect(test, &sel.r, 0, 322, 1072, 804, "compose padded");
	sel.target = 0xffff;
	KUNIT_EXPECT_EQ(test, ops->cap_selection(ctx, &sel, false), -EINVAL);

	/* Only the compose rectangle can be set. */
	sel.target = V4L2_SEL_TGT_COMPOSE;
	sel.r = (struct v4l2_rect){ 12, 3, 540, 401 };
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, true), 0);
	expect_rect(test, &sel.r, 8, 2, 536, 400, "set compose");
	sel.target = V4L2_SEL_TGT_CROP;
	KUNIT_EXPECT_EQ(test, ops->cap_selection(ctx, &sel, true), -EINVAL);
	sel.target = V4L2_SEL_TGT_COMPOSE_DEFAULT;
	KUNIT_EXPECT_EQ(test, ops->cap_selection(ctx, &sel, true), -EINVAL);

	/* The flags: -ERANGE where the adjusted rectangle breaks them. */
	sel.target = V4L2_SEL_TGT_COMPOSE;
	sel.flags = V4L2_SEL_FLAG_LE;
	sel.r = (struct v4l2_rect){ 8, 2, 540, 401 };
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, true), 0);
	expect_rect(test, &sel.r, 8, 2, 536, 400, "LE");
	sel.flags = V4L2_SEL_FLAG_GE;
	sel.r = (struct v4l2_rect){ 8, 2, 540, 401 };
	KUNIT_EXPECT_EQ(test, ops->cap_selection(ctx, &sel, true), -ERANGE);
	sel.r = (struct v4l2_rect){ 12, 2, 536, 400 };
	KUNIT_EXPECT_EQ(test, ops->cap_selection(ctx, &sel, true), -ERANGE);
	sel.flags = V4L2_SEL_FLAG_GE | V4L2_SEL_FLAG_LE;
	sel.r = (struct v4l2_rect){ 8, 2, 536, 400 };
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, true), 0);
	expect_rect(test, &sel.r, 8, 2, 536, 400, "GE | LE");

	/* The decoder's own buffer: the compose rectangle stays the picture. */
	ctx->dst_fmt.pixelformat = V4L2_PIX_FMT_NV16;
	ctx->dst_fmt.width = 4000;
	ctx->dst_fmt.height = 3008;
	sel.flags = 0;
	sel.r = (struct v4l2_rect){ 0, 0, 536, 400 };
	KUNIT_ASSERT_EQ(test, ops->cap_selection(ctx, &sel, true), 0);
	expect_rect(test, &sel.r, 0, 0, 4000, 3000, "NV16");
	sel.flags = V4L2_SEL_FLAG_LE;
	sel.r = (struct v4l2_rect){ 0, 0, 536, 400 };
	KUNIT_EXPECT_EQ(test, ops->cap_selection(ctx, &sel, true), -ERANGE);
	ctx->dst_fmt.pixelformat = V4L2_PIX_FMT_NV12;

	/* TRY_FMT: a format the decoder neither writes nor post-processes. */
	KUNIT_EXPECT_EQ(test, ops->cap_try_fmt(ctx, V4L2_PIX_FMT_NV21, &w, &h),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, ops->cap_try_fmt(ctx, V4L2_PIX_FMT_NV12, &w, &h),
			0);
	KUNIT_EXPECT_EQ(test, w, 4000);
	KUNIT_EXPECT_EQ(test, h, 3008);
}

/* Bits hi:lo of a register value. */
static u32 reg_field(u32 reg, unsigned int hi, unsigned int lo)
{
	return (reg & GENMASK(hi, lo)) >> lo;
}

/* G1_REG_PP_DEV_CONFIG for RGB: OUT_SWAP32_E 5, burst length 16. */
#define PP_DEV_CONFIG_RGB	(BIT(5) | 16)

/* The post-processor's output formats, as a job programs them. */
static const struct {
	u32 fourcc;
	u32 out_fmt;		/* PP_OUT_FORMAT, 28:26 */
	u32 cpp;
	bool rgb, rgb16;
	u32 mask_r, mask_g, mask_b;
	u32 padd_r, padd_g, padd_b;
} pp_formats[] = {
	{ V4L2_PIX_FMT_NV12, 5, 1 },
	{ V4L2_PIX_FMT_YUYV, 3, 2 },
	{ V4L2_PIX_FMT_RGB565, 0, 2, true, true,
	  0xf800f800, 0x07e007e0, 0x001f001f, 0, 5, 11 },
	{ V4L2_PIX_FMT_XBGR32, 0, 4, true, false,
	  0x00ff0000, 0x0000ff00, 0x000000ff, 8, 16, 24 },
	{ V4L2_PIX_FMT_XRGB32, 0, 4, true, false,
	  0x0000ff00, 0x00ff0000, 0xff000000, 16, 8, 0 },
};

/*
 * The whole register image of a job, per output format and sampling: the
 * format field, byte order, conversion, masks and their padding, the
 * input and scaling, the stride, the position and the chroma plane.
 */
static void jpeg_dec_pp_formats(struct kunit *test)
{
	static const struct {
		enum v4l2_jpeg_chroma_subsampling sampling;
		u32 in_fmt;
		u16 w, h;			/* the picture */
		u32 buf_w, buf_h;
		struct v4l2_rect compose;
		u32 input_size, scaling1, scaling_inv, crops, orig_width;
	} jobs[] = {
		/* 4:2:0, scaled down to 536x400 at (16, 2) */
		{ V4L2_JPEG_CHROMA_SUBSAMPLING_420, 1, 1000, 744, 1072, 736,
		  { 16, 2, 536, 400 }, 47 << 9 | 63, PP_MODES_DOWN,
		  (536u << 16) / 1000 << 16 | ((400u << 16) / 744 + 1),
		  BIT(1) | BIT(0), 63 },
		/* 4:2:2 at 1:1, whole macroblocks */
		{ V4L2_JPEG_CHROMA_SUBSAMPLING_422, 4, 1000, 744, 1008, 752,
		  { 0, 0, 1000, 744 }, 47 << 9 | 63, 0, 0, 0, 63 },
		/* greyscale, scaled down to 104x200 */
		{ V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, 3, 216, 408, 224, 416,
		  { 0, 0, 104, 200 }, 26 << 9 | 14, PP_MODES_DOWN,
		  (104u << 16) / 216 << 16 | ((200u << 16) / 408 + 1),
		  BIT(1) | BIT(0), 14 },
	};
	struct hantro_g1_jpeg_dec_out out;
	struct hantro_g1_pp_regs pp;
	struct test_jpeg j;
	unsigned int f, i;
	bool use_pp;

	for (i = 0; i < ARRAY_SIZE(jobs); i++) {
		make_jpeg(&j, jobs[i].sampling, jobs[i].w, jobs[i].h);
		for (f = 0; f < ARRAY_SIZE(pp_formats); f++) {
			u32 fourcc = pp_formats[f].fourcc;
			const struct v4l2_rect *c = &jobs[i].compose;
			u32 w = c->width, h = c->height;
			u32 scaling0 = 0, swap16 = 0;
			u32 chroma = 0;

			/* At 1:1, whole macroblocks: 1008x752. */
			if (!jobs[i].scaling1) {
				w = ALIGN(w, 16);
				h = ALIGN(h, 16);
			}
			if (pp_formats[f].rgb)
				scaling0 = BIT(29) | pp_formats[f].padd_r << 23 |
					   pp_formats[f].padd_g << 18;
			if (pp_formats[f].rgb16) {
				scaling0 |= BIT(28);
				swap16 = BIT(2);
			}
			if (fourcc == V4L2_PIX_FMT_NV12)
				chroma = jobs[i].buf_w * jobs[i].buf_h +
					 c->top / 2 * jobs[i].buf_w + c->left;

			out = (struct hantro_g1_jpeg_dec_out){
				.src_width = jobs[i].w, .src_height = jobs[i].h,
				.fourcc = fourcc, .width = jobs[i].buf_w,
				.height = jobs[i].buf_h, .compose = *c,
				.pp_max_width = TEST_PP_MAX_WIDTH,
			};
			KUNIT_ASSERT_EQ_MSG(test, check_out(&j, &out, &pp, &use_pp),
					    0, "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_TRUE_MSG(test, use_pp, "job %u %p4cc", i,
					      &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.dev_config,
					    pp_formats[f].rgb ? PP_DEV_CONFIG_RGB :
					    PP_DEV_CONFIG_YUV,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.input_size,
					    jobs[i].input_size,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.control,
					    PP_CONTROL(jobs[i].in_fmt,
						       pp_formats[f].out_fmt, w, h) |
					    swap16 | jobs[i].crops,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.scaling0, scaling0,
					    "job %u %p4cc", i, &fourcc);
			/* SCALING1: RGB_B_PADD 22:18 next to the modes. */
			KUNIT_EXPECT_EQ_MSG(test, pp.scaling1,
					    jobs[i].scaling1 |
					    pp_formats[f].padd_b << 18,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.scaling_inv,
					    jobs[i].scaling_inv,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.mask_r, pp_formats[f].mask_r,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.mask_g, pp_formats[f].mask_g,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.mask_b, pp_formats[f].mask_b,
					    "job %u %p4cc", i, &fourcc);
			/* The conversion for RGB only (its weights below). */
			KUNIT_EXPECT_EQ_MSG(test, pp.color_conv != 0,
					    pp_formats[f].rgb,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.orig_width,
					    jobs[i].orig_width << 23,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.display_width, jobs[i].buf_w,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.luma_offset,
					    (c->top * jobs[i].buf_w + c->left) *
					    pp_formats[f].cpp,
					    "job %u %p4cc", i, &fourcc);
			KUNIT_EXPECT_EQ_MSG(test, pp.chroma_offset, chroma,
					    "job %u %p4cc", i, &fourcc);
		}
	}
}

/*
 * The bytes the post-processor stores for one output word, as the driver
 * reads the fields: each 8-bit colour value shifted right from the top of
 * the word by its PADD and masked; for a 16-bit format the first pixel in
 * the upper half, the second in the lower one (the masks repeat there);
 * OUT_SWAP16_E swaps the halves; the word goes to memory most significant
 * byte first with OUT_ENDIAN, least significant byte first without it.
 * This only checks that the registers give the V4L2 layouts under that
 * reading; the reading itself is the hardware's to confirm.
 */
static void pp_store(const struct hantro_g1_pp_regs *pp, const u8 rgb[2][3],
		     u8 *bytes)
{
	const u32 padd[3] = {
		reg_field(pp->scaling0, 27, 23),
		reg_field(pp->scaling0, 22, 18),
		reg_field(pp->scaling1, 22, 18),
	};
	const u32 mask[3] = { pp->mask_r, pp->mask_g, pp->mask_b };
	bool rgb16 = pp->scaling0 & BIT(28);
	unsigned int c;
	u32 word = 0;

	for (c = 0; c < 3; c++) {
		if (!rgb16) {
			word |= ((u32)rgb[0][c] << 24 >> padd[c]) & mask[c];
			continue;
		}
		word |= ((u32)rgb[0][c] << 24 >> padd[c]) & mask[c] &
			GENMASK(31, 16);
		word |= ((u32)rgb[1][c] << 24 >> (padd[c] + 16)) & mask[c] &
			GENMASK(15, 0);
	}

	if (pp->control & BIT(2))
		word = ror32(word, 16);

	if (pp->dev_config & BIT(6))
		put_unaligned_be32(word, bytes);
	else
		put_unaligned_le32(word, bytes);
}

/* A 640x480 picture at its own size in a buffer of fourcc. */
static void pp_regs_640(struct kunit *test, u32 fourcc,
			struct hantro_g1_pp_regs *pp)
{
	struct hantro_g1_jpeg_dec_out out;
	struct test_jpeg j;
	bool use_pp;

	make_jpeg(&j, V4L2_JPEG_CHROMA_SUBSAMPLING_420, 640, 480);
	make_out(&out, 640, 480, fourcc, 640, 480);
	KUNIT_ASSERT_EQ(test, check_out(&j, &out, pp, &use_pp), 0);
	KUNIT_ASSERT_TRUE(test, use_pp);
}

/* The memory layouts the V4L2 RGB formats are defined by. */
static void jpeg_dec_pp_byte_order(struct kunit *test)
{
	static const u8 rgb[2][3] = { { 0xf3, 0x1c, 0xa5 }, { 0x3b, 0xe6, 0x0f } };
	struct hantro_g1_pp_regs pp;
	u8 bytes[4];
	u16 px;

	/* XBGR32: B, G, R, X (DRM XRGB8888). */
	pp_regs_640(test, V4L2_PIX_FMT_XBGR32, &pp);
	pp_store(&pp, rgb, bytes);
	KUNIT_EXPECT_EQ(test, bytes[0], 0xa5);
	KUNIT_EXPECT_EQ(test, bytes[1], 0x1c);
	KUNIT_EXPECT_EQ(test, bytes[2], 0xf3);

	/* XRGB32: X, R, G, B (DRM BGRX8888). */
	pp_regs_640(test, V4L2_PIX_FMT_XRGB32, &pp);
	pp_store(&pp, rgb, bytes);
	KUNIT_EXPECT_EQ(test, bytes[1], 0xf3);
	KUNIT_EXPECT_EQ(test, bytes[2], 0x1c);
	KUNIT_EXPECT_EQ(test, bytes[3], 0xa5);

	/*
	 * RGB565: two little-endian 16-bit pixels in order, R in 15:11,
	 * G in 10:5, B in 4:0 (DRM RGB565).
	 */
	pp_regs_640(test, V4L2_PIX_FMT_RGB565, &pp);
	pp_store(&pp, rgb, bytes);
	px = (0xf3 >> 3) << 11 | (0x1c >> 2) << 5 | 0xa5 >> 3;
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(bytes), px);
	px = (0x3b >> 3) << 11 | (0xe6 >> 2) << 5 | 0x0f >> 3;
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(bytes + 2), px);

	/* NV12 and YUYV keep the byte order of the decoder's own output. */
	pp_regs_640(test, V4L2_PIX_FMT_YUYV, &pp);
	KUNIT_EXPECT_TRUE(test, pp.dev_config & BIT(6));
	KUNIT_EXPECT_FALSE(test, pp.control & BIT(2));
}

/*
 * The colour conversion: JFIF's coefficients (ITU-T T.871) in the 10-bit
 * fields with 8 fractional bits, Y at full range, no contrast or
 * brightness change.
 */
static void jpeg_dec_pp_coefficients(struct kunit *test)
{
	/* The coefficients in millionths. */
	static const s64 cr_r = 1402000, cr_g = 714136, cb_g = 344136,
			 cb_b = 1772000;
	struct hantro_g1_pp_regs pp;
	s64 a1, a2, b, c, d, e, f, err_r, err_g, err_b;

	pp_regs_640(test, V4L2_PIX_FMT_XBGR32, &pp);

	/* COEFFA2 27:18, COEFFA1 17:8, CONTRAST_THR2 7:0 */
	a2 = reg_field(pp.color_conv, 27, 18);
	a1 = reg_field(pp.color_conv, 17, 8);
	KUNIT_EXPECT_EQ(test, a1, 256);
	KUNIT_EXPECT_EQ(test, a2, 256);
	KUNIT_EXPECT_EQ(test, pp.color_conv & GENMASK(7, 0), 0);
	/* COEFFD 29:20 (Cb in G), COEFFC 19:10 (Cr in G), COEFFB 9:0 (Cr in R) */
	d = reg_field(pp.color_conv0, 29, 20);
	c = reg_field(pp.color_conv0, 19, 10);
	b = reg_field(pp.color_conv0, 9, 0);
	/* COEFFF 17:10 (brightness), COEFFE 9:0 (Cb in B) */
	f = reg_field(pp.color_conv1, 17, 10);
	e = reg_field(pp.color_conv1, 9, 0);
	KUNIT_EXPECT_EQ(test, f, 0);
	/* YCBCR_RANGE 29: full range. */
	KUNIT_EXPECT_EQ(test, reg_field(pp.scaling0, 29, 29), 1);

	KUNIT_EXPECT_EQ(test, b, 359);
	KUNIT_EXPECT_EQ(test, c, 183);
	KUNIT_EXPECT_EQ(test, d, 88);
	KUNIT_EXPECT_EQ(test, e, 454);

	/*
	 * At the largest chroma, |Cb - 128| = |Cr - 128| = 128, the rounded
	 * coefficients move R, G and B by less than a quarter of a level.
	 */
	err_r = abs(b * 1000000 / 2 - cr_r * 128);
	err_g = abs(c * 1000000 / 2 - cr_g * 128) +
		abs(d * 1000000 / 2 - cb_g * 128);
	err_b = abs(e * 1000000 / 2 - cb_b * 128);
	KUNIT_EXPECT_LT(test, err_r, 250000);
	KUNIT_EXPECT_LT(test, err_g, 250000);
	KUNIT_EXPECT_LT(test, err_b, 250000);

	/* The same conversion for every RGB format, none for YUV. */
	pp_regs_640(test, V4L2_PIX_FMT_RGB565, &pp);
	KUNIT_EXPECT_EQ(test, reg_field(pp.color_conv0, 9, 0), b);
	KUNIT_EXPECT_EQ(test, pp.color_conv1, e);
	pp_regs_640(test, V4L2_PIX_FMT_YUYV, &pp);
	KUNIT_EXPECT_EQ(test, pp.color_conv | pp.color_conv0 | pp.color_conv1 |
			pp.scaling0 | pp.mask_r | pp.mask_g | pp.mask_b, 0);
}

/* What the register builder refuses. */
static void pp_regs_refused(struct kunit *test)
{
	const struct hantro_g1_pp_cfg ok = {
		.in_fmt = 1, .in_width = 640, .in_height = 480,
		.orig_mb_width = 40, .fourcc = V4L2_PIX_FMT_XBGR32,
		.width = 320, .height = 240, .stride = 320, .buf_height = 240,
		.csc = &hantro_g1_pp_csc_bt601_full,
	};
	struct hantro_g1_pp_regs pp;
	struct hantro_g1_pp_cfg cfg;

	KUNIT_EXPECT_EQ(test, hantro_g1_pp_regs(&ok, &pp), 0);
	KUNIT_EXPECT_TRUE(test, hantro_g1_pp_writes(V4L2_PIX_FMT_RGB565));
	KUNIT_EXPECT_FALSE(test, hantro_g1_pp_writes(V4L2_PIX_FMT_NV16));

	cfg = ok;
	cfg.fourcc = V4L2_PIX_FMT_GREY;
	KUNIT_EXPECT_EQ(test, hantro_g1_pp_regs(&cfg, &pp), -EINVAL);
	/* RGB needs a conversion, YUV takes none. */
	cfg = ok;
	cfg.csc = NULL;
	KUNIT_EXPECT_EQ(test, hantro_g1_pp_regs(&cfg, &pp), -EINVAL);
	cfg.fourcc = V4L2_PIX_FMT_YUYV;
	KUNIT_EXPECT_EQ(test, hantro_g1_pp_regs(&cfg, &pp), 0);
}

/*
 * The other formats the post-processor writes: TRY_FMT sizes, the
 * stepwise sizes, the default compose rectangle, and its position at a
 * multiple of 8 bytes.
 */
static void jpeg_dec_pp_cap(struct kunit *test)
{
	static const u32 sizes[][2] = {
		{ 48, 48 }, { 4088, 8176 }, { 56, 50 }, { 1072, 1448 },
		{ 1448, 1072 }, { 1008, 752 }, { 1000, 744 }, { 4000, 48 },
	};
	struct v4l2_frmsizeenum fsize = { };
	const struct v4l2_frmsize_stepwise *sw = &fsize.stepwise;
	struct hantro_g1_jpeg_dec_out out;
	struct v4l2_rect r;
	unsigned int i;
	u32 w, h;

	/* A size of 0: the whole picture, as large as it is written. */
	make_out(&out, 4000, 3000, V4L2_PIX_FMT_YUYV, 0, 0);
	w = 0;
	h = 0;
	hantro_g1_jpeg_dec_cap_size(&out, &w, &h);
	KUNIT_EXPECT_EQ(test, w, 1920);
	KUNIT_EXPECT_EQ(test, h, 1440);
	make_out(&out, 640, 480, V4L2_PIX_FMT_RGB565, 0, 0);
	w = 0;
	h = 0;
	hantro_g1_jpeg_dec_cap_size(&out, &w, &h);
	KUNIT_EXPECT_EQ(test, w, 640);
	KUNIT_EXPECT_EQ(test, h, 480);
	/* Any other size: a buffer, also one that holds the picture. */
	w = 1008;
	h = 752;
	hantro_g1_jpeg_dec_cap_size(&out, &w, &h);
	KUNIT_EXPECT_EQ(test, w, 1008);
	KUNIT_EXPECT_EQ(test, h, 752);
	w = 1071;
	h = 1447;
	hantro_g1_jpeg_dec_cap_size(&out, &w, &h);
	KUNIT_EXPECT_EQ(test, w, 1072);
	KUNIT_EXPECT_EQ(test, h, 1448);

	/* Stepwise, every size taken as it is. */
	make_out(&out, 1000, 744, V4L2_PIX_FMT_YUYV, 0, 0);
	KUNIT_ASSERT_EQ(test, hantro_g1_jpeg_dec_cap_frmsize(&out, &fsize), 0);
	KUNIT_EXPECT_EQ(test, fsize.type, V4L2_FRMSIZE_TYPE_STEPWISE);
	KUNIT_EXPECT_EQ(test, sw->min_width, 48);
	KUNIT_EXPECT_EQ(test, sw->max_width, 4088);
	KUNIT_EXPECT_EQ(test, sw->step_width, 8);
	KUNIT_EXPECT_EQ(test, sw->min_height, 48);
	KUNIT_EXPECT_EQ(test, sw->max_height, 8176);
	KUNIT_EXPECT_EQ(test, sw->step_height, 2);
	for (i = 0; i < ARRAY_SIZE(sizes); i++) {
		w = sizes[i][0];
		h = sizes[i][1];
		KUNIT_EXPECT_EQ(test, (w - sw->min_width) % sw->step_width, 0);
		KUNIT_EXPECT_EQ(test, (h - sw->min_height) % sw->step_height, 0);
		hantro_g1_jpeg_dec_cap_size(&out, &w, &h);
		KUNIT_EXPECT_EQ_MSG(test, w, sizes[i][0], "%ux%u", sizes[i][0],
				    sizes[i][1]);
		KUNIT_EXPECT_EQ_MSG(test, h, sizes[i][1], "%ux%u", sizes[i][0],
				    sizes[i][1]);
	}
	fsize.index = 1;
	KUNIT_EXPECT_EQ(test, hantro_g1_jpeg_dec_cap_frmsize(&out, &fsize),
			-EINVAL);

	/*
	 * The default compose rectangle: the crop rectangle where the buffer
	 * holds it and the post-processor writes it that large, else the
	 * picture fitted, aspect ratio kept.
	 */
	make_out(&out, 1000, 744, V4L2_PIX_FMT_YUYV, 1000, 744);
	expect_rect(test, &out.compose, 0, 0, 1000, 744, "YUYV picture size");
	make_out(&out, 640, 480, V4L2_PIX_FMT_XRGB32, 1072, 1448);
	expect_rect(test, &out.compose, 0, 0, 640, 480, "XRGB32 larger");
	make_out(&out, 4000, 3000, V4L2_PIX_FMT_YUYV, 4000, 3000);
	expect_rect(test, &out.compose, 0, 0, 1920, 1440, "YUYV 4000 wide");
	make_out(&out, 1000, 3000, V4L2_PIX_FMT_RGB565, 1000, 3000);
	expect_rect(test, &out.compose, 0, 0, 680, 2046, "RGB565 3000 high");

	make_out(&out, 1000, 744, V4L2_PIX_FMT_YUYV, 1072, 736);
	r = (struct v4l2_rect){ 7, 0, 536, 400 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	KUNIT_EXPECT_EQ(test, r.left, 4);
	out.fourcc = V4L2_PIX_FMT_XRGB32;
	r = (struct v4l2_rect){ 7, 0, 536, 400 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	KUNIT_EXPECT_EQ(test, r.left, 6);
	out.fourcc = V4L2_PIX_FMT_NV12;
	r = (struct v4l2_rect){ 7, 0, 536, 400 };
	hantro_g1_jpeg_dec_compose_adjust(&out, &r);
	KUNIT_EXPECT_EQ(test, r.left, 0);
}

/*
 * The JPEG decoder sets the post-processor up for each job itself: no
 * generic post-processing for its YUYV, unlike the video decoders'.
 */
static void jpeg_dec_needs_postproc(struct kunit *test)
{
	static const struct {
		u32 src, dst;
		bool needs;
	} cases[] = {
		{ V4L2_PIX_FMT_JPEG, V4L2_PIX_FMT_YUYV, false },
		{ V4L2_PIX_FMT_JPEG, V4L2_PIX_FMT_NV12, false },
		{ V4L2_PIX_FMT_H264_SLICE, V4L2_PIX_FMT_YUYV, true },
		{ V4L2_PIX_FMT_VP8_FRAME, V4L2_PIX_FMT_YUYV, true },
		{ V4L2_PIX_FMT_VP8_FRAME, V4L2_PIX_FMT_NV12, false },
	};
	const struct hantro_variant *rk3026 = &rk3026_vpu_variant;
	const struct hantro_fmt *dst;
	struct hantro_ctx *ctx;
	struct hantro_dev *dev;
	unsigned int c, count;

	if (!IS_ENABLED(CONFIG_VIDEO_HANTRO_ROCKCHIP))
		kunit_skip(test, "no Rockchip variants");

	dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	dev->variant = rk3026;
	ctx->dev = dev;

	for (c = 0; c < ARRAY_SIZE(cases); c++) {
		ctx->vpu_src_fmt = find_fmt(rk3026, cases[c].src, &count);
		KUNIT_ASSERT_EQ(test, count, 1);
		if (cases[c].dst == V4L2_PIX_FMT_YUYV) {
			dst = &rk3026->postproc_fmts[0];
		} else {
			dst = find_fmt(rk3026, cases[c].dst, &count);
			KUNIT_ASSERT_EQ(test, count, 1);
		}
		KUNIT_EXPECT_EQ_MSG(test, hantro_needs_postproc(ctx, dst),
				    cases[c].needs, "case %u", c);
	}
}

/*
 * TRY_FMT on a JPEG decoder's CAPTURE queue: the buffer sizes and line
 * lengths, and the JFIF colorimetry for every raw format; a video
 * decoder's CAPTURE keeps the colorimetry it is given.
 */
static void jpeg_dec_try_fmt(struct kunit *test)
{
	static const struct {
		u32 fourcc, w, h;
		u32 want_w, want_h, bpl, sizeimage;
	} cases[] = {
		{ V4L2_PIX_FMT_NV12, 0, 0, 1008, 752, 1008, 1008 * 752 * 3 / 2 },
		{ V4L2_PIX_FMT_NV12, 1000, 744, 1008, 752, 1008, 1008 * 752 * 3 / 2 },
		{ V4L2_PIX_FMT_NV12, 536, 400, 536, 400, 536, 536 * 400 * 3 / 2 },
		{ V4L2_PIX_FMT_NV16, 0, 0, 1008, 752, 1008, 1008 * 752 * 2 },
		{ V4L2_PIX_FMT_GREY, 640, 480, 1008, 752, 1008, 1008 * 752 },
		{ V4L2_PIX_FMT_YUYV, 1072, 1448, 1072, 1448, 2144, 2144 * 1448 },
		{ V4L2_PIX_FMT_RGB565, 536, 400, 536, 400, 1072, 1072 * 400 },
		{ V4L2_PIX_FMT_XBGR32, 0, 0, 1000, 744, 4000, 4000 * 744 },
		{ V4L2_PIX_FMT_XRGB32, 536, 401, 536, 402, 2144, 2144 * 402 },
	};
	const struct hantro_variant *rk3026 = &rk3026_vpu_variant;
	struct v4l2_pix_format_mplane pix;
	struct hantro_ctx *ctx;
	struct hantro_dev *dev;
	unsigned int i, count;

	if (!IS_ENABLED(CONFIG_VIDEO_HANTRO_ROCKCHIP))
		kunit_skip(test, "no Rockchip variants");

	dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	dev->variant = rk3026;
	ctx->dev = dev;
	ctx->vpu_src_fmt = find_fmt(rk3026, V4L2_PIX_FMT_JPEG, &count);
	KUNIT_ASSERT_EQ(test, count, 1);
	ctx->src_fmt.pixelformat = V4L2_PIX_FMT_JPEG;
	ctx->src_fmt.width = 1000;
	ctx->src_fmt.height = 744;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		memset(&pix, 0, sizeof(pix));
		pix.pixelformat = cases[i].fourcc;
		pix.width = cases[i].w;
		pix.height = cases[i].h;
		pix.colorspace = V4L2_COLORSPACE_JPEG;
		KUNIT_ASSERT_EQ(test, hantro_try_fmt(ctx, &pix,
						     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE),
				0);
		KUNIT_EXPECT_EQ_MSG(test, pix.pixelformat, cases[i].fourcc,
				    "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, pix.width, cases[i].want_w, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, pix.height, cases[i].want_h,
				    "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, pix.plane_fmt[0].bytesperline,
				    cases[i].bpl, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, pix.plane_fmt[0].sizeimage,
				    cases[i].sizeimage, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, pix.colorspace, V4L2_COLORSPACE_SRGB,
				    "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, pix.xfer_func, V4L2_XFER_FUNC_SRGB,
				    "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, pix.ycbcr_enc, V4L2_YCBCR_ENC_601,
				    "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, pix.quantization,
				    V4L2_QUANTIZATION_FULL_RANGE, "case %u", i);
	}

	/* A video decoder's CAPTURE: the colorimetry as given. */
	ctx->vpu_src_fmt = find_fmt(rk3026, V4L2_PIX_FMT_VP8_FRAME, &count);
	KUNIT_ASSERT_EQ(test, count, 1);
	ctx->src_fmt.pixelformat = V4L2_PIX_FMT_VP8_FRAME;
	ctx->src_fmt.width = 640;
	ctx->src_fmt.height = 480;
	memset(&pix, 0, sizeof(pix));
	pix.pixelformat = V4L2_PIX_FMT_NV12;
	pix.colorspace = V4L2_COLORSPACE_REC709;
	KUNIT_ASSERT_EQ(test, hantro_try_fmt(ctx, &pix,
					     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE), 0);
	KUNIT_EXPECT_EQ(test, pix.width, 640);
	KUNIT_EXPECT_EQ(test, pix.colorspace, V4L2_COLORSPACE_REC709);
	KUNIT_EXPECT_EQ(test, pix.quantization, V4L2_QUANTIZATION_DEFAULT);
}

/* One field of a register file, as the video post-processing wrote them. */
struct pp_field {
	unsigned int swreg;
	unsigned int shift;
	u32 mask;
};

static void pp_field_write(u32 *swreg, const struct pp_field *f, u32 val)
{
	swreg[f->swreg] &= ~(f->mask << f->shift);
	swreg[f->swreg] |= (val & f->mask) << f->shift;
}

/*
 * The G1 video post-processing set-up as it was programmed field by field
 * (read-modify-write) before the register builder, applied to registers
 * that hold 0: the picture at its own size, NV12 to YUYV.
 */
static void old_video_pp(u32 *swreg, u32 width, u32 height, u32 dst)
{
	static const struct pp_field pipeline_en = { 60, 1, 0x1 };
	static const struct pp_field max_burst = { 61, 0, 0x1f };
	static const struct pp_field clk_gate = { 61, 1, 0x1 };
	static const struct pp_field out_swap32 = { 61, 5, 0x1 };
	static const struct pp_field out_endian = { 61, 6, 0x1 };
	static const struct pp_field out_luma_base = { 66, 0, 0xffffffff };
	static const struct pp_field input_width = { 72, 0, 0x1ff };
	static const struct pp_field input_height = { 72, 9, 0x1ff };
	static const struct pp_field output_width = { 85, 4, 0x7ff };
	static const struct pp_field output_height = { 85, 15, 0x7ff };
	static const struct pp_field input_fmt = { 85, 29, 0x7 };
	static const struct pp_field output_fmt = { 85, 26, 0x7 };
	static const struct pp_field orig_width = { 88, 23, 0x1ff };
	static const struct pp_field display_width = { 92, 0, 0xfff };
	static const struct pp_field input_width_ext = { 92, 26, 0x7 };
	static const struct pp_field input_height_ext = { 92, 29, 0x7 };

	pp_field_write(swreg, &pipeline_en, 1);
	pp_field_write(swreg, &clk_gate, 1);
	pp_field_write(swreg, &out_endian, 1);
	pp_field_write(swreg, &out_swap32, 1);
	pp_field_write(swreg, &max_burst, 16);
	pp_field_write(swreg, &out_luma_base, dst);
	pp_field_write(swreg, &input_width, MB_WIDTH(width));
	pp_field_write(swreg, &input_height, MB_HEIGHT(height));
	pp_field_write(swreg, &input_fmt, 1);	/* NV12 */
	pp_field_write(swreg, &output_fmt, 3);	/* YUYV */
	pp_field_write(swreg, &output_width, width);
	pp_field_write(swreg, &output_height, height);
	pp_field_write(swreg, &orig_width, MB_WIDTH(width));
	pp_field_write(swreg, &display_width, width);
	pp_field_write(swreg, &input_width_ext, MB_WIDTH(width) >> 9);
	pp_field_write(swreg, &input_height_ext, MB_HEIGHT(height) >> 8);
}

/*
 * The video post-processing set-up from the register builder gives the
 * registers the field-by-field set-up gave, for the sizes the video
 * formats take (up to 4K), and 0 in the registers it did not write. The
 * one difference is in the pipeline word: the builder's write also masks
 * the post-processor's interrupt.
 */
static void pp_video_regs(struct kunit *test)
{
	static const struct {
		u32 width, height;
	} sizes[] = {
		{ 48, 48 }, { 176, 144 }, { 640, 480 }, { 1280, 720 },
		{ 1920, 1088 }, { 3840, 2160 }, { 4096, 2304 },
	};
	const u32 dst = 0x12345600;
	struct hantro_g1_pp_regs regs;
	unsigned int i, r;
	u32 swreg[101];

	for (i = 0; i < ARRAY_SIZE(sizes); i++) {
		u32 w = sizes[i].width, h = sizes[i].height;

		memset(swreg, 0, sizeof(swreg));
		old_video_pp(swreg, w, h, dst);
		KUNIT_ASSERT_EQ(test, hantro_g1_pp_video_regs(V4L2_PIX_FMT_YUYV,
							      w, h, &regs), 0);

		KUNIT_EXPECT_EQ_MSG(test, swreg[60], G1_REG_PP_PIPELINE_EN,
				    "%ux%u", w, h);
		/* CLK_GATE_E as written was cleared by the burst length. */
		KUNIT_EXPECT_EQ_MSG(test, regs.dev_config, swreg[61],
				    "%ux%u", w, h);
		KUNIT_EXPECT_EQ_MSG(test, swreg[66], dst, "%ux%u", w, h);
		KUNIT_EXPECT_EQ_MSG(test, regs.input_size, swreg[72],
				    "%ux%u", w, h);
		KUNIT_EXPECT_EQ_MSG(test, regs.control, swreg[85], "%ux%u", w, h);
		KUNIT_EXPECT_EQ_MSG(test, regs.orig_width, swreg[88],
				    "%ux%u", w, h);
		KUNIT_EXPECT_EQ_MSG(test, regs.display_width, swreg[92],
				    "%ux%u", w, h);
		/* No scaling, at the start of the buffer, one plane. */
		KUNIT_EXPECT_EQ_MSG(test, regs.scaling1, 0, "%ux%u", w, h);
		KUNIT_EXPECT_EQ_MSG(test, regs.scaling_inv, 0, "%ux%u", w, h);
		KUNIT_EXPECT_EQ_MSG(test, regs.luma_offset, 0, "%ux%u", w, h);
		KUNIT_EXPECT_EQ_MSG(test, regs.chroma_offset, 0, "%ux%u", w, h);
		/* Chroma base, colour conversion, scaling, masks: 0. */
		for (r = 67; r <= 84; r++)
			if (r != 72)
				KUNIT_EXPECT_EQ_MSG(test, swreg[r], 0,
						    "%ux%u swreg%u", w, h, r);
	}

	/* The only format the video post-processing writes. */
	KUNIT_EXPECT_EQ(test, hantro_g1_pp_video_regs(V4L2_PIX_FMT_NV16,
						      640, 480, &regs), -EINVAL);
}

/*
 * The kernel mapping of a decoder's buffers: both queues of the JPEG decoder
 * map theirs, so that an importer can vmap an exported CAPTURE buffer, and
 * the video decoders map neither. The attribute follows the codec selected
 * when a queue gets its first buffers: a queue with buffers keeps the
 * attribute they were allocated with, and the encoder's queues are left as
 * they are.
 */
static void jpeg_dec_kernel_mapping(struct kunit *test)
{
	static const struct {
		u32 fourcc;
		bool map;
	} cases[] = {
		{ V4L2_PIX_FMT_JPEG, true },
		{ V4L2_PIX_FMT_VP8_FRAME, false },
		{ V4L2_PIX_FMT_H264_SLICE, false },
		{ V4L2_PIX_FMT_MPEG2_SLICE, false },
		{ V4L2_PIX_FMT_JPEG, true },
	};
	const unsigned long base = DMA_ATTR_ALLOC_SINGLE_PAGES |
				   DMA_ATTR_NO_WARN;
	const unsigned long nomap = base | DMA_ATTR_NO_KERNEL_MAPPING;
	const struct hantro_variant *rk3026 = &rk3026_vpu_variant;
	struct vb2_queue *src_vq, *dst_vq;
	unsigned long *bufs_bitmap;
	struct hantro_ctx *ctx;
	struct hantro_dev *dev;
	unsigned int c, count;
	unsigned long want;

	if (!IS_ENABLED(CONFIG_VIDEO_HANTRO_ROCKCHIP))
		kunit_skip(test, "no Rockchip variants");

	dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	bufs_bitmap = kunit_kzalloc(test, BITS_TO_LONGS(8) * sizeof(long),
				    GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, bufs_bitmap);
	ctx->fh.m2m_ctx = kunit_kzalloc(test, sizeof(*ctx->fh.m2m_ctx),
					GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->fh.m2m_ctx);
	dev->variant = rk3026;
	ctx->dev = dev;
	src_vq = v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx);
	dst_vq = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);

	/* As the decoder's queues are set up, both empty. */
	src_vq->dma_attrs = nomap;
	dst_vq->dma_attrs = nomap;
	for (c = 0; c < ARRAY_SIZE(cases); c++) {
		ctx->vpu_src_fmt = find_fmt(rk3026, cases[c].fourcc, &count);
		KUNIT_ASSERT_EQ(test, count, 1);
		hantro_update_dec_kernel_mapping(ctx, src_vq);
		hantro_update_dec_kernel_mapping(ctx, dst_vq);
		want = nomap;
		if (cases[c].map)
			want = base;
		KUNIT_EXPECT_EQ_MSG(test, src_vq->dma_attrs, want, "case %u", c);
		KUNIT_EXPECT_EQ_MSG(test, dst_vq->dma_attrs, want, "case %u", c);
	}

	/*
	 * The JPEG decoder's CAPTURE buffers stay while the codec changes to
	 * VP8 and back: a queue with buffers keeps its attribute.
	 */
	dst_vq->max_num_buffers = 8;
	dst_vq->bufs_bitmap = bufs_bitmap;
	__set_bit(0, bufs_bitmap);
	ctx->vpu_src_fmt = find_fmt(rk3026, V4L2_PIX_FMT_VP8_FRAME, &count);
	hantro_update_dec_kernel_mapping(ctx, src_vq);
	hantro_update_dec_kernel_mapping(ctx, dst_vq);
	KUNIT_EXPECT_EQ(test, src_vq->dma_attrs, nomap);
	KUNIT_EXPECT_EQ(test, dst_vq->dma_attrs, base);

	/* The VP8 decoder allocates its CAPTURE buffers anew. */
	__clear_bit(0, bufs_bitmap);
	hantro_update_dec_kernel_mapping(ctx, dst_vq);
	KUNIT_EXPECT_EQ(test, dst_vq->dma_attrs, nomap);

	/*
	 * The codec changes to JPEG while CAPTURE has the video buffers; the
	 * client frees them and allocates new ones without another S_FMT.
	 */
	__set_bit(0, bufs_bitmap);
	ctx->vpu_src_fmt = find_fmt(rk3026, V4L2_PIX_FMT_JPEG, &count);
	hantro_update_dec_kernel_mapping(ctx, dst_vq);
	KUNIT_EXPECT_EQ(test, dst_vq->dma_attrs, nomap);
	__clear_bit(0, bufs_bitmap);
	hantro_update_dec_kernel_mapping(ctx, dst_vq);
	KUNIT_EXPECT_EQ(test, dst_vq->dma_attrs, base);

	/* The encoder: a mapped CAPTURE queue, an unmapped OUTPUT one. */
	src_vq->dma_attrs = nomap;
	dst_vq->dma_attrs = base;
	ctx->is_encoder = true;
	hantro_update_dec_kernel_mapping(ctx, src_vq);
	hantro_update_dec_kernel_mapping(ctx, dst_vq);
	KUNIT_EXPECT_EQ(test, src_vq->dma_attrs, nomap);
	KUNIT_EXPECT_EQ(test, dst_vq->dma_attrs, base);
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
	KUNIT_CASE(jpeg_dec_raw_fmt_allowed),
	KUNIT_CASE(pp_video_regs),
	KUNIT_CASE(jpeg_dec_cap_size),
	KUNIT_CASE(jpeg_dec_cap_frmsize),
	KUNIT_CASE(jpeg_dec_compose),
	KUNIT_CASE(jpeg_dec_job_out),
	KUNIT_CASE(jpeg_dec_selection),
	KUNIT_CASE(jpeg_dec_pp_formats),
	KUNIT_CASE(jpeg_dec_pp_byte_order),
	KUNIT_CASE(jpeg_dec_pp_coefficients),
	KUNIT_CASE(pp_regs_refused),
	KUNIT_CASE(jpeg_dec_pp_cap),
	KUNIT_CASE(jpeg_dec_needs_postproc),
	KUNIT_CASE(jpeg_dec_try_fmt),
	KUNIT_CASE(jpeg_dec_kernel_mapping),
	{ }
};

static struct kunit_suite hantro_jpeg_dec_suite = {
	.name = "hantro-jpeg-dec",
	.test_cases = hantro_jpeg_dec_cases,
};

kunit_test_suite(hantro_jpeg_dec_suite);
