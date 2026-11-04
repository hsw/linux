/* SPDX-License-Identifier: GPL-2.0+ */

#include <media/v4l2-jpeg.h>

#define JPEG_HEADER_SIZE	624
#define JPEG_QUANT_SIZE		64

struct hantro_jpeg_ctx {
	int width;
	int height;
	int quality;
	unsigned char *buffer;
	unsigned char hw_luma_qtable[JPEG_QUANT_SIZE];
	unsigned char hw_chroma_qtable[JPEG_QUANT_SIZE];
};

void hantro_jpeg_header_assemble(struct hantro_jpeg_ctx *ctx);

/* Huffman value lists in the decoder's table buffer */
#define HANTRO_JPEG_DEC_AC_SIZE	162
#define HANTRO_JPEG_DEC_DC_SIZE	12

void hantro_jpeg_get_default_huffman_tables(struct v4l2_jpeg_reference *huffman_tables);
int hantro_jpeg_prepare_vlc_hw_table(struct v4l2_jpeg_header *header,
				     u8 *vlc_hw_table);

#if IS_ENABLED(CONFIG_KUNIT)
u32 hantro_g1_jpeg_dec_fourcc(int subsampling);
int hantro_g1_jpeg_dec_check(const struct v4l2_jpeg_header *hdr,
			     size_t payload, u32 fourcc, u32 width, u32 height);
#endif
