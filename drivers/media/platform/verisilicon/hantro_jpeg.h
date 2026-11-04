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
#define HANTRO_JPEG_DEC_AC_SIZE		162
#define HANTRO_JPEG_DEC_DC_SIZE		12
/* Three quantization tables, two AC and two DC value lists, 4 spare bytes */
#define HANTRO_JPEG_DEC_TABLE_SIZE	(3 * JPEG_QUANT_SIZE + \
					 2 * HANTRO_JPEG_DEC_AC_SIZE + \
					 2 * HANTRO_JPEG_DEC_DC_SIZE + 4)

/* Huffman code count registers, 0x040 to 0x068 */
#define HANTRO_G1_JPEG_DEC_VLC_REGS	11

/**
 * struct hantro_g1_jpeg_dec_regs - G1 registers of one JPEG decoding job
 * @ctrl0:	G1_REG_DEC_CTRL0
 * @ctrl1:	G1_REG_DEC_CTRL1
 * @ctrl2:	G1_REG_DEC_CTRL2
 * @ctrl3:	G1_REG_DEC_CTRL3
 * @ctrl4:	G1_REG_DEC_CTRL4
 * @ctrl5:	G1_REG_DEC_CTRL5
 * @jpeg_ctrl:	G1_REG_JPEG_CTRL
 * @vlc:	the Huffman code counts, G1_REG_DEC_JPEG_VLC_AC_CODE_LENGTHS_A
 *		onwards
 * @stream:	bus address of the stream, 8-byte aligned, for G1_REG_ADDR_STR
 */
struct hantro_g1_jpeg_dec_regs {
	u32 ctrl0;
	u32 ctrl1;
	u32 ctrl2;
	u32 ctrl3;
	u32 ctrl4;
	u32 ctrl5;
	u32 jpeg_ctrl;
	u32 vlc[HANTRO_G1_JPEG_DEC_VLC_REGS];
	dma_addr_t stream;
};

void hantro_jpeg_get_default_huffman_tables(struct v4l2_jpeg_reference *huffman_tables);
int hantro_jpeg_prepare_vlc_hw_table(const struct v4l2_jpeg_header *header,
				     u8 *vlc_hw_table);

#if IS_ENABLED(CONFIG_KUNIT)
u32 hantro_g1_jpeg_dec_fourcc(int subsampling);
int hantro_g1_jpeg_dec_mode(int subsampling);
void hantro_g1_jpeg_dec_regs(const struct v4l2_jpeg_header *hdr,
			     dma_addr_t src, size_t payload,
			     struct hantro_g1_jpeg_dec_regs *regs);
int hantro_g1_jpeg_dec_check(const struct v4l2_jpeg_header *hdr,
			     size_t payload, u32 fourcc, u32 width, u32 height);
#endif
