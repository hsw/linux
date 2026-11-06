/* SPDX-License-Identifier: GPL-2.0+ */

#include <linux/videodev2.h>
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

/**
 * struct hantro_g1_jpeg_dec_out - what a JPEG decoding context writes
 * @src_width:		OUTPUT width: the picture as the client set it up,
 *			the crop rectangle
 * @src_height:		OUTPUT height
 * @fourcc:		CAPTURE format
 * @width:		CAPTURE width: the buffer
 * @height:		CAPTURE height
 * @compose:		where the picture goes in the buffer
 * @pp_max_width:	widest post-processor output, 0 without one
 */
struct hantro_g1_jpeg_dec_out {
	u32 src_width;
	u32 src_height;
	u32 fourcc;
	u32 width;
	u32 height;
	struct v4l2_rect compose;
	u32 pp_max_width;
};

struct hantro_g1_pp_regs;

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
			     size_t payload, u32 src_width, u32 src_height);
void hantro_g1_jpeg_dec_cap_size(const struct hantro_g1_jpeg_dec_out *out,
				 u32 *width, u32 *height);
int hantro_g1_jpeg_dec_cap_frmsize(const struct hantro_g1_jpeg_dec_out *out,
				   struct v4l2_frmsizeenum *fsize);
void hantro_g1_jpeg_dec_compose_default(const struct hantro_g1_jpeg_dec_out *out,
					struct v4l2_rect *r);
void hantro_g1_jpeg_dec_compose_adjust(const struct hantro_g1_jpeg_dec_out *out,
				       struct v4l2_rect *r);
void hantro_g1_jpeg_dec_compose_padded(const struct hantro_g1_jpeg_dec_out *out,
				       struct v4l2_rect *r);
int hantro_g1_jpeg_dec_job_out(const struct hantro_g1_jpeg_dec_out *out,
			       int subsampling, struct hantro_g1_pp_regs *pp,
			       bool *use_pp);
#endif
