// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip "rkf" waveform container walker for the RK3026 EBC.
 *
 * Layout, all offsets from the container base:
 *
 *   +0x000  u32 total size; the CRC32 word sits at +size
 *   +0x004  u32 payload start (0xad0 = 0x6d0 + 0x400)
 *   +0x008  magic "rkf:v" + version triplet + build date
 *   +0x01c  vendor tag, "Einks" or "OED"
 *   +0x02c  panel name, ASCII
 *   +0x090  five temperature-boundary tables, 0x40 bytes each
 *   +0x1d0  five u32 offset tables, 0x40 slots each
 *   +0x6d0  256 x u32 Huffman weights, XOR-obfuscated in a pristine file
 *   +0xad0  segmented Huffman payload, one segment per (mode, temperature)
 *
 * A segment is a frame-count byte, three pad bytes, then the bitstream. One
 * Huffman tree, built from the weight table, covers the whole container.
 *
 * Copyright (C) 2026
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include <kunit/visibility.h>

#include "rk3026_ebc.h"

#define RKF_MAX_SIZE		0x40000
#define RKF_WEIGHTS_OFF		0x6d0
#define RKF_PAYLOAD_OFF		0xad0
#define RKF_NSYM		0x100
#define RKF_NNODE		(2 * RKF_NSYM - 1)
#define RKF_ROOT		(RKF_NNODE - 1)
/* A segment's header: the frame count, three pad bytes. */
#define RKF_SEG_HDR		4

/* Largest LUT the SRAM window can be fed: 255 frames of 0x40 bytes. */
#define RKF_LUT_MAX		(EBC_WAVEFORM_MAX_FRAMES * EBC_LUT_FRAME_BYTES)
/* Largest raw segment we decode: mode 7 emits frames << 3. */
#define RKF_RAW_MAX		(EBC_WAVEFORM_MAX_FRAMES * 8)

/* Container LUT modes, as selected by epd_lut_data_get(). */
#define RKF_LUT_RESET		1
#define RKF_LUT_GRAY16		2
#define RKF_LUT_MODE6		6
#define RKF_LUT_GRAY2		7
#define RKF_LUT_AUTO		8
#define RKF_LUT_GRAYA2		9

/* Largest WBF we accept: the panel flash a board can put one on is 512 KiB. */
#define WBF_MAX_SIZE		0x80000
#define WBF_HEADER_MIN		0x30
/* Mode version 0x19: INIT DU GC16 GL16 GLR16 GLD16 A2 DU4, the Carta-era set. */
#define WBF_MODE_VERSION	0x19
#define WBF_MAX_MODES		8
#define WBF_MAX_RANGES		0x20
/* One code per (frame, target, source); 255 frames of 32 x 32 is the ceiling. */
#define WBF_MAX_CODES		(EBC_WAVEFORM_MAX_FRAMES * 32 * 32)

/* Header fields, all little-endian. */
#define WBF_FILESIZE_OFF	0x04
#define WBF_MODE_VER_OFF	0x10
#define WBF_XWIA_OFF		0x1c
#define WBF_WMTA_OFF		0x20
#define WBF_LUTS_OFF		0x24
#define WBF_MODE_COUNT_OFF	0x25
#define WBF_RANGE_COUNT_OFF	0x26
#define WBF_EB_OFF		0x28
#define WBF_SB_OFF		0x29
#define WBF_TEMPS_OFF		0x30

/* WBF mode index -> container LUT mode, as tools/mainline-rc2/wbf2rkf.py maps it. */
#define WBF_MODE_INIT		0
#define WBF_MODE_DU		1
#define WBF_MODE_GC16		2
#define WBF_MODE_GL16		3
#define WBF_MODE_A2		6

/* The part of a parsed waveform that only the WBF source has. */
struct ebc_wbf {
	const u8	*d;
	size_t		size;		/* the extent the header states */
	u32		wmta;
	u8		eb;
	u8		sb;
	u8		levels;		/* 16 or 32, from the bits-per-pixel field */
	u8		step;		/* levels / 16: the 16-level view strides */
	u8		modes;
	u8		ranges;
	u8		temps[WBF_MAX_RANGES + 1];
};

struct ebc_waveform {
	struct device	*dev;
	const u8	*base;
	size_t		size;

	s16		left[RKF_NNODE];
	s16		right[RKF_NNODE];

	/* Cache of the last (mode, temperature) pair, as the vendor keeps. */
	u32		cached_mode;
	int		cached_temp;
	bool		cached_valid;
	u32		frames;

	u8		*lut;	/* frames * EBC_LUT_FRAME_BYTES */
	u8		*raw;	/* Huffman output before the post-pass */

	bool		is_wbf;
	struct ebc_wbf	wbf;
};

/*
 * The container CRC32 is MSB-first with polynomial 0x04c10db7 - the vendor's
 * typo of 0x04c11db7, bit 12 clear. Init 0, no reflection, no final xor, and
 * the extent is [base, base + size) so it covers the size field itself.
 * Computed bitwise: a table would cost 1 KiB for one call per probe.
 */
#define RKF_CRC_POLY		0x04c10db7u

static u32 rkf_crc32(const u8 *p, size_t n, size_t xor_from, size_t xor_to,
		     const u32 *key)
{
	u32 crc = 0;
	size_t i;

	for (i = 0; i < n; i++) {
		u8 b = p[i];
		int k;

		/*
		 * A container read back out of the driver's staging buffer has
		 * already had its weight table deobfuscated in place, so the
		 * stored checksum only closes if the key is reapplied here.
		 */
		if (key && i >= xor_from && i < xor_to) {
			size_t w = (i - xor_from) / 4;

			b ^= (key[w & 1] >> (8 * ((i - xor_from) & 3))) & 0xff;
		}

		crc ^= (u32)b << 24;
		for (k = 0; k < 8; k++)
			crc = (crc << 1) ^ (crc & 0x80000000u ? RKF_CRC_POLY : 0);
	}
	return crc;
}

static u32 rkf_u32(const u8 *p)
{
	return get_unaligned_le32(p);
}

/*
 * Canonical Huffman build: repeatedly merge the two lightest parentless nodes.
 * Only nodes that already exist take part, and ties keep the lower index.
 */
static void rkf_build_tree(struct ebc_waveform *wf, const u32 *weights,
			   u32 *w, s16 *parent)
{
	unsigned int count = RKF_NSYM;
	unsigned int i, k;

	for (i = 0; i < RKF_NNODE; i++) {
		w[i] = i < RKF_NSYM ? weights[i] : 0;
		parent[i] = -1;
		wf->left[i] = -1;
		wf->right[i] = -1;
	}

	for (k = 0; k < RKF_NSYM - 1; k++) {
		u32 m1 = U32_MAX, m2 = U32_MAX;
		unsigned int i1 = 0, i2 = 0;

		for (i = 0; i < count; i++) {
			if (parent[i] != -1)
				continue;
			if (w[i] < m1) {
				m2 = m1;
				i2 = i1;
				m1 = w[i];
				i1 = i;
			} else if (w[i] < m2) {
				m2 = w[i];
				i2 = i;
			}
		}

		parent[i1] = count;
		parent[i2] = count;
		w[count] = w[i1] + w[i2];
		wf->left[count] = i1;
		wf->right[count] = i2;
		count++;
	}
}

/**
 * rk3026_ebc_rkf_decode - decode a Huffman bitstream of known symbol count
 * @left: left child per node, -1 for a leaf
 * @right: right child per node
 * @root: index of the root node
 * @dst: output, @dst_len bytes
 * @dst_len: room in @dst
 * @n: symbols to decode
 * @src: bitstream
 * @len: bytes of @src that may be read
 *
 * MSB-first walk from the root; a symbol is the index of the leaf reached.
 * A segment carries no length of its own, so the walk is bounded by @len,
 * the bytes left in the container after the segment start.
 *
 * Returns 0, or -EINVAL if @n does not fit @dst_len or @src runs out before
 * @n symbols are decoded.
 */
VISIBLE_IF_KUNIT int rk3026_ebc_rkf_decode(const s16 *left, const s16 *right,
					   int root, u8 *dst, size_t dst_len,
					   size_t n, const u8 *src, size_t len)
{
	const u8 *end = src + len;
	unsigned int bit = 0;
	int node = root;

	if (n > dst_len)
		return -EINVAL;

	while (n) {
		if (src == end)
			return -EINVAL;
		node = (*src & (0x80 >> bit)) ? right[node] : left[node];
		if (++bit == 8) {
			bit = 0;
			src++;
		}
		if (left[node] == -1) {
			*dst++ = node;
			n--;
			node = root;
		}
	}
	return 0;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_rkf_decode);

/* Decode the bitstream of the segment at @seg, which follows its header. */
static int rkf_decode(struct ebc_waveform *wf, u8 *dst, size_t dst_len,
		      size_t n, const u8 *seg)
{
	const u8 *src = seg + RKF_SEG_HDR;
	size_t len = wf->base + wf->size - src;
	int ret;

	ret = rk3026_ebc_rkf_decode(wf->left, wf->right, RKF_ROOT, dst,
				    dst_len, n, src, len);
	if (ret)
		dev_err_ratelimited(wf->dev,
				    "waveform segment at %#tx truncated or oversized\n",
				    seg - wf->base);
	return ret;
}

/*
 * Temperature bucket: clamp to 0..48 (49 for anything hotter), then walk the
 * boundary table. A leading 0x00 is a real boundary, not a terminator - the
 * scan stops when the *next* byte is zero.
 */
static int rkf_temp_bucket(const u8 *tbl, int temp)
{
	int i;

	if (temp < 0)
		temp = 0;
	else if (temp > 0x30)
		temp = 0x31;

	for (i = 0; i < 0x32; i++) {
		if (temp < tbl[i])
			return i ? i - 1 : 0;
		if (!tbl[i + 1])
			return i;
	}
	return 0x31;
}

/* Per-LUT-mode table bases, as read out of epd_lut_data_get(). */
struct rkf_mode_desc {
	u16 temp_off;
	u16 offs_off;
};

static const struct rkf_mode_desc rkf_modes[] = {
	[RKF_LUT_RESET]  = { 0x090, 0x1d0 },
	[RKF_LUT_GRAY16] = { 0x0d0, 0x2d0 },
	[RKF_LUT_MODE6]  = { 0x150, 0x4d0 },
	[RKF_LUT_GRAY2]  = { 0x190, 0x5d0 },
	[RKF_LUT_GRAYA2] = { 0x110, 0x3d0 },
};

/*
 * Locate a segment. Returns its base, ERR_PTR(-ENOENT) when the mode is
 * absent, or ERR_PTR(-EINVAL) when its header does not fit the container.
 */
static const u8 *rkf_segment(struct ebc_waveform *wf, u32 mode, int temp,
			     u32 *frames)
{
	const struct rkf_mode_desc *d;
	const u8 *seg;
	u32 off;
	int idx;

	if (mode >= ARRAY_SIZE(rkf_modes) || !rkf_modes[mode].temp_off)
		return ERR_PTR(-ENOENT);
	d = &rkf_modes[mode];

	idx = rkf_temp_bucket(wf->base + d->temp_off, temp);
	off = rkf_u32(wf->base + d->offs_off + idx * 4);

	/* A zero slot means the mode is not present in this container. */
	if (off < RKF_PAYLOAD_OFF)
		return ERR_PTR(-ENOENT);
	if (off > wf->size - RKF_SEG_HDR) {
		dev_err_ratelimited(wf->dev,
				    "waveform segment at %#x past the container end %#zx\n",
				    off, wf->size);
		return ERR_PTR(-EINVAL);
	}

	seg = wf->base + off;
	*frames = seg[0];
	return *frames ? seg : ERR_PTR(-ENOENT);
}

/*
 * rst_format_change(): every 2-bit field of the decoded stream is one reset
 * frame, expanded to a full 0x40-byte frame of that same value.
 */
static void rkf_expand_reset(u8 *dst, const u8 *src, u32 frames)
{
	u32 f;

	for (f = 0; f < frames; f++) {
		u32 phase = (src[f / 4] >> (2 * (f % 4))) & 3;
		u32 word = phase * 0x55555555u;
		u32 i;

		for (i = 0; i < EBC_LUT_FRAME_BYTES / 4; i++)
			put_unaligned_le32(word, dst + f * EBC_LUT_FRAME_BYTES + i * 4);
	}
}

/*
 * gray2_data_change(): two source words per frame supply the low and the high
 * 2-bit field of each of the 16 LUT words.
 */
static void rkf_expand_gray2(u8 *dst, const u8 *src, u32 frames)
{
	u32 f;

	for (f = 0; f < frames; f++) {
		u32 w0 = rkf_u32(src + f * 8);
		u32 w1 = rkf_u32(src + f * 8 + 4);
		u32 i;

		for (i = 0; i < 16; i++) {
			u32 v = ((w0 >> (2 * i)) & 3) | ((w1 >> (2 * i)) << 30);

			put_unaligned_le32(v, dst + f * EBC_LUT_FRAME_BYTES + i * 4);
		}
	}
}

/*
 * auto_data_change(): overlay a gray2 stream onto an already decoded gray16
 * LUT, replacing only the outermost grey levels.
 */
static void rkf_overlay_gray2(u8 *dst, const u8 *src, u32 frames)
{
	u32 f;

	for (f = 0; f < frames; f++) {
		u32 w0 = rkf_u32(src + f * 8);
		u32 w1 = rkf_u32(src + f * 8 + 4);
		u32 i;

		for (i = 0; i < 16; i++) {
			u8 *p = dst + f * EBC_LUT_FRAME_BYTES + i * 4;
			u32 v = rkf_u32(p) & 0x3ffffffcu;

			v |= ((w0 >> (2 * i)) & 3) | ((w1 >> (2 * i)) << 30);
			put_unaligned_le32(v, p);
		}
	}
}

static int rkf_load_direct(struct ebc_waveform *wf, u32 mode, int temp,
			   u32 *frames)
{
	const u8 *seg = rkf_segment(wf, mode, temp, frames);

	if (IS_ERR(seg))
		return PTR_ERR(seg);

	return rkf_decode(wf, wf->lut, RKF_LUT_MAX,
			  *frames * EBC_LUT_FRAME_BYTES, seg);
}

static int rkf_load_gray2(struct ebc_waveform *wf, int temp, u32 *frames)
{
	const u8 *seg = rkf_segment(wf, RKF_LUT_GRAY2, temp, frames);
	int ret;

	if (IS_ERR(seg))
		return PTR_ERR(seg);

	ret = rkf_decode(wf, wf->raw, RKF_RAW_MAX, *frames * 8, seg);
	if (ret)
		return ret;
	rkf_expand_gray2(wf->lut, wf->raw, *frames);
	return 0;
}

static int rkf_load_lut(struct ebc_waveform *wf, u32 mode, int temp,
			u32 *frames)
{
	const u8 *seg;
	u32 n;
	int ret;

	switch (mode) {
	case RKF_LUT_RESET:
		seg = rkf_segment(wf, RKF_LUT_RESET, temp, frames);
		if (IS_ERR(seg))
			return PTR_ERR(seg);
		ret = rkf_decode(wf, wf->raw, RKF_RAW_MAX, (*frames + 3) >> 2,
				 seg);
		if (ret)
			return ret;
		rkf_expand_reset(wf->lut, wf->raw, *frames);
		return 0;

	case RKF_LUT_MODE6:
		ret = rkf_load_direct(wf, RKF_LUT_MODE6, temp, frames);
		if (!ret)
			return 0;
		return rkf_load_direct(wf, RKF_LUT_GRAY16, temp, frames);

	case RKF_LUT_GRAYA2:
		ret = rkf_load_direct(wf, RKF_LUT_GRAYA2, temp, frames);
		if (!ret)
			return 0;
		return rkf_load_gray2(wf, temp, frames);

	case RKF_LUT_GRAY2:
		return rkf_load_gray2(wf, temp, frames);

	case RKF_LUT_AUTO:
		ret = rkf_load_direct(wf, RKF_LUT_GRAY16, temp, frames);
		if (ret)
			return ret;
		/* The overlay is optional; a damaged one is not. */
		seg = rkf_segment(wf, RKF_LUT_GRAY2, temp, &n);
		if (IS_ERR(seg))
			return PTR_ERR(seg) == -ENOENT ? 0 : PTR_ERR(seg);
		if (n > *frames)
			return 0;
		ret = rkf_decode(wf, wf->raw, RKF_RAW_MAX, n * 8, seg);
		if (ret)
			return ret;
		rkf_overlay_gray2(wf->lut, wf->raw, n);
		return 0;

	default:
		return rkf_load_direct(wf, RKF_LUT_GRAY16, temp, frames);
	}
}

/* EBC work mode to container LUT mode, as dispatched by ebc_register_update(). */
static u32 rkf_lut_mode(u32 ebc_mode)
{
	switch (ebc_mode) {
	case EBC_MODE_AUTO:
		return RKF_LUT_AUTO;
	case EBC_MODE_A2:
	case EBC_MODE_DIRECT_A2:
		return RKF_LUT_GRAYA2;
	case EBC_MODE_RESET:
		return RKF_LUT_RESET;
	case EBC_MODE_BLACK_WHITE:
		return RKF_LUT_GRAY2;
	case EBC_MODE_OED_PART:
		return RKF_LUT_MODE6;
	default:
		return RKF_LUT_GRAY16;
	}
}

/*
 * WBF: E Ink's own waveform file, the format a panel keeps in its own flash
 * (the RC2 panel carries 320_R177_AE6A41_ED060KD1U7_TC at 0x886 of its SPI
 * flash). Where rkf is a Huffman-compressed container built for this driver,
 * a WBF holds a table of pointers into run-length streams of 2-bit drive
 * codes - one code per (frame, target level, source level), 5 bits per pixel
 * per frame on the Carta-era files, so the 16 levels the driver wants are the
 * even indices. Mode and temperature are addressed through the file's own
 * tables, and a mode version 0x19 file has the same eight modes wbf2rkf.py
 * maps: INIT/DU/GC16/GL16/A2 become RESET/GRAY2/GRAY16/MODE6/GRAYA2 and
 * GLR16/GLD16 are dropped, exactly as the host converter does it.
 *
 * Nothing here writes the flash: the WBF is read out through nvmem, parsed in
 * place, and every LUT the driver asks for is decoded out of it.
 */

static u32 wbf_u24(const u8 *p)
{
	return p[0] | (p[1] << 8) | (p[2] << 16);
}

/*
 * The file checksum is a plain reflected CRC-32 (zlib's) over the whole file
 * with the checksum word itself read as zero - the four leading bytes count,
 * so the crc cannot start at byte 4. Bitwise, like the container CRC: one
 * call per probe does not pay for a table.
 */
static u32 wbf_crc32(const u8 *p, size_t n)
{
	u32 crc = 0xffffffffu;
	size_t i;
	int k;

	for (i = 0; i < n; i++) {
		crc ^= i < 4 ? 0 : p[i];
		for (k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (crc & 1 ? 0xedb88320u : 0);
	}
	return ~crc;
}

/*
 * A stream is a list of bytes, four 2-bit codes each (least significant pair
 * first), until the terminator byte; a lone "single" byte toggles a flag that
 * makes the next byte stand for itself instead of carrying a repeat count.
 * Codes arrive in file order: frame, then target level, then source level.
 */
typedef int (*wbf_emit_fn)(void *ctx, u32 idx, u8 code);

static int wbf_walk(const struct ebc_wbf *w, u32 off, wbf_emit_fn emit,
		    void *ctx, u32 *count)
{
	u32 i = off;
	u32 n = 0;
	bool single = false;

	if (off < WBF_HEADER_MIN || off >= w->size)
		return -EINVAL;

	for (;;) {
		u32 rep;
		u8 b;
		int k;

		if (i >= w->size)
			return -EINVAL;
		b = w->d[i];
		if (b == w->eb)
			break;
		if (b == w->sb) {
			single = !single;
			i++;
			continue;
		}
		if (single) {
			rep = 1;
			i++;
		} else {
			if (i + 1 >= w->size)
				return -EINVAL;
			rep = w->d[i + 1] + 1;
			i += 2;
		}

		for (; rep; rep--) {
			for (k = 0; k < 4; k++) {
				if (emit && emit(ctx, n, (b >> (2 * k)) & 3))
					return -EINVAL;
				if (++n > WBF_MAX_CODES)
					return -EINVAL;
			}
		}
	}

	/* The count is codes, not frames: the caller divides by levels squared. */
	if (count)
		*count = n;
	return 0;
}

/* Resolve a mode and a temperature to its stream, and count its frames. */
static int wbf_stream(const struct ebc_wbf *w, u8 mode, int temp, u32 *frames,
		      u32 *off)
{
	u32 mode_ptr, rng, lv2 = (u32)w->levels * w->levels;

	if (mode >= w->modes)
		return -EINVAL;

	mode_ptr = wbf_u24(w->d + w->wmta + 4 * mode);
	if (mode_ptr + 4 * w->ranges > w->size)
		return -EINVAL;

	/* The table holds each range's lower bound: the last one at or below. */
	for (rng = 0; rng + 1 < w->ranges && temp >= w->temps[rng + 1]; rng++)
		;
	*off = wbf_u24(w->d + mode_ptr + 4 * rng);

	if (wbf_walk(w, *off, NULL, NULL, frames))
		return -EINVAL;
	if (!*frames || *frames % lv2)
		return -EINVAL;
	*frames /= lv2;
	if (*frames > EBC_WAVEFORM_MAX_FRAMES)
		return -EINVAL;
	return 0;
}

/* A full 16 x 16 frame per decoded frame: GC16, GL16 and A2 go through here. */
struct wbf_direct {
	u8	*lut;
	u32	levels;
	u32	step;
	u32	max_frames;
};

static int wbf_emit_direct(void *ctx, u32 idx, u8 code)
{
	struct wbf_direct *c = ctx;
	u32 levels = c->levels;
	u32 frame = idx / (levels * levels);
	u32 new = (idx / levels) % levels;
	u32 old = idx % levels;
	u8 *p;

	if (frame >= c->max_frames)
		return -EINVAL;
	if (new % c->step || old % c->step)
		return 0;
	p = c->lut + frame * EBC_LUT_FRAME_BYTES + (old / c->step) * 4;
	put_unaligned_le32(get_unaligned_le32(p) |
			   ((u32)code << (2 * (new / c->step))), p);
	return 0;
}

/*
 * INIT drives every pixel of a frame the same way, so one code per frame
 * reaches the LUT as a whole uniform frame. Only the 16 levels count, as in
 * wbf2rkf.py's raw_reset().
 */
struct wbf_reset {
	u8	*lut;
	u32	levels;
	u32	step;
	u8	first;
	bool	have;
};

static int wbf_emit_reset(void *ctx, u32 idx, u8 code)
{
	struct wbf_reset *c = ctx;
	u32 levels = c->levels;
	u32 new = (idx / levels) % levels;
	u32 old = idx % levels;
	u32 frame = idx / (levels * levels);
	u32 word, i;

	if (new % c->step || old % c->step)
		return 0;
	if (frame >= EBC_WAVEFORM_MAX_FRAMES)
		return -EINVAL;

	if (!c->have) {
		c->first = code;
		c->have = true;
		word = c->first * 0x55555555u;
		for (i = 0; i < EBC_LUT_FRAME_BYTES / 4; i++)
			put_unaligned_le32(word, c->lut + frame * EBC_LUT_FRAME_BYTES + i * 4);
		return 0;
	}
	if (code != c->first)
		return -EINVAL;
	if (new == levels - c->step && old == levels - c->step)
		c->have = false;
	return 0;
}

/*
 * DU drives only toward the outermost levels, so it reaches the LUT as two
 * columns: bits 1:0 of each word for the drive to black, bits 31:30 for the
 * drive to white.
 */
struct wbf_gray2 {
	u8	*lut;
	u32	levels;
	u32	step;
	u32	max_frames;
};

static int wbf_emit_gray2(void *ctx, u32 idx, u8 code)
{
	struct wbf_gray2 *c = ctx;
	u32 levels = c->levels;
	u32 frame = idx / (levels * levels);
	u32 new = (idx / levels) % levels;
	u32 old = idx % levels;
	u8 *p;

	if (new % c->step || old % c->step)
		return 0;
	if (frame >= c->max_frames)
		return -EINVAL;
	new /= c->step;
	old /= c->step;
	if (new && new != 15)
		return code ? -EINVAL : 0;

	/*
	 * Clear the field first: over a decoded GC16 LUT (the AUTO mode) the
	 * DU drive replaces what is there, it does not add to it.
	 */
	p = c->lut + frame * EBC_LUT_FRAME_BYTES + old * 4;
	put_unaligned_le32((get_unaligned_le32(p) & ~(3u << (new ? 30 : 0))) |
			   (new ? ((u32)code << 30) : code), p);
	return 0;
}

/* WBF mode index for a container LUT mode; 0xff when the file has no such mode. */
static u8 wbf_mode_of(u32 lut_mode)
{
	switch (lut_mode) {
	case RKF_LUT_RESET:
		return WBF_MODE_INIT;
	case RKF_LUT_GRAY16:
	case RKF_LUT_AUTO:
		return WBF_MODE_GC16;
	case RKF_LUT_MODE6:
		return WBF_MODE_GL16;
	case RKF_LUT_GRAY2:
		return WBF_MODE_DU;
	case RKF_LUT_GRAYA2:
		return WBF_MODE_A2;
	default:
		return 0xff;
	}
}

static int wbf_load_direct(struct ebc_waveform *wf, u8 mode, int temp,
			   u32 *frames)
{
	struct wbf_direct c = {
		.lut = wf->lut,
		.levels = wf->wbf.levels,
		.step = wf->wbf.step,
		.max_frames = EBC_WAVEFORM_MAX_FRAMES,
	};
	u32 off;

	if (wbf_stream(&wf->wbf, mode, temp, frames, &off))
		return -EINVAL;
	return wbf_walk(&wf->wbf, off, wbf_emit_direct, &c, NULL) ?
	       -EINVAL : 0;
}

static int wbf_load_reset(struct ebc_waveform *wf, int temp, u32 *frames)
{
	struct wbf_reset c = {
		.lut = wf->lut,
		.levels = wf->wbf.levels,
		.step = wf->wbf.step,
	};
	u32 off;

	if (wbf_stream(&wf->wbf, WBF_MODE_INIT, temp, frames, &off))
		return -EINVAL;
	return wbf_walk(&wf->wbf, off, wbf_emit_reset, &c, NULL) ?
	       -EINVAL : 0;
}

static int wbf_load_gray2(struct ebc_waveform *wf, int temp, u32 *frames,
			  u32 max_frames)
{
	struct wbf_gray2 c = {
		.lut = wf->lut,
		.levels = wf->wbf.levels,
		.step = wf->wbf.step,
		.max_frames = max_frames,
	};
	u32 off;

	if (wbf_stream(&wf->wbf, WBF_MODE_DU, temp, frames, &off))
		return -EINVAL;
	if (*frames > max_frames)
		return -EINVAL;
	return wbf_walk(&wf->wbf, off, wbf_emit_gray2, &c, NULL) ?
	       -EINVAL : 0;
}

/* The driver's own mode selection, mirrored onto WBF mode indices. */
static int wbf_load_lut(struct ebc_waveform *wf, u32 lut_mode, int temp,
			u32 *frames)
{
	u8 mode = wbf_mode_of(lut_mode);
	u32 du, off;
	struct wbf_gray2 c;

	if (mode == 0xff)
		return -ENOENT;

	memset(wf->lut, 0, RKF_LUT_MAX);

	switch (lut_mode) {
	case RKF_LUT_RESET:
		return wbf_load_reset(wf, temp, frames);

	case RKF_LUT_GRAY2:
		return wbf_load_gray2(wf, temp, frames, EBC_WAVEFORM_MAX_FRAMES);

	case RKF_LUT_AUTO:
		/*
		 * AUTO is GC16 with the DU drive laid over the outermost
		 * levels. A DU stream longer than the GC16 one is no overlay
		 * at all, so it is left off - as the container path does.
		 */
		if (wbf_load_direct(wf, WBF_MODE_GC16, temp, frames))
			return -EINVAL;
		if (wbf_stream(&wf->wbf, WBF_MODE_DU, temp, &du, &off))
			return 0;
		if (du > *frames)
			return 0;
		c.lut = wf->lut;
		c.levels = wf->wbf.levels;
		c.step = wf->wbf.step;
		c.max_frames = *frames;
		return wbf_walk(&wf->wbf, off, wbf_emit_gray2, &c, &du) ?
		       -EINVAL : 0;

	default:
		return wbf_load_direct(wf, mode, temp, frames);
	}
}

/**
 * rk3026_ebc_waveform_lut - fetch the packed LUT for one mode and temperature
 * @wf: parsed waveform
 * @mode: EBC work mode
 * @temp: panel temperature in degrees Celsius
 * @frames: number of waveform frames, on success
 *
 * Returns a pointer to @frames * 0x40 bytes ready for the LUT SRAM window, or
 * NULL if the waveform has none for this mode.
 */
const u8 *rk3026_ebc_waveform_lut(struct ebc_waveform *wf, u32 mode, int temp,
				  u32 *frames)
{
	u32 lut_mode = rkf_lut_mode(mode);
	int ret;

	if (wf->cached_valid && wf->cached_mode == lut_mode &&
	    wf->cached_temp == temp) {
		*frames = wf->frames;
		return wf->lut;
	}

	wf->cached_valid = false;
	if (wf->is_wbf)
		ret = wbf_load_lut(wf, lut_mode, temp, &wf->frames);
	else
		ret = rkf_load_lut(wf, lut_mode, temp, &wf->frames);
	if (ret) {
		dev_warn_ratelimited(wf->dev,
				     "no waveform for mode %u at %d C (%d)\n",
				     mode, temp, ret);
		return NULL;
	}

	wf->cached_mode = lut_mode;
	wf->cached_temp = temp;
	wf->cached_valid = true;
	*frames = wf->frames;
	return wf->lut;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_waveform_lut);

/**
 * rk3026_ebc_waveform_parse - validate a container and build its Huffman tree
 * @dev: device to allocate against and to log through
 * @buf: container bytes, including the trailing CRC32 word
 * @len: length of @buf
 */
struct ebc_waveform *rk3026_ebc_waveform_parse(struct device *dev,
					       const u8 *buf, size_t len)
{
	/* Checkerboard key; the phase is chosen by the vendor tag. */
	static const u32 key_einks[2] = { 0xaa55aa55, 0x55aa55aa };
	static const u32 key_oed[2] = { 0x55aa55aa, 0xaa55aa55 };
	const u32 *key = key_einks;
	struct ebc_waveform *wf;
	u32 *weights, *work;
	s16 *parent;
	bool plain;
	size_t size;
	u32 crc;
	int i;

	if (len < RKF_PAYLOAD_OFF + 4) {
		dev_err(dev, "waveform truncated: %zu bytes\n", len);
		return ERR_PTR(-EINVAL);
	}

	size = rkf_u32(buf);
	if (size > RKF_MAX_SIZE || size + 4 > len || size <= RKF_PAYLOAD_OFF) {
		dev_err(dev, "waveform size %#zx out of range for %zu bytes\n",
			size, len);
		return ERR_PTR(-EINVAL);
	}

	if (memcmp(buf + 8, "rkf:v", 5)) {
		dev_err(dev, "waveform is not an rkf container\n");
		return ERR_PTR(-EINVAL);
	}

	if (!memcmp(buf + 0x1c, "Einks", 5)) {
		key = key_einks;
	} else if (!memcmp(buf + 0x1c, "OED", 3)) {
		key = key_oed;
	} else {
		dev_err(dev, "unknown waveform vendor tag %5ph\n", buf + 0x1c);
		return ERR_PTR(-EINVAL);
	}

	wf = devm_kzalloc(dev, sizeof(*wf), GFP_KERNEL);
	if (!wf)
		return ERR_PTR(-ENOMEM);
	wf->dev = dev;
	wf->base = buf;
	wf->size = size;

	weights = kmalloc_array(RKF_NSYM, sizeof(*weights), GFP_KERNEL);
	if (!weights)
		return ERR_PTR(-ENOMEM);

	/*
	 * A weight is a symbol count over at most 0x40000 payload bytes, so a
	 * word of 2^28 or more means the table is still obfuscated.
	 */
	plain = true;
	for (i = 0; i < RKF_NSYM; i++) {
		weights[i] = rkf_u32(buf + RKF_WEIGHTS_OFF + i * 4);
		if (weights[i] >= (1u << 28))
			plain = false;
	}

	crc = rkf_crc32(buf, size, RKF_WEIGHTS_OFF, RKF_PAYLOAD_OFF,
			plain ? key : NULL);
	if (crc != rkf_u32(buf + size)) {
		dev_err(dev, "waveform CRC mismatch: got %08x want %08x\n",
			crc, rkf_u32(buf + size));
		kfree(weights);
		return ERR_PTR(-EINVAL);
	}

	if (!plain)
		for (i = 0; i < RKF_NSYM; i++)
			weights[i] ^= key[i & 1];

	work = kmalloc_array(RKF_NNODE, sizeof(*work) + sizeof(*parent),
			     GFP_KERNEL);
	if (!work) {
		kfree(weights);
		return ERR_PTR(-ENOMEM);
	}
	parent = (s16 *)(work + RKF_NNODE);

	rkf_build_tree(wf, weights, work, parent);
	kfree(work);
	kfree(weights);

	wf->lut = devm_kmalloc(dev, RKF_LUT_MAX, GFP_KERNEL);
	wf->raw = devm_kmalloc(dev, RKF_RAW_MAX, GFP_KERNEL);
	if (!wf->lut || !wf->raw)
		return ERR_PTR(-ENOMEM);

	dev_info(dev, "waveform %.11s, panel %.32s, %zu bytes\n",
		 buf + 8, buf + 0x2c, size);
	return wf;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_waveform_parse);

/**
 * rk3026_ebc_waveform_parse_wbf - validate a WBF file and describe it
 * @dev: device to allocate against and to log through
 * @buf: the file, as read out of the panel's flash or a firmware blob
 * @len: bytes in @buf; the file's own size word may be shorter, the rest is
 *	 the erased flash around it
 *
 * The streams stay in @buf: every LUT is decoded on demand, so @buf has to
 * outlive the returned waveform.
 *
 * Returns the waveform, or an ERR_PTR.
 */
struct ebc_waveform *rk3026_ebc_waveform_parse_wbf(struct device *dev,
						   const u8 *buf, size_t len)
{
	struct ebc_waveform *wf;
	size_t size;
	const u8 *name;
	u32 bits, crc, i;

	if (len < WBF_HEADER_MIN) {
		dev_err(dev, "wbf truncated: %zu bytes\n", len);
		return ERR_PTR(-EINVAL);
	}

	size = get_unaligned_le32(buf + WBF_FILESIZE_OFF);
	if (size < WBF_HEADER_MIN || size > len || size > WBF_MAX_SIZE) {
		dev_err(dev, "wbf size %#zx out of range for %zu bytes\n",
			size, len);
		return ERR_PTR(-EINVAL);
	}

	if (buf[WBF_MODE_VER_OFF] != WBF_MODE_VERSION) {
		dev_err(dev, "wbf mode version %#x, not %#x\n",
			buf[WBF_MODE_VER_OFF], WBF_MODE_VERSION);
		return ERR_PTR(-EINVAL);
	}

	wf = devm_kzalloc(dev, sizeof(*wf), GFP_KERNEL);
	if (!wf)
		return ERR_PTR(-ENOMEM);
	wf->dev = dev;
	wf->base = buf;
	wf->size = size;
	wf->is_wbf = true;

	wf->wbf.d = buf;
	wf->wbf.size = size;
	wf->wbf.modes = buf[WBF_MODE_COUNT_OFF] + 1;
	wf->wbf.ranges = buf[WBF_RANGE_COUNT_OFF] + 1;
	wf->wbf.wmta = wbf_u24(buf + WBF_WMTA_OFF);
	wf->wbf.eb = buf[WBF_EB_OFF];
	wf->wbf.sb = buf[WBF_SB_OFF];

	if (wf->wbf.modes > WBF_MAX_MODES || wf->wbf.ranges > WBF_MAX_RANGES) {
		dev_err(dev, "wbf %u modes and %u ranges\n", wf->wbf.modes,
			wf->wbf.ranges);
		return ERR_PTR(-EINVAL);
	}
	if (wf->wbf.wmta < WBF_HEADER_MIN ||
	    wf->wbf.wmta + 4 * wf->wbf.modes > size) {
		dev_err(dev, "wbf mode table at %#x does not fit %#zx bytes\n",
			wf->wbf.wmta, size);
		return ERR_PTR(-EINVAL);
	}
	if (wf->wbf.eb == wf->wbf.sb) {
		dev_err(dev, "wbf terminator and lone byte are both %#x\n",
			wf->wbf.eb);
		return ERR_PTR(-EINVAL);
	}
	if (WBF_TEMPS_OFF + wf->wbf.ranges + 1 > size) {
		dev_err(dev, "wbf temperature table does not fit %#zx bytes\n",
			size);
		return ERR_PTR(-EINVAL);
	}
	for (i = 0; i < wf->wbf.ranges; i++) {
		wf->wbf.temps[i] = buf[WBF_TEMPS_OFF + i];
		if (i && wf->wbf.temps[i] <= wf->wbf.temps[i - 1]) {
			dev_err(dev, "wbf temperature bounds not ascending at %u\n", i);
			return ERR_PTR(-EINVAL);
		}
	}
	wf->wbf.temps[wf->wbf.ranges] = buf[WBF_TEMPS_OFF + wf->wbf.ranges];

	bits = (buf[WBF_LUTS_OFF] & 0x0c) == 0x04 ? 5 : 4;
	wf->wbf.levels = 1 << bits;
	wf->wbf.step = wf->wbf.levels / 16;

	crc = wbf_crc32(buf, size);
	if (crc != get_unaligned_le32(buf)) {
		dev_err(dev, "wbf CRC mismatch: got %08x want %08x\n", crc,
			get_unaligned_le32(buf));
		return ERR_PTR(-EINVAL);
	}

	wf->lut = devm_kmalloc(dev, RKF_LUT_MAX, GFP_KERNEL);
	if (!wf->lut)
		return ERR_PTR(-ENOMEM);

	/* The waveform name sits behind its own length byte; it may be absent. */
	name = (const u8 *)"";
	i = wbf_u24(buf + WBF_XWIA_OFF);
	if (i >= WBF_HEADER_MIN && (size_t)i + 2 + buf[i] <= size)
		name = buf + i;

	dev_info(dev, "wbf waveform %.*s, %u modes, %u ranges, %zu bytes\n",
		 name[0], name + 1, wf->wbf.modes, wf->wbf.ranges, size);
	return wf;
}
EXPORT_SYMBOL_IF_KUNIT(rk3026_ebc_waveform_parse_wbf);
