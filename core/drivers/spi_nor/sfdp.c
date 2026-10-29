// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <inttypes.h>
#include <io.h>
#include <string.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#include "spi_nor_priv.h"

/*
 * Serial Flash Discoverable Parameters, JEDEC JESD216.
 *
 * The SFDP space starts with an 8-byte header, followed by 8-byte parameter
 * headers. The first parameter header describes the Basic Flash Parameter
 * Table (BFPT), which carries the geometry used here.
 */
#define SFDP_SIGNATURE		0x50444653	/* "SFDP", little-endian */
#define SFDP_HDR_SIZE		8
#define SFDP_PARAM_HDR_SIZE	8
#define SFDP_BFPT_ID		0xff00
#define SFDP_MAX_PARAM_HDRS	8

/* BFPT length in dwords: 9 in JESD216, 16 from JESD216B */
#define BFPT_MIN_DWORDS		9
#define BFPT_MAX_DWORDS		16

/* BFPT dword indexes and fields, dword 1 being index 0 */
#define BFPT_DW1		0
#define BFPT_DW1_ERASE_4K_MASK	GENMASK_32(1, 0)
#define BFPT_DW1_ERASE_4K	0x1
#define BFPT_DW1_ADDR_BYTES	GENMASK_32(18, 17)
#define BFPT_DW1_ADDR_3B	0x0
#define BFPT_DW1_ADDR_3B_4B	0x1
#define BFPT_DW1_ADDR_4B	0x2
#define BFPT_DW2		1
#define BFPT_DW2_EXP		BIT32(31)
#define BFPT_DW8		7
#define BFPT_DW9		8
#define BFPT_DW11		10
#define BFPT_DW11_PAGE_EXP	GENMASK_32(7, 4)

#define SPI_NOR_DEFAULT_PAGE_SIZE	256

struct sfdp_param_hdr {
	uint16_t id;
	uint8_t len_dwords;
	uint32_t ptr;
};

static void decode_param_hdr(const uint8_t *raw, struct sfdp_param_hdr *hdr)
{
	hdr->id = (raw[7] << 8) | raw[0];
	hdr->len_dwords = raw[3];
	hdr->ptr = raw[4] | (raw[5] << 8) | (raw[6] << 16);
}

static TEE_Result find_bfpt(sfdp_read_func read, void *ctx,
			    struct sfdp_param_hdr *bfpt)
{
	uint8_t raw[SFDP_HDR_SIZE] = { };
	TEE_Result res = TEE_ERROR_GENERIC;
	unsigned int nph = 0;
	unsigned int n = 0;

	res = read(ctx, 0, raw, sizeof(raw));
	if (res)
		return res;

	if (get_le32(raw) != SFDP_SIGNATURE)
		return TEE_ERROR_ITEM_NOT_FOUND;

	/* Byte 6 is the number of parameter headers minus one */
	nph = MIN(raw[6] + 1U, (unsigned int)SFDP_MAX_PARAM_HDRS);

	for (n = 0; n < nph; n++) {
		res = read(ctx, SFDP_HDR_SIZE + n * SFDP_PARAM_HDR_SIZE, raw,
			   SFDP_PARAM_HDR_SIZE);
		if (res)
			return res;

		decode_param_hdr(raw, bfpt);
		if (bfpt->id == SFDP_BFPT_ID)
			return TEE_SUCCESS;
	}

	return TEE_ERROR_BAD_FORMAT;
}

static TEE_Result decode_density(uint32_t dw2, uint64_t *size)
{
	uint32_t val = dw2 & ~BFPT_DW2_EXP;

	if (dw2 & BFPT_DW2_EXP) {
		/* Density is 2^val bits; keep it within 2^63 bytes */
		if (val < 3 || val > 66)
			return TEE_ERROR_BAD_FORMAT;
		*size = BIT64(val - 3);
	} else {
		/* Density is val + 1 bits */
		*size = ((uint64_t)val + 1) / 8;
	}

	return *size ? TEE_SUCCESS : TEE_ERROR_BAD_FORMAT;
}

static void add_erase_type(struct spi_nor_geometry *geo, uint8_t size_exp,
			   uint8_t opcode)
{
	struct spi_nor_erase_type et = { };
	unsigned int n = 0;

	/* A size exponent of 0 marks an unused erase type */
	if (!size_exp || size_exp > 31)
		return;

	et.size = BIT32(size_exp);
	et.opcode = opcode;

	/* Insert sorted by increasing size, skip duplicates */
	for (n = 0; n < SPI_NOR_MAX_ERASE_TYPES; n++) {
		if (geo->erase[n].size == et.size)
			return;
		if (!geo->erase[n].size || geo->erase[n].size > et.size) {
			struct spi_nor_erase_type tmp = geo->erase[n];

			geo->erase[n] = et;
			et = tmp;
			if (!et.size)
				return;
		}
	}
}

TEE_Result spi_nor_sfdp_parse(sfdp_read_func read, void *ctx,
			      struct spi_nor_geometry *geo)
{
	uint8_t raw[BFPT_MAX_DWORDS * 4] = { };
	struct sfdp_param_hdr bfpt = { };
	uint32_t dw[BFPT_MAX_DWORDS] = { };
	TEE_Result res = TEE_ERROR_GENERIC;
	unsigned int ndw = 0;
	unsigned int n = 0;

	res = find_bfpt(read, ctx, &bfpt);
	if (res)
		return res;

	if (bfpt.len_dwords < BFPT_MIN_DWORDS)
		return TEE_ERROR_BAD_FORMAT;

	ndw = MIN(bfpt.len_dwords, (uint8_t)BFPT_MAX_DWORDS);
	res = read(ctx, bfpt.ptr, raw, ndw * 4);
	if (res)
		return res;

	for (n = 0; n < ndw; n++)
		dw[n] = get_le32(raw + n * 4);

	memset(geo, 0, sizeof(*geo));

	res = decode_density(dw[BFPT_DW2], &geo->size);
	if (res)
		return res;

	for (n = 0; n < 2; n++) {
		uint32_t erase = dw[BFPT_DW8 + n];

		add_erase_type(geo, erase & 0xff, (erase >> 8) & 0xff);
		add_erase_type(geo, (erase >> 16) & 0xff, erase >> 24);
	}

	/* Older tables may only describe the 4 KiB erase in dword 1 */
	if (!geo->erase[0].size &&
	    (dw[BFPT_DW1] & BFPT_DW1_ERASE_4K_MASK) == BFPT_DW1_ERASE_4K)
		add_erase_type(geo, 12, (dw[BFPT_DW1] >> 8) & 0xff);

	if (!geo->erase[0].size || geo->erase[0].size > geo->size)
		return TEE_ERROR_BAD_FORMAT;

	switch ((dw[BFPT_DW1] & BFPT_DW1_ADDR_BYTES) >> 17) {
	case BFPT_DW1_ADDR_3B:
		geo->addr4_capable = false;
		break;
	case BFPT_DW1_ADDR_3B_4B:
	case BFPT_DW1_ADDR_4B:
		geo->addr4_capable = true;
		break;
	default:
		return TEE_ERROR_BAD_FORMAT;
	}

	if (ndw > BFPT_DW11)
		geo->page_size = BIT32((dw[BFPT_DW11] & BFPT_DW11_PAGE_EXP) >>
				       4);
	else
		geo->page_size = SPI_NOR_DEFAULT_PAGE_SIZE;

	if (geo->page_size > geo->erase[0].size) {
		EMSG("SFDP: page size %"PRIu32" above erase size %"PRIu32,
		     geo->page_size, geo->erase[0].size);
		return TEE_ERROR_BAD_FORMAT;
	}

	return TEE_SUCCESS;
}
