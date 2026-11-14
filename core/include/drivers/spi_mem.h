/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __DRIVERS_SPI_MEM_H
#define __DRIVERS_SPI_MEM_H

#include <kernel/dt_driver.h>
#include <libfdt.h>
#include <spi.h>
#include <stdbool.h>
#include <stdint.h>
#include <tee_api_types.h>
#include <util.h>

/*
 * SPI memory interface
 *
 * A SPI memory operation is one command sent to a memory device such as a
 * SPI-NOR flash, made of up to four phases sent with chip-select asserted:
 * an opcode, an address, dummy cycles and a data transfer. Each phase can
 * use its own bus width (1, 2, 4 or 8 lines).
 *
 * Controller drivers implement struct spi_mem_ctrl_ops and register
 * themselves as DT_DRIVER_SPI providers with spi_mem_register_provider().
 * Memory device drivers get their struct spi_mem with spi_mem_dt_get_dev(),
 * typically through DEFINE_SPI_MEM_DEV_DRIVER().
 *
 * The interface follows the Linux and U-Boot spi-mem layer so controller
 * drivers can be ported with little change. Callers serialize accesses to
 * a device; the interface does not lock.
 */

/**
 * DEFINE_SPI_MEM_DEV_DRIVER - Declare a SPI memory device driver
 *
 * @__name: SPI memory device driver name
 * @__match_table: match table associated to the driver
 * @__spi_mem_probe: probe function with the following prototype:
 *	TEE_Result (*probe)(struct spi_mem *mem, const void *fdt,
 *			    int node, const void *compat_data);
 */
#define DEFINE_SPI_MEM_DEV_DRIVER(__name, __match_table, __spi_mem_probe) \
	static TEE_Result __name ## _probe_spi_mem(const void *fdt, int node, \
						   const void *compat_data) \
	{ \
		struct spi_mem *mem = NULL; \
		TEE_Result res = TEE_ERROR_GENERIC; \
		\
		res = spi_mem_dt_get_dev(fdt, node, &mem); \
		if (res) \
			return res; \
		return __spi_mem_probe(mem, fdt, node, compat_data); \
	} \
	DEFINE_DT_DRIVER(__name ## _dt_driver) = { \
		.name = # __name, \
		.type = DT_DRIVER_SPI, \
		.match_table = __match_table, \
		.probe = __name ## _probe_spi_mem, \
	}

/* Device capabilities, from DT properties spi-cpha, spi-cpol */
#define SPI_MEM_CPHA		BIT(0)
#define SPI_MEM_CPOL		BIT(1)

enum spi_mem_data_dir {
	SPI_MEM_NO_DATA,
	SPI_MEM_DATA_IN,
	SPI_MEM_DATA_OUT,
};

/**
 * struct spi_mem_op - SPI memory operation
 *
 * @cmd.nbytes: Opcode size in bytes, 1 or 2
 * @cmd.buswidth: Number of lines used to send the opcode
 * @cmd.opcode: Opcode, sent most significant byte first
 * @addr.nbytes: Address size in bytes, 0 to 8; 0 means no address phase
 * @addr.buswidth: Number of lines used to send the address
 * @addr.val: Address, sent most significant byte first
 * @dummy.nbytes: Dummy cycles expressed in bytes at @dummy.buswidth;
 *	0 means no dummy phase
 * @dummy.buswidth: Number of lines used for the dummy cycles
 * @data.buswidth: Number of lines used for the data
 * @data.dir: Data direction, SPI_MEM_NO_DATA when there is no data phase
 * @data.nbytes: Number of data bytes
 * @data.buf.in: Input buffer when @data.dir is SPI_MEM_DATA_IN
 * @data.buf.out: Output buffer when @data.dir is SPI_MEM_DATA_OUT
 */
struct spi_mem_op {
	struct {
		uint8_t nbytes;
		uint8_t buswidth;
		uint16_t opcode;
	} cmd;
	struct {
		uint8_t nbytes;
		uint8_t buswidth;
		uint64_t val;
	} addr;
	struct {
		uint8_t nbytes;
		uint8_t buswidth;
	} dummy;
	struct {
		uint8_t buswidth;
		enum spi_mem_data_dir dir;
		size_t nbytes;
		union {
			void *in;
			const void *out;
		} buf;
	} data;
};

struct spi_mem;

/**
 * struct spi_mem_ctrl_ops - Operations provided by SPI memory controllers
 *
 * @exec_op: Execute one SPI memory operation on @mem. Mandatory.
 * @supports_op: Return true if the controller can execute @op on @mem.
 *	Optional; when NULL, any operation within the device bus widths is
 *	assumed to be supported.
 * @max_xfer_size: Largest data phase in bytes the controller can execute in
 *	one operation. Optional; when NULL, there is no limit.
 */
struct spi_mem_ctrl_ops {
	TEE_Result (*exec_op)(struct spi_mem *mem, const struct spi_mem_op *op);
	bool (*supports_op)(struct spi_mem *mem, const struct spi_mem_op *op);
	size_t (*max_xfer_size)(struct spi_mem *mem);
};

/**
 * struct spi_mem_ctrl - SPI memory controller
 *
 * @ops: Operations associated to the controller
 */
struct spi_mem_ctrl {
	const struct spi_mem_ctrl_ops *ops;
};

/**
 * struct spi_mem - SPI memory device on a controller
 *
 * @ctrl: Controller the device is connected to
 * @cs: Chip-select index, from the "reg" DT property
 * @max_hz: Maximum clock frequency, from "spi-max-frequency"; 0 if unset
 * @rx_buswidth: Lines usable to receive, from "spi-rx-bus-width"; 1 if unset
 * @tx_buswidth: Lines usable to send, from "spi-tx-bus-width"; 1 if unset
 * @mode: SPI_MEM_CPHA and SPI_MEM_CPOL flags
 * @priv: Controller driver private data for this device
 */
struct spi_mem {
	struct spi_mem_ctrl *ctrl;
	uint32_t cs;
	uint32_t max_hz;
	uint8_t rx_buswidth;
	uint8_t tx_buswidth;
	uint32_t mode;
	void *priv;
};

/**
 * spi_mem_dt_get_func - Typedef of function to get a SPI memory device from
 * devicetree properties
 *
 * @args: Pointer to devicetree description of the device to parse
 * @data: Pointer to data given at spi_mem_register_provider() call
 * @out_mem: Output pointer to the SPI memory device upon success
 */
typedef TEE_Result (*spi_mem_dt_get_func)(struct dt_pargs *args, void *data,
					  struct spi_mem **out_mem);

#ifdef CFG_DRIVERS_SPI_MEM
/**
 * spi_mem_create_dev() - Create a SPI memory device from a DT node
 *
 * @ctrl: Controller the device is connected to
 * @fdt: Device tree to work on
 * @node: Node of the memory device, a child of the controller node
 *
 * Return a struct spi_mem filled from the node, or NULL on error
 */
struct spi_mem *spi_mem_create_dev(struct spi_mem_ctrl *ctrl, const void *fdt,
				   int node);

/**
 * spi_mem_supports_op() - Check if an operation can be executed on a device
 *
 * @mem: SPI memory device
 * @op: Operation to check
 *
 * Return true if @op is well formed, within the device bus widths and
 * supported by the controller
 */
bool spi_mem_supports_op(struct spi_mem *mem, const struct spi_mem_op *op);

/**
 * spi_mem_exec_op() - Execute a SPI memory operation
 *
 * @mem: SPI memory device
 * @op: Operation to execute
 *
 * Return TEE_SUCCESS on success
 * Return TEE_ERROR_BAD_PARAMETERS if @op is not well formed
 * Return TEE_ERROR_NOT_SUPPORTED if @op cannot run on this device
 * Return another TEE_Result compliant code on controller error
 */
TEE_Result spi_mem_exec_op(struct spi_mem *mem, const struct spi_mem_op *op);

/**
 * spi_mem_max_xfer_size() - Largest data phase the controller can execute
 *
 * @mem: SPI memory device
 *
 * Return the maximum data phase size in bytes, SIZE_MAX if unlimited
 */
size_t spi_mem_max_xfer_size(struct spi_mem *mem);

/**
 * spi_mem_dt_get_dev() - Get a SPI memory device from its DT node
 *
 * @fdt: Device tree to work on
 * @nodeoffset: Node offset of the memory device
 * @out_mem: Output SPI memory device upon success
 *
 * Return TEE_SUCCESS in case of success
 * Return TEE_ERROR_DEFER_DRIVER_INIT if the controller is not initialized
 * Return a TEE_Result compliant code in case of error
 */
static inline TEE_Result spi_mem_dt_get_dev(const void *fdt, int nodeoffset,
					    struct spi_mem **out_mem)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	void *mem = NULL;

	res = dt_driver_device_from_parent(fdt, nodeoffset, DT_DRIVER_SPI,
					   &mem);
	if (!res)
		*out_mem = mem;

	return res;
}

/**
 * spi_mem_register_provider() - Register a SPI memory controller provider
 *	and add its child nodes to the DT probe list
 *
 * @fdt: Device tree to work on
 * @nodeoffset: Node offset of the controller
 * @get_dt_mem: Callback that returns the struct spi_mem for a child node
 * @data: Data passed to @get_dt_mem
 *
 * Return a TEE_Result compliant code
 */
static inline TEE_Result
spi_mem_register_provider(const void *fdt, int nodeoffset,
			  spi_mem_dt_get_func get_dt_mem, void *data)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	int subnode = -1;

	res = dt_driver_register_provider(fdt, nodeoffset,
					  (get_of_device_func)get_dt_mem,
					  data, DT_DRIVER_SPI);
	if (res)
		return res;

	fdt_for_each_subnode(subnode, fdt, nodeoffset)
		dt_driver_maybe_add_probe_node(fdt, subnode);

	return TEE_SUCCESS;
}

/**
 * spi_mem_legacy_get_dev() - Create a SPI memory device on a legacy
 *	struct spi_chip controller
 *
 * @chip: Configured legacy SPI controller, for example pl022 or ls_dspi
 * @out_mem: Output SPI memory device upon success
 *
 * The device uses one line in each direction. Each operation is sent as one
 * spi_ops::txrx8() transfer between spi_ops::start() and spi_ops::end(), so
 * the controller keeps chip-select asserted for the whole operation.
 *
 * Return a TEE_Result compliant code
 */
TEE_Result spi_mem_legacy_get_dev(struct spi_chip *chip,
				  struct spi_mem **out_mem);

/**
 * spi_mem_legacy_free_dev() - Free a device from spi_mem_legacy_get_dev()
 *
 * @mem: SPI memory device, may be NULL
 */
void spi_mem_legacy_free_dev(struct spi_mem *mem);
#else
static inline struct spi_mem *
spi_mem_create_dev(struct spi_mem_ctrl *ctrl __unused,
		   const void *fdt __unused, int node __unused)
{
	return NULL;
}

static inline bool spi_mem_supports_op(struct spi_mem *mem __unused,
				       const struct spi_mem_op *op __unused)
{
	return false;
}

static inline TEE_Result spi_mem_exec_op(struct spi_mem *mem __unused,
					 const struct spi_mem_op *op __unused)
{
	return TEE_ERROR_NOT_SUPPORTED;
}

static inline size_t spi_mem_max_xfer_size(struct spi_mem *mem __unused)
{
	return 0;
}

static inline TEE_Result spi_mem_dt_get_dev(const void *fdt __unused,
					    int nodeoffset __unused,
					    struct spi_mem **out_mem __unused)
{
	return TEE_ERROR_NOT_SUPPORTED;
}

static inline TEE_Result
spi_mem_register_provider(const void *fdt __unused, int nodeoffset __unused,
			  spi_mem_dt_get_func get_dt_mem __unused,
			  void *data __unused)
{
	return TEE_ERROR_NOT_SUPPORTED;
}

static inline TEE_Result
spi_mem_legacy_get_dev(struct spi_chip *chip __unused,
		       struct spi_mem **out_mem __unused)
{
	return TEE_ERROR_NOT_SUPPORTED;
}

static inline void spi_mem_legacy_free_dev(struct spi_mem *mem __unused)
{
}
#endif /* CFG_DRIVERS_SPI_MEM */

#endif /* __DRIVERS_SPI_MEM_H */
