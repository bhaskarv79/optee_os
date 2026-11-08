/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2023, Linaro Limited
 */

#ifndef __KERNEL_NV_COUNTER_H
#define __KERNEL_NV_COUNTER_H

#include <tee_api_types.h>
#include <types_ext.h>

TEE_Result nv_counter_get_ree_fs(uint32_t *value);
TEE_Result nv_counter_incr_ree_fs_to(uint32_t value);

/*
 * Rollback anchor of SPI-NOR FS when the flash has no RPMC: a counter
 * that the platform keeps outside the flash and that can only increase.
 * The default implementations return TEE_ERROR_NOT_IMPLEMENTED.
 */
TEE_Result nv_counter_get_spinor_fs(uint32_t *value);
TEE_Result nv_counter_incr_spinor_fs_to(uint32_t value);

#endif /*__KERNEL_NV_COUNTER_H*/
