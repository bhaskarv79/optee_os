// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * Platform NV counter of SPI-NOR FS kept in secure RAM, to test the
 * software fallback anchor (CFG_SPI_NOR_TEST_NV_COUNTER). It only lasts
 * until the next boot, so it is not a rollback anchor: never use it in a
 * product.
 */

#include <kernel/mutex.h>
#include <kernel/nv_counter.h>
#include <tee_api_types.h>

static uint32_t counter;
static struct mutex counter_mu = MUTEX_INITIALIZER;

TEE_Result nv_counter_get_spinor_fs(uint32_t *value)
{
	mutex_lock(&counter_mu);
	*value = counter;
	mutex_unlock(&counter_mu);

	return TEE_SUCCESS;
}

TEE_Result nv_counter_incr_spinor_fs_to(uint32_t value)
{
	mutex_lock(&counter_mu);
	if (value > counter)
		counter = value;
	mutex_unlock(&counter_mu);

	return TEE_SUCCESS;
}
