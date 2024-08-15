/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * 		    GNU GENERAL PUBLIC LICENSE
 * 		       Version 2, June 1991
 *
 * Copyright (C) 1989, 1991 Free Software Foundation, Inc.
 *                       51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 */

#ifndef CLINK_T241_CLINK_H
#define CLINK_T241_CLINK_H

#include <linux/device/class.h>
#include <linux/cdev.h>
#include <linux/mutex.h>

#define DEFAULT_LOOP_SLEEP_US		100
#define DEFAULT_LOOP_TIMEOUT_US		1000000 /* 1sec */
#define EOM_DONE_LOOP_TIMEOUT_US	2000 /* 2ms */
#define MIN_SOCKET_ID			0
#define MAX_SOCKET_ID			3
#define MIN_LINK_ID			0
#define MAX_LINK_ID			11

struct tegra_nvlink_dev {
	phys_addr_t nvlw_base[4][2];
	struct mutex nvlink_lock;

	struct class class;
	dev_t dev_t;
	struct cdev cdev;
	struct device *dev;
};

#endif /* CLINK_T241_CLINK_H */
