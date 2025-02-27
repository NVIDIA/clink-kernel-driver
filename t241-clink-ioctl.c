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

#define NVLINK_MODULE_NAME	"t241-clink"
#define pr_fmt(fmt) "%s:%s:%d " fmt, NVLINK_MODULE_NAME, __func__, __LINE__

#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include "t241-clink.h"
#include "t241-clink-hw.h"
#include "t241-clink-ioctl.h"

struct tnvlink_ioctl {
	const char *const name;
	const size_t struct_size;
	int (*handler)(struct tegra_nvlink_dev *tdev, void *ioctl_struct);
};

static int send_dlcmd_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct);
static int send_dlstat_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct);
static int lnk_status_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct);
static int get_eom_status_ioctl(struct tegra_nvlink_dev *tdev,
			void *ioctl_struct);
static int setup_tc_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct);
static int log_uphy_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct);
static int err_status_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct);
static int clr_err_status_ioctl(struct tegra_nvlink_dev *tdev,
			void *ioctl_struct);
static int lp_cntr_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct);
static int do_reg_read_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct);
static int do_reg_write_ioctl(struct tegra_nvlink_dev *tdev,
			void *ioctl_struct);

static const struct tnvlink_ioctl ioctls[] = {
	[TNVLINK_IOCTL_SEND_DLCMD] = {
		.name			= "send_dlcmd",
		.struct_size		= sizeof(struct
						tegra_nvlink_send_dlcmd),
		.handler		= send_dlcmd_ioctl,
	},
	[TNVLINK_IOCTL_SEND_DLSTAT] = {
		.name			= "send_dlstat",
		.struct_size		= sizeof(struct
						tegra_nvlink_send_dlstat),
		.handler		= send_dlstat_ioctl,
	},
	[TNVLINK_IOCTL_LNK_STATUS] = {
		.name			= "lnk_status",
		.struct_size		= sizeof(struct
						tegra_nvlink_lnk_status),
		.handler		= lnk_status_ioctl,
	},
	[TNVLINK_IOCTL_GET_EOM_STATUS] = {
		.name			= "get_eom_status",
		.struct_size		= sizeof(struct tegra_nvlink_get_eom),
		.handler		= get_eom_status_ioctl,
	},
	[TNVLINK_IOCTL_SETUP_TC] = {
		.name			= "setup_tc",
		.struct_size		= sizeof(struct tegra_nvlink_setup_tc),
		.handler		= setup_tc_ioctl,
	},
	[TNVLINK_IOCTL_LOG_UPHY] = {
		.name			= "log_uphy",
		.struct_size		= sizeof(struct tegra_nvlink_log_uphy),
		.handler		= log_uphy_ioctl,
	},
	[TNVLINK_IOCTL_ERR_STATUS] = {
		.name			= "err_status",
		.struct_size		= sizeof(struct
						tegra_nvlink_err_status),
		.handler		= err_status_ioctl,
	},
	[TNVLINK_IOCTL_CLR_ERR_STATUS] = {
		.name			= "clr_err_status",
		.struct_size		= sizeof(struct
						tegra_nvlink_clr_err_status),
		.handler		= clr_err_status_ioctl,
	},
	[TNVLINK_IOCTL_LP_CNTR] = {
		.name			= "lp_cntr",
		.struct_size		= sizeof(struct
						tegra_nvlink_lp_cntr),
		.handler		= lp_cntr_ioctl,
	},
	[TNVLINK_IOCTL_DO_REG_READ] = {
		.name			= "reg_read",
		.struct_size		= sizeof(struct
						tegra_nvlink_do_reg_read),
		.handler		= do_reg_read_ioctl,
	},
	[TNVLINK_IOCTL_DO_REG_WRITE] = {
		.name			= "reg_write",
		.struct_size		= sizeof(struct
						tegra_nvlink_do_reg_write),
		.handler		= do_reg_write_ioctl,
	},
};

static u32 nvlw_minion_readl(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 reg)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			MINION_BASE + reg, 0x4);
	u32 val = __raw_readl(ptr);

	iounmap(ptr);
	return val;
}

static void nvlw_minion_writel(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			MINION_BASE + reg, 0x4);
	__raw_writel(val, ptr);

	iounmap(ptr);
}

static u32 nvlw_cpr_readl(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 reg)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			CPR_BASE + reg, 0x4);
	u32 val = __raw_readl(ptr);

	iounmap(ptr);
	return val;
}

static void nvlw_cpr_writel(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			CPR_BASE + reg, 0x4);
	__raw_writel(val, ptr);

	iounmap(ptr);
}

static u32 nvlw_nvldl_readl(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 link, u32 reg)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			NVLDL_LINK_BASE(link) + reg, 0x4);
	u32 val = __raw_readl(ptr);

	iounmap(ptr);
	return val;
}

static void nvlw_nvldl_writel(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 link, u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			NVLDL_LINK_BASE(link) + reg, 0x4);
	__raw_writel(val, ptr);

	iounmap(ptr);
}

static u32 nvlw_nvlipt_readl(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 reg)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			NVLIPT_BASE + reg, 0x4);
	u32 val = __raw_readl(ptr);

	iounmap(ptr);
	return val;
}

static void nvlw_nvlipt_writel(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			NVLIPT_BASE + reg, 0x4);
	__raw_writel(val, ptr);

	iounmap(ptr);
}

static u32 nvlw_nvlipt_lnk_readl(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 link, u32 reg)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			NVLIPT_LINK_BASE(link) + reg, 0x4);
	u32 val = __raw_readl(ptr);

	iounmap(ptr);
	return val;
}

static void nvlw_nvlipt_lnk_writel(struct tegra_nvlink_dev *tdev, u32 socket_id,
				u32 nvlw_id, u32 link, u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			NVLIPT_LINK_BASE(link) + reg, 0x4);
	__raw_writel(val, ptr);

	iounmap(ptr);
}

static u32 nvlw_tlc_readl(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 link, u32 reg)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			TLC_LINK_BASE(link) + reg, 0x4);
	u32 val = __raw_readl(ptr);

	iounmap(ptr);
	return val;
}

static void nvlw_tlc_writel(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 link, u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] +
			TLC_LINK_BASE(link) + reg, 0x4);
	__raw_writel(val, ptr);

	iounmap(ptr);
}

static u32 nvlw_nvlw_readl(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 reg)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] + reg,
				0x4);
	u32 val = __raw_readl(ptr);

	iounmap(ptr);
	return val;
}

static void nvlw_nvlw_writel(struct tegra_nvlink_dev *tdev, u32 socket_id,
			u32 nvlw_id, u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(tdev->nvlw_base[socket_id][nvlw_id] + reg,
				0x4);
	__raw_writel(val, ptr);

	iounmap(ptr);
}

static u32 car_readl(struct tegra_nvlink_dev *tdev, u32 socket_id, u32 reg)
{
	void __iomem *ptr = ioremap(SOCKET_x_MMIO_BASE(socket_id) + CAR_BASE +
			reg, 0x4);
	u32 val = __raw_readl(ptr);

	iounmap(ptr);
	return val;
}

static void car_writel(struct tegra_nvlink_dev *tdev, u32 socket_id, u32 reg,
		u32 val)
{
	void __iomem *ptr = ioremap(SOCKET_x_MMIO_BASE(socket_id) + CAR_BASE +
			reg, 0x4);
	__raw_writel(val, ptr);

	iounmap(ptr);
}

/*
 * Wait for a bit to be set or cleared in an NVLINK register. If the desired bit
 * condition doesn't happen in a certain amount of time, a timeout will happen.
 */
static int wait_for_minion_reg_cond(struct tegra_nvlink_dev *tdev,
				u32 socket_id, u32 nvlw_id, u32 reg, u32 bit,
				bool check_for_bit_set, char *bit_name,
				u32 (*reg_readl)(struct tegra_nvlink_dev *, u32,
				u32, u32), u32 *reg_val, u32 timeout_us)
{
	u32 elapsed_us = 0;

	do {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US*2);
		elapsed_us += DEFAULT_LOOP_SLEEP_US;

		*reg_val = reg_readl(tdev, socket_id, nvlw_id, reg);
		if ((check_for_bit_set && (*reg_val & BIT(bit))) ||
		    (!check_for_bit_set && ((*reg_val & BIT(bit)) == 0)))
			break;
	} while (elapsed_us < timeout_us);
	if (elapsed_us >= timeout_us) {
		if (check_for_bit_set) {
			pr_err("Timeout waiting for the %s bit to get set",
				bit_name);
		} else {
			pr_err("Timeout waiting for the %s bit to get cleared",
				bit_name);
		}
		return -ETIMEDOUT;
	}

	return 0;
}

static int wait_for_nvlphyctl_reg_cond(struct tegra_nvlink_dev *tdev,
				u32 socket_id, u32 nvlw_id, u32 link, u32 reg,
				u32 bit, bool check_for_bit_set, char *bit_name,
				u32 (*reg_readl)(struct tegra_nvlink_dev *, u32,
				u32, u32, u32), u32 *reg_val, u32 timeout_us)
{
	u32 elapsed_us = 0;

	do {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US*2);
		elapsed_us += DEFAULT_LOOP_SLEEP_US;

		*reg_val = reg_readl(tdev, socket_id, nvlw_id, link, reg);
		if ((check_for_bit_set && (*reg_val & BIT(bit))) ||
		    (!check_for_bit_set && ((*reg_val & BIT(bit)) == 0)))
			break;
	} while (elapsed_us < timeout_us);
	if (elapsed_us >= timeout_us) {
		if (check_for_bit_set) {
			pr_err("Timeout waiting for the %s bit to get set",
				bit_name);
		} else {
			pr_err("Timeout waiting for the %s bit to get cleared",
				bit_name);
		}
		return -ETIMEDOUT;
	}

	return 0;
}

/* Send a DL command to the MINION and wait for command completion */
static int minion_send_dlcmd(struct tegra_nvlink_dev *tdev, u32 socket_id,
		u32 link_id, u32 cmd, u32 scratch0_val, bool use_dlcmd_data,
		u32 dlcmd_data)
{
	int err = 0;
	u32 reg_val = 0, nvlw_id, link;

	nvlw_id = link_id / 6;
	link = link_id % 6;

	if (use_dlcmd_data)
		nvlw_minion_writel(tdev, socket_id, nvlw_id,
				MINION_NVLINK_DL_CMD_DATA(link), dlcmd_data);

	/* Verify that the last MINION command executed successfully */
	/* Ensure MINION_NVLINK_DL_CMD_READY bit is set */
	err = wait_for_minion_reg_cond(tdev, socket_id, nvlw_id,
			MINION_NVLINK_DL_CMD(link), MINION_NVLINK_DL_CMD_READY,
			true, "MINION_NVLINK_DL_CMD_READY", nvlw_minion_readl,
			&reg_val, DEFAULT_LOOP_TIMEOUT_US);
	if (err < 0) {
		pr_err("previous Minion command not completed!");
		goto fail;
	}

	if (reg_val & BIT(MINION_NVLINK_DL_CMD_FAULT)) {
		reg_val = BIT(MINION_NVLINK_DL_CMD_FAULT);
		nvlw_minion_writel(tdev, socket_id, nvlw_id,
				MINION_NVLINK_DL_CMD(link), reg_val);
		pr_err("previous Minion command fault!");
		err = -EOPNOTSUPP;
		goto fail;
	}

	/* Write to minion scratch if needed by command */
	if (cmd == MINION_NVLINK_DL_CMD_COMMAND_CONFIGEOM)
		nvlw_minion_writel(tdev, socket_id, nvlw_id, MINION_MISC_0,
				scratch0_val);

	/* Send command to MINION */
	reg_val = (cmd << MINION_NVLINK_DL_CMD_COMMAND_SHIFT) &
			MINION_NVLINK_DL_CMD_COMMAND_MASK;
	reg_val |= BIT(MINION_NVLINK_DL_CMD_FAULT);
	nvlw_minion_writel(tdev, socket_id, nvlw_id, MINION_NVLINK_DL_CMD(link),
			reg_val);

	/* Wait for MINION_NVLINK_DL_CMD_READY bit to be set */
	err = wait_for_minion_reg_cond(tdev, socket_id, nvlw_id,
			MINION_NVLINK_DL_CMD(link), MINION_NVLINK_DL_CMD_READY,
			true, "MINION_NVLINK_DL_CMD_READY", nvlw_minion_readl,
			&reg_val, DEFAULT_LOOP_TIMEOUT_US);
	if (err < 0) {
		pr_err("MINION command timeout!");
		goto fail;
	}

	if (reg_val & BIT(MINION_NVLINK_DL_CMD_FAULT)) {
		pr_err("MINION command fault!");
		reg_val = BIT(MINION_NVLINK_DL_CMD_FAULT);
		nvlw_minion_writel(tdev, socket_id, nvlw_id,
				MINION_NVLINK_DL_CMD(link), reg_val);
		err = -EFAULT;
		goto fail;
	}

	goto success;

fail:
	pr_err("MINION dlcmd(cmd %d, socket %d, link_id %d, err %d) failed",
		cmd, socket_id, link_id, err);
success:
	return err;
}

static int minion_send_dlstat(struct tegra_nvlink_dev *tdev, u32 socket_id,
		u32 link_id, u32 dlstat_id, u32 args, u32 *dlstat_data)
{
	int err = 0;
	u32 reg_val = 0, nvlw_id, link;

	nvlw_id = link_id / 6;
	link = link_id % 6;

	reg_val = nvlw_minion_readl(tdev, socket_id, nvlw_id,
				MINION_NVLINK_DL_STAT(link));
	if (!(reg_val & BIT(MINION_NVLINK_DL_STAT_READY))) {
		pr_err("MINION DLSTAT interface not Ready\n");
		return -EPERM;
	}

	reg_val = MINION_NVLINK_DL_STAT_STATUSIDX(dlstat_id);
	reg_val |= MINION_NVLINK_DL_STAT_ARGS(args);
	reg_val |= BIT(MINION_NVLINK_DL_STAT_READY);
	nvlw_minion_writel(tdev, socket_id, nvlw_id,
			MINION_NVLINK_DL_STAT(link), reg_val);

	/* Wait for MINION_NVLINK_DL_STAT_READY bit to be set */
	err = wait_for_minion_reg_cond(tdev, socket_id, nvlw_id,
				MINION_NVLINK_DL_STAT(link),
				MINION_NVLINK_DL_STAT_READY,
				true, "MINION_NVLINK_DL_STAT_READY",
				nvlw_minion_readl, &reg_val,
				DEFAULT_LOOP_TIMEOUT_US);
	if (err < 0) {
		pr_err("MINION dlstat(id %d, socket %d, link_id %d) failed!",
			dlstat_id, socket_id, link_id);
		return err;
	}

	/* READ DL STATUS DATA */
	*dlstat_data = nvlw_minion_readl(tdev, socket_id, nvlw_id,
					MINION_NVLINK_DL_STATDATA(link));

	return err;
}

static int send_dlcmd_ioctl(struct tegra_nvlink_dev *tdev,
			void *ioctl_struct)
{
	struct tegra_nvlink_send_dlcmd *send_dlcmd =
			(struct tegra_nvlink_send_dlcmd *)ioctl_struct;
	int ret = 0;

	if (send_dlcmd->socket_id < MIN_SOCKET_ID ||
			send_dlcmd->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (send_dlcmd->link_id < MIN_LINK_ID ||
			send_dlcmd->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}

	ret = minion_send_dlcmd(tdev, send_dlcmd->socket_id,
			send_dlcmd->link_id, send_dlcmd->dlcmd,
			send_dlcmd->dlcmd_data,
			send_dlcmd->use_dlcmd_data, send_dlcmd->dlcmd_data);
	if (ret < 0) {
		pr_err("Error sending %d dlcmd to MINION\n",
			send_dlcmd->dlcmd);
		goto fail;
	}
	goto success;

fail:
	pr_err("%s has failed", __func__);
success:
	send_dlcmd->status = ret;
	return ret;
}

static int send_dlstat_ioctl(struct tegra_nvlink_dev *tdev,
			void *ioctl_struct)
{
	struct tegra_nvlink_send_dlstat *send_dlstat =
			(struct tegra_nvlink_send_dlstat *)ioctl_struct;
	int ret = 0;
	u32 dlstat_data;

	if (send_dlstat->socket_id < MIN_SOCKET_ID ||
			send_dlstat->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (send_dlstat->link_id < MIN_LINK_ID ||
			send_dlstat->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}

	ret = minion_send_dlstat(tdev, send_dlstat->socket_id,
			send_dlstat->link_id, send_dlstat->dlstat_id,
			send_dlstat->args, &dlstat_data);
	if (ret < 0) {
		pr_err("Error sending %d dlstat to MINION\n",
			send_dlstat->dlstat_id);
		goto fail;
	} else {
		send_dlstat->dlstat_data = dlstat_data;
		goto success;
	}

fail:
	pr_err("%s has failed", __func__);
success:
	send_dlstat->status = ret;
	return ret;
}

static const u32 lnk_stat_dlstat_val[] = {
	[NV_NVLSTAT_LNK2] = 0x12,
	[NV_NVLSTAT_LNK3] = 0x13,
	[NV_NVLSTAT_LNK4] = 0x14,
	[NV_NVLSTAT_LNK5] = 0x15,
};

static int lnk_status_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_lnk_status *lnk_status =
				(struct tegra_nvlink_lnk_status *)ioctl_struct;
	int ret = 0;
	u32 dlstat_data;

	if (lnk_status->socket_id < MIN_SOCKET_ID ||
				lnk_status->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (lnk_status->link_id < MIN_LINK_ID ||
				lnk_status->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (lnk_status->id >= INVALID_LNK_STAT_TYPE) {
		pr_err("Invalid lnk status id specified");
		ret = -EINVAL;
		goto fail;
	}

	ret = minion_send_dlstat(tdev, lnk_status->socket_id,
				lnk_status->link_id,
				lnk_stat_dlstat_val[lnk_status->id], 0,
				&dlstat_data);
	if (ret < 0) {
		pr_err("Error sending %d dlstat to MINION\n",
			lnk_stat_dlstat_val[lnk_status->id]);
		goto fail;
	} else {
		lnk_status->lnk_status = dlstat_data;
		goto success;
	}

fail:
	pr_err("%s has failed", __func__);
success:
	lnk_status->status = ret;
	return ret;
}

static int get_eom_status_ioctl(struct tegra_nvlink_dev *tdev,
			void *ioctl_struct)
{
	struct tegra_nvlink_get_eom *get_eom =
			(struct tegra_nvlink_get_eom *)ioctl_struct;
	int ret = 0, eom_done_status = 0;
	u32 nvlw_id, link, reg_val;

	if (get_eom->socket_id < MIN_SOCKET_ID ||
				get_eom->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (get_eom->link_id < MIN_LINK_ID || get_eom->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}

	nvlw_id = get_eom->link_id / 6;
	link = get_eom->link_id % 6;

	/* Issue DLCMD[CONFIGEOM] */
	ret = minion_send_dlcmd(tdev, get_eom->socket_id, get_eom->link_id,
			MINION_NVLINK_DL_CMD_COMMAND_CONFIGEOM,
			get_eom->params, false, 0);
	if (ret < 0) {
		pr_err("Error sending CONFIGEOM command to MINION\n");
		goto fail;
	}

	/* Wait for PHYCTL_LANE PAD_CTL_4[RX_EOM_DONE] = 0 on lane 0 */
	ret = wait_for_nvlphyctl_reg_cond(tdev, get_eom->socket_id, nvlw_id,
				link, NVLPHYCTL_LANE_PAD_CTL_4_0,
				NVLPHYCTL_LANE_PAD_CTL_4_0_RX_EOM_DONE,
				false, "NVLPHYCTL_LANE_PAD_CTL_4_0_RX_EOM_DONE",
				nvlw_nvldl_readl, &reg_val,
				EOM_DONE_LOOP_TIMEOUT_US);
	if (ret < 0) {
		pr_err("Timeout polling for RX_EOM_DONE=0 on lane 0\n");
		goto fail;
	}

	/* Wait for PHYCTL_LANE PAD_CTL_4[RX_EOM_DONE] = 0 on lane 1*/
	ret = wait_for_nvlphyctl_reg_cond(tdev, get_eom->socket_id, nvlw_id,
				link, NVLPHYCTL_LANE_PAD_CTL_4_1,
				NVLPHYCTL_LANE_PAD_CTL_4_1_RX_EOM_DONE,
				false, "NVLPHYCTL_LANE_PAD_CTL_4_1_RX_EOM_DONE",
				nvlw_nvldl_readl, &reg_val,
				EOM_DONE_LOOP_TIMEOUT_US);
	if (ret < 0) {
		pr_err("Timeout polling for RX_EOM_DONE=0 on lane 1\n");
		goto fail;
	}

	/* Issue DLCMD[STARTEOM] */
	ret = minion_send_dlcmd(tdev, get_eom->socket_id, get_eom->link_id,
			MINION_NVLINK_DL_CMD_COMMAND_STARTEOM, 0, false, 0);
	if (ret < 0) {
		pr_err("Error sending STARTEOM command to MINION\n");
		goto fail;
	}

	/* Wait for PHYCTL_LANE PAD_CTL_4[RX_EOM_DONE] = 1 on lane 0 */
	ret = wait_for_nvlphyctl_reg_cond(tdev, get_eom->socket_id, nvlw_id,
				link, NVLPHYCTL_LANE_PAD_CTL_4_0,
				NVLPHYCTL_LANE_PAD_CTL_4_0_RX_EOM_DONE,
				true, "NVLPHYCTL_LANE_PAD_CTL_4_0_RX_EOM_DONE",
				nvlw_nvldl_readl, &reg_val,
				EOM_DONE_LOOP_TIMEOUT_US);
	if (ret < 0) {
		pr_err("Timeout polling for RX_EOM_DONE=1 on lane 0\n");
		eom_done_status = ret;
	}

	/* Wait for PHYCTL_LANE PAD_CTL_4[RX_EOM_DONE] = 1 on lane 1 */
	ret = wait_for_nvlphyctl_reg_cond(tdev, get_eom->socket_id, nvlw_id,
				link, NVLPHYCTL_LANE_PAD_CTL_4_1,
				NVLPHYCTL_LANE_PAD_CTL_4_1_RX_EOM_DONE,
				true, "NVLPHYCTL_LANE_PAD_CTL_4_1_RX_EOM_DONE",
				nvlw_nvldl_readl, &reg_val,
				EOM_DONE_LOOP_TIMEOUT_US);
	if (ret < 0) {
		pr_err("Timeout polling for RX_EOM_DONE=1 on lane 1\n");
		eom_done_status = ret;
	}

	/* Issue DLCMD[ENDEOM] */
	ret = minion_send_dlcmd(tdev, get_eom->socket_id, get_eom->link_id,
			MINION_NVLINK_DL_CMD_COMMAND_ENDEOM, 0, false, 0);
	if (ret < 0) {
		pr_err("Error sending ENDEOM command to MINION\n");
		if (eom_done_status < 0) {
			pr_err("EOM DONE not set for all lanes on link");
			ret = eom_done_status;
		}
		goto fail;
	} else {
		/* If any timeouts from EOM_DONE=1 polling, ignore.
		 * Get EOM Status.
		 */
		reg_val = nvlw_nvldl_readl(tdev, get_eom->socket_id, nvlw_id,
					link, NVLPHYCTL_LANE_PAD_CTL_4_0);
		get_eom->l0_rx_eom_status =
			NVLPHYCTL_LANE_PAD_CTL_4_0_RX_EOM_STATUS_V(reg_val);

		reg_val = nvlw_nvldl_readl(tdev, get_eom->socket_id, nvlw_id,
					link, NVLPHYCTL_LANE_PAD_CTL_4_1);
		get_eom->l1_rx_eom_status =
			NVLPHYCTL_LANE_PAD_CTL_4_1_RX_EOM_STATUS_V(reg_val);

		goto success;
	}

fail:
	pr_err("%s has failed", __func__);
success:
	get_eom->status = ret;
	return ret;
}

static int setup_tc_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_setup_tc *setup_tc =
				(struct tegra_nvlink_setup_tc *)ioctl_struct;
	u32 dlcmd_data = 0, dlstat_data, args, cmask, counter;
	u64 reg_hi;
	unsigned long ulong_cmask;
	int ret = 0;

	if (setup_tc->socket_id < MIN_SOCKET_ID ||
					setup_tc->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (setup_tc->link_id < MIN_LINK_ID ||
					setup_tc->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (setup_tc->counter_mask >> 4) {
		pr_err("Invalid counter_mask specified");
		ret = -EINVAL;
		goto fail;
	}

	cmask = setup_tc->counter_mask;

	switch (setup_tc->exp) {
	case STOP_TC:
		dlcmd_data &= ~BIT(MINION_NVLINK_DL_CMD_DATA_ENABLE);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_TX_TC_F(cmask);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_RX_TC_F(cmask);

		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_SETUPTC, 0,
				true, dlcmd_data);
		if (ret < 0) {
			pr_err("Error sending SETUPTC command to MINION\n");
			goto fail;
		}

		break;

	case RESET_TC:
		dlcmd_data |= BIT(MINION_NVLINK_DL_CMD_DATA_RESET);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_TX_TC_F(cmask);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_RX_TC_F(cmask);

		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_SETUPTC, 0,
				true, dlcmd_data);
		if (ret < 0) {
			pr_err("Error sending SETUPTC command to MINION\n");
			goto fail;
		}

		break;

	case HWRESET_TC:
		dlcmd_data |= BIT(MINION_NVLINK_DL_CMD_DATA_HWRESET);

		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_SETUPTC, 0,
				true, dlcmd_data);
		if (ret < 0) {
			pr_err("Error sending SETUPTC command to MINION\n");
			goto fail;
		}

		break;

	case SETUP_NOP:
		dlcmd_data |= BIT(MINION_NVLINK_DL_CMD_DATA_ENABLE);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_EXP_F(
			MINION_NVLINK_DL_CMD_DATA_EXP_NOP);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_TX_TC_F(cmask);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_RX_TC_F(cmask);

		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_SETUPTC, 0,
				true, dlcmd_data);
		if (ret < 0) {
			pr_err("Error sending SETUPTC command to MINION\n");
			goto fail;
		}

		break;

	case SETUP_BYTES_COUNT:
		dlcmd_data |= BIT(MINION_NVLINK_DL_CMD_DATA_ENABLE);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_EXP_F(
			MINION_NVLINK_DL_CMD_DATA_EXP_BYTES_COUNT);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_TX_TC_F(cmask);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_RX_TC_F(cmask);

		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_SETUPTC, 0,
				true, dlcmd_data);
		if (ret < 0) {
			pr_err("Error sending SETUPTC command to MINION\n");
			goto fail;
		}

		break;

	case SETUP_CYCLES_COUNT:
		dlcmd_data |= BIT(MINION_NVLINK_DL_CMD_DATA_ENABLE);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_EXP_F(
			MINION_NVLINK_DL_CMD_DATA_EXP_CYCLES_COUNT);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_TX_TC_F(cmask);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_RX_TC_F(cmask);

		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_SETUPTC, 0,
				true, dlcmd_data);
		if (ret < 0) {
			pr_err("Error sending SETUPTC command to MINION\n");
			goto fail;
		}

		break;

	case SETUP_PACKETS_COUNT:
		dlcmd_data |= BIT(MINION_NVLINK_DL_CMD_DATA_ENABLE);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_EXP_F(
			MINION_NVLINK_DL_CMD_DATA_EXP_PACKETS_COUNT);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_TX_TC_F(cmask);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_RX_TC_F(cmask);

		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_SETUPTC, 0,
				true, dlcmd_data);
		if (ret < 0) {
			pr_err("Error sending SETUPTC command to MINION\n");
			goto fail;
		}

		break;

	case SETUP_FLITS_COUNT:
		dlcmd_data |= BIT(MINION_NVLINK_DL_CMD_DATA_ENABLE);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_EXP_F(
			MINION_NVLINK_DL_CMD_DATA_EXP_FLITS_COUNT);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_TX_TC_F(cmask);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_RX_TC_F(cmask);

		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_SETUPTC, 0,
				true, dlcmd_data);
		if (ret < 0) {
			pr_err("Error sending SETUPTC command to MINION\n");
			goto fail;
		}

		break;

	case START_TC:
		dlcmd_data |= BIT(MINION_NVLINK_DL_CMD_DATA_ENABLE);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_TX_TC_F(cmask);
		dlcmd_data |= MINION_NVLINK_DL_CMD_DATA_RX_TC_F(cmask);

		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_SETUPTC, 0,
				true, dlcmd_data);
		if (ret < 0) {
			pr_err("Error sending SETUPTC command to MINION\n");
			goto fail;
		}

		break;

	case CAPTURE_TC:
		ret = minion_send_dlcmd(tdev, setup_tc->socket_id,
				setup_tc->link_id,
				MINION_NVLINK_DL_CMD_COMMAND_CAPTURETC, 0,
				false, 0);
		if (ret < 0) {
			pr_err("Error sending CAPTURETC MINION command\n");
			goto fail;
		}

		break;

	case READ_TC:
		args = MINION_NVLINK_DL_STAT_ARGS_TC_TX;
		ulong_cmask = cmask;
		for_each_set_bit(counter, &ulong_cmask, 4) {
			setup_tc->tx_counter[counter] = 0;
			args |= MINION_NVLINK_DL_STAT_ARGS_TC_COUNTER(counter);

			// DEBUG_TP_CNTR_CAPTURE_LO for TX
			ret = minion_send_dlstat(tdev, setup_tc->socket_id,
					setup_tc->link_id, NV_NVLSTAT_TC00,
					args, &dlstat_data);
			if (ret < 0) {
				pr_err("Error sending NV_NVLSTAT_TC00\n");
				goto fail;
			} else {
				setup_tc->tx_counter[counter] = dlstat_data;
			}

			// DEBUG_TP_CNTR_CAPTURE_HI for TX
			ret = minion_send_dlstat(tdev, setup_tc->socket_id,
					setup_tc->link_id, NV_NVLSTAT_TC01,
					args, &dlstat_data);
			if (ret < 0) {
				pr_err("Error sending NV_NVLSTAT_TC01\n");
				goto fail;
			} else {
				reg_hi = dlstat_data;
				setup_tc->tx_counter[counter] |=
					(((u64) 0xffffffff & reg_hi) << 32);
			}
		}
		args = MINION_NVLINK_DL_STAT_ARGS_TC_RX;
		for_each_set_bit(counter, &ulong_cmask, 4) {
			setup_tc->rx_counter[counter] = 0;
			args |= MINION_NVLINK_DL_STAT_ARGS_TC_COUNTER(counter);

			// DEBUG_TP_CNTR_CAPTURE_LO for RX
			ret = minion_send_dlstat(tdev, setup_tc->socket_id,
					setup_tc->link_id, NV_NVLSTAT_TC00,
					args, &dlstat_data);
			if (ret < 0) {
				pr_err("Error sending NV_NVLSTAT_TC00\n");
				goto fail;
			} else {
				setup_tc->rx_counter[counter] = dlstat_data;
			}

			// DEBUG_TP_CNTR_CAPTURE_HI for RX
			ret = minion_send_dlstat(tdev, setup_tc->socket_id,
					setup_tc->link_id, NV_NVLSTAT_TC01,
					args, &dlstat_data);
			if (ret < 0) {
				pr_err("Error sending NV_NVLSTAT_TC01\n");
				goto fail;
			} else {
				reg_hi = dlstat_data;
				setup_tc->rx_counter[counter] |=
					(((u64) 0xffffffff & reg_hi) << 32);
			}
		}

		break;
	default:
		pr_err("Invalid TC Experiment specified");
		ret = -EINVAL;
		goto fail;
	}

	goto success;

fail:
	pr_err("%s has failed", __func__);
success:
	setup_tc->status = ret;
	return ret;
}

static int log_uphy_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_log_uphy *log_uphy =
				(struct tegra_nvlink_log_uphy *)ioctl_struct;
	int ret = 0;
	u32 dlstat_data, args;

	if (log_uphy->socket_id < MIN_SOCKET_ID ||
				log_uphy->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (log_uphy->link_id < MIN_LINK_ID ||
				log_uphy->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (log_uphy->lane_id > 1) {
		pr_err("Invalid lane ID specified");
		ret = -EINVAL;
		goto fail;
	}

	args = MINION_NVLINK_DL_STAT_ARGS_UPHY_ADDR(log_uphy->addr);
	args |= MINION_NVLINK_DL_STAT_ARGS_UPHY_LANEID(log_uphy->lane_id);
	ret = minion_send_dlstat(tdev, log_uphy->socket_id, log_uphy->link_id,
			NV_NVLSTAT_DB10, args, &dlstat_data);
	if (ret < 0) {
		pr_err("Error sending NV_NVLSTAT_DB10 dlstat to MINION\n");
		goto fail;
	} else {
		log_uphy->dlstat_data = dlstat_data;
		goto success;
	}

fail:
	pr_err("%s has failed", __func__);
success:
	log_uphy->status = ret;
	return ret;
}

static const u32 err_dlstat_val[] = {
	[NV_NVLSTAT_LNK1] = 0x11,
	[NV_NVLSTAT_TX09] = 0x29,
	[NV_NVLSTAT_RX00] = 0x40,
	[NV_NVLSTAT_RX01] = 0x41,
	[NV_NVLSTAT_RX02] = 0x42,
	[NV_NVLSTAT_RX11] = 0x4b,
	[NV_NVLSTAT_RX12] = 0x4c,
	[NV_NVLSTAT_RX13] = 0x4d,
	[NV_NVLSTAT_DB01] = 0x81,
	[NV_NVLSTAT_NC00] = 0xb0,
	[NV_NVLSTAT_NL00] = 0xb1,
	[NV_NVLSTAT_TS00] = 0xb2,
	[NV_NVLSTAT_TS01] = 0xb3,
	[NV_NVLSTAT_TL00] = 0xb4,
	[NV_NVLSTAT_TL01] = 0xb5,
	[NV_NVLSTAT_TL02] = 0xb6,
	[NV_NVLSTAT_TL03] = 0xb7,
	[NV_NVLSTAT_TL04] = 0xb8,
	[NV_NVLSTAT_TL05] = 0xb9,
	[NV_NVLSTAT_RL00] = 0xba,
	[NV_NVLSTAT_RL01] = 0xbb,
	[NV_NVLSTAT_RS00] = 0xbc,
	[NV_NVLSTAT_RS01] = 0xbd,
	[NV_NVLSTAT_RS02] = 0xbe,
	[NV_NVLSTAT_RS03] = 0xbf,
	[NV_NVLSTAT_MN01] = 0xc0,
};
static int err_status_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_err_status *err_status =
				(struct tegra_nvlink_err_status *)ioctl_struct;
	int ret = 0;
	u32 dlstat_data;

	if (err_status->socket_id < MIN_SOCKET_ID ||
					err_status->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (err_status->link_id < MIN_LINK_ID ||
					err_status->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (err_status->id >= INVALID_ERR_TYPE) {
		pr_err("Invalid error status id specified");
		ret = -EINVAL;
		goto fail;
	}

	ret = minion_send_dlstat(tdev, err_status->socket_id,
			err_status->link_id, err_dlstat_val[err_status->id], 0,
			&dlstat_data);
	if (ret < 0) {
		pr_err("Error sending %d dlstat to MINION\n",
			err_dlstat_val[err_status->id]);
		goto fail;
	} else {
		err_status->error_status = dlstat_data;
		goto success;
	}

fail:
	pr_err("%s has failed", __func__);
success:
	err_status->status = ret;
	return ret;
}

static const u32 clr_err_dlcmd_val[] = {
	[CLR_DLERRCNT] = 0x70,
	[CLR_DLLPCNT] = 0x71,
	[CLR_DLTHROUGHPUTCNT] = 0x72,
	[CLR_MINION_MISCCNT] = 0x73,
	[CLR_PLERROR] = 0x74,
	[CLR_PHYCTLERR] = 0x75,
	[CLR_TLC_ERRORS] = 0x24,
	[CLR_NVLIPT_ERRORS] = 0x25,
};
static int clr_err_status_ioctl(struct tegra_nvlink_dev *tdev,
			void *ioctl_struct)
{
	struct tegra_nvlink_clr_err_status *clr_err_status =
			(struct tegra_nvlink_clr_err_status *)ioctl_struct;
	int ret = 0;

	if (clr_err_status->socket_id < MIN_SOCKET_ID ||
				clr_err_status->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (clr_err_status->link_id < MIN_LINK_ID ||
				clr_err_status->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (clr_err_status->id >= CLR_INVALID_ID) {
		pr_err("Invalid clr error status id specified");
		ret = -EINVAL;
		goto fail;
	}

	ret = minion_send_dlcmd(tdev, clr_err_status->socket_id,
			clr_err_status->link_id,
			clr_err_dlcmd_val[clr_err_status->id], 0,
			false, 0);
	if (ret < 0) {
		pr_err("Error sending %d command to MINION\n",
			clr_err_dlcmd_val[clr_err_status->id]);
		goto fail;
	}

	goto success;

fail:
	pr_err("%s has failed", __func__);
success:
	clr_err_status->status = ret;
	return ret;
}

static const u32 lp_cntr_dlstat_val[] = {
	[NV_NVLSTAT_TX01] = 0x21,
	[NV_NVLSTAT_TX10] = 0x2A,
	[NV_NVLSTAT_TX02] = 0x22,
	[NV_NVLSTAT_TX06] = 0x26,
	[NV_NVLSTAT_TX05] = 0x25,
};
static int lp_cntr_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_lp_cntr *lp_cntr =
				(struct tegra_nvlink_lp_cntr *)ioctl_struct;
	int ret = 0;
	u32 dlstat_data;

	if (lp_cntr->socket_id < MIN_SOCKET_ID ||
					lp_cntr->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (lp_cntr->link_id < MIN_LINK_ID || lp_cntr->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (lp_cntr->id >= INVALID_LP_CNTR_TYPE) {
		pr_err("Invalid lp cntr id specified");
		ret = -EINVAL;
		goto fail;
	}

	ret = minion_send_dlstat(tdev, lp_cntr->socket_id, lp_cntr->link_id,
			lp_cntr_dlstat_val[lp_cntr->id], 0, &dlstat_data);
	if (ret < 0) {
		pr_err("Error sending %d dlstat to MINION\n",
			lp_cntr_dlstat_val[lp_cntr->id]);
		goto fail;
	} else {
		lp_cntr->lp_cntr_status = dlstat_data;
		goto success;
	}

fail:
	pr_err("%s has failed", __func__);
success:
	lp_cntr->status = ret;
	return ret;
}

static int do_reg_read_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_do_reg_read *reg_read =
				(struct tegra_nvlink_do_reg_read *)ioctl_struct;
	int ret = 0;
	u32 reg_val = 0, nvlw_id, link;

	if (reg_read->socket_id < MIN_SOCKET_ID ||
					reg_read->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (reg_read->link_id < MIN_LINK_ID ||
					reg_read->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}

	nvlw_id = reg_read->link_id / 6;
	link = reg_read->link_id % 6;

	switch (reg_read->id) {
	case CPR:
		reg_val = nvlw_cpr_readl(tdev, reg_read->socket_id, nvlw_id,
					reg_read->offset);
		break;
	case MINION:
		reg_val = nvlw_minion_readl(tdev, reg_read->socket_id, nvlw_id,
					reg_read->offset);
		break;
	case NVLDL:
		reg_val = nvlw_nvldl_readl(tdev, reg_read->socket_id, nvlw_id,
					link, reg_read->offset);
		break;
	case NVLIPT:
		reg_val = nvlw_nvlipt_readl(tdev, reg_read->socket_id, nvlw_id,
					reg_read->offset);
		break;
	case NVLIPT_LNK:
		reg_val = nvlw_nvlipt_lnk_readl(tdev, reg_read->socket_id,
					nvlw_id, link, reg_read->offset);
		break;
	case NVLTLC:
		reg_val = nvlw_tlc_readl(tdev, reg_read->socket_id, nvlw_id,
					link, reg_read->offset);
		break;
	case NVLW:
		reg_val = nvlw_nvlw_readl(tdev, reg_read->socket_id, nvlw_id,
					reg_read->offset);
		break;
	case CAR:
		reg_val = car_readl(tdev, reg_read->socket_id,
					reg_read->offset);
		break;
	default:
		pr_err("Invalid nvlink domain\n");
		ret = -EINVAL;
		goto fail;
	}
	goto success;

fail:
	pr_err("%s has failed", __func__);
success:
	reg_read->status = ret;
	reg_read->read_val = reg_val;
	return ret;
}

static int do_reg_write_ioctl(struct tegra_nvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_do_reg_write *reg_write =
			(struct tegra_nvlink_do_reg_write *)ioctl_struct;
	int ret = 0;
	u32 nvlw_id, link;

	if (reg_write->socket_id < MIN_SOCKET_ID ||
					reg_write->socket_id > MAX_SOCKET_ID) {
		pr_err("Invalid socket ID specified");
		ret = -EINVAL;
		goto fail;
	}
	if (reg_write->link_id < MIN_LINK_ID ||
					reg_write->link_id > MAX_LINK_ID) {
		pr_err("Invalid link ID specified");
		ret = -EINVAL;
		goto fail;
	}

	nvlw_id = reg_write->link_id / 6;
	link = reg_write->link_id % 6;

	switch (reg_write->id) {
	case CPR:
		nvlw_cpr_writel(tdev, reg_write->socket_id, nvlw_id,
				reg_write->offset, reg_write->write_val);
		break;
	case MINION:
		nvlw_minion_writel(tdev, reg_write->socket_id, nvlw_id,
				reg_write->offset, reg_write->write_val);
		break;
	case NVLDL:
		nvlw_nvldl_writel(tdev, reg_write->socket_id, nvlw_id, link,
				reg_write->offset, reg_write->write_val);
		break;
	case NVLIPT:
		nvlw_nvlipt_writel(tdev, reg_write->socket_id, nvlw_id,
				reg_write->offset, reg_write->write_val);
		break;
	case NVLIPT_LNK:
		nvlw_nvlipt_lnk_writel(tdev, reg_write->socket_id, nvlw_id,
				link, reg_write->offset, reg_write->write_val);
		break;
	case NVLTLC:
		nvlw_tlc_writel(tdev, reg_write->socket_id, nvlw_id, link,
				reg_write->offset, reg_write->write_val);
		break;
	case NVLW:
		nvlw_nvlw_writel(tdev, reg_write->socket_id, nvlw_id,
				reg_write->offset, reg_write->write_val);
		break;
	case CAR:
		car_writel(tdev, reg_write->socket_id, reg_write->offset,
				reg_write->write_val);
		break;
	default:
		pr_err("Invalid nvlink domain\n");
		ret = -EINVAL;
		goto fail;
	}
	goto success;

fail:
	pr_err("%s has failed", __func__);
success:
	reg_write->status = ret;
	return ret;
}

static long t241_nvlink_dev_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct tegra_nvlink_dev *tdev = file->private_data;
	enum tnvlink_ioctl_num ioctl_num = _IOC_NR(cmd);
	u32 ioc_dir = _IOC_DIR(cmd);
	u32 arg_size = _IOC_SIZE(cmd);
	void *arg_copy = NULL;
	int ret = 0;

	if (!tdev) {
		pr_err("Invalid Tegra nvlink device");
		return -ENODEV;
	}

	if ((_IOC_TYPE(cmd) != T241_NVLINK_IOC_MAGIC) ||
		(ioctl_num < 0) ||
		(ioctl_num >= TNVLINK_IOCTL_NUM_IOCTLS)) {
		pr_err("Unsupported IOCTL call");
		return -EINVAL;
	}

	if (arg_size != ioctls[ioctl_num].struct_size) {
		pr_err("Invalid IOCTL struct passed from userspace");
		ret = -EINVAL;
		goto fail;
	}

	/* Only allocate a buffer if the IOCTL needs a buffer */
	if (!(ioc_dir & _IOC_NONE)) {
		arg_copy = devm_kzalloc(tdev->dev, arg_size, GFP_KERNEL);
		if (!arg_copy) {
			ret = -ENOMEM;
			goto fail;
		}
	}

	if (ioc_dir & _IOC_WRITE) {
		if (copy_from_user(arg_copy, (void __user *)arg, arg_size)) {
			pr_err("copy_from_user failed!");
			ret = -EFAULT;
			goto fail;
		}
	}

	mutex_lock(&tdev->nvlink_lock);

	ret = ioctls[ioctl_num].handler(tdev, arg_copy);
	if (ret < 0) {
		mutex_unlock(&tdev->nvlink_lock);
		goto fail;
	}

	mutex_unlock(&tdev->nvlink_lock);

	if (ioc_dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, arg_copy, arg_size)) {
			pr_err("copy_to_user failed!");
			ret = -EFAULT;
			goto fail;
		}
	}

	goto cleanup;

fail:
	pr_err("The %s IOCTL failed! ret=%d", ioctls[ioctl_num].name, ret);
cleanup:
	devm_kfree(tdev->dev, arg_copy);
	return ret;
}

static int t241_nvlink_dev_open(struct inode *in, struct file *filp)
{
	int ret = 0;
	unsigned int minor = iminor(in);
	struct tegra_nvlink_dev *tdev = container_of(in->i_cdev,
					struct tegra_nvlink_dev,
					cdev);

	if (minor > 0) {
		pr_err("Incorrect minor number");
		return -EBADFD;
	}

	filp->private_data = tdev;

	return ret;
}

static ssize_t t241_nvlink_dev_read(struct file *file,
				char __user *ubuf,
				size_t count,
				loff_t *offp)
{
	return 0;
}

static int t241_nvlink_dev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* File ops for device node */
static const struct file_operations t241_nvlink_dev_ops = {
	.owner = THIS_MODULE,
	.open = t241_nvlink_dev_open,
	.read = t241_nvlink_dev_read,
	.release = t241_nvlink_dev_release,
	.unlocked_ioctl = t241_nvlink_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = t241_nvlink_dev_ioctl,
#endif
};

static void t241_clink_populate_nvlw_base(struct tegra_nvlink_dev *tdev)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		tdev->nvlw_base[i][0] = SOCKET_x_MMIO_BASE(i) + NVLW_0_BASE;
		tdev->nvlw_base[i][1] = SOCKET_x_MMIO_BASE(i) + NVLW_1_BASE;
	}
}

static int t241_nvlink_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct tegra_nvlink_dev *tdev;
	struct device *dev = NULL;

	tdev = devm_kzalloc(&pdev->dev, sizeof(struct tegra_nvlink_dev),
			GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	t241_clink_populate_nvlw_base(tdev);

	tdev->dev = &pdev->dev;
#if KERNEL_VERSION(6, 4, 0) > LINUX_VERSION_CODE
	tdev->class.owner = THIS_MODULE;
#endif
	tdev->class.name = NVLINK_MODULE_NAME;

	/* Create device node */
	ret = class_register(&tdev->class);
	if (ret) {
		pr_err("Failed to register class");
		goto fail;
	}

	ret = alloc_chrdev_region(&tdev->dev_t, 0, 1, dev_name(tdev->dev));
	if (ret) {
		pr_err("Failed to allocate dev_t");
		goto fail;
	}

	cdev_init(&tdev->cdev, &t241_nvlink_dev_ops);
	tdev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&tdev->cdev, tdev->dev_t, 1);
	if (ret) {
		pr_err("Failed to add cdev");
		goto fail;
	}

	dev = device_create(&tdev->class, NULL, tdev->dev_t, NULL,
			NVLINK_MODULE_NAME);
	if (IS_ERR(dev)) {
		pr_err("Failed to create device");
		ret = PTR_ERR(dev);
		goto fail;
	}

	mutex_init(&tdev->nvlink_lock);

	platform_set_drvdata(pdev, tdev);

	goto success;
fail:
	pr_err("Probe failed! ret=%d", ret);
success:
	return ret;
}

static int t241_nvlink_dev_remove(struct platform_device *pdev)
{
	struct tegra_nvlink_dev *tdev = platform_get_drvdata(pdev);

	mutex_destroy(&tdev->nvlink_lock);

	device_destroy(&tdev->class, tdev->dev_t);
	cdev_del(&tdev->cdev);
	unregister_chrdev_region(tdev->dev_t, 1);
	class_unregister(&tdev->class);

	devm_kfree(&pdev->dev, tdev);
	return 0;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id t241_clink_dev_acpi_match[] = {
	{.id = "NVDA2004", .driver_data = (kernel_ulong_t)NULL},
	{}
};
MODULE_DEVICE_TABLE(acpi, t241_clink_dev_acpi_match);
#endif

static struct platform_driver t241_nvlink_dev_pdrv = {
	.probe		= t241_nvlink_dev_probe,
	.remove		= t241_nvlink_dev_remove,
	.driver		= {
		.name	= NVLINK_MODULE_NAME,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(t241_clink_dev_acpi_match),
#endif
	},
};

module_platform_driver(t241_nvlink_dev_pdrv);

MODULE_ALIAS(NVLINK_MODULE_NAME);
MODULE_DESCRIPTION("T241 NVLINK Device Driver");
MODULE_LICENSE("GPL v2");
