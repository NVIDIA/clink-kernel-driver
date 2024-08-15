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

#ifndef CLINK_T241_CLINK_IOCTL_H
#define CLINK_T241_CLINK_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

enum tnvlink_ioctl_num {
	TNVLINK_IOCTL_SEND_DLCMD,
	TNVLINK_IOCTL_SEND_DLSTAT,
	TNVLINK_IOCTL_LNK_STATUS,
	TNVLINK_IOCTL_GET_EOM_STATUS,
	TNVLINK_IOCTL_SETUP_TC,
	TNVLINK_IOCTL_LOG_UPHY,
	TNVLINK_IOCTL_ERR_STATUS,
	TNVLINK_IOCTL_CLR_ERR_STATUS,
	TNVLINK_IOCTL_LP_CNTR,
	TNVLINK_IOCTL_DO_REG_READ,
	TNVLINK_IOCTL_DO_REG_WRITE,
	TNVLINK_IOCTL_NUM_IOCTLS,
};

/* T241_CTRL_CMD_NVLINK_SEND_DLCMD */
struct tegra_nvlink_send_dlcmd {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	__u32 dlcmd;
	bool use_dlcmd_data;
	__u32 dlcmd_data;

	/* o/p */
	__s32 status;
};

/* T241_CTRL_CMD_NVLINK_SEND_DLSTAT */
struct tegra_nvlink_send_dlstat {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	__u32 dlstat_id;
	__u32 args;
	/* o/p */
	__s32 status;
	__u32 dlstat_data;
};

enum lnk_stat_type {
	NV_NVLSTAT_LNK2,
	NV_NVLSTAT_LNK3,
	NV_NVLSTAT_LNK4,

	NV_NVLSTAT_LNK5,

	INVALID_LNK_STAT_TYPE,
};

/* T241_CTRL_CMD_NVLINK_LNK_STATUS */
struct tegra_nvlink_lnk_status {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	enum lnk_stat_type id;

	/* o/p */
	__s32 status;
	__u32 lnk_status;
};

/* T241_CTRL_CMD_NVLINK_GET_EOM_STATUS */
struct tegra_nvlink_get_eom {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	__u32 params;
	/* o/p */
	__s32 status;
	__u16 l0_rx_eom_status;
	__u16 l1_rx_eom_status;
};

enum tnvlink_tc_exp {
	STOP_TC,
	RESET_TC,
	HWRESET_TC,
	SETUP_NOP,
	SETUP_BYTES_COUNT,
	SETUP_CYCLES_COUNT,
	SETUP_PACKETS_COUNT,
	SETUP_FLITS_COUNT,
	START_TC,
	CAPTURE_TC,
	READ_TC,
	NUM_TC_EXP,
};

/* T241_CTRL_CMD_NVLINK_SETUP_TC */
struct tegra_nvlink_setup_tc {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;

	__u8 counter_mask;

	enum tnvlink_tc_exp exp;

	/* o/p */
	__s32 status;

	__u64 tx_counter[4];
	__u64 rx_counter[4];
};

/* T241_CTRL_CMD_NVLINK_LOG_UPHY */
struct tegra_nvlink_log_uphy {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	__u8 lane_id;
	__u32 addr;
	/* o/p */
	__s32 status;
	__u32 dlstat_data;
};

enum err_type {
	NV_NVLSTAT_LNK1,
	NV_NVLSTAT_TX09,
	NV_NVLSTAT_RX00,
	NV_NVLSTAT_RX01,
	NV_NVLSTAT_RX02,
	NV_NVLSTAT_RX11,
	NV_NVLSTAT_RX12,
	NV_NVLSTAT_RX13,
	NV_NVLSTAT_DB01,

	NV_NVLSTAT_NC00,
	NV_NVLSTAT_NL00,

	NV_NVLSTAT_TS00,
	NV_NVLSTAT_TS01,
	NV_NVLSTAT_TL00,
	NV_NVLSTAT_TL01,
	NV_NVLSTAT_TL02,
	NV_NVLSTAT_TL03,
	NV_NVLSTAT_TL04,
	NV_NVLSTAT_TL05,

	NV_NVLSTAT_RL00,
	NV_NVLSTAT_RL01,
	NV_NVLSTAT_RS00,
	NV_NVLSTAT_RS01,
	NV_NVLSTAT_RS02,
	NV_NVLSTAT_RS03,

	NV_NVLSTAT_MN01,

	INVALID_ERR_TYPE,
};

/* T241_CTRL_CMD_NVLINK_ERR_STATUS */
struct tegra_nvlink_err_status {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	enum err_type id;

	/* o/p */
	__s32 status;
	__u32 error_status;
};

enum clr_err_type {
	CLR_DLERRCNT,
	CLR_DLLPCNT,
	CLR_DLTHROUGHPUTCNT,
	CLR_MINION_MISCCNT,
	CLR_PLERROR,
	CLR_PHYCTLERR,
	CLR_TLC_ERRORS,
	CLR_NVLIPT_ERRORS,
	CLR_INVALID_ID,
};

/* T241_CTRL_CMD_NVLINK_CLR_ERR_STATUS */
struct tegra_nvlink_clr_err_status {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	enum clr_err_type id;

	/* o/p */
	__s32 status;
};

enum lp_cntr_type {
	NV_NVLSTAT_TX01,

	NV_NVLSTAT_TX10,

	NV_NVLSTAT_TX02,

	NV_NVLSTAT_TX06,

	NV_NVLSTAT_TX05,

	INVALID_LP_CNTR_TYPE,
};

/* T241_CTRL_CMD_NVLINK_LP_CNTR */
struct tegra_nvlink_lp_cntr {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	enum lp_cntr_type id;

	/* o/p */
	__s32 status;
	__u32 lp_cntr_status;
};

enum nvlink_domain {
	CPR,
	MINION,
	NVLDL,
	NVLIPT,
	NVLIPT_LNK,
	NVLTLC,
	NVLW,

	CAR,
};

/* T241_CTRL_CMD_NVLINK_DO_REG_READ */
struct tegra_nvlink_do_reg_read {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	enum nvlink_domain id;
	__u32 offset;

	/* o/p */
	__s32 status;
	__u32 read_val;
};

/* T241_CTRL_CMD_NVLINK_DO_REG_WRITE */
struct tegra_nvlink_do_reg_write {
	/* i/p */
	__u8 socket_id;
	__u8 link_id;
	enum nvlink_domain id;
	__u32 offset;
	__u32 write_val;

	/* o/p */
	__s32 status;
};

#define T241_NVLINK_IOC_MAGIC	  'T'
#define	T241_CTRL_CMD_NVLINK_SEND_DLCMD			\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_SEND_DLCMD,	\
				struct tegra_nvlink_send_dlcmd)
#define	T241_CTRL_CMD_NVLINK_SEND_DLSTAT			\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_SEND_DLSTAT,	\
				struct tegra_nvlink_send_dlstat)
#define	T241_CTRL_CMD_NVLINK_LNK_STATUS			\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_LNK_STATUS,	\
				struct tegra_nvlink_lnk_status)
#define	T241_CTRL_CMD_NVLINK_GET_EOM_STATUS			\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_GET_EOM_STATUS,	\
				struct tegra_nvlink_get_eom)
#define	T241_CTRL_CMD_NVLINK_SETUP_TC				\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_SETUP_TC,		\
				struct tegra_nvlink_setup_tc)
#define	T241_CTRL_CMD_NVLINK_LOG_UPHY				\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_LOG_UPHY,		\
				struct tegra_nvlink_log_uphy)
#define	T241_CTRL_CMD_NVLINK_ERR_STATUS			\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_ERR_STATUS,	\
				struct tegra_nvlink_err_status)
#define	T241_CTRL_CMD_NVLINK_CLR_ERR_STATUS			\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_CLR_ERR_STATUS,	\
				struct tegra_nvlink_clr_err_status)
#define	T241_CTRL_CMD_NVLINK_LP_CNTR				\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_LP_CNTR,		\
				struct tegra_nvlink_lp_cntr)
#define	T241_CTRL_CMD_NVLINK_DO_REG_READ			\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_DO_REG_READ,	\
				struct tegra_nvlink_do_reg_read)
#define	T241_CTRL_CMD_NVLINK_DO_REG_WRITE			\
			_IOWR(T241_NVLINK_IOC_MAGIC,		\
				TNVLINK_IOCTL_DO_REG_WRITE,	\
				struct tegra_nvlink_do_reg_write)

#endif /* CLINK_T241_CLINK_IOCTL_H */
