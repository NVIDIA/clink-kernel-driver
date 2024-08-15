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

#ifndef CLINK_T241_CLINK_HW_H
#define CLINK_T241_CLINK_HW_H

#define SOCKET_x_MMIO_BASE(x)		(x * 0x10 * 0x10000000000)

#define NVLW_0_BASE				0x03b80000
#define NVLW_1_BASE				0x03bc0000
#define CAR_BASE				0x20000000

#define MINION_BASE				0x4000
#define CPR_BASE				0x2800
#define NVLIPT_BASE				0x2000

#define NVLIPT_LINK_1_BASE			0x1F000
#define NVLIPT_LINK_0_BASE			0x17000
#define NVLIPT_LINK_BASE_STRIDE		(NVLIPT_LINK_1_BASE - \
						NVLIPT_LINK_0_BASE)
#define NVLIPT_LINK_BASE(link)		(NVLIPT_LINK_0_BASE + \
						(link * \
						NVLIPT_LINK_BASE_STRIDE))

#define TLC_LINK_0_BASE				0x15000
#define TLC_LINK_1_BASE				0x1D000
#define TLC_LINK_BASE_STRIDE		(TLC_LINK_1_BASE - TLC_LINK_0_BASE)
#define TLC_LINK_BASE(link)		(TLC_LINK_0_BASE + \
						(link * TLC_LINK_BASE_STRIDE))

#define NVLDL_LINK_0_BASE			0x10000
#define NVLDL_LINK_1_BASE			0x18000
#define NVLDL_LINK_BASE_STRIDE		(NVLDL_LINK_1_BASE - NVLDL_LINK_0_BASE)
#define NVLDL_LINK_BASE(link)		(NVLDL_LINK_0_BASE + \
						(link * NVLDL_LINK_BASE_STRIDE))

/* MIONION */
#define MINION_MISC_0					0x28b0

/* <i> belongs to <0..5> */
#define MINION_NVLINK_DL_CMD_DATA(i)			(0x2920 + (i * 0x4))
#define MINION_NVLINK_DL_CMD_DATA_ENABLE		0
#define MINION_NVLINK_DL_CMD_DATA_RESET			1
#define MINION_NVLINK_DL_CMD_DATA_HWRESET		2
#define MINION_NVLINK_DL_CMD_DATA_EXP_F(x)		(((x) & 0x3f) << 3)
#define MINION_NVLINK_DL_CMD_DATA_EXP_NOP		0
#define MINION_NVLINK_DL_CMD_DATA_EXP_BYTES_COUNT	1
#define MINION_NVLINK_DL_CMD_DATA_EXP_CYCLES_COUNT	2
#define MINION_NVLINK_DL_CMD_DATA_EXP_PACKETS_COUNT	3
#define MINION_NVLINK_DL_CMD_DATA_EXP_FLITS_COUNT	4
#define MINION_NVLINK_DL_CMD_DATA_TX_TC_F(x)		(((x) & 0xf) << 16)
#define MINION_NVLINK_DL_CMD_DATA_RX_TC_F(x)		(((x) & 0xf) << 24)

#define MINION_NVLINK_DL_CMD(i)				(0x2900 + (i * 0x4))
#define MINION_NVLINK_DL_CMD_COMMAND_MASK		0xff
#define MINION_NVLINK_DL_CMD_COMMAND_SHIFT		0
#define MINION_NVLINK_DL_CMD_COMMAND_CONFIGEOM		0x40
#define MINION_NVLINK_DL_CMD_COMMAND_STARTEOM		0x44
#define MINION_NVLINK_DL_CMD_COMMAND_ENDEOM		0x45
#define MINION_NVLINK_DL_CMD_COMMAND_SETUPTC		0x94
#define MINION_NVLINK_DL_CMD_COMMAND_CAPTURETC		0x95
#define MINION_NVLINK_DL_CMD_FAULT			30
#define MINION_NVLINK_DL_CMD_READY			31

#define MINION_NVLINK_DL_STAT(i)			(0x2980 + (i * 0x4))
#define MINION_NVLINK_DL_STAT_ARGS(x)			(((x) & 0xffff) << 0)
#define MINION_NVLINK_DL_STAT_ARGS_TC_TX		1
#define MINION_NVLINK_DL_STAT_ARGS_TC_RX		0
#define MINION_NVLINK_DL_STAT_ARGS_TC_COUNTER(x)	(((x) & 0x3) << 1)
#define MINION_NVLINK_DL_STAT_ARGS_UPHY_ADDR(x)	(((x) & 0xfff) << 0)
#define MINION_NVLINK_DL_STAT_ARGS_UPHY_LANEID(x)	(((x) & 0xf) << 12)
#define MINION_NVLINK_DL_STAT_STATUSIDX(x)		(((x) & 0xff) << 16)
#define MINION_NVLINK_DL_STAT_READY			31

#define NV_NVLSTAT_TC00					0xd0
#define NV_NVLSTAT_TC01					0xd1
#define NV_NVLSTAT_DB10					0x8a

#define MINION_NVLINK_DL_STATDATA(i)			(0x29c0 + (i * 0x4))

/* NVLPHYCTL */
#define NVLPHYCTL_LANE_PAD_CTL_4_0			0x2a0c
#define NVLPHYCTL_LANE_PAD_CTL_4_0_RX_EOM_DONE		9
#define NVLPHYCTL_LANE_PAD_CTL_4_0_RX_EOM_STATUS_V(x)	(((x) >> 16) & 0xffff)

#define NVLPHYCTL_LANE_PAD_CTL_4_1			0x2b0c
#define NVLPHYCTL_LANE_PAD_CTL_4_1_RX_EOM_DONE		9
#define NVLPHYCTL_LANE_PAD_CTL_4_1_RX_EOM_STATUS_V(x)	(((x) >> 16) & 0xffff)

#endif /* CLINK_T241_CLINK_HW_H */
