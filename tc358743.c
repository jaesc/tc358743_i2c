/*
 * tc358743 - Toshiba HDMI to CSI-2 bridge
 *
 * Copyright 2015 Cisco Systems, Inc. and/or its affiliates. All rights
 * reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/*
 * References (c = chapter, p = page):
 * REF_01 - Toshiba, TC358743XBG (H2C), Functional Specification, Rev 0.60
 * REF_02 - Toshiba, TC358743XBG_HDMI-CSI_Tv11p_nm.xls
 */


//pll_prd = 4.5
//pll_fbd = 66

#include "tc358743.h"
#include "tc358743_regs.h"

static int debug;

#define EDID_NUM_BLOCKS_MAX 8
#define EDID_BLOCK_SIZE 128
#define I2C_MAX_XFER_SIZE  (EDID_BLOCK_SIZE + 2)
#define POLL_INTERVAL_MS	1000

/* --------------- I2C --------------- */

static void i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358743_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = values,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		v4l2_err(sd, "%s: reading register 0x%x from 0x%x failed\n",
				__func__, reg, client->addr);
	}
}

static void i2c_wr(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358743_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err, i;
	struct i2c_msg msg;
	u8 data[I2C_MAX_XFER_SIZE];

	if ((2 + n) > I2C_MAX_XFER_SIZE) {
		n = I2C_MAX_XFER_SIZE - 2;
		v4l2_warn(sd, "i2c wr reg=%04x: len=%d is too big!\n",
			  reg, 2 + n);
	}

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2 + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	for (i = 0; i < n; i++)
		data[2 + i] = values[i];

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		v4l2_err(sd, "%s: writing register 0x%x from 0x%x failed\n",
				__func__, reg, client->addr);
		return;
	}

	if (debug < 3)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x",
				reg, data[2]);
		break;
	case 2:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x",
				reg, data[3], data[2]);
		break;
	case 4:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x%02x%02x",
				reg, data[5], data[4], data[3], data[2]);
		break;
	default:
		v4l2_info(sd, "I2C write %d bytes from address 0x%04x\n",
				n, reg);
	}
}

static noinline u32 i2c_rdreg(struct v4l2_subdev *sd, u16 reg, u32 n)
{
	__le32 val = 0;

	i2c_rd(sd, reg, (u8 __force *)&val, n);

	return le32_to_cpu(val);
}

static noinline void i2c_wrreg(struct v4l2_subdev *sd, u16 reg, u32 val, u32 n)
{
	__le32 raw = cpu_to_le32(val);

	i2c_wr(sd, reg, (u8 __force *)&raw, n);
}

static u8 i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 1);
}

static void i2c_wr8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	i2c_wrreg(sd, reg, val, 1);
}

static void i2c_wr8_and_or(struct v4l2_subdev *sd, u16 reg,
		u8 mask, u8 val)
{
	i2c_wrreg(sd, reg, (i2c_rdreg(sd, reg, 1) & mask) | val, 1);
}

static u16 i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 2);
}

static void i2c_wr16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	i2c_wrreg(sd, reg, val, 2);
}

static void i2c_wr16_and_or(struct v4l2_subdev *sd, u16 reg, u16 mask, u16 val)
{
	i2c_wrreg(sd, reg, (i2c_rdreg(sd, reg, 2) & mask) | val, 2);
}

static u32 i2c_rd32(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 4);
}

static void i2c_wr32(struct v4l2_subdev *sd, u16 reg, u32 val)
{
	i2c_wrreg(sd, reg, val, 4);
}

/* --------------- STATUS --------------- */

static inline bool is_hdmi()
{
	return i2c_rd8(sd, SYS_STATUS) & MASK_S_HDMI;
}

static inline bool tx_5v_power_present()
{
	return i2c_rd8(sd, SYS_STATUS) & MASK_S_DDC5V;
}

static inline bool no_signal()
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_TMDS);
}

static inline bool no_sync()
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_SYNC);
}

static inline bool audio_present()
{
	return i2c_rd8(sd, AU_STATUS0) & MASK_S_A_SAMPLE;
}

/* --------------- HOTPLUG / HDCP / EDID --------------- */

static void tc358743_delayed_work_enable_hotplug()
{
	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0, MASK_HPD_OUT0);
}

static void tc358743_set_hdmi_hdcp(bool enable)
{
	if (enable) {
		i2c_wr8_and_or(sd, HDCP_REG3, ~KEY_RD_CMD, KEY_RD_CMD);

		i2c_wr8_and_or(sd, HDCP_MODE, ~MASK_MANUAL_AUTHENTICATION, 0);

		i2c_wr8_and_or(sd, HDCP_REG1, 0xff,
				MASK_AUTH_UNAUTH_SEL_16_FRAMES |
				MASK_AUTH_UNAUTH_AUTO);

		i2c_wr8_and_or(sd, HDCP_REG2, ~MASK_AUTO_P3_RESET,
				SET_AUTO_P3_RESET_FRAMES(0x0f));
	} else {
		i2c_wr8_and_or(sd, HDCP_MODE, ~MASK_MANUAL_AUTHENTICATION,
				MASK_MANUAL_AUTHENTICATION);
	}
}

static void tc358743_disable_edid()
{
	/* DDC access to EDID is also disabled when hotplug is disabled. See
	 * register DDC_CTL */
	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0, 0x0);
}

static void tc358743_enable_edid()
{

}

static void tc358743_erase_bksv()
{
	int i;

	for (i = 0; i < 5; i++)
		i2c_wr8(sd, BKSV + i, 0);
}

/* --------------- INIT --------------- */

static void tc358743_reset_phy()
{
	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, 0);
	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, MASK_RESET_CTRL);
}

static void tc358743_reset(uint16_t mask)
{
	u16 sysctl = i2c_rd16(sd, SYSCTL);

	i2c_wr16(sd, SYSCTL, sysctl | mask);
	i2c_wr16(sd, SYSCTL, sysctl & ~mask);
}

static inline void tc358743_sleep_mode(bool enable)
{
	i2c_wr16_and_or(sd, SYSCTL, ~MASK_SLEEP, enable ? MASK_SLEEP : 0);
}

static inline void enable_stream(bool enable)
{
	if (enable) {
		/* It is critical for CSI receiver to see lane transition
		 * LP11->HS. Set to non-continuous mode to enable clock lane
		 * LP11 state. */
		i2c_wr32(sd, TXOPTIONCNTRL, 0);
		/* Set to continuous mode to trigger LP11->HS transition */
		i2c_wr32(sd, TXOPTIONCNTRL, MASK_CONTCLKMODE);
		/* Unmute video */
		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE);
	} else {
		/* Mute video so that all data lanes go to LSP11 state.
		 * No data is output to CSI Tx block. */
		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE | MASK_VI_MUTE);
	}

	i2c_wr16_and_or(sd, CONFCTL, ~(MASK_VBUFEN | MASK_ABUFEN), enable ? (MASK_VBUFEN | MASK_ABUFEN) : 0x0);
}

static void tc358743_set_pll(u32 refclk_hz, u16 pll_prd, u16 pll_fbd)
{
	u16 pllctl0 = i2c_rd16(sd, PLLCTL0);
	u16 pllctl1 = i2c_rd16(sd, PLLCTL1);
	u16 pllctl0_new = SET_PLL_PRD(pll_prd) | SET_PLL_FBD(pll_fbd);
	u32 hsck = (refclk_hz / pll_prd) * pll_fbd;

	/* Only rewrite when needed (new value or disabled), since rewriting
	 * triggers another format change event. */
	if ((pllctl0 != pllctl0_new) || ((pllctl1 & MASK_PLL_EN) == 0)) {
		u16 pll_frs;

		if (hsck > 500000000)
			pll_frs = 0x0;
		else if (hsck > 250000000)
			pll_frs = 0x1;
		else if (hsck > 125000000)
			pll_frs = 0x2;
		else
			pll_frs = 0x3;

		tc358743_sleep_mode(true);
		i2c_wr16(sd, PLLCTL0, pllctl0_new);
		i2c_wr16_and_or(sd, PLLCTL1, ~(MASK_PLL_FRS | MASK_RESETB | MASK_PLL_EN), (SET_PLL_FRS(pll_frs) | MASK_RESETB | MASK_PLL_EN));
		udelay(10); /* REF_02, Sheet "Source HDMI" */
		i2c_wr16_and_or(sd, PLLCTL1, ~MASK_CKEN, MASK_CKEN);
		tc358743_sleep_mode(false);
	}
}

static void tc358743_set_ref_clk(u32 refclk_hz)
{
	u32 sys_freq;
	u32 lockdet_ref;
	u16 fh_min;
	u16 fh_max;

	sys_freq = refclk_hz / 10000;
	i2c_wr8(sd, SYS_FREQ0, sys_freq & 0x00ff);
	i2c_wr8(sd, SYS_FREQ1, (sys_freq & 0xff00) >> 8);

	i2c_wr8_and_or(sd, PHY_CTL0, ~MASK_PHY_SYSCLK_IND, (refclk_hz == 42000000) ? MASK_PHY_SYSCLK_IND : 0x0);

	fh_min = refclk_hz / 100000;
	i2c_wr8(sd, FH_MIN0, fh_min & 0x00ff);
	i2c_wr8(sd, FH_MIN1, (fh_min & 0xff00) >> 8);

	fh_max = (fh_min * 66) / 10;
	i2c_wr8(sd, FH_MAX0, fh_max & 0x00ff);
	i2c_wr8(sd, FH_MAX1, (fh_max & 0xff00) >> 8);

	lockdet_ref = refclk_hz / 100;
	i2c_wr8(sd, LOCKDET_REF0, lockdet_ref & 0x0000ff);
	i2c_wr8(sd, LOCKDET_REF1, (lockdet_ref & 0x00ff00) >> 8);
	i2c_wr8(sd, LOCKDET_REF2, (lockdet_ref & 0x0f0000) >> 16);

	i2c_wr8_and_or(sd, NCO_F0_MOD, ~MASK_NCO_F0_MOD, (refclk_hz == 27000000) ? MASK_NCO_F0_MOD_27MHZ : 0x0);
}

static void tc358743_set_csi_color_space()
{
	i2c_wr8_and_or(sd, VOUT_SET2,~(MASK_SEL422 | MASK_VOUT_422FIL_100) & 0xff,MASK_SEL422 | MASK_VOUT_422FIL_100);
	i2c_wr8_and_or(sd, VI_REP, ~MASK_VOUT_COLOR_SEL & 0xff,MASK_VOUT_COLOR_601_YCBCR_LIMITED);
	i2c_wr16_and_or(sd, CONFCTL, ~MASK_YCBCRFMT,MASK_YCBCRFMT_422_8_BIT);
}

static unsigned tc358743_num_csi_lanes_needed()
{
	return 4;
}

static void tc358743_set_csi()
{
	unsigned lanes = tc358743_num_csi_lanes_needed();

	tc358743_reset(MASK_CTXRST);

	if (lanes < 1)
		i2c_wr32(sd, CLW_CNTRL, MASK_CLW_LANEDISABLE);
	if (lanes < 1)
		i2c_wr32(sd, D0W_CNTRL, MASK_D0W_LANEDISABLE);
	if (lanes < 2)
		i2c_wr32(sd, D1W_CNTRL, MASK_D1W_LANEDISABLE);
	if (lanes < 3)
		i2c_wr32(sd, D2W_CNTRL, MASK_D2W_LANEDISABLE);
	if (lanes < 4)
		i2c_wr32(sd, D3W_CNTRL, MASK_D3W_LANEDISABLE);

	i2c_wr32(sd, LINEINITCNT, 0xe80);
	i2c_wr32(sd, LPTXTIMECNT, 0x003);
	i2c_wr32(sd, TCLK_HEADERCNT, 0x1403);
	i2c_wr32(sd, TCLK_TRAILCNT, 0x00);
	i2c_wr32(sd, THS_HEADERCNT, 0x0103);
	i2c_wr32(sd, TWAKEUP, 0x4882);
	i2c_wr32(sd, TCLK_POSTCNT, 0x008);
	i2c_wr32(sd, THS_TRAILCNT, 0x2);
	i2c_wr32(sd, HSTXVREGCNT, 0);

	i2c_wr32(sd, HSTXVREGEN,
			((lanes > 0) ? MASK_CLM_HSTXVREGEN : 0x0) |
			((lanes > 0) ? MASK_D0M_HSTXVREGEN : 0x0) |
			((lanes > 1) ? MASK_D1M_HSTXVREGEN : 0x0) |
			((lanes > 2) ? MASK_D2M_HSTXVREGEN : 0x0) |
			((lanes > 3) ? MASK_D3M_HSTXVREGEN : 0x0));

	i2c_wr32(sd, TXOPTIONCNTRL, 0);
	i2c_wr32(sd, STARTCNTRL, MASK_START);
	i2c_wr32(sd, CSI_START, MASK_STRT);

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_SET |
			MASK_ADDRESS_CSI_CONTROL |
			MASK_CSI_MODE |
			MASK_TXHSMD |
			((lanes == 4) ? MASK_NOL_4 :
			 (lanes == 3) ? MASK_NOL_3 :
			 (lanes == 2) ? MASK_NOL_2 : MASK_NOL_1));

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_SET |
			MASK_ADDRESS_CSI_ERR_INTENA | MASK_TXBRK | MASK_QUNK |
			MASK_WCER | MASK_INER);

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_CLEAR |
			MASK_ADDRESS_CSI_ERR_HALT | MASK_TXBRK | MASK_QUNK);

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_SET |
			MASK_ADDRESS_CSI_INT_ENA | MASK_INTER);
}

static void tc358743_set_hdmi_phy()
{
	/* Default settings from REF_02, sheet "Source HDMI"
	 * and custom settings as platform data */
	i2c_wr8_and_or(sd, PHY_EN, ~MASK_ENABLE_PHY, 0x0);
	i2c_wr8(sd, PHY_CTL1, SET_PHY_AUTO_RST1_US(1600) | SET_FREQ_RANGE_MODE_CYCLES(1));
    i2c_wr8_and_or(sd, PHY_CTL2, ~MASK_PHY_AUTO_RSTn, 0);
	/*i2c_wr8_and_or(sd, PHY_CTL2, ~MASK_PHY_AUTO_RSTn,
			(pdata->hdmi_phy_auto_reset_tmds_detected ?
			 MASK_PHY_AUTO_RST2 : 0) |
			(pdata->hdmi_phy_auto_reset_tmds_in_range ?
			 MASK_PHY_AUTO_RST3 : 0) |
			(pdata->hdmi_phy_auto_reset_tmds_valid ?
			 MASK_PHY_AUTO_RST4 : 0));*/
	i2c_wr8(sd, PHY_BIAS, 0x40);
	i2c_wr8(sd, PHY_CSQ, SET_CSQ_CNT_LEVEL(0x0a));
	i2c_wr8(sd, AVM_CTL, 45);
	i2c_wr8_and_or(sd, HDMI_DET, ~MASK_HDMI_DET_V, 0 << 4);
	i2c_wr8_and_or(sd, HV_RST, ~(MASK_H_PI_RST | MASK_V_PI_RST), 0);
    /*i2c_wr8_and_or(sd, HV_RST, ~(MASK_H_PI_RST | MASK_V_PI_RST),
			(pdata->hdmi_phy_auto_reset_hsync_out_of_range ?
			 MASK_H_PI_RST : 0) |
			(pdata->hdmi_phy_auto_reset_vsync_out_of_range ?
			 MASK_V_PI_RST : 0));*/
	i2c_wr8_and_or(sd, PHY_EN, ~MASK_ENABLE_PHY, MASK_ENABLE_PHY);
}

static void tc358743_set_hdmi_audio()
{
	/* Default settings from REF_02, sheet "Source HDMI" */
	i2c_wr8(sd, FORCE_MUTE, 0x00);
	i2c_wr8(sd, AUTO_CMD0, MASK_AUTO_MUTE7 | MASK_AUTO_MUTE6 |
			MASK_AUTO_MUTE5 | MASK_AUTO_MUTE4 |
			MASK_AUTO_MUTE1 | MASK_AUTO_MUTE0);
	i2c_wr8(sd, AUTO_CMD1, MASK_AUTO_MUTE9);
	i2c_wr8(sd, AUTO_CMD2, MASK_AUTO_PLAY3 | MASK_AUTO_PLAY2);
	i2c_wr8(sd, BUFINIT_START, SET_BUFINIT_START_MS(500));
	i2c_wr8(sd, FS_MUTE, 0x00);
	i2c_wr8(sd, FS_IMODE, MASK_NLPCM_SMODE | MASK_FS_SMODE);
	i2c_wr8(sd, ACR_MODE, MASK_CTS_MODE);
	i2c_wr8(sd, ACR_MDF0, MASK_ACR_L2MDF_1976_PPM | MASK_ACR_L1MDF_976_PPM);
	i2c_wr8(sd, ACR_MDF1, MASK_ACR_L3MDF_3906_PPM);
	i2c_wr8(sd, SDO_MODE1, MASK_SDO_FMT_I2S);
	i2c_wr8(sd, DIV_MODE, SET_DIV_DLY_MS(100));
	i2c_wr16_and_or(sd, CONFCTL, 0xffff, MASK_AUDCHNUM_2 |
			MASK_AUDOUTSEL_I2S | MASK_AUTOINDEX);
}

static void tc358743_set_hdmi_info_frame_mode()
{
	/* Default settings from REF_02, sheet "Source HDMI" */
	i2c_wr8(sd, PK_INT_MODE, MASK_ISRC2_INT_MODE | MASK_ISRC_INT_MODE |
			MASK_ACP_INT_MODE | MASK_VS_INT_MODE |
			MASK_SPD_INT_MODE | MASK_MS_INT_MODE |
			MASK_AUD_INT_MODE | MASK_AVI_INT_MODE);
	i2c_wr8(sd, NO_PKT_LIMIT, 0x2c);
	i2c_wr8(sd, NO_PKT_CLR, 0x53);
	i2c_wr8(sd, ERR_PK_LIMIT, 0x01);
	i2c_wr8(sd, NO_PKT_LIMIT2, 0x30);
	i2c_wr8(sd, NO_GDB_LIMIT, 0x10);
}

static void tc358743_initial_setup()
{
	/* CEC and IR are not supported by this driver */
	i2c_wr16_and_or(sd, SYSCTL, ~(MASK_CECRST | MASK_IRRST),
			(MASK_CECRST | MASK_IRRST));

	tc358743_reset(MASK_CTXRST | MASK_HDMIRST);
	tc358743_sleep_mode(false);

	i2c_wr16(sd, FIFOCTL, 16);

	tc358743_set_ref_clk(27000000);

	i2c_wr8_and_or(sd, DDC_CTL, ~MASK_DDC5V_MODE, 2 & MASK_DDC5V_MODE);
	i2c_wr8_and_or(sd, EDID_MODE, ~MASK_EDID_MODE, MASK_EDID_MODE_E_DDC);

	tc358743_set_hdmi_phy();
	tc358743_set_hdmi_hdcp(false);
	tc358743_set_hdmi_audio();
	tc358743_set_hdmi_info_frame_mode();
    tc358743_disable_edid();

	/* All CE and IT formats are detected as RGB full range in DVI mode */
	i2c_wr8_and_or(sd, VI_MODE, ~MASK_RGB_DVI, 0);

	i2c_wr8_and_or(sd, VOUT_SET2, ~MASK_VOUTCOLORMODE, MASK_VOUTCOLORMODE_AUTO);
	i2c_wr8(sd, VOUT_SET3, MASK_VOUT_EXTCNT);
}

/* --------------- IRQ --------------- */

static void tc358743_init_interrupts()
{
	u16 i;

	/* clear interrupt status registers */
	for (i = SYS_INT; i <= KEY_INT; i++)
		i2c_wr8(sd, i, 0xff);

	i2c_wr16(sd, INTSTATUS, 0xffff);
}

static void tc358743_enable_interrupts(bool cable_connected)
{
	if (cable_connected) {
		i2c_wr8(sd, SYS_INTM, ~(MASK_M_DDC | MASK_M_DVI_DET |
					MASK_M_HDMI_DET) & 0xff);
		i2c_wr8(sd, CLK_INTM, ~MASK_M_IN_DE_CHG);
		i2c_wr8(sd, CBIT_INTM, ~(MASK_M_CBIT_FS | MASK_M_AF_LOCK |
					MASK_M_AF_UNLOCK) & 0xff);
		i2c_wr8(sd, AUDIO_INTM, ~MASK_M_BUFINIT_END);
		i2c_wr8(sd, MISC_INTM, ~MASK_M_SYNC_CHG);
	} else {
		//i2c_wr8(sd, SYS_INTM, ~MASK_M_DDC & 0xff);
        i2c_wr8(sd, SYS_INTM, 0xff);
		i2c_wr8(sd, CLK_INTM, 0xff);
		i2c_wr8(sd, CBIT_INTM, 0xff);
		i2c_wr8(sd, AUDIO_INTM, 0xff);
		i2c_wr8(sd, MISC_INTM, 0xff);
	}
}

static int tc358743_probe()
{
	tc358743_initial_setup();
	enable_stream(false);
	tc358743_set_pll(27000000,4.5,66);   //tc358743_set_pll(u32 refclk_hz, u16 pll_prd, u16 pll_fbd)
	tc358743_set_csi();
	tc358743_set_csi_color_space();
	tc358743_init_interrupts();

	//tc358743_enable_interrupts(tx_5v_power_present());
    tc358743_enable_interrupts(false);

	i2c_wr16(sd, INTMASK, ~(MASK_HDMI_MSK | MASK_CSI_MSK) & 0xffff);

    enable_stream(true);
	return 0;
}
