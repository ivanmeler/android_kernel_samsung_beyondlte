/*
 * driver/muic/s2mu004.c - S2MU004 micro USB switch device driver
 *
 * Copyright (C) 2017 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#define pr_fmt(fmt)	"[MUIC] " fmt

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/muic/muic_core.h>
#include <linux/mfd/samsung/s2mu004.h>
#include <linux/mfd/samsung/s2mu004-private.h>
#include <linux/muic/s2mu004-muic.h>
#include <linux/muic/s2mu004-muic-sysfs.h>

#include <linux/power_supply.h>
#include <linux/muic/s2mu004-muic-hv.h>

#include <linux/muic/s2mu004-muic-notifier.h>
#include <linux/usb_notify.h>
#include <linux/muic/muic_interface.h>

#if IS_ENABLED(CONFIG_CCIC_NOTIFIER)
#include <linux/ccic/ccic_notifier.h>
#endif
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
#include <linux/vbus_notifier.h>
#endif
#if IS_ENABLED(CONFIG_MUIC_UART_SWITCH)
#include <mach/pinctrl-samsung.h>
#endif
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
#include <soc/samsung/exynos-modem-ctrl.h>
#endif

static const char *dev_to_str(muic_attached_dev_t n)
{
	char *ret;

	switch (n) {
	ENUM_STR(ATTACHED_DEV_NONE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_USB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CDP_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_OTG_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_ANY_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_USB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_CDP_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNDEFINED_CHARGING_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_DESKDOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNKNOWN_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_DESKDOCK_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CARDOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_VB_FG_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_ON_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_ON_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_USB_OFF_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_USB_ON_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_RID_OPEN_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_USB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNIVERSAL_MMDOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AUDIODOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_MHL_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CHARGING_CABLE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_PREPARE_DUPLI_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_5V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_5V_DUPLI_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_9V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_ERR_V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_ERR_V_DUPLI_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_PREPARE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_5V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_ERR_V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_9V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_HV_ID_ERR_UNDEFINED_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_HV_ID_ERR_UNSUPPORTED_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_HV_ID_ERR_SUPPORTED_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_HMT_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_VZW_ACC_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_VZW_INCOMPATIBLE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_USB_LANHUB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TYPE2_CHG_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TYPE3_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TYPE3_MUIC_TA, ret);
	ENUM_STR(ATTACHED_DEV_TYPE3_ADAPTER_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TYPE3_CHARGER_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_NONE_TYPE3_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNSUPPORTED_ID_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNSUPPORTED_ID_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TIMEOUT_OPEN_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_WIRELESS_PAD_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_POWERPACK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNDEFINED_RANGE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_WATER_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CHK_WATER_REQ, ret);
	ENUM_STR(ATTACHED_DEV_CHK_WATER_DRY_REQ, ret);
	ENUM_STR(ATTACHED_DEV_GAMEPAD_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CHECK_OCP, ret);
	ENUM_STR(ATTACHED_DEV_RDU_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_9V_DUPLI_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNKNOWN_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_NUM, ret);
	default:
		return "invalid";
	}
	return ret;
}

static struct s2mu004_muic_data *static_data;

static int s2mu004_muic_set_com_sw(struct s2mu004_muic_data *muic_data,
		u8 reg_val);
static void s2mu004_muic_detect_dev_ccic(struct s2mu004_muic_data *muic_data,
		muic_attached_dev_t new_dev);
#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
static int s2mu004_i2c_update_bit(struct i2c_client *client,
				  u8 reg, u8 mask, u8 shift, u8 value);
#endif /* CONFIG_MUIC_S2MU004_RID */

#if IS_ENABLED(DEBUG_MUIC)
#define MAX_LOG 25
#define READ 0
#define WRITE 1

static u8 s2mu004_log_cnt;
static u8 s2mu004_log[MAX_LOG][3];

static void s2mu004_reg_log(u8 reg, u8 value, u8 rw)
{
	s2mu004_log[s2mu004_log_cnt][0] = reg;
	s2mu004_log[s2mu004_log_cnt][1] = value;
	s2mu004_log[s2mu004_log_cnt][2] = rw;
	s2mu004_log_cnt++;
	if (s2mu004_log_cnt >= MAX_LOG)
		s2mu004_log_cnt = 0;
}

static void s2mu004_print_reg_log(void)
{
	int i;
	u8 reg, value, rw;
	char mesg[256] = "";

	for (i = 0; i < MAX_LOG; i++) {
		reg = s2mu004_log[s2mu004_log_cnt][0];
		value = s2mu004_log[s2mu004_log_cnt][1];
		rw = s2mu004_log[s2mu004_log_cnt][2];
		s2mu004_log_cnt++;

		if (s2mu004_log_cnt >= MAX_LOG)
			s2mu004_log_cnt = 0;
		sprintf(mesg + strlen(mesg), "%x(%x)%x ", reg, value, rw);
	}
	pr_info("%s %s\n", __func__, mesg);
}

void s2mu004_read_reg_dump(struct s2mu004_muic_data *muic, char *mesg)
{
	u8 val;

	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_CTRL1, &val);
	sprintf(mesg + strlen(mesg), "CTRL1:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_SW_CTRL, &val);
	sprintf(mesg + strlen(mesg), "SW_CTRL:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_INT1_MASK, &val);
	sprintf(mesg + strlen(mesg), "IM1:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_INT2_MASK, &val);
	sprintf(mesg + strlen(mesg), "IM2:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_CHG_TYPE, &val);
	sprintf(mesg + strlen(mesg), "CHG_T:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_DEVICE_APPLE, &val);
	sprintf(mesg + strlen(mesg), "APPLE_DT:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_ADC, &val);
	sprintf(mesg + strlen(mesg), "ADC:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_DEVICE_TYPE1, &val);
	sprintf(mesg + strlen(mesg), "DT1:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_DEVICE_TYPE2, &val);
	sprintf(mesg + strlen(mesg), "DT2:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_DEVICE_TYPE3, &val);
	sprintf(mesg + strlen(mesg), "DT3:%x ", val);
}

void s2mu004_print_reg_dump(struct s2mu004_muic_data *muic_data)
{
	char mesg[256] = "";

	s2mu004_read_reg_dump(muic_data, mesg);

	pr_info("%s %s\n", __func__, mesg);
}
#endif

/* interface functions */
static int s2mu004_if_com_to_open(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_com_to_open(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_com_to_audio(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_com_to_audio(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_com_to_otg(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_com_to_otg(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
static int s2mu004_if_get_adc(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_recheck_adc(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret & 0x1F;
}
#endif

static int s2mu004_if_switch_to_usb(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	ret = s2mu004_muic_com_to_usb(muic_data);
	if (ret)
		pr_err("%s set_com_usb err\n", __func__);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_switch_to_uart(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_com_to_uart(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_get_vbus(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_get_vbus_state(muic_data);
}

static void s2mu004_if_set_cable_state(void *mdata, muic_attached_dev_t new_dev)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	s2mu004_muic_detect_dev_ccic(muic_data, new_dev);

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);
}

static void s2mu004_if_set_otg_detect_en(void *mdata, bool en)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	s2mu004_muic_control_vbus_det(muic_data, en);
}

static void s2mu004_if_dcd_rescan(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	muic_data->is_dcd_recheck = true;
	s2mu004_muic_dcd_rescan(muic_data);

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);
}

static int s2mu004_if_bcd_rescan(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_bcd_rescan(muic_data);
}

static int s2mu004_if_control_rid_adc(void *mdata, bool enable)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_control_rid_adc(muic_data, enable);
}

#if defined(CONFIG_MUIC_S2MU004_HV)
static int s2mu004_if_set_afc_reset(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;
	struct muic_interface_t *muic_if = muic_data->if_data;

	pr_info("%s\n", __func__);
	muic_if->is_afc_reset = true;

	return 0;
}

static muic_attached_dev_t s2mu004_if_check_id_err(void *mdata,
						   muic_attached_dev_t new_dev)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	return hv_muic_check_id_err(muic_data, new_dev);
}

static int s2mu004_if_reset_hvcontrol_reg(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	s2mu004_hv_muic_reset_hvcontrol_reg(muic_data);

	return 0;
}

static int s2mu004_if_check_afc_ready(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_check_afc_ready(muic_data);
}

static int s2mu004_if_reset_afc_register(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_reset_afc_register(muic_data);
}
#endif

static int s2mu004_if_jig_on(void *mdata)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_jig_on(muic_data);
}

static int s2mu004_if_set_gpio_uart_sel(void *mdata, int uart_path)
{
	struct s2mu004_muic_data *muic_data = (struct s2mu004_muic_data *)mdata;

	return s2mu004_set_gpio_uart_sel(muic_data, uart_path);
}

int s2mu004_i2c_read_byte(struct i2c_client *client, u8 command)
{
	int ret = 0;
	int retry = 0;
	u8 data = 0;

	ret = s2mu004_read_reg(client, command, &data);

	while (ret < 0) {
		pr_info("failed to read reg(0x%x) retry(%d)\n", command, retry);
		if (retry > 10) {
			pr_err("%s  retry failed!!\n", __func__);
			break;
		}
		msleep(100);
		ret = s2mu004_read_reg(client, command, &data);
		retry++;
	}

#if IS_ENABLED(DEBUG_MUIC)
	s2mu004_reg_log(command, ret, retry << 1 | READ);
#endif
	return data;
}

int s2mu004_i2c_write_byte(struct i2c_client *client, u8 command, u8 value)
{
	int ret_r = 0;
	int ret_w = 0;
	int retry = 0;
	u8 written = 0;

	ret_w = s2mu004_write_reg(client, command, value);

	while (ret_w < 0) {
		pr_info("failed to write reg(0x%x) retry(%d)\n", command, retry);
		ret_r = s2mu004_read_reg(client, command, &written);
		if (ret_r < 0)
			pr_err("%s reg(0x%x)\n", __func__, command);
		msleep(100);
		ret_w = s2mu004_write_reg(client, command, value);
		retry++;
	}
#if IS_ENABLED(DEBUG_MUIC)
	s2mu004_reg_log(command, value, retry << 1 | WRITE);
#endif
	return ret_w;
}

static int s2mu004_i2c_guaranteed_wbyte(struct i2c_client *client,
					u8 command, u8 value)
{
	int ret;
	int retry = 0;
	int written;

	ret = s2mu004_i2c_write_byte(client, command, value);
	written = s2mu004_i2c_read_byte(client, command);
	while (written != value) {
		pr_info("reg(0x%x): written(0x%x) != value(0x%x)\n",
			command, written, value);
		if (retry > 10) {
			pr_err("%s  retry failed!!\n", __func__);
			break;
		}
		msleep(100);
		retry++;
		ret = s2mu004_i2c_write_byte(client, command, value);
		written = s2mu004_i2c_read_byte(client, command);
	}
	return ret;
}

int s2mu004_muic_control_rid_adc(struct s2mu004_muic_data *muic_data, bool enable)
{
	int data = 0;

	pr_info("%s (%s)\n", __func__, enable ? "Enable" : "Disable");

	data = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_RID_CTRL);

	if (enable)
		data &= ~MUIC_CTRL2_ADC_OFF_MASK;
	else
		data |= MUIC_CTRL2_ADC_OFF_MASK;

	s2mu004_i2c_write_byte(muic_data->i2c, S2MU004_REG_MUIC_RID_CTRL, data);

	return 0;
}

int s2mu004_muic_bcd_rescan(struct s2mu004_muic_data *muic_data)
{
	int data = 0;

	pr_info("%s\n", __func__);

	/* start secondary dp dm detect */
	data = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_BCD_RESCAN);
	data |= MUIC_BCD_RESCAN_MASK;
	s2mu004_i2c_write_byte(muic_data->i2c, S2MU004_REG_MUIC_BCD_RESCAN, data);

	return 0;
}

#if IS_ENABLED(CONFIG_MUIC_S2MU004_HV)
int s2mu004_muic_reset_afc_register(struct s2mu004_muic_data *muic_data)
{
	int data = 0;

	pr_info("%s\n", __func__);

	data = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_AFC_LOGIC_CTRL2);
	data |= MUIC_AFC_LOGIC_CTRL2_AFC_RST_MASK;
	s2mu004_i2c_write_byte(muic_data->i2c, S2MU004_REG_AFC_LOGIC_CTRL2, data);

	return 0;
}

static int s2mu004_muic_check_afc_ready(struct s2mu004_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s\n", __func__);

	/* check muic ready for afc */
	if (muic_pdata->afc_disable)
		pr_info("%s AFC Disable(%d) by USER!\n",
			__func__, muic_pdata->afc_disable);
	else {
		pr_info("%s ready:%d  afc_check:%d\n", __func__,
			muic_data->is_afc_muic_ready, muic_data->afc_check);
		if (muic_data->is_afc_muic_ready == false && muic_data->afc_check)
			s2mu004_muic_prepare_afc_charger(muic_data);
	}

	return 0;
}
#endif

void s2mu004_muic_control_vbus_det(struct s2mu004_muic_data *muic_data, bool enable)
{
	int data = 0;

	pr_info("%s (%s)\n", __func__, enable ? "Enable" : "Disable");

	/* enable vbus det for interrupt */
	data = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_AFC_OTP6);

	if (enable)
		data |= MUIC_AFC_OTP6_EN_VBUS_DET_MUIC_MASK;
	else
		data &= ~MUIC_AFC_OTP6_EN_VBUS_DET_MUIC_MASK;

	s2mu004_i2c_write_byte(muic_data->i2c, S2MU004_REG_AFC_OTP6, data);

}

#if defined(GPIO_USB_SEL)
static int s2mu004_set_gpio_usb_sel(int uart_sel)
{
	return 0;
}
#endif /* GPIO_USB_SEL */

int s2mu004_set_gpio_uart_sel(struct s2mu004_muic_data *muic_data, int uart_sel)
{
	const char *mode;
	struct muic_platform_data *muic_pdata = muic_data->pdata;
#if !IS_ENABLED(CONFIG_MUIC_UART_SWITCH)
	int uart_sel_gpio = muic_pdata->gpio_uart_sel;
	int uart_sel_val;
	int ret;

	ret = gpio_request(uart_sel_gpio, "GPIO_UART_SEL");
	if (ret) {
		pr_err("failed to gpio_request GPIO_UART_SEL\n");
		return ret;
	}

	uart_sel_val = gpio_get_value(uart_sel_gpio);

	switch (uart_sel) {
	case MUIC_PATH_UART_AP:
		mode = "AP_UART";
		if (gpio_is_valid(uart_sel_gpio))
			gpio_direction_output(uart_sel_gpio, 1);
		break;
	case MUIC_PATH_UART_CP:
		mode = "CP_UART";
		if (gpio_is_valid(uart_sel_gpio))
			gpio_direction_output(uart_sel_gpio, 0);
		break;
	default:
		mode = "Error";
		break;
	}

	uart_sel_val = gpio_get_value(uart_sel_gpio);

	gpio_free(uart_sel_gpio);

	pr_info("%s, GPIO_UART_SEL(%d)=%c\n",
		mode, uart_sel_gpio, (uart_sel_val == 0 ? 'L' : 'H'));
#else
	switch (uart_sel) {
	case MUIC_PATH_UART_AP:
		mode = "AP_UART";
		pin_config_set(muic_pdata.uart_addr, muic_pdata.uart_rxd,
			       PINCFG_PACK(PINCFG_TYPE_FUNC, 0x2));
		pin_config_set(muic_pdata.uart_addr, muic_pdata.uart_txd,
			       PINCFG_PACK(PINCFG_TYPE_FUNC, 0x2));
		break;
	case MUIC_PATH_UART_CP:
		mode = "CP_UART";
		pin_config_set(muic_pdata.uart_addr, muic_pdata.uart_rxd,
			       PINCFG_PACK(PINCFG_TYPE_FUNC, 0x3));
		pin_config_set(muic_pdata.uart_addr, muic_pdata.uart_txd,
			       PINCFG_PACK(PINCFG_TYPE_FUNC, 0x3));
		break;
	default:
		mode = "Error";
		break;
	}

	pr_info("%s %s\n", __func__, mode);
#endif /* CONFIG_MUIC_UART_SWITCH */
	return 0;
}

#if IS_ENABLED(GPIO_DOC_SWITCH)
static int s2mu004_set_gpio_doc_switch(struct s2mu004_muic_data *muic_data, int val)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	int doc_switch_gpio = muic_pdata->gpio_doc_switch;
	int doc_switch_val;
	int ret;

	ret = gpio_request(doc_switch_gpio, "GPIO_DOC_SWITCH");
	if (ret) {
		pr_err("failed to gpio_request GPIO_DOC_SWITCH\n");
		return ret;
	}

	doc_switch_val = gpio_get_value(doc_switch_gpio);

	if (gpio_is_valid(doc_switch_gpio))
		gpio_set_value(doc_switch_gpio, val);
	doc_switch_val = gpio_get_value(doc_switch_gpio);

	gpio_free(doc_switch_gpio);

	pr_info("%s (%d)%c\n", __func__,
		doc_switch_gpio, (doc_switch_val == 0 ? 'L' : 'H'));

	return 0;
}
#endif /* GPIO_DOC_SWITCH */

#if IS_ENABLED(CONFIG_SEC_FACTORY)
int s2mu004_muic_set_otg_reg(struct s2mu004_muic_data *muic_data, bool on)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val;
	int ret = 0;

	if (on) {
		/* enable vbus det for interrupt */
		s2mu004_muic_control_vbus_det(muic_data, true);
	} else {
		/* disable vbus det for interrupt */
		s2mu004_muic_control_vbus_det(muic_data, false);
	}

	/* 0x1e : hidden register */
	ret = s2mu004_i2c_read_byte(i2c, 0x1e);
	if (ret < 0)
		pr_err("%s err read 0x1e reg(%d)\n", __func__, ret);

	/* Set 0x1e[5:4] bit to 0x11 or 0x01 */
	if (on)
		reg_val = ret | (0x1 << 5);
	else
		reg_val = ret & ~(0x1 << 5);

	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu004_i2c_guaranteed_wbyte(i2c, 0x1e, reg_val);
		if (ret < 0)
			pr_err("%s err write(%d)\n", __func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n", __func__, reg_val, ret);
		return 0;
	}

	ret = s2mu004_i2c_read_byte(i2c, 0x1e);
	if (ret < 0)
		pr_err("%s err read reg 0x1e(%d)\n", __func__, ret);
	else
		pr_info("%s after change(0x%x)\n", __func__, ret);

	return ret;
}

static int s2mu004_muic_init_otg_reg(struct s2mu004_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val;
	int ret = 0;

	/* 0x73 : check EVT0 or EVT1 */
	ret = s2mu004_i2c_read_byte(i2c, 0x73);
	if (ret < 0)
		pr_err("%s err read 'reg 0x73'(%d)\n", __func__, ret);

	if ((ret & 0xF) > 0)
		return 0;

	/* 0x89 : hidden register */
	ret = s2mu004_i2c_read_byte(i2c, 0x89);
	if (ret < 0)
		pr_err("%s err read 'reg 0x89'(%d)\n", __func__, ret);

	/* Set 0x89[1] bit : T_DET_VAL */
	reg_val = ret | (0x1 << 1);

	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu004_i2c_guaranteed_wbyte(i2c, 0x89, reg_val);
		if (ret < 0)
			pr_err("%s err write(%d)\n", __func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n", __func__, reg_val, ret);
		return 0;
	}

	ret = s2mu004_i2c_read_byte(i2c, 0x89);
	if (ret < 0)
		pr_err("%s err read 'reg 0x89'(%d)\n", __func__, ret);
	else
		pr_info("%s after change(0x%x)\n", __func__, ret);

	/* 0x92 : hidden register */
	ret = s2mu004_i2c_read_byte(i2c, 0x92);
	if (ret < 0)
		pr_err("%s err read 'reg 0x92'(%d)\n", __func__, ret);

	/* Set 0x92[7] bit : EN_JIG_AP */
	reg_val = ret | (0x1 << 7);

	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu004_i2c_guaranteed_wbyte(i2c, 0x92, reg_val);
		if (ret < 0)
			pr_err("%s err write(%d)\n", __func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n",	__func__, reg_val, ret);
		return 0;
	}

	ret = s2mu004_i2c_read_byte(i2c, 0x92);
	if (ret < 0)
		pr_err("%s err read 'reg 0x92'(%d)\n", __func__, ret);
	else
		pr_info("%s after change(0x%x)\n", __func__, ret);

	return ret;
}
#endif

/* TODO: There is no needs to use JIGB pin by MUIC if CCIC is supported */
int s2mu004_muic_jig_on(struct s2mu004_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	bool en = muic_pdata->is_jig_on;
	int reg = 0, ret = 0;

	pr_err("%s: %s\n", __func__, en ? "on" : "off");

	reg = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_SW_CTRL);

	if (en)
		reg |= MANUAL_SW_JIG_EN;
	else
		reg &= ~(MANUAL_SW_JIG_EN);

	ret = s2mu004_i2c_write_byte(muic_data->i2c,
				     S2MU004_REG_MUIC_SW_CTRL, (u8) reg);
	if (en) {
		reg = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_CTRL1);
		reg |= CTRL_MANUAL_SW_MASK;

		return s2mu004_i2c_guaranteed_wbyte(muic_data->i2c,
						    S2MU004_REG_MUIC_CTRL1,
						    (u8) reg);
	} else
		return ret;

}

static int s2mu004_muic_set_ctrl_reg(struct s2mu004_muic_data *muic_data, int shift, bool on)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val;
	int ret = 0;

	ret = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_CTRL1);
	if (ret < 0)
		pr_err("%s err read CTRL(%d)\n", __func__, ret);

	if (on)
		reg_val = ret | (0x1 << shift);
	else
		reg_val = ret & ~(0x1 << shift);

	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu004_i2c_guaranteed_wbyte(i2c, S2MU004_REG_MUIC_CTRL1,
						   reg_val);
		if (ret < 0)
			pr_err("%s err write(%d)\n", __func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n",
			__func__, reg_val, ret);
		return 0;
	}

	ret = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_CTRL1);
	if (ret < 0)
		pr_err("%s err read CTRL(%d)\n", __func__, ret);
	else
		pr_info("%s after change(0x%x)\n", __func__, ret);

	return ret;
}

static int s2mu004_muic_set_int_mask(struct s2mu004_muic_data *muic_data, bool on)
{
	int shift = CTRL_INT_MASK_SHIFT;
	int ret = 0;

	ret = s2mu004_muic_set_ctrl_reg(muic_data, shift, on);

	return ret;
}

static int s2mu004_muic_set_manual_sw(struct s2mu004_muic_data *muic_data, bool on)
{
	int shift = CTRL_MANUAL_SW_SHIFT;
	int ret = 0;

	ret = s2mu004_muic_set_ctrl_reg(muic_data, shift, on);

	return ret;
}

static int s2mu004_muic_set_com_sw(struct s2mu004_muic_data *muic_data,
				   u8 reg_val)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret = 0;
	int temp = 0;

	/*  --- MANSW [7:5][4:2][1][0] : DM DP RSVD JIG  --- */
	temp = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_SW_CTRL);
	if (temp < 0)
		pr_err("%s err read MANSW(0x%x)\n", __func__, temp);

	if ((reg_val & MANUAL_SW_DM_DP_MASK) != (temp & MANUAL_SW_DM_DP_MASK)) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__,
			(reg_val & MANUAL_SW_DM_DP_MASK), (temp & MANUAL_SW_DM_DP_MASK));

		ret = s2mu004_i2c_guaranteed_wbyte(i2c,
			S2MU004_REG_MUIC_SW_CTRL, ((reg_val & MANUAL_SW_DM_DP_MASK)|(temp & 0x03)));
		if (ret < 0)
			pr_err("%s err write MANSW(0x%x)\n", __func__,
			(reg_val & MANUAL_SW_DM_DP_MASK) | (temp & 0x03));
	} else {
		pr_info("%s MANSW reg(0x%x), just pass\n", __func__, reg_val);
	}

	return ret;
}

int s2mu004_muic_com_to_open(struct s2mu004_muic_data *muic_data)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = s2mu004_muic_set_com_sw(muic_data, MANSW_OPEN);
	if (ret)
		pr_err("%s set_com_sw err\n", __func__);

	return ret;
}

int s2mu004_muic_com_to_usb(struct s2mu004_muic_data *muic_data)
{
	u8 reg_val;
	int ret = 0;

	reg_val = MANSW_USB;
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_usb err\n", __func__);

	return ret;
}

int s2mu004_muic_com_to_otg(struct s2mu004_muic_data *muic_data)
{
	u8 reg_val;
	int ret = 0;

	reg_val = MANSW_OTG;
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_otg err\n", __func__);

	return ret;
}

int s2mu004_muic_com_to_uart(struct s2mu004_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	u8 reg_val;
	int ret = 0;

	if (muic_data->pdata->is_rustproof) {
		pr_info("%s rustproof mode\n", __func__);
		return ret;
	}

	if (muic_pdata->uart_path == MUIC_PATH_UART_AP) {
		reg_val = MANSW_UART_AP;
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
		send_uart_noti_to_modem(MODEM_CTRL_UART_AP);
#endif
	} else {
		reg_val = MANSW_UART_CP;
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
		send_uart_noti_to_modem(MODEM_CTRL_UART_CP);
#endif
	}

	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_uart err\n", __func__);

	return ret;
}

int s2mu004_muic_com_to_audio(struct s2mu004_muic_data *muic_data)
{
	u8 reg_val;
	int ret = 0;

	reg_val = MANSW_AUDIO;
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_audio err\n", __func__);

	return ret;
}

#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
static int s2mu004_i2c_update_bit(struct i2c_client *i2c,
				  u8 reg, u8 mask, u8 shift, u8 value)
{
	int ret;
	u8 reg_val = 0;

	reg_val = s2mu004_i2c_read_byte(i2c, reg);
	reg_val &= ~mask;
	reg_val |= value << shift;
	ret = s2mu004_i2c_write_byte(i2c, reg, reg_val);
	pr_info("%s reg(0x%x) value(0x%x)\n", __func__, reg, reg_val);
	if (ret < 0)
		pr_err("%s  Reg = 0x%X, mask = 0x%X, val = 0x%X write err : %d\n",
		     __func__, reg, mask, value, ret);

	return ret;
}

static int s2mu004_muic_set_rid_adc_en(struct s2mu004_muic_data *muic_data, bool en)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret = 0;

	pr_info("%s rid en : (%d)\n", __func__, en);
	if (en) {
		/* enable rid detection for muic dp dm detect */
		ret = s2mu004_i2c_update_bit(i2c,
					     S2MU004_REG_MUIC_RID_CTRL,
					     RID_CTRL_ADC_OFF_MASK,
					     RID_CTRL_ADC_OFF_SHIFT, 0x0);
	} else {
		/* disable rid detection for muic dp dm detect */
		ret = s2mu004_i2c_update_bit(i2c,
					     S2MU004_REG_MUIC_RID_CTRL,
					     RID_CTRL_ADC_OFF_MASK,
					     RID_CTRL_ADC_OFF_SHIFT, 0x1);
	}

	return ret;
}

int s2mu004_muic_recheck_adc(struct s2mu004_muic_data *muic_data)
{
	s2mu004_muic_set_rid_adc_en(muic_data, false);
	usleep_range(10000, 12000);
	s2mu004_muic_set_rid_adc_en(muic_data, true);
	msleep(RID_REFRESH_DURATION_MS);
	return s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_ADC) & ADC_MASK;
}

int s2mu004_muic_refresh_adc(struct s2mu004_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int adc = 0;
	u8 reg_data, b_Rid_en = 0;

	reg_data = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_RID_CTRL);
	if (!(reg_data & 0x2)) {
		b_Rid_en = 1;
	} else {
		pr_info("%s, enable the RID\n", __func__);
		reg_data &= ~(0x01 << 1);
		s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_RID_CTRL, reg_data);
		msleep(35);
	}

	adc = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_ADC);
	pr_info("%s, adc : 0x%X\n", __func__, adc);

	if (!b_Rid_en) {
		pr_info("%s, disable the RID\n", __func__);
		reg_data |= (0x01 << 1);
		s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_RID_CTRL, reg_data);
	}
	return adc;
}
#endif

int s2mu004_muic_get_vbus_state(struct s2mu004_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val = 0;
	int vbus = 0;

	reg_val = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_APPLE);
	vbus = !!(reg_val & DEV_TYPE_APPLE_VBUS_WAKEUP);
	pr_info("%s vbus : (%d)\n", __func__, vbus);

	return vbus;
}

static void s2mu004_muic_get_detect_info(struct s2mu004_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	struct i2c_client *i2c = muic_data->i2c;

	muic_data->reg[DEVICE_TYPE1] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_TYPE1);
	muic_data->reg[DEVICE_TYPE2] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_TYPE2);
	muic_data->reg[DEVICE_TYPE3] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_TYPE3);
	muic_data->reg[REV_ID] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_REV_ID);
	muic_data->reg[ADC] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_ADC);
	muic_data->reg[DEVICE_APPLE] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_APPLE);
	muic_data->reg[CHG_TYPE] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_CHG_TYPE);
	muic_data->reg[SC_STATUS2] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_SC_STATUS2);

	muic_data->adc = muic_pdata->adc = muic_data->reg[ADC];
	muic_data->vbvolt = muic_pdata->vbvolt =
			!!(muic_data->reg[DEVICE_APPLE] & DEV_TYPE_APPLE_VBUS_WAKEUP);
	muic_data->vmid = muic_pdata->vmid = !!(muic_data->reg[SC_STATUS2] & 0x7);

	pr_info("dev[1:0x%02x, 2:0x%02x, 3:0x%02x]\n", muic_data->reg[DEVICE_TYPE1],
		muic_data->reg[DEVICE_TYPE2], muic_data->reg[DEVICE_TYPE3]);
	pr_info("adc:0x%02x, vbvolt:0x%02x, apple:0x%02x\n",
		muic_data->adc, muic_data->vbvolt, muic_data->reg[DEVICE_APPLE]);
	pr_info("chg_type:0x%02x, vmid:0x%02x, dev_id:0x%02x\n",
		muic_data->reg[CHG_TYPE], muic_data->vmid, muic_data->reg[REV_ID]);
}

static int s2mu004_muic_reg_init(struct s2mu004_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret = 0;
	struct muic_interface_t *muic_if =
	    (struct muic_interface_t *)muic_data->if_data;
	int data = 0;

	pr_info("%s\n", __func__);

	s2mu004_muic_get_detect_info(muic_data);

	if (muic_if->opmode == OPMODE_DEVICE) {
		s2mu004_muic_control_rid_adc(muic_data, S2MU004_DISABLE);

		/* adc, RID int masking */
		data = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_INT1_MASK);
		data |= (INTm_ATTACH_MASK | INTm_KP_MASK);
		s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_INT1_MASK, data);

		data = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_INT2_MASK);
		data |= INTm_ADC_CHANGE_MASK;
		s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_INT2_MASK, data);
	}

	s2mu004_muic_control_vbus_det(muic_data, false);
	ret = s2mu004_i2c_guaranteed_wbyte(i2c,
		S2MU004_REG_MUIC_CTRL1, CTRL_MASK);
	if (ret < 0)
		pr_err("failed to write ctrl(%d)\n", ret);

	/*
	 * These registers represents the RID ADC LDO voltage control.
	 * Low / High LDO initialized to 3V, 2.7V each.
	 */

#ifdef CONFIG_MUIC_S2MU004_RID
	s2mu004_muic_control_rid_adc(muic_data, S2MU004_ENABLE);

	s2mu004_i2c_update_bit(i2c,
			S2MU004_REG_LDOADC_VSETL, LDOADC_VSETH_MASK, 0, LDOADC_VSET_3V);
	s2mu004_i2c_update_bit(i2c,
			S2MU004_REG_LDOADC_VSETH, LDOADC_VSETH_MASK, 0, LDOADC_VSET_2_7V);

	s2mu004_i2c_update_bit(i2c,
			       S2MU004_REG_LDOADC_VSETH,
			       LDOADC_VSETH_WAKE_HYS_MASK,
			       LDOADC_VSETH_WAKE_HYS_SHIFT, 0x1);
#endif

#if defined(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_handle((!!muic_data->vbvolt) ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */

	return ret;
}

static void s2mu004_muic_detect_dev_ccic(struct s2mu004_muic_data *muic_data,
	muic_attached_dev_t new_dev)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	int adc = 0, vbvolt = 0;

	pr_info("%s (new_dev=%s)\n", __func__, dev_to_str(new_dev));

	if (muic_pdata->attached_dev == new_dev) {
		pr_info("Skip to handle duplicated type\n");
		return;
	}

	if (new_dev == ATTACHED_DEV_NONE_MUIC) {
		/* Detach from CCIC */
		if (muic_core_get_ccic_cable_state(muic_data->pdata) == false) {
			pr_info("Skip to detach legacy type\n");
			return;
		}
		if (muic_pdata->attached_dev == ATTACHED_DEV_OTG_MUIC) {
			/* disable vbus det for interrupt */
			s2mu004_muic_control_vbus_det(muic_data, false);
		}
		muic_pdata->attached_dev = ATTACHED_DEV_NONE_MUIC;
	} else {
		/* Attach from CCIC */
		muic_pdata->attached_dev = new_dev;
		pr_info("%s DETECTED\n", dev_to_str(new_dev));

		switch (new_dev) {
		case ATTACHED_DEV_OTG_MUIC:
			s2mu004_muic_set_com_sw(muic_data, MANSW_USB);

			/* enable vbus det for interrupt */
			s2mu004_muic_control_vbus_det(muic_data, true);
			break;
		case ATTACHED_DEV_TYPE3_CHARGER_MUIC: /* PD Charger */
			s2mu004_muic_set_com_sw(muic_data, MANSW_OPEN);
			return;
		default:
			break;
		}
	}

	if (muic_pdata->attached_dev != ATTACHED_DEV_NONE_MUIC) {
		adc = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_ADC);
		vbvolt = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_DEVICE_APPLE);
		muic_core_handle_attach(muic_data->pdata, new_dev, adc, !!vbvolt);
	} else if (muic_pdata->attached_dev == ATTACHED_DEV_NONE_MUIC) {
		muic_core_handle_detach(muic_data->pdata);
	}
}

static int s2mu004_muic_detect_dev_bc1p2(struct s2mu004_muic_data *muic_data)
{
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;

	muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;

	/* Attached */
	switch (muic_data->reg[DEVICE_TYPE1]) {
	case DEV_TYPE1_CDP:
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_CDP_MUIC;
			pr_info("USB_CDP DETECTED\n");
		}
		break;
	case DEV_TYPE1_USB:
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("USB DETECTED\n");
		}
		break;
	case DEV_TYPE1_DEDICATED_CHG:
	case 0x44:
	case 0x60:
		if (muic_data->vbvolt) {
			muic_if->is_dcp_charger = true;
			muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
			muic_data->afc_check = true;
			pr_info("DEDICATED CHARGER DETECTED\n");
		}
		break;
#ifdef CONFIG_MUIC_S2MU004_RID
	case DEV_TYPE1_USB_OTG:
		if (muic_if->opmode == OPMODE_DEVICE) {
			muic_data->new_dev = ATTACHED_DEV_OTG_MUIC;
			pr_info("USB_OTG DETECTED\n");
		}
		break;
#endif /* CONFIG_MUIC_S2MU004_RID */
	case DEV_TYPE1_T1_T2_CHG:
		if (muic_data->vbvolt) {
			/* 200K, 442K should be checkef */
			if (muic_data->adc == ADC_CEA936ATYPE2_CHG) {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
				pr_info("CEA936ATYPE2_CHG DETECTED\n");
				muic_data->afc_check = false;
			} else {
				muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
				pr_info("T1_T2_CHG DETECTED\n");
			}
		}
		break;
	default:
		break;
	}

	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC &&
			muic_data->new_dev != ATTACHED_DEV_NONE_MUIC)
		goto detect_done;

	switch (muic_data->reg[DEVICE_TYPE2]) {
	case DEV_TYPE2_SDP_1P8S:
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_TIMEOUT_OPEN_MUIC;
			pr_info("%s:%s: SDP_1P8S DETECTED\n", MUIC_DEV_NAME, __func__);
			muic_if->is_dcdtmr_intr = true;
			schedule_delayed_work(&muic_data->dcd_recheck, 0);
		}
		break;
	default:
		break;
	}

	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC &&
			muic_data->new_dev != ATTACHED_DEV_NONE_MUIC)
		goto detect_done;

	if (muic_data->vbvolt && ((muic_data->reg[DEVICE_APPLE] & DEV_TYPE_APPLE_APPLE2P4A_CHG)
		|| (muic_data->reg[DEVICE_APPLE] & DEV_TYPE_APPLE_APPLE2A_CHG)
		|| (muic_data->reg[DEVICE_APPLE] & DEV_TYPE_APPLE_APPLE1A_CHG)
		|| (muic_data->reg[DEVICE_APPLE] & DEV_TYPE_APPLE_APPLE0P5A_CHG))) {
		muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
		muic_data->afc_check = false;
		pr_info("APPLE_CHG DETECTED\n");
		muic_if->is_dcdtmr_intr = true;
		schedule_delayed_work(&muic_data->dcd_recheck, 0);
	}

	if ((muic_data->reg[CHG_TYPE] & DEV_TYPE_CHG_TYPE) &&
			(muic_data->new_dev == ATTACHED_DEV_UNKNOWN_MUIC)) {
		muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
		muic_data->afc_check = false;
		pr_info("CHG_TYPE DETECTED\n");
		muic_if->is_dcdtmr_intr = true;
		schedule_delayed_work(&muic_data->dcd_recheck, 0);
	}

detect_done:
	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC)
		return S2MU004_DETECT_DONE;
	else
		return S2MU004_DETECT_NONE;
}

#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
static int s2mu004_muic_detect_dev_jig_type(struct s2mu004_muic_data *muic_data)
{
	switch (muic_data->reg[DEVICE_TYPE2]) {
	case DEV_TYPE2_JIG_UART_OFF:
		if (muic_data->vbvolt)
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		else
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		pr_info("JIG_UART_OFF DETECTED\n");
		break;
	case DEV_TYPE2_JIG_USB_OFF:
		if (!muic_data->vbvolt)
			break;
		muic_data->new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
		pr_info("JIG_USB_OFF DETECTED\n");
		break;
	case DEV_TYPE2_JIG_USB_ON:
		if (!muic_data->vbvolt)
			break;
		muic_data->new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		pr_info("JIG_USB_ON DETECTED\n");
		break;
	case DEV_TYPE2_JIG_UART_ON:
		if (muic_data->new_dev != ATTACHED_DEV_JIG_UART_ON_MUIC) {
			if (!muic_data->vbvolt) {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
				pr_info("ADC JIG_UART_ON DETECTED\n");
			} else {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
				pr_info("ADC JIG_UART_ON_VB DETECTED\n");
			}
		}
		break;
	default:
		break;
	}

	return muic_data->new_dev;
}

static int s2mu004_muic_detect_dev_rid_array(struct s2mu004_muic_data *muic_data)
{
	switch (muic_data->adc) {
	case ADC_JIG_UART_OFF:
		pr_info("ADC_JIG_UART_OFF\n");
		if (muic_data->vbvolt)
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		else
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		break;
	case ADC_JIG_USB_ON:
	case ADC_JIG_USB_OFF:
	case ADC_DESKDOCK:
		muic_data->new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		pr_info("ADC JIG_USB_ON DETECTED\n");
		break;
#if defined(CONFIG_SEC_FACTORY)
	case ADC_CEA936ATYPE1_CHG:
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_CARKIT_MUIC;
			pr_info("SMD DL 255k 200k charger disable\n");
		} else {
			return;
		}
		break;
#endif
	default:
		pr_info("%s unsupported ADC(0x%02x)\n", __func__, muic_data->adc);
	}

	return muic_data->new_dev;
}

static int s2mu004_muic_detect_dev_rid_device(struct s2mu004_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	switch (muic_data->adc) {
	case ADC_CEA936ATYPE1_CHG:	/*200k ohm */
		if (muic_data->vbvolt) {
			/* This is workaournd for LG USB cable which has 219k ohm ID */
			muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("TYPE1 CHARGER DETECTED(USB)\n");
		}
		break;
	case ADC_CEA936ATYPE2_CHG:
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
			muic_data->afc_check = false;
			pr_info("%s unsupported ADC(0x%02x)\n", __func__, muic_data->adc);
		}
		break;
	case ADC_JIG_USB_OFF:	/* 255k */
		if (!muic_data->vbvolt)
			break;
		if (muic_data->new_dev != ATTACHED_DEV_JIG_USB_OFF_MUIC) {
			muic_data->new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
			pr_info("ADC JIG_USB_OFF DETECTED\n");
		}
		break;
	case ADC_JIG_USB_ON:
		if (!muic_data->vbvolt)
			break;
		if (muic_data->new_dev != ATTACHED_DEV_JIG_USB_ON_MUIC) {
			if (!muic_data->vbvolt) {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
				pr_info("ADC JIG_UART_ON DETECTED\n");
			} else {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
				pr_info("ADC JIG_UART_ON_VB DETECTED\n");
			}
		}
		break;
	case ADC_JIG_UART_OFF:
		if (muic_pdata->is_otg_test) {
			mdelay(100);
			if (muic_data->vmid == 0x4) {
				pr_info("OTG_TEST DETECTED, vmid = %d\n", muic_data->vmid);
				muic_data->vbvolt = 1;
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC;
			} else
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		} else if (muic_data->vbvolt)
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		else
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;

		pr_info("ADC JIG_UART_OFF DETECTED\n");
		break;
	case ADC_JIG_UART_ON:
		if (muic_data->new_dev != ATTACHED_DEV_JIG_UART_ON_MUIC) {
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
			pr_info("ADC JIG_UART_ON DETECTED\n");
		}
		break;
	case ADC_SMARTDOCK: /* 0x10000 40.2K ohm */
		/* SMARTDOCK is not supported */
		/* force not to charge the device with SMARTDOCK */
		muic_data->new_dev = ATTACHED_DEV_UNSUPPORTED_ID_VB_MUIC;
		pr_info("%s unsupported ADC(0x%02x) but charging\n",
			__func__, muic_data->adc);
		break;
	case ADC_HMT:	/* 0x10001 49.9K ohm */
		muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
		pr_info("%s unsupported ADC(0x%02x) not charging\n",
			__func__, muic_data->adc);
		break;
	case ADC_AUDIODOCK:
#ifdef CONFIG_MUIC_S2MU004_SUPPORT_AUDIODOCK
		muic_data->new_dev = ATTACHED_DEV_AUDIODOCK_MUIC;
#else
		muic_data->new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
#endif
		pr_info("ADC AUDIODOCK DETECTED\n");
		break;
	case ADC_UNIVERSAL_MMDOCK:
		muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
		pr_info("%s unsupported ADC(0x%02x) not charging\n",
			__func__, muic_data->adc);
		break;
	case ADC_OPEN:
		/* sometimes muic fails to catch JIG_UART_OFF detaching */
		/* double check with ADC */
		if (muic_data->new_dev == ATTACHED_DEV_JIG_UART_OFF_MUIC) {
			muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
			pr_info("ADC OPEN DETECTED\n");
		}
		break;
	default:
		muic_data->new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
		pr_info("%s unsupported ADC(0x%02x)\n", __func__, muic_data->adc);
	}

	return muic_data->new_dev;
}

static int s2mu004_muic_detect_dev_mrid_adc(struct s2mu004_muic_data *muic_data)
{
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;

	muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;

	if (muic_data->adc & ADC_CONVERSION_ERR_MASK) {
		pr_info("%s ADC conversion error! (adc=%#x)\n", __func__, muic_data->adc);
		return muic_data->new_dev;
	}

	if (muic_if->opmode == OPMODE_SMD_ARRAY)
		muic_data->new_dev = s2mu004_muic_detect_dev_rid_array(muic_data);
	else if (muic_if->opmode == OPMODE_DEVICE)
		muic_data->new_dev = s2mu004_muic_detect_dev_rid_device(muic_data);

	s2mu004_muic_detect_dev_jig_type(muic_data);

	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC)
		return S2MU004_DETECT_DONE;
	else
		return S2MU004_DETECT_NONE;
}
#endif /* CONFIG_MUIC_S2MU004_RID */

static irqreturn_t s2mu004_muic_attach_isr(int irq, void *data)
{
	struct s2mu004_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	int ret = 0;

	if (muic_data == NULL || muic_pdata == NULL) {
		pr_err("%s, data NULL\n", __func__);
		return IRQ_NONE;
	}

	pr_info("%s start\n", __func__);

	if (muic_core_is_cable_attached(muic_pdata)) {
		pr_info("%s Cable type already detected\n", __func__);
		goto irq_skip;
	}

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	s2mu004_muic_get_detect_info(muic_data);

	if (muic_pdata->vbvolt) {
		ret = s2mu004_muic_detect_dev_bc1p2(muic_data);
		if (ret == S2MU004_DETECT_DONE)
			goto attach_handle;
	}

#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
	ret = s2mu004_muic_detect_dev_mrid_adc(muic_data);
#endif

attach_handle:
	if (muic_data->new_dev != ATTACHED_DEV_NONE_MUIC &&
				muic_data->new_dev != muic_pdata->attached_dev) {
		muic_manager_set_legacy_dev(muic_pdata->muic_if, muic_data->new_dev);
		muic_core_handle_attach(muic_pdata, muic_data->new_dev,
				muic_pdata->adc, muic_pdata->vbvolt);
	}

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

irq_skip:
	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	return IRQ_HANDLED;
}

static irqreturn_t s2mu004_muic_detach_isr(int irq, void *data)
{
	struct s2mu004_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	struct muic_interface_t *muic_if = muic_data->if_data;

	if (muic_data == NULL || muic_pdata == NULL || muic_if == NULL) {
		pr_err("%s, data NULL\n", __func__);
		return IRQ_NONE;
	}

	pr_info("%s start\n", __func__);

	if (muic_core_is_cable_attached(muic_pdata) == false) {
		pr_info("%s Cable type already detached\n", __func__);
		goto irq_skip;
	}

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	if (muic_if->opmode == OPMODE_DEVICE) {
		if (muic_core_get_ccic_cable_state(muic_data->pdata))
			pr_info("%s skip to handle ccic dev\n", __func__);
		else
			muic_core_handle_detach(muic_data->pdata);
	} else if (muic_if->opmode == OPMODE_SMD_ARRAY)
		muic_core_handle_detach(muic_data->pdata);

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

irq_skip:
	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	return IRQ_HANDLED;
}

static irqreturn_t s2mu004_muic_vbus_isr(int irq, void *data)
{
	struct s2mu004_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	if (muic_data == NULL || muic_pdata == NULL) {
		pr_err("%s, data NULL\n", __func__);
		return IRQ_NONE;
	}

	pr_info("%s start\n", __func__);

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	muic_pdata->vbvolt = muic_data->vbvolt = s2mu004_muic_get_vbus_state(muic_data);

#if defined(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_handle(muic_data->vbvolt ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	return IRQ_HANDLED;
}

#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
static irqreturn_t s2mu004_muic_rid_chg_isr(int irq, void *data)
{
	struct s2mu004_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s start\n", __func__);

	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	return IRQ_HANDLED;
}

static irqreturn_t s2mu004_muic_adc_change_isr(int irq, void *data)
{
	struct s2mu004_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s start\n", __func__);

	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	return IRQ_HANDLED;

}
#endif

static irqreturn_t s2mu004_muic_reserved_isr(int irq, void *data)
{
	struct s2mu004_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s start\n", __func__);

	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	return IRQ_HANDLED;

}

static void s2mu004_muic_dcd_recheck(struct work_struct *work)
{
	struct s2mu004_muic_data *muic_data =
	    container_of(work, struct s2mu004_muic_data, dcd_recheck.work);
	struct muic_interface_t *muic_if = muic_data->if_data;

	pr_info("%s\n", __func__);
	muic_manager_dcd_rescan(muic_if);
}

void s2mu004_muic_dcd_rescan(struct s2mu004_muic_data *muic_data)
{
	int ret = 0;
	int reg_val = 0;
	struct i2c_client *i2c = muic_data->i2c;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s call\n", __func__);

	/* muic mux switch open */
	reg_val = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_SW_CTRL);
	ret = s2mu004_muic_com_to_open(muic_data);
	if (ret < 0)
		pr_err("%s, fail to open mansw\n", __func__);

#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
	s2mu004_muic_control_rid_adc(muic_data, S2MU004_ENABLE);
	msleep(50);
	s2mu004_muic_control_rid_adc(muic_data, S2MU004_DISABLE);
#endif /* CONFIG_MUIC_S2MU004_RID */
	msleep(100);

	s2mu004_muic_bcd_rescan(muic_data);
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
	msleep(650);
#else
	msleep(50);
#endif

	/* restore muic mux switch */
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret < 0)
		pr_err("%s, fail to restore mansw\n", __func__);

	mutex_unlock(&muic_data->switch_mutex);
}

static int s2mu004_init_rev_info(struct s2mu004_muic_data *muic_data)
{
	u8 dev_id;
	int ret = 0;

	dev_id = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_REV_ID);
	if (dev_id < 0) {
		pr_err("%s(%d)\n", __func__, dev_id);
		ret = -ENODEV;
	} else {
		muic_data->muic_vendor = 0x05;
		muic_data->muic_version = (dev_id & 0x0F);
		muic_data->ic_rev_id = (dev_id & 0xF0) >> 4;
		pr_info("%s : vendor=0x%x, ver=0x%x, dev_id=0x%x\n rev id =0x%x\n",
		     __func__, muic_data->muic_vendor, muic_data->muic_version,
		     dev_id, muic_data->ic_rev_id);
	}
	return ret;
}

static int s2mu004_muic_irq_init(struct s2mu004_muic_data *muic_data)
{
	int ret = 0;

	if (muic_data->mfd_pdata && (muic_data->mfd_pdata->irq_base > 0)) {
		int irq_base = muic_data->mfd_pdata->irq_base;

		/* request MUIC IRQ */
		muic_data->irq_attach = irq_base + S2MU004_MUIC_IRQ1_ATTATCH;
		REQUEST_IRQ(muic_data->irq_attach, muic_data,
			"muic-attach", &s2mu004_muic_attach_isr);

		muic_data->irq_detach = irq_base + S2MU004_MUIC_IRQ1_DETACH;
		REQUEST_IRQ(muic_data->irq_detach, muic_data,
			"muic-detach", &s2mu004_muic_detach_isr);

#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
		muic_data->irq_rid_chg = irq_base + S2MU004_MUIC_IRQ1_RID_CHG;
		REQUEST_IRQ(muic_data->irq_rid_chg, muic_data,
			"muic-rid_chg", &s2mu004_muic_rid_chg_isr);
#endif

		muic_data->irq_vbus_on = irq_base + S2MU004_MUIC_IRQ2_VBUS_ON;
		REQUEST_IRQ(muic_data->irq_vbus_on, muic_data,
			"muic-vbus_on", &s2mu004_muic_vbus_isr);

		muic_data->irq_rsvd_attach = irq_base + S2MU004_MUIC_IRQ2_RSVD_ATTACH;
		REQUEST_IRQ(muic_data->irq_rsvd_attach, muic_data,
			"muic-rsvd_attach", &s2mu004_muic_reserved_isr);

#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
		muic_data->irq_adc_change = irq_base + S2MU004_MUIC_IRQ2_ADC_CHANGE;
		REQUEST_IRQ(muic_data->irq_adc_change, muic_data,
			"muic-adc_change", &s2mu004_muic_adc_change_isr);
#endif

		muic_data->irq_av_charge = irq_base + S2MU004_MUIC_IRQ2_AV_CHARGE;
		REQUEST_IRQ(muic_data->irq_av_charge, muic_data,
			"muic-av_charge", &s2mu004_muic_reserved_isr);

		muic_data->irq_vbus_off = irq_base + S2MU004_MUIC_IRQ2_VBUS_OFF;
		REQUEST_IRQ(muic_data->irq_vbus_off, muic_data,
			"muic-vbus_off", &s2mu004_muic_vbus_isr);

	}

	pr_info("%s muic-attach(%d), muic-detach(%d), muic-rid_chg(%d), muic-vbus_on(%d)",
		__func__, muic_data->irq_attach, muic_data->irq_detach, muic_data->irq_rid_chg,
			muic_data->irq_vbus_on);
	pr_info("muic-rsvd_attach(%d), muic-adc_change(%d), muic-av_charge(%d), muic-vbus_off(%d)\n",
		muic_data->irq_rsvd_attach, muic_data->irq_adc_change, 
		muic_data->irq_av_charge, muic_data->irq_vbus_off);

	return ret;
}

static void s2mu004_muic_free_irqs(struct s2mu004_muic_data *muic_data)
{
	pr_info("%s\n", __func__);

	/* free MUIC IRQ */
	FREE_IRQ(muic_data->irq_attach, muic_data, "muic-attach");
	FREE_IRQ(muic_data->irq_detach, muic_data, "muic-detach");
#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
	FREE_IRQ(muic_data->irq_rid_chg, muic_data, "muic-rid_chg");
#endif
	FREE_IRQ(muic_data->irq_vbus_on, muic_data, "muic-vbus_on");
	FREE_IRQ(muic_data->irq_rsvd_attach, muic_data, "muic-rsvd_attach");
#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
	FREE_IRQ(muic_data->irq_adc_change, muic_data, "muic-adc_change");
#endif
	FREE_IRQ(muic_data->irq_av_charge, muic_data, "muic-av_charge");
	FREE_IRQ(muic_data->irq_vbus_off, muic_data, "muic-vbus_off");
}

#if IS_ENABLED(CONFIG_OF)
static int of_s2mu004_muic_dt(struct device *dev,
			      struct s2mu004_muic_data *muic_data)
{
	struct device_node *np, *np_muic;
	int ret = 0;

	np = dev->parent->of_node;
	if (!np) {
		pr_err("%s : could not find np\n", __func__);
		return -ENODEV;
	}

	np_muic = of_find_node_by_name(np, "muic");
	if (!np_muic) {
		pr_err("%s : could not find muic sub-node np_muic\n", __func__);
		return -EINVAL;
	}

/* FIXME */
#if !IS_ENABLED(CONFIG_MUIC_UART_SWITCH)
	if (of_gpio_count(np_muic) < 1) {
		pr_err("%s : could not find muic gpio\n", __func__);
		muic_data->pdata->gpio_uart_sel = 0;
	} else
		muic_data->pdata->gpio_uart_sel = of_get_gpio(np_muic, 0);
#else
	muic_data->pdata->uart_addr =
	    (const char *)of_get_property(np_muic, "muic,uart_addr", NULL);
	muic_data->pdata->uart_txd =
	    (const char *)of_get_property(np_muic, "muic,uart_txd", NULL);
	muic_data->pdata->uart_rxd =
	    (const char *)of_get_property(np_muic, "muic,uart_rxd", NULL);
#endif

	return ret;
}
#endif /* CONFIG_OF */

static void s2mu004_muic_init_drvdata(struct s2mu004_muic_data *muic_data,
				      struct s2mu004_dev *s2mu004,
				      struct platform_device *pdev,
				      struct s2mu004_platform_data *mfd_pdata)
{
	/* save platfom data for gpio control functions */
	muic_data->s2mu004_dev = s2mu004;
	muic_data->dev = &pdev->dev;
	muic_data->i2c = s2mu004->i2c;
	muic_data->mfd_pdata = mfd_pdata;
	muic_data->re_detect = 0;
	muic_data->afc_check = false;
	muic_data->is_dcd_recheck = false;
}

static void s2mu004_muic_init_interface(struct s2mu004_muic_data *muic_data,
					struct muic_interface_t *muic_if)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s, muic_if : 0x%p, muic_data : 0x%p\n",
		__func__, muic_if, muic_data);

	muic_if->muic_data = (void *)muic_data;
	muic_if->set_com_to_open = s2mu004_if_com_to_open;
	muic_if->set_switch_to_usb = s2mu004_if_switch_to_usb;
	muic_if->set_switch_to_uart = s2mu004_if_switch_to_uart;
	muic_if->get_vbus = s2mu004_if_get_vbus;
	muic_if->set_cable_state = s2mu004_if_set_cable_state;
	muic_if->set_otg_detect_en = s2mu004_if_set_otg_detect_en;
	muic_if->set_dcd_rescan = s2mu004_if_dcd_rescan;
	muic_if->set_jig_ctrl_on = s2mu004_if_jig_on;
	muic_if->set_com_to_audio = s2mu004_if_com_to_audio;
	muic_if->set_com_to_otg = s2mu004_if_com_to_otg;
	muic_if->set_gpio_uart_sel = s2mu004_if_set_gpio_uart_sel;
	muic_if->bcd_rescan = s2mu004_if_bcd_rescan;
	muic_if->control_rid_adc = s2mu004_if_control_rid_adc;
#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
	muic_if->get_adc = s2mu004_if_get_adc;
#endif
#if defined(CONFIG_MUIC_S2MU004_HV)
	muic_if->set_afc_reset = s2mu004_if_set_afc_reset;
	muic_if->check_id_err = s2mu004_if_check_id_err;
	muic_if->reset_hvcontrol_reg = s2mu004_if_reset_hvcontrol_reg;
	muic_if->check_afc_ready = s2mu004_if_check_afc_ready;
	muic_if->reset_afc_register = s2mu004_if_reset_afc_register;
#endif
	muic_data->if_data = muic_if;
	muic_pdata->muic_if = muic_if;
}

static int s2mu004_muic_probe(struct platform_device *pdev)
{
	struct s2mu004_dev *s2mu004 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu004_platform_data *mfd_pdata = dev_get_platdata(s2mu004->dev);
	struct s2mu004_muic_data *muic_data;
	struct muic_platform_data *muic_pdata;
	int ret = 0;
	struct muic_interface_t *muic_if;

	pr_info("%s start\n", __func__);
	muic_data = devm_kzalloc(&pdev->dev, sizeof(*muic_data), GFP_KERNEL);
	if (unlikely(!muic_data)) {
		pr_err("%s out of memory\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}
	static_data = muic_data;

	if (unlikely(!mfd_pdata)) {
		pr_err("%s failed to get s2mu004 mfd platform data\n",
		       __func__);
		ret = -ENOMEM;
		goto err_kfree1;
	}

	muic_pdata = muic_core_init(muic_data);
	if (unlikely(!muic_pdata))
		goto err_kfree1;

	muic_data->pdata = muic_pdata;
	muic_if = muic_manager_init(muic_pdata, muic_data);
	if (!muic_if) {
		pr_err("%s failed to init muic manager, ret : 0x%X\n",
		       __func__, ret);
		goto err_init_manager;
	}
	s2mu004_muic_init_interface(muic_data, muic_if);
	s2mu004_muic_init_drvdata(muic_data, s2mu004, pdev, mfd_pdata);
#if defined(CONFIG_OF)
	ret = of_s2mu004_muic_dt(&pdev->dev, muic_data);
	if (ret < 0)
		pr_err("no muic dt! ret[%d]\n", ret);
#endif /* CONFIG_OF */

	mutex_init(&muic_data->muic_mutex);
	mutex_init(&muic_data->switch_mutex);
	wake_lock_init(&muic_data->wake_lock, WAKE_LOCK_SUSPEND, "muic_wake");
	platform_set_drvdata(pdev, muic_data);

	if (muic_data->pdata->init_gpio_cb)
		ret = muic_data->pdata->init_gpio_cb(muic_data->pdata, get_switch_sel());
	if (ret) {
		pr_err("%s failed to init gpio(%d)\n", __func__, ret);
		goto fail_init_gpio;
	}

	muic_pdata->uart_path = MUIC_PATH_UART_AP;
	muic_pdata->usb_path = MUIC_PATH_USB_AP;
	pr_info("%s: usb_path(%d), uart_path(%d)\n", __func__,
		muic_pdata->usb_path, muic_pdata->uart_path);

	ret = s2mu004_muic_init_sysfs(muic_data);
	if (ret) {
		pr_err("failed to create sysfs\n");
		goto fail_init_sysfs;
	}

	ret = s2mu004_init_rev_info(muic_data);
	if (ret) {
		pr_err("failed to init muic(%d)\n", ret);
		goto fail;
	}

	ret = s2mu004_muic_reg_init(muic_data);
	if (ret) {
		pr_err("failed to init muic(%d)\n", ret);
		goto fail;
	}
#if IS_ENABLED(CONFIG_MUIC_S2MU004_HV)
	/* initial hv cable detection */
	if (muic_data->is_afc_muic_ready)
		s2mu004_hv_muic_init_detect(muic_data);

	s2mu004_hv_muic_initialize(muic_data);
#endif /* CONFIG_MUIC_S2MU004_HV */

	if (muic_pdata->is_rustproof) {
		pr_err("%s rustproof is enabled\n", __func__);
		s2mu004_muic_com_to_open(muic_data);
	}
#if IS_ENABLED(CONFIG_MUIC_S2MU004_HV)
	if (get_afc_mode() == CH_MODE_AFC_DISABLE_VAL) {
		pr_info("AFC mode disabled\n");
		muic_data->pdata->afc_disable = true;
	} else {
		pr_info("AFC mode enabled\n");
		muic_data->pdata->afc_disable = false;
	}
#endif /* CONFIG_MUIC_S2MU004_HV */

	ret = s2mu004_muic_irq_init(muic_data);
	if (ret) {
		pr_err("%s failed to init irq(%d)\n", __func__, ret);
		goto fail_init_irq;
	}
#if IS_ENABLED(CONFIG_MUIC_S2MU004_HV)
	ret = s2mu004_afc_muic_irq_init(muic_data);
	if (ret < 0) {
		pr_err("%s Failed to initialize HV MUIC irq:%d\n",
		       __func__, ret);
		s2mu004_hv_muic_free_irqs(muic_data);
	}
#endif /* CONFIG_MUIC_S2MU004_HV */

	/* initial cable detection */
	s2mu004_muic_set_int_mask(muic_data, false);
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	s2mu004_muic_init_otg_reg(muic_data);
#endif

	pr_info("%s muic_if->opmode(%d)\n", __func__, muic_if->opmode);

	INIT_DELAYED_WORK(&muic_data->dcd_recheck, s2mu004_muic_dcd_recheck);

	s2mu004_muic_attach_isr(-1, muic_data);

#if IS_ENABLED(CONFIG_MUIC_S2MU004_RID)
	if (!s2mu004_muic_get_vbus_state(muic_data)) {
		pr_info("%s : init adc : 0x%X\n", __func__,
			s2mu004_muic_recheck_adc(muic_data));
	}
#endif

	return 0;

fail_init_irq:
fail:
#ifdef CONFIG_DRV_SAMSUNG
	s2mu004_muic_deinit_sysfs(muic_data);
#endif
fail_init_sysfs:
fail_init_gpio:
	mutex_destroy(&muic_data->muic_mutex);
err_init_manager:
err_kfree1:
err_return:
	return ret;
}

/* if need to set s2mu004 pdata */
static const struct of_device_id s2mu004_muic_match_table[] = {
	{.compatible = "samsung,s2mu004-muic",},
	{},
};

static int s2mu004_muic_remove(struct platform_device *pdev)
{
	struct s2mu004_muic_data *muic_data = platform_get_drvdata(pdev);

	if (muic_data) {
		pr_info("%s\n", __func__);

#ifdef CONFIG_DRV_SAMSUNG
		s2mu004_muic_deinit_sysfs(muic_data);
#endif
		muic_manager_exit(muic_data->if_data);
		muic_core_exit(muic_data->pdata);

		disable_irq_wake(muic_data->i2c->irq);
		s2mu004_muic_free_irqs(muic_data);
		mutex_destroy(&muic_data->muic_mutex);
		mutex_destroy(&muic_data->switch_mutex);
		i2c_set_clientdata(muic_data->i2c, NULL);
	}

	return 0;
}

static void s2mu004_muic_shutdown(struct platform_device *pdev)
{
	struct s2mu004_muic_data *muic_data = platform_get_drvdata(pdev);
	int ret;

	pr_info("%s\n", __func__);

	if (!muic_data->i2c) {
		pr_err("%s no muic i2c client\n", __func__);
		return;
	}
#if IS_ENABLED(CONFIG_MUIC_S2MU004_HV)
	s2mu004_hv_muic_remove(muic_data);
#endif

	ret = s2mu004_muic_com_to_open(muic_data);
	if (ret < 0)
		pr_err("fail to open mansw\n");

	/* set auto sw mode before shutdown to make sure device goes into */
	/* LPM charging when TA or USB is connected during off state */
	pr_info("muic auto detection enable\n");
	ret = s2mu004_muic_set_manual_sw(muic_data, true);
	if (ret < 0) {
		pr_err("%s fail to update reg\n", __func__);
		return;
	}
}

#if IS_ENABLED(CONFIG_PM)
static int s2mu004_muic_suspend(struct device *dev)
{
	struct s2mu004_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	muic_pdata->suspended = true;

	return 0;
}

static int s2mu004_muic_resume(struct device *dev)
{
	struct s2mu004_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	muic_pdata->suspended = false;

	if (muic_pdata->need_to_noti) {
		if (muic_pdata->attached_dev) {
			MUIC_SEND_NOTI_ATTACH(muic_pdata->attached_dev);
		} else {
			MUIC_SEND_NOTI_DETACH(muic_pdata->attached_dev);
		}

		muic_pdata->need_to_noti = false;
	}

	return 0;
}
#else
#define s2mu004_muic_suspend NULL
#define s2mu004_muic_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(s2mu004_muic_pm_ops, s2mu004_muic_suspend,
			 s2mu004_muic_resume);

static struct platform_driver s2mu004_muic_driver = {
	.probe = s2mu004_muic_probe,
	.remove = s2mu004_muic_remove,
	.shutdown = s2mu004_muic_shutdown,
	.driver = {
		   .name = "s2mu004-muic",
		   .owner = THIS_MODULE,
		   .of_match_table = s2mu004_muic_match_table,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &s2mu004_muic_pm_ops,
#endif
		   },
};

static int __init s2mu004_muic_init(void)
{
	return platform_driver_register(&s2mu004_muic_driver);
}

module_init(s2mu004_muic_init);

static void __exit s2mu004_muic_exit(void)
{
	platform_driver_unregister(&s2mu004_muic_driver);
}

module_exit(s2mu004_muic_exit);

MODULE_DESCRIPTION("Samsung S2MU004 Micro USB IC driver");
MODULE_LICENSE("GPL");
