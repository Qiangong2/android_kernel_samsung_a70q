/*
 * Copyrights (C) 2019 Samsung Electronics, Inc.
 * Copyrights (C) 2019 Silicon Mitus, Inc.
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
 */

#ifndef __PM6150_TYPEC_H__
#define __PM6150_TYPEC_H__

#if defined(CONFIG_CCIC_NOTIFIER)
#include <linux/usb/typec/pdic_notifier.h>
#include <linux/usb/typec/pdic_core.h>
#endif
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
#include <linux/usb/class-dual-role.h>
#elif defined(CONFIG_TYPEC)
#include <linux/usb/typec.h>
#endif
#if defined(CONFIG_IF_CB_MANAGER)
#include <linux/usb/typec/if_cb_manager.h>
#endif
#include <linux/usb/usbpd.h>
#include <linux/power_supply.h>
#include <linux/usb/typec/pm6150/samsung_usbpd.h>

//#define USBPD_DEV_NAME					"usbpd-pm6150"

#define DATA_ROLE_SWAP 1
#define POWER_ROLE_SWAP 2

enum pm6150_power_role {
	PDIC_SINK,
	PDIC_SOURCE
};

enum pm6150_pdic_rid {
	REG_RID_UNDF = 0x00,
	REG_RID_255K = 0x03,
	REG_RID_301K = 0x04,
	REG_RID_523K = 0x05,
	REG_RID_619K = 0x06,
	REG_RID_OPEN = 0x07,
	REG_RID_MAX  = 0x08,
};

/* Function Status from s2mm005 definition */
typedef enum {
	pm6150_State_PE_Initial_detach	= 0,
	pm6150_State_PE_SRC_Send_Capabilities = 3,
	pm6150_State_PE_SNK_Wait_for_Capabilities = 17,
} pm6150_pd_state_t;

typedef enum {
	PLUG_CTRL_RP0 = 0,
	PLUG_CTRL_RP80 = 1,
	PLUG_CTRL_RP180 = 2,
	PLUG_CTRL_RP330 = 3
} CCIC_RP_SCR_SEL;

typedef enum {
	PD_DISABLE = 0,
	PD_ENABLE = 1,
} PD_FUNC_MODE;

typedef enum {
	SBU_SOURCING_OFF = 0,
	SBU_SOURCING_ON = 1,
} ADC_REQUEST_MODE;

typedef enum {
	WATER_MODE_OFF = 0,
	WATER_MODE_ON = 1,
} CCIC_WATER_MODE;

#if defined(CONFIG_SEC_FACTORY)
#define FAC_ABNORMAL_REPEAT_STATE			12
#define FAC_ABNORMAL_REPEAT_RID				5
#define FAC_ABNORMAL_REPEAT_RID0			3
struct AP_REQ_GET_STATUS_Type {
	uint32_t FAC_Abnormal_Repeat_State;
	uint32_t FAC_Abnormal_Repeat_RID;
	uint32_t FAC_Abnormal_RID0;
};
#endif

struct pm6150_phydrv_data {
	struct device *dev;
	struct usbpd *pd;
#if defined(CONFIG_CCIC_NOTIFIER)
	struct workqueue_struct *ccic_wq;
#endif
	struct mutex _mutex;
	struct mutex poll_mutex;
	struct mutex lpm_mutex;
	int vconn_en;
	int irq_gpio;
	int irq;
	int power_role;
	int vbus_state;
	int vbus_tran_st;
	int data_role;
	int vconn_source;
	u64 status_reg;
	bool lpm_mode;
	bool detach_valid;
	bool is_cc_abnormal_state;
	bool is_sbu_abnormal_state;
	bool is_factory_mode;
	bool is_water_detect;
	bool is_otg_vboost;
	bool is_jig_case_on;
	int check_msg_pass;
	int rid;	
	int pd_state;
	int cable;
	int is_attached;
	int reset_done;
	struct delayed_work role_swap_work;
#if defined(CONFIG_SEC_FACTORY)
	struct AP_REQ_GET_STATUS_Type factory_mode;
	struct delayed_work factory_state_work;
	struct delayed_work factory_rid_work;
#endif
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	struct dual_role_phy_instance *dual_role;
	struct dual_role_phy_desc *desc;
	struct completion reverse_completion;
	int data_role_dual;
	int power_role_dual;
	int try_state_change;
#elif defined(CONFIG_TYPEC)
	struct typec_port *port;
	struct typec_partner *partner;
	struct usb_pd_identity partner_identity;
	struct typec_capability typec_cap;
	struct completion typec_reverse_completion;
	int typec_power_role;
	int typec_data_role;
	int typec_try_state_change;
	int pwr_opmode;
	int pd_support;
#endif
#if defined(CONFIG_VBUS_NOTIFIER)
	struct delayed_work vbus_noti_work;
#endif
	struct delayed_work rx_buf_work;
#if defined(CONFIG_IF_CB_MANAGER)
	struct usbpd_dev	*usbpd_d;
	struct if_cb_manager	*man;

#endif

	/* To Support Samsung UVDM Protocol Message */
	uint32_t acc_type;
	uint32_t Vendor_ID;
	uint32_t Product_ID;
	uint32_t Device_Version;
	struct delayed_work	acc_detach_handler;
	uint32_t is_samsung_accessory_enter_mode;
	bool pn_flag;
	struct samsung_usbpd_private *samsung_usbpd;
	uint32_t dr_swap_cnt;

	int num_vdos;
	msg_header_type 	uvdm_msg_header;
	data_obj_type		uvdm_data_obj[USBPD_MAX_COUNT_MSG_OBJECT];
	bool uvdm_first_req;
	bool uvdm_dir;
	struct completion uvdm_out_wait;
	struct completion uvdm_in_wait;
};

extern int samsung_usbpd_uvdm_ready(void);
extern void samsung_usbpd_uvdm_close(void);
extern int samsung_usbpd_uvdm_out_request_message(void *data, int size);
extern int samsung_usbpd_uvdm_in_request_message(void *data);
#if defined(CONFIG_CCIC_NOTIFIER)
extern void pm6150_protocol_layer_reset(void *_data);
extern void pm6150_cc_state_hold_on_off(void *_data, int onoff);
extern bool pm6150_check_vbus_state(void *_data);
void select_pdo(int num);
void pm6150_ccic_event_work(int dest, int id, int attach, int event, int sub);
int pm6150_usbpd_create(struct device* dev, struct pm6150_phydrv_data** parent_data);
int pm6150_usbpd_destroy(void);
#endif
#if defined(CONFIG_TYPEC)
extern int pm6150_get_pd_support(struct pm6150_phydrv_data *usbpd_data);
#endif
void pm6150_set_enable_pd_function(void *_data, int enable);
void pm6150_vbus_turn_on_ctrl(struct pm6150_phydrv_data *usbpd_data, bool enable);
void pm6150_src_transition_to_default(void *_data);
void pm6150_src_transition_to_pwr_on(void *_data);
void pm6150_snk_transition_to_default(void *_data);
#endif /* __PM6150_TYPEC_H__ */
