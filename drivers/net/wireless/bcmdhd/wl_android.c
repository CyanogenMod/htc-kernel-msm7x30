/*
 * Linux cfg80211 driver - Android related functions
 *
 * Copyright (C) 1999-2011, Broadcom Corporation
 * 
 *         Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: wl_android.c,v 1.1.4.1.2.14 2011/02/09 01:40:07 Exp $
 */

#include <linux/module.h>
#include <linux/netdevice.h>

#include <wl_android.h>
#include <wldev_common.h>
#include <wlioctl.h>
#include <bcmutils.h>
#include <linux_osl.h>
#include <dhd_dbg.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <bcmsdbus.h>
/* HTC_CSP_START */
#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif
/* HTC_CSP_END */
#ifdef WL_CFG80211
#include <wl_cfg80211.h>
#endif
#if defined(CONFIG_WIFI_CONTROL_FUNC)
#include <linux/platform_device.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
#include <linux/wlan_plat.h>
#else
#include <linux/wifi_tiwlan.h>
#endif
#endif /* CONFIG_WIFI_CONTROL_FUNC */

/*
 * Android private command strings, PLEASE define new private commands here
 * so they can be updated easily in the future (if needed)
 */

#define CMD_START			"START"
#define CMD_STOP			"STOP"
#define CMD_SCAN_ACTIVE			"SCAN-ACTIVE"
#define CMD_SCAN_PASSIVE		"SCAN-PASSIVE"
#define CMD_RSSI			"RSSI"
#define CMD_LINKSPEED			"LINKSPEED"
#define CMD_RXFILTER_START		"RXFILTER-START"
#define CMD_RXFILTER_STOP		"RXFILTER-STOP"
#define CMD_RXFILTER_ADD		"RXFILTER-ADD"
#define CMD_RXFILTER_REMOVE		"RXFILTER-REMOVE"
#define CMD_BTCOEXSCAN_START		"BTCOEXSCAN-START"
#define CMD_BTCOEXSCAN_STOP		"BTCOEXSCAN-STOP"
#define CMD_BTCOEXMODE			"BTCOEXMODE"
#define CMD_SETSUSPENDOPT		"SETSUSPENDOPT"
#define CMD_MAC_ADDR		"MACADDR"
#define CMD_P2P_DEV_ADDR		"P2P_DEV_ADDR"
#define CMD_SETFWPATH			"SETFWPATH"
#define CMD_SETBAND			"SETBAND"
#define CMD_GETBAND			"GETBAND"
#define CMD_COUNTRY			"COUNTRY"
#define CMD_P2P_SET_NOA			"P2P_SET_NOA"
#define CMD_P2P_GET_NOA			"P2P_GET_NOA"
#define CMD_P2P_SET_PS			"P2P_SET_PS"
#define CMD_SET_AP_WPS_P2P_IE 		"SET_AP_WPS_P2P_IE"
#define CMD_P2P_SET_MPC 		"P2P_MPC_SET"
#define CMD_DEAUTH_STA 		"DEAUTH_STA"

#define CMD_DTIM_SKIP_SET	"DTIMSKIPSET"
#define CMD_DTIM_SKIP_GET	"DTIMSKIPGET"
#define CMD_TXPOWER_SET		"TXPOWER"
#define CMD_POWER_MODE_SET	"POWERMODE"
#define CMD_POWER_MODE_GET	"GETPOWER"
#define CMD_AP_TXPOWER_SET	"AP_TXPOWER_SET"
#define CMD_AP_ASSOC_LIST_GET	"AP_ASSOC_LIST_GET"
#define CMD_AP_MAC_LIST_SET	"AP_MAC_LIST_SET"
#define CMD_SCAN_MINRSSI_SET	"SCAN_MINRSSI"
#define CMD_LOW_RSSI_SET	"LOW_RSSI_IND_SET"

/* HTC_CSP_START */
#define CMD_GETWIFILOCK		"GETWIFILOCK"
#define CMD_SETWIFICALL		"WIFICALL"
#define CMD_SETPROJECT		"SET_PROJECT"
#define CMD_SETLOWPOWERMODE		"SET_LOWPOWERMODE"
#define CMD_GATEWAYADD		"GATEWAY-ADD"
/* HTC_CSP_END */

#ifdef PNO_SUPPORT
#define CMD_PNOSSIDCLR_SET	"PNOSSIDCLR"
#define CMD_PNOSETUP_SET	"PNOSETUP "
#define CMD_PNOENABLE_SET	"PNOFORCE"
#define CMD_PNODEBUG_SET	"PNODEBUG"
#define CMD_PFN_REMOVE		"PFN_REMOVE"

#define PNO_TLV_PREFIX			'S'
#define PNO_TLV_VERSION			'1'
#define PNO_TLV_SUBVERSION 		'2'
#define PNO_TLV_RESERVED		'0'
#define PNO_TLV_TYPE_SSID_IE		'S'
#define PNO_TLV_TYPE_TIME		'T'
#define PNO_TLV_FREQ_REPEAT		'R'
#define PNO_TLV_FREQ_EXPO_MAX		'M'

/* HTC specific command */
#define CMD_GET_AUTO_CHANNEL	"AUTOCHANNELGET"

typedef struct cmd_tlv {
	char prefix;
	char version;
	char subver;
	char reserved;
} cmd_tlv_t;
#endif /* PNO_SUPPORT */

typedef struct android_wifi_priv_cmd {
	char *buf;
	int used_len;
	int total_len;
} android_wifi_priv_cmd;

#define htod32(i) i
/**
 * Extern function declarations (TODO: move them to dhd_linux.h)
 */
void dhd_customer_gpio_wlan_ctrl(int onoff);
uint dhd_dev_reset(struct net_device *dev, uint8 flag);
void dhd_dev_init_ioctl(struct net_device *dev);
#ifdef WL_CFG80211
int wl_cfg80211_get_mac_addr(struct net_device *net, struct ether_addr *mac_addr);
int wl_cfg80211_get_p2p_dev_addr(struct net_device *net, struct ether_addr *p2pdev_addr);
int wl_cfg80211_set_btcoex_dhcp(struct net_device *dev, char *command);
#else
int wl_cfg80211_get_p2p_dev_addr(struct net_device *net, struct ether_addr *p2pdev_addr)
{ return 0; }
int wl_cfg80211_set_p2p_noa(struct net_device *net, char* buf, int len)
{ return 0; }
int wl_cfg80211_get_p2p_noa(struct net_device *net, char* buf, int len)
{ return 0; }
int wl_cfg80211_set_p2p_ps(struct net_device *net, char* buf, int len)
{ return 0; }
int wl_cfg80211_set_mpc(struct net_device *net, char* buf, int len)
{ return 0; }
int wl_cfg80211_deauth_sta(struct net_device *net, char* buf, int len)
{ return 0; }

#endif
extern int dhd_os_check_if_up(void *dhdp);
extern void *bcmsdh_get_drvdata(void);

extern bool ap_fw_loaded;
#ifdef CUSTOMER_HW2
extern char iface_name[IFNAMSIZ];
#endif

/**
 * Local (static) functions and variables
 */

/* Initialize g_wifi_on to 1 so dhd_bus_start will be called for the first
 * time (only) in dhd_open, subsequential wifi on will be handled by
 * wl_android_wifi_on
 */
static int g_wifi_on = TRUE;
int block_ap_event = 0;

/**
 * Local (static) function definitions
 */
static int wl_android_get_link_speed(struct net_device *net, char *command, int total_len)
{
	int link_speed;
	int bytes_written;
	int error;

	error = wldev_get_link_speed(net, &link_speed);
	if (error)
		return -1;

	/* Convert Kbps to Android Mbps */
	link_speed = link_speed / 1000;
	bytes_written = snprintf(command, total_len, "LinkSpeed %d", link_speed);
	DHD_INFO(("%s: command result is %s\n", __FUNCTION__, command));
	return bytes_written;
}

/* HTC_CSP_START */
/* traffic indicate parameters */
/* framework will obtain RSSI every 3000 ms*/
/* The throughput mapping to packet count is as below:
 *  2Mbps: ~280 packets / second
 *  4Mbps: ~540 packets / second
 *  6Mbps: ~800 packets / second
 *  8Mbps: ~1200 packets / second
 * 12Mbps: ~1500 packets / second
 * 14Mbps: ~1800 packets / second
 * 16Mbps: ~2000 packets / second
 * 18Mbps: ~2300 packets / second
 * 20Mbps: ~2600 packets / second
 */
#define TRAFFIC_HIGH_WATER_MARK	        2300*(3000/1000)
#define TRAFFIC_LOW_WATER_MARK          256*(3000/1000)
typedef enum traffic_ind {
	TRAFFIC_STATS_HIGH = 0,
	TRAFFIC_STATS_NORMAL,
} traffic_ind_t;


static int traffic_stats_flag = TRAFFIC_STATS_NORMAL;
static unsigned long current_traffic_count = 0;
static unsigned long last_traffic_count = 0;
extern struct perf_lock wlan_perf_lock;

static void wl_android_traffic_monitor(struct net_device *dev)
{
	unsigned long rx_packets_count = 0;
	unsigned long tx_packets_count = 0;
	unsigned long traffic_diff = 0;

	/*for Traffic High/Low indication */
	dhd_get_txrx_stats(dev, &rx_packets_count, &tx_packets_count);
	current_traffic_count = rx_packets_count + tx_packets_count;

	if (current_traffic_count >= last_traffic_count) {
		traffic_diff = current_traffic_count - last_traffic_count;
		if (traffic_stats_flag == TRAFFIC_STATS_NORMAL) {
			if (traffic_diff > TRAFFIC_HIGH_WATER_MARK) {
				traffic_stats_flag = TRAFFIC_STATS_HIGH;
#ifdef CONFIG_PERFLOCK
				if (!is_perf_lock_active(&wlan_perf_lock))
					perf_lock(&wlan_perf_lock);
#endif
				printf("lock cpu here, traffic-count=%ld\n", traffic_diff / 3);
			}
		} else {
			if (traffic_diff < TRAFFIC_LOW_WATER_MARK) {
				traffic_stats_flag = TRAFFIC_STATS_NORMAL;
#ifdef CONFIG_PERFLOCK
				if (is_perf_lock_active(&wlan_perf_lock))
					perf_unlock(&wlan_perf_lock);
#endif
				printf("unlock cpu here, traffic-count=%ld\n", traffic_diff / 3);
			}
		}
	}
	last_traffic_count = current_traffic_count;
	/*End of Traffic High/Low indication */
}
/* HTC_CSP_END */

static int wl_android_get_rssi(struct net_device *net, char *command, int total_len)
{
	wlc_ssid_t ssid = {0};
	int rssi;
	int bytes_written = 0;
	int error;

	error = wldev_get_rssi(net, &rssi);
	if (error)
		return -1;

	error = wldev_get_ssid(net, &ssid);
	if (error)
		return -1;
	if ((ssid.SSID_len == 0) || (ssid.SSID_len > DOT11_MAX_SSID_LEN)) {
		DHD_TRACE(("%s: wldev_get_ssid failed\n", __FUNCTION__));
	} else {
		memcpy(command, ssid.SSID, ssid.SSID_len);
		bytes_written = ssid.SSID_len;
	}
	bytes_written += snprintf(&command[bytes_written], total_len, " rssi %d", rssi);
	DHD_INFO(("%s: command result is %s (%d)\n", __FUNCTION__, command, bytes_written));
/* HTC_CSP_START */
	wl_android_traffic_monitor(net);
/* HTC_CSP_END */
	return bytes_written;
}

static int wl_android_set_suspendopt(struct net_device *dev, char *command, int total_len)
{
	int suspend_flag;
	int ret_now;
	int ret = 0;

	suspend_flag = *(command + strlen(CMD_SETSUSPENDOPT) + 1) - '0';

	if (suspend_flag != 0)
		suspend_flag = 1;
	ret_now = net_os_set_suspend_disable(dev, suspend_flag);

	if (ret_now != suspend_flag) {
		if (!(ret = net_os_set_suspend(dev, ret_now)))
			DHD_INFO(("%s: Suspend Flag %d -> %d\n",
				__FUNCTION__, ret_now, suspend_flag));
		else
			DHD_ERROR(("%s: failed %d\n", __FUNCTION__, ret));
	}
	return ret;
}

static int wl_android_get_band(struct net_device *dev, char *command, int total_len)
{
	uint band;
	int bytes_written;
	int error;

	error = wldev_get_band(dev, &band);
	if (error)
		return -1;
	bytes_written = snprintf(command, total_len, "Band %d", band);
	return bytes_written;
}

#if 0
static int wl_android_set_active_scan(struct net_device *dev, char *command, int total_len)
{
	uint as = 0;
	int bytes_written;
	int error = 0;

	error = wldev_ioctl(dev, WLC_SET_PASSIVE_SCAN, &as, sizeof(as), 0);
	if (error)
		return -1;
	bytes_written = snprintf(command, total_len, "OK");
	return bytes_written;
}

static int wl_android_set_passive_scan(struct net_device *dev, char *command, int total_len)
{
	uint ps = 1;
	int bytes_written;
	int error = 0;

	error = wldev_ioctl(dev, WLC_SET_PASSIVE_SCAN, &ps, sizeof(ps), 1);
	if (error)
		return -1;
	bytes_written = snprintf(command, total_len, "OK");
	return bytes_written;
}
#endif

static int wl_android_get_dtim_skip(struct net_device *dev, char *command, int total_len)
{
	char iovbuf[32];
	int bytes_written;
	int error = 0;

	memset(iovbuf, 0, sizeof(iovbuf));
	strcpy(iovbuf, "bcn_li_dtim");
	
	if ((error = wldev_ioctl(dev, WLC_GET_VAR, &iovbuf, sizeof(iovbuf), 0)) >= 0) {
		bytes_written = snprintf(command, total_len, "Dtim_skip %d", iovbuf[0]);
		DHD_INFO(("%s: get dtim_skip = %d\n", __FUNCTION__, iovbuf[0]));
		return bytes_written;
	}
	else  {
		DHD_ERROR(("%s: get dtim_skip failed code %d\n", __FUNCTION__, error));
		return -1;
	}
}

static int wl_android_set_dtim_skip(struct net_device *dev, char *command, int total_len)
{
	char iovbuf[32];
	int bytes_written;
	int error = -1;
	int bcn_li_dtim;

	bcn_li_dtim = htod32((uint)*(command + strlen(CMD_DTIM_SKIP_SET) + 1) - '0');
	
	if ((bcn_li_dtim >= 0) || ((bcn_li_dtim <= 5))) {
		memset(iovbuf, 0, sizeof(iovbuf));
		bcm_mkiovar("bcn_li_dtim", (char *)&bcn_li_dtim, 4, iovbuf, sizeof(iovbuf));
	
		if ((error = wldev_ioctl(dev, WLC_SET_VAR, &iovbuf, sizeof(iovbuf), 1)) >= 0) {
			bytes_written = snprintf(command, total_len, "Dtim_skip %d", iovbuf[0]);
			DHD_INFO(("%s: set dtim_skip = %d\n", __FUNCTION__, iovbuf[0]));
			return bytes_written;
		}
		else {
			DHD_ERROR(("%s: set dtim_skip failed code %d\n", __FUNCTION__, error));
		}
	}
	else {
		DHD_ERROR(("%s Incorrect dtim_skip setting %d, ignored\n", __FUNCTION__, bcn_li_dtim));
	}
	return -1;
}

static int wl_android_set_txpower(struct net_device *dev, char *command, int total_len)
{
	int bytes_written;
	int error = -1;
	int txpower = -1;

	txpower = bcm_atoi(command + strlen(CMD_TXPOWER_SET) + 1);
	
	if ((txpower >= 0) || ((txpower <= 127))) {
		txpower |= WL_TXPWR_OVERRIDE;
		txpower = htod32(txpower);
		if ((error = wldev_iovar_setint(dev, "qtxpower", txpower)) >= 0)  {
			bytes_written = snprintf(command, total_len, "OK");
        	        DHD_INFO(("%s: set TXpower 0x%X is OK\n", __FUNCTION__, txpower));
			return bytes_written;
		}
		else {
                	DHD_ERROR(("%s: set tx power failed, txpower=%d\n", __FUNCTION__, txpower));
		}
        } else {
                DHD_ERROR(("%s: Unsupported tx power value, txpower=%d\n", __FUNCTION__, txpower));
        }

	bytes_written = snprintf(command, total_len, "FAIL");
	return -1;
}

static int wl_android_set_ap_txpower(struct net_device *dev, char *command, int total_len)
{
	int ap_txpower = 0, ap_txpower_orig = 0;
	char iovbuf[32];

	ap_txpower = bcm_atoi(command + strlen(CMD_AP_TXPOWER_SET) + 1);
	if (ap_txpower == 0) {
		/* get the default txpower */
		memset(iovbuf, 0, sizeof(iovbuf));
		bcm_mkiovar("qtxpower", (char *)&ap_txpower_orig, 4, iovbuf, sizeof(iovbuf));
		wldev_ioctl(dev, WLC_GET_VAR, &iovbuf, sizeof(iovbuf), 0);
		DHD_ERROR(("default tx power is %d\n", ap_txpower_orig));
		ap_txpower_orig |= WL_TXPWR_OVERRIDE;
	}

	if (ap_txpower == 99) {
		/* set to default value */
		ap_txpower = ap_txpower_orig;
	} else {
		ap_txpower |= WL_TXPWR_OVERRIDE;
	}

	DHD_ERROR(("set tx power to 0x%x\n", ap_txpower));
	bcm_mkiovar("qtxpower", (char *)&ap_txpower, 4, iovbuf, sizeof(iovbuf));
	wldev_ioctl(dev, WLC_SET_VAR, &iovbuf, sizeof(iovbuf), 1);

	return 0;
}

static int wl_android_set_scan_minrssi(struct net_device *dev, char *command, int total_len)
{
	int minrssi = 0;
	int err = 0;
	int bytes_written;

	DHD_TRACE(("%s\n", __FUNCTION__));

	minrssi = bcm_strtoul((char *)(command + strlen("SCAN_MINRSSI") + 1), NULL, 10);

	err = wldev_iovar_setint(dev, "scanresults_minrssi", minrssi);

	if (err) {
		DHD_ERROR(("set scan_minrssi fail!\n"));
		bytes_written = snprintf(command, total_len, "FAIL");
	} else
		bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;
}

#ifdef WLC_E_RSSI_LOW 
static int wl_android_low_rssi_set(struct net_device *dev, char *command, int total_len)
{
	int low_rssi_trigger;
	int low_rssi_duration;
	int trigger_offset;
	int duration_offset;
	char tran_buf[16] = {0};
	char *pp;
	int err = 0;
	int bytes_written;
	
	DHD_TRACE(("%s\n", __FUNCTION__));

	trigger_offset = strcspn(command, " ");
	pp = command + trigger_offset + 1;
	duration_offset = strcspn(pp, " ");

	memcpy(tran_buf, pp, duration_offset);
	low_rssi_trigger = bcm_strtoul(tran_buf, NULL, 10);
	err |= wldev_iovar_setint(dev, "low_rssi_trigger", low_rssi_trigger);

	memset(tran_buf, 0, 16);
	pp = command + trigger_offset + duration_offset + 2;
	memcpy(tran_buf, pp, strlen(command) - (trigger_offset+duration_offset+1));
	low_rssi_duration = bcm_strtoul(tran_buf, NULL, 10);
	err |= wldev_iovar_setint(dev, "low_rssi_duration", low_rssi_duration);

	DHD_TRACE(("set low rssi trigger %d, duration %d\n", low_rssi_trigger, low_rssi_duration));

	if (err) {
		DHD_ERROR(("set low rssi ind fail!\n"));
		bytes_written = snprintf(command, total_len, "FAIL");
	} else
		bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;
}
#endif /* WLC_E_RSSI_LOW */ 


#if 0
static int wl_android_process_private_ascii_cmd(struct net_device *dev, char *cmd_str, int total_len)
{
	int ret = 0;
	char *sub_cmd = cmd_str + PROFILE_OFFSET + strlen("ASCII_CMD=");

	WL_SOFTAP(("\n %s: ASCII_CMD: offs_0:%s, offset_32:\n'%s'\n",
		__FUNCTION__, cmd_str, cmd_str + PROFILE_OFFSET));

	if ( (strnicmp(sub_cmd, "AP_CFG", strlen("AP_CFG")) == 0) ||
		(strnicmp(sub_cmd, "APSTA_CFG", strlen("APSTA_CFG")) == 0)) {
		/* broadcom, add apsta cfg detection above. */

		WL_SOFTAP((" AP_CFG or APSTA_CFG\n"));

		
		if (init_ap_profile_from_string(cmd_str+PROFILE_OFFSET, &my_ap) != 0) {
				WL_ERROR(("ERROR: SoftAP CFG prams !\n"));
				ret = -1;
		} else {
#ifndef AP_ONLY
			if ( apsta_enable ) 
				ret = set_apsta_cfg(dev, &my_ap);
			else
#endif
				ret = set_ap_cfg(dev, &my_ap);
		}

	} else if (strnicmp(sub_cmd, "AP_BSS_START", strlen("AP_BSS_START")) == 0) {

		WL_SOFTAP(("\n SOFTAP - ENABLE BSS \n"));
		
		WL_SOFTAP(("\n!!! got 'WL_AP_EN_BSS' from WPA supplicant, dev:%s\n", dev->name));

#ifndef AP_ONLY
		if (ap_net_dev == NULL) {
				 printf("\n ERROR: SOFTAP net_dev* is NULL !!!\n");
		} else {
			  
			if ((ret = iwpriv_en_ap_bss(ap_net_dev, info, dwrq, cmd_str)) < 0)
				WL_ERROR(("%s line %d fail to set bss up\n",
					__FUNCTION__, __LINE__));
		}
#else
		if ((ret = iwpriv_en_ap_bss(dev, info, dwrq, cmd_str)) < 0)
				WL_ERROR(("%s line %d fail to set bss up\n",
					__FUNCTION__, __LINE__));
#endif
	} else if (strnicmp(sub_cmd, "ASSOC_LST", strlen("ASSOC_LST")) == 0) {

		

	} else if (strnicmp(sub_cmd, "AP_BSS_STOP", strlen("AP_BSS_STOP")) == 0) {

		WL_SOFTAP((" \n temp DOWN SOFTAP\n"));
#ifndef AP_ONLY
		if ((ret = dev_iw_write_cfg1_bss_var(dev, 0)) < 0) {
				WL_ERROR(("%s line %d fail to set bss down\n",
					__FUNCTION__, __LINE__));
		}
#endif
	}

	return ret;

}
#endif

static struct mac_list_set android_mac_list_buf;
static struct mflist android_ap_black_list;
static int android_ap_macmode = MACLIST_MODE_DISABLED;
int wl_android_black_list_match(char *mac)
{
	int i;

	if (android_ap_macmode) {
		for (i = 0; i < android_ap_black_list.count; i++) {
			if (!bcmp(mac, &android_ap_black_list.ea[i],
				sizeof(struct ether_addr))) {
				DHD_ERROR(("mac in black list, ignore it\n"));
				break;
			}
		}

		if (i == android_ap_black_list.count)
			return 1;
	}

	return 0;
}

static int
wl_android_get_assoc_sta_list(struct net_device *dev, char *buf, int len)
{
	struct maclist *maclist = (struct maclist *) buf;
	int ret,i;
	char mac_lst[256];
	char *p_mac_str;

	bcm_mdelay(500);
	maclist->count = 10;
	ret = wldev_ioctl(dev, WLC_GET_ASSOCLIST, buf, len, 0);

	memset(mac_lst, 0, sizeof(mac_lst));
	p_mac_str = mac_lst;

	/* format: "count|sta 1, sta2, ..."
	*/

	p_mac_str += snprintf(p_mac_str, 80, "%d|", maclist->count);

	for (i = 0; i < maclist->count; i++) {
		struct ether_addr *id = &maclist->ea[i];


		p_mac_str += snprintf(p_mac_str, 80, "%02X:%02X:%02X:%02X:%02X:%02X,",
			id->octet[0], id->octet[1], id->octet[2],
			id->octet[3], id->octet[4], id->octet[5]);
	}

	if (ret != 0) {
		DHD_ERROR(("get assoc count fail\n"));
		maclist->count = 0;
	} else
		printf("get assoc count %d, len %d\n", maclist->count, len);

	memset(buf, 0x0, len);
	memcpy(buf, mac_lst, len);
	return len;
}

static int wl_android_set_ap_mac_list(struct net_device *dev, void *buf)
{
	struct mac_list_set *mac_list_set = (struct mac_list_set *)buf;
	struct maclist *white_maclist = (struct maclist *)&mac_list_set->white_list;
	struct maclist *black_maclist = (struct maclist *)&mac_list_set->black_list;
	int mac_mode = mac_list_set->mode;
	int length;
	int i;

	printf("%s in\n", __func__);
	if (white_maclist->count > 16 || black_maclist->count > 16) {
		DHD_TRACE(("invalid count white: %d black: %d\n", white_maclist->count, black_maclist->count));
		return 0;
	}

	if (buf != (char *)&android_mac_list_buf) {
		DHD_TRACE(("Backup the mac list\n"));
		memcpy((char *)&android_mac_list_buf, buf, sizeof(struct mac_list_set));
	} else
		DHD_TRACE(("recover, don't back up mac list\n"));

	android_ap_macmode = mac_mode;
	if (mac_mode == MACLIST_MODE_DISABLED) {

		bzero(&android_ap_black_list, sizeof(struct mflist));

		wldev_ioctl(dev, WLC_SET_MACMODE, &mac_mode, sizeof(mac_mode), 1);
	} else {
		scb_val_t scbval;
		char mac_buf[256] = {0};
		struct maclist *assoc_maclist = (struct maclist *) mac_buf;

		mac_mode = MACLIST_MODE_ALLOW;

		wldev_ioctl(dev, WLC_SET_MACMODE, &mac_mode, sizeof(mac_mode), 1);

		length = sizeof(white_maclist->count)+white_maclist->count*ETHER_ADDR_LEN;
		wldev_ioctl(dev, WLC_SET_MACLIST, white_maclist, length, 1);
		printf("White List, length %d:\n", length);
		for (i = 0; i < white_maclist->count; i++)
			printf("mac %d: %02X:%02X:%02X:%02X:%02X:%02X\n",
				i, white_maclist->ea[i].octet[0], white_maclist->ea[i].octet[1], white_maclist->ea[i].octet[2],
				white_maclist->ea[i].octet[3], white_maclist->ea[i].octet[4], white_maclist->ea[i].octet[5]);

		/* set the black list */
		bcopy(black_maclist, &android_ap_black_list, sizeof(android_ap_black_list));

		printf("Black List, size %d:\n", sizeof(android_ap_black_list));
		for (i = 0; i < android_ap_black_list.count; i++)
			printf("mac %d: %02X:%02X:%02X:%02X:%02X:%02X\n",
				i, android_ap_black_list.ea[i].octet[0], android_ap_black_list.ea[i].octet[1], android_ap_black_list.ea[i].octet[2],
				android_ap_black_list.ea[i].octet[3], android_ap_black_list.ea[i].octet[4], android_ap_black_list.ea[i].octet[5]);

		/* deauth if there is associated station not in list */
		assoc_maclist->count = 10;
		wldev_ioctl(dev, WLC_GET_ASSOCLIST, assoc_maclist, 256, 0);
		if (assoc_maclist->count) {
			int j;
			for (i = 0; i < assoc_maclist->count; i++) {
				for (j = 0; j < white_maclist->count; j++) {
					if (!bcmp(&assoc_maclist->ea[i], &white_maclist->ea[j], ETHER_ADDR_LEN)) {
						DHD_TRACE(("match allow, let it be\n"));
						break;
					}
				}
				if (j == white_maclist->count) {
						DHD_TRACE(("match black, deauth it\n"));
						scbval.val = htod32(1);
						bcopy(&assoc_maclist->ea[i], &scbval.ea, ETHER_ADDR_LEN);
						wldev_ioctl(dev, WLC_SCB_DEAUTHENTICATE_FOR_REASON, &scbval,
							sizeof(scb_val_t), 1);
				}
			}
		}
	}
	return 0;
}


#ifdef PNO_SUPPORT
static int wl_android_set_pno_setup(struct net_device *dev, char *command, int total_len)
{
	wlc_ssid_t ssids_local[MAX_PFN_LIST_COUNT];
	int res = -1;
	int nssid = 0;
	cmd_tlv_t *cmd_tlv_temp;
	char *str_ptr;
	int tlv_size_left;
	int pno_time = 0;
	int pno_repeat = 0;
	int pno_freq_expo_max = 0;

#ifdef PNO_SET_DEBUG
	int i;
	char pno_in_example[] = {
		'P', 'N', 'O', 'S', 'E', 'T', 'U', 'P', ' ',
		'S', '1', '2', '0',
		'S',
		0x05,
		'd', 'l', 'i', 'n', 'k',
		'S',
		0x04,
		'G', 'O', 'O', 'G',
		'T',
		'0', 'B',
		'R',
		'2',
		'M',
		'2',
		0x00
		};
#endif /* PNO_SET_DEBUG */

	DHD_INFO(("%s: command=%s, len=%d\n", __FUNCTION__, command, total_len));

	if (total_len < (strlen(CMD_PNOSETUP_SET) + sizeof(cmd_tlv_t))) {
		DHD_ERROR(("%s argument=%d less min size\n", __FUNCTION__, total_len));
		goto exit_proc;
	}


#ifdef PNO_SET_DEBUG
	memcpy(command, pno_in_example, sizeof(pno_in_example));
	for (i = 0; i < sizeof(pno_in_example); i++)
		printf("%02X ", command[i]);
	printf("\n");
	total_len = sizeof(pno_in_example);
#endif

	str_ptr = command + strlen(CMD_PNOSETUP_SET);
	tlv_size_left = total_len - strlen(CMD_PNOSETUP_SET);

	cmd_tlv_temp = (cmd_tlv_t *)str_ptr;
	memset(ssids_local, 0, sizeof(ssids_local));

	if ((cmd_tlv_temp->prefix == PNO_TLV_PREFIX) &&
		(cmd_tlv_temp->version == PNO_TLV_VERSION) &&
		(cmd_tlv_temp->subver == PNO_TLV_SUBVERSION)) {

		str_ptr += sizeof(cmd_tlv_t);
		tlv_size_left -= sizeof(cmd_tlv_t);

		if ((nssid = wl_iw_parse_ssid_list_tlv(&str_ptr, ssids_local,
			MAX_PFN_LIST_COUNT, &tlv_size_left)) <= 0) {
			DHD_ERROR(("SSID is not presented or corrupted ret=%d\n", nssid));
			goto exit_proc;
		} else {
			if ((str_ptr[0] != PNO_TLV_TYPE_TIME) || (tlv_size_left <= 1)) {
				DHD_ERROR(("%s scan duration corrupted field size %d\n",
					__FUNCTION__, tlv_size_left));
				goto exit_proc;
			}
			str_ptr++;
			pno_time = simple_strtoul(str_ptr, &str_ptr, 16);
			DHD_INFO(("%s: pno_time=%d\n", __FUNCTION__, pno_time));

			if (str_ptr[0] != 0) {
				if ((str_ptr[0] != PNO_TLV_FREQ_REPEAT)) {
					DHD_ERROR(("%s pno repeat : corrupted field\n",
						__FUNCTION__));
					goto exit_proc;
				}
				str_ptr++;
				pno_repeat = simple_strtoul(str_ptr, &str_ptr, 16);
				DHD_INFO(("%s :got pno_repeat=%d\n", __FUNCTION__, pno_repeat));
				if (str_ptr[0] != PNO_TLV_FREQ_EXPO_MAX) {
					DHD_ERROR(("%s FREQ_EXPO_MAX corrupted field size\n",
						__FUNCTION__));
					goto exit_proc;
				}
				str_ptr++;
				pno_freq_expo_max = simple_strtoul(str_ptr, &str_ptr, 16);
				DHD_INFO(("%s: pno_freq_expo_max=%d\n",
					__FUNCTION__, pno_freq_expo_max));
			}
		}
	} else {
		DHD_ERROR(("%s get wrong TLV command\n", __FUNCTION__));
		goto exit_proc;
	}

	res = dhd_dev_pno_set(dev, ssids_local, nssid, pno_time, pno_repeat, pno_freq_expo_max);

exit_proc:
	return res;
}

static int wl_android_del_pfn(struct net_device *dev, char *command, int total_len)
{
	char ssid[33];
	int ssid_offset;
	int ssid_size;
	int bytes_written;

	DHD_TRACE(("%s\n", __FUNCTION__));

	memset(ssid, 0, sizeof(ssid));

	ssid_offset = strcspn(command, " ");
	ssid_size = strlen(command) - ssid_offset;

	if (ssid_offset == 0) {
		DHD_ERROR(("%s, no ssid specified\n", __FUNCTION__));
		return 0;
	}

	strncpy(ssid, command + ssid_offset+1,
			MIN(ssid_size, sizeof(ssid)));

	DHD_ERROR(("%s: remove ssid: %s\n", __FUNCTION__, ssid));
	dhd_del_pfn_ssid(ssid, ssid_size);

	bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;

}

#endif /* PNO_SUPPORT */

#ifdef WL_CFG80211
static int wl_android_get_mac_addr(struct net_device *ndev, char *command, int total_len)
{
	int ret;
	int bytes_written = 0;
	struct ether_addr id;

	ret = wl_cfg80211_get_mac_addr(ndev, &id);
	if (ret)
		return 0;
	bytes_written = snprintf(command, total_len, "Macaddr = %02X:%02X:%02X:%02X:%02X:%02X\n",
		id.octet[0], id.octet[1], id.octet[2],
		id.octet[3], id.octet[4], id.octet[5]);
	return bytes_written;
}
#endif

static int wl_android_get_p2p_dev_addr(struct net_device *ndev, char *command, int total_len)
{
	int ret;
	int bytes_written = 0;

	ret = wl_cfg80211_get_p2p_dev_addr(ndev, (struct ether_addr*)command);
	if (ret)
		return 0;
	bytes_written = sizeof(struct ether_addr);
	return bytes_written;
}

/* HTC_CSP_START */
extern int dhdcdc_wifiLock;
extern char project_type[33];
static int wl_android_wifi_call = 0;
static struct mutex wl_wificall_mutex;
static struct mutex wl_wifionoff_mutex;
char wl_abdroid_gatewaybuf[8+1]; /* HTC_KlocWork */
#ifdef BCM4329_LOW_POWER
extern int LowPowerMode;
extern bool hasDLNA;
extern bool allowMulticast;
extern int dhd_set_keepalive(int value);
#endif

static int active_level = -80;
static int active_period = 20000; /*in mini secs*/
static int wl_android_active_expired = 0;
struct timer_list *wl_android_active_timer = NULL;
static int screen_off = 0;

static void wl_android_act_time_expire(void)
{
	struct timer_list **timer;
	timer = &wl_android_active_timer;

	if (*timer) {
		WL_TRACE(("ac timer expired\n"));
		del_timer_sync(*timer);
		kfree(*timer);
		*timer = NULL;
		if (screen_off)
			return;
		wl_android_active_expired = 1;
	}
	return;
}

static void wl_android_deactive(void)
{
	struct timer_list **timer;
	timer = &wl_android_active_timer;

	if (*timer) {
		WL_TRACE(("previous ac exist\n"));
		del_timer_sync(*timer);
		kfree(*timer);
		*timer = NULL;
	}
	wl_android_active_expired = 0;
	WL_TRACE(("wl_android_deactive\n"));
	return;
}

void wl_android_set_active_level(int level)
{
	active_level = level;
	printf("set active level to %d\n", active_level);
	return;
}

void wl_android_set_active_period(int period)
{
	active_period = period;
	printf("set active_period to %d\n", active_period);
	return;
}

int wl_android_get_active_level(void)
{
	return active_level;
}

int wl_android_get_active_period(void)
{
	return active_period;
}

void wl_android_set_screen_off(int off)
{
	screen_off = off;
	printf("wl_android_set_screen_off %d\n", screen_off);
	if (screen_off)
		wl_android_deactive();

	return;
}

static int wl_android_set_power_mode(struct net_device *dev, char *command, int total_len)
{

/* HTC_CSP_START */
#if 1
	int bytes_written;
	int pm_val;

	pm_val = bcm_atoi(command + strlen(CMD_POWER_MODE_SET) + 1);

		switch (pm_val) {
		/* Android normal usage */
		case 0:
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_ANDROID_NORMAL);
			dhdhtc_update_wifi_power_mode(screen_off);
			/*btcoex_dhcp_timer_stop(dev);*/
			break;
		case 1:
			dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_ANDROID_NORMAL);
			dhdhtc_update_wifi_power_mode(screen_off);
			/*btcoex_dhcp_timer_start(dev);*/
			break;
		 /* for web loading page */
		case 10:
			wl_android_deactive();
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_BROWSER_LOAD_PAGE);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		case 11:
			if (!screen_off) {
				struct timer_list **timer;
				timer = &wl_android_active_timer;

				if (*timer) {
					mod_timer(*timer, jiffies + active_period * HZ / 1000);
					/* printf("mod_timer\n"); */
				} else {
					*timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
					if (*timer) {
						(*timer)->function = (void *)wl_android_act_time_expire;
						init_timer(*timer);
						(*timer)->expires = jiffies + active_period * HZ / 1000;
						add_timer(*timer);
						/* printf("add_timer\n"); */
					}
				}
				dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_BROWSER_LOAD_PAGE);
				dhdhtc_update_wifi_power_mode(screen_off);
			}
			break;
		 /* for best performance configure */
		case 20:
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_USER_CONFIG);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		case 21:
			dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_USER_CONFIG);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		 /* for fota download */
		case 30:
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_FOTA_DOWNLOADING);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		case 31:
			dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_FOTA_DOWNLOADING);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;

		case 87: /* For wifilock release*/
			printf("wifilock release\n");
			dhdcdc_wifiLock = 0;
			dhdhtc_update_wifi_power_mode(screen_off);
			dhdhtc_update_dtim_listen_interval(screen_off);
			break;

		case 88: /* For wifilock lock*/
			printf("wifilock accquire\n");
			dhdcdc_wifiLock = 1;
			dhdhtc_update_wifi_power_mode(screen_off);
			dhdhtc_update_dtim_listen_interval(screen_off);
			break;

		case 99: /* For debug. power active or not while usb plugin */
			dhdcdc_power_active_while_plugin = !dhdcdc_power_active_while_plugin;
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
#if 0
		/* for wifi phone */
		case 30:
			dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_WIFI_PHONE);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
		case 31:
			dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_WIFI_PHONE);
			dhdhtc_update_wifi_power_mode(screen_off);
			break;
#endif
		default:
			DHD_ERROR(("%s: not support mode: %d\n", __func__, pm_val));
			break;

	}

#else

	static int  pm = PM_FAST;
	int  pm_local = PM_OFF;
	char powermode_val = 0;
	int bytes_written;

	DHD_INFO(("%s: DHCP session cmd:%s\n", __func__, command));

	strncpy((char *)&powermode_val, command + strlen("POWERMODE") + 1, 1);

	if (strnicmp((char *)&powermode_val, "1", strlen("1")) == 0) {

		WL_TRACE(("%s: DHCP session starts\n", __func__));
		wldev_ioctl(dev, WLC_GET_PM, &pm, sizeof(pm), 0);
		wldev_ioctl(dev, WLC_SET_PM, &pm_local, sizeof(pm_local), 1);

		net_os_set_packet_filter(dev, 0);

#ifdef COEX_DHCP
#if 0
		g_bt->ts_dhcp_start = JF2MS;
		g_bt->dhcp_done = FALSE;
		WL_TRACE_COEX(("%s: DHCP start, pm:%d changed to pm:%d\n",
			__func__, pm, pm_local));
#endif
#endif
	} else if (strnicmp((char *)&powermode_val, "0", strlen("0")) == 0) {

		wldev_ioctl(dev, WLC_SET_PM, &pm, sizeof(pm), 1);

		net_os_set_packet_filter(dev, 1);

#ifdef COEX_DHCP
#if 0
		g_bt->dhcp_done = TRUE;
		g_bt->ts_dhcp_ok = JF2MS;
		WL_TRACE_COEX(("%s: DHCP done for:%d ms, restored pm:%d\n",
			__func__, (g_bt->ts_dhcp_ok - g_bt->ts_dhcp_start), pm));
#endif
#endif
	} else {
		DHD_ERROR(("%s Unkwown yet power setting, ignored\n",
			__func__));
	}
#endif
/* HTC_CSP_END*/
	bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;

}

static int wl_android_get_power_mode(struct net_device *dev, char *command, int total_len)
{
	int error = 0;
	int pm_local = PM_FAST;
	int bytes_written;

	error = wldev_ioctl(dev, WLC_GET_PM, &pm_local, sizeof(pm_local), 0);
	if (!error) {
		DHD_TRACE(("%s: Powermode = %d\n", __func__, pm_local));
		if (pm_local == PM_OFF)
			pm_local = 1;
		else
			pm_local = 0;
		bytes_written = snprintf(command, total_len, "powermode = %d\n", pm_local);

	} else {
		DHD_TRACE(("%s: Error = %d\n", __func__, error));
		bytes_written = snprintf(command, total_len, "FAIL\n");
	}
	return bytes_written;
}
static int wl_android_get_wifilock(struct net_device *ndev, char *command, int total_len)
{
	int bytes_written = 0;

	bytes_written = snprintf(command, total_len, "%d", dhdcdc_wifiLock);
	printf("dhdcdc_wifiLock: %s\n", command);

	return bytes_written;
}

int wl_android_is_during_wifi_call(void)
{
	return wl_android_wifi_call;
}

static int wl_android_set_wificall(struct net_device *ndev, char *command, int total_len)
{
	int bytes_written = 0;
	char *s;
	int set_val;

	mutex_lock(&wl_wificall_mutex);
	s =  command + strlen("WIFICALL") + 1;
	set_val = bcm_atoi(s);


	/* 1. change power mode
	  * 2. change dtim listen interval
	 */

	switch (set_val) {
	case 0:
		if (0 == wl_android_is_during_wifi_call()) {
			printf("wifi call is in disconnected state!\n");
			break;
		}

		printf("wifi call ends: %d\n", set_val);
		wl_android_wifi_call = 0;

		dhdhtc_set_power_control(0, DHDHTC_POWER_CTRL_WIFI_PHONE);
		dhdhtc_update_wifi_power_mode(screen_off);

		dhdhtc_update_dtim_listen_interval(screen_off);

		break;
	case 1:
		if (1 == wl_android_is_during_wifi_call()) {
			printf("wifi call is already in running state!\n");
			break;
		}

		printf("wifi call comes: %d\n", set_val);
		wl_android_wifi_call = 1;

		dhdhtc_set_power_control(1, DHDHTC_POWER_CTRL_WIFI_PHONE);
		dhdhtc_update_wifi_power_mode(screen_off);

		dhdhtc_update_dtim_listen_interval(screen_off);

		break;
	default:
		DHD_ERROR(("%s: not support mode: %d\n", __func__, set_val));
		break;

	}

	bytes_written = snprintf(command, total_len, "OK");
	mutex_unlock(&wl_wificall_mutex);

	return bytes_written;
}

int dhd_set_project(char * project, int project_len)
{

        if ((project_len < 1) || (project_len > 32)) {
                printf("Invaild project name length!\n");
                return -1;
        }

        strncpy(project_type, project, project_len);
        DHD_DEFAULT(("%s: set project type: %s\n", __FUNCTION__, project_type));

        return 0;
}

static int wl_android_set_project(struct net_device *ndev, char *command, int total_len)
{
	int bytes_written = 0;
	char project[33];
	int project_offset;
	int project_size;

	DHD_TRACE(("%s\n", __FUNCTION__));
	memset(project, 0, sizeof(project));

	project_offset = strcspn(command, " ");
	project_size = strlen(command) - project_offset;

	if (project_offset == 0) {
		DHD_ERROR(("%s, no project specified\n", __FUNCTION__));
		return 0;
	}

	if (project_size > 32) {
		DHD_ERROR(("%s: project name is too long: %s\n", __FUNCTION__,
				(char *)command + project_offset + 1));
		return 0;
	}

	strncpy(project, command + project_offset + 1, MIN(project_size, sizeof(project)));

	DHD_DEFAULT(("%s: set project: %s\n", __FUNCTION__, project));
	dhd_set_project(project, project_size);

	bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;
}

#ifdef BCM4329_LOW_POWER
static int wl_android_set_lowpowermode(struct net_device *ndev, char *command, int total_len)
{
	int bytes_written = 0;
	char *s;
	int set_val;

	DHD_TRACE(("%s\n", __func__));
	s =  command + strlen("SET_LOWPOWERMODE") + 1;
	set_val = bcm_atoi(s);

	switch (set_val) {
	case 0:
		printf("LowPowerMode: %d\n", set_val);
		LowPowerMode = 0;
		break;
	case 1:
		printf("LowPowerMode: %d\n", set_val);
		LowPowerMode = 1;
		break;
	default:
		DHD_ERROR(("%s: not support mode: %d\n", __func__, set_val));
		break;
	}

	bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;
}
#endif

static int wl_android_gateway_add(struct net_device *ndev, char *command, int total_len)
{
	int bytes_written = 0;

	int i;
	DHD_TRACE(("Driver GET GATEWAY-ADD CMD!!!\n"));
	sscanf(command+12,"%d",&i);
	sprintf( wl_abdroid_gatewaybuf, "%02x%02x%02x%02x",
	i & 0xff, ((i >> 8) & 0xff), ((i >> 16) & 0xff), ((i >> 24) & 0xff)
	);

	if (strcmp(wl_abdroid_gatewaybuf, "00000000") == 0)
		sprintf( wl_abdroid_gatewaybuf, "FFFFFFFF");

	DHD_TRACE(("gatewaybuf: %s",wl_abdroid_gatewaybuf));

#ifdef BCM4329_LOW_POWER
	if (LowPowerMode == 1) {
		if (screen_off && !hasDLNA && !allowMulticast)
			dhd_set_keepalive(1);
	}
#endif

	bytes_written = snprintf(command, total_len, "OK");

	return bytes_written;
}

static int last_auto_channel = 6;
static int wl_android_auto_channel(struct net_device *dev, char *command, int total_len)
{
	int chosen = 0;
	char req_buf[64] = {0};
	wl_uint32_list_t *request = (wl_uint32_list_t *)req_buf;
	int rescan = 0;
	int retry = 0;
	int updown = 0;
	wlc_ssid_t null_ssid;
	int res = 0;
	int spec = 0;
	int start_channel = 1, end_channel = 14;
	int i = 0;
	int channel = 0;
	int isup = 0;
	int bytes_written = 0;
	int apsta_var = 0;

	DHD_TRACE(("Enter %s\n", __func__));

	channel = bcm_atoi(command);

	wldev_ioctl(dev, WLC_GET_UP, &isup, sizeof(isup), 0);

	res = wldev_ioctl(dev, WLC_DOWN, &updown, sizeof(updown), 1);
	if (res) {
		DHD_ERROR(("%s fail to set updown\n", __func__));
		goto fail;
	}

	apsta_var = 0;
	res = wldev_ioctl(dev, WLC_SET_AP, &apsta_var, sizeof(apsta_var), 1);
	if (res) {
		DHD_ERROR(("%s fail to set apsta_var 0\n", __func__));
		goto fail;
	}
	apsta_var = 1;
	res = wldev_ioctl(dev, WLC_SET_AP, &apsta_var, sizeof(apsta_var), 1);
	if (res) {
		DHD_ERROR(("%s fail to set apsta_var 1\n", __func__));
		goto fail;
	}
	res = wldev_ioctl(dev, WLC_GET_AP, &apsta_var, sizeof(apsta_var), 0);

	updown = 1;
	res = wldev_ioctl(dev, WLC_UP, &updown, sizeof(updown), 1);
	if (res < 0) {
		DHD_ERROR(("%s fail to set apsta \n", __func__));
		goto fail;
	}

auto_channel_retry:
	memset(&null_ssid, 0, sizeof(wlc_ssid_t));
	null_ssid.SSID_len = strlen("test");
	strncpy(null_ssid.SSID, "test", null_ssid.SSID_len);

	res |= wldev_ioctl(dev, WLC_SET_SPECT_MANAGMENT, &spec, sizeof(spec), 1);
	res |= wldev_ioctl(dev, WLC_SET_SSID, &null_ssid, sizeof(null_ssid), 1);
	res |= wldev_ioctl(dev, WLC_UP, &updown, sizeof(updown), 1);

	memset(&null_ssid, 0, sizeof(wlc_ssid_t));
	res |= wldev_ioctl(dev, WLC_SET_SSID, &null_ssid, sizeof(null_ssid), 1);

	request->count = htod32(0);
	if (channel >> 8) {
		start_channel = (channel >> 8) & 0xff;
		end_channel = channel & 0xff;
		request->count = end_channel - start_channel + 1;
		DHD_ERROR(("request channel: %d to %d ,request->count =%d\n", start_channel, end_channel, request->count));
		for (i = 0; i < request->count; i++) {
			request->element[i] = CH20MHZ_CHSPEC((start_channel + i));
			/* request->element[i] = (start_channel + i); */
			printf("request.element[%d]=0x%x\n", i, request->element[i]);
		}
	}

	res = wldev_ioctl(dev, WLC_START_CHANNEL_SEL, request, sizeof(req_buf), 1);
	if (res < 0) {
		DHD_ERROR(("can't start auto channel\n"));
		chosen = 6;
		goto fail;
	}

get_channel_retry:
		bcm_mdelay(500);

	res = wldev_ioctl(dev, WLC_GET_CHANNEL_SEL, &chosen, sizeof(chosen), 0);

	if (res < 0 || dtoh32(chosen) == 0) {
		if (retry++ < 6)
			goto get_channel_retry;
		else {
			DHD_ERROR(("can't get auto channel sel, err = %d, "
						"chosen = %d\n", res, chosen));
			chosen = 6; /*Alan: Set default channel when get auto channel failed*/
		}
	}

	if ((chosen == start_channel) && (!rescan++)) {
		retry = 0;
		goto auto_channel_retry;
	}

	if (channel == 0) {
		channel = chosen;
		last_auto_channel = chosen;
	} else {
		DHD_ERROR(("channel range from %d to %d ,chosen = %d\n", start_channel, end_channel, chosen));

		if (chosen > end_channel) {
			if (chosen <= 6)
				chosen = end_channel;
			else
				chosen = start_channel;
		} else if (chosen < start_channel)
			chosen = start_channel;

		channel = chosen;
	}

	res = wldev_ioctl(dev, WLC_DOWN, &updown, sizeof(updown), 1);
	if (res < 0) {
		DHD_ERROR(("%s fail to set up err =%d\n", __func__, res));
		goto fail;
	}

fail :

	bytes_written = snprintf(command, total_len, "%d", channel);
	return bytes_written;

}
/* HTC_CSP_END */

/**
 * Global function definitions (declared in wl_android.h)
 */

int wl_android_wifi_on(struct net_device *dev)
{
	int ret = 0;

	printf("%s in\n", __FUNCTION__);
	if (!dev) {
		DHD_ERROR(("%s: dev is null\n", __FUNCTION__));
		return -EINVAL;
	}
	mutex_lock(&wl_wifionoff_mutex);
	if (!g_wifi_on) {
		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_ON);
		sdioh_start(NULL, 0);
		ret = dhd_dev_reset(dev, FALSE);
		sdioh_start(NULL, 1);
		if (!ret)
			dhd_dev_init_ioctl(dev);
		g_wifi_on = 1;
	}
	mutex_unlock(&wl_wifionoff_mutex);

	return ret;
}

int wl_android_wifi_off(struct net_device *dev)
{
	int ret = 0;

	printf("%s in\n", __FUNCTION__);
	if (!dev) {
		DHD_TRACE(("%s: dev is null\n", __FUNCTION__));
		return -EINVAL;
	}
	mutex_lock(&wl_wifionoff_mutex);
	/* HTC_WIFI_START */
	if (dhd_APUP) {
		printf("apmode off - AP_DOWN\n");
		dhd_APUP = false;
		wl_iw_send_priv_event(dev, "AP_DOWN");
	}
	/* HTC_WIFI_END */
	if (g_wifi_on) {
		ret = dhd_dev_reset(dev, TRUE);
		sdioh_stop(NULL);
		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_OFF);
		g_wifi_on = 0;
	}

	mutex_unlock(&wl_wifionoff_mutex);
	bcm_mdelay(500);
	return ret;
}

static int wl_android_set_fwpath(struct net_device *net, char *command, int total_len)
{
	if ((strlen(command) - strlen(CMD_SETFWPATH)) > MOD_PARAM_PATHLEN)
		return -1;
	bcm_strncpy_s(fw_path, sizeof(fw_path),
		command + strlen(CMD_SETFWPATH) + 1, MOD_PARAM_PATHLEN - 1);
	if (strstr(fw_path, "apsta") != NULL) {
		DHD_INFO(("GOT APSTA FIRMWARE\n"));
		ap_fw_loaded = TRUE;
	} else {
		DHD_INFO(("GOT STA FIRMWARE\n"));
		ap_fw_loaded = FALSE;
	}
	return 0;
}

int wl_android_priv_cmd(struct net_device *net, struct ifreq *ifr, int cmd)
{
	int ret = 0;
	char *command = NULL;
	int bytes_written = 0;
	android_wifi_priv_cmd priv_cmd;

#ifdef BCM4329_LOW_POWER
	struct dd_pkt_filter_s *data;
#endif

	net_os_wake_lock(net);

	if (!ifr->ifr_data) {
		ret = -EINVAL;
		goto exit;
	}
	if (copy_from_user(&priv_cmd, ifr->ifr_data, sizeof(android_wifi_priv_cmd))) {
		ret = -EFAULT;
		goto exit;
	}
	command = kmalloc(priv_cmd.total_len, GFP_KERNEL);
	if (!command)
	{
		DHD_ERROR(("%s: failed to allocate memory\n", __FUNCTION__));
		ret = -ENOMEM;
		goto exit;
	}
	if (copy_from_user(command, priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto exit;
	}

	DHD_INFO(("%s: Android private cmd \"%s\" on %s\n", __FUNCTION__, command, ifr->ifr_name));

	if (strnicmp(command, CMD_START, strlen(CMD_START)) == 0) {
		DHD_INFO(("%s, Received regular START command\n", __FUNCTION__));
		bytes_written = wl_android_wifi_on(net);
	}
	else if (strnicmp(command, CMD_SETFWPATH, strlen(CMD_SETFWPATH)) == 0) {
		bytes_written = wl_android_set_fwpath(net, command, priv_cmd.total_len);
	}

	if (!g_wifi_on) {
		DHD_ERROR(("%s: Ignore private cmd \"%s\" - iface %s is down\n",
			__FUNCTION__, command, ifr->ifr_name));
		ret = 0;
		goto exit;
	}

	if (strnicmp(command, CMD_STOP, strlen(CMD_STOP)) == 0) {
		bytes_written = wl_android_wifi_off(net);
	}
	else if (strnicmp(command, CMD_SCAN_ACTIVE, strlen(CMD_SCAN_ACTIVE)) == 0) {
		/* bytes_written = wl_android_set_active_scan(net, command, priv_cmd.total_len); */
	}
	else if (strnicmp(command, CMD_SCAN_PASSIVE, strlen(CMD_SCAN_PASSIVE)) == 0) {
		/* bytes_written = wl_android_set_passive_scan(net, command, priv_cmd.total_len); */
	}
	else if (strnicmp(command, CMD_RSSI, strlen(CMD_RSSI)) == 0) {
		bytes_written = wl_android_get_rssi(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_LINKSPEED, strlen(CMD_LINKSPEED)) == 0) {
		bytes_written = wl_android_get_link_speed(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_RXFILTER_START, strlen(CMD_RXFILTER_START)) == 0) {
		/* HTC_CSP_START*/
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
		/*bytes_written = net_os_set_packet_filter(net, 1);*/
		/* HTC_CSP_END*/
	}
	else if (strnicmp(command, CMD_RXFILTER_STOP, strlen(CMD_RXFILTER_STOP)) == 0) {
		/* HTC_CSP_START*/
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
		/*bytes_written = net_os_set_packet_filter(net, 0);*/
		/* HTC_CSP_END*/
	}
	else if (strnicmp(command, CMD_RXFILTER_ADD, strlen(CMD_RXFILTER_ADD)) == 0) {
		/*HTC_CSP_START*/
#ifdef BCM4329_LOW_POWER
		if (LowPowerMode == 1) {
			data = (struct dd_pkt_filter_s *)&command[32];
			if ((data->id == ALLOW_IPV6_MULTICAST) || (data->id == ALLOW_IPV4_MULTICAST)) {
				WL_TRACE(("RXFILTER-ADD MULTICAST filter\n"));
				allowMulticast = false;
			}
		}
#endif
		wl_android_set_pktfilter(net, (struct dd_pkt_filter_s *)&command[32]);
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
		/*
		int filter_num = *(command + strlen(CMD_RXFILTER_ADD) + 1) - '0';
		bytes_written = net_os_rxfilter_add_remove(net, TRUE, filter_num);
		*/
		/* HTC_CSP_END */
	}
	else if (strnicmp(command, CMD_RXFILTER_REMOVE, strlen(CMD_RXFILTER_REMOVE)) == 0) {
		/*HTC_CSP_START*/
#ifdef BCM4329_LOW_POWER
		if (LowPowerMode == 1) {
			data = (struct dd_pkt_filter_s *)&command[32];
			if ((data->id == ALLOW_IPV6_MULTICAST) || (data->id == ALLOW_IPV4_MULTICAST)) {
				WL_TRACE(("RXFILTER-REMOVE MULTICAST filter\n"));
				allowMulticast = true;
			}
		}
#endif
		wl_android_set_pktfilter(net, (struct dd_pkt_filter_s *)&command[32]);
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
		/*
		int filter_num = *(command + strlen(CMD_RXFILTER_REMOVE) + 1) - '0';
		bytes_written = net_os_rxfilter_add_remove(net, FALSE, filter_num);
		*/
		/*HTC_CSP_END*/
	}
	else if (strnicmp(command, CMD_BTCOEXSCAN_START, strlen(CMD_BTCOEXSCAN_START)) == 0) {
		/* TBD: BTCOEXSCAN-START */
	}
	else if (strnicmp(command, CMD_BTCOEXSCAN_STOP, strlen(CMD_BTCOEXSCAN_STOP)) == 0) {
		/* TBD: BTCOEXSCAN-STOP */
	}
	else if (strnicmp(command, CMD_BTCOEXMODE, strlen(CMD_BTCOEXMODE)) == 0) {
		uint mode = *(command + strlen(CMD_BTCOEXMODE) + 1) - '0';

		if (mode == 1)
			net_os_set_packet_filter(net, 0); /* DHCP starts */
		else
			net_os_set_packet_filter(net, 1); /* DHCP ends */
#ifdef WL_CFG80211
		bytes_written = wl_cfg80211_set_btcoex_dhcp(net, command);
#endif
	}
	else if (strnicmp(command, CMD_SETSUSPENDOPT, strlen(CMD_SETSUSPENDOPT)) == 0) {
		bytes_written = wl_android_set_suspendopt(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_SETBAND, strlen(CMD_SETBAND)) == 0) {
		uint band = *(command + strlen(CMD_SETBAND) + 1) - '0';
		bytes_written = wldev_set_band(net, band);
	}
	else if (strnicmp(command, CMD_GETBAND, strlen(CMD_GETBAND)) == 0) {
		bytes_written = wl_android_get_band(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_COUNTRY, strlen(CMD_COUNTRY)) == 0) {
		char *country_code = command + strlen(CMD_COUNTRY) + 1;
		bytes_written = wldev_set_country(net, country_code);
	}
#ifdef PNO_SUPPORT
	else if (strnicmp(command, CMD_PNOSSIDCLR_SET, strlen(CMD_PNOSSIDCLR_SET)) == 0) {
		bytes_written = dhd_dev_pno_reset(net);
	}
	else if (strnicmp(command, CMD_PNOSETUP_SET, strlen(CMD_PNOSETUP_SET)) == 0) {
		bytes_written = wl_android_set_pno_setup(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_PNOENABLE_SET, strlen(CMD_PNOENABLE_SET)) == 0) {
		uint pfn_enabled = *(command + strlen(CMD_PNOENABLE_SET) + 1) - '0';
		bytes_written = dhd_dev_pno_enable(net, pfn_enabled);
	}
#endif
#ifdef WL_CFG80211
	else if (strnicmp(command, CMD_MAC_ADDR, strlen(CMD_MAC_ADDR)) == 0) {
		bytes_written = wl_android_get_mac_addr(net, command, priv_cmd.total_len);
	}
#endif
	else if (strnicmp(command, CMD_P2P_DEV_ADDR, strlen(CMD_P2P_DEV_ADDR)) == 0) {
		bytes_written = wl_android_get_p2p_dev_addr(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_P2P_SET_NOA, strlen(CMD_P2P_SET_NOA)) == 0) {
		int skip = strlen(CMD_P2P_SET_NOA) + 1;
		bytes_written = wl_cfg80211_set_p2p_noa(net, command + skip,
			priv_cmd.total_len - skip);
	}
	else if (strnicmp(command, CMD_P2P_GET_NOA, strlen(CMD_P2P_GET_NOA)) == 0) {
		bytes_written = wl_cfg80211_get_p2p_noa(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_P2P_SET_PS, strlen(CMD_P2P_SET_PS)) == 0) {
		int skip = strlen(CMD_P2P_SET_PS) + 1;
		bytes_written = wl_cfg80211_set_p2p_ps(net, command + skip,
			priv_cmd.total_len - skip);
	}
#ifdef WL_CFG80211
	else if (strnicmp(command, CMD_SET_AP_WPS_P2P_IE,
		strlen(CMD_SET_AP_WPS_P2P_IE)) == 0) {
		int skip = strlen(CMD_SET_AP_WPS_P2P_IE) + 3;
		bytes_written = wl_cfg80211_set_wps_p2p_ie(net, command + skip,
			priv_cmd.total_len - skip, *(command + skip - 2) - '0');
	} else if (strnicmp(command, CMD_P2P_SET_MPC, strlen(CMD_P2P_SET_MPC)) == 0) {
		int skip = strlen(CMD_P2P_SET_MPC) + 1;
		bytes_written = wl_cfg80211_set_mpc(net, command + skip,
			priv_cmd.total_len - skip);
	} else if (strnicmp(command, CMD_DEAUTH_STA, strlen(CMD_DEAUTH_STA)) == 0) {
		int skip = strlen(CMD_DEAUTH_STA) + 1;
		bytes_written = wl_cfg80211_deauth_sta(net, command + skip,
			priv_cmd.total_len - skip);
	} 
#endif /* WL_CFG80211 */
	else if (strnicmp(command, CMD_DTIM_SKIP_GET, strlen(CMD_DTIM_SKIP_GET)) == 0) {
		bytes_written = wl_android_get_dtim_skip(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_DTIM_SKIP_SET, strlen(CMD_DTIM_SKIP_SET)) == 0) {
		bytes_written = wl_android_set_dtim_skip(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_TXPOWER_SET, strlen(CMD_TXPOWER_SET)) == 0) {
		bytes_written = wl_android_set_txpower(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_POWER_MODE_SET, strlen(CMD_POWER_MODE_SET)) == 0) {
		bytes_written = wl_android_set_power_mode(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_POWER_MODE_GET, strlen(CMD_POWER_MODE_GET)) == 0) {
		bytes_written = wl_android_get_power_mode(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_AP_TXPOWER_SET, strlen(CMD_AP_TXPOWER_SET)) == 0) {
		bytes_written = wl_android_set_ap_txpower(net, command, priv_cmd.total_len);
	} else if (strnicmp(command, CMD_AP_ASSOC_LIST_GET, strlen(CMD_AP_ASSOC_LIST_GET)) == 0) {
		bytes_written = wl_android_get_assoc_sta_list(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_AP_MAC_LIST_SET, strlen(CMD_AP_MAC_LIST_SET)) == 0) {
		bytes_written = wl_android_set_ap_mac_list(net, command + PROFILE_OFFSET);
	}
	else if (strnicmp(command, CMD_SCAN_MINRSSI_SET, strlen(CMD_SCAN_MINRSSI_SET)) == 0) {
		bytes_written = wl_android_set_scan_minrssi(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_LOW_RSSI_SET, strlen(CMD_LOW_RSSI_SET)) == 0) {
		bytes_written = wl_android_low_rssi_set(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_PFN_REMOVE, strlen(CMD_PFN_REMOVE)) == 0) {
		bytes_written = wl_android_del_pfn(net, command, priv_cmd.total_len);
	}
/* HTC_CSP_START */
#if 0
	else if (strnicmp(command, CMD_GETCSCAN, strlen(CMD_GETCSCAN)) == 0) {
		bytes_written = wl_android_get_cscan(net, command, priv_cmd.total_len);
	}
#endif
	else if (strnicmp(command, CMD_GETWIFILOCK, strlen(CMD_GETWIFILOCK)) == 0) {
		bytes_written = wl_android_get_wifilock(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_SETWIFICALL, strlen(CMD_SETWIFICALL)) == 0) {
		bytes_written = wl_android_set_wificall(net, command, priv_cmd.total_len);
	}
	else if (strnicmp(command, CMD_SETPROJECT, strlen(CMD_SETPROJECT)) == 0) {
		bytes_written = wl_android_set_project(net, command, priv_cmd.total_len);
	}
#ifdef BCM4329_LOW_POWER
	else if (strnicmp(command, CMD_SETLOWPOWERMODE, strlen(CMD_SETLOWPOWERMODE)) == 0) {
		bytes_written = wl_android_set_lowpowermode(net, command, priv_cmd.total_len);
	}
#endif
	else if (strnicmp(command, CMD_GATEWAYADD, strlen(CMD_GATEWAYADD)) == 0) {
		bytes_written = wl_android_gateway_add(net, command, priv_cmd.total_len);
	} else if (strnicmp(command, CMD_GET_AUTO_CHANNEL, strlen(CMD_GET_AUTO_CHANNEL)) == 0) {
		int skip = strlen(CMD_GET_AUTO_CHANNEL) + 1;
		block_ap_event = 1;
		bytes_written = wl_android_auto_channel(net, command + skip,
		priv_cmd.total_len - skip) + skip;
		block_ap_event = 0;
	}
/* HTC_CSP_END */
	else {
		DHD_ERROR(("%s: Unknown PRIVATE command %s - ignored\n", __FUNCTION__, command));
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
	}
	if (bytes_written >= 0) {
		if (bytes_written == 0)
			command[0] = '\0';
		if (bytes_written > priv_cmd.total_len) {
			DHD_ERROR(("%s: bytes_written = %d\n", __FUNCTION__, bytes_written));
			bytes_written = priv_cmd.total_len;
		} else {
			bytes_written++;
		}
		priv_cmd.used_len = bytes_written;
		if (copy_to_user(priv_cmd.buf, command, bytes_written)) {
			DHD_ERROR(("%s: failed to copy data to user buffer\n", __FUNCTION__));
			ret = -EFAULT;
		}
	} else {
		ret = bytes_written;
	}

exit:
	net_os_wake_unlock(net);
	if (command) {
		kfree(command);
	}

	return ret;
}

int wl_android_init(void)
{
	int ret = 0;

	dhd_msg_level = DHD_ERROR_VAL;
#ifdef ENABLE_INSMOD_NO_FW_LOAD
	dhd_download_fw_on_driverload = FALSE;
#endif /* ENABLE_INSMOD_NO_FW_LOAD */
#ifdef CUSTOMER_HW2
	if (!iface_name[0]) {
		memset(iface_name, 0, IFNAMSIZ);
		bcm_strncpy_s(iface_name, IFNAMSIZ, "wlan", IFNAMSIZ);
	}
#endif /* CUSTOMER_HW2 */

/*HTC_CSP_START*/
	mutex_init(&wl_wificall_mutex);
	mutex_init(&wl_wifionoff_mutex);
/*HTC_CSP_END*/
	return ret;
}

int wl_android_exit(void)
{
	int ret = 0;

	return ret;
}

void wl_android_post_init(void)
{
	if (!dhd_download_fw_on_driverload) {
		/* Call customer gpio to turn off power with WL_REG_ON signal */
		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_OFF);
		g_wifi_on = 0;
	}
}
/**
 * Functions for Android WiFi card detection
 */
#if defined(CONFIG_WIFI_CONTROL_FUNC)

static int g_wifidev_registered = 0;
static struct semaphore wifi_control_sem;
static struct wifi_platform_data *wifi_control_data = NULL;
static struct resource *wifi_irqres = NULL;

static int wifi_add_dev(void);
static void wifi_del_dev(void);

int wl_android_wifictrl_func_add(void)
{
	int ret = 0;
	sema_init(&wifi_control_sem, 0);

	ret = wifi_add_dev();
	if (ret) {
		DHD_ERROR(("%s: platform_driver_register failed\n", __FUNCTION__));
		return ret;
	}
	g_wifidev_registered = 1;

	/* Waiting callback after platform_driver_register is done or exit with error */
	if (down_timeout(&wifi_control_sem,  msecs_to_jiffies(1000)) != 0) {
		ret = -EINVAL;
		DHD_ERROR(("%s: platform_driver_register timeout\n", __FUNCTION__));
	}

	return ret;
}

void wl_android_wifictrl_func_del(void)
{
	if (g_wifidev_registered)
	{
		wifi_del_dev();
		g_wifidev_registered = 0;
	}
}

void* wl_android_prealloc(int section, unsigned long size)
{
	void *alloc_ptr = NULL;
	if (wifi_control_data && wifi_control_data->mem_prealloc) {
		alloc_ptr = wifi_control_data->mem_prealloc(section, size);
		if (alloc_ptr) {
			DHD_INFO(("success alloc section %d\n", section));
			bzero(alloc_ptr, size);
			return alloc_ptr;
		}
	}

	DHD_ERROR(("can't alloc section %d\n", section));
	return 0;
}

int wifi_get_irq_number(unsigned long *irq_flags_ptr)
{
	if (wifi_irqres) {
		*irq_flags_ptr = wifi_irqres->flags & IRQF_TRIGGER_MASK;
		return (int)wifi_irqres->start;
	}
#ifdef CUSTOM_OOB_GPIO_NUM
	return CUSTOM_OOB_GPIO_NUM;
#else
	return -1;
#endif
}

int wifi_set_power(int on, unsigned long msec)
{
	DHD_ERROR(("%s = %d\n", __FUNCTION__, on));
	if (wifi_control_data && wifi_control_data->set_power) {
		wifi_control_data->set_power(on);
	}
	if (msec)
		msleep(msec);
	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
int wifi_get_mac_addr(unsigned char *buf)
{
	DHD_DEFAULT(("%s\n", __FUNCTION__));
	if (!buf)
		return -EINVAL;
	if (wifi_control_data && wifi_control_data->get_mac_addr) {
		return wifi_control_data->get_mac_addr(buf);
	}
	return -EOPNOTSUPP;
}
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))
void *wifi_get_country_code(char *ccode)
{
	DHD_TRACE(("%s\n", __FUNCTION__));
	if (!ccode)
		return NULL;
	if (wifi_control_data && wifi_control_data->get_country_code) {
		return wifi_control_data->get_country_code(ccode);
	}
	return NULL;
}
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)) */

static int wifi_set_carddetect(int on)
{
	DHD_DEFAULT(("%s = %d\n", __FUNCTION__, on));
	if (wifi_control_data && wifi_control_data->set_carddetect) {
		wifi_control_data->set_carddetect(on);
	}
	return 0;
}

static int wifi_probe(struct platform_device *pdev)
{
	struct wifi_platform_data *wifi_ctrl =
		(struct wifi_platform_data *)(pdev->dev.platform_data);

	DHD_DEFAULT(("## %s\n", __FUNCTION__));
	wifi_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "bcmdhd_wlan_irq");
	if (wifi_irqres == NULL)
		wifi_irqres = platform_get_resource_byname(pdev,
			IORESOURCE_IRQ, "bcm4329_wlan_irq");
	wifi_control_data = wifi_ctrl;

	wifi_set_power(1, 0);	/* Power On */
	wifi_set_carddetect(1);	/* CardDetect (0->1) */

	up(&wifi_control_sem);
	return 0;
}

static int wifi_remove(struct platform_device *pdev)
{
	struct wifi_platform_data *wifi_ctrl =
		(struct wifi_platform_data *)(pdev->dev.platform_data);

	DHD_DEFAULT(("## %s\n", __FUNCTION__));
	wifi_control_data = wifi_ctrl;

	wifi_set_power(0, 0);	/* Power Off */
	wifi_set_carddetect(0);	/* CardDetect (1->0) */

	up(&wifi_control_sem);
	DHD_DEFAULT(("## %s leave\n", __FUNCTION__));
	return 0;
}

static int wifi_suspend(struct platform_device *pdev, pm_message_t state)
{
	DHD_TRACE(("##> %s\n", __FUNCTION__));
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 39)) && defined(OOB_INTR_ONLY)
	bcmsdh_oob_intr_set(0);
#endif /* (OOB_INTR_ONLY) */
	return 0;
}

static int wifi_resume(struct platform_device *pdev)
{
	DHD_TRACE(("##> %s\n", __FUNCTION__));
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 39)) && defined(OOB_INTR_ONLY)
	if (dhd_os_check_if_up(bcmsdh_get_drvdata()))
		bcmsdh_oob_intr_set(1);
#endif /* (OOB_INTR_ONLY) */
	return 0;
}

static struct platform_driver wifi_device = {
	.probe          = wifi_probe,
	.remove         = wifi_remove,
	.suspend        = wifi_suspend,
	.resume         = wifi_resume,
	.driver         = {
	.name   = "bcmdhd_wlan",
	}
};

static struct platform_driver wifi_device_legacy = {
	.probe          = wifi_probe,
	.remove         = wifi_remove,
	.suspend        = wifi_suspend,
	.resume         = wifi_resume,
	.driver         = {
	.name   = "bcm4329_wlan",
	}
};

static int wifi_add_dev(void)
{
	DHD_TRACE(("## Calling platform_driver_register\n"));
	platform_driver_register(&wifi_device);
	platform_driver_register(&wifi_device_legacy);
	return 0;
}

static void wifi_del_dev(void)
{
	DHD_TRACE(("## Unregister platform_driver_register\n"));
	platform_driver_unregister(&wifi_device);
	platform_driver_unregister(&wifi_device_legacy);
}
#endif /* defined(CONFIG_WIFI_CONTROL_FUNC) */
