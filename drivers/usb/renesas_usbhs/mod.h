/*
 * Renesas USB driver
 *
 * Copyright (C) 2011 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef RENESAS_USB_MOD_H
#define RENESAS_USB_MOD_H

#include <linux/spinlock.h>
#include <linux/usb/renesas_usbhs.h>
#include "./common.h"

/*
 *	struct
 */
struct usbhs_irq_state {
	u16 intsts0;
	u16 intsts1;
	u16 brdysts;
	u16 nrdysts;
	u16 bempsts;
	u16 dvstctr;
};

struct usbhs_mod {
	char *name;

	/*
	 * entry point from common.c
	 */
	int (*start)(struct usbhs_priv *priv);
	int (*stop)(struct usbhs_priv *priv);

	/* INTSTS0 :: DVST (DVSQ) */
	int (*irq_dev_state)(struct usbhs_priv *priv,
			     struct usbhs_irq_state *irq_state);

	/* INTSTS0 :: CTRT (CTSQ) */
	int (*irq_ctrl_stage)(struct usbhs_priv *priv,
			      struct usbhs_irq_state *irq_state);

	/* INTSTS0 :: BEMP */
	/* BEMPSTS */
	int (*irq_empty)(struct usbhs_priv *priv,
			 struct usbhs_irq_state *irq_state);
	u16 irq_bempsts;

	/* INTSTS0 :: BRDY */
	/* BRDYSTS */
	int (*irq_ready)(struct usbhs_priv *priv,
			 struct usbhs_irq_state *irq_state);
	u16 irq_brdysts;

	struct usbhs_priv *priv;
};

struct usbhs_mod_info {
	struct usbhs_mod *mod[USBHS_MAX];
	struct usbhs_mod *curt; /* current mod */

	/*
	 * INTSTS0 :: VBINT
	 *
	 * This function will be used as autonomy mode
	 * when platform cannot call notify_hotplug.
	 *
	 * This callback cannot be member of "struct usbhs_mod"
	 * because it will be used even though
	 * host/gadget has not been selected.
	 */
	int (*irq_vbus)(struct usbhs_priv *priv,
			struct usbhs_irq_state *irq_state);
};

/*
 *		for host/gadget module
 */
struct usbhs_mod *usbhs_mod_get(struct usbhs_priv *priv, int id);
struct usbhs_mod *usbhs_mod_get_current(struct usbhs_priv *priv);
void usbhs_mod_register(struct usbhs_priv *priv, struct usbhs_mod *usb, int id);
int usbhs_mod_is_host(struct usbhs_priv *priv, struct usbhs_mod *mod);
int usbhs_mod_change(struct usbhs_priv *priv, int id);
int usbhs_mod_probe(struct usbhs_priv *priv);
void usbhs_mod_remove(struct usbhs_priv *priv);

void usbhs_mod_autonomy_mode(struct usbhs_priv *priv);

/*
 *		status functions
 */
int usbhs_status_get_usb_speed(struct usbhs_irq_state *irq_state);
int usbhs_status_get_device_state(struct usbhs_irq_state *irq_state);
int usbhs_status_get_ctrl_stage(struct usbhs_irq_state *irq_state);

/*
 *		callback functions
 */
void usbhs_irq_callback_update(struct usbhs_priv *priv, struct usbhs_mod *mod);


#define usbhs_mod_call(priv, func, param...)		\
	({						\
		struct usbhs_mod *mod;			\
		mod = usbhs_mod_get_current(priv);	\
		!mod		? -ENODEV :		\
		!mod->func	? 0 :			\
		 mod->func(param);			\
	})

/*
 * gadget control
 */
#ifdef CONFIG_USB_RENESAS_USBHS_UDC
extern int __devinit usbhs_mod_gadget_probe(struct usbhs_priv *priv);
extern void __devexit usbhs_mod_gadget_remove(struct usbhs_priv *priv);
#else
static inline int usbhs_mod_gadget_probe(struct usbhs_priv *priv)
{
	return 0;
}
static inline void usbhs_mod_gadget_remove(struct usbhs_priv *priv)
{
}
#endif

#endif /* RENESAS_USB_MOD_H */
