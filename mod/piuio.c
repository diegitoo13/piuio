// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020  –  10-button game-pad (bit-level, 16-byte report)
 * plus optional coin counter from Feature-report 0x01.
 *
 * Button is held if its bit is low in any of the player’s 4 bytes.
 * A single “wake” Output-report 0x81 full of 0xFF is required once
 * so the board starts updating the coin counter.
 *
 * v2-2025-04 – Diego Acevedo
 *   • coin support (KEY_COIN pulse on each increment)
 *   • keeps original matrix logic 1-for-1
 */
#define pr_fmt(fmt) "piuio_gp: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>   /* KEY_COIN */
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ratelimit.h>
#include <linux/ktime.h>
#include <linux/string.h>

/* ───── device IDs ─────────────────────────────────────────────── */
#define VID_PIUIO     0x0d2f
#define PID_PIUIO1020 0x1020

/* ───── input report (buttons) ─────────────────────────────────── */
#define REP_ID   0x30        /* GET_REPORT(Input) id */
#define REP_LEN  16          /* 16 sensor bytes      */

/* ───── v2: coin feature report & wake frame  ──────────────────── */
#define COIN_RID         0x01      /* Feature-report id            */
#define COIN_LO          14        /* little-end counter bytes     */
#define COIN_HI          15
#define COIN_BUF_LEN     64        /* plenty                       */
#define WAKE_RID         0x81      /* Output-report id             */
#define WAKE_LEN         19        /* 1 id + 18 data               */

/* ───── button ↔ bit map (active-low) ──────────────────────────── */
struct mapent { u8 mask; u16 p1_btn; u16 p2_btn; };
static const struct mapent map[5] = {
	/* bit↓      P1 code        P2 code          */
	{ 0x01, BTN_SOUTH , BTN_TR      }, /* DL  */
	{ 0x08, BTN_WEST  , BTN_SELECT  }, /* DR  */
	{ 0x04, BTN_NORTH , BTN_START   }, /* UL  */
	{ 0x02, BTN_EAST  , BTN_THUMBL  }, /* UR  */
	{ 0x10, BTN_TL    , BTN_THUMBR  }, /* CTR */
};
#define NBTN 10                                   /* 5 per player */

/* ───── driver private ─────────────────────────────────────────── */
struct piuio {
	struct hid_device *hdev;
	struct usb_device *udev;
	int                iface;

	u8  buf[REP_LEN];
	bool down[NBTN];
	/* --- v2 --- */
	u16 coin_last;
	u8  coin_buf[COIN_BUF_LEN];

	struct input_dev *idev;

	struct work_struct poll_wq;
	struct hrtimer     timer;
	atomic_t           stop;
};
static struct piuio *ctx;

/* ───── module param – poll interval ───────────────────────────── */
static int poll_ms = 4;
module_param(poll_ms, int, 0444);
static inline int clamp_poll(void){ return clamp_val(poll_ms,1,1000); }

/* ───── USB helpers ─────────────────────────────────────────────── */
static int get_matrix(struct piuio *p)
{
	u16 w = (HID_INPUT_REPORT<<8)|REP_ID;
	int r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev,0),
				HID_REQ_GET_REPORT,
				USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
				w,p->iface,p->buf,REP_LEN,30);
	return (r==REP_LEN)?0:(r<0?r:-EIO);
}
static void set_idle(struct piuio *p)
{
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev,0),
			HID_REQ_SET_IDLE,
			USB_DIR_OUT|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
			0,p->iface,NULL,0,10);
}
/* --- v2 --- single wake frame (all-0xFF) once at probe */
static void wake_board(struct piuio *p)
{
	u8 buf[WAKE_LEN];
	buf[0]=WAKE_RID;
	memset(buf+1,0xFF,WAKE_LEN-1);
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev,0),
			HID_REQ_SET_REPORT,
			USB_DIR_OUT|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
			(HID_OUTPUT_REPORT<<8)|WAKE_RID,
			p->iface,buf,WAKE_LEN,30);
}
/* --- v2 --- read coins (feature rpt 0x01) */
static int get_coins(struct piuio *p,u16 *val)
{
	int r=usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev,0),
			      HID_REQ_GET_REPORT,
			      USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
			      (HID_FEATURE_REPORT<<8)|COIN_RID,
			      p->iface,p->coin_buf,COIN_BUF_LEN,30);
	if(r<COIN_HI+1) return -EIO;
	*val=p->coin_buf[COIN_LO] | (p->coin_buf[COIN_HI]<<8);
	return 0;
}

/* ───── translate matrix → events ──────────────────────────────── */
static void push_events(struct piuio *p)
{
	bool now[NBTN]={0};
	struct input_dev *id=p->idev;
	int i,b;

	for(i=0;i<4;i++){
		u8 v=p->buf[i];
		for(b=0;b<5;b++) if(!(v&map[b].mask)) now[b]=true;
	}
	for(i=4;i<8;i++){
		u8 v=p->buf[i];
		for(b=0;b<5;b++) if(!(v&map[b].mask)) now[5+b]=true;
	}

	for(b=0;b<NBTN;b++) if(now[b]!=p->down[b]){
		input_report_key(id,
			(b<5)?map[b].p1_btn:map[b-5].p2_btn, now[b]);
		p->down[b]=now[b];
	}
}

/* ───── poll machinery ─────────────────────────────────────────── */
static DEFINE_RATELIMIT_STATE(rl,HZ,10);

static void poll_work(struct work_struct *w)
{
	if(!ctx||atomic_read(&ctx->stop))return;

	if(!get_matrix(ctx)){
		if(__ratelimit(&rl))pr_debug("%*phN\n",REP_LEN,ctx->buf);
		push_events(ctx);
	}

	/* --- v2 --- coin every ~100 ms */
	static ktime_t last; ktime_t now=ktime_get();
	if(ktime_ms_delta(now,last)>=100){
		u16 v;
		if(!get_coins(ctx,&v) && v!=ctx->coin_last){
			input_report_key(ctx->idev,KEY_COIN,1);
			input_report_key(ctx->idev,KEY_COIN,0);
			ctx->coin_last=v;
		}
		last=now;
	}

	input_sync(ctx->idev);
}
static enum hrtimer_restart t_cb(struct hrtimer *t)
{
	if(ctx&&!atomic_read(&ctx->stop))
		queue_work(system_unbound_wq,&ctx->poll_wq);
	hrtimer_forward_now(t,ms_to_ktime(clamp_poll()));
	return atomic_read(&ctx->stop)?HRTIMER_NORESTART:HRTIMER_RESTART;
}

/* ───── probe/remove ───────────────────────────────────────────── */
static int probe(struct hid_device *hdev,const struct hid_device_id *id)
{
	struct usb_interface *itf=to_usb_interface(hdev->dev.parent);
	struct piuio *p; int r,b;

	if(ctx) return -EBUSY;
	p=devm_kzalloc(&hdev->dev,sizeof(*p),GFP_KERNEL);
	if(!p)return -ENOMEM;
	p->hdev=hdev; p->udev=interface_to_usbdev(itf);
	p->iface=itf->cur_altsetting->desc.bInterfaceNumber;
	hid_set_drvdata(hdev,p);

	if((r=hid_parse(hdev))||(r=hid_hw_start(hdev,HID_CONNECT_HIDRAW)))
		return r;
	set_idle(p);          /* bios does this                */
	wake_board(p);        /* required for coin, quick hack */

	p->idev=devm_input_allocate_device(&hdev->dev);
	if(!p->idev){r=-ENOMEM;goto err_hw;}
	p->idev->name="PIUIO Gamepad + Coin";
	p->idev->phys="piuio/js0";
	p->idev->id.bustype=BUS_USB;
	p->idev->id.vendor=VID_PIUIO;
	p->idev->id.product=PID_PIUIO1020;
	set_bit(EV_KEY,p->idev->evbit);
	for(b=0;b<NBTN;b++)
		set_bit((b<5)?map[b].p1_btn:map[b-5].p2_btn,p->idev->keybit);
	set_bit(KEY_COIN,p->idev->keybit);   /* --- v2 --- */

	get_coins(p,&p->coin_last);          /* prime value */

	if((r=input_register_device(p->idev)))goto err_hw;

	ctx=p;
	INIT_WORK(&p->poll_wq,poll_work);
	hrtimer_init(&p->timer,CLOCK_MONOTONIC,HRTIMER_MODE_REL);
	p->timer.function=t_cb;
	hrtimer_start(&p->timer,ms_to_ktime(clamp_poll()),HRTIMER_MODE_REL);
	pr_info("attached – poll %d ms\n",clamp_poll());
	return 0;

err_hw:
	hid_hw_stop(hdev);
	return r;
}

static void remove(struct hid_device *hdev)
{
	struct piuio *p=hid_get_drvdata(hdev);
	if(!p)return;
	atomic_set(&p->stop,1);
	hrtimer_cancel(&p->timer);
	cancel_work_sync(&p->poll_wq);
	ctx=NULL;
	input_unregister_device(p->idev);
	hid_hw_stop(hdev);
}

/* ───── boiler-plate ───────────────────────────────────────────── */
static const struct hid_device_id ids[]={
	{ HID_USB_DEVICE(VID_PIUIO,PID_PIUIO1020)},{}
};
MODULE_DEVICE_TABLE(hid,ids);

static struct hid_driver drv={
	.name="piuio",
	.id_table=ids,
	.probe=probe,
	.remove=remove,
};
module_hid_driver(drv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO 0x1020");