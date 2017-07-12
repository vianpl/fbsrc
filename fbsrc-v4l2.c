/*
 * fbsrc-v4l2.c - FrameBuffer to v4l2 capture device driver
 *
 * Copyright (c) 2017, Prodys S.L.
 *
 * Author: Nicolas Saenz Julienne <nicolassaenzj@gmail.com>
 *
 * based on some ideas from videobuf2-fb.c by Marek Szyprowski
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-memops.h>

#define MODNAME				"fbsrc"
#define FBSRC_MIN_QUEUED_BUFS		2
#define FBSRC_NAME_LEN			32
#define FBSRC_MIN_WIDTH			320
#define FBSRC_MIN_HEIGHT		240
#define FBSRC_MAX_WIDTH			1920
#define FBSRC_MAX_HEIGHT		1080
#define FBSRC_MAX_FPS			120

static void fbsrc_dev_release(struct device *dev)
{}

static struct platform_device fbsrc_device = {
	.name		= MODNAME,
	.dev.release	= fbsrc_dev_release,
};

struct fbsrc {
	struct platform_device *pdev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct fb_info *fb_info;
	struct mutex lock;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct hrtimer timer;
	ktime_t interval;
	int refcount;
	void *fb;

	struct vb2_queue queue;
	struct fbsrc_buffer *buf;
	spinlock_t qlock;
	struct list_head buf_list;
	unsigned field;
	unsigned sequence;
};

struct fbsrc_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct fbsrc_fmt_desc {
	char name[FBSRC_NAME_LEN];
	__u32 fourcc;
	__u32 bits_per_pixel;
	struct fb_bitfield red;
	struct fb_bitfield green;
	struct fb_bitfield blue;
	struct fb_bitfield transp;
};

static const struct fbsrc_fmt_desc formats[] = {
	{
		.name = "16 bpp RGB, le",
		.fourcc = V4L2_PIX_FMT_RGB565,
		.bits_per_pixel = 16,
		.red = {	.offset = 0,	.length = 5,	},
		.green = {	.offset = 5,	.length = 6,	},
		.blue = {	.offset = 11,	.length = 5,	},
	}, {
		.name = "15 bpp RGB, le",
		.fourcc = V4L2_PIX_FMT_RGB555,
		.bits_per_pixel = 16,
		.red = {	.offset = 1,	.length = 5,	},
		.green = {	.offset = 5,	.length = 5,	},
		.blue = {	.offset = 11,	.length = 5,	},
	}, {
		.name = "32 bpp RGB, le",
		.fourcc = V4L2_PIX_FMT_BGR32,
		.bits_per_pixel = 32,
		.red = {	.offset = 16,	.length = 8,	},
		.green = {	.offset = 8,	.length = 8,	},
		.blue = {	.offset = 0,	.length = 8,	},
		.transp = {	.offset = 24,	.length = 8,	},
	}, {
		.name = "32 bpp RGB, be",
		.fourcc = V4L2_PIX_FMT_RGB32,
		.bits_per_pixel = 32,
		.red = {	.offset = 0,	.length = 8,	},
		.green = {	.offset = 8,	.length = 8,	},
		.blue = {	.offset = 16,	.length = 8,	},
		.transp = {	.offset = 24,	.length = 8,	},
	},
	/* TODO: add more format descriptors */
};

/* Default format is 720p50 */
static const struct v4l2_pix_format fbsrc_pix_format = {
	.width = 1280,
	.height = 720,
	.pixelformat = V4L2_PIX_FMT_RGB565,
	.field = V4L2_FIELD_NONE,
	.bytesperline = 1280 * 2,
	.sizeimage = 1280 * 720 * 2,
	.colorspace = V4L2_COLORSPACE_SRGB,
};

static const struct v4l2_fract fbsrc_fps = {
	.numerator = 1,
	.denominator = 60
};

static inline struct fbsrc_buffer *to_fbsrc_buffer(struct vb2_v4l2_buffer *vb2)
{
	return container_of(vb2, struct fbsrc_buffer, vb);
}

static int s3c_fb_ioctl(struct fb_info *fb_info, unsigned int cmd,
			unsigned long arg)
{
	return 0;
}

static int fbsrc_fb_open(struct fb_info *fb_info, int user)
{
	struct fbsrc *fbsrc = fb_info->par;

	/*
	 * Reject open() call from fb console.
	 */
	if (user == 0)
		return -ENODEV;

	/*
	 * Reject open() calls if the videodevice isn't ready yet
	 */
	if (!fbsrc->buf)
		return -ENODEV;

	return 0;
}

static int fbsrc_fb_release(struct fb_info *fb_info, int user)
{
	return 0;
}

static int fbsrc_fb_mmap(struct fb_info *fb_info, struct vm_area_struct *vma)
{
	struct fbsrc *fbsrc = fb_info->par;

	vma->vm_flags |= VM_READ;

	return vb2_mmap(&fbsrc->queue, vma);
}

static int fbsrc_fb_blank(int blank_mode, struct fb_info *fb_info)
{
	return 0;
}

static struct fb_ops fbsrc_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= fbsrc_fb_open,
	.fb_release	= fbsrc_fb_release,
	.fb_mmap	= fbsrc_fb_mmap,
	.fb_blank	= fbsrc_fb_blank,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_ioctl	= s3c_fb_ioctl,
};

static enum hrtimer_restart fbsrc_timer_work(struct hrtimer *timer)
{
	struct fbsrc *fbsrc = container_of(timer, struct fbsrc, timer);
	struct fbsrc_buffer *buf, *node;

	/*
	 * A basic frame buffer application mmaps a memory area and updates it's
	 * content based on the image to show any point in time. This conflicts
	 * whith the queue/dequeue scheme used by v4l2 as most applications
	 * (i.e. gstreamer, qv4l2) won't work if they are not provided with at least
	 * 2 buffers. What we do here is allow for any number of buffers to be
	 * queued, yet we only process and dequeue the one that the framebuffer
	 * application mmaped.
	 */
	spin_lock(&fbsrc->qlock);
	list_for_each_entry_safe(buf, node, &fbsrc->buf_list, list) {
		if (buf == fbsrc->buf) {
			list_del(&buf->list);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
			buf->vb.vb2_buf.timestamp = ktime_get_ns();
#else
			v4l2_get_timestamp(&buf->vb.timestamp);
#endif
			buf->vb.sequence = fbsrc->sequence++;
			buf->vb.field = fbsrc->field;
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		}
	}
	spin_unlock(&fbsrc->qlock);

	hrtimer_forward(&fbsrc->timer, ktime_get(), fbsrc->interval);

	return HRTIMER_RESTART;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
static int fbsrc_queue_setup(struct vb2_queue *vq,
			     unsigned int *nbuffers, unsigned int *nplanes,
			     unsigned int sizes[], struct device *alloc_devs[])
{
	struct fbsrc *fbsrc = vb2_get_drv_priv(vq);

	fbsrc->field = fbsrc->pix.field;

	if (vq->num_buffers + *nbuffers < FBSRC_MIN_QUEUED_BUFS)
		*nbuffers = FBSRC_MIN_QUEUED_BUFS - vq->num_buffers;

	if (*nplanes)
		return sizes[0] < fbsrc->pix.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = fbsrc->pix.sizeimage;

	return 0;
}
#else
static int fbsrc_queue_setup(struct vb2_queue *vq, const void *parg,
			    unsigned int *nbuffers, unsigned int *nplanes,
		   	    unsigned int sizes[], void *alloc_ctxs[])
{
	struct fbsrc *fbsrc = vb2_get_drv_priv(vq);
	unsigned long size = fbsrc->pix.sizeimage;
	const struct v4l2_format *fmt = parg;

	if (*nbuffers < FBSRC_MIN_QUEUED_BUFS)
		*nbuffers = FBSRC_MIN_QUEUED_BUFS;
	*nplanes = 1;

	if (fmt) {
		if (fmt->fmt.pix.sizeimage < size)
			return -EINVAL;
		size = fmt->fmt.pix.sizeimage;
	}
	sizes[0] = size;

	return 0;
}
#endif

static int fbsrc_buffer_prepare(struct vb2_buffer *vb)
{
	struct fbsrc *fbsrc = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = fbsrc->pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(&fbsrc->pdev->dev, "buffer too small (%lu < %lu)\n",
			 vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void fbsrc_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb);
	struct fbsrc *fbsrc = vb2_get_drv_priv(vb->vb2_queue);
	struct fbsrc_buffer *buf = to_fbsrc_buffer(v4l2_buf);
	unsigned long flags;

	spin_lock_irqsave(&fbsrc->qlock, flags);
	list_add_tail(&buf->list, &fbsrc->buf_list);
	if (!fbsrc->buf) {
		fbsrc->buf = buf;
		fbsrc->fb_info->screen_base = vb2_plane_vaddr(vb, 0);
	}
	spin_unlock_irqrestore(&fbsrc->qlock, flags);
}

static int fbsrc_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct fbsrc *fbsrc = vb2_get_drv_priv(vq);
	int ret = 0;

	fbsrc->sequence = 0;
	hrtimer_start(&fbsrc->timer, fbsrc->interval, HRTIMER_MODE_REL);
	return ret;
}

static void fbsrc_stop_streaming(struct vb2_queue *vq)
{
	struct fbsrc *fbsrc = vb2_get_drv_priv(vq);
	struct fbsrc_buffer *buf, *node;
	unsigned long flags;

	spin_lock_irqsave(&fbsrc->qlock, flags);
	list_for_each_entry_safe(buf, node, &fbsrc->buf_list, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&fbsrc->qlock, flags);

	hrtimer_cancel(&fbsrc->timer);
	fbsrc->buf = NULL;
}

static struct vb2_ops fbsrc_qops = {
	.queue_setup		= fbsrc_queue_setup,
	.buf_prepare		= fbsrc_buffer_prepare,
	.buf_queue		= fbsrc_buffer_queue,
	.start_streaming	= fbsrc_start_streaming,
	.stop_streaming		= fbsrc_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int fbsrc_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	struct fbsrc *fbsrc = video_drvdata(file);

	strlcpy(cap->driver, MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, "fbsrc", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(&fbsrc->pdev->dev));
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	cap->device_caps |= V4L2_CAP_VIDEO_CAPTURE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS |
			    V4L2_CAP_VIDEO_CAPTURE;
	return 0;
}

static const struct fbsrc_fmt_desc *fbsrc_get_fmt_desc(__u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++)
		if (fourcc == formats[i].fourcc)
			return &formats[i];

	return NULL;
}

static void fbsrc_fill_pix_format(struct fbsrc *fbsrc,
				  const struct v4l2_pix_format *pix)
{
	struct fb_info *fb_info = fbsrc->fb_info;
	struct fb_var_screeninfo *var = &fb_info->var;
	const struct fbsrc_fmt_desc *conv;

	conv = fbsrc_get_fmt_desc(pix->pixelformat);
	if (!conv)
		return;

	/* Fill v4l2 input src format */
	fbsrc->pix.width = pix->width;
	fbsrc->pix.height = pix->height;
	fbsrc->pix.bytesperline = pix->width * (conv->bits_per_pixel >> 3);
	fbsrc->pix.sizeimage = fbsrc->pix.bytesperline * pix->height;
	fbsrc->pix.pixelformat = pix->pixelformat;
	fbsrc->pix.field = pix->field;

	/* Fill frame buffer format */
	fb_info->screen_size = fbsrc->pix.sizeimage;
	fb_info->fix.line_length = fbsrc->pix.bytesperline;
	fb_info->fix.smem_len = fb_info->fix.mmio_len = fbsrc->pix.sizeimage;
	var->xres = var->xres_virtual = var->width = fbsrc->pix.width;
	var->yres = var->yres_virtual = var->height = fbsrc->pix.height;
	var->bits_per_pixel = conv->bits_per_pixel;
	var->red = conv->red;
	var->green = conv->green;
	var->blue = conv->blue;
	var->transp = conv->transp;
}

static int fbsrc_try_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	const struct fbsrc_fmt_desc *conv;

	conv = fbsrc_get_fmt_desc(pix->pixelformat);
	if (!conv) {
		pix->pixelformat = fbsrc_pix_format.pixelformat;
		conv = fbsrc_get_fmt_desc(pix->pixelformat);
	}

	if (pix->width < FBSRC_MIN_WIDTH)
		pix->width = FBSRC_MIN_WIDTH;
	if (pix->width > FBSRC_MAX_WIDTH)
		pix->width = FBSRC_MAX_WIDTH;
	if (pix->height < FBSRC_MIN_HEIGHT)
		pix->height = FBSRC_MIN_HEIGHT;
	if (pix->height > FBSRC_MAX_HEIGHT)
		pix->height = FBSRC_MAX_HEIGHT;

	pix->bytesperline = pix->width * (conv->bits_per_pixel >> 3);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static int fbsrc_s_fmt_vid_cap(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct fbsrc *fbsrc = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ret;

	if (vb2_is_busy(&fbsrc->queue))
		return -EBUSY;

	ret = fbsrc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	fbsrc_fill_pix_format(fbsrc, pix);

	return 0;
}

static int fbsrc_g_fmt_vid_cap(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct fbsrc *fbsrc = video_drvdata(file);

	f->fmt.pix = fbsrc->pix;

	return 0;
}

static int fbsrc_enum_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	strlcpy(f->description, formats[f->index].name, sizeof(f->description));
	f->pixelformat = formats[f->index].fourcc;

	return 0;
}

static int fbsrc_g_parm(struct file *file, void *priv,
		        struct v4l2_streamparm *sp)
{
	struct fbsrc *fbsrc = video_drvdata(file);
	struct v4l2_captureparm *cp = &sp->parm.capture;

	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe = fbsrc->timeperframe;

	return 0;
}

static void fbsrc_update_time_interval(struct fbsrc *fbsrc)
{
	struct v4l2_fract *timeperframe = &fbsrc->timeperframe;
	unsigned long nsecs;

	nsecs = (timeperframe->numerator * NSEC_PER_SEC) /
		 timeperframe->denominator;
	fbsrc->interval = ktime_set(0, nsecs);
}

static int fbsrc_s_parm(struct file *file, void *priv,
		        struct v4l2_streamparm *sp)
{
	struct fbsrc *fbsrc = video_drvdata(file);
	struct v4l2_captureparm *cp = &sp->parm.capture;
	struct v4l2_fract *timeperframe = &cp->timeperframe;

	if (vb2_is_busy(&fbsrc->queue))
		return -EBUSY;

	cp->capability = V4L2_CAP_TIMEPERFRAME;
	if (!timeperframe->denominator || !timeperframe->numerator)
		*timeperframe = fbsrc->timeperframe;
	else
		fbsrc->timeperframe = *timeperframe;
	fbsrc_update_time_interval(fbsrc);

	return 0;
}

static int fbsrc_enum_framesizes(struct file *file, void *priv,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index != 0)
		return -EINVAL;

	if (!fbsrc_get_fmt_desc(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = FBSRC_MIN_WIDTH;
	fsize->stepwise.min_height = FBSRC_MIN_HEIGHT;
	fsize->stepwise.max_width = FBSRC_MAX_WIDTH;
	fsize->stepwise.max_height = FBSRC_MAX_HEIGHT;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int fbsrc_enum_frameintervals(struct file *file, void *priv,
			             struct v4l2_frmivalenum *ival)
{
	if (ival->index != 0)
		return -EINVAL;

	if (!fbsrc_get_fmt_desc(ival->pixel_format) ||
	    ival->width < FBSRC_MIN_WIDTH || ival->height < FBSRC_MIN_HEIGHT ||
	    ival->width > FBSRC_MAX_WIDTH || ival->height > FBSRC_MAX_HEIGHT)
		return -EINVAL;

	ival->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	ival->stepwise.min.numerator = 1;
	ival->stepwise.min.denominator = FBSRC_MAX_FPS;
	ival->stepwise.max.numerator = 1;
	ival->stepwise.max.denominator = 1;
	ival->stepwise.step.numerator = 1;
	ival->stepwise.step.denominator = 1;

	return 0;
}


static int fbsrc_enum_input(struct file *file, void *priv,
			    struct v4l2_input *i)
{
	if (i->index > 0)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	strlcpy(i->name, "fbsrc", sizeof(i->name));
	i->capabilities = 0;
	i->std = 0;

	return 0;
}

static int fbsrc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct fbsrc *fbsrc = video_drvdata(file);

	if (i > 0)
		return -EINVAL;

	if (vb2_is_busy(&fbsrc->queue))
		return -EBUSY;

	return 0;
}

static int fbsrc_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static const struct v4l2_ioctl_ops fbsrc_ioctl_ops = {
	.vidioc_querycap = fbsrc_querycap,
	.vidioc_try_fmt_vid_cap = fbsrc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = fbsrc_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = fbsrc_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = fbsrc_enum_fmt_vid_cap,

	.vidioc_g_parm	= fbsrc_g_parm,
	.vidioc_s_parm	= fbsrc_s_parm,
	.vidioc_enum_framesizes	= fbsrc_enum_framesizes,
	.vidioc_enum_frameintervals = fbsrc_enum_frameintervals,

	.vidioc_enum_input = fbsrc_enum_input,
	.vidioc_g_input = fbsrc_g_input,
	.vidioc_s_input = fbsrc_s_input,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations fbsrc_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static int fbsrc_probe(struct platform_device *pdev)
{
	struct video_device *vdev;
	struct fb_info *fb_info;
	struct vb2_queue *q;
	struct fbsrc *fbsrc;
	int ret;

	/* Allocate a new instance */
	fbsrc = devm_kzalloc(&pdev->dev, sizeof(struct fbsrc), GFP_KERNEL);
	if (!fbsrc)
		return -ENOMEM;

	fbsrc->pdev = pdev;
	hrtimer_init(&fbsrc->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	fbsrc->timer.function = fbsrc_timer_work;
	fbsrc->timeperframe = fbsrc_fps;
	fbsrc->pix = fbsrc_pix_format;
	fbsrc_update_time_interval(fbsrc);

	/* Initialize the top-level structure */
	ret = v4l2_device_register(&pdev->dev, &fbsrc->v4l2_dev);
	if (ret)
		goto exit;

	mutex_init(&fbsrc->lock);

	/* Initialize the vb2 queue */
	q = &fbsrc->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
	q->dev = &pdev->dev;
#endif
	q->drv_priv = fbsrc;
	q->buf_struct_size = sizeof(struct fbsrc_buffer);
	q->ops = &fbsrc_qops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 1;
	q->lock = &fbsrc->lock;
	ret = vb2_queue_init(q);
	if (ret)
		goto free_v4l2;

	INIT_LIST_HEAD(&fbsrc->buf_list);
	spin_lock_init(&fbsrc->qlock);

	/* Initialize the video_device */
	vdev = &fbsrc->vdev;
	strlcpy(vdev->name, MODNAME, sizeof(vdev->name));
	vdev->release = video_device_release_empty;
	vdev->fops = &fbsrc_fops,
	vdev->ioctl_ops = &fbsrc_ioctl_ops,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
#endif
	vdev->lock = &fbsrc->lock;
	vdev->queue = q;
	vdev->v4l2_dev = &fbsrc->v4l2_dev;
	video_set_drvdata(vdev, fbsrc);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto free_v4l2;

	/* Initialize the frame buffer device */
	fb_info = framebuffer_alloc(sizeof(struct fbsrc *), &vdev->dev);
	if (!fb_info) {
		ret = -ENOMEM;
		goto free_vdev;
	}

	fbsrc->fb_info = fb_info;
	fb_info->par = fbsrc;
	fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb_info->fix.accel = FB_ACCEL_NONE;
	fb_info->fix.visual = FB_VISUAL_TRUECOLOR,
	fb_info->var.activate = FB_ACTIVATE_NOW;
	fb_info->var.vmode = FB_VMODE_NONINTERLACED;
	fb_info->fbops = &fbsrc_fb_ops;
	fb_info->flags = FBINFO_FLAG_DEFAULT;
	fb_info->screen_base = NULL;
	fbsrc_fill_pix_format(fbsrc, &fbsrc_pix_format);
	ret = register_framebuffer(fb_info);
	if (ret)
		goto free_fb;

	v4l2_info(&fbsrc->v4l2_dev, "fb%d: registered frame buffer emulation for /dev/%s\n",
	          fb_info->node, dev_name(&vdev->dev));

	return 0;

free_fb:
	framebuffer_release(fb_info);
free_vdev:
	video_unregister_device(&fbsrc->vdev);
free_v4l2:
	v4l2_device_unregister(&fbsrc->v4l2_dev);
exit:
	return ret;
}

static int fbsrc_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct fbsrc *fbsrc = container_of(v4l2_dev, struct fbsrc, v4l2_dev);

	unregister_framebuffer(fbsrc->fb_info);
	video_unregister_device(&fbsrc->vdev);
	v4l2_device_unregister(&fbsrc->v4l2_dev);

	return 0;
}

static struct platform_driver fbsrc_driver = {
	.probe          = fbsrc_probe,
	.remove         = fbsrc_remove,
	.driver         = {
		.name           = MODNAME,
	},
};

static void __exit fbsrc_exit(void)
{
	platform_driver_unregister(&fbsrc_driver);
	platform_device_unregister(&fbsrc_device);
}

static int __init fbsrc_init(void)
{
	int ret;

	ret = platform_device_register(&fbsrc_device);
	if (ret)
		return ret;

	ret = platform_driver_register(&fbsrc_driver);
	if (ret)
		platform_device_unregister(&fbsrc_device);

	return ret;
}

module_init(fbsrc_init);
module_exit(fbsrc_exit);

MODULE_DESCRIPTION("Frame buffer to v4l2 capture device driver");
MODULE_AUTHOR("Nicolas Saenz Julienne");
MODULE_LICENSE("GPL v2");
