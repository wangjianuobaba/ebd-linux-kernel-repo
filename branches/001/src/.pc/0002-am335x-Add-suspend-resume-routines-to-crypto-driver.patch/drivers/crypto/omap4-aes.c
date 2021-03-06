/*
 * Cryptographic API.
 *
 * Support for OMAP AES HW acceleration.
 *
 * Copyright (c) 2010 Nokia Corporation
 * Author: Dmitry Kasatkin <dmitry.kasatkin@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 */
/*
 * Copyright © 2011 Texas Instruments Incorporated
 * Author: Herman Schuurman
 * Change: July 2011 - Adapted the omap-aes.c driver to support Netra
 *	implementation of AES hardware accelerator.
 */
/*
 * Copyright © 2011 Texas Instruments Incorporated
 * Author: Greg Turner
 * Change: November 2011 - Adapted for AM33x support HW accelerator.
 */

//#define	DEBUG

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/crypto.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <crypto/scatterwalk.h>
#include <crypto/aes.h>

#include <plat/cpu.h>
#include <plat/dma.h>
#include <mach/edma.h>
#include <mach/hardware.h>
#include "omap4.h"

#define DEFAULT_TIMEOUT		(5*HZ)

#define FLAGS_MODE_MASK		0x000f
#define FLAGS_ENCRYPT		BIT(0)
#define FLAGS_CBC		BIT(1)
#define	FLAGS_CTR		BIT(2)
#define FLAGS_GIV		BIT(3)

#define FLAGS_INIT		BIT(4)
#define FLAGS_FAST		BIT(5)
#define FLAGS_BUSY		BIT(6)

struct omap4_aes_ctx {
	struct omap4_aes_dev *dd;

	int		keylen;
	u32		key[AES_KEYSIZE_256 / sizeof(u32)];
	unsigned long	flags;
};

struct omap4_aes_reqctx {
	unsigned long mode;
};

#define AM33X_AES_QUEUE_LENGTH	1
#define AM33X_AES_CACHE_SIZE	0

struct omap4_aes_dev {
	struct list_head		list;
	unsigned long			phys_base;
	void __iomem			*io_base;
	struct clk			*iclk;
	struct omap4_aes_ctx		*ctx;
	struct device			*dev;
	unsigned long			flags;
	int				err;

	spinlock_t			lock;
	struct crypto_queue		queue;

	struct tasklet_struct		done_task;
	struct tasklet_struct		queue_task;

	struct ablkcipher_request	*req;
	size_t				total;
	struct scatterlist		*in_sg;
	size_t				in_offset;
	struct scatterlist		*out_sg;
	size_t				out_offset;

	size_t				buflen;
	void				*buf_in;
	size_t				dma_size;
	int				dma_in;
	int				dma_lch_in;
	dma_addr_t			dma_addr_in;
	void				*buf_out;
	int				dma_out;
	int				dma_lch_out;
	dma_addr_t			dma_addr_out;
};

/* keep registered devices data here */
static LIST_HEAD(dev_list);
static DEFINE_SPINLOCK(list_lock);

static inline u32 omap4_aes_read(struct omap4_aes_dev *dd, u32 offset)
{
	return __raw_readl(dd->io_base + offset);
}

static inline void omap4_aes_write(struct omap4_aes_dev *dd, u32 offset,
				  u32 value)
{
	__raw_writel(value, dd->io_base + offset);
}

static inline void omap4_aes_write_mask(struct omap4_aes_dev *dd, u32 offset,
				       u32 value, u32 mask)
{
	u32 val;

	val = omap4_aes_read(dd, offset);
	val &= ~mask;
	val |= value;
	omap4_aes_write(dd, offset, val);
}

static void omap4_aes_write_n(struct omap4_aes_dev *dd, u32 offset,
			     u32 *value, int count)
{
	for (; count--; value++, offset += 4)
		omap4_aes_write(dd, offset, *value);
}

static int omap4_aes_hw_init(struct omap4_aes_dev *dd)
{
	omap4_aes_write(dd, AES_REG_SYSCFG, 0);

	if (!(dd->flags & FLAGS_INIT)) {
		dd->flags |= FLAGS_INIT;
		dd->err = 0;
	}

	return 0;
}

static int omap4_aes_write_ctrl(struct omap4_aes_dev *dd)
{
	unsigned int key32;
	int i, err;
	u32 val, mask;

	err = omap4_aes_hw_init(dd);
	if (err)
		return err;

	pr_debug("Set key\n");
	key32 = dd->ctx->keylen / sizeof(u32);

	/* set a key */
	for (i = 0; i < key32; i++) {
		omap4_aes_write(dd, AES_REG_KEY1(i),
			       __le32_to_cpu(dd->ctx->key[i]));
	}

	if ((dd->flags & (FLAGS_CBC | FLAGS_CTR)) && dd->req->info)
		omap4_aes_write_n(dd, AES_REG_IV(0), dd->req->info, 4);

	val = FLD_VAL(((dd->ctx->keylen >> 3) - 1), 4, 3);
	if (dd->flags & FLAGS_CBC)
		val |= AES_REG_CTRL_CBC;
	else if (dd->flags & FLAGS_CTR)
		val |= AES_REG_CTRL_CTR | AES_REG_CTRL_CTR_WIDTH_32;
	if (dd->flags & FLAGS_ENCRYPT)
		val |= AES_REG_CTRL_DIRECTION;

	mask = AES_REG_CTRL_CBC | AES_REG_CTRL_CTR | AES_REG_CTRL_DIRECTION |
		AES_REG_CTRL_KEY_SIZE_MASK | AES_REG_CTRL_CTR_WIDTH_MASK;

	omap4_aes_write_mask(dd, AES_REG_CTRL, val, mask);

	return 0;
}

static struct omap4_aes_dev *omap4_aes_find_dev(struct omap4_aes_ctx *ctx)
{
	struct omap4_aes_dev *dd = NULL, *tmp;

	spin_lock_bh(&list_lock);
	if (!ctx->dd) {
		list_for_each_entry(tmp, &dev_list, list) {
			/* FIXME: take fist available aes core */
			dd = tmp;
			break;
		}
		ctx->dd = dd;
	} else {
		/* already found before */
		dd = ctx->dd;
	}
	spin_unlock_bh(&list_lock);

	return dd;
}

static void omap4_aes_dma_callback(unsigned int lch, u16 ch_status, void *data)
{
	struct omap4_aes_dev *dd = data;

	edma_stop(lch);

	if (ch_status != DMA_COMPLETE) {
		pr_err("omap4-aes DMA error status: 0x%hx\n", ch_status);
		dd->err = -EIO;
		dd->flags &= ~FLAGS_INIT; /* request to re-initialize */
	} else if (lch == dd->dma_lch_in) {
		return;
	}

	/* dma_lch_out - completed */
	tasklet_schedule(&dd->done_task);
}

static int omap4_aes_dma_init(struct omap4_aes_dev *dd)
{
	int err = -ENOMEM;

	dd->dma_lch_out = -1;
	dd->dma_lch_in = -1;

	dd->buf_in = (void *)__get_free_pages(GFP_KERNEL, AM33X_AES_CACHE_SIZE);
	dd->buf_out = (void *)__get_free_pages(GFP_KERNEL, AM33X_AES_CACHE_SIZE);
	dd->buflen = PAGE_SIZE << AM33X_AES_CACHE_SIZE;
	dd->buflen &= ~(AES_BLOCK_SIZE - 1);

	if (!dd->buf_in || !dd->buf_out) {
		dev_err(dd->dev, "unable to alloc pages.\n");
		goto err_alloc;
	}

	/* MAP here */
	dd->dma_addr_in = dma_map_single(dd->dev, dd->buf_in, dd->buflen,
					 DMA_TO_DEVICE);
	if (dma_mapping_error(dd->dev, dd->dma_addr_in)) {
		dev_err(dd->dev, "dma %d bytes error\n", dd->buflen);
		err = -EINVAL;
		goto err_map_in;
	}

	dd->dma_addr_out = dma_map_single(dd->dev, dd->buf_out, dd->buflen,
					  DMA_FROM_DEVICE);
	if (dma_mapping_error(dd->dev, dd->dma_addr_out)) {
		dev_err(dd->dev, "dma %d bytes error\n", dd->buflen);
		err = -EINVAL;
		goto err_map_out;
	}

	dd->dma_lch_in = edma_alloc_channel(dd->dma_in, omap4_aes_dma_callback,
					    dd, EVENTQ_DEFAULT);

	if (dd->dma_lch_in < 0) {
		dev_err(dd->dev, "Unable to request DMA channel\n");
		goto err_dma_in;
	}

	dd->dma_lch_out = edma_alloc_channel(dd->dma_out, omap4_aes_dma_callback, dd, EVENTQ_2);

	if (dd->dma_lch_out < 0) {
		dev_err(dd->dev, "Unable to request DMA channel\n");
		goto err_dma_out;
	}

	return 0;

err_dma_out:
	edma_free_channel(dd->dma_lch_in);
err_dma_in:
	dma_unmap_single(dd->dev, dd->dma_addr_out, dd->buflen,
			 DMA_FROM_DEVICE);
err_map_out:
	dma_unmap_single(dd->dev, dd->dma_addr_in, dd->buflen, DMA_TO_DEVICE);
err_map_in:
	free_pages((unsigned long)dd->buf_out, AM33X_AES_CACHE_SIZE);
	free_pages((unsigned long)dd->buf_in, AM33X_AES_CACHE_SIZE);
err_alloc:
	if (err)
		pr_err("error: %d\n", err);
	return err;
}

static void omap4_aes_dma_cleanup(struct omap4_aes_dev *dd)
{
	edma_free_channel(dd->dma_lch_out);
	edma_free_channel(dd->dma_lch_in);
	dma_unmap_single(dd->dev, dd->dma_addr_out, dd->buflen,
			 DMA_FROM_DEVICE);
	dma_unmap_single(dd->dev, dd->dma_addr_in, dd->buflen, DMA_TO_DEVICE);
	free_pages((unsigned long)dd->buf_out, AM33X_AES_CACHE_SIZE);
	free_pages((unsigned long)dd->buf_in, AM33X_AES_CACHE_SIZE);
}

static void sg_copy_buf(void *buf, struct scatterlist *sg,
			unsigned int start, unsigned int nbytes, int out)
{
	struct scatter_walk walk;

	if (!nbytes)
		return;

	scatterwalk_start(&walk, sg);
	scatterwalk_advance(&walk, start);
	scatterwalk_copychunks(buf, &walk, nbytes, out);
	scatterwalk_done(&walk, out, 0);
}

static int sg_copy(struct scatterlist **sg, size_t *offset, void *buf,
		   size_t buflen, size_t total, int out)
{
	unsigned int count, off = 0;

	while (buflen && total) {
		count = min((*sg)->length - *offset, total);
		count = min(count, buflen);

		if (!count)
			return off;

		/*
		 * buflen and total are AES_BLOCK_SIZE size aligned,
		 * so count should be also aligned
		 */

		sg_copy_buf(buf + off, *sg, *offset, count, out);

		off += count;
		buflen -= count;
		*offset += count;
		total -= count;

		if (*offset == (*sg)->length) {
			*sg = sg_next(*sg);
			if (*sg)
				*offset = 0;
			else
				total = 0;
		}
	}

	return off;
}

static int omap4_aes_crypt_dma(struct crypto_tfm *tfm, dma_addr_t dma_addr_in,
			      dma_addr_t dma_addr_out, int length)
{
	struct omap4_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	struct omap4_aes_dev *dd = ctx->dd;
	int nblocks;
	struct edmacc_param p_ram;

	pr_debug("len: %d\n", length);

	dd->dma_size = length;

	if (!(dd->flags & FLAGS_FAST))
		dma_sync_single_for_device(dd->dev, dma_addr_in, length,
					   DMA_TO_DEVICE);

	nblocks = DIV_ROUND_UP(length, AES_BLOCK_SIZE);

	/* EDMA IN */
	p_ram.opt	   = TCINTEN |
		EDMA_TCC(EDMA_CHAN_SLOT(dd->dma_lch_in));
	p_ram.src	   = dma_addr_in;
	p_ram.a_b_cnt      = AES_BLOCK_SIZE | nblocks << 16;
	p_ram.dst          = dd->phys_base + AES_REG_DATA;
	p_ram.src_dst_bidx = AES_BLOCK_SIZE;
	p_ram.link_bcntrld = 1 << 16 | 0xFFFF;
	p_ram.src_dst_cidx = 0;
	p_ram.ccnt         = 1;
	edma_write_slot(dd->dma_lch_in, &p_ram);

	/* EDMA OUT */
	p_ram.opt	   = TCINTEN |
		EDMA_TCC(EDMA_CHAN_SLOT(dd->dma_lch_out));
	p_ram.src	   = dd->phys_base + AES_REG_DATA;
	p_ram.dst          = dma_addr_out;
	p_ram.src_dst_bidx = AES_BLOCK_SIZE << 16;
	edma_write_slot(dd->dma_lch_out, &p_ram);

	edma_start(dd->dma_lch_in);
	edma_start(dd->dma_lch_out);

	/* write data length info out */
	omap4_aes_write(dd, AES_REG_LENGTH_N(0), length);
	omap4_aes_write(dd, AES_REG_LENGTH_N(1), 0);
	/* start DMA or disable idle mode */
	omap4_aes_write_mask(dd, AES_REG_SYSCFG,
			   AES_REG_SYSCFG_DREQ_DATA_OUT_EN | AES_REG_SYSCFG_DREQ_DATA_IN_EN,
			   AES_REG_SYSCFG_DREQ_MASK);

	return 0;
}

static int omap4_aes_crypt_dma_start(struct omap4_aes_dev *dd)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(
					crypto_ablkcipher_reqtfm(dd->req));
	int err, fast = 0, in, out;
	size_t count;
	dma_addr_t addr_in, addr_out;

	pr_debug("total: %d\n", dd->total);

	if (sg_is_last(dd->in_sg) && sg_is_last(dd->out_sg)) {
		/* check for alignment */
		in = IS_ALIGNED((u32)dd->in_sg->offset, sizeof(u32));
		out = IS_ALIGNED((u32)dd->out_sg->offset, sizeof(u32));

		fast = in && out;
	}

	if (fast)  {
		count = min(dd->total, sg_dma_len(dd->in_sg));
		count = min(count, sg_dma_len(dd->out_sg));

		if (count != dd->total) {
			pr_err("request length != buffer length\n");
			return -EINVAL;
		}

		pr_debug("fast\n");

		err = dma_map_sg(dd->dev, dd->in_sg, 1, DMA_TO_DEVICE);
		if (!err) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			return -EINVAL;
		}

		err = dma_map_sg(dd->dev, dd->out_sg, 1, DMA_FROM_DEVICE);
		if (!err) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			dma_unmap_sg(dd->dev, dd->in_sg, 1, DMA_TO_DEVICE);
			return -EINVAL;
		}

		addr_in = sg_dma_address(dd->in_sg);
		addr_out = sg_dma_address(dd->out_sg);

		dd->flags |= FLAGS_FAST;

	} else {
		/* use cache buffers */
		count = sg_copy(&dd->in_sg, &dd->in_offset, dd->buf_in,
				dd->buflen, dd->total, 0);

		addr_in = dd->dma_addr_in;
		addr_out = dd->dma_addr_out;

		dd->flags &= ~FLAGS_FAST;

	}

	dd->total -= count;

	err = omap4_aes_crypt_dma(tfm, addr_in, addr_out, count);
	if (err) {
		dma_unmap_sg(dd->dev, dd->in_sg, 1, DMA_TO_DEVICE);
		dma_unmap_sg(dd->dev, dd->out_sg, 1, DMA_TO_DEVICE);
	}

	return err;
}

static void omap4_aes_finish_req(struct omap4_aes_dev *dd, int err)
{
	struct ablkcipher_request *req = dd->req;

	pr_debug("err: %d\n", err);

	dd->flags &= ~FLAGS_BUSY;

	req->base.complete(&req->base, err);
}

static int omap4_aes_crypt_dma_stop(struct omap4_aes_dev *dd)
{
	int err = 0;
	size_t count;

	pr_debug("total: %d\n", dd->total);

	omap4_aes_write_mask(dd, AES_REG_SYSCFG, 0, AES_REG_SYSCFG_DREQ_MASK);

	edma_stop(dd->dma_lch_in);
	edma_clean_channel(dd->dma_lch_in);
	edma_stop(dd->dma_lch_out);
	edma_clean_channel(dd->dma_lch_out);

	if (dd->flags & FLAGS_FAST) {
		dma_unmap_sg(dd->dev, dd->out_sg, 1, DMA_FROM_DEVICE);
		dma_unmap_sg(dd->dev, dd->in_sg, 1, DMA_TO_DEVICE);
	} else {
		dma_sync_single_for_device(dd->dev, dd->dma_addr_out,
					   dd->dma_size, DMA_FROM_DEVICE);

		/* copy data */
		count = sg_copy(&dd->out_sg, &dd->out_offset, dd->buf_out,
				dd->buflen, dd->dma_size, 1);
		if (count != dd->dma_size) {
			err = -EINVAL;
			pr_err("not all data converted: %u\n", count);
		}
	}

	return err;
}

static int omap4_aes_handle_queue(struct omap4_aes_dev *dd,
				 struct ablkcipher_request *req)
{
	struct crypto_async_request *async_req, *backlog;
	struct omap4_aes_ctx *ctx;
	struct omap4_aes_reqctx *rctx;
	unsigned long flags;
	int err, ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (req)
		ret = ablkcipher_enqueue_request(&dd->queue, req);

	if (dd->flags & FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&dd->queue);
	async_req = crypto_dequeue_request(&dd->queue);
	if (async_req)
		dd->flags |= FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ablkcipher_request_cast(async_req);

	/* assign new request to device */
	dd->req = req;
	dd->total = req->nbytes;
	dd->in_offset = 0;
	dd->in_sg = req->src;
	dd->out_offset = 0;
	dd->out_sg = req->dst;

	rctx = ablkcipher_request_ctx(req);
	ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	rctx->mode &= FLAGS_MODE_MASK;
	dd->flags = (dd->flags & ~FLAGS_MODE_MASK) | rctx->mode;

	dd->ctx = ctx;
	ctx->dd = dd;

	err = omap4_aes_write_ctrl(dd);
	if (!err)
		err = omap4_aes_crypt_dma_start(dd);
	if (err) {
		/* aes_task will not finish it, so do it here */
		omap4_aes_finish_req(dd, err);
		tasklet_schedule(&dd->queue_task);
	}

	return ret; /* return ret, which is enqueue return value */
}

static void omap4_aes_done_task(unsigned long data)
{
	struct omap4_aes_dev *dd = (struct omap4_aes_dev *)data;
	int err;

	pr_debug("enter\n");

	err = omap4_aes_crypt_dma_stop(dd);

	err = dd->err ? : err;

	if (dd->total && !err) {
		err = omap4_aes_crypt_dma_start(dd);
		if (!err)
			return; /* DMA started. Not finishing. */
	}

	omap4_aes_finish_req(dd, err);
	omap4_aes_handle_queue(dd, NULL);

	pr_debug("exit\n");
}

static void omap4_aes_queue_task(unsigned long data)
{
	struct omap4_aes_dev *dd = (struct omap4_aes_dev *)data;

	omap4_aes_handle_queue(dd, NULL);
}

static int omap4_aes_crypt(struct ablkcipher_request *req, unsigned long mode)
{
	struct omap4_aes_ctx *ctx = crypto_ablkcipher_ctx(
		crypto_ablkcipher_reqtfm(req));
	struct omap4_aes_reqctx *rctx = ablkcipher_request_ctx(req);
	struct omap4_aes_dev *dd;

	pr_debug("nbytes: %d, enc: %d, cbc: %d, ctr: %d\n", req->nbytes,
		 !!(mode & FLAGS_ENCRYPT),
		 !!(mode & FLAGS_CBC),
		 !!(mode & FLAGS_CTR));

	if (!IS_ALIGNED(req->nbytes, AES_BLOCK_SIZE)) {
		pr_err("request size is not exact amount of AES blocks\n");
		return -EINVAL;
	}

	dd = omap4_aes_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	rctx->mode = mode;

	return omap4_aes_handle_queue(dd, req);
}

/* ********************** ALG API ************************************ */

static int omap4_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			   unsigned int keylen)
{
	struct omap4_aes_ctx *ctx = crypto_ablkcipher_ctx(tfm);

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256)
		return -EINVAL;

	pr_debug("enter, keylen: %d\n", keylen);

	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;

	return 0;
}

static int omap4_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	return omap4_aes_crypt(req, FLAGS_ENCRYPT);
}

static int omap4_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	return omap4_aes_crypt(req, 0);
}

static int omap4_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	return omap4_aes_crypt(req, FLAGS_ENCRYPT | FLAGS_CBC);
}

static int omap4_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	return omap4_aes_crypt(req, FLAGS_CBC);
}

static int omap4_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	return omap4_aes_crypt(req, FLAGS_ENCRYPT | FLAGS_CTR);
}

static int omap4_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	return omap4_aes_crypt(req, FLAGS_CTR);
}

static int omap4_aes_cra_init(struct crypto_tfm *tfm)
{
	pr_debug("enter\n");

	tfm->crt_ablkcipher.reqsize = sizeof(struct omap4_aes_reqctx);

	return 0;
}

static void omap4_aes_cra_exit(struct crypto_tfm *tfm)
{
	pr_debug("enter\n");
}

/* ********************** ALGS ************************************ */

static struct crypto_alg algs[] = {
	{
		.cra_name		= "ecb(aes)",
		.cra_driver_name	= "ecb-aes-omap4",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct omap4_aes_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= omap4_aes_cra_init,
		.cra_exit		= omap4_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= omap4_aes_setkey,
			.encrypt	= omap4_aes_ecb_encrypt,
			.decrypt	= omap4_aes_ecb_decrypt,
		}
	},
	{
		.cra_name		= "cbc(aes)",
		.cra_driver_name	= "cbc-aes-omap4",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct omap4_aes_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= omap4_aes_cra_init,
		.cra_exit		= omap4_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.geniv		= "eseqiv",
			.ivsize		= AES_BLOCK_SIZE,
			.setkey		= omap4_aes_setkey,
			.encrypt	= omap4_aes_cbc_encrypt,
			.decrypt	= omap4_aes_cbc_decrypt,

		}
	},
	{
		.cra_name		= "ctr(aes)",
		.cra_driver_name	= "ctr-aes-omap4",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct omap4_aes_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= omap4_aes_cra_init,
		.cra_exit		= omap4_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.geniv		= "eseqiv",
			.ivsize		= AES_BLOCK_SIZE,
			.setkey		= omap4_aes_setkey,
			.encrypt	= omap4_aes_ctr_encrypt,
			.decrypt	= omap4_aes_ctr_decrypt,
		}
	}
};

static int omap4_aes_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct omap4_aes_dev *dd;
	struct resource *res;
	int err = -ENOMEM, i, j;
	u32 reg;

	dd = kzalloc(sizeof(struct omap4_aes_dev), GFP_KERNEL);
	if (dd == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		goto err_data;
	}
	dd->dev = dev;
	platform_set_drvdata(pdev, dd);

	spin_lock_init(&dd->lock);
	crypto_init_queue(&dd->queue, AM33X_AES_QUEUE_LENGTH);

	/* Get the base address */
	//res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//if (!res) {
	//	dev_err(dev, "invalid resource type\n");
	//	err = -ENODEV;
	//	goto err_res;
	//}

	//dd->phys_base = res->start;
	dd->phys_base = AM33XX_AES0_P_BASE;

	/* Get the DMA */
	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!res)
		dev_info(dev, "no DMA info\n");
	else
		dd->dma_out = res->start;

	/* Get the DMA */
	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (!res)
		dev_info(dev, "no DMA info\n");
	else
		dd->dma_in = res->start;

	pm_runtime_enable(dev);
	udelay(1);
	pm_runtime_get_sync(dev);
	udelay(1);

	dd->io_base = ioremap(dd->phys_base, SZ_4K);
	if (!dd->io_base) {
		dev_err(dev, "can't ioremap\n");
		err = -ENOMEM;
		goto err_io;
	}

	omap4_aes_hw_init(dd);
	reg = omap4_aes_read(dd, AES_REG_REV);
	
	dev_info(dev, "AM33X AES hw accel rev: %u.%02u\n",
		 ((reg & AES_REG_REV_X_MAJOR_MASK) >> 8),
		 (reg & AES_REG_REV_Y_MINOR_MASK));

	tasklet_init(&dd->done_task, omap4_aes_done_task, (unsigned long)dd);
	tasklet_init(&dd->queue_task, omap4_aes_queue_task, (unsigned long)dd);

	err = omap4_aes_dma_init(dd);
	if (err)
		goto err_dma;

	INIT_LIST_HEAD(&dd->list);
	spin_lock(&list_lock);
	list_add_tail(&dd->list, &dev_list);
	spin_unlock(&list_lock);

	for (i = 0; i < ARRAY_SIZE(algs); i++) {
		pr_debug("reg alg: %s\n", algs[i].cra_name);
		INIT_LIST_HEAD(&algs[i].cra_list);
		err = crypto_register_alg(&algs[i]);
		if (err)
			goto err_algs;
	}

	pr_info("probe() done\n");

	return 0;

err_algs:
	for (j = 0; j < i; j++)
		crypto_unregister_alg(&algs[j]);
	omap4_aes_dma_cleanup(dd);
err_dma:
	tasklet_kill(&dd->done_task);
	tasklet_kill(&dd->queue_task);
	iounmap(dd->io_base);

err_io:
	pm_runtime_put_sync(dev);
	udelay(1);
	pm_runtime_disable(dev);
	udelay(1);


err_res:
	kfree(dd);
	dd = NULL;
err_data:
	dev_err(dev, "initialization failed.\n");
	return err;
}

static int omap4_aes_remove(struct platform_device *pdev)
{
	struct omap4_aes_dev *dd = platform_get_drvdata(pdev);
	int i;

	if (!dd)
		return -ENODEV;

	spin_lock(&list_lock);
	list_del(&dd->list);
	spin_unlock(&list_lock);

	for (i = 0; i < ARRAY_SIZE(algs); i++)
		crypto_unregister_alg(&algs[i]);

	tasklet_kill(&dd->done_task);
	tasklet_kill(&dd->queue_task);
	omap4_aes_dma_cleanup(dd);
	iounmap(dd->io_base);
	pm_runtime_put_sync(&pdev->dev);
	udelay(1);
	pm_runtime_disable(&pdev->dev);
	udelay(1);

	kfree(dd);
	dd = NULL;

	return 0;
}

static struct platform_driver omap4_aes_driver = {
	.probe	= omap4_aes_probe,
	.remove	= omap4_aes_remove,
	.driver	= {
		.name	= "omap4-aes",
		.owner	= THIS_MODULE,
	},
};

static int __init omap4_aes_mod_init(void)
{
	pr_info("loading AM33X AES driver\n");

	/* This only works on a GP device */
	if (!cpu_is_am33xx() || omap_type() != OMAP2_DEVICE_TYPE_GP) {
		pr_err("Unsupported cpu\n");
		return -ENODEV;
	}
	return  platform_driver_register(&omap4_aes_driver);
}

static void __exit omap4_aes_mod_exit(void)
{
	pr_info("unloading AM33X AES driver\n");

	platform_driver_unregister(&omap4_aes_driver);
}

module_init(omap4_aes_mod_init);
module_exit(omap4_aes_mod_exit);

MODULE_DESCRIPTION("AM33X AES acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Herman Schuurman");
