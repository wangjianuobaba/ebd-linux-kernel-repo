/*
 *  Copyright 2013
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>


/* Partitions:
 * 0: boot 		(0x00000000 - 0x005FFFFF)	6M
 * 1: root   	(0x00600000 - 0x00FFFFFF)	10M
 */

#define FLASH_PHYS_ADDR 		0x10000000
#define FLASH_SIZE 				0x01000000

#define FLASH_PARTITION0_ADDR 	0x00000000
#define FLASH_PARTITION0_SIZE 	0x00600000

#define FLASH_PARTITION1_ADDR 	0x00600000
#define FLASH_PARTITION1_SIZE 	0x00A00000


struct map_info vnet30_map = {
		.name =		"VNet30 NorFlash Device",
		.size =		FLASH_SIZE,
		.bankwidth =	2,
};

struct mtd_partition vnet30_parts[] = {
	{
		.name =		"boot",
		.offset	=	FLASH_PARTITION0_ADDR,
		.size =		FLASH_PARTITION0_SIZE
	},
	{
		.name =		"root",
		.offset =	FLASH_PARTITION1_ADDR,
		.size =		FLASH_PARTITION1_SIZE
	},
};

#define PARTITION_COUNT ARRAY_SIZE(vnet30_parts)

static struct mtd_info *mymtd;

static int __init init_vnet30(void)
{
	printk(KERN_NOTICE "VNet30 NorFlash Device %x at %x\n",
			FLASH_SIZE, FLASH_PHYS_ADDR);

	vnet30_map.phys = FLASH_PHYS_ADDR;
	vnet30_map.virt = ioremap(FLASH_PHYS_ADDR, FLASH_SIZE);

	if (!vnet30_map.virt) {
		printk("Failed to ioremap\n");
		return -EIO;
	}

	simple_map_init(&vnet30_map);

	mymtd = do_map_probe("cfi_probe", &vnet30_map);
	if (mymtd) {
		mymtd->owner = THIS_MODULE;
		mtd_device_register(mymtd, vnet30_parts, PARTITION_COUNT);
		printk(KERN_NOTICE "VNet30 NorFlash Device initialized\n");
		return 0;
	}

	iounmap((void *)vnet30_map.virt);
	return -ENXIO;
}

static void __exit cleanup_vnet30(void)
{
	if (mymtd) {
		mtd_device_unregister(mymtd);
		map_destroy(mymtd);
	}
	if (vnet30_map.virt) {
		iounmap((void *)vnet30_map.virt);
		vnet30_map.virt = 0;
	}
}

module_init(init_vnet30);
module_exit(cleanup_vnet30);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("annoy");
MODULE_DESCRIPTION("MTD map driver for VNet30");
