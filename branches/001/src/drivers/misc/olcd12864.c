#define DEBUG
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>

#include <linux/spi/spi.h>

#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/olcd12864.h>
#include <linux/delay.h>



#define SPIDEV_MAJOR 153	//assigned, don't change it.
#define N_SPI_MINORS 32 

static DECLARE_BITMAP(minors, N_SPI_MINORS);

#define USER_BUFF_SIZE	4096

struct lcd_spi_dev {
    dev_t devt;
    spinlock_t spin_spi_lock; //for muti-processor lock
    struct spi_device *spi; //spi device instance
    struct list_head device_entry; //for file operation
    struct mutex buf_lock;
    char *user_buff;
    unsigned char cmd;
    unsigned users;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#define W_CMD   0
#define W_DATA  1
#define IS_LCD_CMD(state) (state? 0 : 1)

#define start_trans(gpio, value) gpio_direction_output(gpio, value)
#define end_trans(gpio, value) gpio_direction_output(gpio, value)

#define ACTIVE          0
#define DISACTIVE       1

/*      cmd: dc = 0, cs = 0
 *      data: dc = 1, cs = 0
 *      end: cs = 1
 */

static void led_reset(int gpio)
{
    gpio_direction_output(gpio, ACTIVE);
    udelay(10);
    gpio_direction_output(gpio, DISACTIVE);
}

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void write_complete(void *arg)
{
    complete(arg);
}

static ssize_t write_sync(struct lcd_spi_dev *lcd_dev, struct spi_message *message)
{
    DECLARE_COMPLETION_ONSTACK(done);
    int status;
    
    message->complete = write_complete;
    message->context = &done;

    spin_lock_irq(&lcd_dev->spin_spi_lock);
    if (lcd_dev->spi == NULL)
        status = -ESHUTDOWN;
    else
        status = spi_async(lcd_dev->spi, message);
    spin_unlock_irq(&lcd_dev->spin_spi_lock);

    if (status == 0) {
        wait_for_completion(&done);
        status = message->status;
        if (status == 0)
            status = message->actual_length;
    }
    return status;
}

static inline int lcd_write(struct lcd_spi_dev *lcd_dev, size_t len, bool state, u8 cmd)
{
    int err = 0;
    struct lcd12864_plat_data *pdata;
    struct spi_transfer t_send;
    struct spi_message spi_msg;

    /* fill in data to be transfered */
    if (IS_LCD_CMD(state)) {
        lcd_dev->cmd = cmd;
        t_send.tx_buf = &lcd_dev->cmd;
    } else {
        t_send.tx_buf = lcd_dev->user_buff;
    }
    t_send.len = len;
    t_send.bits_per_word = lcd_dev->spi->bits_per_word;
    t_send.speed_hz = lcd_dev->spi->max_speed_hz;
    t_send.rx_buf = NULL;
    t_send.delay_usecs = 5; //add this line to speed up transfer

    pdata = lcd_dev->spi->dev.platform_data;

    start_trans(pdata->lcd_dc, state);
    spi_message_init(&spi_msg);
    spi_message_add_tail(&t_send, &spi_msg);
    err = write_sync(lcd_dev, &spi_msg);
    //end_trans(pdata->lcd_dc, !state);
    return err;
}

#if 0

static void screen_init(struct lcd_spi_dev *lcd_dev)
{
    struct lcd12864_plat_data *pdata;
    pdata = lcd_dev->spi->dev.platform_data;
    led_reset(pdata->lcd_reset);
    //unsigned char *pch = kmalloc(5 *sizeof(unsigned char), GFP_KERNEL);
    lcd_write(lcd_dev, 1, W_CMD, DISPLAY_ON);
    lcd_write(lcd_dev, 1, W_CMD, DISPLAY_CONTRAST);
    lcd_write(lcd_dev, 1, W_CMD, DEF_CONTRAST_LEVEL);
    lcd_write(lcd_dev, 1, W_CMD, SEGMENT_REMAP_C131_SEG0);
    lcd_write(lcd_dev, 1, W_CMD, SCAN_DIRECTION_N_0);
    //kfree(pch);
}
#endif

static int lcd12864_open(struct inode *inode, struct file *filp)
{
    struct lcd_spi_dev *lcd_dev;
    int status = -ENXIO;

    mutex_lock(&device_list_lock); //lock this device

    /* search for device info from system, and fill in */
    list_for_each_entry(lcd_dev, &device_list, device_entry)
    {
        if (lcd_dev->devt == inode->i_rdev) {
            status = 0;
            break;
        }
    }

    if (status == 0) {
        if (lcd_dev->user_buff == NULL) {
            lcd_dev->user_buff = kmalloc(USER_BUFF_SIZE, GFP_KERNEL);
            if (lcd_dev->user_buff == NULL) {
                dev_dbg(&lcd_dev->spi->dev, "open/ENOMEM\n");
                printk(KERN_ALERT ">>>>%s: reaching file:%d, at line:%d\n"
                        , __func__, status, __LINE__);
                status = -ENOMEM;
            }
        }
        if (status == 0) {
            lcd_dev->users++;
            //return driver data to user space, store in file pointer
            //when you try to read/write, the driver info will be passed in along
            //with the file pointer
            filp->private_data = lcd_dev;
            nonseekable_open(inode, filp);
        }
    } else {
        //iminor() retrieve sub-device number from node
        pr_debug("lcd_spi: nothing for minor %d\n", iminor(inode));
    }

    /* do lcd initialization here or what? */
#if 0
    screen_init(lcd_dev);
#endif

    mutex_unlock(&device_list_lock); //release this device

    return status;
}

static int lcd12864_release(struct inode *inode, struct file *filp)
{
    int status = 0;
    struct lcd_spi_dev *lcd_dev;

    mutex_lock(&device_list_lock); //lock this device

    lcd_dev = filp->private_data;
    filp->private_data = NULL;
    lcd_dev->users--;
    if (!lcd_dev->users) {
        int dofree;

        kfree(lcd_dev->user_buff);
        lcd_dev->user_buff = NULL;

        /* ... after we unbound from the underlying device? */
        spin_lock_irq(&lcd_dev->spin_spi_lock);
        dofree = (lcd_dev->spi == NULL);
        spin_unlock_irq(&lcd_dev->spin_spi_lock);

        if (dofree)
            kfree(lcd_dev);
    }

    mutex_unlock(&device_list_lock); //release this device
    return status;
}

static long lcd12864_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int status = 0;
    struct lcd12864_plat_data *pdata;
    struct lcd_spi_dev *lcd_dev;
    struct spi_device *spi;
    u8 tmp = 0;
    int err = 0;

    if (_IOC_TYPE(cmd) != LCD_IOC_MAGIC)//check if it's our command
        return -ENOTTY;

    /* check write access, do it later */
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,
            (void __user *) arg, _IOC_SIZE(cmd));
    if (err)
        return -EFAULT;

    /* get spi data */
    lcd_dev = filp->private_data;
    spin_lock_irq(&lcd_dev->spin_spi_lock);
    spi = spi_dev_get(lcd_dev->spi);
    spin_unlock_irq(&lcd_dev->spin_spi_lock);

    if (spi == NULL) {
        return -ESHUTDOWN;
    }

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
     */
    mutex_lock(&lcd_dev->buf_lock);

    pdata = lcd_dev->spi->dev.platform_data;

    switch (cmd) {
        case IOC_LCD_CMD:
            err = __get_user(tmp, (__u8 __user *) arg);
            dev_dbg(&lcd_dev->spi->dev, "ioctl: sending in:0x%x, err:%d\n"
                    , tmp, err);
            if (err == 0) {
                lcd_write(lcd_dev, 1, W_CMD, tmp);
            }

            break;

        case IOC_LCD_POWER_ON:
            dev_warn(&lcd_dev->spi->dev, "power up!\n");
            led_reset(pdata->lcd_reset);
            lcd_write(lcd_dev, 1, W_CMD, DISPLAY_ON);
            break;

        case IOC_LCD_POWER_OFF:
            dev_warn(&lcd_dev->spi->dev, "power off!\n");
            lcd_write(lcd_dev, 1, W_CMD, DISPLAY_OFF);
            break;
            /*...*/
        default:
            status = -EINVAL;
            break;
    }

    mutex_unlock(&lcd_dev->buf_lock);
    return status;
}

static int lcd12864_write(struct file *filp, const char __user *buf
        , size_t count, loff_t *f_pos)
{
    int status = 0;
    struct lcd_spi_dev *lcd_dev;
    unsigned long missing;

    if (count > USER_BUFF_SIZE)
        return -EMSGSIZE;

    lcd_dev = filp->private_data;

    mutex_lock(&lcd_dev->buf_lock);
    missing = copy_from_user(lcd_dev->user_buff, buf, count);
    if (missing == 0) {
        status = lcd_write(lcd_dev, count, W_DATA, 0);
    } else {
        status = -EFAULT;
    }
    mutex_unlock(&lcd_dev->buf_lock);
    return status;
}

static struct class *lcd_spi_class;

static int __devinit lcd12864_probe(struct spi_device *spi)
{
    struct lcd_spi_dev *lcd_dev;
    struct lcd12864_plat_data *pdata;
    int status;
    unsigned long minor;

    /* allocate driver data */
    lcd_dev = kzalloc(sizeof (struct lcd_spi_dev), GFP_KERNEL);
    if (lcd_dev == NULL) {
        return -ENOMEM;
    }

    /* initialize the driver data, "spi" contains the platform info if we had passed in */
    lcd_dev->spi = spi;
    lcd_dev->cmd = 0;
    lcd_dev->users = 0;
    /* if you do pdata=spi->dev.platform_data, kernel might crash */
    pdata = lcd_dev->spi->dev.platform_data;

    /* initialize resource locker */
    spin_lock_init(&lcd_dev->spin_spi_lock); //in case of a multi-processor system
    mutex_init(&lcd_dev->buf_lock);

    /* initialize device entry(for file operation) */
    INIT_LIST_HEAD(&lcd_dev->device_entry);

    mutex_lock(&device_list_lock); //enter critical section
    minor = find_first_zero_bit(minors, N_SPI_MINORS); //find sub-device number
    if (minor < N_SPI_MINORS) {
        struct device *dev;
        lcd_dev->devt = MKDEV(SPIDEV_MAJOR, minor); //create node under /dev
        //assigned name to the node
        dev = device_create(lcd_spi_class, &spi->dev, lcd_dev->devt
                , lcd_dev, "lcd_spi%d.%d", spi->master->bus_num, spi->chip_select);
        if (IS_ERR(dev)) {
            status = PTR_ERR(dev);
        } else {
            printk(KERN_ALERT "%s registered at spi-bus:%d, cs:%d."
                    , "olcd12864", spi->master->bus_num
                    , spi->chip_select);
            status = 0;
        }
    } else {
        dev_dbg(&spi->dev, "no minor number available!\n");
        status = -ENODEV;
    }
    if (status == 0) {
        set_bit(minor, minors); //inform kernel that this number has been used
        //add your device to device list, so that others can reach it
        list_add(&lcd_dev->device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);

    /* initialize gpios, and save them for further use */
    status = gpio_request_one(pdata->lcd_reset, GPIOF_OUT_INIT_LOW, "lcd_reset");
    if (status) {
        printk(KERN_ALERT "Error requesting lcd reset gpio: %d.\n", pdata->lcd_reset);
        return -1;
    }

    status = gpio_request_one(pdata->lcd_dc, GPIOF_OUT_INIT_LOW, "lcd_d_c");
    if (status) {
        printk(KERN_ALERT "Error requesting lcd data-command gpio: %d.\n", pdata->lcd_dc);
        return -1;
    }


    if (status == 0) {
        spi_set_drvdata(spi, lcd_dev); //add driver to kernels
    } else
        kfree(lcd_dev);

    return status;

}

static struct file_operations lcd_file_instance = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = lcd12864_ioctl,
    .open = lcd12864_open,
    .write = lcd12864_write,
    .release = lcd12864_release,
};

static int __devexit lcd12864_remove(struct spi_device *spi)
{
    struct lcd_spi_dev *lcd_dev = spi_get_drvdata(spi);
    struct lcd12864_plat_data *pdata = lcd_dev->spi->dev.platform_data;
    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&lcd_dev->spin_spi_lock);
    lcd_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&lcd_dev->spin_spi_lock);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    list_del(&lcd_dev->device_entry); //delete device file entry
    device_destroy(lcd_spi_class, lcd_dev->devt); //delete device node from /dev
    clear_bit(MINOR(lcd_dev->devt), minors); //release sub-device number

    gpio_free(pdata->lcd_reset); //release gpio
    gpio_free(pdata->lcd_dc);

    if (lcd_dev->users == 0)
        kfree(lcd_dev);
    mutex_unlock(&device_list_lock);

    return 0;
}

static struct spi_driver lcd_spi_driver = {
    .driver =
    {
        .name = "olcd12864",
        //.bus = spi_bus_type,
        .owner = THIS_MODULE,
    },
    .probe = lcd12864_probe,
    .remove = __devexit_p(lcd12864_remove),
};

static int __init lcd12864_init(void)
{
    int status;

    //register major-device node number
    BUILD_BUG_ON(N_SPI_MINORS > 256); //
    status = register_chrdev(SPIDEV_MAJOR, "spi", &lcd_file_instance);
    if (status < 0) {
        return status;
    }

    //expose our userspace API
    lcd_spi_class = class_create(THIS_MODULE, "olcd");
    if (IS_ERR(lcd_spi_class)) {
        //actually, when unregistering, the name of driver is not used
        unregister_chrdev(SPIDEV_MAJOR, lcd_spi_driver.driver.name);
        return PTR_ERR(lcd_spi_class);
    }

    //register driver
    status = spi_register_driver(&lcd_spi_driver);
    if (status < 0) {
        class_destroy(lcd_spi_class);
        unregister_chrdev(SPIDEV_MAJOR, lcd_spi_driver.driver.name);
    }

    /*  it seems the initialization is finished, actually, system will call
     *  lcd12864_probe() because you had just register its driver
     */

    return status;
}

static void __exit lcd12864_exit(void)
{
    /*  reverse what we had done to initialize our device
     */
    spi_unregister_driver(&lcd_spi_driver); //uninstall driver
    class_destroy(lcd_spi_class); //uninstall API
    unregister_chrdev(SPIDEV_MAJOR, lcd_spi_driver.driver.name); //kill device file
    printk(KERN_ALERT "Good bye, world.\n");
}

module_init(lcd12864_init);
module_exit(lcd12864_exit);

MODULE_AUTHOR("xbrahen@gmail.com");
MODULE_DESCRIPTION("lcd12864 spi-driver module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
