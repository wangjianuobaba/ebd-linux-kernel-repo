/* 
 * File:   olcd12864.h
 * Author: root
 *
 * Created on May 28, 2013, 11:14 AM
 */

#ifndef OLCD12864_H
#define	OLCD12864_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <linux/types.h>

    //#ifdef __KERNEL__

    struct lcd12864_plat_data {
        int lcd_dc;
        int lcd_reset;
    };
    //#endif

#define LCD_IOC_MAGIC   'x'     //seed for io command generation


    //io commands

    enum {
        IOC_DISPLAY_CLEAR = _IO(LCD_IOC_MAGIC, 1),
        IOC_DISPLAY_REVERSE = _IO(LCD_IOC_MAGIC, 2),
        IOC_CONTRAST_LEVEL = _IOW(LCD_IOC_MAGIC, 3, u8),
        IOC_BRIGHTNESS_LEVEL = _IOW(LCD_IOC_MAGIC, 4, u8),
        IOC_DISPLAY_ON = _IO(LCD_IOC_MAGIC, 5),
        IOC_DISPLAY_OFF = _IO(LCD_IOC_MAGIC, 6),
        IOC_MEM_ADDR_MODE = _IOW(LCD_IOC_MAGIC, 7, u8),
        IOC_DISPLAY_POSITION = _IOW(LCD_IOC_MAGIC, 8, u16),
    };

    //lcd command

    enum {
        DISPLAY_CONTRAST = 0x81,
        SEGMENT_REMAP_C131_SEG0 = 0xA1,
        DISPLAY_OFF = 0xAE,
        DISPLAY_ON,
        SCAN_DIRECTION_N_0 = 0xC8,


    };

#define DEF_CONTRAST_LEVEL 0x80
    
#ifdef	__cplusplus
}
#endif

#endif	/* OLCD12864_H */

