#ifndef __PANEL_H__
#define __PANEL_H__
#include "../de/bsp_display.h"
#include "lcd_source.h"

extern void LCD_OPEN_FUNC(u32 sel, LCD_FUNC func, u32 delay/*ms*/);
extern void LCD_CLOSE_FUNC(u32 sel, LCD_FUNC func, u32 delay/*ms*/);

typedef struct{
	char name[32];
	disp_lcd_panel_fun func;
}__lcd_panel_t;

extern __lcd_panel_t * panel_array[];

struct sunxi_lcd_drv
{
  struct sunxi_disp_source_ops      src_ops;
};

extern int sunxi_disp_get_source_ops(struct sunxi_disp_source_ops *src_ops);
int lcd_init(void);

#endif
