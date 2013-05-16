#ifndef __LINUX_I2C_MAX7359_H
#define __LINUX_I2C_MAX7359_H

/* linux/i2c/max7359.h */

#include <linux/types.h>
#include <linux/input/matrix_keypad.h>

struct max7359_platform_data {
	const struct matrix_keymap_data *keymap_data;
  u8 debounce_reg_val;
	int	(*init_platform_hw)(void);
	void	(*exit_platform_hw)(void);
};

#endif
