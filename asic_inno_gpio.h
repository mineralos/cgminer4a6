#ifndef _ASIC_INNO_GPIO_
#define _ASIC_INNO_GPIO_

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>


#define SYSFS_GPIO_EXPORT	"/sys/class/gpio/export"
#define SYSFS_GPIO_DIR_STR	"/sys/class/gpio/gpio%d/direction"
#define SYSFS_GPIO_VAL_STR	"/sys/class/gpio/gpio%d/value"

#define SYSFS_GPIO_DIR_OUT	"out"
#define SYSFS_GPIO_DIR_IN	"in"

#define SYSFS_GPIO_VAL_LOW	"0"
#define SYSFS_GPIO_VAL_HIGH	"1"

extern int SPI_PIN_START_EN[ASIC_CHAIN_NUM];
extern int SPI_PIN_RESET[ASIC_CHAIN_NUM];



void asic_gpio_init(int gpio, int direction);

void asic_gpio_write(int gpio, int value);

int asic_gpio_read(int gpio);



#endif