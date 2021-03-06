#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "A6_inno.h"
#include "A6_inno_gpio.h"

#define IOCTL_SET_VALUE_0 _IOR(MAGIC_NUM, 0, char *)
#define IOCTL_SET_CHAIN_0 _IOR(MAGIC_NUM, 1, char *)

extern int g_hwver;

int SPI_PIN_POWER_EN[] = {
872,
873,
874,
875,
876,
877,
878,
879,
};

int SPI_PIN_START_EN[] = {
854,
856,
858,
860,
862,
864,
890,
892,
};

int SPI_PIN_RESET[] = {
855,
857,
859,
861,
863,
865,
891,
893,
};

int SPI_PIN_LED[] = {
881,
882,
883,
884,
885,
886,
887,
888,
};

int SPI_PIN_PLUG[] = {
896,
897,
898,
899,
900,
901,
902,
903,
};

void set_vid_value_g9(int level)
{
    int fd; 
    
    //printf("%s:%d.\n", __func__, level);

    fd = open(SYSFS_VID_DEV, O_RDWR);
    if(fd < 0)
    {
        fprintf(stderr, "open %s fail.\n", SYSFS_VID_DEV);
        return;
    }

    if(ioctl(fd, IOCTL_SET_VALUE_0, 0x100 | level) < 0)
    {
        fprintf(stderr, "set vid value fail.\n");
        close(fd);
        return;
    }
    close(fd);
    return;
}

void set_vid_value_g19(int level, int chainNum)
{
    int fd; 
        
    printf("%s:%d,chain %d.\n", __func__, level, chainNum);
    
    fd = open(SYSFS_VID_DEV, O_RDWR);
    if(fd < 0)
    {
        fprintf(stderr, "open %s fail.\n", SYSFS_VID_DEV);
        return;
    }
    
    if(ioctl(fd, IOCTL_SET_CHAIN_0, chainNum) < 0)
    {
        fprintf(stderr, "set vid value fail.\n");
        close(fd);
        return;
    }
    
    if(ioctl(fd, IOCTL_SET_VALUE_0, 0x100 | level) < 0)
    {
        fprintf(stderr, "set vid value fail.\n");
        close(fd);
        return;
    }
    close(fd);
    return;
}

void set_vid_value(int level, int cid)
{
    if(HARDWARE_VERSION_G9 == g_hwver){
        set_vid_value_g9(level);
    }else if(HARDWARE_VERSION_G19 == g_hwver){
        set_vid_value_g19(level, cid); 
    }else{
        fprintf(stderr, "Set vid but hardware version is unknown!!!");
    }
    return;
}

void asic_spi_init(void)
{
    int fd;
    char fvalue[64];

    fd = open(SYSFS_SPI_EXPORT, O_WRONLY);
    if(fd == -1)
    {
        return;
    }
    memset(fvalue, 0, sizeof(fvalue));
    sprintf(fvalue, "%s", "fclk1");
    write(fd, fvalue, strlen(fvalue));
    close(fd);
    return;
}

uint32_t set_spi_speed(uint32_t speed)
{
    int fd;                               
    char fvalue[64];                  
    uint32_t rdspeed;                     
                                      
    fd = open(SYSFS_SPI_VAL_STR, O_WRONLY);
    if(fd == -1)                           
    {                                      
            return -1;  
    }                   
    memset(fvalue, 0, sizeof(fvalue));
    sprintf(fvalue, "%d", speed*16);      
    write(fd, fvalue, strlen(fvalue));
    close(fd);                        
                                           
    fd = open(SYSFS_SPI_VAL_STR, O_RDONLY);
    if(fd == -1)                           
    {                                      
            return -1;                     
    }                                 
    memset(fvalue, 0, sizeof(fvalue));
    read(fd, fvalue, 12);             
    rdspeed = atoi(fvalue);           
    close(fd);
    
    return rdspeed;

}

uint32_t get_spi_speed(void)
{
    int fd;                                                                       
    char fvalue[64];                                                              
    uint32_t speed;                                                                   
                                                                                  
    fd = open(SYSFS_SPI_VAL_STR, O_RDONLY);                                       
    if(fd == -1)                      
    {                                 
            return -1;                
    }                                     
    memset(fvalue, 0, sizeof(fvalue));    
    read(fd, fvalue, 12);                 
    speed = atoi(fvalue);                 
    close(fd);
    
    return speed;
}

void loop_blink_led(int pos, int cnt)
{
   #if  0   //add by lzl 20180509
    //while(--cnt)
    while(1)
    {
        usleep(500000);
        asic_gpio_write(pos, 1);
        usleep(500000);
        asic_gpio_write(pos, 0);
    }
    return;
	#else
	while(1)
    {
       usleep(500000);
       mcompat_set_led(pos, 1);
       usleep(500000);
       mcompat_set_led(pos, 0);
    }
	#endif
}

void asic_gpio_init(int gpio, int direction)
{
    int fd;
    char fvalue[64];
    char fpath[64];

    fd = open(SYSFS_GPIO_EXPORT, O_WRONLY);
    if(fd == -1)
    {
        return;
    }
    memset(fvalue, 0, sizeof(fvalue));
    sprintf(fvalue, "%d", gpio);
    write(fd, fvalue, strlen(fvalue));
    close(fd);

    memset(fpath, 0, sizeof(fpath));
    sprintf(fpath, SYSFS_GPIO_DIR_STR, gpio);   
    fd = open(fpath, O_WRONLY);
    if(fd == -1)
    {
        return;
    }
    if(direction == 0)
    {
        write(fd, SYSFS_GPIO_DIR_OUT, sizeof(SYSFS_GPIO_DIR_OUT));
    }
    else
    {
        write(fd, SYSFS_GPIO_DIR_IN, sizeof(SYSFS_GPIO_DIR_IN));
    }   
    close(fd);
    return;
}

void asic_gpio_write(int gpio, int value)
{
    int fd;
    char fpath[64];

#if 0
    memset(fpath, 0, sizeof(fpath));
    sprintf(fpath, SYSFS_GPIO_DIR_STR, gpio);   
    fd = open(fpath, O_WRONLY);
    if(fd == -1)
    {
        return;
    }
    write(fd, SYSFS_GPIO_DIR_OUT, sizeof(SYSFS_GPIO_DIR_OUT));
    close(fd);
#endif  
    memset(fpath, 0, sizeof(fpath));
    sprintf(fpath, SYSFS_GPIO_VAL_STR, gpio);
    fd = open(fpath, O_WRONLY);
    if(fd == -1)
    {
        return;
    }

    if(value == 0)
    {
        write(fd, SYSFS_GPIO_VAL_LOW, sizeof(SYSFS_GPIO_VAL_LOW));
    }
    else
    {
        write(fd, SYSFS_GPIO_VAL_HIGH, sizeof(SYSFS_GPIO_VAL_HIGH));
    }   
    close(fd);
    return;
}

int asic_gpio_read(int gpio)
{
    int fd;
    char fvalue[64];
    char fpath[64];

#if 0
    memset(fpath, 0, sizeof(fpath));
    sprintf(fpath, SYSFS_GPIO_DIR_STR, gpio);
    fd = open(fpath, O_WRONLY);
    if(fd == -1)
    {
        //printf("%s\n", strerror(errno));
        return -1;
    }
    write(fd, SYSFS_GPIO_DIR_IN, sizeof(SYSFS_GPIO_DIR_IN));
    close(fd);
#endif  
    memset(fpath, 0, sizeof(fpath));
    sprintf(fpath, SYSFS_GPIO_VAL_STR, gpio);
    fd = open(fpath, O_RDONLY);
    if(fd == -1)
    {
        return -1;
    }
    memset(fvalue, 0, sizeof(fvalue));
    read(fd, fvalue, 1);
    if(fvalue[0] == '0')
    {
        close(fd);
        return 0;
    }
    else
    {
        close(fd);
        return 1;
    }   
}  

