
/*
VESC firmware designed for G2 aircraft of Vayu.co
Firmware has been developed based on VESC project with hardware configuration hw_410.c, hw_410.h, drv8301.c, drv8301.h

Developer: sheng-yu.lo@vayu.us
Date: July, 15, 2019
*/

#ifndef HWCONF_DRV8353_H_
#define HWCONF_DRV8353_H_

#include "datatypes.h"

// Functions
void drv8353_init(void);
void drv8353_set_oc_adj(int val);
void drv8353_set_oc_mode(drv8301_oc_mode mode); // use the same enum from drv8301, check datatypes.h
int drv8353_read_faults(void);
void drv8353_reset_faults(void);
char* drv8353_faults_to_string(int faults);
unsigned int drv8353_read_reg(int reg);
void drv8353_write_reg(int reg, int data);

// Defines
#define DRV8353_FAULT_FETLC_OC		(1 << 0)
#define DRV8353_FAULT_FETHC_OC		(1 << 1)
#define DRV8353_FAULT_FETLB_OC		(1 << 2)
#define DRV8353_FAULT_FETHB_OC		(1 << 3)
#define DRV8353_FAULT_FETLA_OC		(1 << 4)
#define DRV8353_FAULT_FETHA_OC		(1 << 5)
#define DRV8353_FAULT_OTW			(1 << 6)
#define DRV8353_FAULT_OTSD			(1 << 7)
#define DRV8353_FAULT_PVDD_UV		(1 << 8)
#define DRV8353_FAULT_GVDD_UV		(1 << 9)
#define DRV8353_FAULT_FAULT			(1 << 10)
#define DRV8353_FAULT_GVDD_OV		(1 << 11)

#endif /* HWCONF_DRV8353_H_ */
