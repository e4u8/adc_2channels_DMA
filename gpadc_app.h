/*
 * gpadc_app.h
 *
 *  Created on: Feb 6, 2026
 *      Author: User
 *      Application logic of GPADC
 */

#ifndef GPADC_APP_H_
#define GPADC_APP_H_

#include <stdint.h>

/* Task entry to be used by OS task create */
void gpadc_app_task(void *pvParameters);

/*  The hardware configuration happens at main.c inside prvSetupHardware()
 *  because I chose centralized hardware init.
 *  So, this function will be empty!
 * */
void gpadc_app_init(void);


#endif /* GPADC_APP_H_ */
