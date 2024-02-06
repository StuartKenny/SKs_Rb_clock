/**
 ******************************************************************************
 * @file           : laserlock.c
 * @brief          : Code for locking laser frequency to dip
 * @author		   : Stuart Kenny
 ******************************************************************************
   * @attention
  *
  * Copyright (c) 2024 Stuart Kenny.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "laserlock.h"
//#include "main.h" //needed for port and timer definitions
//#include <stdio.h>
//#include <stdint.h>
//#include <stdbool.h>
//#include <string.h>
//#include <math.h>

/* Typedefs -----------------------------------------------------------*/

/* Defines ------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
static const uint16_t LASER_STEP = 3; //circa 60uA @ 20.1uA/step
static uint16_t LASER_MOD_VALUE = 3; //can vary between 0 and 4095

/* Function prototypes -----------------------------------------------*/
//__attribute__((section(".itcm"))) static uint32_t template_function(const uint32_t data, const bool verify);
//extern void Error_Handler(void);

/**
  * @brief  Function x.
  * @retval int
  */

