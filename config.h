/**
 ******************************************************************************
 * @file    hardware_config.cpp/h
 * @brief   Hardware configuration. 
 ******************************************************************************
 * Copyright (c) 
 * All rights reserved.
 ******************************************************************************
 */

#ifndef CONFIG_H
#define CONFIG_H


#define stm32f407

#define BMI088_SPI &hspi1;
#define IST8310_I2C &hi2c3;
#define IMU_TIM &htim10
#define IMU_TIM_CHANNEL TIM_CHANNEL_1

#endif