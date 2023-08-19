/**
 ******************************************************************************
 * @file    bsp_led.cpp/h
 * @brief   Development board LED. 板载LED
 ******************************************************************************
 * Copyright (c) 2023 
 * All rights reserved.
 ******************************************************************************
 */

#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>
#include "tim.h"

class BoardLed {
public:
    BoardLed(void);
    
    void init(void);

    void setColor(uint8_t a, uint8_t r, uint8_t g, uint8_t b);
    void setColor(uint8_t r, uint8_t g, uint8_t b);

    void setModeOn(void) { mode_ = ON; };
    void setModeOff(void) { mode_ = OFF; };
    void setModeBreath(uint32_t period);
    void setModeBlink(uint8_t times, uint32_t dt0, uint32_t dt1);

    void handle(void);
		
		uint32_t loopLimit(uint32_t val, uint32_t min, uint32_t max);
private:

    uint8_t a_, r_, g_, b_;
    
    enum LedMode {
        OFF, 
        ON, 
        BREATH, 
        BLINK,
    } mode_;

    uint32_t breath_period_;

    struct BlinkParam_t
    {
        uint8_t times;
        uint32_t dt[2];
        uint32_t period;
        uint32_t period_start_tick;
    } blink_param_;
    
    TIM_HandleTypeDef* htim_;
    uint32_t r_ch_, g_ch_, b_ch_;

};

#endif // !BSP_LED_H
