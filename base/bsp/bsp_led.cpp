#include "base/bsp/bsp_led.h"

TIM_HandleTypeDef* led_htim = &htim5;
const uint32_t b_ch = TIM_CHANNEL_1;
const uint32_t g_ch = TIM_CHANNEL_2;
const uint32_t r_ch = TIM_CHANNEL_3;

    uint16_t r_pwm, g_pwm, b_pwm;


BoardLed::BoardLed(void)
    : htim_(led_htim), r_ch_(r_ch), g_ch_(g_ch), b_ch_(b_ch) {}

void BoardLed::init() {
    HAL_TIM_PWM_Start(htim_, r_ch_);
    HAL_TIM_PWM_Start(htim_, g_ch_);
    HAL_TIM_PWM_Start(htim_, b_ch_);
}

void BoardLed::setColor(uint8_t a, uint8_t r, uint8_t g, uint8_t b) {
    a_ = a;
    r_ = r;
    g_ = g;
    b_ = b;
}

void BoardLed::setColor(uint8_t r, uint8_t g, uint8_t b) {
    r_ = r;
    g_ = g;
    b_ = b;
}

void BoardLed::setModeBreath(uint32_t period) {
    mode_ = BREATH;
    breath_period_ = period;
}

void BoardLed::setModeBlink(uint8_t times = 1, uint32_t dt0 = 100, uint32_t dt1 = 1000) {
    mode_ = BLINK;
    blink_param_.times = times;
    blink_param_.dt[0] = dt0;
    blink_param_.dt[1] = dt1;
    blink_param_.period = 2 * times * dt0 + dt1;
}

void BoardLed::handle(void) {
    if (mode_ == ON) {
        a_ = 255;
    } else if (mode_ == BREATH) {
        float t_ = loopLimit(HAL_GetTick(), 0, breath_period_); 
        if (t_ < breath_period_ / 2) {
            a_ = (uint8_t)(t_ / breath_period_ * 2 * 255);
        } else {
            a_ = (uint8_t)(255 - t_ / breath_period_ * 2 * 255);
        }
    } else if (mode_ == BLINK) {
        if (HAL_GetTick() - blink_param_.period_start_tick > blink_param_.period) {
            blink_param_.period_start_tick = HAL_GetTick();
        } else if (HAL_GetTick() - blink_param_.period_start_tick >
            2 * blink_param_.times * blink_param_.dt[0]) {
            a_ = 0;
        } else if ((HAL_GetTick() - blink_param_.period_start_tick) %
                   (2 * blink_param_.dt[0]) > blink_param_.dt[0]) {
            a_ = 255;
        } else {
            a_ = 0;
        }
    } else {
        a_ = 0;
    }

    r_pwm = r_ * a_;
    g_pwm = g_ * a_;
    b_pwm = b_ * a_;

		__HAL_TIM_SetCompare(htim_, r_ch_, r_pwm);
		__HAL_TIM_SetCompare(htim_, g_ch_, g_pwm);
		__HAL_TIM_SetCompare(htim_, b_ch_, b_pwm);
}

uint32_t BoardLed::loopLimit(uint32_t val, uint32_t min, uint32_t max) {
  if (min >= max)
    return val;
  if (val > max) {
    while (val > max)
      val -= (max - min);
  } else if (val < min) {
    while (val < min)
      val += (max - min);
  }
  return val;
}
