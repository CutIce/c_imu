#include "base/bsp/bsp_adc.h"

ADC_HandleTypeDef* v_hadc = &hadc3;
uint32_t v_ch = ADC_CHANNEL_8;

ADC_HandleTypeDef* vrefint_hadc = &hadc1;
uint32_t vrefint_ch = ADC_CHANNEL_VREFINT;

ADC_HandleTypeDef* temp_hadc = &hadc1;
uint32_t temp_ch = ADC_CHANNEL_TEMPSENSOR;

Board_Monitor::Board_Monitor() 
    : v_hadc_(v_hadc), v_ch_(v_ch), vrefint_hadc_(vrefint_hadc), vrefint_ch_(vrefint_ch), temp_hadc_(temp_hadc), temp_ch_(temp_ch) {
    
    sample_cnt = 500;
    vrefint_ = 1.2f;
}

void Board_Monitor::init_vrefint_reciprocal(void) {
    uint32_t total_vrefinit = 0;
    uint16_t i = 0;
    for (i = 0; i < sample_cnt; ++i) {
        total_vrefinit += adcxGetChxValue(vrefint_hadc_, vrefint_ch_);
    }

    v_vrefint_proportion = sample_cnt * 1.2f / total_vrefinit;
}

uint16_t Board_Monitor::adcxGetChxValue(ADC_HandleTypeDef *ADCx, uint32_t ch) {
    static ADC_ChannelConfTypeDef config = {0};
    config.Channel = ch;
    config.Rank = 1;
    config.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &config) != HAL_OK) {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 20);
    return (int16_t)HAL_ADC_GetValue(ADCx);
}

void Board_Monitor::sampleBatteryVoltage(void) {
    uint16_t val = adcxGetChxValue(v_hadc_, v_ch_);
    voltage = val * v_vrefint_proportion * 10.090909090909090909090909090909f;
}

void Board_Monitor::sampleTemperate(void) {
    uint16_t val = adcxGetChxValue(temp_hadc_, temp_ch_);
    temperate = (float)val * v_vrefint_proportion;

    temperate = (temperate - 0.76f) * 400.f + 25.0f;
}

void Board_Monitor::handle(void) {

    sampleBatteryVoltage();
    sampleTemperate();
}