#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>
#include "adc.h"

class Board_Monitor {
public:
    Board_Monitor(void);
    void init();

    void init_vrefint_reciprocal(void);
    void sampleTemperate(void);
    void sampleBatteryVoltage(void);

    float getTemperate(void) { return temperate; }
    float getBatteryVoltage(void) { return voltage; };

    void handle(void);
private:
    ADC_HandleTypeDef* v_hadc_;
    uint32_t v_ch_;

    ADC_HandleTypeDef* vrefint_hadc_;
    uint32_t vrefint_ch_;

    ADC_HandleTypeDef* temp_hadc_;
    uint32_t temp_ch_;

    float temperate;
    float voltage;
    uint8_t version_;

    uint16_t sample_cnt;

    float vrefint_;
    float v_vrefint_proportion;

    volatile float voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;

    uint16_t adcxGetChxValue(ADC_HandleTypeDef *ADCx, uint32_t ch);

};

#endif // !BSP_ADC_H
