#include <iostm8l152c8.h>
#include <stm8l.h>
#include <stdint.h>

#include "roll.h"

static uint16_t scramble(uint16_t value) {
    value ^= (uint16_t)(value << 7);
    value ^= (uint16_t)(value >> 9);
    value ^= (uint16_t)(value << 8);
    return value;
}

static uint16_t adc_noise_sample(uint8_t channel) {
    uint8_t sqr2_backup = ADC1_SQR2;
    uint8_t sqr3_backup = ADC1_SQR3;
    uint8_t sqr4_backup = ADC1_SQR4;

    ADC1_SQR2 = 0;
    ADC1_SQR3 = 0;
    ADC1_SQR4 = 0;

    switch (channel) {
    case 0:
        ADC1_SQR4_bit.CHSEL_S0 = 1;
        break;
    case 1:
        ADC1_SQR4_bit.CHSEL_S1 = 1;
        break;
    case 9:
        ADC1_SQR3_bit.CHSEL_S9 = 1;
        break;
    case 10:
        ADC1_SQR3_bit.CHSEL_S10 = 1;
        break;
    case 15:
        ADC1_SQR3_bit.CHSEL_S15 = 1;
        break;
    case 16:
        ADC1_SQR2_bit.CHSEL_S16 = 1;
        break;
    default:
        break;
    }

    ADC1_CR1_bit.START = 1;
    while (!(ADC1_SR & MASK_ADC1_SR_EOC)) {
    }

    uint16_t sample = ((uint16_t)ADC1_DRH << 8) | ADC1_DRL;

    ADC1_SQR2 = sqr2_backup;
    ADC1_SQR3 = sqr3_backup;
    ADC1_SQR4 = sqr4_backup;

    return sample;
}

uint16_t roll(void) {
    static const uint8_t channels[] = {0, 1, 9, 10, 15, 16};
    uint16_t entropy = 0x1D3Du; // seed with non-zero constant

    for (uint8_t i = 0; i < sizeof(channels); ++i) {
        uint16_t sample = adc_noise_sample(channels[i]);
        entropy ^= (uint16_t)(sample << (i & 0x01 ? 5 : 0));
        entropy = scramble(entropy);
    }

    return scramble(entropy ^ adc_noise_sample(1));
}
