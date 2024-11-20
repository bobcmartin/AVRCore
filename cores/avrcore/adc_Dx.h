/*
    configuration primative for DA/DB and DD ADC0 blocks

    Bob Martin
    microchip Sept 2024

*/


// ADC 0 config

typedef struct ADC0_config
{
    uint8_t prescaler;
    uint8_t left_adjust;
    uint8_t sample_number;
    uint8_t mode;
    uint8_t mux_pos;
    uint8_t mux_neg;
    uint8_t init_delay;
    uint8_t samp_delay;

} ADC0_config_t;



void init_ADC0(void);




