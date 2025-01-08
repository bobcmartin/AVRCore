/*
    configuration primative for DA/DB and DD ADC0 blocks

    Bob Martin
    microchip Sept 2024

*/

static const int16_t adc_prescale_to_clkadc[0x0F] PROGMEM = {(F_CPU /   2000L),(F_CPU /   4000L),(F_CPU /  8000L),(F_CPU / 12000L),
                                                              (F_CPU /  16000L),(F_CPU /  20000L),(F_CPU / 24000L),(F_CPU / 28000L),
                                                              (F_CPU /  32000L),(F_CPU /  48000L),(F_CPU / 64000L),(F_CPU / 96000L),
                                                              (F_CPU / 128000L),(F_CPU / 256000L),1};


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
    uint8_t reference;

} ADC0_config_t;

void analogReference(uint8_t mode);
void check_valid_analog_pin(pin_size_t pin);
void check_valid_analog_ref(uint8_t mode);
void init_ADC0(void);




