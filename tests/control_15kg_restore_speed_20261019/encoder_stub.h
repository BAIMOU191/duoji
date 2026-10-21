/* Hardware stub for the raw-ADC diagnostics added since the old protocol fixture. */
uint8_t D_ADC_Encoder_Read(uint16_t *out)
{
    if (encoder_bad) return 0;
    double raw=(ENCODER_POT_ADC_MIN+plant_p*ENCODER_POT_ADC_SPAN/ENCODER_POT_SPAN_CDEG)*16;
    if(raw<0)raw=0;
    if(raw>65520)raw=65520;
    *out=(uint16_t)lround(raw);
    return 1;
}
