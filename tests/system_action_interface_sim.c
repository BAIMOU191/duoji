/* Exercise the real UART parser and config code for the three ADC APPs. */
#define main firmware_test_main
#include "variant_alignment_sim.c"
#undef main

#ifdef PARAM_SYSTEM_ACTION
#error "The user-facing RAM system-action interface must not be defined"
#endif

uint8_t D_ADC_Encoder_Read(uint16_t *out) { *out=0U;return 0U; }

int main(void)
{
    reset_commands();
    return 0;
}
