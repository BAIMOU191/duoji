/* Current fixture needs the new debug ADC boundary. Production code is untouched. */
#include "../variant_alignment_sim.c"
uint8_t D_ADC_Encoder_Read(uint16_t *out){*out=0;return 1;}
