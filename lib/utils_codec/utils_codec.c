/**
 * @file codif.c
 * @brief Implementación codec 7-bit (MSB3+LSB4) por duty.
 */
#include "utils_codec.h"

/** Limita a [0,1]. */
static inline float clamp01(float x){ return x<0.f?0.f:(x>1.f?1.f:x); }

/** Centro del bin i en [min,max] con N bins. */
static inline float duty_center(float min,float max,uint8_t i,uint8_t N){
    return clamp01(min + (((float)i+0.5f)/(float)N)*(max-min));
}

/** Índice de bin más cercano en [min,max] con N bins. */
static inline uint8_t duty_to_bin(float duty,float min,float max,uint8_t N){
    if(max<=min) return 0;
    float t=(duty-min)/(max-min); if(t<0)t=0; if(t>1)t=1;
    int idx=(int)(t*(float)N); if(idx>=N) idx=N-1; if(idx<0) idx=0;
    return (uint8_t)idx;
}

float codif_msb3_to_duty(uint8_t msb3){
    if(msb3>7) msb3&=0x07;
    return duty_center(CODIF_DUTY_MIN_MSB3,CODIF_DUTY_MAX_MSB3,msb3,CODIF_BINS_MSB3);
}
float codif_lsb4_to_duty(uint8_t lsb4){
    if(lsb4>15) lsb4&=0x0F;
    return duty_center(CODIF_DUTY_MIN_LSB4,CODIF_DUTY_MAX_LSB4,lsb4,CODIF_BINS_LSB4);
}
uint8_t codif_duty_to_msb3(float duty){
    return duty_to_bin(duty,CODIF_DUTY_MIN_MSB3,CODIF_DUTY_MAX_MSB3,CODIF_BINS_MSB3)&0x07;
}
uint8_t codif_duty_to_lsb4(float duty){
    return duty_to_bin(duty,CODIF_DUTY_MIN_LSB4,CODIF_DUTY_MAX_LSB4,CODIF_BINS_LSB4)&0x0F;
}

void codif_encode_byte7(uint8_t byte7,float* duty_msb,float* duty_lsb){
    uint8_t b7=byte7&0x7F, msb3=(b7>>4)&0x07, lsb4=b7&0x0F;
    if(duty_msb) *duty_msb = codif_msb3_to_duty(msb3);
    if(duty_lsb) *duty_lsb = codif_lsb4_to_duty(lsb4);
}
uint8_t codif_decode_byte7(float duty_msb,float duty_lsb){
    uint8_t msb3=codif_duty_to_msb3(duty_msb)&0x07;
    uint8_t lsb4=codif_duty_to_lsb4(duty_lsb)&0x0F;
    return (uint8_t)((msb3<<4)|lsb4);
}

uint8_t codif_checksum7(const uint8_t* data,size_t n){
    uint8_t c=0; if(!data) return 0;
    for(size_t i=0;i<n;++i) c^=(uint8_t)(data[i]&0x7F);
    return (uint8_t)(c&0x7F);
}
