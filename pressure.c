#include <stdint.h>
#include "pressure.h"

const int16_t VDO_table[13][2]={
{0,0},
{39,109},
{50,125},
{79,160},
{107,189},
{141,220},
{177,249},
{217,279},
{265,311},
{427,404},
{494,438},
{577,476},
{614,493}};
/*{0,0},
{39,108},
{50,124},
{79,159},
{107,187},
{141,218},
{177,247},
{217,276},
{265,308},
{427,400},
{494,434},
{577,472},
{614,489}};*/
/*
{0,0},
{39,71},
{50,82},
{79,105},
{107,124},
{141,244},
{177,163},
{217,182},
{265,203},
{427,264},
{494,282},
{577,312},
{614,322}};
*/

//                                   Y                   Y0                    Y1                    X0                   X1
uint16_t interp(uint16_t ADC, uint16_t ADC0, uint16_t ADC1, uint16_t PSI0, uint16_t PSI1)
{
    //PSI=(ADC-ADC0)*(PSI1-PSI0)/(ADC1-ADC0)+PSI0
    uint32_t t = (PSI1-PSI0)*(ADC-ADC0);
    return (t/(ADC1-ADC0))+PSI0;
}

uint16_t ADC1Pressure(uint16_t ADC)
{
    for (uint8_t i=1;i<12;i++)
        if (ADC<=VDO_table[i][1])
            return interp(ADC,VDO_table[i-1][1],VDO_table[i][1],VDO_table[i-1][0],VDO_table[i][0]);
    return 614;
}

uint16_t ADC2Pressure(uint16_t ADC)
{
    //0 = -71
    //1023 = 3.3 = 403
    //5.0 = 647
    const int32_t min_press = -460;
    const uint32_t max_press = 423;
    uint16_t t = ((uint32_t)(1023-ADC)<<15)/1023;
    int32_t yt = t*min_press;
    uint16_t t1 = ((uint16_t)1<<15)-t;
    int32_t result = (yt+(t1*max_press));
    if (result<0) return 0;
    return result>>15;
}
