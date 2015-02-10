/* 
 * File:   pressure.h
 * Author: User
 *
 * Created on 27 Август 2014 г., 21:15
 */

#ifndef PRESSURE_H
#define	PRESSURE_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

uint16_t ADC1Pressure(uint16_t ADC);
uint16_t ADC2Pressure(uint16_t ADC);

#ifdef	__cplusplus
}
#endif

#endif	/* PRESSURE_H */

