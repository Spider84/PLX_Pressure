/* 
 * File:   main.c
 * Author: User
 *
 * Created on 20 Август 2014 г., 17:06
 */

#define _XTAL_FREQ 32000000 //The speed of your internal(or)external oscillator
#define USE_AND_MASKS

#include <xc.h>
#include <stdint.h>
#include "config.h"
#include "fifo.h"
#include <plib/timers.h>
#include <plib/adc.h>
#include "pressure.h"

fifo_t   plxBufferIn;
fifo_t   plxBufferOut;
uint8_t  plx_in_buffer[128];
uint8_t  plx_out_buffer[128];

uint8_t PLXdata[20];

uint8_t timer0_cnt=0;
uint8_t send_my_pkt = 0;
uint8_t dac_write = 0;

#define ADC_VALUES_CNT 16

typedef struct {
    uint16_t Values[ADC_VALUES_CNT];
    uint32_t Summ;
    uint8_t pos;
    uint16_t value;
} adc_chan_t;

adc_chan_t ChanADC[2];

#undef di
#define di() INTCONbits.GIEH =0;

#undef ei
#define ei() INTCONbits.GIEH = 1;

static void put_adc_value(uint8_t chan,uint16_t value)
{
    uint8_t prev_pos = (ChanADC[chan].pos)&(ADC_VALUES_CNT-1);
    ChanADC[chan].Summ-=ChanADC[chan].Values[prev_pos];
    ChanADC[chan].Values[prev_pos]=value;
    ChanADC[chan].Summ+=value;
    ChanADC[chan].value=ChanADC[chan].Summ/ADC_VALUES_CNT;
    ChanADC[chan].pos=(prev_pos+1)&(ADC_VALUES_CNT-1);
}

void interrupt high_priority InterruptHandlerHigh ()
{
    if (PIE1bits.TXIE==1 && PIR1bits.TXIF==1) {
        uint8_t sendChar;
        if (fifo_read(&plxBufferOut,&sendChar,1)) {
            TXREG=sendChar;            // load txreg with data
        } else
            PIE1bits.TXIE = 0;
    }
    if (PIR1bits.RCIF==1)//is interrupt occured by EUSART receive?,
                        //then RCREG is full we have new data (cleared when RCREG is read)
  {
    if(RCSTA&0x06) //more efficient way than following commented method to check for reception error
    //if(RCSTAbits.FERR==1 || RCSTAbits.OERR==1 )
    {
      RCSTAbits.CREN=0;    //Overrun error (can be cleared by clearing bit CREN)
      //cUART_char=RCREG;    //clear Framing error
      //unsigned char sendChar = RCREG;
      RCSTAbits.CREN=1;
    }
    else
    {
        fifo_put(&plxBufferIn,RCREG); // read new data into variable
    }
  }
}
void interrupt low_priority InterruptHandlerLow ()
{
    if (PIR1bits.ADIF == 1)
    {
        PIR1bits.ADIF = 0;
        uint16_t ADCValue = ReadADC();
        //Reset interrupt flag and start conversion again
        if (!(ADCON0 & 0x3C)) {
            put_adc_value(0,ADCValue);
            SetChanADC(ADC_CH1);
        } else {
            put_adc_value(1,ADCValue);
            SetChanADC(ADC_CH0);
            dac_write = 1;
        }
        ConvertADC();
    }
    if(INTCONbits.TMR0IF == 1)
    {
        INTCONbits.TMR0IF = 0;
        WriteTimer0(0x10); //Please use HEX. Decimal don't work
        if (++timer0_cnt>12) {
            timer0_cnt = 0;
            send_my_pkt = 1;
        }
    }
}

static inline void init_uart(void) // init UART module for 9600bps boud, start bit 1, stopbit 1, parity NONE
{
    TRISCbits.TRISC7=1; //Make UART RX pin input
    TRISCbits.TRISC6=0; //Make UART TX pin output
    SPBRGH  = 0x01;     //19200bps 32MHz Osc
    SPBRG   = 0xA0;

    RCSTAbits.CREN=1;   //1 = Enables receiver
    RCSTAbits.SPEN=1;   //1 = Serial port enabled (configures RX/DT and TX/CK pins as serial port pins)
    BAUDCONbits.BRG16=1;//1 = 16-bit Baud Rate Generator – SPBRGH and SPBRG

    TXSTAbits.SYNC=0;  //0 = Asynchronous mode
    TXSTAbits.BRGH=1;  //1 = High speed
    TXSTAbits.TXEN=1;  //1 = Transmit enabled
    TXSTAbits.TXEN=1;  //1 = Transmit enabled

    RCONbits.IPEN = 1;  //enable Interrupt priority levels
    IPR1bits.RCIP=1;    // EUSART Receive Interrupt Priority 0 = Low priority
    IPR1bits.TXIP=1;    // EUSART Transmit Interrupt Priority 0 = Low priority
    PIE1bits.RCIE=1;    // 1 = Enables the EUSART receive interrupt
}

static inline void init_ports()
{
    TRISA = 0xFF; //All as Input
    TRISB = 0x00;
    TRISC = 0xFF; //Not Connected All, but will be Inputs

    LATB = 0x00;
}

static inline void init_send_timer()
{
    uint8_t Timer0Config = TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_256;
    OpenTimer0(Timer0Config);
    WriteTimer0(0x10); //Please use HEX. Decimal don't work
    INTCON2bits.TMR0IP = 0;
    INTCONbits.TMR0IF = 0; //reset Interrupt Flag

}

inline void do_send_my_pkt()
{
    const uint8_t addr = 10;
    uint16_t data = 0x0000;
    uint8_t addr_h, addr_l, data_h, data_l;
    addr_l = addr & 0x3F;
    addr_h = (addr>>6) & 0x3F;
    di();
    fifo_put(&plxBufferOut,addr_h);
    fifo_put(&plxBufferOut,addr_l);
    fifo_put(&plxBufferOut,PLXdata[addr]++);
    ei();
    PIE1bits.TXIE=1;
    //ADC_INT_DISABLE();
    GIEL = 0;
    data = ADC1Pressure(ChanADC[0].value);
    //data = ((unsigned long)ChanADC[0].value*5115)/10000;
    GIEL = 1;
    //ADC_INT_ENABLE();
    data_l = data & 0x3F;
    data_h = (data>>6) & 0x3F;
    di();
    fifo_put(&plxBufferOut,data_h);
    fifo_put(&plxBufferOut,data_l);
    fifo_put(&plxBufferOut,addr_h);
    fifo_put(&plxBufferOut,addr_l);
    fifo_put(&plxBufferOut,PLXdata[addr]++);
    ei();
    PIE1bits.TXIE=1;
    //ADC_INT_DISABLE();
    GIEL = 0;
    data = ADC2Pressure(ChanADC[1].value);
    GIEL = 1;
    //ADC_INT_ENABLE();
    data_l = data & 0x3F;
    data_h = (data>>6) & 0x3F;
    di();
    fifo_put(&plxBufferOut,data_h);
    fifo_put(&plxBufferOut,data_l);
    ei();
}

inline void init_adc(void)
{
    for (uint8_t i=0; i<2; i++) {
        ChanADC[i].Summ=0;
        ChanADC[i].pos=0;
        ChanADC[i].value=0;
        for (uint8_t j=0; j<ADC_VALUES_CNT; j++) ChanADC[i].Values[j]=0;
    }
    OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD, ADC_CH0 & ADC_INT_ON & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_2ANA);
    ADIP = 0;
}

void eraseBuffers(void)
{
    for (uint8_t i=0;i<=20;i++) {
        PLXdata[i]=0;
    }
    return;
}

void sendItem(uint16_t addr,uint8_t Instance,uint16_t data)
{
    di();
    fifo_put(&plxBufferOut,(addr>>6) & 0x3F);
    fifo_put(&plxBufferOut,addr & 0x3F);
    fifo_put(&plxBufferOut,Instance);
    fifo_put(&plxBufferOut,(data>>6) & 0x3F);
    fifo_put(&plxBufferOut,data & 0x3F);
    ei();
    PIE1bits.TXIE=1;
    return;
}

/*
 * 
 */
int main(void)
{
    fifo_init(&plxBufferIn,plx_in_buffer,sizeof(plx_in_buffer));
    fifo_init(&plxBufferOut,plx_out_buffer,sizeof(plx_out_buffer));
    init_uart(); // init UART module
    init_ports();
    init_adc();

    //Wait for all system Init
    for (volatile uint8_t i=0;i<5;i++)
        __delay_ms(20);

    //Check for Terminator-jumper
    if (PORTAbits.RA4==0) {
        init_send_timer();
    }

    //Enabling Interupts
    ADC_INT_ENABLE();
    ei();

    ConvertADC();

    while (1) // infinite loop which handles ncoming data as they arrive
    {
        uint8_t sendChar;
        uint8_t state;
        uint16_t wTmp,wData;
        uint8_t flags;
        if (fifo_read(&plxBufferIn,&sendChar,1)) {
            if (sendChar==0x80) {
                //Begin of packet
                eraseBuffers();
                state = 1;
                flags = 0;
            } else
            if (sendChar==0x40) {
                if (state==1) {
                    //End of packet
                    do_send_my_pkt();
                    di();
                    fifo_put(&plxBufferOut,0x40);
                    ei();
                    PIE1bits.TXIE=1;
                }
                state = 0;
            } else {
                //Add Data to Buffers
                switch (state++) {
                    case 1: // Addr_hi
                        wTmp=((uint16_t)(sendChar&0x3F))<<6;
                        break;
                    case 2: //Addr_lo
                        wTmp|=(uint16_t)(sendChar&0x3F);
                        break;
                    case 3: //Instance
                        if (sendChar>20) {
                            state = 0;
                            break;
                        }
                        //Instance=sendChar;
                        break;
                    case 4: //Data_hi
                        wData = ((uint16_t)(sendChar&0x3F))<<6;
                        break;
                    case 5: //Data_lo
                        wData |= (uint16_t)(sendChar&0x3F);
                        if ((flags & 1)==0) {
                            flags |= 1;
                            di();
                            fifo_put(&plxBufferOut,0x80);
                            ei();
                        }
                        if (wTmp<20) {
                            sendItem(wTmp,PLXdata[wTmp]++,wData);
                            PIE1bits.TXIE=1;
                        }
                        state = 1;
                        break;
                    default: //Error. Imposible
                        state =0;
                        break;
                }
            }
        } else
        //Send my own packet every 10Hz
        if (send_my_pkt) { //Only When JUMPER set
            di();
            fifo_put(&plxBufferOut,0x80);
            ei();
            do_send_my_pkt();
            di();
            fifo_put(&plxBufferOut,0x40);
            ei();
            PIE1bits.TXIE=1;
            send_my_pkt=0;
            eraseBuffers();
        }

        if (dac_write) {
            dac_write = 0;
            GIEL = 0;
            LATB = ChanADC[0].value/4;
            GIEL = 1;
        }
    }
    return 0;
}

