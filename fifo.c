#include <stdint.h>
#include "fifo.h"

//This initializes the FIFO structure with the given buffer and size
void fifo_init(fifo_t * f, uint8_t * buf, uint8_t size){
     f->head = 0;
     f->tail = 0;
     f->size = size;
     f->buf = buf;
}

//This reads nbytes bytes from the FIFO
//The number of bytes read is returned
uint8_t fifo_read(fifo_t * f, void * buf, uint8_t nbytes){
     uint8_t i;
     uint8_t * p;
     p = buf;
     for(i=0; i < nbytes; i++){
          if( f->tail != f->head ){ //see if any data is available
               *p++ = f->buf[f->tail];  //grab a byte from the buffer
               f->tail++;  //increment the tail
               if( f->tail == f->size ){  //check for wrap-around
                    f->tail = 0;
               }
          } else {
               return i; //number of bytes read
          }
     }
     return nbytes;
}

uint8_t fifo_put(fifo_t * f, const uint8_t byte)
{
    if((f->head + 1 == f->tail) ||
        ((f->head + 1 == f->size) && (f->tail == 0))){
        return 1; //no more room
    } else {
        f->buf[f->head] = byte;
        f->head++;  //increment the head
        if( f->head == f->size ){  //check for wrap-around
            f->head = 0;
        }
        return 0;
    }
}

//This writes up to nbytes bytes to the FIFO
//If the head runs in to the tail, not all bytes are written
//The number of bytes written is returned
uint8_t fifo_write(fifo_t * f, const void * buf, uint8_t nbytes)
{
     const uint8_t * p = buf;
     for(uint8_t i=0; i < nbytes; i++)
        //first check to see if there is space in the buffer
        if (fifo_put(f,*p++)) return i;
     return nbytes;
}
