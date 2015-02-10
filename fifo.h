/*
 * fifo.h
 *
 * A simple FIFO implementation.
 *
 * Copyright (C) 2014
 */

#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>

typedef struct {
     uint8_t * buf;
     uint8_t head;
     uint8_t tail;
     uint8_t size;
} fifo_t;

void fifo_init(fifo_t * f, uint8_t * buf, uint8_t size);
uint8_t fifo_read(fifo_t * f, void * buf, uint8_t nbytes);
uint8_t fifo_put(fifo_t * f, const uint8_t byte);
uint8_t fifo_write(fifo_t * f, const void * buf, uint8_t nbytes);

#endif	/* FIFO_H */
