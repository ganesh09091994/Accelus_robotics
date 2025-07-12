/*
 * RingBuf.h
 *
 *  Created on: Apr 9, 2010
 *      Author: Amit Goradia
 */

#ifndef RINGBUF_H_
#define RINGBUF_H_

#include <stdint.h>
#include <stdbool.h>

struct ring_buffer_t {
    uint8_t *data; /* block of memory or array of data */
    unsigned element_size;      /* how many bytes for each chunk */
    unsigned element_count;     /* number of chunks of data */
    unsigned head; /* where the writes go */
    unsigned tail; /* where the reads come from */
};
typedef struct ring_buffer_t RING_BUFFER;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

    bool Ringbuf_Empty(
        RING_BUFFER const *b);
    uint8_t *Ringbuf_Get_Front(
        RING_BUFFER const *b);
    uint8_t *Ringbuf_Pop_Front(
        RING_BUFFER * b);
    bool Ringbuf_Put(
        RING_BUFFER * b,        /* ring buffer structure */
        uint8_t *data_element);    /* one element to add to the ring */
    /* Note: element_count should be a power of two,
       or it will only use the next lower power of two */
    void Ringbuf_Init(
        RING_BUFFER * b,        /* ring buffer structure */
        uint8_t *data,     /* data block or array of data */
        unsigned element_size,  /* size of one element in the data block */
        unsigned element_count);        /* number of elements in the data block */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RINGBUF_H_ */
