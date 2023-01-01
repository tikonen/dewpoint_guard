#pragma once

#include <stdint.h>

// Ring buffer size N should be power of 2 so that the compiler can optimize the
// expensive modulo division to a simple bitwise AND
template <class T, int N>
struct RingBuffer {
    T data[N];
    uint8_t ridx;
    uint8_t widx;

    inline T& operator[](unsigned int idx) { return data[idx % N]; }

    inline int available() const
    {
        // Unsigned arithmentic ensures that available bytes is correct even
        // when the widx has rolled back to the beginning and is smaller than ridx.
        // e.g. 0 - 255 => 1, 1 - 254 => 3, 253 - 254 => 255 etc..
        // The result must be type casted for this to work.
        int available = (uint8_t)(widx - ridx);
        return available;
    }

    void clear()
    {
        ridx = 0;
        widx = 0;
    }
};