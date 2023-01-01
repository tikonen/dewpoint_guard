#include <Arduino.h>

#include "serial_cmd.hpp"
#include "ringbuffer.hpp"

// Ring buffer length should be power of 2 so that the compiler can optimize the
// expensive modulo division to a simple bitwise AND
#define RING_BUFFER_LEN 32
#define LINE_BUFFER_LEN 16

static RingBuffer<char, RING_BUFFER_LEN> recvRingBuffer;

static char linebuffer[LINE_BUFFER_LEN];

void serial_clear()
{
    recvRingBuffer.clear();
    while (Serial.available()) Serial.read();
}

// parse lines separated by carriage return and a newline (\r\n)
static char* parse_line(struct RingBuffer<char, RING_BUFFER_LEN>& rb)
{
    const int available = rb.available();

    for (int i = 0; i < available - 1; i++) {
        if (rb[i + rb.ridx] == '\r' && rb[i + rb.ridx + 1] == '\n') {
            // found end of line sequence (\r\n), copy line data (if any)
            int idx = 0;
            while (i-- > 0) {
                char c = rb[rb.ridx++];

                // check for target buffer overflow and copy character
                if (idx < LINE_BUFFER_LEN - 1) {
                    linebuffer[idx++] = c;
                }
            }
            // terminate string and clear trailing whitespace
            do {
                linebuffer[idx--] = '\0';
            } while (idx >= 0 && linebuffer[idx] == ' ');

            // skip end of line sequence
            rb.ridx += 2;

            return linebuffer;
        }
    }

    return NULL;
}

const char* serial_read_line()
{
    // read data from serial to local ringbuffer.
    // *CAUTION* There is a change that too long lines will wrap the ringbuffer and
    // overwrite yet unread data.
    while (Serial.available()) {
        byte c = Serial.read();
        recvRingBuffer[recvRingBuffer.widx++] = c;
    }

    return parse_line(recvRingBuffer);
}

int serial_read(char* buffer, int count)
{
    int n = 0;
    while (recvRingBuffer.available() > 0 && count-- > 0) {
        *buffer++ = recvRingBuffer[recvRingBuffer.ridx++];
        n++;
    }
    while (count-- > 0 && Serial.available()) {
        *buffer++ = Serial.read();
        n++;
    }
    return n;
}

void serial_echo_loop()
{
    const char* cmd = serial_read_line();
    if (cmd) {
        serial_write_line(cmd);
    }
}
