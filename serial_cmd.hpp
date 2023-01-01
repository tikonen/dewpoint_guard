#pragma once

#define SER_EOL "\r\n"

#define serial_write_line(msg) Serial.println(msg)
#define serial_write_char(c) Serial.write(c)
#define serial_write(msg) Serial.print(msg)
void serial_clear();
int serial_read(char* buffer, int count);
const char* serial_read_line();
void serial_echo_loop();  // debug

#define serial_printf(format, ...) printf(format, ##__VA_ARGS__)
#define serial_printfln(format, ...) printf(format SER_EOL, ##__VA_ARGS__)
#define serial_print(str) serial_write(str)
#define serial_println(str) serial_write_line(str)
