#ifndef C88_INSTRUCTIONS_INCLUDE
#define C88_INSTRUCTIONS_INCLUDE
#include <stdint.h>
const uint8_t C88_LOAD = 0b00000000;
const uint8_t C88_SWAP = 0b00001000;
const uint8_t C88_STORE = 0b00010000;
const uint8_t C88_STOP = 0b00011000;
const uint8_t C88_TSG = 0b00100000;
const uint8_t C88_TSL = 0b00101000;
const uint8_t C88_TSE = 0b00110000;
const uint8_t C88_TSI = 0b00111000;
const uint8_t C88_JMP = 0b01000000;
const uint8_t C88_JMA = 0b01001000;
const uint8_t C88_IOW = 0b01100000;
const uint8_t C88_IOR = 0b01101000;
const uint8_t C88_IOS = 0b01110000;
const uint8_t C88_IOC = 0b01111000;
const uint8_t C88_ADD = 0b10000000;
const uint8_t C88_SUB = 0b10001000;
const uint8_t C88_MUL = 0b10010000;
const uint8_t C88_DIV = 0b10011000;
const uint8_t C88_SHL = 0b10100000;
const uint8_t C88_SHR = 0b10101000;
const uint8_t C88_ROL = 0b10110000;
const uint8_t C88_ROR = 0b10111000;
const uint8_t C88_ADDU = 0b11000000;
const uint8_t C88_SUBU = 0b11001000;
const uint8_t C88_MULU = 0b11010000;
const uint8_t C88_DIVU = 0b11011000;
const uint8_t C88_INC = 0b11100000;
const uint8_t C88_DEC = 0b11101000;
const uint8_t C88_DOUBLE = 0b11110000;
const uint8_t C88_HALF = 0b11111000;
#endif //C88_INSTRUCTIONS_INCLUDE
