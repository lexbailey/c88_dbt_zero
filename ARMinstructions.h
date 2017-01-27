#ifndef ARM_INSTRUCTIONS_INCLUDE
#define ARM_INSTRUCTIONS_INCLUDE
#include <stdint.h>

const uint16_t THUMB_COND_EQ = 0b0000;
const uint16_t THUMB_COND_NE = 0b0001;
const uint16_t THUMB_COND_CS = 0b0010;
const uint16_t THUMB_COND_CC = 0b0011;
const uint16_t THUMB_COND_MI = 0b0100;
const uint16_t THUMB_COND_PL = 0b0101;
const uint16_t THUMB_COND_VS = 0b0110;
const uint16_t THUMB_COND_VC = 0b0111;
const uint16_t THUMB_COND_HI = 0b1000;
const uint16_t THUMB_COND_LS = 0b1001;
const uint16_t THUMB_COND_GE = 0b1010;
const uint16_t THUMB_COND_LT = 0b1011;
const uint16_t THUMB_COND_GT = 0b1100;
const uint16_t THUMB_COND_LE = 0b1101;
const uint16_t THUMB_COND_AL = 0b1110;

const uint16_t ARM_R0 = 0;
const uint16_t ARM_R1 = 1;
const uint16_t ARM_R2 = 2;
const uint16_t ARM_R3 = 3;
const uint16_t ARM_R4 = 4;
const uint16_t ARM_R5 = 5;
const uint16_t ARM_R6 = 6;
const uint16_t ARM_R7 = 7;
const uint16_t ARM_R8 = 8;
const uint16_t ARM_R9 = 9;
const uint16_t ARM_R10 = 10;
const uint16_t ARM_R11 = 11;
const uint16_t ARM_R12 = 12;
const uint16_t ARM_R13 = 13;
const uint16_t ARM_R14 = 14;
const uint16_t ARM_R15 = 15;

const uint16_t ARM_SP = 13;
const uint16_t ARM_LR = 14;
const uint16_t ARM_PC = 15;

typedef struct {
  uint16_t opcode;  // Opcode value
  int operands;     // Number of operands
  const uint16_t *masks; // Operand masks
  const int *shifts;     // Operand offsets
} THUMB16_t;

const uint16_t  THUMB_ADD_imm_1_masks[]  = {0b0000000000000111, 0b0000000000111000, 0b0000000111000000};
const int       THUMB_ADD_imm_1_shifts[] = {                 0,               3   ,            6      };
const THUMB16_t THUMB_ADD_imm_1          = {0b0001110000000000, 3, THUMB_ADD_imm_1_masks, THUMB_ADD_imm_1_shifts};

const uint16_t  THUMB_ADD_SUB_imm_2_masks[]  = {0b0000011100000000, 0b0000000011111111};
const int       THUMB_ADD_SUB_imm_2_shifts[] = {         8        ,                  0};
const THUMB16_t THUMB_ADD_imm_2              = {0b0011000000000000, 2, THUMB_ADD_SUB_imm_2_masks, THUMB_ADD_SUB_imm_2_shifts};
const THUMB16_t THUMB_SUB_imm_2              = {0b0011100000000000, 2, THUMB_ADD_SUB_imm_2_masks, THUMB_ADD_SUB_imm_2_shifts};

const uint16_t  THUMB_B_1_masks[]  = {0b0000111100000000, 0b0000000011111111};
const int       THUMB_B_1_shifts[] = {         8        ,                  0};
const THUMB16_t THUMB_B_1          = {0b1101000000000000, 2, THUMB_B_1_masks, THUMB_B_1_shifts};

const uint16_t  THUMB_B_2_masks[]  = {0b0000011111111111};
const int       THUMB_B_2_shifts[] = {                 0};
const THUMB16_t THUMB_B_2          = {0b1110000000000000, 1, THUMB_B_2_masks, THUMB_B_2_shifts};

const uint16_t  THUMB_NOP_masks[]  = {};
const int       THUMB_NOP_shifts[] = {};
const THUMB16_t THUMB_NOP          = {0b1011111100000000, 0, THUMB_NOP_masks, THUMB_NOP_shifts};

const uint16_t  THUMB_SVC_1_masks[] = {0b0000000011111111};
const int       THUMB_SVC_1_shifts[]= {0};
const THUMB16_t THUMB_SVC_1         = {0b1101111100000000, 1, THUMB_SVC_1_masks, THUMB_SVC_1_shifts};

const uint16_t  THUMB_BX_1_masks[] = {0b0000000001111000};
const int       THUMB_BX_1_shifts[]= {              3   };
const THUMB16_t THUMB_BX_1         = {0b0100011100000000, 1, THUMB_BX_1_masks, THUMB_BX_1_shifts};

const uint16_t  THUMB_LDRB_imm_1_masks[] = {0b0000000000000111, 0b00000000000111000, 0b0000011111000000};
const int       THUMB_LDRB_imm_1_shifts[]= {                 0,                3   ,            6      };
const THUMB16_t THUMB_LDRB_imm_1         = {0b0111100000000000, 3, THUMB_LDRB_imm_1_masks, THUMB_LDRB_imm_1_shifts};

const uint16_t  THUMB_ADD_SUB_1_masks[] = {0b0000000000000111, 0b00000000000111000, 0b0000000111000000};
const int       THUMB_ADD_SUB_1_shifts[]= {                 0,                3   ,            6      };
const THUMB16_t THUMB_ADD_1              = {0b0001100000000000, 3, THUMB_ADD_SUB_1_masks, THUMB_ADD_SUB_1_shifts};
const THUMB16_t THUMB_SUB_1              = {0b0001101000000000, 3, THUMB_ADD_SUB_1_masks, THUMB_ADD_SUB_1_shifts};

const uint16_t  THUMB_CMP_1_masks[] = {0b0000000000000111, 0b00000000000111000};
const int       THUMB_CMP_1_shifts[]= {                 0,                3   };
const THUMB16_t THUMB_CMP_1         = {0b0100001010000000, 2, THUMB_CMP_1_masks, THUMB_CMP_1_shifts};

const uint16_t  THUMB_LSL_LSR_imm_1_masks[] = {0b0000000000000111, 0b00000000000111000, 0b0000011111000000};
const int       THUMB_LSL_LSR_imm_1_shifts[]= {                 0,                3   ,            6      };
const THUMB16_t THUMB_LSL_imm_1             = {0b0000000000000000, 3, THUMB_LSL_LSR_imm_1_masks, THUMB_LSL_LSR_imm_1_shifts};
const THUMB16_t THUMB_LSR_imm_1             = {0b0000100000000000, 3, THUMB_LSL_LSR_imm_1_masks, THUMB_LSL_LSR_imm_1_shifts};

const uint16_t  THUMB_MOV_imm_1_masks[]  = {0b0000011100000000, 0b0000000011111111};
const int       THUMB_MOV_imm_1_shifts[] = {         8        ,                  0};
const THUMB16_t THUMB_MOV_imm_1          = {0b0010000000000000, 2, THUMB_MOV_imm_1_masks, THUMB_MOV_imm_1_shifts};

const uint16_t  THUMB_AND_ORR_1_masks[] = {0b0000000000000111, 0b00000000000111000};
const int       THUMB_AND_ORR_1_shifts[]= {                 0,                3   };
const THUMB16_t THUMB_AND_1             = {0b0100001010000000, 2, THUMB_AND_ORR_1_masks, THUMB_AND_ORR_1_shifts};
const THUMB16_t THUMB_ORR_1             = {0b0100001100000000, 2, THUMB_AND_ORR_1_masks, THUMB_AND_ORR_1_shifts};

#endif //ARM_INSTRUCTIONS_INCLUDE
