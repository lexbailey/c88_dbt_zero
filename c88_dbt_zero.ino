
/*
 * C88 DBT program. For running C88 programs on an ARM Cortex M device. 
 * Should support all M devices, only tested on M0+.
 * 
 * Basic operating principle:
 *  Whenever a program is entered or modified (including user modification and 
 *  self-modifications by the program), the entire program is translated and
 *  then executed.
 *  
 *  When the program is executed (by branching to it with BLX) the DBT will
 *  ensure that R0 of the host CPU contains the current main register value
 *  of the imaginary C88 in the lowest eight bits.
 *  
 *  If the translated program is to run at a speed other than full speed, there 
 *  will be an SVC instruction after each of the instructions in the translated
 *  program. This calls the supervisor, which will usually update the display and
 *  then set the return address to be the DBT's main loop and then return from
 *  supervisor handler. The DBT now does the appropriate amount of waiting before
 *  jumping back to the translated program to continue execution.
 *  
 *  The DBT, when calling into the translated program, will also set some other
 *  special values before it does. R2 will contain the return address for the DBT
 *  main loop, so that the supervisor know what address to use to force a return
 *  to the DBT main loop.
 *  
 *  The DBT will also set R1 to zero and expect it to be non-zero upon return if
 *  the program is to stop as a result of a C88 STOP instruction.
 *  
 *  R1 is also used by the SELFMOD_MARK supervisor call. The caller should reset
 *  R1 after the call.
 *  
 *  There is also a need for the C88 program to access the untranslated version of
 *  itself. The base pointer to the original C88 program is therefore placed in R3.
 *  This allows use of LDRB Rt, [R3, #n] to get the nth byte of the C88 program
 *  
 *  Notes:
 *    As the ARM registers are 32 bits wide, calculations that cause overflows when
 *    done in the C88's 8 bit register, will not cause overflows in R0. This is not
 *    usually a problem as the C88 does not take any special action when an overflow
 *    occurs. This can be a problem for some instructions however. It is therefore the 
 *    responsibility of each instruction to sanitize R0 if necassary. It is not the
 *    responsibility of the instruction that produced the overflow result to sanitize
 *    R0.
 *    
 *    This is typically done with this code:
 *    MOV  R4, 0xFF
 *    AND  R0, R4
 *  
 */

#include <stdarg.h>
#include <stdint.h>
#include "c88instructions.h"
#include "ARMinstructions.h"

#ifndef _VARIANT_ARDUINO_ZERO_
#error "This program will only run on an arduino (or genuino) zero. It could be made to also work on a DUE with some extra effort."
#endif

#define HFSR_ADDR     ((uint32_t *)0xE000ED2C)
#define HFSR          (*HFSR_ADDR)
#define HFSR_DEBUGEVT (1<<31)
#define HFSR_FORCED   (1<<30)
#define HFSR_VECTTBL  (1<<1)

#define CFSR_ADDR     ((uint32_t *)0xE000ED28)
#define CFSR          (*CFSR_ADDR)

#define MAX_TRANSLATED_LENGTH (255)

#define SVC_NUMBER(x) (x)
#define SVC_STRING_SUB(x) #x
#define SVC_STRING(x) SVC_STRING_SUB(x)

#define SVC_EXIT_PROGRAM    0 // Handy for testing, just so I don't have to insert a branch to the exit function
#define SVC_SELF_TEST       1 // Used to test that the SVC handler is working before the DBT starts
#define SVC_REGULATE_SPEED  2 // Causes a delay according to the current speed setting
#define SVC_SET_STOP        3 // Sets the stop code
#define SVC_IO_READ         4 // Reads the IO in into the register
#define SVC_IO_WRITE        5 // Writes the register to the IO out
#define SVC_IO_CLEAR        6 // Sets the IO out to 0
#define SVC_IO_SWAP         7 // Alias for Write followed by Read
#define SVC_SELFMOD_MARK    8 // Mark a line that has been modified
#define SVC_SELFMOD_REACHED 9 // A line marked with SVC_SELFMOD_MARK has been reached.

bool supervisorTestComplete = false;

// The program provided by the user (C88)
volatile uint8_t user_program[8] = {0,0,0,0,0,0,0,0};

// Offsets into thumb program for start of each c88 instruction.
// These offsets are specified in bytes. This is used to store
// locations for branch targets, which are patched up after the
// rest of the program is translated.
int thumb_offsets[8] = {0,0,0,0,0,0,0,0};
int thumb_b_patch_points[8] = {0,0,0,0,0,0,0,0};

// The translated program (thumb)
//uint16_t translated_program[100];
volatile uint16_t *translated_program;

// Offset into the translated program that represents where the C88 PC is now
uint32_t translated_pc_offset = 0;

bool isRunning = false;

const int C88_CONFIG_SPEED_SLOW = 1;
const int C88_CONFIG_SPEED_FAST = 2;
const int C88_CONFIG_SPEED_FULL = 3;

int runSpeed = C88_CONFIG_SPEED_SLOW;

int stopCode = 0;


uint32_t c88_reg = 0;
uint32_t c88_io_reg = 0;

int retranslate_required_from = -1;


// WARNING! The Arduino IDE does some nasty stuff with automatically adding forward declarations
// for things. This seems to fall over spectacularly when there is a C++ function defined before
// this extern "C" block. Don't define any functions above this line.
extern "C"{
  // Supervisor handler must be in an extern "C" block to prevent name mangling.
  // Name mangling would prevent the SVC_Handler inline ASM from finding the supervisor symbol.
  void supervisor(uint32_t *svc_args){
    unsigned int svc_number;
    /*
    * Stack contains:
    * r0, r1, r2, r3, r12, r14, the return address and xPSR
    * First argument (r0) is svc_args[0]
    */
    svc_number = ((char *)svc_args[6])[-2]; // Two chars back from the return address
    Serial.print("Supervisor call: ");
    Serial.println(svc_number);
    if (svc_number == SVC_NUMBER(SVC_EXIT_PROGRAM)){
      Serial.println("Supervisor: program exit");
      exit(1);
    }
    if (svc_number == SVC_NUMBER(SVC_SELF_TEST)){
      Serial.println("Supervisor: self test");
      supervisorTestComplete = true;
      return;
    }
    if (svc_number == SVC_NUMBER(SVC_REGULATE_SPEED)){
      Serial.println("Supervisor: regulate speed");
      uint32_t R0 = svc_args[0];
      uint32_t R1 = svc_args[1];
      uint32_t R2 = svc_args[2];
      uint32_t PC = svc_args[6];
      Serial.print("Supervisor: R0: 0x"); Serial.println(R0, HEX);
      Serial.print("Supervisor: R1: 0x"); Serial.println(R1, HEX);
      Serial.print("Supervisor: R2: 0x"); Serial.println(R2, HEX);
      Serial.print("Supervisor: PC: 0x"); Serial.println(PC, HEX);

      //translated_pc_offset = (uint32_t)((uint16_t *)PC - translated_program);

      translated_pc_offset = (PC - (uint32_t)translated_program);
      
      svc_args[6] = R2;
      
      return;
      exit(1);
    }
    if (svc_number == SVC_NUMBER(SVC_SET_STOP)){
      Serial.println("Supervisor: set stop code");
      uint32_t R1 = svc_args[1];
      stopCode = R1;
      isRunning = false;
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_IO_READ)){
      Serial.println("Supervisor: IO Read");
      svc_args[0] = readIO();
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_IO_WRITE)){
      Serial.println("Supervisor: IO Write");
      uint32_t R0 = svc_args[0];
      Serial.print("Write to IO: 0x"); Serial.println(R0, HEX);
      c88_io_reg = R0;
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_IO_CLEAR)){
      Serial.println("Supervisor: IO Clear");
      Serial.println("Write to IO: 0x00");
      c88_io_reg = 0;
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_IO_SWAP)){
      uint32_t R0 = svc_args[0];
      svc_args[0] = readIO();
      Serial.println("Supervisor: IO Swap");
      Serial.print("Write to IO: 0x"); Serial.println(R0);
      c88_io_reg = R0;
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_SELFMOD_MARK)){
      Serial.println("Supervisor: Mark self-modified code");
      // Mark a line of code as being self-modified by injecting an SVC_SELFMOD_REACHED call
      uint32_t R1 = svc_args[1];
      uint32_t offset = (uint32_t)thumb_offsets[R1];
      Serial.print("Supervisor: C88 offset is: "); Serial.println(R1);
      Serial.print("Supervisor: Thumb offset is: 0x"); Serial.println(thumb_offsets[R1], HEX);
      Serial.print("Supervisor: Translated program starts at: 0x"); Serial.println((uint32_t)translated_program, HEX);
      Serial.print("Supervisor: Overwrite address: 0x"); Serial.println((uint32_t)translated_program + offset, HEX);
      uint16_t *targetInstruction = (uint16_t *)(((uint32_t)translated_program) + offset);
      *targetInstruction = encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_SELFMOD_REACHED));
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_SELFMOD_REACHED)){
      Serial.println("Supervisor: Reached self-modified code, must retranslate");
      // A self-modified line has been reached, retranslate
      uint32_t PC = svc_args[6];
      translated_pc_offset = (uint32_t)(PC - (uint32_t)translated_program) - 2;

      Serial.print("Supervisor: PC: 0x"); Serial.println(PC, HEX);
      Serial.print("Supervisor: translated_pc_offset: 0x"); Serial.println(translated_pc_offset, HEX);
      
      for (int i = 0; i<= 7; i++){
        Serial.print("Supervisor: check offset: 0x"); Serial.println(thumb_offsets[i], HEX);  
        if (translated_pc_offset == thumb_offsets[i]){
          int restart_address = i;
          retranslate_required_from = i;
          uint32_t R2 = svc_args[2];
          svc_args[6] = R2;
          return;
        }
      }
      Serial.println("Supervisor: Invalid SELFMOD_REACHED call");
      exit(1);
    }
  }
}

void SVC_Handler(void) __attribute__ ((naked));
void SVC_Handler(void){
  asm volatile(
    "  movs R0, #4           \n"
    "  mov  R1, LR           \n"
    "  tst  R0, R1           \n"
    "  beq  svc_MSP          \n"
    "  mrs  R0, PSP          \n"
    "  b    supervisor       \n"
    "svc_MSP:                \n"
    "  mrs  R0, MSP          \n"
    "  b    supervisor       \n"
  );
}


extern "C"{
  void hardfault(uint32_t *fault_stack){

    uint32_t hfsr, cfsr;
    hfsr = HFSR;
    cfsr = CFSR;
    
    Serial.println("====  Hard Fault  ====");
    Serial.println("======================");
    Serial.print("= Stack frame at: "); Serial.println((uint32_t)fault_stack, HEX);
    Serial.print("= R0:   0x"); Serial.println(fault_stack[0], HEX);
    Serial.print("= R1:   0x"); Serial.println(fault_stack[1], HEX);
    Serial.print("= R2:   0x"); Serial.println(fault_stack[2], HEX);
    Serial.print("= R3:   0x"); Serial.println(fault_stack[3], HEX);
    Serial.print("= R12:  0x"); Serial.println(fault_stack[4], HEX);
    Serial.print("= LR:   0x"); Serial.println(fault_stack[5], HEX);
    Serial.print("= PC:   0x"); Serial.println(fault_stack[6], HEX);
    Serial.print("= xPSR: 0x"); Serial.println(fault_stack[7], HEX);
    Serial.println("======================");
    Serial.println("= Fault analysis...");
    Serial.print("= HFSR: 0x"); Serial.println(hfsr, HEX);
    Serial.print("= CFSR: 0x"); Serial.println(cfsr, HEX);
    Serial.println("======================");
    Serial.print("= Exception on instruction with encoding 0x"); Serial.print(*((uint16_t *)fault_stack[6]), HEX);
    Serial.print(" at location 0x"); Serial.print(fault_stack[6], HEX);

    exit(1);
  }
}

void HardFault_Handler(void) __attribute__ ((naked));
void HardFault_Handler(void){
  asm volatile(
    "  movs R0, #4           \n"
    "  mov  R1, LR           \n"
    "  tst  R0, R1           \n"
    "  beq  hf_MSP           \n"
    "  mrs  R0, PSP          \n"
    "  b    hardfault        \n"
    "hf_MSP:                 \n"
    "  mrs  R0, MSP          \n"
    "  b    hardfault        \n"
  );
}

uint32_t readIO(){
  // Currently there is no way to do external IO.
  // This function simulates wiring the IO output to the IO input.
  return c88_io_reg;
}

void setup() {
  Serial.begin(115200);
  Serial.println("C88_DBT_Zero");

  Serial.println("Supervisor self-test...");
  asm volatile("SVC " SVC_STRING(SVC_SELF_TEST));
  if (!supervisorTestComplete){
    Serial.println("Supervisor self-test failed. Abort!");
    Serial.println("This happens if SVC_Handler is not called when a supervisor call instruction is encountered.");
    Serial.println("The Arduino SAMD boards extension should register SVC_Handler for you, is the board set to \"Arduino/Genuino Zero\"?");
    exit(1);
  }
  else{
    Serial.println("Supervisor self-test passed.");
  }

  translated_program = (uint16_t *)malloc(sizeof(uint16_t) * MAX_TRANSLATED_LENGTH);
  
  isRunning = false;
  /*
  user_program[0] = 0b00000111;
  user_program[1] = 0b10001110;
  user_program[2] = 0b01000000;
  user_program[3] = 0b00000000;
  user_program[4] = 0b00000000;
  user_program[5] = 0b00000000;
  user_program[6] = 0b00000011;
  user_program[7] = 0b00000010;
  */

/* //CYLON
  user_program[0] = 0b01000001;
  user_program[1] = 0b00000001;
  user_program[2] = 0b10100001;
  user_program[3] = 0b00101000;
  user_program[4] = 0b01000010;
  user_program[5] = 0b10101001;
  user_program[6] = 0b00110001;
  user_program[7] = 0b01000101;
  */

  user_program[0] = 0b11100000;
  user_program[1] = 0b00110100;
  user_program[2] = 0b01000000;
  user_program[3] = 0b00001111;
  user_program[4] = 0b00001000;
  user_program[5] = 0b00001111;
  user_program[6] = 0b01000000;
  user_program[7] = 0b11101000;
  
  
  Serial.println("Translate...");
  translate();
  Serial.println("Translated program:");
  showTranslatedProgram();
  
  runSpeed = C88_CONFIG_SPEED_SLOW;
  isRunning = true;
}

uint16_t encode_thumb_16(THUMB16_t instruction, ...){
  va_list args;
  va_start(args, instruction);
  int numArgs = instruction.operands;
  uint16_t output = instruction.opcode;
  int i;
  for (i=0; i<= numArgs -1; i++){
    uint16_t thisArg = (uint16_t)va_arg(args, int);
    int thisShift = instruction.shifts[i];
    uint16_t thisMask = instruction.masks[i];
    output |= ((thisArg << thisShift) & thisMask);
  }
  va_end(args);
  
  return output;
}

int curThumbOffset; // Offset into translated program in number of 16 bit thumb instructions

void thumb_asm(uint16_t instr){
  translated_program[curThumbOffset >> 1] = instr;
  curThumbOffset += 2;
}

void thumb_asm_patch(uint16_t instr, int offset){
  translated_program[offset >> 1] = instr;
}

void translate_from(int start){
  int i;
  curThumbOffset = thumb_offsets[start];
  for (i = start; i<= 7; i++){
    uint8_t thisC88Instr   = user_program[i] & 0b11111000;
    uint8_t thisC88Operand = user_program[i] & 0b00000111;
    thumb_offsets[i] = curThumbOffset;
    if (thisC88Instr == C88_INC){
      Serial.println("INC");
      // INC | Increment register
      thumb_asm(encode_thumb_16(THUMB_ADD_imm_2, ARM_R0, 1)); // R0 <= R0 + 1
    }
    if (thisC88Instr == C88_DEC){
      Serial.println("DEC");
      // DEC | Decrement register
      thumb_asm(encode_thumb_16(THUMB_SUB_imm_2, ARM_R0, 1)); // R0 <= R0 - 1
    }
    if (thisC88Instr == C88_LOAD){
      Serial.println("LOAD");
      // LOAD a | Load contents of memory at a to register
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R0, ARM_R3, thisC88Operand)); // R0 <= mem[a]
    }

    if (thisC88Instr == C88_STORE){
      Serial.println("STORE");
      // STORE a | Store register value at memory address a
      thumb_asm(encode_thumb_16(THUMB_STRB_imm_1, ARM_R0, ARM_R3, thisC88Operand)); // mem[a] <= R0
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, thisC88Operand));         // R1 <= a
      thumb_asm(encode_thumb_16(THUMB_SVC_1,      SVC_NUMBER(SVC_SELFMOD_MARK)));   // Supervisor mark selfmod
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, 0));                      // R1 <= 0
    }

    if (thisC88Instr == C88_SWAP){
      Serial.println("SWAP");
      // SWAP a | Swap register value with memory address a
      thumb_asm(encode_thumb_16(THUMB_MOV_2,      ARM_R1, ARM_R0)); // R1 <= R0
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R0, ARM_R3, thisC88Operand)); // R0 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_STRB_imm_1, ARM_R1, ARM_R3, thisC88Operand)); // mem[a] <= R1
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, thisC88Operand));         // R1 <= a
      thumb_asm(encode_thumb_16(THUMB_SVC_1,      SVC_NUMBER(SVC_SELFMOD_MARK)));   // Supervisor mark selfmod
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, 0));                      // R1 <= 0
    }

    if ((thisC88Instr == C88_ADD) || (thisC88Instr == C88_ADDU)){
      Serial.println("ADD[U]");
      // ADD a | Add contents of memory at a to register
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_ADD_1, ARM_R0, ARM_R0, ARM_R4)); // R0 <= R0 + R4
    }

    if ((thisC88Instr == C88_SUB) || (thisC88Instr == C88_SUBU)){
      Serial.println("SUB[U]");
      // SUB a | Subtract contents of memory at a to register
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_SUB_1,      ARM_R0, ARM_R0, ARM_R4)); // R0 <= R0 + R4
    }

    if (thisC88Instr == C88_MUL){
      Serial.println("MUL");
      // MULU a | Multiply contents of memory at a by register, unsigned
      // Sanitize R0 so it is sign extended
      thumb_asm(encode_thumb_16(THUMB_SXTB_1,     ARM_R0, ARM_R0                )); // R0 <= SignExt(R0[7:0])
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_MUL_1,      ARM_R0, ARM_R4                )); // R0 <= R0 * R4
    }
    
    if (thisC88Instr == C88_MULU){
      Serial.println("MULU");
      // MULU a | Multiply contents of memory at a by register, unsigned
      // Sanitize R0 so it is definately not a signed number
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R4, 0xff                  )); // R4 <= 0xff
      thumb_asm(encode_thumb_16(THUMB_AND_1,      ARM_R0, ARM_R4                )); // R0 <= R0 & R4
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_MUL_1,      ARM_R0, ARM_R4                )); // R0 <= R0 * R4
    }

    if (thisC88Instr == C88_DOUBLE){
      Serial.println("DOUBLE");
      // MULU a | Multiply contents of memory at a by register, unsigned
      // Sanitize R0 so it is sign extended
      thumb_asm(encode_thumb_16(THUMB_SXTB_1,     ARM_R0, ARM_R0 )); // R0 <= SignExt(R0[7:0])
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R4, 2      )); // R4 <= 2
      thumb_asm(encode_thumb_16(THUMB_MUL_1,      ARM_R0, ARM_R4 )); // R0 <= R0 * R4
    }

    if (thisC88Instr == C88_SHL){
      Serial.println("SHL");
      // SHL a | Shift left by immediate value
      thumb_asm(encode_thumb_16(THUMB_LSL_imm_1, ARM_R0, ARM_R0, thisC88Operand)); // R0 <= R0 << a
    }

    if (thisC88Instr == C88_SHR){
      Serial.println("SHR");
      // SHR a | Shift right by immediate value
      thumb_asm(encode_thumb_16(THUMB_LSR_imm_1, ARM_R0, ARM_R0, thisC88Operand)); // R0 <= R0 >> a
    }

    if (thisC88Instr == C88_ROL){
      Serial.println("ROL");
      // ROL a | Rotate left by immediate value
      // There is no direct mapping to a thumb instruction here.
      // Put two copies of the register next to each other, then shift backwards
      // by (8-a)
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1, ARM_R4, 0xff                    )); // R4 <= 0xff
      thumb_asm(encode_thumb_16(THUMB_AND_1,     ARM_R0, ARM_R4                  )); // R0 <= R0 & R4
      thumb_asm(encode_thumb_16(THUMB_LSL_imm_1, ARM_R4, ARM_R0, 8               )); // R4 <= R0 << 8
      thumb_asm(encode_thumb_16(THUMB_ORR_1,     ARM_R0, ARM_R4                  )); // R0 <= R0 | R4
      thumb_asm(encode_thumb_16(THUMB_LSR_imm_1, ARM_R0, ARM_R0, 8-thisC88Operand)); // R0 <= R0 >> (8-a)
      
      /*
      MOV  R4, #ff
      AND  R0, R4
      LSL  R4, R0, 8
      ORR  R0, R4
      LSR  R0, R0, #(8-a)
      */
    }

    if (thisC88Instr == C88_ROR){
      Serial.println("ROR");
      // ROL a | Rotate left by immediate value
      // There is no direct mapping to a thumb instruction here.
      // Put two copies of the register next to each other, then shift
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1, ARM_R4, 0xff                  )); // R4 <= 0xff
      thumb_asm(encode_thumb_16(THUMB_AND_1,     ARM_R0, ARM_R4                )); // R0 <= R0 & R4
      thumb_asm(encode_thumb_16(THUMB_LSL_imm_1, ARM_R4, ARM_R0, 8             )); // R4 <= R0 << 8
      thumb_asm(encode_thumb_16(THUMB_ORR_1,     ARM_R0, ARM_R4                )); // R0 <= R0 | R4
      thumb_asm(encode_thumb_16(THUMB_LSR_imm_1, ARM_R0, ARM_R0, thisC88Operand)); // R0 <= R0 >> a
    }

    if (thisC88Instr == C88_IOW){
      Serial.println("IOW");
      // IOW | Write the register value to the output port
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_IO_WRITE))); // (Supervisor call)
    }

    if (thisC88Instr == C88_IOR){
      Serial.println("IOR");
      // IOR | Copy the value on the input port to the register
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_IO_READ))); // (Supervisor call)
    }

    if (thisC88Instr == C88_IOS){
      Serial.println("IOS");
      // IOS | Write the value of the register to the output port and, simultaneously, copy
      // the input port value to the register.
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_IO_SWAP))); // (Supervisor call)
    }

    if (thisC88Instr == C88_IOC){
      Serial.println("IOC");
      // IOC | Set the output port to 0
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_IO_CLEAR))); // (Supervisor call)
    }

    // For debugging (slower run speeds) insert supervisor calls after each instruction.
    if (runSpeed != C88_CONFIG_SPEED_FULL){
      Serial.println("t-SVC (Speed limiter)");
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_REGULATE_SPEED))); // (Supervisor call)
    }

    if ((thisC88Instr == C88_TSG)||
        (thisC88Instr == C88_TSL)||
        (thisC88Instr == C88_TSE)||
        (thisC88Instr == C88_TSI)){
      Serial.println("TSx");
      // TSx a | Skip the next instruction if mem[a] meets some condition
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R4, 0xFF));                   // R4 <= 0xFF
      thumb_asm(encode_thumb_16(THUMB_AND_1,      ARM_R0, ARM_R4));                 // R0 <= R0 & R4
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_CMP_1,      ARM_R4, ARM_R0));                 // flags = flags_for("R4 - R0")
      // Just like the JMP instruction, this will need patching as we don't
      // know how far to jump yet
      thumb_b_patch_points[i] = curThumbOffset;
      thumb_asm(encode_thumb_16(THUMB_NOP));
    }

    if (thisC88Instr == C88_JMP){
      Serial.println("JMP");
      // JMP a | Jump to address a.
      // The branch address (in the thumb program) might not be known yet
      // This code just allocates space for the jump, which will be patched up later.
      // We just need one sixteen bit instruction of space
      thumb_b_patch_points[i] = curThumbOffset;
      thumb_asm(encode_thumb_16(THUMB_NOP));
    }

    if (thisC88Instr == C88_STOP){
      Serial.println("STOP");
      // STOP | Stop the program, the operand is discarded on the C88, but is
      // logged by this DBT, just because it can and it might be useful.
      // The return code is set and then a BX to the link register takes us back
      // to the DBT main loop via the helper function.
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1, ARM_R1, thisC88Operand  )); // R1 <= a
      thumb_asm(encode_thumb_16(THUMB_SVC_1,     SVC_NUMBER(SVC_SET_STOP))); // (Supervisor call)
      thumb_asm(encode_thumb_16(THUMB_BX_1,      ARM_LR                  )); // BX LR
    }
  }

  // Insert a jump at the end of the program, to get back to the start. To simulate the wrap-around behaviour.
  int targetOffset = thumb_offsets[0]; // Should always be zero
  int relativeJump = targetOffset - curThumbOffset;
  Serial.println("t-B (loop to zero)");
  thumb_asm(encode_thumb_16(THUMB_B_2, relativeJump)); // PC <= PC + relativeJump

  // Loop over the program again, patching up the branches that we couldn't
  // assemble in the first pass.
  for (i =0; i<= 7; i++){
    uint8_t thisC88Instr   = user_program[i] & 0b11111000;
    uint8_t thisC88Operand = user_program[i] & 0b00000111;
    if (thisC88Instr == C88_JMP){
      Serial.println("Patching JMP");
      // JMP a | Jump to address a.
      // All jumps on the C88 are absolute, thumb branches are relative.
      // Lookup the offset to the target instruction and do some maths.
      int targetOffset = thumb_offsets[thisC88Operand];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_2, relativeJump), patch_offset); // PC <= PC + relativeJump
    }
    if (thisC88Instr == C88_TSG){
      Serial.println("Patching TSG");
      // TSG a | Skip the next instruction if mem[a] is greater than the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_1, THUMB_COND_GT, relativeJump), patch_offset); // If flags[GT] then PC <= PC + relativeJump
    }
    if (thisC88Instr == C88_TSL){
      Serial.println("Patching TSL");
      // TSL a | Skip the next instruction if mem[a] is less than the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_1, THUMB_COND_LT, relativeJump), patch_offset); // If flags[LT] then PC <= PC + relativeJump
    }
    if (thisC88Instr == C88_TSE){
      Serial.println("Patching TSE");
      // TSE a | Skip the next instruction if mem[a] is equal to the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_1, THUMB_COND_EQ, relativeJump), patch_offset); // If flags[Z] then PC <= PC + relativeJump
    }
    if (thisC88Instr == C88_TSI){
      Serial.println("Patching TSI");
      // TSI a | Skip the next instruction if mem[a] is equal to the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_1, THUMB_COND_NE, relativeJump), patch_offset); // If !flags[Z] then PC <= PC + relativeJump
    }
  }

}


void translate(){
  // Full retranslation.
  // Set the base offset to 0
  thumb_offsets[0] = 0; // This should never become non-zero anyway
  translate_from(0);
}

void showTranslatedProgram(){
  for (int i = 0; i<= 10; i++){
    Serial.print("0x");
    Serial.print((uint32_t)(translated_program + i), HEX);
    Serial.print(": 0x");
    Serial.println(translated_program[i], HEX);
  }
}

// This function will call an array as if it is a function.
// Three parameters can be passed to the function. If the called function
// returns normally, so too will this function.
volatile void __attribute__ ((noinline)) call_translated_program(uint32_t R0in, uint32_t R1in, volatile void *target, volatile uint8_t *R3in) __attribute__ ((naked));
volatile void __attribute__ ((noinline)) call_translated_program(uint32_t R0in, uint32_t R1in, volatile void *target, volatile uint8_t *R3in){
  // Use BLX to call the target code
  // when using BLX, the LSB indicates the instruction set state.
  // This bit must be 1 to indicate that we should use thumb state.
  // This function also copies the return address from the stack into r2
  // so that the supervisor can reliably find the return address.
  asm(
    "  mov  r5, r2   \n" // r5 <= target
    "  mov  r2, lr   \n" // r2 <= lr (return address for DBT)
    "  mov  r4, #1   \n" // r4 <= 1
    "  orr  r5, r4   \n" // r5 <= target | 1 (r5 is the target with the thumb bit set)
    "  blx  r5       \n" // A return from this call happens for stop instructions
                         // however, this code path may be skipped when returning to
                         // the DBT main loop. Is this a tail call? Maybe! This function
                         // Doesn't allocate any stack space, so it's safe to let the
                         // translated program return to the DBT directly. When the
                         // translated program does a stop instruction however, this
                         // is not a tail call at all. Control returns here...
    "  bx   r2       \n" // And this bx takes us back to the DBT main loop.
                         // Why two different paths? Well, it's just arbitrary really.
                         // It felt like a good idea to have the STOP path be a little
                         // more final and to allow it to execute without needing the
                         // supervisor in the way if all it would be doing is setting
                         // the return address to something we already know.
  );
  // IMPORTANT NOTE: R0, R1 and R3 are not touched by this code.
  // By making use of the calling convention, these registers allready contain
  // the values that the translated program expects.
}


volatile void dbt_loop(){
  //Serial.println("Translated program:");
  //showTranslatedProgram();

  if (isRunning){
    Serial.println("Running program");
    
    volatile uint32_t R0 = c88_reg;
    volatile uint32_t R1 = 0;
    //volatile uint16_t *target = translated_program + translated_pc_offset;
    volatile uint16_t *target = (uint16_t*)((uint32_t)translated_program + translated_pc_offset);
    volatile uint8_t  *originalProgram = user_program;
    Serial.print("From address: 0x");Serial.println((uint32_t)target, HEX);
    // This function will call the translated program after setting the registers
    call_translated_program(R0, R1, target, originalProgram);
    //exit(1);
    asm("":::"r2", "r3", "r4", "r5");// does this work? if so why?
    // Now we need to extract the R0 value before anything else has a chance to
    // clobber them. This is probably safe, as nothing should happen after the translated
    // program returns before we do this.
    // This wastes a register, but it's the easiest way.
    asm(
      "mov %[out_r0], r0 \n"
      : [out_r0] "=r" (R0)
      ::
    );
    // TODO, is the above doable by using the result of the function call?
  
    c88_reg = R0;
  
    if (!isRunning){
      Serial.print("STOP instruction encountered, return code was: ");
      Serial.println(stopCode);
    }
    
    Serial.print("R0:  "); Serial.println(R0);
    Serial.print("R1:  "); Serial.println(R1);

    Serial.print("In:  "); Serial.println(readIO());
    Serial.print("Out: "); Serial.println(c88_io_reg);

    if (retranslate_required_from >= 0){
      Serial.println("Retranslating...");
      translate_from(retranslate_required_from);
      // Get the new pc offset value after the [partial] retranslation
      translated_pc_offset = thumb_offsets[retranslate_required_from];
      retranslate_required_from = -1;
      Serial.println("Retranslation done");
    }
    
  }
  else{
    Serial.println("(idle)");
  }

  delay(100);
}

void loop() {
  dbt_loop();
}
