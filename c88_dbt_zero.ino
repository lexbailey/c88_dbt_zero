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
 *  program. This calls the supervisor, which will set the return address to be
 *  the DBT's main loop and then return from the supervisor handler.
 *  The DBT now does the appropriate amount of waiting before jumping back to the
 *  translated program to continue execution.
 *  
 *  WARNING! Because the supervisor will edit the translated program's stack frame,
 *  the translated program must never change the stack pointer. That means no push
 *  and pop allowed! Otherwise the supervisor might overwrite the wrong part of the
 *  stack frame.
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
#include <LedControl.h>
#include "c88instructions.h"
#include "ARMinstructions.h"
#include "inputs.h"

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
#define SVC_DIVIDE         10 // Divide R0 by R1, result in R0
#define SVC_LOCATE_STACK   11 // Called once at the start of the program to find the stack pointer.

LedControl screen=LedControl(12,11,10,1);

bool supervisorTestComplete = false;

// The program provided by the user (C88)
volatile uint8_t user_program[8] = {0,0,0,0,0,0,0,0};

// Offsets into thumb program for start of each c88 instruction.
// These offsets are specified in bytes. This is used to store
// locations for branch targets, which are patched up after the
// rest of the program is translated.
int thumb_offsets[8] = {0,0,0,0,0,0,0,0};
int thumb_b_patch_points[8] = {0,0,0,0,0,0,0,0};

uint16_t *thumb_branch_targets[8] = {0,0,0,0,0,0,0,0};

// The translated program (thumb)
//uint16_t translated_program[100];
volatile uint16_t *translated_program;

// Offset into the translated program that represents where the C88 PC is now
uint32_t translated_pc_offset = 0;

bool isRunning = false;

const int C88_CONFIG_SPEED_SLOW = 1;
const int C88_CONFIG_SPEED_FAST = 2;
const int C88_CONFIG_SPEED_FULL = 3;

int runSpeed = C88_CONFIG_SPEED_FAST;

int stopCode = 0;

int inProgram = 0;

uint32_t c88_reg = 0;
uint32_t c88_io_reg = 0;

int retranslate_required_from = -1;

uint32_t *translatedProgramStackPointer = 0;

int screenIntensity = 4;

int viewMode = VIEW_MEM;

int FAULT_ICON[8] = {0xff, 0xc3, 0xa5, 0x99, 0x99, 0xa5, 0xc3, 0xff};

boolean shouldStep = false;

bool donesetup = false;

#define SUPERVISOR_DEBUG
#define DEBUG_TRANSLATION

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
    #ifdef SUPERVISOR_DEBUG
    Serial.print("Supervisor call: ");
    Serial.println(svc_number);
    #endif
    if (svc_number == SVC_NUMBER(SVC_EXIT_PROGRAM)){
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: program exit");
      #endif
      exit(1);
    }
    if (svc_number == SVC_NUMBER(SVC_SELF_TEST)){
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: self test");
      #endif
      supervisorTestComplete = true;
      return;
    }
    if (svc_number == SVC_NUMBER(SVC_REGULATE_SPEED)){
      inProgram = 0;
      uint32_t R0 = svc_args[0];
      uint32_t R1 = svc_args[1];
      uint32_t R2 = svc_args[2];
      uint32_t PC = svc_args[6];
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: regulate speed");
      Serial.print("Supervisor: R0: 0x"); Serial.println(R0, HEX);
      Serial.print("Supervisor: R1: 0x"); Serial.println(R1, HEX);
      Serial.print("Supervisor: R2: 0x"); Serial.println(R2, HEX);
      Serial.print("Supervisor: PC: 0x"); Serial.println(PC, HEX);
      #endif
      
      translated_pc_offset = (PC - (uint32_t)translated_program);

      // This is a horrible hack!
      // This overwrites the return address the stack frame of the function that called the supervisor.
      svc_args[6] = R2;

      if (shouldStep){
        shouldStep = false;
        isRunning = false;
      }
      
      return;
      exit(1);
    }
    if (svc_number == SVC_NUMBER(SVC_SET_STOP)){
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: set stop code");
      #endif
      uint32_t R1 = svc_args[1];
      stopCode = R1;
      isRunning = false;
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_IO_READ)){
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: IO Read");
      #endif
      svc_args[0] = readIO();
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_IO_WRITE)){
      
      uint32_t R0 = svc_args[0];
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: IO Write");
      Serial.print("Write to IO: 0x"); Serial.println(R0, HEX);
      #endif
      c88_io_reg = R0;
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_IO_CLEAR)){
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: IO Clear");
      Serial.println("Write to IO: 0x00");
      #endif
      c88_io_reg = 0;
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_IO_SWAP)){
      uint32_t R0 = svc_args[0];
      svc_args[0] = readIO();
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: IO Swap");
      Serial.print("Write to IO: 0x"); Serial.println(R0);
      #endif
      c88_io_reg = R0;
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_SELFMOD_MARK)){
      
      // Mark a line of code as being self-modified by injecting an SVC_SELFMOD_REACHED call
      uint32_t R1 = svc_args[1];
      uint32_t offset = (uint32_t)thumb_offsets[R1];
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: Mark self-modified code");
      Serial.print("Supervisor: C88 offset is: "); Serial.println(R1);
      Serial.print("Supervisor: Thumb offset is: 0x"); Serial.println(thumb_offsets[R1], HEX);
      Serial.print("Supervisor: Translated program starts at: 0x"); Serial.println((uint32_t)translated_program, HEX);
      Serial.print("Supervisor: Overwrite address: 0x"); Serial.println((uint32_t)translated_program + offset, HEX);
      #endif
      uint16_t *targetInstruction = (uint16_t *)(((uint32_t)translated_program) + offset);
      *targetInstruction = encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_SELFMOD_REACHED));
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_SELFMOD_REACHED)){
      
      // A self-modified line has been reached, retranslate
      uint32_t PC = svc_args[6];
      translated_pc_offset = (uint32_t)(PC - (uint32_t)translated_program) - 2;
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: Reached self-modified code, must retranslate");
      Serial.print("Supervisor: PC: 0x"); Serial.println(PC, HEX);
      Serial.print("Supervisor: translated_pc_offset: 0x"); Serial.println(translated_pc_offset, HEX);
      #endif
      
      for (int i = 0; i<= 7; i++){
        if (translated_pc_offset == thumb_offsets[i]){
          int restart_address = i;
          retranslate_required_from = i;
          uint32_t R2 = svc_args[2];
          svc_args[6] = R2;
          return;
        }
      }
      
      Serial.println("Error: Invalid SELFMOD_REACHED call");
      exit(1);
    }

    if (svc_number == SVC_NUMBER(SVC_DIVIDE)){
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: Divide");
      #endif
      uint32_t R0 = svc_args[0];
      uint32_t R1 = svc_args[1];
      svc_args[0] = R0 / R1;
      return;
    }

    if (svc_number == SVC_NUMBER(SVC_LOCATE_STACK)){
      #ifdef SUPERVISOR_DEBUG
      Serial.println("Supervisor: Locate stack pointer");
      #endif
      translatedProgramStackPointer = svc_args;
      if (runSpeed == C88_CONFIG_SPEED_FULL){
        inProgram = 1;
      }
      return;
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

    for (int i = 0; i<= 7; i++){
      screen.setRow(0,i, FAULT_ICON[i]);
    }

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

void updateScreen(){

  if (viewMode == VIEW_PC){
    for (int i = 0; i<= 7; i++){
      screen.setRow(0,i, calculateC88PC(translated_pc_offset));
    }
  }
  
  if (viewMode == VIEW_REG){
    for (int i = 0; i<= 7; i++){
      screen.setRow(0,i, c88_reg);
    }
  }

  if (viewMode == VIEW_MEM){
    for (int i = 0; i<= 7; i++){
      screen.setRow(0,i, user_program[i]);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("C88_DBT_Zero");

  initInputs();

  // Test the supervisor, to make sure it works
  // If the build flow is changed slightly or a supervisor handler registered
  // by some other library then this self test will fail.
  asm volatile("SVC " SVC_STRING(SVC_SELF_TEST));
  if (!supervisorTestComplete){
    Serial.println("Supervisor self-test failed. Abort!");
    Serial.println("This happens if SVC_Handler is not called when a supervisor call instruction is encountered.");
    Serial.println("The Arduino SAMD boards extension should register SVC_Handler for you, is the board set to \"Arduino/Genuino Zero\"?");
    exit(1);
  }

  translated_program = (uint16_t *)malloc(sizeof(uint16_t) * MAX_TRANSLATED_LENGTH);
  runSpeed = C88_CONFIG_SPEED_FAST;
  isRunning = false;

  // Start with blank program
  for (int i = 0; i<= 7; i++){
    user_program[i] = 0x0;
  }
  
  translate();
  showTranslatedProgram();
  
  isRunning = true;

  screen.setIntensity(0,screenIntensity);
  screen.clearDisplay(0);
  screen.shutdown(0,false);
  updateScreen();

  donesetup = true;
}

int calculateC88PC(int offset){
  for (int i = 0; i<= 7; i++){
    if (offset == thumb_offsets[i]){
      return i;
    }
    if (offset < thumb_offsets[i]){
      return i-1;
    }
  }
  return 0;
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

void translateDebug(const char * msg){
  #ifdef DEBUG_TRANSLATION
  Serial.println(msg);
  #endif
}

void translate_from(int start){
  int i;
  curThumbOffset = thumb_offsets[start];
  for (i = start; i<= 7; i++){
    uint8_t thisC88Instr   = user_program[i] & 0b11111000;
    uint8_t thisC88Operand = user_program[i] & 0b00000111;
    thumb_offsets[i] = curThumbOffset;
    if (thisC88Instr == C88_INC){
      translateDebug("INC");
      // INC | Increment register
      thumb_asm(encode_thumb_16(THUMB_ADD_imm_2, ARM_R0, 1)); // R0 <= R0 + 1
    }
    if (thisC88Instr == C88_DEC){
      translateDebug("DEC");
      // DEC | Decrement register
      thumb_asm(encode_thumb_16(THUMB_SUB_imm_2, ARM_R0, 1)); // R0 <= R0 - 1
    }
    if (thisC88Instr == C88_LOAD){
      translateDebug("LOAD");
      // LOAD a | Load contents of memory at a to register
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R0, ARM_R3, thisC88Operand)); // R0 <= mem[a]
    }

    if (thisC88Instr == C88_STORE){
      translateDebug("STORE");
      // STORE a | Store register value at memory address a
      thumb_asm(encode_thumb_16(THUMB_STRB_imm_1, ARM_R0, ARM_R3, thisC88Operand)); // mem[a] <= R0
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, thisC88Operand));         // R1 <= a
      thumb_asm(encode_thumb_16(THUMB_SVC_1,      SVC_NUMBER(SVC_SELFMOD_MARK)));   // Supervisor mark selfmod
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, 0));                      // R1 <= 0
    }

    if (thisC88Instr == C88_SWAP){
      translateDebug("SWAP");
      // SWAP a | Swap register value with memory address a
      thumb_asm(encode_thumb_16(THUMB_MOV_2,      ARM_R1, ARM_R0)); // R1 <= R0
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R0, ARM_R3, thisC88Operand)); // R0 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_STRB_imm_1, ARM_R1, ARM_R3, thisC88Operand)); // mem[a] <= R1
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, thisC88Operand));         // R1 <= a
      thumb_asm(encode_thumb_16(THUMB_SVC_1,      SVC_NUMBER(SVC_SELFMOD_MARK)));   // Supervisor mark selfmod
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, 0));                      // R1 <= 0
    }

    if ((thisC88Instr == C88_ADD) || (thisC88Instr == C88_ADDU)){
      translateDebug("ADD[U]");
      // ADD a | Add contents of memory at a to register
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_ADD_1, ARM_R0, ARM_R0, ARM_R4)); // R0 <= R0 + R4
    }

    if ((thisC88Instr == C88_SUB) || (thisC88Instr == C88_SUBU)){
      translateDebug("SUB[U]");
      // SUB a | Subtract contents of memory at a to register
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_SUB_1,      ARM_R0, ARM_R0, ARM_R4)); // R0 <= R0 + R4
    }

    if (thisC88Instr == C88_MUL){
      translateDebug("MUL");
      // MULU a | Multiply contents of memory at a by register, unsigned
      // Sanitize R0 so it is sign extended
      thumb_asm(encode_thumb_16(THUMB_SXTB_1,     ARM_R0, ARM_R0                )); // R0 <= SignExt(R0[7:0])
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_MUL_1,      ARM_R0, ARM_R4                )); // R0 <= R0 * R4
    }
    
    if (thisC88Instr == C88_MULU){
      translateDebug("MULU");
      // MULU a | Multiply contents of memory at a by register, unsigned
      // Sanitize R0 so it is definately not a signed number
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R4, 0xff                  )); // R4 <= 0xff
      thumb_asm(encode_thumb_16(THUMB_AND_1,      ARM_R0, ARM_R4                )); // R0 <= R0 & R4
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_MUL_1,      ARM_R0, ARM_R4                )); // R0 <= R0 * R4
    }

    if (thisC88Instr == C88_DOUBLE){
      translateDebug("DOUBLE");
      // MULU a | Multiply contents of memory at a by register, unsigned
      // Sanitize R0 so it is sign extended
      thumb_asm(encode_thumb_16(THUMB_SXTB_1,     ARM_R0, ARM_R0 )); // R0 <= SignExt(R0[7:0])
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R4, 2      )); // R4 <= 2
      thumb_asm(encode_thumb_16(THUMB_MUL_1,      ARM_R0, ARM_R4 )); // R0 <= R0 * R4
    }

    if ((thisC88Instr == C88_DIV) || (thisC88Instr == C88_MULU)){
      translateDebug("DIV[U]");
      // DIV [u] | Divide register by mem[a]
      // Sanitize R0 so it is sign extended
      thumb_asm(encode_thumb_16(THUMB_SXTB_1,     ARM_R0, ARM_R0                )); // R0 <= SignExt(R0[7:0])
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R1, ARM_R3, thisC88Operand)); // R1 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_SXTB_1,     ARM_R1, ARM_R1                )); // R1 <= SignExt(R1[7:0])
      thumb_asm(encode_thumb_16(THUMB_SVC_1,      SVC_NUMBER(SVC_DIVIDE)        )); // SVC divide
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, 0                     )); // R1 <= 0
    }

    if (thisC88Instr == C88_HALF){
      translateDebug("HALF");
      // HALF | Half the register
      // Sanitize R0 so it is sign extended
      thumb_asm(encode_thumb_16(THUMB_SXTB_1,     ARM_R0, ARM_R0        )); // R0 <= SignExt(R0[7:0])
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, 2             )); // R1 <= 2
      thumb_asm(encode_thumb_16(THUMB_SVC_1,      SVC_NUMBER(SVC_DIVIDE))); // SVC divide
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, 0             )); // R1 <= 0
    }

    if (thisC88Instr == C88_SHL){
      translateDebug("SHL");
      // SHL a | Shift left by immediate value
      thumb_asm(encode_thumb_16(THUMB_LSL_imm_1, ARM_R0, ARM_R0, thisC88Operand)); // R0 <= R0 << a
    }

    if (thisC88Instr == C88_SHR){
      translateDebug("SHR");
      // SHR a | Shift right by immediate value
      thumb_asm(encode_thumb_16(THUMB_LSR_imm_1, ARM_R0, ARM_R0, thisC88Operand)); // R0 <= R0 >> a
    }

    if (thisC88Instr == C88_ROL){
      translateDebug("ROL");
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
      translateDebug("ROR");
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
      translateDebug("IOW");
      // IOW | Write the register value to the output port
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_IO_WRITE))); // (Supervisor call)
    }

    if (thisC88Instr == C88_IOR){
      translateDebug("IOR");
      // IOR | Copy the value on the input port to the register
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_IO_READ))); // (Supervisor call)
    }

    if (thisC88Instr == C88_IOS){
      translateDebug("IOS");
      // IOS | Write the value of the register to the output port and, simultaneously, copy
      // the input port value to the register.
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_IO_SWAP))); // (Supervisor call)
    }

    if (thisC88Instr == C88_IOC){
      translateDebug("IOC");
      // IOC | Set the output port to 0
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_IO_CLEAR))); // (Supervisor call)
    }

    // For debugging (slower run speeds) insert supervisor calls after each instruction.
    if (runSpeed != C88_CONFIG_SPEED_FULL){
      translateDebug("-- SVC(Speed limiter)");
      thumb_asm(encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_REGULATE_SPEED))); // (Supervisor call)
    }

    if ((thisC88Instr == C88_TSG)||
        (thisC88Instr == C88_TSL)||
        (thisC88Instr == C88_TSE)||
        (thisC88Instr == C88_TSI)){
      translateDebug("TSx");
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
      translateDebug("JMP");
      // JMP a | Jump to address a.
      // The branch address (in the thumb program) might not be known yet
      // This code just allocates space for the jump, which will be patched up later.
      // We just need one sixteen bit instruction of space
      thumb_b_patch_points[i] = curThumbOffset;
      thumb_asm(encode_thumb_16(THUMB_NOP));
    }

    if (thisC88Instr == C88_JMA){
      translateDebug("JMA");
      // JMA a | Jump to address stored at memory location a.
      // This jump can't be inserted as a single instruction in the patch-up stage
      // because the jump location isn't known ahead of time. Instead the translated
      // code calculates the address by looking at the thumb offset table.
      // This requires a literal value to be inserted into the code.
      thumb_asm(encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand)); // R4 <= mem[a]
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, 0x07                  )); // R1 <= 0x07
      thumb_asm(encode_thumb_16(THUMB_AND_1,      ARM_R4, ARM_R1                )); // R4 <= R4 & R1
      thumb_asm(encode_thumb_16(THUMB_LSL_imm_1,  ARM_R4, ARM_R4, 2             )); // R4 <= R4 << 2

      // Next we need to insert a literal load, which means calculating where the literal will go now.
      int literalDistance = 1;
      uint32_t nextPC = (uint32_t)translated_program + (uint32_t)curThumbOffset;
      Serial.print("PC of literal load is: 0x"); Serial.println(nextPC, HEX);
      uint32_t nextPCAligned = nextPC &~3;
      Serial.print("Aligned PC of literal load is: 0x"); Serial.println(nextPCAligned, HEX);
      boolean padding_required = nextPC != nextPCAligned;
      if (padding_required){
        literalDistance += 1;
      }
      thumb_asm(encode_thumb_16(THUMB_LDR_lit_1,  ARM_R1, literalDistance       )); // R1 <= thumb_branch_targets
      thumb_asm(encode_thumb_16(THUMB_LDR_reg_1,  ARM_R4, ARM_R1, ARM_R4        )); // R4 <= R1[R4]
      thumb_asm(encode_thumb_16(THUMB_MOV_imm_1,  ARM_R1, 0                     )); // R1 <= 0
      thumb_asm(encode_thumb_16(THUMB_BX_1,       ARM_R4                        )); // PC <= R4
      // Now we can insert the literal, but we need to make sure that it's word-aligned
      if (padding_required){
        thumb_asm(0); // Add one halfword to pad to word-alignment
      }
      Serial.print("thumb_branch_targets are stored at: 0x"); Serial.println((int)thumb_branch_targets, HEX);
      uint16_t literalLow = (uint16_t)((uint32_t)thumb_branch_targets & 0xffff);
      uint16_t literalHigh = (uint16_t)((uint32_t)thumb_branch_targets >> 16);
      thumb_asm(literalLow);
      thumb_asm(literalHigh);
      
    }

    if (thisC88Instr == C88_STOP){
      translateDebug("STOP");
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
  int relativeJump = (targetOffset - curThumbOffset)>>1;
  translateDebug("--B(loop to zero)");
  thumb_asm(encode_thumb_16(THUMB_B_2, relativeJump)); // PC <= PC + relativeJump

  // Loop over the program again, patching up the branches that we couldn't
  // assemble in the first pass.
  for (i =0; i<= 7; i++){
    uint8_t thisC88Instr   = user_program[i] & 0b11111000;
    uint8_t thisC88Operand = user_program[i] & 0b00000111;
    if (thisC88Instr == C88_JMP){
      translateDebug("Patching JMP");
      // JMP a | Jump to address a.
      // All jumps on the C88 are absolute, thumb branches are relative.
      // Lookup the offset to the target instruction and do some maths.
      int targetOffset = thumb_offsets[thisC88Operand];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_2, relativeJump), patch_offset); // PC <= PC + relativeJump
    }
    if (thisC88Instr == C88_TSG){
      translateDebug("Patching TSG");
      // TSG a | Skip the next instruction if mem[a] is greater than the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_1, THUMB_COND_GT, relativeJump), patch_offset); // If flags[GT] then PC <= PC + relativeJump
    }
    if (thisC88Instr == C88_TSL){
      translateDebug("Patching TSL");
      // TSL a | Skip the next instruction if mem[a] is less than the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_1, THUMB_COND_LT, relativeJump), patch_offset); // If flags[LT] then PC <= PC + relativeJump
    }
    if (thisC88Instr == C88_TSE){
      translateDebug("Patching TSE");
      // TSE a | Skip the next instruction if mem[a] is equal to the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_1, THUMB_COND_EQ, relativeJump), patch_offset); // If flags[Z] then PC <= PC + relativeJump
    }
    if (thisC88Instr == C88_TSI){
      translateDebug("Patching TSI");
      // TSI a | Skip the next instruction if mem[a] is equal to the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      thumb_asm_patch(encode_thumb_16(THUMB_B_1, THUMB_COND_NE, relativeJump), patch_offset); // If !flags[Z] then PC <= PC + relativeJump
    }
  }

  // Create a one-step lookup table for JMA instructions.
  // It'd be better to trade some JMA latency for faster translations,
  // because JMA is almost never used.
  // TODO fix JMA so that it does the two lookups required, to remove
  // the need to create this table.
  for (int i = 0; i<= 7; i++){
    thumb_branch_targets[i] = (uint16_t *) ((((uint32_t)translated_program) + ((uint32_t)thumb_offsets[i])) | 0x1);
  }

}

void translate(){
  // Full retranslation.
  // Set the base offset to 0
  thumb_offsets[0] = 0; // This should never become non-zero anyway
  translate_from(0);
}

void showTranslatedProgram(){
  #ifdef DEBUG_TRANSLATION
  for (int i = 0; i<= 99; i++){
    Serial.print("0x");
    Serial.print((uint32_t)(translated_program + i), HEX);
    Serial.print(": 0x");
    Serial.println(translated_program[i], HEX);
  }
  #endif
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
    "  svc " SVC_STRING(SVC_LOCATE_STACK) "\n"
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
    volatile uint32_t R0 = c88_reg;
    volatile uint32_t R1 = 0;
    //volatile uint16_t *target = translated_program + translated_pc_offset;
    volatile uint16_t *target = (uint16_t*)((uint32_t)translated_program + translated_pc_offset);
    volatile uint8_t  *originalProgram = user_program;
    // This function will call the translated program after setting the registers
    call_translated_program(R0, R1, target, originalProgram);
    //exit(1);
    asm("":::"r2", "r3", "r4", "r5");// does this work? if so why?
    // Now we need to extract the R0 value before anything else has a chance to
    // clobber it. This is probably safe, as nothing should happen after the translated
    // program returns before we do this.
    // This wastes a register, but it's the easiest way.
    asm(
      "mov %[out_r0], r0 \n"
      : [out_r0] "=r" (R0)
      ::
    );
    // TODO, is the above doable by using the result of the function call?
  
    c88_reg = R0;
    
    Serial.println("Program interrupted");
    Serial.print("New register value: 0x"); Serial.println(c88_reg);
    if (!isRunning){
      Serial.print("STOP instruction encountered, return code was: ");
      Serial.println(stopCode);
    }

    if (retranslate_required_from >= 0){
      translate_from(retranslate_required_from);
      // Get the new pc offset value after the [partial] retranslation
      translated_pc_offset = thumb_offsets[retranslate_required_from];
      retranslate_required_from = -1;
    }
    
  }
  else{
    // Idle, just wait for a while, but not infinitely so that we
    // can still poll switches to check for a reset.
    delay(100);
  }

  if (runSpeed == C88_CONFIG_SPEED_FAST){
    delay(1);
  }
  else if (runSpeed == C88_CONFIG_SPEED_SLOW){
    delay(100);
  }
  
}

long lastBrightnessChange = millis();

void handleBrightness(){
  if ((lastBrightnessChange + 50) > millis()){return;}
  int intensityChange = getBrightnessChange();
  if (intensityChange != 0){
    lastBrightnessChange = millis();
    screenIntensity += intensityChange;
    screenIntensity = min(16, max(0, screenIntensity));
    screen.setIntensity(0,screenIntensity);
  }
}

void reset(){
  isRunning = false;
  c88_reg = 0;
  translated_pc_offset = 0;
}

void updateClockSpeed(){
  int oldRunSpeed = runSpeed;
  int clockMode = getClockMode();
  if (clockMode == CLOCK_INPUT_SLOW){ runSpeed = C88_CONFIG_SPEED_SLOW; }
  if (clockMode == CLOCK_INPUT_FAST){ runSpeed = C88_CONFIG_SPEED_FAST; }
  if (clockMode == CLOCK_INPUT_FULL){ runSpeed = C88_CONFIG_SPEED_FULL; }
  if ((oldRunSpeed != runSpeed) &&(oldRunSpeed == C88_CONFIG_SPEED_FULL || runSpeed == C88_CONFIG_SPEED_FULL)){
    translate();
    reset();
  }
}

void updateViewMode(){
  viewMode = getViewMode();
}

boolean prevStep = false;

void handleRunAndReset(){
  isRunning = false;
  if (isStepHeld() != prevStep){
    prevStep = isStepHeld();
    if (prevStep){
      shouldStep = true;
      isRunning = true;
    }
  }
  if (!isRunning){
    isRunning = isRunOn();
  }
  if (isInReset()){
    // When held in reset, don't run, clear the register and PC.
    reset();
  }
  if (isInUserMode()){
    isRunning = false;
  }
}

void handleMemWrites(){
  if (isInUserMode()){
    // Poll the user input switches to check if we need to write to memory
    if (isWriteEnabled()){
      // Get the address
      uint32_t addr = getAddrInput(); 
      // Modify the program
      user_program[addr] = getDataInput();
      // Call the supervisor to notify it about the modified instruction.
      asm(
        "mov r1, %[modifiedAddress] \n\t"
        "svc " SVC_STRING(SVC_SELFMOD_MARK) "\n\t"
        : 
        : [modifiedAddress] "r" (addr)
        : "r1"
      );
    }
  }
}

void handleInputs(){
  handleBrightness();
  updateClockSpeed();
  updateViewMode();
  handleRunAndReset();
  handleMemWrites();
}

void loop() {
  if (!powerIsOn()){
    screen.clearDisplay(0);
    return;
  }
  handleInputs();
  dbt_loop();
  updateScreen();
  //debugInputs();
}

int sysTickHook(){
  noInterrupts();
  if (donesetup){
    if (inProgram){
      Serial.println("Program needs interrupting.");
      // If this flag is set, we need to bail out of the program
      inProgram = 0;
      if (translatedProgramStackPointer != 0){
        translated_pc_offset = (translatedProgramStackPointer[6] - (uint32_t)translated_program);
        translatedProgramStackPointer[6] = translatedProgramStackPointer[2];
      }
    }
  }
  interrupts();
  return 0;
}


