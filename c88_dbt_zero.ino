
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
 *  There is also a need for the C88 program to access the untranslated version of
 *  itself. The base pointer to the original C88 program is therefore placed in R3.
 *  This allows use of LDRB Rt, [R3, #n] to get the nth byte of the C88 program
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

#define MAX_TRANSLATED_LENGTH (100)

#define SVC_NUMBER(x) (x)
#define SVC_STRING_SUB(x) #x
#define SVC_STRING(x) SVC_STRING_SUB(x)
#define SVC_EXIT_PROGRAM   0 // Handy for testing, just so I don't have to insert a branch to the exit function
#define SVC_SELF_TEST      1
#define SVC_REGULATE_SPEED 2

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

bool isRun = false;

const int C88_CONFIG_SPEED_SLOW = 1;
const int C88_CONFIG_SPEED_FAST = 2;
const int C88_CONFIG_SPEED_FULL = 3;

int runSpeed = C88_CONFIG_SPEED_SLOW;

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

      translated_pc_offset = (uint32_t)((uint16_t *)PC - translated_program);
      
      svc_args[6] = R2;
      
      return;
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
  
  isRun = false;
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

  user_program[0] = 0b11100000;
  user_program[1] = 0b10110001;
  user_program[2] = 0b01000001;
  user_program[3] = 0b00000000;
  user_program[4] = 0b00000000;
  user_program[5] = 0b00000000;
  user_program[6] = 0b00000000;
  user_program[7] = 0b00000000;
  
  Serial.println("Translate...");
  translate();
  Serial.println("Translated program:");
  showTranslatedProgram();
  
  runSpeed = C88_CONFIG_SPEED_SLOW;
  isRun = true;
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

void translate(){
  int i;
  int curThumbOffset = 0;
  for (i =0; i<= 7; i++){
    uint8_t thisC88Instr   = user_program[i] & 0b11111000;
    uint8_t thisC88Operand = user_program[i] & 0b00000111;
    uint16_t thisThumbInstr = encode_thumb_16(THUMB_NOP);
    thumb_offsets[i] = curThumbOffset;
    if (thisC88Instr == C88_INC){
      Serial.println("INC");
      // INC | Increment register
      thisThumbInstr = encode_thumb_16(THUMB_ADD_imm_2, ARM_R0, 1); // R0 <= R0 + 1
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
    }
    if (thisC88Instr == C88_DEC){
      Serial.println("DEC");
      // DEC | Decrement register
      thisThumbInstr = encode_thumb_16(THUMB_SUB_imm_2, ARM_R0, 1); // R0 <= R0 - 1
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
    }
    if (thisC88Instr == C88_LOAD){
      Serial.println("LOAD");
      // LOAD a | Load contents of memory at a to register
      thisThumbInstr = encode_thumb_16(THUMB_LDRB_imm_1, ARM_R0, ARM_R3, thisC88Operand); // R0 <= mem[a]
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
    }

    if ((thisC88Instr == C88_ADD) || (thisC88Instr == C88_ADDU)){
      Serial.println("ADD[U]");
      // ADD a | Add contents of memory at a to register
      thisThumbInstr = encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand); // R4 <= mem[a]
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      thisThumbInstr = encode_thumb_16(THUMB_ADD_1, ARM_R0, ARM_R0, ARM_R4); // R0 <= R0 + R4
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
    }

    if ((thisC88Instr == C88_SUB) || (thisC88Instr == C88_SUBU)){
      Serial.println("SUB[U]");
      // SUB a | Subtract contents of memory at a to register
      thisThumbInstr = encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand); // R4 <= mem[a]
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      thisThumbInstr = encode_thumb_16(THUMB_SUB_1, ARM_R0, ARM_R0, ARM_R4); // R0 <= R0 + R4
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
    }

    if (thisC88Instr == C88_SHL){
      Serial.println("SHL");
      // SHL a | Shift left by immediate value
      thisThumbInstr = encode_thumb_16(THUMB_LSL_imm_1, ARM_R0, ARM_R0, thisC88Operand); // R0 <= R0 << a
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
    }

    if (thisC88Instr == C88_SHR){
      Serial.println("SHR");
      // SHR a | Shift right by immediate value
      thisThumbInstr = encode_thumb_16(THUMB_LSR_imm_1, ARM_R0, ARM_R0, thisC88Operand); // R0 <= R0 >> a
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
    }

    if (thisC88Instr == C88_ROL){
      // TODO Fix the bug that means the overflow bit is always considered to be 1
      Serial.println("ROL");
      // ROL a | Rotate left by immediate value
      // There is no direct mapping to a thumb instruction here.
      // We need to shift, extract the MSB+1 bit, shift it to the LSB, then OR it in.
      thisThumbInstr = encode_thumb_16(THUMB_LSL_imm_1, ARM_R0, ARM_R0, thisC88Operand); // R0 <= R0 << a
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      thisThumbInstr = encode_thumb_16(THUMB_MOV_imm_1, ARM_R4, 1); // R4 <= 1
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      thisThumbInstr = encode_thumb_16(THUMB_LSL_imm_1, ARM_R4, ARM_R4, 8); // R4 <= R4 << 8 (R4 <= 512)
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      thisThumbInstr = encode_thumb_16(THUMB_AND_1, ARM_R4, ARM_R0); // R4 <= R4 & R0
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      thisThumbInstr = encode_thumb_16(THUMB_LSR_imm_1, ARM_R4, ARM_R4, 8); // R4 <= R4 >> 8
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      thisThumbInstr = encode_thumb_16(THUMB_ORR_1, ARM_R0, ARM_R4); // R0 <= R4 | R0
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      
      /* LSL  R0, R0, #a
       * MOV  R4, #1
       * LSL  R4, R4, #8
       * AND  R4, R0
       * LSR  R4, R4, #8
       * ORR  R0, R4
       */
    }

    if ((thisC88Instr == C88_TSG)||
        (thisC88Instr == C88_TSL)||
        (thisC88Instr == C88_TSE)||
        (thisC88Instr == C88_TSI)){
      Serial.println("TSx");
      // TSG a | Skip the next instruction if mem[a] is greater than the register
      thisThumbInstr = encode_thumb_16(THUMB_LDRB_imm_1, ARM_R4, ARM_R3, thisC88Operand); // R4 <= mem[a]
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      thisThumbInstr = encode_thumb_16(THUMB_CMP_1, ARM_R4, ARM_R0); // flags = flags_for("R4 - R0")
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
      // Just like the JMP instruction, this will need patching as we don't
      // know how far to jump yet
      translated_program[curThumbOffset>>1] = encode_thumb_16(THUMB_NOP);
      thumb_b_patch_points[i] = curThumbOffset;
      curThumbOffset += 2;
    }

    // For debugging (slower run speeds) insert supervisor calls after each instruction.
    if (runSpeed != C88_CONFIG_SPEED_FULL){
      Serial.println("t-SVC (Speed limiter)");
      thisThumbInstr = encode_thumb_16(THUMB_SVC_1, SVC_NUMBER(SVC_REGULATE_SPEED)); // (Supervisor call)
      translated_program[curThumbOffset>>1] = thisThumbInstr;
      curThumbOffset += 2;
    }

    if (thisC88Instr == C88_JMP){
      Serial.println("JMP");
      // JMP a | Jump to address a.
      // The branch address (in the thumb program) might not be known yet
      // This code just allocates space for the jump, which will be patched up later.
      // We just need one sixteen bit instruction of space
      translated_program[curThumbOffset>>1] = encode_thumb_16(THUMB_NOP);
      thumb_b_patch_points[i] = curThumbOffset;
      curThumbOffset += 2;
    }
  }

  // Insert a jump at the end of the program, to get back to the start. To simulate the wrap-around behaviour.
  int targetOffset = thumb_offsets[0]; // Should always be zero
  int relativeJump = targetOffset - curThumbOffset;
  Serial.println("t-B (loop to zero)");
  translated_program[curThumbOffset>>1] = encode_thumb_16(THUMB_B_2, relativeJump); // PC <= PC + relativeJump
  curThumbOffset += 2;

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
      translated_program[patch_offset>>1] = encode_thumb_16(THUMB_B_2, relativeJump); // PC <= PC + relativeJump
      curThumbOffset += 2;
    }
    if (thisC88Instr == C88_TSG){
      Serial.println("Patching TSG");
      // TSG a | Skip the next instruction if mem[a] is greater than the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      translated_program[patch_offset>>1] = encode_thumb_16(THUMB_B_1, THUMB_COND_GT, relativeJump); // If flags[GT] then PC <= PC + relativeJump
      curThumbOffset += 2;
    }
    if (thisC88Instr == C88_TSL){
      Serial.println("Patching TSL");
      // TSL a | Skip the next instruction if mem[a] is less than the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      translated_program[patch_offset>>1] = encode_thumb_16(THUMB_B_1, THUMB_COND_LT, relativeJump); // If flags[LT] then PC <= PC + relativeJump
      curThumbOffset += 2;
    }
    if (thisC88Instr == C88_TSE){
      Serial.println("Patching TSE");
      // TSE a | Skip the next instruction if mem[a] is equal to the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      translated_program[patch_offset>>1] = encode_thumb_16(THUMB_B_1, THUMB_COND_EQ, relativeJump); // If flags[Z] then PC <= PC + relativeJump
      curThumbOffset += 2;
    }
    if (thisC88Instr == C88_TSI){
      Serial.println("Patching TSI");
      // TSI a | Skip the next instruction if mem[a] is equal to the register
      int targetOffset = thumb_offsets[(i+2)%8];
      int patch_offset = thumb_b_patch_points[i];
      int relativeJump = (targetOffset - patch_offset -3)>>1;
      translated_program[patch_offset>>1] = encode_thumb_16(THUMB_B_1, THUMB_COND_NE, relativeJump); // If !flags[Z] then PC <= PC + relativeJump
      curThumbOffset += 2;
    }
  }

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
    "  bx   lr       \n"
  );
  // IMPORTANT NOTE: R0, R1 and R3 are not touched by this code.
  // By making use of the calling convention, these registers allready contain
  // the values that the translated program expects.
}

uint32_t c88_reg = 0;

volatile void dbt_loop(){
  //Serial.println("Translated program:");
  //showTranslatedProgram();
  Serial.println("Running program");
  
  volatile uint32_t R0 = c88_reg;
  volatile uint32_t R1 = 0;
  volatile uint16_t *target = translated_program + translated_pc_offset;
  volatile uint8_t  *originalProgram = user_program;
  Serial.print("From address: 0x");Serial.println((uint32_t)target, HEX);
  // This function will call the translated program after setting the registers
  call_translated_program(R0, R1, target, originalProgram);
  //exit(1);
  asm("":::"lr", "r2", "r3", "r4");// does this work? if so why?
  // Now we need to extract the R0 and R1 values before anything else has a chance to
  // clobber them. This is probably safe, as nothing should happen after the translated
  // program returns before we do this.
  // This wastes registers a little, but it's the easiest way.
  asm(
    "mov %[out_r0], r0 \n"
    "mov %[out_r1], r1 \n"
    : [out_r0] "=r" (R0), [out_r1] "=r" (R1)
    ::
  );

  c88_reg = R0;
  
  Serial.print("R0: "); Serial.println(R0);
  Serial.print("R1: "); Serial.println(R1);
  delay(1000);

  if (c88_reg > 256){
    exit(1);
  }
}

void loop() {
  dbt_loop();
}
