# Testbench

## Processor 1 (main processor)
### Load program state (100%)
- Receive instruction from RX_1 module               ✔️
- Load instruction into Program memory               ✔️
- Case finish loading instruction                    ✔️
  + Time out                                         ✔️
  + Detect FINISH_INSTRUCTION_OPCODE at opcode space ✔️
# Fetch & Execute Instruction state (100%)
- Processor received instruction completely          ✔️
- Encode instruction
- Add / addi / sub / mul / mul / and / or instruction✔️
- load instruction                                   ✔️
- store instruction (unchecked)                      ➖
## Multi-processor mamanger (30%)
### Without Interrupt control & Synchronization primitive:
- Simple Arithmetic (add, sub, mul)
- J , JAL (x1 checked)
- Simple Dual-core case 
