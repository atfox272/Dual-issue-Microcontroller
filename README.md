# Dual-core-Microcontroller
Dual-core Microcontroller ver1.0
## 1. General:
### a. Block diagram:

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/b60e30fe-ed5e-4ad9-a4c3-bc2ca97939b6)


### b. Chức năng các khối
#### i. Multi-processor Manager:
- Phân phối instruction vào Processor đang trong trạn thái rỗi (IDLE_STATE)
- Kiểm soát dữ liệu trong câu lệnh tiếp thep có nguy cơ bị out-dated khi xử lý song song hay không

#### ii. Synchronization Primitive:
- Đông bộ việc xử lý song song (parallel mamnagement)
  + Cơ chế chống đụng độ khi truy xuất main memory : _mutual exclusion_
- Quản lý việc thanh ghi bên trong Processor này là **new data** (dữ liệu mới nhất được load vào register)

#### ii. Processor 1:
- Program device
- I/O Manager
- Debugger (via UART_1)
- Execute instruction

#### iii. Processor 2:
- Quản lý các ngoại vi truyền thông (UART_2, SPI, I2C)
- Xử lý lệnh

#### iv. I/O Peripheral
- Thiết lập Input/Output
- Bộ đếm cho dữ liệu input/output

#### v. Memory:
- In <Interface between Processor and Memory & Memory structure>

## 2. Interface between Processor and Memory & Memory structure :
- Memory Hierachy :

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/fe1c6162-6781-4c77-b61f-daee985725db)

- Interface between Processor and Memory:

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/16e1149e-28bf-4a10-91d0-d6dfa7d76270)


## 3. Instruction requirement:
  + add instruction (R-type & I-type)
  + sub instruction (R-type)
  + mul instruction (R-type)
  + and/or/xor/shift instruction (R-type & I-type)
  + load/store instruction  (BYTE - WORD - DOUBLEWORD)
  + conditional branch instruction (BEQ - BNE - BLT = BGE)
  + jump instruction (J - JAL - JALR)
  + Hardware support instruction (MISC-MEM FENCE instruction)
    
## 4. Idea notation:

### a. Processor_1:

#### i. State:
* Main processor:
  
![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/6c4e1334-19dc-492a-9996-0caf2c2ce499)

#### ii. Testing case:
* LOAD_PROGRAM:
- Module in testbench:
  + Processor_1 (main processor)
  + UART_1
- Testing:
  + Behavior of interactive action between Processor and Multi-processor manager
  + Behavior of interactive action between Processor and Synchronization primitive
  + R-type/I-type instruction
  + Load and Store instruction 
### b. Multi-processor manager (MPM):

#### i. Interface:

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/6064626b-6645-4a39-b84b-af5d8831e6c3)
![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/3dddbce4-5c5f-47e8-b2e9-703f42b1a010)



#### ii. Task:

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/52fff71e-e606-4477-828e-4be7f20929f0)

### c. Synchronization Primitive (SP) :

#### i. Idea:
  - SP will **polling** "i want to access memory" signal from Processor_1 and Processor_2 (via _if_ block and Processor_1 have higher priority), then SP will give **access** to 1 Processor. After The Processor had processed and **sent signal to SP**, SP will take back the _lock_ and go on **polling**

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/0a0711d4-68f4-4dc0-8943-a56c03f0511a)
  
#### ii. Interface:
![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/38402923-92a7-4882-8aff-81557afd3be6)
![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/164e1e82-80a0-4786-8691-8f9c60993102)


#### iii. State:
##### a. Read state:
![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/54d148a5-5842-4e93-96bd-9942d0a8fea1)

##### b. Write state:
![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/68948543-20c4-4372-b0b0-ee63004a6a81)


### d. Interrupt control (INTC) :
#### i. Idea:
- Interrupt control will manage FIFO_INT_1 & FIFO_INT_2 & FIFO_INT_3
  + _interrupt_request_1_ is **write_ins** of Queue (FIFO)
  + _RETI_ is **read_ins** of Queue (FIFO)
  + _interrupt_flag_ is **!empty** of Queue (FIFO)
- This module will decide the order of 3 interrupt vectors (via interrupt_flag_1 is the highest priority interruption)

#### ii. Interface:

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/5eb8c97d-733b-476c-85be-a1e023a85813)


### . Data memory

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/b4856a86-0e39-4c5f-8b58-096444763c32)
  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/216e50bd-2e3c-4334-8826-e0898901f399)
  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/b4f3fe8d-ff7d-4bc8-9d2a-8cf5d92ba8d4)

## 4. Optional Unit:
- Timer Unit
- Interrupt Unit

## 5. Modify main flow:

  _(Sep 11, 2023)_
  - Change _Architecture_ (from _Von Neumann Architecture_ to _Harvard Architecture_) 
    + **Description**: Separate _Main memory_ into 2 block (_Program memory_ & _Data memory_)
    + **Goal**: To Improve the performance of Parallel Computing (Loading instruction is not depended on Loading data)

  _(Sep 12, 2023)_
  - Change _Interface_: modify mode_controller (program mode or running mode) from user to processor  
    + **Description**: In INIT_STATE, MCU is in _Program mode_, then User will load code (bitstream file) via UART_1. Processor must detect _terminated instruction_ and change MCU's mode (from Program mode to Running mode)
    + **Goal**: MCU change mode automatically when it detect _terminated instruction_
  
  _(Sep 14, 2023)_
  - Change _Configuartion register_:  Change location of Configuration register from _Register in Processor_ to _Data memory_
    + **Description**: Change location of Configuration register from _Register in Processor_ to _Data memory_ 
    + **Goal**: Not reserve register in Processor
    + 
  _(Sep 20, 2023)_
  - Change _Function of Multi-processor manager_: The Multi-processor manager process some instruction by itself (not only the processors)
    + **Description**: The MPM will process B-type and J-type instruction by itself (NOT fetch these instructions into the Processor)
    + **Goal**: Reduce the number of PC registers (merge all PC register into Multi-processor manager)
      
  _(Sep 21, 2023)_
  - Add _Configuartion segment in Data memoyr_: Add 1 reserved segment for GPIO (address: 0x002)
    + **Description**: Add "**VALUE PORT_N**" to save previous value or output value of I/O 
    + **Goal**:

  _(Sep 23, 2023)_
  - Add _Timer/Interrupt Function_: External / Pin Change / Timer interrupt
    + **Description**: Add External / Pin Change / Timer interrupt
    + **Goal**:

  _(Sep 28, 2023)_
  - Change _Synchronization registers automaticly_ from _Using register_renew_:
    + **Description**: Register_management (RM) block sends sync_signal to Processor 1 and Processor 2 (if RM detects that some processors have modified register_owner)
    + **Goal**:
        
## 6. Testcase:
  - Testcase for parallel processing:

        Case:
        <Instruction 1>: Instruct MCU send _data1_ with UART_1
        <Instruction 2>: Instruct MCU send _data2_ with UART_2
        Then Using LogicAnalyzer to record TX from UART_1 and UART_2

        Expect outcome:
        TX from UART_1 is working:    _____/--------------------\__________________
        TX from UART_2 is working:    ____________/----------------------\_________ 

        Case:
        <Instructoins>:  assign 2 numbers to register
                         multiply 2 numbers, then assign to regi 
        <Instruction 1>: Instruct MCU send _data1_ with UART_1
        <Instruction 2>: Instruct MCU send _data2_ with UART_2
        Then Using LogicAnalyzer to record TX from UART_1 and UART_2

        Expect outcome:
        TX from UART_1 is working:    _____/--------------------\__________________
        TX from UART_2 is working:    ____________/----------------------\_________ 
