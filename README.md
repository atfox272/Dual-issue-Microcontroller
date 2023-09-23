# Dual-core-Microcontroller
Dual-core Microcontroller ver1.0
## 1. Tổng quan:
- Ở đề tài này, nhóm em sẽ làm 1 con Microcontroller sử dụng tập lệnh RISC-V đơn giản gồm 2 nhân và các ngoại vi cần thiết (gồm ngoại vi giao tiếp và I/O)
### a. Sơ đồ khối:

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/3b57f4e2-6f29-4d72-ac94-0bab97081b08)


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
  + mul instruction (R-type & I-type)
  + and/or/xor/shift instruction (R-type & I-type)
  + load/store instruction  (BYTE - WORD - DOUBLEWORD)
  + conditional branch instruction 
  + jump instruction

## 4. Idea notation:

### a. Processor_1:

#### i. State:
* Main processor:

* ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/86486bad-80e2-4063-bea4-7710ab93bb42)

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

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/8ab3aabd-1734-4026-9752-ac8f46c91219)
![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/9ce337f2-aefb-44de-8051-cc6f21fbfcc9)


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


### . Data memory

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/4523388c-3585-4c10-9a91-1477672bba95)
  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/a73ebcbb-d61f-45e9-8fe4-3ec39e6e450d)


### . UART_1:
#### a. UART_1:
- RX module (Just use for Programming device):
  + Send signal (RX_flag) to Processor_1 directly (disable _Internal FIFO_)

        parameter RX_FLAG_CONFIG = 0;

  + **Can't configure module** (only 8N1_9600)
  + Just enable in _program mode_
- TX module (Debugger)
  + Enable via _DEBUGGER_ register
  + 
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
