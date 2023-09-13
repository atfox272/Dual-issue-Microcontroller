# Dual-core-Microcontroller
Dual-core Microcontroller ver1.0
## 1. Tổng quan:
- Ở đề tài này, nhóm em sẽ làm 1 con Microcontroller sử dụng tập lệnh RISC-V đơn giản gồm 2 nhân và các ngoại vi cần thiết (gồm ngoại vi giao tiếp và I/O)
### a. Sơ đồ khối:

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/c12f0b50-1c50-4cd7-8e85-9e6de20613f0)

### b. Chức năng các khối
#### i. Multi-processor Manager:
- Phân phối instruction vào Processor đang trong trạn thái rỗi (IDLE_STATE)
- Kiểm soát dữ liệu trong câu lệnh tiếp thep có nguy cơ bị out-dated khi xử lý song song hay không

#### ii. Synchronization Primitive:
- Đông bộ việc xử lý song song (parallel mamnagement)
  + Cơ chế chống đụng độ khi truy xuất main memory : _mutual exclusion_
- Quản lý việc thanh ghi bên trong Processor này là **new data** (dữ liệu mới nhất được load vào register)

#### ii. Processor 1:
- Nạp chương trình (Program device)
- Quản lý I/O
- Debugger (via UART_1)
- Xử lý lệnh 

#### iii. Processor 2:
- Quản lý các ngoại vi truyền thông (UART_2, SPI, I2C)
- Xử lý lệnh

#### iv. I/O Peripheral
- Thiết lập Input/Output
- Bộ đếm cho dữ liệu input/output

#### v. Memory:
- Chi tiết trong phần <Interface between Processor and Memory & Memory structure>

## 2. Interface between Processor and Memory & Memory structure :
- Memory Hierachy :

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/fe1c6162-6781-4c77-b61f-daee985725db)

- Interface between Processor and Memory:

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/70110588-7eb1-4edc-8ca1-bdd662dd2ab5)

## 3. Tính khả thi của đề tài đối với thời gian làm đồ án:

- Ở đây vì để tài có thể **phát sinh thời gian khá lớn** nên nhóm bọn em sẽ hạ số lượng lệnh mà processor phải xử lý xuống, gồm các lệnh sau đây:
  + add instruction 
  + sub instruction 
  + mul instruction
  + load/store instruction
  + conditional branch instruction
  + jump instruction
  + and/or/xor/shift instruction

- Từ các các lệnh đơn giản trên processor thiết lập thanh ghi và gửi tín hiệu đến các ngoại vi để xử lý các tác vụ yêu cầu
  
- Ngoài ra về các ngoại vi thì bên em đã có thiết kế trước các ngoại vi như UART, SPI, RAM (main memory), I2C, nên khối lượng còn lại chỉ là xử lý hành vi của **Processor**, **Multi-processor Manager** và **Synchronization Primitive** (_như hình_)

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/4edd43f2-d21f-4b0e-a8ab-fd0a56db43da)

## 3. Processor:

### Register
#### Special register in Processor_1
  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/9c48abcc-c7b3-4325-964a-9f000d7d6758)


#### Special register in Processor_2
  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/67869342-c0d0-47a4-90cf-2ff1088c409a)

## 4. UART_1:
### a. UART_1:
- RX module (Just use for Programing device):
  + Send signal (RX_flag) to Processor_1 directly (disable _Internal FIFO_)

        parameter RX_FLAG_CONFIG = 0;

  + **Can't configure module** (only 8N1_9600)
  + Just enable in _program mode_
- TX module (Debugger)
  + Enable via _DEBUGGER_ register
 
## 4. Optional Unit:
- Timer Unit
- Interrupt Unit

## 5. Modify main flow:

  _(Sep 11, 2023)_
  - Change architecture (from _Von Neumann Architecture_ to _Harvard Architecture_) 
    + **Description**: Seperate _Main memory_ into 2 block (_Program memory_ & _Data memory_)
    + **Goal**: To Improve performence of Parallel Computing (Loading instruction is not depended on Loading data)

  _(Sep 12, 2023)_
  - Change Interface: modify mode_controller (program mode or running mode) from user to processor  
    + **Description**: In INIT_STATE, MCU is in _Program mode_, then User will load code (bitstream file) via UART_1. Processor must detect _terminated instruction_ and change MCU's mode (from Program mode to Running mode)
    + **Goal**: MCU change mode automatically when it detect _terminated instruction_

  
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
