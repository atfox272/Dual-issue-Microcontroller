# Dual-core-Microcontroller
Dual-core Microcontroller ver1.0
## 1. Tổng quan:
### a. Sơ đồ khối:

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/00fda3c4-e28f-433b-9e3b-24a0143f0bfa)

### b. Chức năng các khối
#### i. Multi-processor Manager (Synchronization Primitive):
- Đông bộ việc xử lý song song (parallel mamnagement)
  + Cơ chế chống đụng độ: _mutual exclusion_
- Phân phối câu lệnh (instruction) vào bộ xử lý (processor) tương ứng tùy vào:
  + Câu lệnh
  + Thanh ghi đích (destination register).

#### ii. Processor 1:
- Nạp chương trình (Program device)
- Quản lý I/O
- Debugger (via UART_1)
- Xử lý một số câu lệnh tương ứng Processor 1

#### iii. Processor 2:
- Quản lý các ngoại vi truyền thông (UART_2, SPI, I2C)
- Xử lý một số loại câu lệnh tương ứng Processor 2

#### iv. I/O Peripheral
- Thiết lập Input/Output

#### v. Main memory:
- Chi tiết trong phần <Interface between Processor and Memory & Memory structure>

## 2. Interface between Processor and Memory & Memory structure :
- Memory Hierachy :

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/fe1c6162-6781-4c77-b61f-daee985725db)

- Interface between Processor and Memory:
  
  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/b140daae-d70b-4a2e-aa18-c5591abcec8e)


## 3. Optional:
- Timer Unit
- Interrupt Unit
