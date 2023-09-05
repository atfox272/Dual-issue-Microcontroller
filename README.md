# Dual-core-Microcontroller
Dual-core Microcontroller ver1.0
## 1. General:
### a. Block Diagram:

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/1afb255e-3e41-4651-a099-0d9ae36852a9)

### b. Function of Blocks
1. Multi-processor Manager (Synchronization Primitive):
- Đông bộ việc xử lý song song (parallel mamnagement)
  + Cơ chế chống đụng độ: _mutual exclusion_
- Phân phối câu lệnh (instruction) vào bộ xử lý (processor) tương ứng tùy vào:
  + Câu lệnh
  + Thanh ghi đích (destination register).

2. Processor 1:
- Nạp chương trình (Program device)
- Quản lý I/O
- Debugger (via UART_1)
- Xử lý một số câu lệnh tương ứng Processor 1

3. Processor 2:
- Quản lý các ngoại vi truyền thông (UART_2, SPI, I2C)
- Xử lý một số loại câu lệnh tương ứng Processor 2

3. I/O Peripheral
- Thiết lập Input/Output

4. Main memory:
- Chi tiết trong phần <Cấu trúc bộ nhớ>

## 2. Interface between Processor and Memory & Memory structure :
- Memory Hierachy :

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/0fb794b1-fa6f-45e9-a7ff-4194a2d85e3b)

- Interface between Processor and Memory:
  
  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/bbffea01-07c4-4ec2-9cf4-5ef8ead6d501)

## 3. Optional:
- Timer Unit
- Interrupt Unit
