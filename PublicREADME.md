# Dual-core-Microcontroller
Dual-core Microcontroller ver1.0
## 1. Tổng quan:
- Ở đề tài này, nhóm em sẽ làm 1 con Microcontroller sử dụng tập lệnh RISC-V đơn giản gồm 2 nhân và các ngoại vi cần thiết (gồm ngoại vi giao tiếp và I/O)
### a. Sơ đồ khối:

![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/00fda3c4-e28f-433b-9e3b-24a0143f0bfa)

### b. Chức năng các khối
#### i. Multi-processor Manager (Synchronization Primitive):
- Phân phối instruction vào Processor đang trong trạn thái rỗi (IDLE_STATE)
- Đông bộ việc xử lý song song (parallel mamnagement)
  + Cơ chế chống đụng độ khi truy xuất main memory : _mutual exclusion_
- Quản lý việc thanh ghi bên trong Processor này là **new data** (dữ liệu mới nhất được load vào register)
- Kiểm soát dữ liệu trong thanh ghi có nguy cơ bị out-dated khi xử lý song song hay không
  
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

#### v. Main memory:
- Chi tiết trong phần <Interface between Processor and Memory & Memory structure>

## 2. Interface between Processor and Memory & Memory structure :
- Memory Hierachy :

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/fe1c6162-6781-4c77-b61f-daee985725db)

- Interface between Processor and Memory:
  
  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/24fd89c8-7826-42cf-8822-c5d54abee516)

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
  
- Ngoài ra về các ngoại vi thì bên em đã có thiết kế trước các ngoại vi như UART, SPI, RAM (main memory), I2C, nên khối lượng còn lại chỉ là xử lý hành vi của **Processor** và **Multi-processor Manager** (_như hình_)

  ![image](https://github.com/atfox272/Dual-core-Microcontroller/assets/99324602/81e9d469-296c-43ec-afd2-d6b19e24b14d)


## 3. Optional Unit:
- Timer Unit
- Interrupt Unit
