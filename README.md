# Gimbal Project

<div style="font-size:16px"><span style="float:right">By: Yujie Zang 23/11/2024 </div>

---
## Task1: Construct a System Framework

Here’s a framework to implement the described functionality, designed with modularity, clarity, and extensibility in mind. This architecture is flexible and supports incremental optimization as the project progresses.

### Step1: Interrupt Handlers to Receive DUBS and CAN Feedback

#### 1.1 CAN

Using CAN to receive the feedback of Motor, Attention to the choice of Pin for different chip.

<img src=".\Materials\P1.png" alt="P1" style="zoom:67%;" /> 

#### 1.2 DBUS

Using DBUS (like UART) to receive the instruction from remote control. Attention to Baud_Rate, World Length and Panty which are related to the other board setting. PIN is also needed to check to the schematic.

<img src=".\Materials\P2.png" alt="P2" style="zoom:67%;" />

### Step 2: IWDG Timer

Use an independent watchdog with a timeout period of 1.5 seconds to ensure system safety. Attention to the function to calculate the timeout.

<img src=".\Materials\P3.png" alt="P3" style="zoom:80%;" />

### Step 3:Main Loop Execution

Triggered by a **Timer interrupt every 1ms.** Attention to the function to calculate the period

<img src=".\Materials\P5.png" alt="P5" style="zoom:67%;" />

<img src=".\Materials\P4.png" alt="P4" style="zoom:67%;" />

Tasks include:

- Compute target values based on remote control inputs.
- Calculate motor control outputs using PID.
- Send motor control frames via CAN communication.