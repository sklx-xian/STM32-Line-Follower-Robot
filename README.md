# STM32-Line-Follower-Robot
STM32 Line Follower &amp; Obstacle Avoidance Robot

## Project Overview
This project is a smart line-following and obstacle-avoiding robot based on the STM32F103C8T6 microcontroller. 
Moving beyond traditional `if-else` logic, this project utilizes **Weighted Error Normalization + PD Closed-Loop Control** to achieve incredibly smooth high-speed cornering. It also integrates an HC-SR04 ultrasonic sensor for highly responsive emergency braking and reverse obstacle avoidance.

## Hardware
**Microcontroller:** STM32F103C8T6 Core Board ("Blue Pill")
 **Motor Driver:** L298N Dual H-Bridge Motor Driver
 **Chassis & Motors:** Acrylic chassis + 2 x TT DC Gear Motors
 **Line Tracking**:yahboom YB-MUX04-1.0
 **Obstacle Avoidance:** HC-SR04 Ultrasonic Distance Sensor
 **Power Supply**:14500 lithium-ion battery

 ## Core Algorithm
 - **Weighted Error Normalization:** Converts discrete digital IR sensor signals into a continuous, smooth linear error curve (-2500 to 2500). This completely eliminates the "jittering" or "oscillation" phenomenon often seen on straightaways and slight curves.
-  **PD Closed-Loop Control:** Utilizes Proportional (P) and Derivative (D) controllers to output dynamic differential PWM to the motors. The Integral (I) term was intentionally discarded to prevent system oscillation and lag, ensuring ultra-fast response times.
-   **Sharp Turn State Machine Lock:** When the outermost sensors trigger the maximum error threshold, the system forces a "tank-style pivot" mode. This state lock is only released once the robot is centered again, allowing it to perfectly conquer 90-degree and acute angle turns.
-  **Two-Stage Lost-Line Recovery:** Introduces a lost-line timer. A brief signal loss (< 150ms) triggers a gentle correction; a complete track deviation (> 150ms) triggers a strong reverse recovery maneuver based on the last known error memory.
## pin mapping
| Module | Pin Function | STM32 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **HC-SR04** | Trig  | `PA10` | Out Push Pull |
| **HC-SR04** | Echo  | `PA0` | TIM2_CH1/TIM2_CH2 |
| **L298N** | ENA | `PA8` | PWM |
| **L298N** | ENB  | `PA9` | PWM |
| **L298N** | IN1 - IN4 | `GPIO 4-7` | Out Push Pull |
| **YB-MUX04-1.0(I2C)** | SCL  | `PB8` | I2C  |
| **YB-MUX04-1.0(I2C)** | SDA  | `PB9` | I2C  |
