# STM32 Maze Car

A pathfinding robot car based on the **STM32F103VET6** microcontroller,
developed with the **STM32 VS Code Extension** and a CMake build system.

---

## Features

| Feature | Details |
|---------|---------|
| MCU | STM32F103VET6 (Cortex-M3, 72 MHz, 512 KB Flash, 64 KB SRAM) |
| Motor driver | L298N dual H-bridge; 20 kHz PWM via TIM3 |
| Obstacle sensing | 1 × HC-SR04 on steering servo (left/front/right scan) |
| Path finding | **Right-hand rule** (default) or **Flood-fill** (compile-time selection) |
| Wall following | PID controller trims left/right wheel speeds |
| Debug output | USART1 at 115 200 baud via USB–UART adapter |
| IDE | VS Code + STM32 VS Code Extension + CMake / Ninja |
| Programmer | ST-Link V2 (SWD) via OpenOCD or `st-flash` |

---

## Hardware Wiring

### Pin Map

| Function | MCU Pin | Peripheral |
|----------|---------|------------|
| Left motor PWM | **PA6** | TIM3\_CH1 (AF push-pull) |
| Right motor PWM | **PA7** | TIM3\_CH2 (AF push-pull) |
| Left motor IN1 | **PB0** | GPIO output |
| Left motor IN2 | **PB1** | GPIO output |
| Right motor IN3 | **PB10** | GPIO output |
| Right motor IN4 | **PB11** | GPIO output |
| US trigger | **PC0** | GPIO output |
| US echo | **PC1** | GPIO input (pull-down) |
| US servo PWM | **PB6** | TIM4\_CH1 (AF push-pull, 50 Hz) |
| Debug TX | **PA9** | USART1\_TX |
| Debug RX | **PA10** | USART1\_RX |
| Status LED | **PE0** | GPIO output (active-high) |
| HSE crystal | PH0 / PH1 | 8 MHz external oscillator |

### L298N Wiring

```
STM32           L298N Module
──────          ────────────
PA6  ──────►  ENA  (left  motor speed)
PA7  ──────►  ENB  (right motor speed)
PB0  ──────►  IN1  (left  motor dir)
PB1  ──────►  IN2  (left  motor dir)
PB10 ──────►  IN3  (right motor dir)
PB11 ──────►  IN4  (right motor dir)
5 V  ──────►  +5V  (logic supply)
GND  ──────►  GND
             OUT1/OUT2 → Left  motor
             OUT3/OUT4 → Right motor
```

### HC-SR04 Wiring (×1, servo-mounted)

```
HC-SR04      STM32
───────      ──────
VCC    ──►  3.3–5 V
GND    ──►  GND
TRIG   ──►  PC0
ECHO   ──►  PC1  (use a 5 V→3.3 V voltage divider on ECHO!)

Servo (steering mount)   STM32
──────────────────────   ──────
VCC                 ──►  5 V (external recommended)
GND                 ──►  GND (common ground with STM32)
PWM signal          ──►  PB6 (TIM4_CH1, 50 Hz)
```

> ⚠️ The HC-SR04 ECHO line is 5 V logic. Protect the STM32 GPIO with a
> voltage divider (e.g. 1 kΩ + 2 kΩ) or a level-shifter module.

---

## Software Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        main.c                               │
│  SystemClock_Config → peripheral init → MazeSolver loop     │
└───────────────────────┬─────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────────┐
        ▼               ▼                   ▼
  ┌───────────┐  ┌─────────────┐  ┌──────────────────┐
  │  motor.c  │  │ultrasonic.c │  │  maze_solver.c   │
  │ L298N PWM │  │ HC-SR04 +   │  │ Right-hand rule  │
  │  control  │  │ servo scan  │  │  or Flood-fill   │
  └─────┬─────┘  └──────┬──────┘  └────────┬─────────┘
        │               │                  │
        ▼               ▼                  ▼
  ┌───────────┐  ┌─────────────┐  ┌──────────────────┐
  │  tim.c    │  │   tim.c     │  │     pid.c        │
  │ TIM3/TIM4 │  │ TIM2 1 MHz  │  │  wall-following  │
  │ PWM       │  │ timing      │  │                  │
  └───────────┘  └─────────────┘  └──────────────────┘
```

### Maze-Solving Algorithms

#### Right-Hand Rule (default)

The car keeps its right "hand" touching the right wall at all times.

Decision priority at each cell:
1. **Right open** → turn right, move forward.
2. **Front open** → go straight.
3. **Left open** → turn left, move forward.
4. **All blocked** → U-turn (180°), move forward.

Works for any *simply-connected* (loop-free) maze. O(1) memory.

#### Flood-Fill (optional)

Define `MAZE_ALGO_FLOOD_FILL` in CMakeLists.txt to enable:

```cmake
add_compile_definitions(MAZE_ALGO_FLOOD_FILL)
```

A 16 × 16 distance map is maintained in SRAM. BFS propagates from the
goal outward; the car always moves toward the neighbour with the smallest
flood value.  Wall knowledge is built incrementally as the car explores.
Always finds the shortest path.

#### PID Wall-Following

While driving forward the car uses a PID controller (Kp = 1.8, Ki = 0.05,
Kd = 0.8) to maintain a 12 cm gap from the side wall, correcting left/right
wheel speeds in real-time.

---

## Project Structure

```
STM32-Maze-Car/
├── App/
│   ├── Inc/
│   │   ├── maze_solver.h   – maze-solving algorithm interface
│   │   ├── motor.h         – DC motor control interface
│   │   ├── pid.h           – generic discrete PID controller
│   │   └── ultrasonic.h    – HC-SR04 + servo-scan driver interface
│   └── Src/
│       ├── maze_solver.c   – right-hand rule + flood-fill implementation
│       ├── motor.c         – L298N H-bridge + TIM3 PWM driver
│       ├── pid.c           – PID controller
│       └── ultrasonic.c    – servo positioning + HC-SR04 measurement
├── Core/
│   ├── Inc/
│   │   ├── gpio.h
│   │   ├── main.h          – pin definitions
│   │   ├── stm32f1xx_hal_conf.h
│   │   ├── stm32f1xx_it.h
│   │   ├── tim.h
│   │   └── usart.h
│   └── Src/
│       ├── gpio.c          – GPIO initialisation for all pins
│       ├── main.c          – entry point, clock config, main loop
│       ├── stm32f1xx_hal_msp.c
│       ├── stm32f1xx_it.c  – exception handlers
│       ├── system_stm32f1xx.c
│       ├── tim.c           – TIM2 (1 MHz) + TIM3/TIM4 PWM
│       └── usart.c         – USART1 115200 baud + printf redirect
├── Drivers/                – STM32CubeF1 HAL (see Installation below)
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
├── Startup/
│   └── startup_stm32f103vetx.s  – Cortex-M3 reset + vector table
├── .vscode/
│   ├── c_cpp_properties.json
│   ├── launch.json         – OpenOCD debug configuration
│   └── tasks.json          – build / flash tasks
├── cmake/
│   └── gcc-arm-none-eabi.cmake  – toolchain file
├── CMakeLists.txt
├── STM32F103VETx_FLASH.ld  – linker script (512 KB Flash / 64 KB SRAM)
└── STM32F103VETx.ioc       – STM32CubeMX project file
```

---

## Prerequisites

| Tool | Download |
|------|---------|
| arm-none-eabi-gcc (≥ 10) | https://developer.arm.com/downloads/-/gnu-rm |
| CMake (≥ 3.22) | https://cmake.org/download/ |
| Ninja | https://ninja-build.org/ |
| OpenOCD (≥ 0.12) | https://openocd.org/ |
| ST-Link V2 drivers | https://www.st.com/en/development-tools/stsw-link009.html |
| VS Code | https://code.visualstudio.com/ |
| STM32 VS Code Extension | VS Code marketplace: *STM32 VS Code Extension* |
| STM32CubeMX | https://www.st.com/en/development-tools/stm32cubemx.html |

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/wowwowww/STM32-Maze-Car.git
cd STM32-Maze-Car
```

### 2. Install the STM32CubeF1 HAL drivers

Open **STM32CubeMX**, load `STM32F103VETx.ioc`, then click
**Project → Generate Code**.  The `Drivers/` folder will be populated
automatically with the correct CMSIS and HAL files.

Alternatively, download STM32CubeF1 manually and copy the two directories:

```bash
# Example using the STM32CubeF1 package
cp -r ~/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.5/Drivers/CMSIS       Drivers/
cp -r ~/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.5/Drivers/STM32F1xx_HAL_Driver Drivers/
```

### 3. Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j4
```

Or use the VS Code **Tasks** (`Ctrl+Shift+B` → *CMake: Build*).

### 4. Flash

**Via OpenOCD:**

```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
        -c "program build/STM32-Maze-Car.elf verify reset exit"
```

**Via st-flash:**

```bash
st-flash --reset write build/STM32-Maze-Car.bin 0x8000000
```

Or use the VS Code Task *Flash (OpenOCD + ST-Link)*.

### 5. Debug

1. Connect the ST-Link V2 to the SWD header (`SWDIO`, `SWCLK`, `GND`, `3.3 V`).
2. In VS Code open the **Run and Debug** panel (`Ctrl+Shift+D`).
3. Select *Debug (OpenOCD + ST-Link)* and press **F5**.

---

## Configuration

Key constants are defined in the headers and can be tuned without
recompiling the full project:

| Constant | File | Default | Description |
|----------|------|---------|-------------|
| `MOTOR_BASE_SPEED` | `motor.h` | 60 % | Cruise speed |
| `MOTOR_TURN_SPEED` | `motor.h` | 50 % | Speed during turns |
| `MAZE_CELL_DURATION_MS` | `maze_solver.h` | 600 ms | Time to cross one cell |
| `MAZE_TURN_DURATION_MS` | `maze_solver.h` | 500 ms | Time for a 90° turn |
| `MAZE_UTURN_DURATION_MS` | `maze_solver.h` | 1000 ms | Time for a 180° U-turn |
| `US_WALL_THRESHOLD_CM` | `ultrasonic.h` | 20 cm | Wall detection threshold |
| `WALL_SETPOINT_CM` | `maze_solver.c` | 12 cm | PID target side-wall gap |
| `PID_KP / KI / KD` | `maze_solver.c` | 1.8 / 0.05 / 0.8 | Wall-following gains |

To select the flood-fill algorithm add to `CMakeLists.txt`:

```cmake
add_compile_definitions(MAZE_ALGO_FLOOD_FILL)
```

---

## Serial Debug Output

Connect a USB–UART adapter to **PA9 (TX)** and **GND**, open a terminal at
**115 200 8N1**, and monitor the solver's decisions in real time:

```
==============================
  STM32 Maze Car  v1.0
  SYSCLK = 72 MHz
==============================

[MAZE] Initialised. Algorithm: Right-hand rule
[INFO] Starting maze solver...
[MAZE] Start detected – entering maze.
[MAZE] Straight ahead
[MAZE] Turn RIGHT
[MAZE] Straight ahead
[MAZE] Turn LEFT
...
```

---

## License

This project is released under the **MIT License**.

The STM32CubeF1 HAL library (Drivers/) is provided by STMicroelectronics
under a modified BSD license; refer to the header of each driver file.
