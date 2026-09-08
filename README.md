# Maze Generator & Solver with Joystick Control

A C++ application that generates random mazes, finds an optimal solution using A*, and allows the user to navigate the maze using a physical joystick connected through an Arduino.

The project combines maze-generation and pathfinding algorithms with interactive graphics and hardware input.

## Features

- Random maze generation using Prim's algorithm
- A* pathfinding for finding an optimal solution
- Interactive maze visualization using SFML
- Physical joystick control through an Arduino
- Serial communication between the Arduino and host application
- Support for different maze grid sizes

## Tech Stack

- **C++20** — core application and algorithms
- **SFML** — graphics, window management, and rendering
- **Arduino** — joystick input
- **Serial communication** — communication between the Arduino and C++ application
- **CMake** — project configuration and build system

## Project Structure

```text
Maze-Generator/
├── arduino/
│   └── Analog_Joystick.ino
├── include/
│   ├── gameLogic.h
│   ├── joystick.h
│   ├── mazeGenerator.h
│   └── render.h
├── src/
│   └── mazeGenerator.cpp
├── .env.example
├── .gitignore
├── CMakeLists.txt
└── README.md
```

The project is divided into modular components for maze generation, game logic, joystick communication, and rendering.

## Arduino Setup

The Arduino sketch used for joystick input is located at:

```text
arduino/Analog_Joystick.ino
```

Upload the sketch to your Arduino board using the Arduino IDE and connect the Arduino to your computer over USB.

The C++ application reads the joystick state from the Arduino through a serial connection and uses that input to control movement through the maze.

## Installation and Build

The project is designed primarily for Linux and WSL2.

### 1. Install System Dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake libsfml-dev libserialport-dev
```

### 2. Build the Project

From the project root:

```bash
mkdir -p build
cd build
cmake ..
cmake --build .
```

### 3. Run the Application

From the `build` directory:

```bash
./maze_app
```

## Arduino USB Setup for WSL2

WSL2 may not automatically detect an Arduino connected to Windows through USB.

If the application reports:

```text
No valid ports found!
```

use the following steps to expose the Arduino USB device to WSL.

### 1. Install usbipd-win

Open Windows PowerShell as Administrator and run:

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

### 2. Find the Arduino USB Device

```powershell
usbipd list
```

Find the Arduino or USB Serial Device in the list and note its `BUSID`.

### 3. Bind the Arduino

From an Administrator PowerShell:

```powershell
usbipd bind --busid 1-2
```

Replace `1-2` with the BUSID reported by `usbipd list`.

Binding normally only needs to be performed once.

### 4. Attach the Arduino to WSL

```powershell
usbipd attach --wsl --busid 1-2
```

Again, replace `1-2` with the appropriate BUSID.

### 5. Verify the Arduino Inside WSL

From the WSL terminal:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

A connected Arduino will typically appear as:

```text
/dev/ttyACM0
```

### 6. Check Serial Port Permissions

If the serial device exists but the application cannot open it, verify that your Linux user has permission to access the serial device.

## WSL2 USB Note

`usbipd bind` is persistent, but attaching the device to WSL may need to be repeated after:

- Restarting WSL
- Rebooting Windows
- Disconnecting and reconnecting the Arduino

To reattach the device:

```powershell
usbipd attach --wsl --busid <BUSID>
```

## Recent Updates

### Code Quality

The original application logic has been refactored into modular components to improve readability, maintainability, and separation of responsibilities:

- Maze generation
- Maze and gameplay logic
- Joystick input/output
- SFML rendering

## Planned Updates

### Interactive Interface

- Add a UI overlay containing a timer, move counter, and restart option
- Display the player's current path progress compared with the optimal path

### Gameplay

- Add difficulty levels using different maze sizes and generation configurations
- Add optional keyboard controls as a fallback when a joystick is unavailable

### Code Quality

- Add automated tests
- Add continuous integration builds using GitHub Actions

## About the Project

This project explores the integration of algorithms, graphics programming, and physical hardware in a single C++ application.

The maze is procedurally generated using Prim's algorithm, while A* is used to determine an optimal path through the generated maze. SFML provides the graphical visualization, and an Arduino-based joystick provides physical user input through serial communication.