# Maze Generator & Solver with Joystick Control

## A C++ project that:

   # Generates a random maze using Prim’s algorithm 
    
   # Finds the optimal path using A*
    
   # Displays the maze and path with SFML
    
   # Lets the user traverse the maze with a joystick via an Arduino serial interface


## Features : 

   # Random maze generation on any grid size
    
   # A* pathfinding for optimal solution
    
   # Interactive rendering with SFML
    
   # Hardware joystick integration using Arduino


## Tech Stack : 

   # C++20
    
   # SFML (graphics, input, rendering)
    
   # Arduino (serial communication for joystick input)
   
   # The Arduino sketch used for joystick input is in [`arduino/Analog_Joystick.ino`]                (arduino/Analog_Joystick.ino).

   # Upload it to your Arduino board using the Arduino IDE, then connect over USB. The PC program will read joystick input from the serial port.


## Installation and Build (WSL/Linux) :

   ### This project is optimized for Linux and WSL2. Follow these steps to set up your environment and compile the application

   # 1. install system dependencies
   ### sudo apt update && sudo apt install -y build-essential cmake libsfml-dev libserialport-dev

   # 2. Build the project
   ### mkdir -p build
   ### cd build
   ### cmake ..
   ### cmake --build .

   # 3. Run the application
   ### ./maze_app

   ## Arduino USB Setup (WSL2)

   ### WSL2 may not automatically detect an Arduino connected through USB.

   ### If the application displays "No valid ports found!", follow these steps.

   # 1. Install usbipd-win from Windows PowerShell (Administrator)
   ### winget install --interactive --exact dorssel.usbipd-win

   # 2. Find the Arduino USB device
   ### usbipd list
   ### Look for a USB Serial Device or Arduino device and note its BUSID.

   # 3. Bind the Arduino to usbipd (Administrator)
   ### usbipd bind --busid 1-2
   ### Replace 1-2 with your Arduino's BUSID.This normally only needs to be done once.

   # 4. Attach the Arduino to WSL
   ### usbipd attach --wsl --busid 1-2

   # 5. Verify the Arduino is visible inside WSL
   ### ls /dev/ttyACM* /dev/ttyUSB*
   ### A connected Arduino will typically appear as: /dev/ttyACM0

   # 6. If the serial port exists but cannot be opened, check permissions groups

   ## Note:
   ### usbipd bind is persistent, but the WSL attachment may need to be repeated after restarting WSL, rebooting Windows, or reconnecting the Arduino: usbipd attach --wsl --busid <BUSID>

## Recent Updates:

 ### Code Quality
  
   # Refactored into modular components to improve readability (maze generation, maze logic, joystick input/output, rendering)   


## Planned Updates : 

  ### Interactive Interface
    
   # Add a simple UI overlay (timer, move counter, restart option)
      
   # Display current path progress vs. optimal path
  
  ### Gameplay Features
  
   # Difficulty levels (different maze sizes, generation styles)
      
   # Optional keyboard controls as a fallback to the joystick
      
  ### Code Quality
     
   # Add automated tests and CI builds via GitHub Actions


## How AI should help:

   # DO NOT PRODUCE OR UPDATE CODE

   # You are my personal mentor and are VERY invested in my learning and understanding

   # Help enforce best coding practices and standards

   # Propose fixes to help optimize code base

   # Propose additional features to improve user experience
  
