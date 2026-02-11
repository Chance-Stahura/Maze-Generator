Maze Generator & Solver with Joystick Control

A C++ project that:

    Generates a random maze using Prim’s algorithm 
    
    Finds the optimal path using A*
    
    Displays the maze and path with SFML
    
    Lets the user traverse the maze with a joystick via an Arduino serial interface



Features : 

    Random maze generation on any grid size
    
    A* pathfinding for optimal solution
    
    Interactive rendering with SFML
    
    Hardware joystick integration using Arduino

Tech Stack : 

    C++20
    
    SFML (graphics, input, rendering)
    
    Arduino (serial communication for joystick input)
            The Arduino sketch used for joystick input is in [`arduino/joystick_control.ino`]                (arduino/joystick_control.ino).

            Upload it to your Arduino board using the Arduino IDE, then connect over USB.  
            The PC program will read joystick input from the serial port.



Installation and Build (WSL/Linux) :
    This project is optiomized for Linux and WSL2. Follow these steps to set up your environment and compile the application

        1. install system dependencies
            sudo apt update && sudo apt install -y build-essential cmake libsfml-dev libserialport-dev

        2. Build the project
            mkdir -p build
            cd build
            cmake ..
            cmake --build .

        3. Run the application
            ./maze_app



Planned Updates : 

  Interactive Interface
    
      Add a simple UI overlay (timer, move counter, restart option)
      
      Display current path progress vs. optimal path
  Gameplay Features
  
      Difficulty levels (different maze sizes, generation styles)
      
      Optional keyboard controls as a fallback to the joystick
  Code Quality
  
      Refactor into modular components (algorithms, rendering, input)
      
      Add automated tests and CI builds via GitHub Actions
  
