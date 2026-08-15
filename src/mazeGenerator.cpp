#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <string>
#include <ctime>

#include "mazeGenerator.h"
#include "joystick.h"
#include "render.h"
#include "gameLogic.h"

using namespace std;


int main() {

    graphGenerator generator;
    graphSolver solver;
    graphDisplay display;
    gameLogic logic;

    srand(time(0)); //this is to help choose a random index for the graph generator!
    //this helps choose a random wall(s) to remove inside of the prim's implementation
    //Serial.print("Serial port initialized");
    int height = 0, width = 0;
    int cellSize = 10; // Size of each cell in pixels
    int playerX = 0;
    int playerY = 0;
   
    std::cout << "Welcome to the Super Sweet Graph Solver" << endl;
    std::cout << "Please enter dimensions for graph" << endl;
    std::cout << "\n" << endl;
    std::cout << "Height : " << flush;
    std::cin >> height;
    std::cout << "Width : " << flush;
    std::cin >> width;
    std::cout << "\n" << endl;
    //cout << "Thank you for your input! here is your graph, and the associated best path!" << endl;
    std::cout.flush();

    generator.gridInitializer(height, width); 
    generator.prims(generator.grid);
    display.solver.aStarSearch(generator.grid);
    display.createWindow(generator.grid);
    if(logic.initJoystick()) { 
        while(display.isOpen()){
            display.handleEvents();
            logic.interpretDirection(generator.grid);
            playerX = logic.getPlayerX();
            playerY = logic.getPlayerY();
            display.displayGraph(generator.grid, playerX, playerY);
        }
    }
    logic.joystickCleanup();
    
    return 0;
}