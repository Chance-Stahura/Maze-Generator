#pragma once

#include <cmath>
#include <climits>
#include <ctime>
#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include "joystick.h"
#include "mazeGenerator.h"

//if a complete line reading is available:
    //retrieve state
    //intepret movement
struct gameLogic {

    sf::Clock joystickClock;
    joystick jstick;
    int playerX = 0, playerY = 0; // Player starting position

    bool initJoystick() {
        if(!jstick.initialize()) {
            std::cout << "Failed to initialize joystick" << std::endl;
            return false;
        }
        return true;
    }
 
    // GAME CONTROL LOGIC
    //"can the player move through the maze"
    void interpretDirection(std::vector<std::vector<cell>>& grid) {

        int height = grid.size();
        int width = grid[0].size();

        jstick.updateState();
        //if joystick position values have changed
        if(jstick.hasNewState()) {
            auto state = jstick.getState();

            //if enough time has passed in between movements
            if(/*swVal == 1 &&*/ joystickClock.getElapsedTime().asMilliseconds() > 200) {
                // Button is not pressed and enough time has passed
                const int DEADZONE = 50; // Define a deadzone for joystick movement
                const int CENTERX = 518; // Center value for X-axis
                const int CENTERY = 507; // Center value for Y-axis
                int dx = 0, dy = 0;
                static int lastdx = 0, lastdy = 0; // Store last movement direction
                if(state.x > CENTERX + DEADZONE) {
                    dx = 1; // Move right
                    std::cout << "Moving right" << std::endl;
                } else if(state.x < CENTERX - DEADZONE) {
                    dx = -1; // Move left
                    std::cout << "Moving left" << std::endl;
                } 

                if(state.y > CENTERY + DEADZONE) {
                    dy = 1; // Move down
                    std::cout << "Moving down" << std::endl;
                } else if(state.y < CENTERY - DEADZONE) {
                    dy = -1; // Move up
                    std::cout << "Moving up" << std::endl;
                }
                                        
                if(dx != 0 /*&& dx != lastdx*/ || dy != 0 /*&& dy != lastdy*/) {                                        
                    std::cout << "Movement command: dx=" << dx << ", dy=" << dy << std::endl;
                    if(dx != 0 || dy != 0) {
                        int newX = playerX + dx;
                        int newY = playerY + dy;
                        // Check bounds
                        if(newX >= 0 && newX < width && newY >= 0 && newY < height) {
                            bool validMove = true;
                            
                            // Check walls
                            if(dx == -1 && grid[playerY][playerX].leftWall) validMove = false;
                            if(dx == 1 && grid[playerY][playerX].rightWall) validMove = false;
                            if(dy == -1 && grid[playerY][playerX].topWall) validMove = false;
                            if(dy == 1 && grid[playerY][playerX].bottomWall) validMove = false;
                            // Check diagonal walls
                            if(dy == -1 && dx == -1) { // top-left
                                if(grid[playerY][playerX].topWall || grid[playerY][playerX].leftWall ||
                                    grid[playerY-1][playerX].leftWall || grid[playerY][playerX-1].topWall) {
                                    validMove = false;
                                }
                            } else if(dy == -1 && dx == 1) { // top-right
                                if(grid[playerY][playerX].topWall || grid[playerY][playerX].rightWall ||
                                    grid[playerY-1][playerX].rightWall || grid[playerY][playerX+1].topWall) {
                                    validMove = false;
                                }
                            } else if(dy == 1 && dx == -1) { // bottom-left
                                if(grid[playerY][playerX].bottomWall || grid[playerY][playerX].leftWall ||
                                    grid[playerY+1][playerX].leftWall || grid[playerY][playerX-1].bottomWall) {
                                    validMove = false;
                                }
                            } else if(dy == 1 && dx == 1) { // bottom-right
                                if(grid[playerY][playerX].bottomWall || grid[playerY][playerX].rightWall ||
                                    grid[playerY+1][playerX].rightWall || grid[playerY][playerX+1].bottomWall) {
                                    validMove = false;
                                }
                            }
                            //move player if move is valid
                            if(validMove) {
                                playerX = newX;
                                playerY = newY;
                                lastdx = dx; // Update lastdx only if there is a change
                                lastdy = dy; // Update lastdy only if there is a change
                                std::cout << "Player moved to: (" << playerX << ", " << playerY << ")" << std::endl;
                                joystickClock.restart();
                            } else {
                                std::cout << "Move blocked by wall!" << std::endl;
                            }
                        } else {
                            std::cout << "Move would go out of bounds!" << std::endl;
                        }
                    }
                }
            }
        }
    }

    int getPlayerX() {
        return playerX;
    }

    int getPlayerY() {
        return playerY;
    }

    void joystickCleanup() {
        jstick.cleanup();
    }
};