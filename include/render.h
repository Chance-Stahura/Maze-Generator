#pragma once 

#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <string>
#include <map>
#include <cmath>
#include <climits>
#include <ctime>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include "mazeGenerator.h"
#include "gameLogic.h"

//SFML
struct graphDisplay{
    graphSolver solver;
    sf::RenderWindow window;
    sf::Event event;
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    int cellSize;

    void createWindow(std::vector<std::vector<cell>>& grid) {
        int height = grid.size();
        int width = grid[0].size();
        int cellWidth = desktop.width / width;
        int cellHeight = desktop.height / height;
        if(cellWidth < cellHeight) {
            cellSize = cellWidth;
        } else {
            cellSize = cellHeight;
        }
        window.create(sf::VideoMode(width * cellSize, height * cellSize), "Maze Display");
        window.setPosition(sf::Vector2i(100,100));
        window.setFramerateLimit(60);
    }
    
    void handleEvents() {
        while(window.pollEvent(event)) {
            if(event.type == sf::Event::Closed) {
                window.close();
            }
        }    
    }
    //draws one frame of current game state
    void displayGraph(std::vector<std::vector<cell>>& grid, int playerX, int playerY) {
        static sf::Clock debugClock;
        
        int height = grid.size();
        int width = grid[0].size();
        
        window.clear(sf::Color::White);
                
        // Debug output every 2 seconds
        if(debugClock.getElapsedTime().asMilliseconds() > 2000) {
            debugClock.restart();
            std::cout << "Clock has been restarted!" << std::endl;
        }    
           
        for(int i = 0; i < height; i++) {
            for(int j = 0; j < width; j++) {
                int x = j * cellSize;
                int y = i * cellSize;

                //draw cell
                sf::RectangleShape cellShape(sf::Vector2f(cellSize, cellSize));
                cellShape.setPosition(x, y);
                cellShape.setFillColor(sf::Color::White);
                window.draw(cellShape);

                //bool isPathCell = false;
                for(cell* node : solver.finalPath) {
                    if(node->x == j && node->y == i) {
                        //isPathCell = true;
                        break;
                    }
                }

                //draw walls
                if(grid[i][j].topWall) {
                    sf::RectangleShape topWall(sf::Vector2f(cellSize, 1));
                    topWall.setPosition(x, y);
                    topWall.setFillColor(sf::Color::Black);
                    window.draw(topWall);
                }
                if(grid[i][j].bottomWall) {
                    sf::RectangleShape bottomWall(sf::Vector2f(cellSize, 1));
                    bottomWall.setPosition(x, y + cellSize - 1);
                    bottomWall.setFillColor(sf::Color::Black);
                    window.draw(bottomWall);
                }
                if(grid[i][j].leftWall) {
                    sf::RectangleShape leftWall(sf::Vector2f(1, cellSize));
                    leftWall.setPosition(x, y);
                    leftWall.setFillColor(sf::Color::Black);
                    window.draw(leftWall);
                }
                if(grid[i][j].rightWall) {
                    sf::RectangleShape rightWall(sf::Vector2f(1, cellSize));
                    rightWall.setPosition(x + cellSize - 1, y);
                    rightWall.setFillColor(sf::Color::Black);
                    window.draw(rightWall);
                }
                //draw best path
                    for(cell* node : solver.finalPath) {
                    if(node->y == i && node->x == j) {
                        float pathSize = cellSize / 2.0f;
                        float offset = (cellSize - pathSize) / 2.0f; // Center the path cell
                        sf::RectangleShape pathCell(sf::Vector2f(cellSize/2, cellSize/2));
                        //cout << "Path cell at: " << node->x << ", " << node->y << endl;
                        pathCell.setPosition(x + offset, y + offset);
                        pathCell.setFillColor(sf::Color::Green);
                        window.draw(pathCell);
                    }
                }
                if(i == playerY && j == playerX) {
                    //draw player
                    sf::CircleShape playerShape(cellSize / 4.0f);
                    playerShape.setFillColor(sf::Color::Red);
                    playerShape.setPosition(x + cellSize / 4.0f, y + cellSize / 4.0f);
                    window.draw(playerShape);
                }
            }
        } 
        window.display();
    }

    bool isOpen() {
        if(window.isOpen()) {
            return true;
        }
        return false;
    }
};