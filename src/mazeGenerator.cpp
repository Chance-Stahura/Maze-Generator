#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <string>
#include <map>
#include <cmath>
#include <ctime>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include "C:/Users/koa/Desktop/maze_generator/libserialport/include/libserialport/libserialport.h"

using namespace std;

//part of grid/maze
//this is the basis of the actual 'terrain' that will be navigated
struct cell {
    cell* parent = nullptr;
    bool topWall = true, bottomWall = true, leftWall = true, rightWall = true;
    bool visited = false;

    int x = 0, y = 0;

    int currentCost = INT_MAX, estCost = 0, totalCost = INT_MAX;
    //g(n) , h(n) , f(n)
        //estCost = h(n) = currentCost + heuristic
        //currentCost = g(n) = cost to reach this node
        //totalCost = f(n) = currentCost + estCost
};

//connection between two cells
struct wall {
    int x, y, dx, dy, adjX, adjY;
};

//STRUCTS ARE PUBLIC BY DEFAULT! CLASSES ARE PRIVATE BY DEFAULT
struct graphGenerator{
    //this is the basic structure for every cell in the graph
        //each cell will have a height and width
    int height, width;
    //need a 2d vector of 'cells'
    //vector<vector<cell>>grid(int height, vector<cell>width);
    vector<vector<cell>>grid;
   
    void gridInitializer(int h, int w) {
    height = h;
    width = w;
    grid = vector<vector<cell>>(h, vector<cell>(w));
   }


    int randIndex(int i) {
        return rand() % i;
    }

    void addWalls(int x, int y, vector<wall>&frontier, vector<vector<cell>>&grid) {
        //top
        if(y > 0 && !grid[y-1][x].visited) {
            wall wallz;
            wallz.dx = 0;
            wallz.dy = -1;
            wallz.x = x;
            wallz.y = y;
            wallz.adjX = x;
            wallz.adjY = y - 1;
            frontier.push_back(wallz);
        }
        //bottom
        if(y < height - 1 && !grid[y+1][x].visited) {
            wall wallz;
            wallz.dx = 0;
            wallz.dy = 1;
            wallz.x = x;
            wallz.y = y;
            wallz.adjX = x;
            wallz.adjY = y + 1;
            frontier.push_back(wallz);
        }
        //left
        if(x > 0 && !grid[y][x-1].visited) {
            wall wallz;
            wallz.dx = -1;
            wallz.dy = 0;
            wallz.x = x;
            wallz.y = y;
            wallz.adjX = x - 1;
            wallz.adjY = y;
            frontier.push_back(wallz);
        }
        //right
        if(x < width - 1 && !grid[y][x+1].visited) {
            wall wallz;
            wallz.dx = 1;
            wallz.dy = 0;
            wallz.x = x;
            wallz.y = y;
            wallz.adjX = x + 1;
            wallz.adjY = y;
            frontier.push_back(wallz);
        }
    }
      
    //prim's implementation takes the generated grid as an arguement. 
    //it will 'remove walls' under the condition that no cycles are formed

    void prims(vector<vector<cell>>& grid) {
        if(grid.empty()) {
            cout << "Grid is empty!" << endl;
            gridInitializer(200, 200);
        }

       //this chooses random start cell 
       int x = randIndex(width-1);
       int y = randIndex(height-1);
       
      //this starts from top left corner
      //int x = 0, y = 0;
       //cell startCell = grid[y][x];
       grid[y][x].visited = true;
       vector<wall> frontier;

       //add all walls of the start cell to the frontier
       //with a helper function
        addWalls(x, y, frontier, grid);

       while(!frontier.empty()) {
            int index = randIndex(frontier.size());
            wall current = frontier[index];
            frontier.erase(frontier.begin() + index);
            //if neighbor is univisited
            if(!grid[current.adjY][current.adjX].visited) {
                //remove neighboring wall
                if(current.dy == -1 && current.dx == 0) {
                    grid[current.y][current.x].topWall = false;
                    grid[current.adjY][current.adjX].bottomWall = false;
                }
                if(current.dy == 1 && current.dx == 0) {
                    grid[current.y][current.x].bottomWall = false;
                    grid[current.adjY][current.adjX].topWall = false;
                }
                if(current.dy == 0 && current.dx == -1) {
                    grid[current.y][current.x].leftWall = false;
                    grid[current.adjY][current.adjX].rightWall = false;
                }
                if(current.dy == 0 && current.dx == 1) {
                    grid[current.y][current.x].rightWall = false;
                    grid[current.adjY][current.adjX].leftWall = false;
                }
                //TO ACCOUNT FOR 8 DIRECTIONS NEED TO CHECK FOR DIAGONAL DIRECTIONS
                /*
                {-1, -1}, // top-left
                {1, -1},  // top-right
                {-1, 1},  // bottom-left
                {1, 1}    // bottom-right
                */
               if(current.dy == -1 && current.dx == -1) {
                    grid[current.y][current.x].topWall = false;
                    grid[current.adjY][current.adjX].bottomWall = false;
                    grid[current.y][current.x].leftWall = false;
                    grid[current.adjY][current.adjX].rightWall = false;
                }
                if(current.dy == -1 && current.dx == 1) {
                    grid[current.y][current.x].topWall = false;
                    grid[current.adjY][current.adjX].bottomWall = false;
                    grid[current.y][current.x].rightWall = false;
                    grid[current.adjY][current.adjX].leftWall = false;
                }
                if(current.dy == 1 && current.dx == -1) {
                    grid[current.y][current.x].bottomWall = false;
                    grid[current.adjY][current.adjX].topWall = false;
                    grid[current.y][current.x].leftWall = false;
                    grid[current.adjY][current.adjX].rightWall = false;
                }
                if(current.dy == 1 && current.dx == 1) {
                    grid[current.y][current.x].bottomWall = false;
                    grid[current.adjY][current.adjX].topWall = false;
                    grid[current.y][current.x].rightWall = false;
                    grid[current.adjY][current.adjX].leftWall = false;
                }
            
                grid[current.adjY][current.adjX].visited = true;
            
                //int x = current.adjX;
                //int y = current.adjY; 
                addWalls(current.adjX, current.adjY, frontier, grid); 
            }
                
        }
    }
};

struct joystick {

    struct sp_port *port = nullptr;
    struct sp_port **ports = nullptr;
    bool initialized = false;

    bool initialize() {
        if(port) {
            sp_close(port);
            port = nullptr;
        }
        if(ports) {
            sp_free_port_list(ports);
            ports = nullptr;
        }

         //find all serial ports
        sp_return result = sp_list_ports(&ports);
         if(result != SP_OK) {
            cout << "Failed to find serial ports " << (void*)ports << endl;
            return false;
         }
            cout << "Serial ports found: " << (void*)ports << endl;
            int count = 0;
           
        while(ports[count] != nullptr) {
            const char* portName = sp_get_port_name(ports[count]);
            cout << "Port " << count << ": " << (sp_get_port_name(ports[count]) ? sp_get_port_name(ports[count]) : "Unknown") << endl;
            count++;
        }

        if (count == 0) {
            std::cout << "No valid ports found!" << std::endl;
            sp_free_port_list(ports);
            ports = nullptr;
            return false;
        }

        result = sp_copy_port(ports[0], &port);
        if(result != SP_OK) {
            cout << "Failed to copy port: " << endl;
            sp_free_port_list(ports);
            ports = nullptr;
            return false;
        }
        
        sp_free_port_list(ports);
        ports = nullptr; //free the port list after copying
        //configure port settings

        if (sp_open(port, SP_MODE_READ_WRITE) != SP_OK) {
            std::cout << "Failed to open port: " << sp_get_port_name(port) << std::endl;
            //sp_last_error_message() << std::endl;
            sp_free_port(port);
            ports = nullptr;
            return false;
        }

        //TRY DIFFERENT BAUDRATES
        if(sp_set_baudrate(port, 9600) != SP_OK) {
            std::cout << "Failed to set baud rate: " << sp_last_error_message() << std::endl;
            sp_close(port);
            sp_free_port(port);
            ports = nullptr;
            return false;
        }

        sp_set_bits(port, 8); //set data bits to 8
        sp_set_parity(port, SP_PARITY_NONE); //set parity to none
        sp_set_stopbits(port, 1); //set stop bits to 1
        sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE); //set flow control to none

        //sp_close(port);
        sp_flush(port, SP_BUF_BOTH); //clear both input and output buffers
        sf::sleep(sf::milliseconds(2000)); //wait for the port to open
        sp_flush(port, SP_BUF_INPUT);
        initialized = true;
        cout << "port initialized successfully: " << sp_get_port_name(port) << endl;
        return true;
    
    //return false;
}
    bool readJoystick(char* buffer, int bufferSize, int timeoutMS = 1000) {
        if(!port || !initialized) return false;
        //sp_flush(port, SP_BUF_INPUT); //clear the input buffer
        int bytesRead = sp_nonblocking_read(port, buffer, bufferSize -1);
        if(bytesRead > 0) {
            buffer[bytesRead] = '\0'; //null-terminate the string
            std::cout << "Joystick input: " << string(buffer, bytesRead) << std::endl;
            return true;
        }
        else if(bytesRead < 0) {
            std::cerr << "Error reading from joystick: " << sp_last_error_code() << std::endl;
            sp_close(port);
            initialized = false;
            return false;
        }
        return false;
    }

    void cleanup() {
        if(port) sp_close(port);
        sp_free_port_list(ports);
        ports = nullptr; //free the port list
    }
 };

//"mouse"
struct graphSolver {

   //graphGenerator generator;
   vector<cell*> finalPath;

    //since edgeweights will all be the same, and we are essentially traversing an unweighted undirected graph,
        //USE BFS or some adaptation of BFS/DFS - A*, Greedy DFS, etc
        //A* requires min heap priority queue implementation
            //push(add), pop(delete), peek, upward(after insertion)/downward(after popping root) heapify
            //USEFUL - update priority, need hash map or index array, or regular map??
    
    struct priorQueue {

        cell nodeTracker;
        vector<cell> heapArr;
        int size;
        map<pair<int, int>, int> index;

        priorQueue() : size(0) {}
        
        //compares the current best path 
          //  vs
        //an estimate of what it will take to finish the path
        
       //prioritize paths based on their total cost
        bool importance(cell A, cell B) {
            // f(n) = g(n) + h(n)
            if(A.totalCost < B.totalCost) 
                return true;
            if(A.totalCost == B.totalCost) 
                return true;
            return false;
        }

        void upHeapify(int i) {
            int parent = (i-1)/2;
            int current = i;

            if(i <= 0 || !importance(heapArr[current], heapArr[parent]))
                return;

            if(current > parent && importance(heapArr[current], heapArr[parent])) {
                swap(heapArr[current], heapArr[parent]);
                //upHeapify(current);
                upHeapify(parent);
            }
                
        }

        void downHeapify(int i) {
            int leftChild = ((i+1)*2)-1;
            int rightChild = ((i+1)*2);
            int current = i;

            if(leftChild < size && importance(heapArr[leftChild], heapArr[current]))
                current = leftChild;
            if(rightChild < size && importance(heapArr[rightChild], heapArr[current]))
                current = rightChild;

            if(current != i) {
                swap(heapArr[i], heapArr[current]);
                downHeapify(current);
            }
        }

        void push(cell n) {
            //traverse the vector, insert at end
            //perform upward heapify
            heapArr.push_back(n);
            index[{n.x, n.y}] = size; //store the index of the cell in the map
            size++;
            //nodeTracker.currentCost++;
            upHeapify(size - 1);
        }
        
        cell pop() {
            if (isEmpty()) return cell{};

            cell top = heapArr.front();
            index.erase({top.x, top.y});
            if(size > 1) {
                heapArr[0] = heapArr.back();
                index[{heapArr[0].x, heapArr[0].y}] = 0;
            }
            heapArr.pop_back();
            size--;
            if(size > 1)
                downHeapify(0);
            
            return top;
        }

        cell peek() {
            if(isEmpty()) {
                return cell{0, 0, 0};
            }
            return heapArr[0];
        }

        bool isEmpty() {
            if(heapArr.size() == 0)
                return true;
            return false;
        }

        bool updatePriority(int x, int y) {
            //when you find a better currentCost to a node already in heap
            if(index.find({x, y}) != index.end()) {
                int i = index[{x, y}];
                //if the new currentCost is less than the old one
                if(nodeTracker.currentCost < heapArr[i].currentCost) {
                    heapArr[i].currentCost = nodeTracker.currentCost;
                    upHeapify(i);
                    return true;
                }
            }
           return false;
        }

        void insert(cell n) {push(n);}

        void dlt(int i) {
            heapArr[i] = heapArr.back();
            heapArr.pop_back();
            size--;
            if(size > 0)
                downHeapify(0);
        }
    };

    //A* needs access to priorQueue struct
    priorQueue pQ;

    std::vector<std::pair<int, int>> directions = {
        {-1, -1}, // top-left
        {0, -1},  // top
        {1, -1},  // top-right
        {-1, 0},  // left
        {1, 0},   // right
        {-1, 1},  // bottom-left
        {0, 1},   // bottom
        {1, 1}    // bottom-right
    };

    void aStarSearch(vector<vector<cell>>& grid) {
        for(int i = 0; i < grid.size(); i++) {
            for(int j = 0; j < grid[0].size(); j++) {
                grid[i][j].visited = false;
                grid[i][j].totalCost = INT_MAX; //initialize totalCost to max value
                //grid[i][j].currentCost = INT_MAX;
                //grid[i][j].totalCost = INT_MAX;
                //grid[i][j].estCost = 0;
                //grid[i][j].parent = nullptr;
                //grid[i][j].x = j; // Initialize coordinates
                //grid[i][j].y = i;
            }
        }
        
        //this chooses random start cell 
        //int x = rand() % grid[0].size();
        //int y = rand() % grid.size();
    
        int x = 0, y = 0; //start at top left corner
        grid[y][x].x = x;
        grid[y][x].y = y;
        cout << "Start node: " << y << ", " << x << std::endl;
        //assign last node in grid as end node
        cell end;
        int endX = grid[0].size() - 1; //last column
        int endY = grid.size() - 1; //last row

        //this chooses a random end node
        //int endX = rand() % grid[0].size() - 1;
        //int endY = rand() % grid.size() - 1;

        end.x = endX;
        end.y = endY;
        cout << "End node: " << endY << ", " << endX << std::endl;
        
        //accounts for 8 directions: up, down, left, right, and diagonals
        int heuristic = sqrt(pow((end.x - x), 2) + pow((end.y - y), 2)); //pythagorean theorem for heuristic

        //only using 4 directions for now
        //int heuristic = abs(end.x - x) + abs(end.y - y); //manhattan distance for heuristic
 
        grid[y][x].currentCost = 0; //starting cost is 0
        grid[y][x].estCost = heuristic; //estCost is 0 at the start
        grid[y][x].totalCost = grid[y][x].estCost; //totalCost is 0 at the start
        grid[y][x].parent = nullptr; //no parent at the start

        pQ.push(grid[y][x]); //push the start cell into the priority queue

        //will be structured like dijkstras inside the main while loop
        while(!pQ.isEmpty()) {        
            cell current = pQ.pop();
              
            grid[current.y][current.x].visited = true; //mark as visited
            cout << "Current node: " << current.y << ", " << current.x << std::endl;
            if(current.x == endX && current.y == endY) {
                finalPath.clear();
                cell* pathNode = &grid[current.y][current.x];
                while(pathNode != nullptr && pathNode->parent != nullptr) {
                    
                    int dx = pathNode->x - pathNode->parent->x;
                    int dy = pathNode->y - pathNode->parent->y;
                    if((dy == -1 && pathNode->parent->topWall) ||
                        (dy == 1 && pathNode->parent->bottomWall) ||
                        (dx == -1 && pathNode->parent->leftWall) ||
                        (dx == 1 && pathNode->parent->rightWall))
                        return;
                    
                    finalPath.push_back(pathNode);
                    pathNode = pathNode->parent; //backtrack to find the path
                }
                if(pathNode != nullptr)
                    finalPath.push_back(pathNode); //add the start node to the path
                
                std::reverse(finalPath.begin(), finalPath.end()); //reverse the path to get the correct order    
                return;
            }
            
            for(const auto& direction : directions) {
                cout << "checking directions" << std::endl; //PROGRAM DOES NOT ENTER THIS
                int dx = direction.first;
                int dy = direction.second;

                int newX = current.x + dx;
                int newY = current.y + dy;
                cout << "New node: " << newY << ", " << newX << std::endl;
                //currentCost gets updated before push() gets called
                float moveCost = (dx == 0 && dy == 0) ? 1.0f : 1.414f; //diagonal moves cost more
                int newCurrentCost = current.currentCost + moveCost; //assuming each step has a cost of 1 or 1.414 for diagonal moves
                //int newCurrentCost = current.currentCost + 1; //assuming each step has a cost of 1
                //int newHeuristic = abs(end.x - newX) + abs(end.y - newY); //manhattan distance for heuristic
                int newHeuristic = sqrt(pow((end.x - newX), 2) + pow((end.y - newY), 2)); //pythagorean theorem for heuristic
                int newTotalCost = newCurrentCost + newHeuristic;

                //check if newX and newY are within bounds of the grid
                if(newX < 0 || newY < 0 || newX >= grid[0].size() || newY >= grid.size()) continue;
                //check if cell is not a wall
                if(grid[current.y][current.x].topWall && dy == -1) continue;
                if(grid[current.y][current.x].bottomWall && dy == 1) continue;
                if(grid[current.y][current.x].leftWall && dx == -1) continue;
                if(grid[current.y][current.x].rightWall && dx == 1) continue;

                 if(dy == -1 && dx == -1)//top left
                    if(grid[current.y][current.x].topWall || grid[current.y][current.x].leftWall ||
                        grid[current.y-1][current.x].leftWall || grid[current.y][current.x-1].topWall) 
                        continue;
                if(dy == -1 && dx == 1)//top right
                    if(grid[current.y][current.x].topWall || grid[current.y][current.x].rightWall ||
                        grid[current.y-1][current.x].rightWall || grid[current.y][current.x+1].topWall) 
                        continue;
                if(dy == 1 && dx == -1)//bottom left
                    if(grid[current.y][current.x].bottomWall || grid[current.y][current.x].leftWall ||
                        grid[current.y+1][current.x].leftWall || grid[current.y][current.x-1].bottomWall) 
                        continue;
                if(dy == 1 && dx == 1)//bottom right
                    if(grid[current.y][current.x].bottomWall || grid[current.y][current.x].rightWall ||
                        grid[current.y+1][current.x].rightWall || grid[current.y][current.x+1].bottomWall) 
                        continue;

                //check if cell is already visited
                if(grid[newY][newX].visited) continue;
                //check if the new path is better than the old one
                if(newTotalCost < grid[newY][newX].totalCost) {
                    grid[newY][newX].currentCost = newCurrentCost;
                    grid[newY][newX].estCost = newHeuristic;
                    grid[newY][newX].totalCost = newTotalCost;
                    grid[newY][newX].x = newX; //set x coordinate
                    grid[newY][newX].y = newY; //set y coordinate
                    grid[newY][newX].parent = &grid[current.y][current.x]; //set parent to current node
                    pQ.push(grid[newY][newX]); //push the new cell into the priority queue'
                }
            }
        }
    }
};

//SFML
struct graphDisplay{
    graphSolver solver;
    joystick jstick;

    void displayGraph(vector<vector<cell>>& grid) {
        static string lineBuffer = "";
        static sf::Clock joystickClock;
        static sf::Clock debugClock;
        
        int height = grid.size();
        int width = grid[0].size();
        int cellSize = 70; // Size of each cell in pixels
        int playerX = 0, playerY = 0; // Player starting position

        sf::RenderWindow window(sf::VideoMode(width * cellSize, height * cellSize), "Graph Display");

        //UNCOMMENT TO SEE THE BEST PATH
        solver.aStarSearch(grid);

        window.setPosition(sf::Vector2i(100,100));
        window.setFramerateLimit(60);

        char buffer[256]; // Buffer for joystick input

        if(!jstick.initialize()) {
            cout << "Failed to initialize joystick" << endl;
        }

        while(window.isOpen()) {
            sf::Event event;

            while(window.pollEvent(event)) {
                if(event.type == sf::Event::Closed) {
                    window.close();
                }
            }
            window.clear(sf::Color::White);


            if(jstick.port && jstick.initialized) {
                char readBuffer[256];
                int result = sp_nonblocking_read(jstick.port, readBuffer, sizeof(readBuffer)-1);

                static int xVal = -1, yVal = -1, swVal = -1;
                static bool xUpdated = false, yUpdated = false, swUpdated = false;

                if(result > 0) {
                    readBuffer[result] = '\0';
                    lineBuffer += string(readBuffer);
        
                    // Debug output every 2 seconds
                    if(debugClock.getElapsedTime().asMilliseconds() > 2000) {
                        debugClock.restart();
                        cout << "Clock has been restarted!" << endl;
                    }    
                   
                    // Process complete lines
                    size_t newlinePos;
                    while((newlinePos = lineBuffer.find_first_of("\r\n")) != string::npos) {
                        string completeLine = lineBuffer.substr(0, newlinePos);
                        lineBuffer = lineBuffer.substr(newlinePos + 1);
            
                        // Skip empty lines
                        if(completeLine.empty()) continue;

                        if(completeLine.find("X-axis:") == string::npos && 
                           completeLine.find("Y-axis:") == string::npos && 
                           completeLine.find("Switch:") == string::npos) {
                            continue;
                           }
            
                        cout << "Processing line: '" << completeLine << "'" << endl;

                                try {
                                    if (completeLine.find("X-axis:") != string::npos) {
                                        cout << "[X Detected] " << completeLine << endl;
                                        size_t colonPos = completeLine.find(":");
                                        if (colonPos != string::npos) {
                                            string xStr = completeLine.substr(colonPos + 1);
                                            // Remove all whitespace
                                            xStr.erase(remove_if(xStr.begin(), xStr.end(), ::isspace), xStr.end());
                                            if (!xStr.empty()) {
                                                xVal = stoi(xStr);
                                                xUpdated = true;
                                                cout << "[X Parsed] xVal = " << xVal << endl;
                                            } else {
                                                cout << "[X Warning] xStr was empty after cleanup!" << endl;
                                         }
                                        } else {
                                            cout << "[X Warning] Could not find ':' in line: " << completeLine << endl;
                                        }
                                    }

                                    if (completeLine.find("Y-axis:") != string::npos) {
                                        cout << "[Y Detected] " << completeLine << endl;
                                        size_t colonPos = completeLine.find(":");
                                        if (colonPos != string::npos) {
                                            string yStr = completeLine.substr(colonPos + 1);
                                            // Remove all whitespace
                                            yStr.erase(remove_if(yStr.begin(), yStr.end(), ::isspace), yStr.end());
                                            if (!yStr.empty()) {
                                                yVal = stoi(yStr);
                                                yUpdated = true;
                                             cout << "[Y Parsed] yVal = " << yVal << endl;
                                            } else {
                                                cout << "[Y Warning] yStr was empty after cleanup!" << endl;
                                            }
                                        } else {
                                            cout << "[Y Warning] Could not find ':' in line: " << completeLine << endl;
                                        }
                                    }

                                    if (completeLine.find("Switch:") != string::npos) {
                                        cout << "[Switch Detected] " << completeLine << endl;
                                        size_t colonPos = completeLine.find(":");
                                        if (colonPos != string::npos) {
                                            string swStr = completeLine.substr(colonPos + 1);
                                            // Remove all whitespace
                                            swStr.erase(remove_if(swStr.begin(), swStr.end(), ::isspace), swStr.end());

                                            if (!swStr.empty()) {
                                                swVal = stoi(swStr);
                                                swUpdated = true;
                                                cout << "[Switch Parsed] swVal = " << swVal << endl;
                                            } else {
                                                cout << "[Switch Warning] swStr was empty after cleanup!" << endl;
                                            }
                                        } else {
                                            cout << "[Switch Warning] Could not find ':' in line: " << completeLine << endl;
                                        }
                                    }


                                } catch(const exception& e) {
                                    cout << "Parse error: " << e.what() << endl;
                                    continue;
                                } 
                    }

                        if(xUpdated && yUpdated && swUpdated) {
                            cout << "Parsed values - X: " << xVal << ", Y: " << yVal << ", SW: " << swVal << endl;

                            // GAME CONTROL LOGIC
                            if(/*swVal == 1 &&*/ joystickClock.getElapsedTime().asMilliseconds() > 200) {
                                // Button is not pressed and enough time has passed
                                const int DEADZONE = 50; // Define a deadzone for joystick movement
                                const int CENTERX = 518; // Center value for X-axis
                                const int CENTERY = 507; // Center value for Y-axis
                                int dx = 0, dy = 0;
                                static int lastdx = 0, lastdy = 0; // Store last movement direction
                                if(xVal > CENTERX + DEADZONE) {
                                    dx = 1; // Move right
                                    cout << "Moving right" << endl;
                                } else if(xVal < CENTERX - DEADZONE) {
                                    dx = -1; // Move left
                                    cout << "Moving left" << endl;
                                } 

                                if(yVal > CENTERY + DEADZONE) {
                                    dy = 1; // Move down
                                    cout << "Moving down" << endl;
                                } else if(yVal < CENTERY - DEADZONE) {
                                    dy = -1; // Move up
                                    cout << "Moving up" << endl;
                                }
                                        
                                if(dx != 0 /*&& dx != lastdx*/ || dy != 0 /*&& dy != lastdy*/) {                                        
                                    cout << "Movement command: dx=" << dx << ", dy=" << dy << endl;
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
                            
                                            if(validMove) {
                                                playerX = newX;
                                                playerY = newY;
                                                lastdx = dx; // Update lastdx only if there is a change
                                                lastdy = dy; // Update lastdy only if there is a change
                                                cout << "Player moved to: (" << playerX << ", " << playerY << ")" << endl;
                                                joystickClock.restart();
                                            } else {
                                                cout << "Move blocked by wall!" << endl;
                                            }
                                        } else {
                                            cout << "Move would go out of bounds!" << endl;
                                        }
                                    }
                                }
                            }

                            xUpdated = false;
                            yUpdated = false;
                            swUpdated = false;
                        }

                    } else if(result < 0) {
                        cout << "Error reading from joystick port" << endl;
                    }
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

                    bool isPathCell = false;
                    for(cell* node : solver.finalPath) {
                        if(node->x == j && node->y == i) {
                            isPathCell = true;
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
        jstick.cleanup(); //cleanup joystick resources
    }
};

int main() {

    graphGenerator generator;
    graphDisplay display;
    //joystick jstick;

    srand(time(0)); //this is to help choose a random index for the graph generator!
    //this helps choose a random wall(s) to remove inside of the prim's implementation
    //Serial.print("Serial port initialized");
    int height = 0, width = 0;
    int cellSize = 10; // Size of each cell in pixels
   
    std::cout << "Welcome to the Super Sweet Graph Solver" << endl;
    std::cout << "Please enter dimensions for graph" << endl;
    cout << "\n" << endl;
    cout << "Height : " << flush;
    cin >> height;
    cout << "Width : " << flush;
    cin >> width;
    cout << "\n" << endl;
    //cout << "Thank you for your input! here is your graph, and the associated best path!" << endl;
    cout.flush();

    generator.gridInitializer(height, width);
    generator.prims(generator.grid);
    display.displayGraph(generator.grid);
    //jstick.readJoystick();

    return 0;
}

/*
maze generator/best path visualization compile command:
!!!DOES NOT WORK WITH JOYSTICK INPUT!!!

g++ mazeGenerator.cpp -o maze.exe -IC:\Users\koa\Desktop\SFML-2.6.1-windows-gcc-13.1.0-mingw-64-bit\SFML-2.6.1\include -LC:\Users\koa\Desktop\SFML-2.6.1-windows-gcc-13.1.0-mingw-64-bit\SFML-2.6.1\lib -lsfml-graphics -lsfml-window -lsfml-system

*/

/*
joystick input compile command:

g++ mazeGenerator.cpp -o maze.exe -IC:\Users\koa\Desktop\SFML-2.6.1-windows-gcc-13.1.0-mingw-64-bit\SFML-2.6.1\include -LC:\Users\koa\Desktop\SFML-2.6.1-windows-gcc-13.1.0-mingw-64-bit\SFML-2.6.1\lib -LC:\Users\koa\Desktop\maze_generator\libserialport\lib -lsfml-graphics -lsfml-window -lsfml-system -lserialport -lsetupapi -lcfgmgr32 -lopengl32 -lglu32

*/