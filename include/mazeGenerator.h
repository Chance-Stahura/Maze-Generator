#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <string>
#include <climits>
#include <cmath>
#include <map>

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
    std::vector<std::vector<cell>>grid;
   
    void gridInitializer(int h, int w) {
    height = h;
    width = w;
    grid = std::vector<std::vector<cell>>(h, std::vector<cell>(w));
   }

    int randIndex(int i) {
        return rand() % i;
    }

    void addWalls(int x, int y, std::vector<wall>&frontier, std::vector<std::vector<cell>>&grid) {
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

    void prims(std::vector<std::vector<cell>>& grid) {
        if(grid.empty()) {
            std::cout << "Grid is empty!" << std::endl;
            gridInitializer(200, 200);
        }

       //this chooses random start cell 
       int x = randIndex(width-1);
       int y = randIndex(height-1);
       
      //this starts from top left corner
      //int x = 0, y = 0;
       //cell startCell = grid[y][x];
       grid[y][x].visited = true;
       std::vector<wall> frontier;

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

//"mouse"
struct graphSolver {

   //graphGenerator generator;
   std::vector<cell*> finalPath;

    //since edgeweights will all be the same, and we are essentially traversing an unweighted undirected graph,
        //USE BFS or some adaptation of BFS/DFS - A*, Greedy DFS, etc
        //A* requires min heap priority queue implementation
            //push(add), pop(delete), peek, upward(after insertion)/downward(after popping root) heapify
            //USEFUL - update priority, need hash map or index array, or regular map??
    
    struct priorQueue {

        cell nodeTracker;
        std::vector<cell> heapArr;
        int size;
        std::map<std::pair<int, int>, int> index;

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
                std::swap(heapArr[current], heapArr[parent]);
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
                std::swap(heapArr[i], heapArr[current]);
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

    void aStarSearch(std::vector<std::vector<cell>>& grid) {
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
        std::cout << "Start node: " << y << ", " << x << std::endl;
        //assign last node in grid as end node
        cell end;
        int endX = grid[0].size() - 1; //last column
        int endY = grid.size() - 1; //last row

        //this chooses a random end node
        //int endX = rand() % grid[0].size() - 1;
        //int endY = rand() % grid.size() - 1;

        end.x = endX;
        end.y = endY;
        std::cout << "End node: " << endY << ", " << endX << std::endl;
        
        //accounts for 8 directions: up, down, left, right, and diagonals
        int heuristic = std::sqrt(std::pow((end.x - x), 2) + std::pow((end.y - y), 2)); //pythagorean theorem for heuristic

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
            std::cout << "Current node: " << current.y << ", " << current.x << std::endl;
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
                std::cout << "checking directions" << std::endl; //PROGRAM DOES NOT ENTER THIS
                int dx = direction.first;
                int dy = direction.second;

                int newX = current.x + dx;
                int newY = current.y + dy;
                std::cout << "New node: " << newY << ", " << newX << std::endl;
                //currentCost gets updated before push() gets called
                float moveCost = (dx == 0 && dy == 0) ? 1.0f : 1.414f; //diagonal moves cost more
                int newCurrentCost = current.currentCost + moveCost; //assuming each step has a cost of 1 or 1.414 for diagonal moves
                //int newCurrentCost = current.currentCost + 1; //assuming each step has a cost of 1
                //int newHeuristic = abs(end.x - newX) + abs(end.y - newY); //manhattan distance for heuristic
                int newHeuristic = std::sqrt(std::pow((end.x - newX), 2) + std::pow((end.y - newY), 2)); //pythagorean theorem for heuristic
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

