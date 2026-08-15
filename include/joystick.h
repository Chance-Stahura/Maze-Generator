#pragma once

#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <libserialport.h>

struct joystick {

public:

    struct joystickState {
        int x, y, sw;
    };

    void updateState() {
        readJoystick(); 
    }

    bool hasNewState() {
        if(completeState) {
            return true;
        }
        return false;      
    }

    joystickState getState() { 
            completeState = false;
            return currentState;
    } 
    
    bool initialize() {
        if(port) {
            sp_close(port);
            sp_free_port(port);
            port = nullptr;
        }
        if(ports) {
            sp_free_port_list(ports);
            ports = nullptr;
        }

         //find all serial ports
        sp_return result = sp_list_ports(&ports);
         if(result != SP_OK) {
            std::cout << "Failed to find serial ports " << (void*)ports << std::endl;
            return false;
         }
            std::cout << "Serial ports found: " << (void*)ports << std::endl;
            int count = 0;
           
        while(ports[count] != nullptr) {
            const char* portName = sp_get_port_name(ports[count]);
            std::cout << "Port " << count << ": " << (sp_get_port_name(ports[count]) ? sp_get_port_name(ports[count]) : "Unknown") << std::endl;
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
            std::cout << "Failed to copy port: " << std::endl;
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
            port = nullptr;
            return false;
        }

        //TRY DIFFERENT BAUDRATES
        if(sp_set_baudrate(port, 9600) != SP_OK) {
            std::cout << "Failed to set baud rate: " << sp_last_error_message() << std::endl;
            sp_close(port);
            sp_free_port(port);
            port = nullptr;
            return false;
        }

        sp_set_bits(port, 8); //set data bits to 8
        sp_set_parity(port, SP_PARITY_NONE); //set parity to none
        sp_set_stopbits(port, 1); //set stop bits to 1
        sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE); //set flow control to none

        //sp_close(port);
        sp_flush(port, SP_BUF_BOTH); //clear both input and output buffers
        //sf::sleep(sf::milliseconds(2000)); //wait for the port to open
        sp_flush(port, SP_BUF_INPUT);
        initialized = true;
        std::cout << "port initialized successfully: " << sp_get_port_name(port) << std::endl;
        return true;
    
    }

    void cleanup() {
        if(port) sp_close(port);
        sp_free_port(port);
        sp_free_port_list(ports);
        port = nullptr; //free the port list
        initialized = false;
    }

private:

    struct sp_port *port = nullptr;
    struct sp_port **ports = nullptr;
    bool initialized = false;
    bool xUpdated = false, yUpdated = false, swUpdated = false;
    int xVal = -1, yVal = -1, swVal = -1;
    std::string lineBuffer = "";
    bool completeState = false;
    joystickState currentState;

    void processJoystickValues(std::string lineBuffer) {
        try {
                if (lineBuffer.find("X-axis:") != std::string::npos) {
                    std::cout << "[X Detected] " << lineBuffer << std::endl;
                    size_t colonPos = lineBuffer.find(":");
                    if (colonPos != std::string::npos) {
                        std::string xStr = lineBuffer.substr(colonPos + 1);
                        // Remove all whitespace
                        xStr.erase(remove_if(xStr.begin(), xStr.end(), ::isspace), xStr.end());
                        if (!xStr.empty()) {
                            xVal = std::stoi(xStr);
                            xUpdated = true;
                            std::cout << "[X Parsed] xVal = " << xVal << std::endl;
                        } else {
                            std::cout << "[X Warning] xStr was empty after cleanup!" << std::endl;
                        }
                    } else {
                            std::cout << "[X Warning] Could not find ':' in line: " << lineBuffer << std::endl;
                    }
                }

                if (lineBuffer.find("Y-axis:") != std::string::npos) {
                    std::cout << "[Y Detected] " << lineBuffer << std::endl;
                    std::size_t colonPos = lineBuffer.find(":");
                    if (colonPos != std::string::npos) {
                        std::string yStr = lineBuffer.substr(colonPos + 1);
                        // Remove all whitespace
                        yStr.erase(remove_if(yStr.begin(), yStr.end(), ::isspace), yStr.end());
                        if (!yStr.empty()) {
                            yVal = std::stoi(yStr);
                            yUpdated = true;
                            std::cout << "[Y Parsed] yVal = " << yVal << std::endl;
                        } else {
                            std::cout << "[Y Warning] yStr was empty after cleanup!" << std::endl;
                        }
                    } else {
                        std::cout << "[Y Warning] Could not find ':' in line: " << lineBuffer << std::endl;
                    }
                }

                if (lineBuffer.find("Switch:") != std::string::npos) {
                    std::cout << "[Switch Detected] " << lineBuffer << std::endl;
                    std::size_t colonPos = lineBuffer.find(":");
                    if (colonPos != std::string::npos) {
                        std::string swStr = lineBuffer.substr(colonPos + 1);
                        // Remove all whitespace
                        swStr.erase(remove_if(swStr.begin(), swStr.end(), ::isspace), swStr.end());

                        if (!swStr.empty()) {
                            swVal = std::stoi(swStr);
                            swUpdated = true;
                            std::cout << "[Switch Parsed] swVal = " << swVal << std::endl;
                        } else {
                            std::cout << "[Switch Warning] swStr was empty after cleanup!" << std::endl;
                        }
                    } else {
                        std::cout << "[Switch Warning] Could not find ':' in line: " << lineBuffer << std::endl;
                    }
                }

                if(xUpdated && yUpdated && swUpdated) {
                    currentState = {xVal, yVal, swVal};
                    completeState = true;
                    xUpdated = false;
                    yUpdated = false;
                    swUpdated = false;
                }
            } catch(const std::exception& e) {
                std::cout << "Parse error: " << e.what() << std::endl;
                return;
            }
        }
    

    void readJoystick() {
        if(!port || !initialized) return;
        char readBuffer[256];
        int result = sp_nonblocking_read(port, readBuffer, sizeof(readBuffer)-1);

        if(result > 0) {
            readBuffer[result] = '\0';
            lineBuffer += std::string(readBuffer);

            // Process complete lines
            std::size_t newlinePos;
            while((newlinePos = lineBuffer.find_first_of("\r\n")) != std::string::npos) {
                std::string completeLine = lineBuffer.substr(0, newlinePos);
                lineBuffer = lineBuffer.substr(newlinePos + 1);
            
                // Skip empty lines
                if(completeLine.empty()) continue;

                if(completeLine.find("X-axis:") == std::string::npos && 
                    completeLine.find("Y-axis:") == std::string::npos && 
                    completeLine.find("Switch:") == std::string::npos) {
                    continue;
                }
            
                std::cout << "Processing line: '" << completeLine << "'" << std::endl;
                processJoystickValues(completeLine);
            }
        } else if(result < 0) {
            std::cerr << "Error: Serial reading failure";

        }
    }
};