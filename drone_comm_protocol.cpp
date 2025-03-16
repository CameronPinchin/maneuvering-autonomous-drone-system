#include <iostream>
#include <string>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <sys/socket.h>
#include <atomic>
#include <netinet/in.h>
#include <chrono>
#include <cmath>
#include <mutex>
#include "collision.h" 

#define DRONE_PORT 5050
#define SIMULATOR_PORT 6005

#define DRONE_IP "192.168.83.199"
#define SIMULATOR_IP "192.168.83.131"

#define TEST_IP "127.0.0.1"
#define SIM_TEST_IP "127.0.0.1"

#define EPSILON 1e-6

using namespace std;

//atomics
atomic<bool> shouldRun(true);
atomic<bool> receiverReady(false);
atomic<bool> hasReceivedData(false);
mutex dataMutex;
DroneData currDroneData = {0, 0, 0, 0, 0, 0};

void updatePosition(DroneData& data) {
    float dirToDestX = data.destX - data.curX;
    float dirToDestY = data.destY - data.curY;

    float length = sqrt(dirToDestX * dirToDestX + dirToDestY * dirToDestY);
    if (length > 0.0f) {
        dirToDestX /= length;
        dirToDestY /= length;
    }

    

    // If the drone is already at the destination, stop updating
    if (length < EPSILON) {
        cout << "[Update] Drone has reached its destination: ("
             << data.curX << ", " << data.curY << ")\n";
        return;
    }

    float speed = 1.1f; 
    data.curX += dirToDestX * speed;
    data.curY += dirToDestY * speed;

    if (length < speed) {
        data.curX = data.destX;
        data.curY = data.destY;
        cout << "[Update] Drone has reached its destination: ("
             << data.curX << ", " << data.curY << ")\n";
    }
}

void receive_data() {
    int socketfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketfd < 0) {
        perror("[Receiver] Socket failed");
        return;
    }

    int opt = 1;
    if (setsockopt(socketfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt(SO_REUSEADDR) failed");
        close(socketfd);
        return;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(DRONE_PORT);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY); // Listen on all interfaces

    if (::bind(socketfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("[Receiver] Bind failed");
        close(socketfd);
        return;
    }

    cout << "[Receiver] Ready on port " << DRONE_PORT << endl;
    receiverReady = true;

    GhostDroneData ghostDroneData = {NAN, NAN, NAN, NAN};  // Struct to store received ghost drone data
    struct sockaddr_in senderAddr;
    socklen_t senderLen = sizeof(senderAddr);

     while (!hasReceivedData) {
        ssize_t n = recvfrom(socketfd, &ghostDroneData, sizeof(ghostDroneData), 0, 
                   (struct sockaddr*)&senderAddr, &senderLen);
        if (n == sizeof(ghostDroneData)) {
            // Check for valid data
            if (std::isnan(ghostDroneData.currentX) || std::isnan(ghostDroneData.currentY)) {
                cout << "[Receiver] Invalid ghost drone data received." << endl;
                continue;
            }

            // Valid data received
            hasReceivedData = true;
            cout << "[Receiver] Valid ghost drone data received." << endl;
        }
    }


    while (shouldRun) {
        // Receive ghost drone data from the simulator
        ssize_t n = recvfrom(socketfd, &ghostDroneData, sizeof(ghostDroneData), 0, 
                   (struct sockaddr*)&senderAddr, &senderLen);
        if (n == sizeof(ghostDroneData)) {
            {
                lock_guard<mutex> lock(dataMutex);
                
                 if (std::isnan(ghostDroneData.currentX) || std::isnan(ghostDroneData.currentY) ||
                ghostDroneData.currentX == 0.0 || ghostDroneData.currentY == 0.0) {
                    cout << "[Receiver] Invalid ghost drone data received." << endl;
                    continue;
                }


                float distance = sqrt((currDroneData.destX - currDroneData.curX) * (currDroneData.destX - currDroneData.curX) +
                              (currDroneData.destY - currDroneData.curY) * (currDroneData.destY - currDroneData.curY));
                if (distance < EPSILON) {
                    cout << "[Receiver] Drone has reached its destination. Stopping updates." << endl;
                    continue; // Skip further updates
                }
                
                
                // Use the received ghost drone data for collision detection
                if (checkCollision(currDroneData, ghostDroneData)) {
                    cout << "[Receiver] Collision detected! Adjusting position..." << endl;
                    Vec2 currDroneLocation(currDroneData.curX, currDroneData.curY);
                    Vec2 currDroneDirection(currDroneData.dirX, currDroneData.dirY);
                    Vec2 newPosition = avoidCollision(currDroneLocation, currDroneDirection);

                    currDroneData.curX = newPosition.x;
                    currDroneData.curY = newPosition.y;

                } else {
                    cout << "[Receiver] No collision detected." << endl;
                    updatePosition(currDroneData);
                }

                cout << "[Receiver] Data received!" << endl;
                cout << "[Receiver] Updated position: ("
                     << currDroneData.curX << ", "
                     << currDroneData.curY << ")\n";
            }

            // Send the updated position back to the simulator
            struct sockaddr_in simulatorAddr;
            memset(&simulatorAddr, 0, sizeof(simulatorAddr));
            simulatorAddr.sin_family = AF_INET;
            simulatorAddr.sin_port = htons(SIMULATOR_PORT);
            if (inet_pton(AF_INET, TEST_IP, &simulatorAddr.sin_addr) <= 0) {
                perror("Invalid simulator IP");
                exit(EXIT_FAILURE);
            }

            ssize_t sent = sendto(socketfd, &currDroneData, sizeof(currDroneData), 0, 
                         (struct sockaddr*)&simulatorAddr, sizeof(simulatorAddr));
            if (sent < 0) {
                perror("[Receiver] Failed to send updated data.");
            }
        }
    }

    close(socketfd);
}

void send_data(const DroneData& data) {
    while (!receiverReady) {
        this_thread::sleep_for(chrono::milliseconds(50));
    }

    int socketfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketfd < 0) {
        perror("[Sender] Socket failed");
        return;
    }

    struct sockaddr_in droneAddr;
    memset(&droneAddr, 0, sizeof(droneAddr));
    droneAddr.sin_family = AF_INET;
    droneAddr.sin_port = htons(DRONE_PORT);
    if (inet_pton(AF_INET, TEST_IP, &droneAddr.sin_addr) <= 0) {
        perror("Invalid Drone-IP");
        exit(EXIT_FAILURE);
    }

    ssize_t n = sendto(socketfd, &data, sizeof(data), 0, 
               (struct sockaddr*)&droneAddr, sizeof(droneAddr));
    if (n < 0) {
        perror("[Sender] sendto failed");
    } else {
        cout << "[Sender] Sent " << n << " bytes" << endl;
    }

    // Transmitting data to SIMULATOR
    struct sockaddr_in simulatorAddr;
    memset(&simulatorAddr, 0, sizeof(simulatorAddr));
    simulatorAddr.sin_family = AF_INET;
    simulatorAddr.sin_port = htons(SIMULATOR_PORT);
    if (inet_pton(AF_INET, TEST_IP, &simulatorAddr.sin_addr) <= 0) {
        perror("Invalid simulator IP");
        exit(EXIT_FAILURE);
    }

    n = sendto(socketfd, &data, sizeof(data), 0, 
        (struct sockaddr*)&simulatorAddr, sizeof(simulatorAddr));

    if (n < 0) {
        cerr << "Failed to send data to simulator. " << endl;
    }

    close(socketfd);
}

void stop_receiving() {
    shouldRun = false;
}

int main() {
    thread receiverThread(receive_data);

    // Wait for receiver to be ready
    while (!receiverReady) {
        this_thread::sleep_for(chrono::milliseconds(50));
    }

    {
        lock_guard<mutex> lock(dataMutex);
        currDroneData = DroneData{10.0, 15.0, 0.0, 0.0, 350.0, 350.0};
        cout << currDroneData.curX << " , " << currDroneData.curY << endl;
    }

    send_data(currDroneData);
    cout << "Press Enter to stop..." << endl;
    cin.ignore();
    stop_receiving();
    receiverThread.join();

    return 0;
}