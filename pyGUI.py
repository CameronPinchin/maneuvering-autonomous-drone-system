import tkinter as tk
from tkinter import ttk
import socket
import struct
import threading
import time
import math

# UDP configuration
TEST_IP = "127.0.0.1"  # IP of the C++ program
DRONE_PORT = 5050  # Port to send data to the C++ program
SIMULATOR_PORT = 6005  # Port to receive data from the C++ program
DRONE_DATA_FORMAT = "6f"  # 6 floats (currentX, currentY, currentDirectionX, currentDirectionY, destinationX, destinationY)

# Real drone's initial position
REAL_DRONE_START_X = 10.0
REAL_DRONE_START_Y = 15.0

# Real drone's destination
REAL_DRONE_DEST_X = 350.0
REAL_DRONE_DEST_Y = 350.0

# Drone speed (real drone)
DRONE_SPEED = 1.5

# Simulated drone speed (slightly faster)
SIMULATED_DRONE_SPEED = 1.8

# Threshold for reaching the destination
EPSILON = 1e-6

class SimulationGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        self.gridsize = 5

        # Set window name
        self.title("Drone GUI")

        # Set window size (smaller window)
        self.windowHeight = 400
        self.windowWidth = 400
        geometryString = (str(self.windowWidth) + "x" + str(self.windowHeight))
        self.geometry(geometryString)

        # Create a canvas for the display window
        self.canvas = tk.Canvas(self, bg="white")
        self.canvas.pack(expand=True, fill=tk.BOTH)

        # Draw a grid on the canvas
        self.draw_grid()

        # Create buttons
        self.create_buttons()

        # Initialize drone positions
        self.simulated_drones = []  # List of simulated drones (blue)
        self.real_drone = None  # Real drone (red)

        # Simulated drone data (list of dictionaries for each drone)
        self.simulated_data = self.initialize_simulated_drones()  # Initialize multiple simulated drones

        # Create the simulated drones in the GUI
        for drone_data in self.simulated_data:
            drone = self.create_drone(drone_data["curX"], drone_data["curY"], "blue")
            self.simulated_drones.append(drone)

        # Set up UDP socket for communication with the C++ program
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", SIMULATOR_PORT))  # Listen for updates from the C++ program
        print(f"Simulator listening on 0.0.0.0:{SIMULATOR_PORT}")

        # Start a thread to receive updates from the C++ program
        self.receive_thread = threading.Thread(target=self.receive_updates)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        # Start the simulation
        self.run()

    def draw_grid(self):
        """Draw a grid on the canvas."""
        for i in range(0, self.windowHeight, 20):
            self.canvas.create_line([(i, 0), (i, self.windowHeight)], tag='grid_line', fill='lightgray')
        for i in range(0, self.windowWidth, 20):
            self.canvas.create_line([(0, i), (self.windowWidth, i)], tag='grid_line', fill='lightgray')

    def create_buttons(self):
        """Create control buttons."""
        buttons = ["Run", "Pause", "Add Drone", "Remove Drone", "Change Destination"]
        for btn_text in buttons:
            button = ttk.Button(self, text=btn_text)
            button.pack(side=tk.LEFT, padx=5, pady=5)

    def create_drone(self, x_pos, y_pos, color="blue"):
        """Create a drone on the canvas."""
        return self.canvas.create_oval(x_pos, y_pos, x_pos + 10, y_pos + 10, fill=color)

    def move_drone(self, drone_body, x, y):
        """Move a drone on the canvas."""
        self.canvas.move(drone_body, x, y)

    def initialize_simulated_drones(self):
        """Initialize multiple simulated drones at different positions and destinations."""
        simulated_drones = [
            {
                "curX": 100.0,  # Starting position
                "curY": 100.0,
                "dirX": -1.0,  # Initial direction (not used directly)
                "dirY": 0.0,
                "destX": 300.0,  # Destination
                "destY": 200.0,
            },
            {
                "curX": 50.0,  # Starting position
                "curY": 300.0,
                "dirX": 0.0,
                "dirY": -1.0,
                "destX": 250.0,  # Destination
                "destY": 100.0,
            },
            {
                "curX": 175.0,  # Starting position
                "curY": 50.0,
                "dirX": 0.0,
                "dirY": 1.0,
                "destX": 100.0,  # Destination
                "destY": 350.0,
            },
            {
                "curX": 100.0,  # Starting position
                "curY": 150.0,
                "dirX": 1.0,
                "dirY": -1.0,
                "destX": 350.0,  # Destination
                "destY": 50.0,
            },
        ]
        return simulated_drones

    def update_simulated_drones(self):
        """Update the positions of all simulated drones."""
        for index, drone_data in enumerate(self.simulated_data):
            # Move toward the destination
            dx = drone_data["destX"] - drone_data["curX"]  # destX - curX
            dy = drone_data["destY"] - drone_data["curY"]  # destY - curY
            distance = (dx**2 + dy**2)**0.5

            if distance > 0:
                # Normalize the direction vector and multiply by speed
                dir_x = (dx / distance) * SIMULATED_DRONE_SPEED
                dir_y = (dy / distance) * SIMULATED_DRONE_SPEED

                # Update the drone's position
                drone_data["curX"] += dir_x
                drone_data["curY"] += dir_y

            # Update the drone's position in the GUI
            self.move_drone(self.simulated_drones[index], dir_x, dir_y)

    def run(self):
        """Run the simulation."""
        # Update the simulated drones' positions
        self.update_simulated_drones()

        # Send simulated drone data to the C++ program
        for drone_data in self.simulated_data:
            packed = struct.pack(DRONE_DATA_FORMAT, drone_data["curX"], drone_data["curY"], drone_data["dirX"], drone_data["dirY"], drone_data["destX"], drone_data["destY"])
            self.sock.sendto(packed, (TEST_IP, DRONE_PORT))
            print(f"Sent simulated drone data: {drone_data}")

        # Schedule the next update
        self.after(100, self.run)

    def receive_updates(self):
        """Receive updated positions from the C++ program."""
        prev_real_x, prev_real_y = None, None  # Track the real drone's previous position

        while True:
            data, addr = self.sock.recvfrom(1024)
            if len(data) == struct.calcsize(DRONE_DATA_FORMAT):
                unpacked = struct.unpack(DRONE_DATA_FORMAT, data)
                print(f"Received updated position: ({unpacked[0]:.2f}, {unpacked[1]:.2f})")

                # Check if the real drone has reached its destination
                distance_to_dest = math.sqrt((unpacked[0] - REAL_DRONE_DEST_X)**2 + (unpacked[1] - REAL_DRONE_DEST_Y)**2)
                if distance_to_dest < EPSILON:
                    print("Real drone has reached its destination. Terminating program.")
                    self.stop_simulation()  # Stop the simulation
                    return  # Exit the thread

                # Ensure the real drone stays within the GUI bounds
                if unpacked[0] < 0 or unpacked[0] > self.windowWidth or unpacked[1] < 0 or unpacked[1] > self.windowHeight:
                    print(f"Warning: Real drone is out of bounds! Position: ({unpacked[0]:.2f}, {unpacked[1]:.2f})")
                    continue  # Skip updating the position if it's out of bounds

                # Calculate the movement delta
                if prev_real_x is not None and prev_real_y is not None:
                    delta_x = unpacked[0] - prev_real_x
                    delta_y = unpacked[1] - prev_real_y
                    print(f"Moving real drone by: ({delta_x:.2f}, {delta_y:.2f})")
                else:
                    delta_x, delta_y = 0.0, 0.0  # No movement on the first update

                # Update the "real" drone's position in the GUI
                if not self.real_drone:
                    self.real_drone = self.create_drone(unpacked[0], unpacked[1], "red")
                else:
                    self.move_drone(self.real_drone, delta_x, delta_y)

                # Update the previous position
                prev_real_x, prev_real_y = unpacked[0], unpacked[1]
            else:
                print(f"Received invalid data size: {len(data)} bytes (expected {struct.calcsize(DRONE_DATA_FORMAT)})")

    def stop_simulation(self):
        """Stop the simulation and terminate the program."""
        # Stop updating the simulated drones
        self.after_cancel(self.run)

        # Terminate the program
        self.quit()

def main():
    gui = SimulationGUI()
    gui.mainloop()

if __name__ == "__main__":
    main()