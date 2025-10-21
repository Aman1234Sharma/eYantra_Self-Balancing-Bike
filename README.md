# eYantra Self‑Balancing Bike

**Project:** Self‑Balancing Motorcycle Platform  
**Author:** Aman Sharma — Final Year Mechanical Engineering Student at NIT Allahabad  
**Repository:** [Aman1234Sharma/eYantra_Self‑Balancing‑Bike](https://github.com/Aman1234Sharma/eYantra_Self‑Balancing‑Bike)

---

## 🎯 Overview  
This project is part of the e‑Yantra competition. The goal is to build a self‑balancing two‑wheeled vehicle (bike) that maintains balance using sensors, actuators, and control software — combining mechanical, electronics, and embedded software skills.  

---

## 🛠️ Features & Modules  
- **Task 0: Software Installation and Testing** — Set up the development environment and validate the simulation tools.  
- **Task 1: Simulation** — Virtual prototype of the self‑balancing vehicle using simulation platforms.  
- **Task 3: Starting Stuff** — Initial hardware interface, sensor integration (IMU, gyroscope/accelerometer).  
- **Task 4: Hardware Implementation** — Build of the physical bike platform, motor drivers, control board.  
- **Task 5: Mini Theme Implementation** — Autonomous control for a themed challenge (e.g., path following, obstacle avoidance).  
- **Task 6: Final Theme Implementation** — Complete end‑to‑end system: balancing + theme challenge.  

---

## 🧩 Technologies Used  
- Programming: **C**, **C++**, **Python**  
- ROS2 (Robot Operating System) for system architecture  
- Simulation Tools: Gazebo, CoppeliaSim  
- Embedded / Microcontroller: Arduino or equivalent development board  
- Sensors: Inertial Measurement Unit (IMU), Gyroscope, Accelerometer  
- Actuators & Mechanics: Motors, Drivers, Custom 2‑wheel platform  

---

## 📁 Repository Structure  
```
/
├── Task0_SoftwareInstallationAndTesting/
├── Task1_Simulation/
├── Task3_StartingStuff/
├── Task4_HardwareImplementation/
├── Task5_MiniThemeImplementation/
└── Task6_FinalThemeImplementation/
```
Each folder includes code files, simulation models, hardware schematics (where applicable), and documentation for that task.

---

## 🚀 Getting Started  
### Prerequisites  
- Ubuntu (18.04 / 20.04) or equivalent Linux distribution  
- ROS2 (e.g., Foxy or Humble) installed  
- Gazebo and/or CoppeliaSim installed  
- C++ and Python toolchain  
- Arduino (or compatible board) environment  

### Installation  
1. Clone this repository:
   ```bash
   git clone https://github.com/Aman1234Sharma/eYantra_Self-Balancing-Bike.git
   cd eYantra_Self-Balancing-Bike
   ```
2. Navigate into the appropriate task folder, e.g.:
   ```bash
   cd Task1_Simulation
   ```
3. Install dependencies and build workspace (for ROS2):
   ```bash
   source /opt/ros/<ros2‑distro>/setup.bash
   colcon build
   ```
4. Run simulation or upload firmware as described in that task’s README.

---

## ✅ How You Can Use or Extend It  
- Modify the balancing algorithm (e.g., PID, LQR, or fuzzy logic) for improved performance.  
- Integrate additional sensors (ultrasonic, LiDAR) for obstacle avoidance.  
- Port the simulation to other platforms or implement a full ROS2 node architecture.  
- Design custom mechanical frame for better stability or aesthetics.

---

## 📌 Notes & Acknowledgements  
- This project is part of the e‑Yantra initiative; please abide by their rules if adapting for competition.  
- Thanks to mentors and labs at Indian Institute of Technology Bombay and Indian Institute of Technology Delhi for the internship experience (Control Systems & Autonomous Mobility Robots).  
- Feel free to reach out: **sharmaaman2334@gmail.com** | +91‑9936064502  

---

## 📝 License  
This project is released under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## 🙋‍♂️ Contact  
Aman Sharma  
Mechanical Engineering, Final Year — NIT Allahabad  
📧 sharmaaman2334@gmail.com  
📱 +91 9936064502  
