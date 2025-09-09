# FleetController V0.2

A more advanced and realistic **3-UAV fleet controller** using ROS2 and MAVROS. This version implements a **triangular formation** and supports leader-follower coordination.

## Features

- Controls **3 UAVs** flying in a triangular formation.
- One UAV acts as the **leader** (can be changed dynamically).
- Leader receives setpoints directly, while **followers subscribe to the leader's setpoints**.
- Terminal commands allow you to:
  - **Set flight modes**  
  - **Move drones in real-time**
- Consists of two main scripts:
  - `drone_mc`: handles individual UAV control  
  - `ground_control`: manages fleet coordination and leader assignment  

## How It Works

1. **Leader-Follower Structure**  
   - The leader drone receives position/velocity commands.  
   - Followers automatically track the leader using ROS2 subscriptions.  

2. **Triangular Formation**  
   - Maintains a stable triangular formation around the leader.  
   - Allows dynamic formation adjustments if the leader changes.

3. **Modes & Commands**  
   - Set the mode of each drone (e.g., OFFBOARD, LAND, ARM).  
   - Move drones by publishing setpoints to the leader.  

