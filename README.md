# MoveIt2 Waypoint Follower

This package implements a waypoint navigation node for a UR10 robot. It features a **Hybrid Plannwe** that prioritizes linear Cartesian path using kdl solver but reverts back to joint space planning if a straight line is not feasible due to singularities.

## Key Features

* **Hybrid Solver:**
    1.  **Primary:** Attempts a computed Cartesian path (straight line) first.
    2.  **Fallback:** If the Cartesian path fraction is `< 1.0` (incomplete), it switches to a standard Joint Space (PTP) planner to ensure the target is reached.
* **Real-time Visualization:** Publishes `MarkerArray` topics to RViz for feedback:
    * **Red:** Pending Waypoints
    * **Yellow:** Currently Executing
    * **Green:** Completed
* **Scene :** Spawns a table to simulate the environment, which also demonstrates the obstacle aware planning.

## Prerequisites

* **ROS 2** (Humble)
* **MoveIt 2**

##  Installation & Build

1.  **Workspace Creation**: 
    ```bash
    mkdir -p ~/devrob_ws/src
    cd ~/devrob_ws/src
    git clone -b humble https://github.com/UniversalRobots/universal_Robots_ROS2_Driver.git
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
    git clone https://github.com/jaxxstar/devrob_assessment.git
    ```

2.  **Install dependencies**:
    ```bash
    cd ~/devrob_ws
    rosdep install -r --from-paths src -y --ignore-src
    ```

3.  **Build the package**:
    ```bash
    colcon build 
    source install/setup.bash
    ```


##  Usage

1.  **Launch run_simulation.launch.py**: 
    ```bash
    ros2 launch devrob_assessment run_simulation.launch.py
    ```

## Configuration
* The Waypoints are specified in `config/waypoints.yaml` 