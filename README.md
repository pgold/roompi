RoomPi
======

A Roomba/Raspberry Pi ROS 2 robot.

### Building


1. Create a workspace:
    ``` bash
    $ mkdir -p workspace/src
    $ cd workspace
    ```

2. Clone this repository:
    ``` bash
    $ git clone https://github.com/pgold/roompi.git src/roompi
    ```
    
3. Fetch source dependencies (using vcstool):
    ``` bash
    $ vcs import src < src/roompi/robot_src_deps.repos
    ```
    
    `vcs` is required for this step -- it can be installed by running:
    ``` bash
    $ sudo apt install python3-vcstool
    ```

4. Install ROS dependencies:
    ``` bash
    $ rosdep update
    $ rosdep install --from-paths src -i
    ```
    
5. Build:
    ``` bash
    $ colcon build
    ```