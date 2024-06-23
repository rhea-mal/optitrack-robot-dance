# Humanoid Robotic Motion Mapping from OptiTrack for Visualizations in Coordinated Motion
## Stanford Robotics Center - Human Fluid Dance Motion Mapping to Toro Robots
Simulating and visualizing the coordinated motion of humanoid robots using data from OptiTrack motion capture systems. The simulation includes multiple humanoid robots (toros) replicating human fluid dance, showcasing synchronized and coordinated robotic motions.
<img width="769" alt="Screenshot 2024-06-23 at 3 17 59 PM" src="https://github.com/rhea-mal/optitrack-robot-dance/assets/70975260/6125dd9c-c260-43d8-b6d0-203dfb07f9aa">

## Setup and Installation

### Step 1: OptiTrack Setup

1. **Connect Laptop**:
   - Connect your laptop to the white ethernet cable.
   - Open the Motive software and power on the OptiTrack cameras.

2. **Create Rigid Bodies**:
   - Under "Assets", box-select a group of markers, right-click, and create a new rigid body.
   - Rename and reorder the rigid bodies in the Assets tab.
   - Initialization Order: Head, Left leg, Left hand, Torso, Right leg, Right Hand.
   - Ensure a minimum of 4 trackers per rigid body.

3. **Streaming Settings**:
   - Go to Edit -> Settings, then click the Streaming tab.
   - Change the IP to `172.24.68.48` in the drop-down tab related to the IP address.
   - Ensure that the streaming setting is Unicast, and that Rigid Bodies and Markers (labeled and unlabeled) are ticked.
   - Enable streaming.

4. **Turn Off Cameras Before Disconnecting**:
   - Before turning off or disconnecting the session, turn off all OptiTrack cameras in Motive's left panel under devices by unchecking 'Enable'.

### Step 2: Local Computer Setup

1. **Set IP Address**:
   - Manually set the IP of your laptop to `172.24.68.64` (System Preferences -> Network).

2. **Build the Project**:
   ```sh
   mkdir build
   cd build
   cmake .. && make -j4
   ```

  If you modify the code, delete the build directory before rebuilding:
  ```rm -rf build```

3. **Stream Data**:
   ```cd optitrack
   python3 StreamData.py
   ```

4. **Run Simviz-Stage.cpp**:
  ```cd bin/optitrack
   ./simviz-stage
  ```
   
6. **Run Toro-controller.cpp**:
   ```./toro-controller
   ```

### Step 3: Visualization Effects
1. **Chose # of Robots **:
   - Currently 1, 3, or 10
   - Set NUM_ROBOTS constant in simviz.cpp

2. **Chose background & stage textures**:
   - Chose jpg from "Assets" and set background in simviz
   - Change stage texture in urdf file
   - working on front end tool for this

3. **Delay**:
   - Staggered motion by setting DELAY flag to true
   - Modify simulation loop to pause between successive mappings (default set to 0.3 seconds)
