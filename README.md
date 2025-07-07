# Real robot in the Robohub

Requires Python 3.7+. In the following it is assumed `python3` points to your python3.7+ executable.

1. Create and activate a Python virtual environment:
```
python3 -m virtualenv ece486_ece687_project_venv
source ece486_ece687_project_venv/bin/activate
```
This step requires the package `virtualenv` to be installed, which can be done via `python3 -m pip install virtualenv`.

2. Install latest release (2.1.0) of the MQTT client package, required to communicate with the backend interfacing with the robots, in order to send commands and receive poses:
```
python3 -m pip install --upgrade pip
python3 -m pip install --upgrade paho-mqtt
```

3. Install numpy and QP solvers, required to develop the optimization-based controller for the mobile manipulator:
```
python3 -m pip install numpy qpsolvers[open_source_solvers]
```

4. Connect to the following Wi-Fi network:
    * SSID: brushbotarium
    * Password: brushbotarium

5. Open the script `test.py` and modify the value of the argument `robot_id` when you call the constructor of the `MobileManipulatorUnicycle` object to match the robot number you would like to use. For example, to use robot 3, modify the line 4 to:
```
robot = MobileManipulatorUnicycle(robot_id=3, backend_server_ip="192.168.0.2")
```

6. Run `test.py` to test the communication with the robot:
```
python3 test.py
```

# Simulator

Only mobile base, no robotic arm, no gripper, no LEDs.

1. Create and activate a Python virtual environment:
```
python3 -m virtualenv ece486_ece687_project_sim_venv
source ece486_ece687_project_sim_venv/bin/activate
```

2. Install numpy, QP solvers, matplotlib, and PyQt5, required to run the simulator and develop the optimization-based controller for the mobile manipulator:
```
python3 -m pip install numpy qpsolvers[open_source_solvers] matplotlib PyQt5
```

3. Open the script `test_sim.py` and uncomment either line 6 or 8 in order to instantiate a simulator with random or specified, respectively, poses of robot and objects. The `robot_id` does not matter for the simulator, but remember to set it when you work in the Robohub.

4. Run `test_sim.py` to test the simulator:
```
python3 test_sim.py
```