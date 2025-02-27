# Instruction Manual of Ophthalmology System
<div align="center">
<img src = "Demonstration Images/nml.png" width = "400pt">   
</div>

# <img src = "Demonstration Images/instruction.jpg" width = "50pt"> About this Instruction Manual<a name = "Main0"></a>

The Ophthalmology System is currently under development and the instruction manual will be updated over time. There are currently a few different computer systems being utilized which are all connected via ROS. The current goal is to develop a method to peel the Inner Limiting Membrane (ILM) along the retina of the eyeball. The Smart Arm System (SAS) is used to control the Denso robots. The phantom eye model utilizes the bionic-eye. 

# Computer Systems
## Forceps
 The forceps computer system utilizes x2 PCI cards (I/O and Encoder) that connect to the forceps control box. This computer may be updated over time as it is quite old and can be integrated to one of the main computers by purchasing equivalent PCIe cards. 

The forceps system can be buggy, so patience is key to diagnose these issues. Also ensure that the min and max positioning is correctly set. The wiring may also tangle if the forceps device rotates enough. It is best to unwind the wiring as it can cause tension on the wiring and soldering which may lead to open circuits.
### Initialization
** **BE PREPARED TO TURN OFF THE FORCEPS VIA SWITCH** ** Reference Debugging section for more details.<br>
**Launch the ROS node** <br>
`roslaunch opthalmic_control oph_robot_driver_maxon.launch --screen` <br>
On a fresh launch, `no initial values obtained.` will be displayed. Exit the program and relaunch so both channels are initialized.  <br><br>
**Opens the UI to control the escon forceps:** <br> 
`rosrun sas_robot_driver_escon escon_operation_check_node` <br>
Ensure this is closed once you control the forceps via script.<br><br>
**Reset the initial joint positions:** <br>
`rosparam delete /escon_initial_joint_poisitions`

### Debugging
- **The forceps continuously rotates** - This is due to the maxon controller having a calibration voltage of 2.5V. The voltage from the PCI encoder may output a constant 5V signal and the "position" via check node may be increasing continuously. Try resetting the initial positions and then turn on the forceps. Run the node after to see results.
- **The motor does not spin** - The controller may be faulting which can be indicated be the red light on the PCB board. Via EPOS software, press the "Fault Reset" button and then the "Enabled" button (turning off and on via switch may work as well). Ensure that the motor is set to velocity mode and that the execution mask is enabled as this will allow escon to control the motors
- **The forceps stops spinning or grasping is stuck** - The minimum or maximum positions may be reached as per escon parameter settings. Reset initial joint positions or adjust min and max position values accordingly.

## Patient Side
The master computer that runs via Ubuntu 20.04

## Operator Side
The haptic devices are run on this windows computer. Initiate the SmartArmMaster executable file. The default settings can be adjusted via JSON file. The parameters for the current setup should be as follows:
- Motion Scaling: 30
- Linear viscosity  0.0001
- Linear stiffness: 30
- Delay: 0 <br>

Ensure the IP and Ports are correct if there are issues connecting.

## Microscope with camera
A PCIe card is utilized to operate the overhead 4K microscopic camera.

# Simulation
## Robot Teleoperation
1. Launch operator side reciever on program on the windows computer
2. Open VREP (CoppeliaSim) and the **eye_ball_manipulation_forecps_v2.ttt** scene
3. Initiate ROS: `roscore`
4. **Set RCM Positions:**<br>
`roslaunch robot_control set_rcm_positions.launch --screen` 
5. **Launch Simulation Teleoperation Program:**<br>
`roslaunch robot_control run_simulation.launch --screen`

# Calibration
Operating real robots can be dangerous. **Be cautious and careful!** <br>
*Currently, the instrument arm is calibrated to 6DoF as it assumes that the translation of the end effector does not change when the 7th joint rotates.  <br>

## Instructions
1. Ensure both robots are set to the auto setting and the red emergency buttons are released
2. **Be ready** with the master emergency switch, launch the robots and calibration program with:

    **Calibration launch command:** <br>
`roslaunch robot_calibration calibrate_end_effector.launch --screen`
<br> 
<div align="center">
<img src = "Demonstration Images/cali_ui.png" width = "450pt"> 
</div> <br>

3. Set the robots back to manual
4. Translate the end effector to the tip of the screwdriver
5. Press the "Add Entry" button
6. Continue to add entries by translating the robot to different poses without moving the screwdriver until satisfied: <br>
    **- Quick Demonstration - 3 entries or more** <br>
    **- Experiment - 8 entries or more**
7. Press the "Calculate" button to calculate the distance vector of the end effector and write to the respective yaml file
8. Press the "Update Calibration File" to update the respective calibrated json file
9. Select the entries and delete them from the list
10. Continue the same process for the other arm

# Real Robot
Operating real robots can be dangerous. **Be cautious and careful!**<br>
You will need another person to be ready to hit the emergency stop upon robot initiation and when operating the robots. 
## System Setup
1. Calibrate the robots
2. Setup the phantom model
3. Manually insert the tool tips 0.5 cm into the rcm points


## Instructions
1. Launch operator side reciever on program on the windows computer
2. Open VREP (CoppeliaSim) and the **eye_ball_manipulation_forecps_v2.ttt** scene
3. **Set RCM Positions:**<br>
`roslaunch robot_control set_rcm_positions.launch --screen`
4. First test through simulation: <br>
**Launch Simulation Teleoperation Program:** <br>
`roslaunch robot_control run_simulation.launch --screen`
5. Check that the settings are correct for `control_parameters.py` and `physical_parameters.py`
6. Get ready to press the emergency stop button and launch the real teleoperation program: <br>
    **Launch Real Teleoperation Program:** <br>
`roslaunch robot_control run_real.launch --screen`
7. Upon new experiment, return to the initial pose: <br>
`rosrun return_to_initial_poses_orbital.py`
8. Once finished, eject the instruments out of the phantom eyeball:
`rosrun eject_instruments.py`
## Arduino Contact Reporter
Arduino board wired to a PCB to notify the user of contact between the instrument and retinal surface. Used for testing of the teleoperation and auto positioning programs. <br>
**Allow the Arduino to connect the PC:** <br>
`sudo chmod 777 /dev/ttyUSB0` <br>
**Launch the contact reporter program:** <br>
`roslaunch robot_control arduino_contact_reporter.launch --screen`
## Video setup
The computer will record using the SimpleScreenRecorder Application. Set the window or area to record accordingly. <br>
### Simple Image Capture
Simple Image capturing program that allows the operator to view the workspace. The arduino contact node is available to use in this program. <br>

1. Turn on the microscopic camera
2. (optional) Turn on the arduino contact node
3. Launch the the simple image program: <br>
`roslaunch robot_control simple_image_capture.launch`

### Keypoint Detection Capture
Keypoint Detection capturing program allows for semi-autonomous or autonomous positioning that allows the operator to view the workspace view along with important keypoints displayed, such as the instrument tip, shadow tip, shaft point, ROI, and more.  <br>

1. Turn on the microscopic camera
2. (optional) Turn on the arduino contact node
3. Launch the the simple image program: <br> 
`roslaunch robot_control image_processing_node.launch` <br> <br>
# Troubleshooting
## Robots <br>
Robots are giving an error on the screen that it cannot connect to the computer
- Ensure the emergency stop buttons are not pressed
- Try physically restarting the robots
