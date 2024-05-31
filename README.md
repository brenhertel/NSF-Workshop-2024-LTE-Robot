# NSF-Workshop-2024-LTE-Robot

This repository contains instructions and files for the robot demonstration of the workshop. To prepare everything for the demo, perform the following steps:
1. Turn the robot on.
2. While the robot is booting up, turn the computer on.
3. Log in to the computer (make sure you log into the ubuntu side).
4. In the computer, open up a terminal.
5. In the terminal, type `cd ~/catkin_ws`.
6. In the terminal, type `source devel/setup.bash`.
7. In the terminal, type `roscore`. This starts a ROS master which other ROS nodes will connect to.
8. Open up a new terminal and type `source devel/setup.bash`.
9. Check that the robot is fully up. Once it is, power on the arm by pressing the red button, then pressing `On`.
10. Check that the robot is connected to the network. This can be done by clicking the hamburger menu in the top right, then going to the `About` page. It should show an IP address.
11. Once the robot is connected to the network, in the terminal type `roslaunch pearl_ur5e robot.launch`. This will bring up all necessary nodes to connect to the robot. Note: if this launch fails, check the connection on the robot and check that the ip address in the `robot.launch` file is correct.
12. On the robot, start the `external_control` program. (It might just be called `ext_control`, it should be brought up automatically).
13. Switch the robot into `remote control` mode. This can be done by clicking the mode in the top right (should currently be on `Automatic`) then clicking on the desired mode. Once in Remote Control, you should see a message in the terminal that `Robot is ready to recieve control commands`. If you do not, check that the ip in the `external_control` program is correct. To check and/or change the ip address in the program, you must be in `Manual` mode on the robot. To switch into this mode you will need a password, the password for the robot is `Pearlur5e`.
14. On the computer, open up a new terminal and type `source devel/setup.bash`.
15. In this terminal, run the `lte_workshop_demo.py` file when ready for a demo. Be sure to replace the `lte.py` file with each student's file before each demo. This demo file will go through multiple steps.
    - First, the user will perform a demonstration. To begin recording the demonstration, press [Enter] once the prompt is available.
    - Next, the user will put in new constraints. I recommend this is done by moving the robot to a point and looking at `rostopic echo \tf` to see the robot position.
    - Finally, a reproduction is performed according to LTE. Be careful during the reproduction and always be ready to press the emergency stop in case.
