Each lines of command should inside baxter shell and soure

First set motor setting and make sure you change the mode of USB port.

roslaunch baxter_pickup_cup controller_manager.launch

roslaunch baxter_pickup_cup start_tilt_controller.launch

rostopic echo /tilt_controller/state (check current_pos data. current position should be -33)

roslaunch baxter_pickup_cup kinect2_bridge.launch

rosrun baxter_pickup_cup cup_detector.py

rosrun baxter_pickup_cup location_convertor_three_cups.py

rosrun baxter_pickup_cup baxter_left_arm_move_new.py
