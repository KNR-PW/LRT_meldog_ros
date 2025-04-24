# Package for testing single 2DOF meldog leg including:

# Instruction for launching demos:
1. Make all necessary changes to launch files (parameters are defined in __UPPERCASE__)
2. In directory ```meldog-ros/```: ```colcon build --packages-select meldog_leg_tests```
3. Run ```ros2 launch meldog_leg_tests moteus_controller.launch.py``` and wait for successful activation
4. Run suitable launch file:
  1. Linear motion: ```ros2 launch meldog_leg_tests linear_trajectory_demo.launch.py```
  2. Circle motion: ```ros2 launch meldog_leg_tests circle_trajectory_demo.launch.py```
  3. Jumping: ```ros2 launch meldog_leg_tests jumping_demo.launch.py```
  4. Simple sinusoidal movement for single motor: ```ros2 launch meldog_leg_tests demo_single_motor.launch.py```

# Nodes:

## Moteus nodes:
__Controller nodes__:
- ```multi_moteus_controller_single_thread.py```
- ```multi_moteus_controller_two_threads.py```
##
__Demo for single motor__:
- ```demo_single_motor.py```
## Nodes for 2DOF leg:
__Forward and inverse kinematics__:
##
- ```forward_kinematics_2D.py```
- ```inverse_kinematics_2D.py```
##
__Linear motion in y axis__:

- ```linear_trajectory.py```
##
__Circle motion__:

- ```circle_trajectory.py```
##
__Jumping using precalculated data__:

- ```jump_control_node.py```
- ```jump_data/```
##
__Simple plotting__:

- ```moteus_ploter.py```

# Topics:
-  ```multi_moteus_control```: Control topic for moteuses (```MultiMoteusControl```)
-  ```multi_moteus_state```: State topic for moteuses (```MultiMoteusState```)
-  ```end_effector_desired_trajectory```: trajectory of tip of the leg (```Vector3```)
-  ```end_effector_actual_trajectory```: trajectory of tip of the leg (```Vector3```)
# Custom messages:
-  ```MoteusControl```: message for sending desired state to one moteus controller
-  ```MultiMoteusControl```: array of ```MoteusControl```
-  ```MoteusState```: message for receiving actual state of one moteus controller
-  ```MultiMoteusState```: array of ```MoteusState```
