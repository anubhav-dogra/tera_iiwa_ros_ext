# tera_iiwa_ros_ext

### TO DO:
~~Do check on the robot with out any end-effector  ! ~~

steps:

~~- Do only with the manipulability !~~

~~- Do check the value of wrench uncertainity.~~

- Do with the full opti function

~~- record rosbag for ext torques ~~

- compute covariance matrix !

- re-implement and check ! 

Tried to check tau_cmd - tau_ext
--- Nothing found coz both are equal and opposite in every condition. 

Just check how much torque is off ! 

### Done and Working:
    force_estimator node working, : 
        Computes Wrenches wrt base ! using RBDyn library (for jacobian) and using commanded torques (simulation)

    Cubic_p2p.py Node working: Verified in Gazebo! 
        Computes cubic trajectory (q(t)) for the PositionController mode of the robot. 

    trapezoidal_p2p.py:
        Computed trapezoidal trajectory. CURRENTLY COMPUTING WRONG! 
        
    Moveit_joint_position.py:
        Moveit needs to launch that uses PositionTrajectoryController mode. This node is also initiated using a launch file, that should be defined in the launch folder. (Launch after moveit launch!)

### Changes:
    optimize_test_no_file_sim/real :   there is a change in `cartesian_damping_factors` and `cartesian_damping` between real and sim. 
    similarly: for nullspace
