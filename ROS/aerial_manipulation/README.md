# aerial_manipulation
This repository contains nodes for interfacing with the DJI Matrice 100 and a 2-DoF (Planar RR) OR 3-DOF (SCARA type) serial manipulator mounted on top the Matrice 100 for contact-inspection tasks.

This README provides a brief overview of the functions performed by individual nodes in this package.

**Author**: Varun Nayak
**Maintainer**: Varun Nayak (varunnayak3101@gmail.com)
**Affiliations**: Autonomous Robots Lab, University of Nevada, Reno; Birla Institute of Technology and Science (BITS), Pilani, Goa Campus, India.

-------------------------------------------------------------

TODO: Parameters values are included in each script. So the **params_am.yaml** in /config is useless as of now.

-------------------------------------------------------------




-------------------***NODES***-------------------


1) **force_from_serial.py**

READS:
Parses data from the serial port connected to the Arduino. The data contains the value of the force experienced at the end-effector
Format available on serial port: "<F_value>"


PUBLISHES:
/force:     Value of force in Newtons (float64)



2) **force_to_mode.py**

SUBSCRIBES TO:
/force: value of force published by the force_from_serial node (float64)

PUBLISHES:
/contact_mode:  as described below (Bool)-->
''
contact_mode == False => Trajectory Tracking Mode
contact_mode == True => Manipulation Mode
''

The mode selection will use Schmitt Trigger switching method for force.



3) **free_flight_trajectory.py**

SUBSCRIBES TO:
/raven/msf_core/odometry:     Pose of Quadrotor COM (in ground frame)   variable name: quadrotor_pose_gf
/contact_mode:          Contact mode as defined in force_to_mode.py
/desired_pose_gf:       Desired Pose of the quadrotor (for transition) (in ground frame)

PUBLISHES:
/raven/command/trajectory:       Tracking of Quadrotor COM to Desired Point only if contact_mode==False



4) **ik_publish_theta.py**

SUBSCRIBES TO:
/wall_position_qf:      Position vector of the point on the wall to be manipulated (in the quadrotor COM frame) (Vector3)

PUBLISHES:
/man_config:    Three float values theta1, theta2 and theta3  in degrees based on inverse_kinematics()


The theta values are calculated using the inverse kinematics. The default values are theta1=0; theta2=30; theta3=-25. TIP: Exaggerate the last link length a little so that the wall point comes into the workspace sooner and a certain pressure is applied (say, by 2-3cm)



5) **pitch_dynamics.py**

SUBSCRIBES TO:
/raven/msf_core/odometry:     Pose of Quadrotor COM in ground frame: to check whether reference has been achieved
/desired_pose_gf:             Pose of Desired state of Quadrotor COM (in ground frame)
/contact_mode:                To check whether the system is in manipulation mode OR free-flight mode

PUBLISHES:
/raven/command/pose:    Pitch angle for manipulation while holding altitude (for mav_control) only if contact_mode == TRUE AND reference_achieved() == TRUE, control input variable name is 'manipulation_pose' (PoseStamped)


The pitch reference angle is calculated using the non-linear passivity based PD controller (refer thesis). Do all this only if it is in contact mode i.e. contact_mode == True



6) **pose_publisher_vicon.py**

PUBLISHES:
/wall_position_qf:      Position of Wall Point expressed in quadrotor COM frame(for IK) (Vector3)
/desired_pose_gf:       Pose of Desired State   in ground frame(for mav_control): This is the pose to be achieved just before manipulation (transition)
/raven/msf_core/odometry:     Pose of Quadrotor COM (in ground frame)   variable name: quadrotor_pose_gf

SUBSCRIBES TO:
VICON data (topic name depends on vicon_node publisher)



7) **theta_to_serial**

SUBSCRIBES TO:
/man_config:    Theta values published by the inverse kinematics engine. (Vector3)

WRITES:
writes the three theta values to the serial port of the NUC (connected to the Arduino/any other MCU) in the format "<theta1,theta2,theta3>". This is a string of information. Eg: Write "<24.52,21.01,54.16>".

The theta values are in DEGREES.


PLEASE IGNORE the node **free_flight_pose** as it was used only for MPC waypoint navigation testing.







----------------------***TOPICS***--------------------------


1)**/force**
The value of the force experienced at the end-effector.
Message type: Float64

2)**/contact_mode**
The mode of the mission. False for free flight and True for manipulation mode. Determined by force value only using Schmitt Trigger Logic.
Message type: Bool

3)**/raven/msf_core/odometry**
The odometry data of the quadrotor COM. Assumed zero covariance for VICON data.
Message type: Odometry

4)**/desired_pose_gf**
Desired Pose of the quadrotor (for transition) COM in ground frame. This is the pose value required just before transition.
Message type: PoseStamped

5)**/raven/command/trajectory**
Desired trajectory of Quadrotor COM. In this implementation, this is published only if contact_mode==False. The final desired state is **/desired_pose_gf**.
Message type: MultiDOFJointTrajectory

6)**/wall_position_qf**
Position vector of the point on the wall to be manipulated (in the quadrotor COM frame)
Message type: Vector3

7)**/man_config**
The configuration of the manipulator. Contains three float values -- theta1, theta2 and theta3 -- in degrees based on the inverse kinematics problem for 2-DoF(RR Planar) OR 3-DoF (RRR SCARA type) manipulator

8)**/raven/command/pose**
This is the pose reference command to the quadrotor (MPC). In this implementation, this is published as the a pose including the desired pitch angle and altitude maintenance for manipulation only if contact_mode==TRUE AND reference_achieved()==TRUE. The name of the control input is **manipulation_pose**
Message type: PoseStamped







-----------------------***STEPS FOR OPERATION***--------------------------




------------------------------------------------------------------------------------------------------------
