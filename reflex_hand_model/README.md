# urdf_controller_test
Package for display URDF with controller for hand, for visualization using RViz (and Gazebo later)

To Run RViz with specific model, do: 

	roslaunch urdf_display display_urdf.launch model:='$(find urdf_display)/urdf/02-myfirst_hand.urdf'

To Run Gazebo with a specific model, do: 

	roslaunch urdf_display gazebo.launch model:='$(find urdf_display)/urdf/02-myfirst_hand.urdf'

To Run Reflex Hand model with controller, do 
	
	roslaunch urdf_display joints.launch

To pub to Gazebo to move:

	rostopic pub  /r2d2_head_controller/command std_msgs/Float64MultiArray "layout:
	  dim:
	  - label: ''
	    size: 9
	    stride: 1
	  data_offset: 0
	data: [0.5, 0.5, 0.5, -0.5, 0.5, 0.0, 0, 0.0, 0.0]"

Data structure data[0] to data[2] is f1, f2, f3. data[3] to data[5] is preshape for f1, f2, f3. data[6] to data[8] is for flexible joints.

We can also control the hand model by publishing a reflex_msgs.msg.Hand message to /reflex_sf/hand_state. The script in src for the model to subscribe to /reflex_sf/hand_state to receive a reflex_msgs.msg.Hand message type. The script will translate reflex_msgs.msg.Hand information into std_msgs/Float64MultiArray and publishing it to /r2d2_head_controller/command for visualizing the model in gazebo.  
