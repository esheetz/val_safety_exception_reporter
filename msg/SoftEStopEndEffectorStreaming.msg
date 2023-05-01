# end-effector name
string ee_name

# distance between current and desired position
float64 position_distance

# distance between current and desired rotation
float64 rotation_distance

# distance threshold between current and desired positions
float64 position_distance_threshold

# distance threshold between current and desired rotations
float64 rotation_distance_threshold

# desired pose in world frame
geometry_msgs/Point desired_position_in_world
geometry_msgs/Quaternion desired_orientation_in_world

# current pose in world frame
geometry_msgs/Point current_position_in_world
geometry_msgs/Quaternion current_orientation_in_world
