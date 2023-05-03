# types of bodies
string ROBOT_LINK = robot_link
string ROBOT_ATTACHED = robot_attached
string WORLD_OBJECT = world_object

# name of first body involved in collision
string body1_name

# name of second body involved in collision
string body2_name

# type of first body involved in collision
string body1_type

# type of second body involved in collision
string body2_type

# point of collision
geometry_msgs/Point collision_point
