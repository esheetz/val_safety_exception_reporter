# causes of soft e-stops
int32 END_EFFECTOR_STREAMING = 0
int32 JOINT_STATE = 1
int32 JOINT_STATE_DELTA = 2

# list of soft e-stop causes; used to determine which of the remaining message fields are populated with non-empty messages
int32[] soft_estop_causes

# list of end-effector streaming soft e-stop messages
# each message in the list describes one end-effector that is causing the soft e-stop
val_safety_exception_reporter/SoftEStopEndEffectorStreaming[] ee_streaming_msg

# list of joint state soft e-stop messages
# each message in the list describes one joint state that is causing the soft e-stop
val_safety_exception_reporter/SoftEStopJointState[] joint_state_msg

# list of joint state delta e-stop messages
# each message in the list describes one joint state delta that is causing the soft e-stop
val_safety_exception_reporter/SoftEStopJointStateDelta[] joint_state_delta_msg
