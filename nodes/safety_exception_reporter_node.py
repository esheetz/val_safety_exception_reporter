#!/usr/bin/env python3
"""
Safety Exception Reporter Node
Emily Sheetz, NSTGRO VTE 2023
"""

"""
Tutorial for dynamic_reconfigure for a Python node:
http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28python%29
"""

import rospy
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server
from val_safety_exception_reporter.cfg import SafetyExceptionReporterParamsConfig

# sensing issues
from val_safety_exception_reporter.msg import IgnoringCommands, InvalidCommand, InvalidFeedback, PauseCommand, StopCommand
# planning issues
from val_safety_exception_reporter.msg import AgentNotReady, NotActionable, CannotGetMotionPlan, Collisions, Collision
# acting issues
from val_safety_exception_reporter.msg import CannotExecuteMotion, SoftEStop, SoftEStopEndEffectorStreaming, SoftEStopJointState, SoftEStopJointStateDelta
# generic issue
from val_safety_exception_reporter.msg import GenericIssue

class SafetyExceptionReporterNode:
    def __init__(self):
        # set subscribers for each message type
        self.ignoring_commands_sub      = rospy.Subscriber("valkyrie_safety_reporter/ignoring_commands",
                                                           IgnoringCommands,
                                                           self.ignoring_commands_callback)

        self.invalid_command_sub        = rospy.Subscriber("/valkyrie_safety_reporter/invalid_command",
                                                           InvalidCommand,
                                                           self.invalid_command_callback)

        self.invalid_feedback_sub       = rospy.Subscriber("/valkyrie_safety_reporter/invalid_feedback",
                                                           InvalidFeedback,
                                                           self.invalid_feedback_callback)

        self.pause_command_sub          = rospy.Subscriber("/valkyrie_safety_reporter/pause_command",
                                                           PauseCommand,
                                                           self.pause_command_callback)

        self.stop_command_sub           = rospy.Subscriber("/valkyrie_safety_reporter/stop_command",
                                                           StopCommand,
                                                           self.stop_command_callback)

        self.agent_not_ready_sub        = rospy.Subscriber("/valkyrie_safety_reporter/agent_not_ready",
                                                           AgentNotReady,
                                                           self.agent_not_ready_callback)

        self.not_actionable_sub         = rospy.Subscriber("/valkyrie_safety_reporter/not_actionable",
                                                           NotActionable,
                                                           self.not_actionable_callback)

        self.cannot_get_motion_plan_sub = rospy.Subscriber("/valkyrie_safety_reporter/cannot_get_motion_plan",
                                                           CannotGetMotionPlan,
                                                           self.cannot_get_motion_plan_callback)

        self.collisions_sub             = rospy.Subscriber("/valkyrie_safety_reporter/collisions",
                                                           Collisions,
                                                           self.collisions_callback)

        self.cannot_execute_motion_sub  = rospy.Subscriber("/valkyrie_safety_reporter/cannot_execute_motion",
                                                           CannotExecuteMotion,
                                                           self.cannot_execute_motion_callback)

        self.soft_estop_sub             = rospy.Subscriber("/valkyrie_safety_reporter/soft_estop",
                                                           SoftEStop,
                                                           self.soft_estop_callback)

        self.generic_issue_sub          = rospy.Subscriber("valkyrie_safety_reporter/generic_issue",
                                                           GenericIssue,
                                                           self.generic_issue_callback)

        # set subscriber for operator suggestions
        self.provide_suggestions_sub = rospy.Subscriber("/valkyrie_safety_reporter/provide_operator_suggestions",
                                                        Bool,
                                                        self.provide_suggestions_callback)

        # string for spacing out reports
        self.spacer_stub = "-"

        # string for tabbing reports
        self.tab_str = " " * 4

        # strings for advanced operator report
        self.advanced_operator = ["\n", "CONTROL-LEVEL INFORMATION FOR OPERATOR"] #"Control-level information for operator:"]

        # strings for operator suggestions
        self.suggestion = ["\n", "SUGGESTION FOR OPERATOR"] #"Suggestion for operator:"]

        # flag for providing suggestions
        self.provide_operator_suggestions = True

        # flag for providing control-level info
        self.provide_control_level_info = True

        # set up dynamic reconfigure server
        self.reconfigure_server = Server(SafetyExceptionReporterParamsConfig, self.param_reconfigure_callback)

        rospy.loginfo("[Safety Exception Reporter Node] Node constructed! Ready to report!")

    def param_reconfigure_callback(self, config, level):
        # take params from reconfigure request and store them internally
        self.provide_operator_suggestions = config.operator_suggestions
        self.provide_control_level_info = config.control_level_info

        rospy.loginfo("[Safety Exception Reporter Node] Reconfigured to %sprovide suggestions to operator about resolving issues" %
                      ("" if self.provide_operator_suggestions else "NOT "))
        rospy.loginfo("[Safety Exception Reporter Node] Reconfigured to %sprovide control-level information about issues" %
                      ("" if self.provide_control_level_info else "NOT "))

        return config

    def provide_suggestions_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received request to %sprovide suggestions to operator" % ("" if self.provide_operator_suggestions else "not "))

        # set flag based on message
        self.provide_operator_suggestions = msg.data

        # create report
        report = ""
        if self.provide_operator_suggestions:
            report = "Providing suggestions to operator about resolving issues!"
        else:
            report = "Will not provide suggestions to operator about resolving issues"

        # print report
        self.print_report(report)

        return

    def ignoring_commands_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received ignoring commands message")

        # create report
        report = []
        report.append("Received command: \"" + msg.command_received + "\", but robot is currently ignoring verbal commands.")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("Please tell robot to start listening for verbal commands and repeat command.")

        # print report
        self.print_report(report)

        return

    def invalid_command_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received invalid command message")

        # create report
        report = []
        report.append("Received invalid command: \"" + msg.command_received + "\".")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("Please provide a valid command.")

        # print report
        self.print_report(report)

        return

    def invalid_feedback_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received invalid feedback message")

        # create report
        report = []
        report.append("Received invalid feedback: \"" + msg.feedback_received + "\" while executing commanded task: \"" + msg.commanded_task + "\".")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("Please re-issue task command and provide valid feedback.")

        # print report
        self.print_report(report)

        return

    def pause_command_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received pause command message")

        # create report
        report = "Robot is " + ("pausing" if msg.pause else "resuming") + " execution of commanded task: \"" + msg.commanded_task + "\"."

        # print report
        self.print_report(report)

        return

    def stop_command_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received stop command message")

        # create report
        report = "Robot is stopping execution of commanded task: \"" + msg.commanded_task + "\"."

        # print report
        self.print_report(report)

        return

    def agent_not_ready_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received agent not ready message")

        # get name of agent/user who is not ready
        full_name = ""
        name = ""
        if msg.agent_name == msg.NO_AGENT_NAME:
            full_name = "Agent"
            name = "Agent"
        else: # given some name for the agent
            full_name = "Agent " + msg.agent_name
            name = msg.agent_name

        # create report
        report = []
        report.append(full_name + " is not ready for action: \"" + msg.action + "\".")
        report.append("Robot cannot continue executing task " + msg.commanded_task + " until " + name + " is ready.")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append(name + ", please prepare for robot to execute action " + msg.action + " or abort task.")

        # print report
        self.print_report(report)

        return

    def not_actionable_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received not actionable message")

        # create report
        report = []
        report.append("Robot cannot execute an unactionable action: \"" + msg.action + "\" as part of task " + msg.commanded_task + ".")
        report.append("Action \"" + msg.action + "\" has unmet pre-condition: " + msg.unmet_precondition + ".")
        report.append("Robot cannot continue executing task " + msg.commanded_task + " until pre-conditions are met.")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("Please intervene to satisfy unmet pre-condition " + msg.unmet_precondition + ".")

        # print report
        self.print_report(report)

        return

    def cannot_get_motion_plan_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received cannot get motion plan message")

        # create report
        report = []
        report.append("Robot cannot get a motion plan for group " + msg.planning_group + " due to exception: " + msg.planning_exception + ".")
        if self.provide_control_level_info:
            report = report + self.advanced_operator
            report = self.report_pose_stamped(report, pose_msg=msg.requested_plan_target,
                                              pose_title="Requested motion plan target",
                                              frame_id_title="Planning frame")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("Please address planning exception or change the requested motion plan target pose.")

        # print report
        self.print_report(report)

        return

    def collisions_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received collisions message")

        # create report
        report = []

        for i in range(len(msg.collisions)):
            c_msg = msg.collisions[i]
            report.append("Robot will not be able to motion plan or move due to collision with between body " + c_msg.body1_name + " (of type " + c_msg.body1_type + ") and body " + c_msg.body2_name + " (of type " + c_msg.body2_type + ").")
            if self.provide_control_level_info:
                report = report + self.advanced_operator
                report.append("Collision point:")
                report = self.report_point(report, point_msg=c_msg.collision_point, tab_size=1)
            if i < len(msg.collisions)-1:
                report.append("\n")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("Please intervene to resolve collisions, change the requested motion plan target pose, or change the motion to be executed.")

        # print report
        self.print_report(report)

        return

    def cannot_execute_motion_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received cannot execute motion message")

        # create report
        report = []
        report.append("Robot cannot execute motion due to exception: " + msg.execution_exception + ".")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("Please address execution exception or change the requested motion to be executed.")

        # print report
        self.print_report(report)

        return

    def soft_estop_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received soft e-stop message")

        # check for soft e-stop causes
        if (msg.END_EFFECTOR_STREAMING in msg.soft_estop_causes) or (len(msg.ee_streaming_msg) > 0):
            # report end-effector streaming soft e-stop
            self.process_soft_estop_end_effector_streaming(msg.ee_streaming_msg)
        if (msg.JOINT_STATE in msg.soft_estop_causes) or (len(msg.joint_state_msg) > 0):
            # report joint state soft e-stop
            self.process_soft_estop_joint_state(msg.joint_state_msg)
        if (msg.JOINT_STATE_DELTA in msg.soft_estop_causes) or (len(msg.joint_state_delta_msg) > 0):
            # report joint state delta soft e-stop
            self.process_soft_estop_joint_state_delta(msg.joint_state_delta_msg)

        return

    def process_soft_estop_end_effector_streaming(self, msg_list):
        rospy.loginfo("[Safety Exception Reporter Node] Soft e-stop cause: end-effector streaming")

        # create report
        report = []
        for i in range(len(msg_list)):
            ee_msg = msg_list[i]
            if ee_msg.position_distance >= ee_msg.position_distance_threshold:
                report.append("End-effector " + ee_msg.ee_name + " commanded position is " + str(ee_msg.position_distance) + " away from actual position.")
                report.append("Difference between commanded and actual position is greater than threshold " + str(ee_msg.position_distance_threshold) + ".")
            if ee_msg.rotation_distance >= ee_msg.rotation_distance_threshold:
                report.append("End-effector " + ee_msg.ee_name + " commanded orientation is " + str(ee_msg.rotation_distance) + " away from actual orientation.")
                report.append("Difference between commanded and actual orientation is greater than threshold " + str(ee_msg.rotation_distance_threshold) + ".")
            if self.provide_control_level_info:
                report = report + self.advanced_operator
                report.append("Actual pose of " + ee_msg.ee_name + " (in world frame):")
                report = self.report_point(report, point_msg=ee_msg.current_position_in_world, tab_size=1)
                report = self.report_quaternion(report, quat_msg=ee_msg.current_orientation_in_world, tab_size=1)
                report.append("Commanded pose of " + ee_msg.ee_name + " (in world frame):")
                report = self.report_point(report, point_msg=ee_msg.desired_position_in_world, tab_size=1)
                report = self.report_quaternion(report, quat_msg=ee_msg.desired_orientation_in_world, tab_size=1)
            if i < len(msg_list)-1:
                report.append("\n")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("Large distances between commanded and actual end-effector poses may indicate that the robot is fighting against an obstacle.")
            report.append("Please verify that the robot is not in collision with any objects.")
            report.append("Please slow down while streaming to robot to reduce distance between commanded and actual end-effector poses.")
            report.append("If this end-effector distance should have been allowed, please reconfigure the distance threshold parameters.")

        # print report
        self.print_report(report)

        return

    def process_soft_estop_joint_state(self, msg_list):
        rospy.loginfo("[Safety Exception Reporter Node] Soft e-stop cause: joint state")

        # create report
        report = []
        for i in range(len(msg_list)):
            j_msg = msg_list[i]
            if abs(j_msg.joint_velocity) >= j_msg.velocity_threshold:
                report.append("Joint " + j_msg.joint_name + " velocity is " + str(j_msg.joint_velocity) + ", which is greater than threshold " + str(j_msg.velocity_threshold) + ".")
            if abs(j_msg.joint_torque) >= j_msg.torque_threshold:
                report.append("Joint " + j_msg.joint_name + " torque is " + str(j_msg.joint_torque) + ", which is greater than threshold " + str(j_msg.torque_threshold) + ".")
            if i < len(msg_list)-1:
                report.append("\n")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("High commanded velocities and torques may indicate that the robot is fighting against an obstacle.")
            report.append("Please verify that the robot is not in collision with any objects.")
            report.append("Please reduce joint velocity and torque commands.")
            report.append("If this velocity/torque should have been allowed, please reconfigure the velocity and torque limit parameters.")

        # print report
        self.print_report(report)

        return

    def process_soft_estop_joint_state_delta(self, msg_list):
        rospy.loginfo("[Safety Exception Reporter Node] Soft e-stop cause: change in joint state")

        # create report
        report = []
        for i in range(len(msg_list)):
            j_msg = msg_list[i]
            vel_delta = abs(j_msg.curr_joint_velocity - j_msg.prev_joint_velocity)
            trq_delta = abs(j_msg.curr_joint_torque - j_msg.prev_joint_torque)
            if vel_delta >= j_msg.velocity_delta_threshold:
                report.append("Joint " + j_msg.joint_name + " has change in velocity of " + str(vel_delta) + ", which is greater than threshold " + str(j_msg.velocity_delta_threshold) + ".")
            if trq_delta >= j_msg.torque_delta_threshold:
                report.append("Joint " + j_msg.joint_name + " has change in torque of " + str(trq_delta) + ", which is greater than threshold " + str(j_msg.torque_delta_threshold) + ".")
            if self.provide_control_level_info:
                report = report + self.advanced_operator
                # create padding based on longest string
                pad_size = len("Previous velocity of joint " + j_msg.joint_name + ": ")
                report.append(("Previous velocity of joint " + j_msg.joint_name + ": ").ljust(pad_size) +
                              str(j_msg.prev_joint_velocity))
                report.append(("Current velocity of joint " + j_msg.joint_name + ": ").ljust(pad_size) + 
                              str(j_msg.curr_joint_velocity))
                report.append(("Previous torque of joint " + j_msg.joint_name + ": ").ljust(pad_size) + 
                              str(j_msg.prev_joint_torque))
                report.append(("Current torque of joint " + j_msg.joint_name + ": ").ljust(pad_size) + 
                              str(j_msg.curr_joint_torque))
            if i < len(msg_list)-1:
                report.append("\n")
        if self.provide_operator_suggestions:
            report = report + self.suggestion
            report.append("High changes in joint velocities and torques may indicate that the robot is fighting against an obstacle.")
            report.append("Please verify that the robot is not in collision with any objects.")
            report.append("Please reduce how quickly robot is commanded to move to reduce changes in joint velocities and torques.")
            report.append("If this change in velocity/torque should have been allowed, please reconfigure the change in velocity and change in torque limit parameters.")

        # print report
        self.print_report(report)

        return

    def generic_issue_callback(self, msg):
        rospy.loginfo("[Safety Exception Reporter Node] Received generic issue message")

        # create report
        report = []
        report.append("General safety issue: " + msg.issue_report + ".")
        if self.provide_control_level_info and msg.has_control_info:
            report = report + self.advanced_operator
            report.append(msg.control_info)
        if self.provide_operator_suggestions and msg.has_suggestion:
            report = report + self.suggestion
            report.append(msg.suggestion)

        # print report
        self.print_report(report)

        return

    def print_report(self, report):
        if isinstance(report, str):
            self.print_single_line_report(report)
        elif isinstance(report, list):
            self.print_multi_line_report(report)
        else: # should not get here
            rospy.logerr("[Safety Exception Reporter Node] Unrecognized message string type")
            print("Got message string of type " + str(type(report)) + ", but expected type list or string")
        
        return

    def print_single_line_report(self, report):
        print(self.spacer_stub * len(report))
        print(report)
        print(self.spacer_stub * len(report))

        return

    def print_multi_line_report(self, report):
        # get longest line of text
        longest_str = self.find_longest_text_string(report)

        print(self.spacer_stub * len(longest_str))
        for line in report:
            print(line)
        print(self.spacer_stub * len(longest_str))

        return

    def find_longest_text_string(self, str_list):
        # initialize longest string
        longest_str = str_list[0]

        # look through list of strings
        for item in str_list:
            # check for longer string than current longest
            if len(item) > len(longest_str):
                longest_str = item

        return longest_str

    def report_pose_stamped(self, report, pose_msg, pose_title, frame_id_title):
        report.append(pose_title + ":")
        report.append(self.tab_str + frame_id_title + ": " + pose_msg.header.frame_id)
        report.append(self.tab_str + "Pose:")
        report = self.report_point(report, point_msg=pose_msg.pose.position)
        report = self.report_quaternion(report, quat_msg=pose_msg.pose.orientation)

        return report

    def report_point(self, report, point_msg, tab_size=2):
        report.append((self.tab_str * tab_size) + "Position:")
        report.append((self.tab_str * (tab_size+1)) + "x: " + str(point_msg.x))
        report.append((self.tab_str * (tab_size+1)) + "y: " + str(point_msg.y))
        report.append((self.tab_str * (tab_size+1)) + "z: " + str(point_msg.z))

        return report

    def report_quaternion(self, report, quat_msg, tab_size=2):
        report.append((self.tab_str * tab_size) + "Orientation:")
        report.append((self.tab_str * (tab_size+1)) + "x: " + str(quat_msg.x))
        report.append((self.tab_str * (tab_size+1)) + "y: " + str(quat_msg.y))
        report.append((self.tab_str * (tab_size+1)) + "z: " + str(quat_msg.z))
        report.append((self.tab_str * (tab_size+1)) + "w: " + str(quat_msg.w))

        return report

if __name__ == '__main__':
    # initialize node
    rospy.init_node("ValkyrieSafetyExceptionReporterNode")

    # create reporter node
    reporter_node = SafetyExceptionReporterNode()

    # run node, wait for reports
    while not rospy.is_shutdown():
        rospy.spin()

    rospy.loginfo("[Safety Exception Reporter Node] Node stopped, all done!")
