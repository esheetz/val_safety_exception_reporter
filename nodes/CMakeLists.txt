#-----------------------------------------------------------------------
# Safety Exception Reporter Node:
# for reporting issues during task execution
#-----------------------------------------------------------------------
install(PROGRAMS
	safety_exception_reporter_node.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
