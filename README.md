# Valkyrie Safety Exception Reporter
Reports common safety conditions and task execution issues for NASA's Valkyrie robot.

This package contains a node that process different issues that may arise during task execution and prints messages to describe the issue.  Nodes may publish messages of the appropriate type to allow this node to report the issue.  The package depends on `dynamic_reconfigure`.

To launch the safety reporter, run:
```
roslaunch val_safety_exception_reporter safety_exception_reporter.launch
```

## Important Note
This node _only handles exception reporting_, not pausing/stopping behaviors.  Whatever node reports the issue should immediately handle pausing or stopping the violating behaviors.  This ensures that:

1.  Any violating behaviors are stopped more promptly, without having to send additional messages before pausing/stopping any robot behaviors.
2.  This node just focuses on reporting issues to the user.  By not putting any logic to pause/stop behaviors in this node, this node becomes completely optional when the robot is performing tasks.
