string TASK_UNKNOWN = TaskUnknown

# not actionable action
string action

# unmet pre-condition causing action to be unactionable
string unmet_precondition

# commanded task that action is part of
string commanded_task

# list of remaining actions in task; these cannot be completed until unactionable action is completed
string[] remaining_task_actions
