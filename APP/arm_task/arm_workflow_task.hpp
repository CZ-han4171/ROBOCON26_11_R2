#pragma once

// Owns ArmWorkflow and publishes arm_cmd every 5 ms. The command source only
// publishes arm_sequence_cmd; it never calls the planner or the motors.
void armWorkflowTask(void* argument);
