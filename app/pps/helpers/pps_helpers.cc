#include "pps_helpers.h"
#include "board.h"
#include "motor_support/dc_motor.h"

namespace
{
void motorDeployImpl(LBR::Motor& motor)
{
    motor.motorEnable(true);
    motor.motorDirection(true);  // true = deploy direction
    motor.motorSpeed(100);       // Full speed
}

void motorTargetImpl(LBR::Motor& motor)
{
    // Example: move to a target position (implement as needed)
    // motor.moveDegrees(target_degrees, speed);
    motor.motorEnable(true);
    motor.motorDirection(true);  // true = target direction
    motor.motorSpeed(100);       // Full speed
}

void motorRetractImpl(LBR::Motor& motor)
{
    motor.motorEnable(true);
    motor.motorDirection(false);  // false = retract direction
    motor.motorSpeed(100);        // Full speed reverse
}
}  // namespace

namespace LBR
{
void motorDeploy()
{
    motorDeployImpl(*get_board().motor);
}
void motorTarget()
{
    motorTargetImpl(*get_board().motor);
}
void motorRetract()
{
    motorRetractImpl(*get_board().motor);
}
}  // namespace LBR