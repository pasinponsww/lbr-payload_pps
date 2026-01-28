
/**
 * @file pps_helpers.h
 * @brief PPS-specific motor helper functions.
 * @note These functions are specific to the PPS application and should not be part of the generic DC motor interface.
 * @author Bex Saw
 * @date 2026/01/13
 */
#pragma once
#include "motor_support/dc_motor.h"

/**
 * @brief Move the motor in the deploy direction (extend mechanism).
 * @note Used during the Deploy state to drive the mechanism until the deployed limit switch is reached.
 * TODO: Implement DRV#### PWM logic for deployment.
 */
namespace LBR
{
void motorDeploy();
void motorTarget();
void motorRetract();
}  // namespace LBR
