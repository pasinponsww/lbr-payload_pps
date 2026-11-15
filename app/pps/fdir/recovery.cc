#include "recovery.h"

namespace LBR
{

Recovery::Recovery(Motor& motor, const RecoveryConfig& config)
    : motor_(motor), config_(config)
{
}

bool Recovery::begin(const Plan& plan, uint32_t now_ms)
{
    plan_ = plan;
    failure_ = Fault::None;
    command_failed_ = false;
    advance(Phase::Quiesce, now_ms);
    return motor_.standby();
}

Recovery::Outcome Recovery::step(uint32_t now_ms)
{
    const uint32_t elapsed = now_ms - phase_ms_;

    switch (phase_)
    {
        case Phase::Quiesce:
            if (elapsed >= plan_.backoff_ms)
            {
                drive(0);
                advance(
                    plan_.unjam_direction != 0 ? Phase::Unjam : Phase::Verify,
                    now_ms);
            }
            return Outcome::Busy;

        case Phase::Unjam:
            drive(plan_.unjam_direction * config_.unjam_duty);
            if (elapsed >= config_.unjam_ms)
            {
                drive(0);
                advance(Phase::Verify, now_ms);
            }
            return Outcome::Busy;

        case Phase::Verify:
            if (elapsed < config_.verify_ms)
            {
                return Outcome::Busy;
            }
            if (motor_.encoder_fault())
            {
                failure_ = Fault::EncoderFault;
                return Outcome::Failed;
            }
            if (motor_.driver_fault() || command_failed_)
            {
                failure_ = Fault::DriverFault;
                return Outcome::Failed;
            }
            return Outcome::Cleared;
    }
    return Outcome::Busy;
}

Fault Recovery::failure() const
{
    return failure_;
}

void Recovery::advance(Phase next, uint32_t now_ms)
{
    phase_ = next;
    phase_ms_ = now_ms;
}

bool Recovery::drive(int duty)
{
    const bool ok = motor_.drive(duty);
    command_failed_ = command_failed_ || !ok;
    return ok;
}

}  // namespace LBR
