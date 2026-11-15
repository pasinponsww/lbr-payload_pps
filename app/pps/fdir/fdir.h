#pragma once
/**
 * @file fdir.h
 * @brief Fault detection, isolation and recovery policy for the PPS motor axis
 * @note Detection watches one supervised motion at a time. The policy decides
 *       between retrying, falling back to a safer goal, or latching safe mode.
 * @author Bex Saw
 * @date 2026/11/15
 */

#include <cstdint>

namespace LBR
{

enum class Fault : uint8_t
{
    None = 0,
    DriverFault,     ///< DRV8245 nFAULT asserted
    EncoderFault,    ///< encoder reports an error, position is untrusted
    Stall,           ///< driven but the encoder is not moving
    WrongDirection,  ///< encoder moves against the commanded direction
    Timeout,         ///< motion did not finish inside its time budget
    LimitMismatch    ///< limit switch disagrees with the encoder position
};

enum class Response : uint8_t
{
    Retry = 0,     ///< back off, clear the fault, resume the interrupted motion
    Fallback,  ///< give up on the goal and go to the safer one
    Safe       ///< stop driving and latch until reset
};

/**
 * @struct FdirConfig
 * @brief Detection thresholds and retry policy
 */
struct FdirConfig
{
    int stall_min_duty{20};       ///< below this duty a stall is not judged
    int32_t stall_min_counts{4};  ///< progress required per stall window
    uint32_t stall_ms{400};       ///< stall window length
    int32_t reverse_counts{60};   ///< wrong-way travel that trips the fault
    uint8_t max_retries{3};       ///< retries per goal before falling back
    uint32_t backoff_ms{250};     ///< first backoff, doubles on every retry
};

/**
 * @struct MotionSample
 * @brief One control-tick snapshot of the axis
 */
struct MotionSample
{
    uint32_t now_ms;
    int32_t position;
    int duty;
    bool driver_fault;
    bool encoder_fault;
};

class Fdir
{
public:
    explicit Fdir(const FdirConfig& config);

    /**
     * @brief Start supervising a motion
     * @param budget_ms time the motion is allowed to take
     */
    void arm(uint32_t now_ms, int32_t position, uint32_t budget_ms);

    /**
     * @brief Detection: evaluate one sample of the supervised motion
     * @return the first fault found, Fault::None when healthy
     */
    Fault monitor(const MotionSample& sample);

    /**
     * @brief Isolation and policy: record the fault and pick the response
     */
    Response respond(Fault fault);

    /** @brief Quiet time to wait before the current retry */
    uint32_t backoff_ms() const;

    /** @brief Forget retry history after a goal succeeds or changes */
    void clear_retries();

    /** @brief Clear retries and the latched fault */
    void reset();

    /** @brief Most recent fault, kept until reset */
    Fault last_fault() const;

    /** @brief Retries spent on the current goal */
    uint8_t retries() const;

private:
    FdirConfig config_;
    Fault last_fault_{Fault::None};
    uint8_t retries_{0};
    uint32_t armed_ms_{0};
    uint32_t budget_ms_{0};
    uint32_t window_ms_{0};
    int32_t window_position_{0};
    int32_t reverse_travel_{0};
};

/** @brief Short printable name of a fault */
const char* to_string(Fault fault);

}  // namespace LBR
