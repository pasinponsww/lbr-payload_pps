#include "board.h"
#include "console/console.h"
#include "pps.h"

namespace
{
constexpr uint32_t kControlPeriodMs = 10;
constexpr uint32_t kHeartbeatTicks = 50;

void indicate(LBR::Board& board, const LBR::PpsStatus& status, uint32_t tick)
{
    const bool faulted = status.state == LBR::PpsState::Recovery ||
                         status.state == LBR::PpsState::Safed;
    board.led_red.set(faulted);
    board.led_green.set(status.state == LBR::PpsState::Holding);
    if (tick % kHeartbeatTicks == 0)
    {
        board.led_blue.toggle();
    }
}
}  // namespace

int main()
{
    const bool board_ok = LBR::bsp_init();
    LBR::Board& board = LBR::get_board();

    LBR::Pps pps(board.motor, board.limit_retracted, board.limit_extended);
    LBR::Console console(board.console, pps);

    // A board that failed bring-up stays safed but still answers the console
    if (!board_ok)
    {
        pps.command(LBR::PpsCommand::Abort);
    }

    uint32_t next_ms = LBR::millis();
    uint32_t tick = 0;

    while (true)
    {
        if (static_cast<int32_t>(LBR::millis() - next_ms) < 0)
        {
            continue;
        }
        next_ms += kControlPeriodMs;

        LBR::PpsInputs inputs;
        inputs.now_ms = LBR::millis();
        if (pps.wants_imu())
        {
            inputs.imu_valid = LBR::read_imu(inputs.accel, inputs.gyro);
        }

        pps.step(inputs);
        console.poll(inputs.now_ms);
        indicate(board, pps.status(), tick++);
    }
}
