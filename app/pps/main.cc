#include "board.h"
#include "core/rtask.h"

int main()
{
    const bool board_ok = LBR::bsp_init();

    static LBR::Rtask rtask(LBR::get_board());

    // A board that failed bring-up stays safed but still answers the console
    if (!board_ok)
    {
        rtask.abort();
    }

    while (true)
    {
        rtask.spin(LBR::millis());
    }
}
