#include "pps.h"
#include "board.h"
#include "pps_helpers.h"

using LBR::Pps;
using LBR::Board;



int main() {
    Board& board = LBR::get_board();
    Pps pps(board.gpio, *board.motor); // board.motor is a pointer, so dereference
    while (true) {
        // Simulate fetching IMU data
        LBR::Quaternion imu_data; 
        pps.fetchImuData(imu_data);
        LBR::Vec3 accel_data; 
        pps.fetchAccelData(accel_data);
        
        // Update PPS state machine
        pps.update();
    }
    return 0;
}