// WFS_DMP40_ClosedLoop_Entry.cpp â€” wrapper exposing WFS_DMP40_ClosedLoop_Example()
#include "TestFunctions.h"
#include <iostream>

// Lightweight wrapper: ensures a linkable definition exists
int WFS_DMP40_ClosedLoop_Example()
{
    std::cout << "[TRACE] Enter WFS_DMP40_ClosedLoop_Example (wrapper)" << std::endl;

    if (IsSimulate())
    {
        std::cout << "[SIM] WFS + DMP40 closed-loop (wrapper, offline)" << std::endl;
        return run_WFS_DMP40_ZernikeClosedLoop(20, 0.7, true, 80);
    }

    if (create_WFS_DMP40_ControlSystem() == Rtn_ERROR)
    {
        std::cerr << "[Error] Could not create WFS + DMP40 control system (wrapper)." << std::endl;
        return Rtn_ERROR;
    }

    int    maxModes   = 20;
    double gain       = 0.1
    ;
    bool   cancelTilt = true;
    int    sleepMs    = 80;

    std::cout << "[TRACE] Loop params (wrapper): maxModes=" << maxModes
              << ", gain=" << gain
              << ", cancelTilt=" << (cancelTilt?"true":"false")
              << ", sleepMs=" << sleepMs << std::endl;

    int r = run_WFS_DMP40_ZernikeClosedLoop(maxModes, gain, cancelTilt, sleepMs);
    closeControlSystem();
    return r;
}


