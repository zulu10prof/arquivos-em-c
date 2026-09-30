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
    double gain       = 0.05;
    bool   cancelTilt = true;
    int    sleepMs    = 10;

    std::cout << "[TRACE] Loop params (wrapper): maxModes=" << maxModes
              << ", gain=" << gain
              << ", cancelTilt=" << (cancelTilt?"true":"false")
              << ", sleepMs=" << sleepMs << std::endl;

    // Prepare CSV file (truncate and write header)
    {
        std::ofstream csvHead("wfs_rms_log.csv", std::ios::out | std::ios::trunc);
        if (csvHead.is_open())
        {
            csvHead << "timestamp,iteration,rms,unit" << std::endl;
        }
        else
        {
            std::cerr << "[Warn] Could not open wfs_rms_log.csv for writing. Proceeding without CSV logging." << std::endl;
        }
    }

    // Background logger thread: polls WavefrontRMS and appends to CSV
    volatile bool stop = false;
    auto loggerFn = [&stop, sleepMs]() {
        int iter = 0;
        while (!stop)
        {
            // Read unit selection
            int unitSel = 0; // 0=waves, 1=um
            GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "WavefrontUnitSel", &unitSel);

            // Select and fetch WavefrontRMS (best-effort; no abort on error)
            double rms = 0.0;
            SetOutputByName(hSensorElement, "WavefrontRMS");
            CatchElementOutput(hSensorElement);
            GetNumericOutput(hSensorElement, "WavefrontRMS", &rms);

            // Timestamp (Windows API)
            SYSTEMTIME st; GetLocalTime(&st);
            char ts[32];
            sprintf_s(ts, "%04u-%02u-%02u %02u:%02u:%02u",
                      st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

            // Append to CSV (open/close each time to avoid shared stream/mutex)
            std::ofstream csv("wfs_rms_log.csv", std::ios::out | std::ios::app);
            if (csv.is_open())
            {
                csv << ts << "," << iter << "," << std::setprecision(10) << rms << ","
                    << (unitSel == 0 ? "waves" : "um") << std::endl;
            }

            ++iter;
            int period = std::max(20, sleepMs / 2);
            Sleep(period);
        }
    };

    std::thread logger(loggerFn);

    int r = run_WFS_DMP40_ZernikeClosedLoop(maxModes, gain, cancelTilt, sleepMs);

    // Stop logger and cleanup
    stop = true;
    if (logger.joinable()) logger.join();

    closeControlSystem();
    return r;
}

