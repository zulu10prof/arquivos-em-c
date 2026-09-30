// WFS_DMP40_ClosedLoop_ANSI.cpp - Loop fechado (WFS -> DMP40) com mapeamento ANSI
// Explicação geral:
// - O WFS mede coeficientes ANSI com j iniciando em 0 (inclui piston/tilts/defocus)
// - O DMP40 aplica correções a partir de j=3, então mapeamos dm_idx -> sensor_idx = dm_idx + 3
// - Calibramos por modo medindo a sensibilidade do sensor a ±1 de comando no DM (ganho = 1/sens)
// - O loop usa correção direta invertida, com saturação dos comandos em [-1, 1]
// - Tecla 'm' liga/desliga a atuação do espelho; OFF força coeficientes nulos enviados ao DM

#include "TestFunctions.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <conio.h>
#include <cmath>
#include <fstream>
#include <chrono>
#include <iomanip>

// Reads current WFS Zernike coefficients with fallback on order
static int get_WFS_ZernikeCoeffs_ansi(std::vector<double>& coeffs)
{
    SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficients"));
    SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficientsRMS"));

    // Try to compute Zernike; reduce order if "Insufficient spots"
    {
        int ord_cur = 0;
        GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "ZernikeOrderSel", &ord_cur);
        int tryOrd = (ord_cur > 0 ? ord_cur : 8);
        bool ok = false;
        for (; tryOrd >= 2; --tryOrd)
        {
            SetNumericParameter(hSensorElement, "ZernikeOrderSel", &tryOrd);
            ExecuteApiFunction(hSensorElement, "WFS_ZernikeLsf");
            if (!GetError(errStr, STRING_LENGTH_BUFFER_512)) { ok = true; break; }
            std::string msg(errStr);
            if (msg.find("Insufficient spots") == std::string::npos)
            {
                std::cerr << "[Erro] WFS_ZernikeLsf: " << msg << std::endl;
                return Rtn_ERROR;
            }
        }
        if (!ok)
        {
            std::cerr << "[Erro] ZernikeLsf falhou: pontos insuficientes mesmo na menor ordem." << std::endl;
            return Rtn_ERROR;
        }
        SDKErrChk(CatchElementOutput(hSensorElement));
    }

    DataHandle hArray = GetDataObjectHandle(hSensorElement, "ArrayZernikeCoefficients");
    ErrChk("GetDataObjectHandle");
    int iSize1 = GetDataPropertyInt(hArray, _DataPropertyInt::Data_Size1); ErrChk("GetDataPropertyInt");
    iSize1 = (iSize1 == 0) ? 1 : iSize1;
    int iComp = GetDataPropertyInt(hArray, _DataPropertyInt::Data_ComponentsPerData); ErrChk("GetDataPropertyInt");
    int iBytes = GetDataPropertyInt(hArray, _DataPropertyInt::Data_BytesPerComponent); ErrChk("GetDataPropertyInt");
    int iType  = GetDataPropertyInt(hArray, _DataPropertyInt::Data_Type); ErrChk("GetDataPropertyInt");

    size_t total = static_cast<size_t>(iSize1) * static_cast<size_t>(iComp) * static_cast<size_t>(iBytes);
    if (total == 0 || total > (static_cast<size_t>(1) << 30))
    {
        std::cerr << "[Erro] Tamanho inválido ao ler Zernike do WFS." << std::endl;
        return Rtn_ERROR;
    }
    void* byteArray = new byte[total];
    SDKErrChk(CopyDataContent(hArray, byteArray));
    coeffs.assign(iSize1, 0.0);
    for (int i = 0; i < iSize1; ++i)
    {
        if (iType == _DataType::Float)       coeffs[i] = ((float*)byteArray)[i * iComp];
        else if (iType == _DataType::Double) coeffs[i] = ((double*)byteArray)[i * iComp];
    }
    delete[] (byte*)byteArray;
    return Rtn_OK;
}

// Single acquisition + processing pass
static int wfs_grab_and_process_once()
{
    int dynCut = 0; SDKErrChk(SetNumericParameter(hSensorElement, "DynamicNoiseCut", &dynCut));
    SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));
    SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));
    SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotToReferenceDeviations"));
    SDKErrChk(SetOutputByName(hSensorElement, "ArrayWavefront"));
    SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefront"));
    SDKErrChk(SetOutputByName(hSensorElement, "WavefrontRMS"));
    SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefrontStatistics"));
    return Rtn_OK;
}

// Sends Zernike amplitudes to DM40
static int send_dm_zernike_ansi(const std::vector<double>& dmCoeffs)
{
    if (!hSelectedZernikeTermAmplitudes) return Rtn_ERROR;
    int dt  = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_Type); ErrChk("GetDataPropertyInt");
    int bpc = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_BytesPerComponent); ErrChk("GetDataPropertyInt");
    int cpd = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_ComponentsPerData); ErrChk("GetDataPropertyInt");
    //deve-se alterar aqui o valor do coeficiente
    if (cpd <= 1 && bpc == 4 && dt == _DataType::Float)
    {
        std::vector<float> buf(dmCoeffs.size());
        for (size_t i = 0; i < dmCoeffs.size(); ++i) buf[i] = static_cast<float>(-dmCoeffs[i]);
        SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, buf.data()));
    }
    else
    {
        // SDK expects a non-const buffer pointer; create a mutable copy
        std::vector<double> buf(dmCoeffs.begin(), dmCoeffs.end());
        SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, buf.data()));
    }
    SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));
    SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));
    return Rtn_OK;
}

// Per-mode calibration: DM +1 and -1, read sensor at j = dm_idx + 3 (ANSI)
static int ansi_calibrate_gains(std::vector<double>& gains, int K)
{
    gains.assign(iZernikeCoefficientCount, 0.0);
    std::vector<double> dm(iZernikeCoefficientCount, 0.0);

    // Enable and capture User Reference, then make it active (relative)
    {
        int allow = 1; SDKErrChk(SetNumericParameter(hSensorElement, "AllowUserReference", &allow));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetSpotsToUserReference"));
        int refSelUser = 1; SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &refSelUser));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane"));
    }
    // Prime the relative path
    SDKErrChk(wfs_grab_and_process_once());

    auto read_sensor_j = [](int j, double& val) -> int {
        std::vector<double> z;
        if (get_WFS_ZernikeCoeffs_ansi(z) != Rtn_OK) return Rtn_ERROR;
        if (j < 0 || j >= (int)z.size()) { val = 0.0; return Rtn_ERROR; }
        val = z[j];
        return Rtn_OK;
    };

    int Kuse = std::min(K, iZernikeCoefficientCount);
    for (int dm_idx = 0; dm_idx < Kuse; ++dm_idx)
    {
        std::fill(dm.begin(), dm.end(), 0.0);
        int s_idx = dm_idx + 3; // ANSI: sensor j starts at 0; DM starts correcting at j=3

        // +1
        dm[dm_idx] = +1.0;
        SDKErrChk(send_dm_zernike_ansi(dm));
        Sleep(80);
        SDKErrChk(wfs_grab_and_process_once());
        double z_plus = 0.0; if (read_sensor_j(s_idx, z_plus) != Rtn_OK) z_plus = 0.0;

        // -1
        dm[dm_idx] = -1.0;
        SDKErrChk(send_dm_zernike_ansi(dm));
        Sleep(80);
        SDKErrChk(wfs_grab_and_process_once());
        double z_minus = 0.0; if (read_sensor_j(s_idx, z_minus) != Rtn_OK) z_minus = 0.0;

        // Back to 0
        dm[dm_idx] = 0.0;
        SDKErrChk(send_dm_zernike_ansi(dm));
        Sleep(50);

        double sens = (z_plus - z_minus) * 0.5; // sensor delta per 1.0 DM unit
        gains[dm_idx] = (std::abs(sens) < 1e-12) ? 0.0 : (1.0 / sens);

        std::cout << "[CAL] dm_idx=" << dm_idx << " sensor_idx=" << s_idx
                  << " sens=" << sens << " gain=" << gains[dm_idx] << std::endl;
    }
    return Rtn_OK;
}

int run_WFS_DMP40_ANSI_Calibrated(int maxModes, double overallGain, int sleepMs)
{
    if (IsSimulate())
    {
        std::cout << "[SIM] ANSI calibrated loop (offline). Pressione 'x' para sair." << std::endl;
        while (true) { if (_kbhit()) { char c = _getch(); if (c=='x'||c=='X') break; } Sleep(120); }
        return Rtn_OK;
    }

    if (!hSensorElement || !hControllerElement || !hSelectedZernikeTermAmplitudes)
    {
        std::cerr << "[Erro] Elementos/handles não inicializados (WFS/DM)." << std::endl;
        return Rtn_ERROR;
    }

    int avgCnt = 1; SDKErrChk(SetNumericParameter(hSensorElement, "AverageCount", &avgCnt));
    int cancelTilt0 = 0; SDKErrChk(SetNumericParameter(hSensorElement, "CancelWavefrontTilt", &cancelTilt0));

    auto order_from_K = [](int K){ long long n=1; for(;;){ long long m=(n+1ll)*(n+2ll)/2ll; if(m>=K) break; if(n>1000) break; ++n;} return (int)n; };
    int needK = std::max(4, (int)maxModes + 3); // ensure sensor has at least 3 + K
    int ord = order_from_K(needK);
    SDKErrChk(SetNumericParameter(hSensorElement, "ZernikeOrderSel", &ord));

    // Enable and capture User Reference, then make it active (relative)
    {
        int allow = 1; SDKErrChk(SetNumericParameter(hSensorElement, "AllowUserReference", &allow));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetSpotsToUserReference"));
        int refSelUser = 1; SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &refSelUser));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane"));
    }
    // Prime the sensor once so RMS and Zernike are relative before calibration
    SDKErrChk(wfs_grab_and_process_once());

    // Calibrate per-mode gains
    std::vector<double> gains;
    if (ansi_calibrate_gains(gains, maxModes <= 0 ? iZernikeCoefficientCount : maxModes) != Rtn_OK)
    {
        std::cerr << "[Erro] Calibração ANSI falhou." << std::endl;
        return Rtn_ERROR;
    }

    std::cout << "[TRACE] Iniciando loop fechado ANSI. Pressione 'x' para parar. Tecla 'm' liga/desliga o DM (zera coeficientes ao desligar)." << std::endl;
    // CSV RMS logger
    std::ofstream rmsCsv("wfs_rms_ansi.csv");
    if (rmsCsv)
    {
        rmsCsv << "iter,time_ms,rms,unit" << std::endl;
    }
    auto t0 = std::chrono::steady_clock::now();
    long long iter = 0;
    std::vector<double> dmCmd(iZernikeCoefficientCount, 0.0); // vetor de comandos do DM40
    bool dm_on = true;                                        // estado da atuação do espelho (toggle com 'm')
    for (;;)
    {
        if (_kbhit())
        {
            char c = _getch();
            if (c=='x'||c=='X') break;              // sair
            if (c=='m'||c=='M')                     // toggle DM
            {
                dm_on = !dm_on;
                std::cout << "[TRACE] DM=" << (dm_on?"ON":"OFF (coeficientes zerados)") << std::endl;
                if (!dm_on)
                {
                    std::fill(dmCmd.begin(), dmCmd.end(), 0.0); // zera vetor
                    send_dm_zernike_ansi(dmCmd);                 // garante DM em zero quando desliga
                }
            }
        }

        SDKErrChk(wfs_grab_and_process_once());
        // Read and plot RMS
        int wunit = 0; GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "WavefrontUnitSel", &wunit);
        double rms = 0.0;
        SDKErrChk(CatchElementOutput(hSensorElement));
        SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontRMS", &rms));
        {
            const char* unit_label = (wunit==0?"waves":"um");
            double scale = (wunit==0?20.0:10.0);
            int bars = (int)std::round(std::min(60.0, std::max(0.0, rms*scale)));
            std::cout << "[RMS] " << std::setprecision(10) << rms << " " << unit_label << " |";
            for (int i=0;i<bars;++i) std::cout << '#';
            std::cout << std::endl;
            if (rmsCsv)
            {
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-t0).count();
                rmsCsv << iter << "," << ms << "," << rms << "," << unit_label << std::endl;
                if ((iter % 10)==0) rmsCsv.flush();
            }
        }
        std::vector<double> z; if (get_WFS_ZernikeCoeffs_ansi(z) != Rtn_OK) { Sleep(sleepMs); continue; }

        int K = (maxModes <= 0) ? iZernikeCoefficientCount : std::min(iZernikeCoefficientCount, maxModes);
        for (int dm_idx = 0; dm_idx < K; ++dm_idx)
        {
            int s_idx = dm_idx + 3; // ignore j=0..2 from sensor
            double zi = (s_idx < (int)z.size()) ? z[s_idx] : 0.0;
            double g = gains[dm_idx];
            double cmd = -overallGain * g * zi;
            if (!std::isfinite(cmd)) cmd = 0.0;
            if (cmd >  1.0) cmd =  1.0;
            if (cmd < -1.0) cmd = -1.0;
            dmCmd[dm_idx] = cmd;
        }
        for (int dm_idx = K; dm_idx < iZernikeCoefficientCount; ++dm_idx) dmCmd[dm_idx] = 0.0;

        if (dm_on)
        {
            SDKErrChk(send_dm_zernike_ansi(dmCmd)); // envia comandos calculados
        }
        else
        {
            std::fill(dmCmd.begin(), dmCmd.end(), 0.0); // mantém zero enquanto OFF
            SDKErrChk(send_dm_zernike_ansi(dmCmd));
        }
        ++iter;
        if (sleepMs > 0) Sleep(sleepMs);
    }

    std::fill(dmCmd.begin(), dmCmd.end(), 0.0);
    send_dm_zernike_ansi(dmCmd);
    if (rmsCsv) rmsCsv.flush();
    std::cout << "Loop ANSI encerrado." << std::endl;
    return Rtn_OK;
}

int WFS_DMP40_ANSI_Calibrated_Example()
{
    if (create_WFS_DMP40_ControlSystem() != Rtn_OK) return Rtn_ERROR;
    int maxModes = iZernikeCoefficientCount;
    double overallGain = 1.0;
    int sleepMs = 100;
    int r = run_WFS_DMP40_ANSI_Calibrated(maxModes, overallGain, sleepMs);
    closeControlSystem();
    return r;
}
