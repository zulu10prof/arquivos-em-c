
// WFS_DMP40_ClosedLoop.cpp - Closed-loop (WFS -> DMP40) with DM40 mapping starting at Noll 5 (AST45)
#include "TestFunctions.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <conio.h>
#include <cmath>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>

// Referência absoluta vs relativa (WFS) neste modo fechado
// - Esta rotina define explicitamente a referência interna do WFS no início:
//   ReferenceIndexSel=0 + WFS_SetReferencePlane. Assim, a frente de onda e o RMS
//   (WavefrontRMS) calculados passam a ser relativos a esse plano interno capturado.
// - Se fosse usado modo absoluto (sem referência), os desvios e a wavefront refletiriam
//   as aberrações/alinhamentos estáticos; no relativo, essas componentes “congeladas” na
//   referência são subtraídas e o RMS tende a representar variações em torno do plano.
// - Cancelamento de tilt: o código não ativa CancelWavefrontTilt (fica 0), portanto o RMS
//   é reportado conforme calculado pelo plugin do WFS, sem remover tilt/defocus no sensor.
//   A unidade do RMS segue WavefrontUnitSel (0=waves, 1=um).

// Lê os coeficientes de Zernike atuais do WFS, com fallback de ordem
static int get_WFS_ZernikeCoeffs(std::vector<double>& coeffs)
{
    SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficients"));
    SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficientsRMS"));

    // Tenta calcular Zernike; reduz ordem se houver "Insufficient spots"
    {
        int ord_cur = 0;
        GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "ZernikeOrderSel", &ord_cur);
        int tryOrd = (ord_cur > 0 ? ord_cur : 8);
        bool ok = false;
        for (; tryOrd >= 2; --tryOrd)
        {
            SetNumericParameter(hSensorElement, "ZernikeOrderSel", &tryOrd);
            ExecuteApiFunction(hSensorElement, "WFS_ZernikeLsf"); // intencionalmente sem SDKErrChk
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

// Executa o laço fechado WFS -> DMP40 usando termos de Zernike
int run_WFS_DMP40_ZernikeClosedLoop(int maxModes, double gain, bool cancelTilt, int sleepMs)
{
    if (IsSimulate())
    {
        std::cout << "[SIM] Closed-loop (WFS->DMP40) gain=" << gain
                  << ", maxModes=" << maxModes << " (offline). Pressione 'x' para sair." << std::endl;
        while (true) { if (_kbhit()) { char c = _getch(); if (c=='x'||c=='X') break; } Sleep(120); }
        return Rtn_OK;
    }

    if (!hSensorElement || !hControllerElement || !hSelectedZernikeTermAmplitudes)
    {
        std::cerr << "[Erro] Elementos/handles não inicializados (WFS/DM)." << std::endl;
        return Rtn_ERROR;
    }

    // Determina ordem a partir de K (mínimo 4)
    auto order_from_K = [](int K){ long long n=1; for(;;){ long long m=(n+1ll)*(n+2ll)/2ll; if(m>=K) break; if(n>1000) break; ++n;} return (int)n; };
    int ord = order_from_K(std::max(4, maxModes));
    SDKErrChk(SetNumericParameter(hSensorElement, "ZernikeOrderSel", &ord));

    // Ajustes básicos do WFS
    int avgCnt = 1; SDKErrChk(SetNumericParameter(hSensorElement, "AverageCount", &avgCnt));
    int cancelTilt0 = 0; SDKErrChk(SetNumericParameter(hSensorElement, "CancelWavefrontTilt", &cancelTilt0));

    // Layout do vetor de Zernike do DM40
    int dm_dt  = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_Type); ErrChk("GetDataPropertyInt");
    int dm_bpc = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_BytesPerComponent); ErrChk("GetDataPropertyInt");
    int dm_cpd = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_ComponentsPerData); ErrChk("GetDataPropertyInt");

    std::cout << "[TRACE] Iniciando loop fechado. Pressione 'x' para parar." << std::endl;
    // Define plano de referência interno (SDK: 0 = Internal Wavefront)
    {
        // Nota: a wavefront e o WavefrontRMS passam a ser RELATIVOS a este plano
        int refSel = 0;
        SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &refSel));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane"));
        std::cout << "[TRACE] Plano de referência do WFS: Internal Wavefront (relativo)." << std::endl;
    }

    // CSV logging setup (RMS vs time)
    std::ofstream rmsCsv;
    bool csvEnabled = false;
    int iteration = 0;
    auto t0 = std::chrono::steady_clock::now(); // marco de tempo para ms relativos
    {
        rmsCsv.open("wfs_rms_log.csv", std::ios::out | std::ios::trunc);
        if (rmsCsv.is_open())
        {
            rmsCsv << "ms,iteration,rms,unit" << std::endl; // cabeçalho com ms relativos
            csvEnabled = true;
            std::cout << "[TRACE] CSV RMS logging -> wfs_rms_log.csv" << std::endl;
        }
        else
        {
            std::cerr << "[Warn] Could not open wfs_rms_log.csv for writing. Continuing without CSV." << std::endl;
        }
    }
    bool dm_on = false;
    for (;;)
    {
        if (_kbhit()) {
            char c = _getch();
            if (c=='x'||c=='X') break;
            if (c=='m'||c=='M') { dm_on=!dm_on; std::cout<<"[TRACE] DM="<<(dm_on?"OFF":"ON")<<std::endl; }
            if (c=='u'||c=='U')
            {
                // Captura e aplica Referência de Usuário durante o loop
                int on = 1; SDKErrChk(SetNumericParameter(hSensorElement, "AllowUserReference", &on));
                SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));
                SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));
                SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetSpotsToUserReference"));
                int refIdxUser = 1; SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &refIdxUser));
                ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane");
                if (GetError(errStr, STRING_LENGTH_BUFFER_512))
                    std::cout << "[WFS] User Reference falhou: " << errStr << std::endl;
                else
                    std::cout << "[WFS] Referência do USUÁRIO capturada e aplicada (ReferenceIndexSel=1)." << std::endl;
            }
        }

        int dynCut = 0; SDKErrChk(SetNumericParameter(hSensorElement, "DynamicNoiseCut", &dynCut));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotToReferenceDeviations"));
        SDKErrChk(SetOutputByName(hSensorElement, "ArrayWavefront"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefront"));
        SDKErrChk(SetOutputByName(hSensorElement, "WavefrontRMS"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefrontStatistics"));

        std::vector<double> z;
        if (get_WFS_ZernikeCoeffs(z) != Rtn_OK)
        {
            std::cerr << "[Erro] Falha ao obter coeficientes Zernike do WFS." << std::endl;
            break;
        }

        int nz = (int)z.size();
        int K  = iZernikeCoefficientCount; // capacidade do vetor do DM (número de termos a partir de Noll 5)
        if (maxModes > 0) K = std::min(K, maxModes);
        // Unidade da wavefront -> escala
        int wunit = 0; SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "WavefrontUnitSel", &wunit));
        double lambda_um = 0.6328;
        double unit_scale = (wunit == 0) ? lambda_um : 1.0;

		// RMS da frente de onda: captura e plota em ASCII
		double rms = 0.0;
		SDKErrChk(CatchElementOutput(hSensorElement));
		SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontRMS", &rms));
        {
            const char* unit_label = (wunit == 0 ? "waves" : "um");
            double scale = (wunit == 0 ? 20.0 : 10.0);
            int bars = (int)std::round(std::min(60.0, std::max(0.0, rms * scale)));
            std::cout << "[RMS] " << rms << " " << unit_label << " |";
            for (int i = 0; i < bars; ++i) std::cout << '#';
            std::cout << std::endl;
                if (csvEnabled)
                {
                    // ms relativos desde o início do loop
                    long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
                    rmsCsv << ms << ","
                           << iteration << ","
                           << std::setprecision(10) << rms << ","
                           << unit_label;
                    // Acrescenta Zernike z1..z15 usando vetor 'z' obtido nesta iteracao
                    for (int zi=0; zi<15; ++zi)
                    {
                        double v = (zi < (int)z.size() ? z[zi] : 0.0);
                        rmsCsv << "," << v;
                    }
                    rmsCsv << std::endl;
                }
        }
        // Monta vetor para o DM com mapeamento: DM idx 0 = Noll 5 (AST45)
        std::vector<double> dmCoeffs(iZernikeCoefficientCount, 0.0);
        for (int j = 0; j < nz; ++j)
        {
            int noll = j + 1;
            if (noll <= 4) continue;                 // ignora pistão/tilts/defocus
            if (cancelTilt && (noll == 2 || noll == 3)) continue; // redundante, mantido por clareza
            int dm_idx = noll - 5;                   // 5->0, 6->1, ...
            if (dm_idx >= 0 && dm_idx < K)
                dmCoeffs[dm_idx] = -gain * (z[j] * unit_scale);
        }

        // OPD -> deslocamento da superfície (espelho reflexivo)
        for (int jj = 0; jj < iZernikeCoefficientCount; ++jj) dmCoeffs[jj] *= 0.5;

        // Clamp de segurança
        const double clamp_um =  1.0;
        int countClamped=0; for (int jj = 0; jj < iZernikeCoefficientCount; ++jj) { double v = dmCoeffs[jj]; if (!std::isfinite(v)) v = 0.0; if (v > clamp_um) { v = clamp_um; ++countClamped; } if (v < -clamp_um) { v = -clamp_um; ++countClamped; } dmCoeffs[jj] = v; }

        // Opcional: log de amostra do início do vetor do DM
        {
            int show = std::min(iZernikeCoefficientCount, 12);
            std::cout << "[DM ] Cmd[DMIdx 0.." << (show-1) << "]=";
            for (int ii = 0; ii < show; ++ii) std::cout << (ii?", ":"") << dmCoeffs[ii];
            std::cout << std::endl;
        }

        // Envio ao DM40
        if (dm_on)
        {
            if (dm_cpd <= 1 && dm_bpc == 4 && dm_dt == _DataType::Float)
            {
                std::vector<float> buf(iZernikeCoefficientCount, 0.0f);
                for (int i = 0; i < iZernikeCoefficientCount; ++i) buf[i] = static_cast<float>(dmCoeffs[i]);
                SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, buf.data()));
            }
            else
            {
                SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, dmCoeffs.data()));
            }
        }
        SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));
        SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));

        ++iteration;
        if (sleepMs > 0) Sleep(sleepMs);
    }

    std::cout << "Loop fechado encerrado." << std::endl;
    // Ensure CSV flush/close
    if (csvEnabled)
    {
        rmsCsv.flush();
        rmsCsv.close();
    }
    return Rtn_OK;
}

