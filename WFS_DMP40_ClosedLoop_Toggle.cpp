// WFS_DMP40_ClosedLoop_Toggle.cpp — Closed-loop (WFS -> DM40) com opção de ligar/desligar o espelho durante o loop (WFS -> DM40) com opÃ§Ã£o de ligar/desligar o espelho durante o loop
#include "TestFunctions.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <conio.h>
#include <cmath>
#include <fstream>
#include <chrono>

// Helper local: lÃª coeficientes de Zernike do WFS (como no caso 'k')
static int get_WFS_Z(std::vector<double>& coeffs)
{
    SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficients"));
    SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficientsRMS"));
    // Tenta calcular Zernike; reduz ordem se houver "Insufficient spots"
    int ord_cur = 0; GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "ZernikeOrderSel", &ord_cur);
    int tryOrd = (ord_cur > 0 ? ord_cur : 8);
    bool ok = false;
    for (; tryOrd >= 2; --tryOrd)
    {
        SetNumericParameter(hSensorElement, "ZernikeOrderSel", &tryOrd);
        ExecuteApiFunction(hSensorElement, "WFS_ZernikeLsf");
        if (!GetError(errStr, STRING_LENGTH_BUFFER_512)) { ok = true; break; }
        std::string msg(errStr);
        if (msg.find("Insufficient spots") == std::string::npos)
            return Rtn_ERROR;
    }
    if (!ok) return Rtn_ERROR;
    SDKErrChk(CatchElementOutput(hSensorElement));

    DataHandle hArray = GetDataObjectHandle(hSensorElement, "ArrayZernikeCoefficients"); ErrChk("GetDataObjectHandle");
    int n = GetDataPropertyInt(hArray, _DataPropertyInt::Data_Size1); ErrChk("GetDataPropertyInt"); n = (n==0)?1:n;
    int comp = GetDataPropertyInt(hArray, _DataPropertyInt::Data_ComponentsPerData); ErrChk("GetDataPropertyInt");
    int bpc  = GetDataPropertyInt(hArray, _DataPropertyInt::Data_BytesPerComponent); ErrChk("GetDataPropertyInt");
    int dt   = GetDataPropertyInt(hArray, _DataPropertyInt::Data_Type); ErrChk("GetDataPropertyInt");
    size_t total = (size_t)n*(size_t)comp*(size_t)bpc; if (total==0 || total > ((size_t)1<<30)) return Rtn_ERROR;
    void* raw = new byte[total]; SDKErrChk(CopyDataContent(hArray, raw));
    coeffs.assign(n, 0.0);
    for (int i=0;i<n;++i){ if (dt==_DataType::Float) coeffs[i]=((float*)raw)[i*comp]; else if (dt==_DataType::Double) coeffs[i]=((double*)raw)[i*comp]; }
    delete[] (byte*)raw; return Rtn_OK;
}

// Loop fechado com toggle de atuaÃ§Ã£o do DM (tecla 'm')
int run_WFS_DMP40_ZernikeClosedLoop_Toggle(int maxModes, double gain, bool cancelTilt, int sleepMs)
{
    if (IsSimulate())
    {
        std::cout << "[SIM] Closed-loop TOGGLE (WFS->DMP40) gain=" << gain << ", maxModes=" << maxModes << ". 'm' liga/desliga DM, 'x' sai." << std::endl;
        bool dm_on = true; while (true){ if(_kbhit()){ char c=_getch(); if(c=='x'||c=='X') break; if(c=='m'||c=='M'){ dm_on=!dm_on; std::cout<<"[SIM] DM="<<(dm_on?"ON":"OFF")<<std::endl; } } Sleep(120);} return Rtn_OK;
    }
    if (!hSensorElement || !hControllerElement || !hSelectedZernikeTermAmplitudes) return Rtn_ERROR;

    auto order_from_K = [](int K){ long long n=1; for(;;){ long long m=(n+1ll)*(n+2ll)/2ll; if(m>=K) break; if(n>1000) break; ++n;} return (int)n; };
    int ord = order_from_K(std::max(4, maxModes)); SDKErrChk(SetNumericParameter(hSensorElement, "ZernikeOrderSel", &ord));
    int avgCnt=1; SDKErrChk(SetNumericParameter(hSensorElement, "AverageCount", &avgCnt)); int cancelTilt0=0; SDKErrChk(SetNumericParameter(hSensorElement, "CancelWavefrontTilt", &cancelTilt0));

    // DM layout
    int dm_dt  = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_Type); ErrChk("GetDataPropertyInt");
    int dm_bpc = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_BytesPerComponent); ErrChk("GetDataPropertyInt");
    int dm_cpd = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_ComponentsPerData); ErrChk("GetDataPropertyInt");

    // ReferÃªncia interna (relativa)
    { int refSel=0; SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &refSel)); SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane")); }

    std::cout << "[TRACE] Loop TOGGLE: 'm' liga/desliga DM, 'x' sai." << std::endl;

    // CSV de saÃ­da para RMS
    std::ofstream rmsCsv("rms_toggle.csv");
    if (rmsCsv)
    {
        rmsCsv << "iter,time_ms,rms,unit" << std::endl;
    }
    auto t0 = std::chrono::steady_clock::now();
    long long iter = 0;
    bool dm_on = false;
    for(;;)
    {
        if (_kbhit()) { char c=_getch(); if (c=='x'||c=='X') break; if (c=='m'||c=='M'){ dm_on=!dm_on; std::cout<<"[TRACE] DM="<<(dm_on?"ON":"OFF")<<std::endl; } }

        int dynCut=0; SDKErrChk(SetNumericParameter(hSensorElement, "DynamicNoiseCut", &dynCut));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotToReferenceDeviations"));
        SDKErrChk(SetOutputByName(hSensorElement, "ArrayWavefront")); SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefront"));
        SDKErrChk(SetOutputByName(hSensorElement, "WavefrontRMS")); SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefrontStatistics"));

        std::vector<double> z; if (get_WFS_Z(z)!=Rtn_OK) break;
        int nz=(int)z.size(); int K=iZernikeCoefficientCount; if (maxModes>0) K=std::min(K,maxModes);
        int wunit=0; SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "WavefrontUnitSel", &wunit)); double lambda_um=0.6328; double unit_scale=(wunit==0?lambda_um:1.0);
        // RMS plot
        double rms=0.0; SDKErrChk(CatchElementOutput(hSensorElement)); SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontRMS", &rms));
        {
            const char* unit_label=(wunit==0?"waves":"um");
            double scale=(wunit==0?20.0:10.0);
            int bars=(int)std::round(std::min(60.0,std::max(0.0,rms*scale)));
            std::cout<<"[RMS] "<<rms<<" "<<unit_label<<" |"; for(int i=0;i<bars;++i) std::cout<<'#'; std::cout<<std::endl;
            if (rmsCsv)
            {
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-t0).count();
                rmsCsv << iter << "," << ms << "," << rms << "," << unit_label << std::endl;
                if ((iter % 10)==0) rmsCsv.flush();
            }
        }

        // Mapeamento DM40: DM idx 0 = Noll 5 (AST45)
        std::vector<double> dmCoeffs(iZernikeCoefficientCount, 0.0);
        for (int j=0;j<nz;++j){ int noll=j+1; if (noll<=4) continue; if (cancelTilt && (noll==2||noll==3)) continue; int dm_idx=noll-5; if (dm_idx>=0 && dm_idx<K) dmCoeffs[dm_idx] = -gain*(z[j]*unit_scale); }
        for (int jj=0;jj<iZernikeCoefficientCount;++jj) dmCoeffs[jj]*=0.5;
        const double clamp_um=1.0; for (int jj=0;jj<iZernikeCoefficientCount;++jj){ double v=dmCoeffs[jj]; if(!std::isfinite(v)) v=0.0; if(v>clamp_um) v=clamp_um; if(v<-clamp_um) v=-clamp_um; dmCoeffs[jj]=v; }

        // Envio ao DM40 (condicionado ao toggle)
        if (dm_on)
        {
            if (dm_cpd <= 1 && dm_bpc == 4 && dm_dt == _DataType::Float)
            { std::vector<float> buf(iZernikeCoefficientCount,0.0f); for(int i=0;i<iZernikeCoefficientCount;++i) buf[i]=(float)dmCoeffs[i]; SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, buf.data())); }
            else
            { SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, dmCoeffs.data())); }
            SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));
            SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));
        }

        ++iter;
        if (sleepMs>0) Sleep(sleepMs);
    }
    if (rmsCsv) rmsCsv.flush();
    std::cout << "Loop TOGGLE encerrado." << std::endl; return Rtn_OK;
}

// Exemplo com defaults e criaÃ§Ã£o de sistema (como caso 'k'), mas com toggle do DM
int WFS_DMP40_ClosedLoop_Toggle_Example()
{
    std::cout << "[TRACE] Enter WFS_DMP40_ClosedLoop_Toggle_Example" << std::endl;
    if (IsSimulate())
    { std::cout << "[SIM] WFS + DMP40 closed-loop (toggle, offline)" << std::endl; return run_WFS_DMP40_ZernikeClosedLoop_Toggle(20, 0.1, true, 80); }

    if (create_WFS_DMP40_ControlSystem() == Rtn_ERROR)
    { std::cout << "\nNao foi possivel criar sistema WFS + DMP40." << std::endl; return Rtn_ERROR; }

    int maxModes=20; double gain=0.07; bool cancelTilt=true; int sleepMs=10;
    std::cout << "[TRACE] Loop params (toggle): maxModes="<<maxModes<<", gain="<<gain<<", cancelTilt="<<(cancelTilt?"true":"false")<<", sleepMs="<<sleepMs<<std::endl;
    int r = run_WFS_DMP40_ZernikeClosedLoop_Toggle(maxModes, gain, cancelTilt, sleepMs);
    closeControlSystem();
    return r;
}

