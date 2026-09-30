#include "TestFunctions.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <conio.h>
#include <iomanip>
#include <fstream>
#include <string>
#include <cmath>

// Referência absoluta vs relativa (WFS)
// - Absoluto: desvios (spots) calculados em relação à grade/centros ideais das microlentes.
//   Usado quando chamamos WFS_GetSpotDeviations. A frente de onda e o RMS resultantes
//   refletem a óptica + desalinhamentos estáticos do sistema.
// - Relativo: desvios calculados em relação a um plano de referência previamente capturado
//   com WFS_SetReferencePlane (após escolher ReferenceIndexSel, por ex. 0 = interna).
//   Usado quando chamamos WFS_CalcSpotToReferenceDeviations. A frente de onda e o RMS
//   ficam “zerados” na referência e passam a indicar variações em torno desse plano.
// - Neste arquivo: tecla 'a' alterna entre absoluto/relativo para os Spot Deviations;
//   tecla 'r' captura uma referência interna. O RMS mostrado vem do plugin do WFS após
//   WFS_CalcWavefront/WFS_CalcWavefrontStatistics e, portanto, é relativo ao modo ativo.
//   Tilt não é cancelado por padrão (CancelWavefrontTilt=0); a unidade do RMS segue
//   WavefrontUnitSel (0=waves, 1=um).

// Simple sensor-only monitor: reads WFS telemetry continuously without DM actuation
// Returns Rtn_OK and sets has_valid=true if at least one (dx,dy) is finite
static int read_spot_deviations_sample(bool& has_valid)
{
    has_valid = false;
    SDKErrChk(SetOutputByName(hSensorElement, "ArraySpotDeviations"));
    SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_GetSpotDeviations"));
    DataHandle hSpotDev = GetDataObjectHandle(hSensorElement, "ArraySpotDeviations");
    ErrChk("GetDataObjectHandle");
    int s1 = GetDataPropertyInt(hSpotDev, _DataPropertyInt::Data_Size1); ErrChk("GetDataPropertyInt"); if (s1<=0) s1=1;
    int s2 = GetDataPropertyInt(hSpotDev, _DataPropertyInt::Data_Size2); ErrChk("GetDataPropertyInt"); if (s2<=0) s2=1;
    int comp = GetDataPropertyInt(hSpotDev, _DataPropertyInt::Data_ComponentsPerData); ErrChk("GetDataPropertyInt"); if (comp<=0) comp=1;
    int bpc  = GetDataPropertyInt(hSpotDev, _DataPropertyInt::Data_BytesPerComponent); ErrChk("GetDataPropertyInt");
    int dt   = GetDataPropertyInt(hSpotDev, _DataPropertyInt::Data_Type); ErrChk("GetDataPropertyInt");
    size_t total = (size_t)s1*(size_t)s2*(size_t)comp*(size_t)bpc;
    if (total == 0 || total > ((size_t)1<<30)) return Rtn_OK; // nothing to show
    void* raw = new byte[total];
    SDKErrChk(CopyDataContent(hSpotDev, raw));
    int count = s1*s2; int show = std::min(count, 6);
    std::cout << "[WFS] SpotDev (dx,dy)=";
    for (int k=0;k<show;++k)
    {
        double dx=0, dy=0;
        if (dt == _DataType::Float)  { float* f=(float*)raw;  dx=f[k*comp+0]; if (comp>1) dy=f[k*comp+1]; }
        else if (dt == _DataType::Double) { double* d=(double*)raw; dx=d[k*comp+0]; if (comp>1) dy=d[k*comp+1]; }
        std::cout << (k?", ":"") << "(" << dx << ", " << dy << ")";
        if (std::isfinite(dx) && std::isfinite(dy)) has_valid = true;
    }
    std::cout << std::endl;
    delete[] (byte*)raw;
    return Rtn_OK;
}

static bool read_wavefront_minmax_from_array(double& a_min, double& a_max)
{
    a_min = 0.0; a_max = 0.0;
    DataHandle hWF = GetDataObjectHandle(hSensorElement, "ArrayWavefront");
    if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int s1 = GetDataPropertyInt(hWF, _DataPropertyInt::Data_Size1); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int s2 = GetDataPropertyInt(hWF, _DataPropertyInt::Data_Size2); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int comp = GetDataPropertyInt(hWF, _DataPropertyInt::Data_ComponentsPerData); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int bpc  = GetDataPropertyInt(hWF, _DataPropertyInt::Data_BytesPerComponent); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int dt   = GetDataPropertyInt(hWF, _DataPropertyInt::Data_Type); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    if (s1<=0) s1=1; if (s2<=0) s2=1; if (comp<=0) comp=1;
    size_t total = (size_t)s1*(size_t)s2*(size_t)comp*(size_t)bpc;
    if (total == 0 || total > ((size_t)1<<30)) return false;
    void* raw = new byte[total];
    if (CopyDataContent(hWF, raw), GetError(errStr, STRING_LENGTH_BUFFER_512)) { delete[] (byte*)raw; return false; }
    size_t n = (size_t)s1*(size_t)s2;
    double mn = 0.0, mx = 0.0;
    bool inited = false;
    if (dt == _DataType::Float)
    {
        float* f = (float*)raw;
        for (size_t i=0;i<n;i++) {
            float v = f[i*comp];
            if (!std::isfinite(v)) continue;
            if (!inited){ mn=mx=v; inited=true;} else { if (v<mn) mn=v; if (v>mx) mx=v; }
        }
    }
    else if (dt == _DataType::Double)
    {
        double* d = (double*)raw;
        for (size_t i=0;i<n;i++) {
            double v = d[i*comp];
            if (!std::isfinite(v)) continue;
            if (!inited){ mn=mx=v; inited=true;} else { if (v<mn) mn=v; if (v>mx) mx=v; }
        }
    }
    delete[] (byte*)raw;
    if (!inited) return false;
    a_min = mn; a_max = mx; return true;
}

static bool read_wavefront_stats_from_array(double& a_min, double& a_max, double& a_rms)
{
    a_min = 0.0; a_max = 0.0; a_rms = 0.0;
    DataHandle hWF = GetDataObjectHandle(hSensorElement, "ArrayWavefront");
    if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int s1 = GetDataPropertyInt(hWF, _DataPropertyInt::Data_Size1); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int s2 = GetDataPropertyInt(hWF, _DataPropertyInt::Data_Size2); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int comp = GetDataPropertyInt(hWF, _DataPropertyInt::Data_ComponentsPerData); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int bpc  = GetDataPropertyInt(hWF, _DataPropertyInt::Data_BytesPerComponent); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    int dt   = GetDataPropertyInt(hWF, _DataPropertyInt::Data_Type); if (GetError(errStr, STRING_LENGTH_BUFFER_512)) return false;
    if (s1<=0) s1=1; if (s2<=0) s2=1; if (comp<=0) comp=1;
    size_t total = (size_t)s1*(size_t)s2*(size_t)comp*(size_t)bpc;
    if (total == 0 || total > ((size_t)1<<30)) return false;
    void* raw = new byte[total];
    if (CopyDataContent(hWF, raw), GetError(errStr, STRING_LENGTH_BUFFER_512)) { delete[] (byte*)raw; return false; }
    size_t n = (size_t)s1*(size_t)s2;
    // First pass: min/max and mean
    double mn=0.0, mx=0.0, mean=0.0; size_t cnt=0; bool inited=false;
    auto accumulate = [&](double v){ if (!std::isfinite(v)) return; if (!inited){ mn=mx=v; inited=true; } else { if (v<mn) mn=v; if (v>mx) mx=v; } mean += v; cnt++; };
    if (dt == _DataType::Float)
    {
        float* f=(float*)raw; for (size_t i=0;i<n;i++) accumulate(f[i*comp]);
    }
    else if (dt == _DataType::Double)
    {
        double* d=(double*)raw; for (size_t i=0;i<n;i++) accumulate(d[i*comp]);
    }
    if (cnt==0) { delete[] (byte*)raw; return false; }
    mean /= (double)cnt;
    // Second pass: RMS about mean
    double sumsq=0.0; size_t cnt2=0;
    auto accum2 = [&](double v){ if (!std::isfinite(v)) return; double dv=v-mean; sumsq += dv*dv; cnt2++; };
    if (dt == _DataType::Float)
    {
        float* f=(float*)raw; for (size_t i=0;i<n;i++) accum2(f[i*comp]);
    }
    else if (dt == _DataType::Double)
    {
        double* d=(double*)raw; for (size_t i=0;i<n;i++) accum2(d[i*comp]);
    }
    delete[] (byte*)raw;
    if (cnt2==0) return false;
    a_min = mn; a_max = mx; a_rms = std::sqrt(sumsq / (double)cnt2);
    return true;
}

int WFS_SensorOnly_Example()
{
    std::cout << "[TRACE] Enter WFS_SensorOnly_Example" << std::endl;
    if (IsSimulate())
    {
        std::cout << "[SIM] Sensor-only (offline). Pressione 'x' para sair." << std::endl;
        while (true) { if (_kbhit() && _getch()=='x') break; Sleep(120); }
        return Rtn_OK;
    }

    // Create WFS-only control system
    if (create_WFS_ControlSystem() == Rtn_ERROR)
    {
        std::cout << "\nNão foi possível criar o sistema WFS." << std::endl;
        return Rtn_ERROR;
    }
    if (set_WFS_Parameters() == Rtn_ERROR) { closeControlSystem(); return Rtn_ERROR; }
    if (set_WFS_Outputs() == Rtn_ERROR)    { closeControlSystem(); return Rtn_ERROR; }
    if (config_WFS() == Rtn_ERROR)         { closeControlSystem(); return Rtn_ERROR; }
    SDKErrChk(RunElement(hSensorElement));

    // Light settings to see variation
    int avgCnt = 1; SDKErrChk(SetNumericParameter(hSensorElement, "AverageCount", &avgCnt));
    int cancelTilt0 = 0; SDKErrChk(SetNumericParameter(hSensorElement, "CancelWavefrontTilt", &cancelTilt0));
    int dynCut = 0; SDKErrChk(SetNumericParameter(hSensorElement, "DynamicNoiseCut", &dynCut));
    // Optional: capture reference plane if you prefer relative deviations
    //int refIdx = 1; SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &refIdx));
    // SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane"));

    std::cout << "[TRACE] Sensor-only: Spots/Exposure/WF(min/max/RMS) + SpotDeviations" << std::endl;
    // Prepare CSV log for Wavefront RMS
    std::ofstream csv("wfs_rms_monitor.csv", std::ios::out | std::ios::trunc);
    if (csv.is_open())
    {
        // Acrescenta colunas para coeficientes de Zernike z1..z15
        csv << "timestamp,iteration,rms,unit";
        for (int zi = 1; zi <= 15; ++zi) csv << ",z" << zi;
        csv << std::endl;
        csv.flush();
        std::cout << "[TRACE] CSV RMS logging to wfs_rms_monitor.csv" << std::endl;
    }
    else
    {
        std::cout << "[WARN] Could not open wfs_rms_monitor.csv for writing. Proceeding without CSV logging." << std::endl;
    }

    std::cout << "        Teclas: x=sair, a=abs/ref dev toggle, r=captura ref interna, u=captura/usa ref usuario, z=Zernike amostra, p=print pupil" << std::endl;
    bool use_ref = true;
    int iter = 0;
    int on = 1; SDKErrChk(SetNumericParameter(hSensorElement, "AllowUserReference", &on));
    // Medida atual de spots
    SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));
    SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));
    // Define spots atuais como referência do usuário
    SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetSpotsToUserReference"));
    // Seleciona índice de referência do usuário (tipicamente 1) e aplica plano
    int refIdxUser = 1; SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &refIdxUser));
    ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane");
    if (GetError(errStr, STRING_LENGTH_BUFFER_512))
    {
        std::cout << "[WFS] User Reference falhou: " << errStr << std::endl;
    }
    else
    {
        use_ref = true; // passa a usar desvios relativos no loop
        std::cout << "[WFS] Referência do USUÁRIO capturada e aplicada (ReferenceIndexSel=1)." << std::endl;
    }
    //teste
    //int refIdx=1; SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndex", &refIdx));
    while (true)
    {
        if (_kbhit())
        {
            char c = _getch();
            if (c=='x' || c=='X') break;
            if (c=='a' || c=='A') { use_ref = !use_ref; std::cout << "[WFS] Deviations=" << (use_ref?"reference":"absolute") << std::endl; }
            if (c=='r' || c=='R')
            {
                // Prefer internal reference (index 0) to avoid "No User Reference available"
                int refIdx=0; SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &refIdx));
                ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane");
                if (GetError(errStr, STRING_LENGTH_BUFFER_512))
                {
                    std::cout << "[WFS] SetReferencePlane falhou: " << errStr << std::endl;
                }
                else
                {
                    std::cout << "[WFS] Reference (internal) configurada" << std::endl;
                }
            }
            if (c=='u' || c=='U')
            {
                // Captura e aplica Referência de Usuário
                int on = 1; SDKErrChk(SetNumericParameter(hSensorElement, "AllowUserReference", &on));
                // Medida atual de spots
                SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));
                SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));
                // Define spots atuais como referência do usuário
                SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetSpotsToUserReference"));
                // Seleciona índice de referência do usuário (tipicamente 1) e aplica plano
                int refIdxUser = 1; SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &refIdxUser));
                ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane");
                if (GetError(errStr, STRING_LENGTH_BUFFER_512))
                {
                    std::cout << "[WFS] User Reference falhou: " << errStr << std::endl;
                }
                else
                {
                    use_ref = true; // passa a usar desvios relativos no loop
                    std::cout << "[WFS] Referência do USUÁRIO capturada e aplicada (ReferenceIndexSel=1)." << std::endl;
                }
            }
            if (c=='z' || c=='Z')
            {
                // Amostra adaptativa de Zernike:
                // - Seleciona as saídas de coeficientes de Zernike no WFS;
                // - Tenta calcular Zernike por LSF (least squares fit) reduzindo a ordem
                //   automaticamente em caso de "Insufficient spots" (poucos spots válidos);
                // - Se obtiver sucesso, lê e imprime os primeiros coeficientes.

                // Seleciona as saídas (buffers) que serão lidas após o cálculo
                SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficients"));      // vetor Z1..ZK
                SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficientsRMS"));   // RMS por ordem (opcional)

                // Ordem inicial: usa a ordem atual se houver; caso contrário, começa em 8
                int ord_cur = 0; GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "ZernikeOrderSel", &ord_cur);
                int tryOrd = (ord_cur > 0 ? ord_cur : 8);

                bool ok = false;
                // Loop de tentativa: diminui a ordem até 2 se faltar spots
                for (; tryOrd >= 2; --tryOrd)
                {
                    // Define ordem a tentar
                    SetNumericParameter(hSensorElement, "ZernikeOrderSel", &tryOrd);

                    // Chama o ajuste de Zernike por mínimos quadrados (LSF).
                    // Intencionalmente sem SDKErrChk para inspeção manual do erro retornado abaixo.
                    ExecuteApiFunction(hSensorElement, "WFS_ZernikeLsf");

                    // Sucesso: sem erro pendente → sai do laço
                    if (!GetError(errStr, STRING_LENGTH_BUFFER_512)) { ok = true; break; }

                    // Houve erro: se não for "Insufficient spots", reporta e interrompe tentativas
                    std::string msg(errStr);
                    if (msg.find("Insufficient spots") == std::string::npos) { std::cout << "[WFS] Zernike error: " << msg << std::endl; break; }
                    // Caso de "Insufficient spots": reduz a ordem e tenta novamente
                }

                if (ok)
                {
                    // Captura as saídas selecionadas do elemento para leitura
                    SDKErrChk(CatchElementOutput(hSensorElement));

                    // Obtém o objeto de dados contendo os coeficientes
                    DataHandle hArray = GetDataObjectHandle(hSensorElement, "ArrayZernikeCoefficients"); ErrChk("GetDataObjectHandle");

                    // Lê metadados do array (tamanho, componentes, tipo)
                    int iSize1 = GetDataPropertyInt(hArray, _DataPropertyInt::Data_Size1); ErrChk("GetDataPropertyInt"); if (iSize1<=0) iSize1=1;
                    int iComp  = GetDataPropertyInt(hArray, _DataPropertyInt::Data_ComponentsPerData); ErrChk("GetDataPropertyInt"); if (iComp<=0) iComp=1;
                    int iBytes = GetDataPropertyInt(hArray, _DataPropertyInt::Data_BytesPerComponent); ErrChk("GetDataPropertyInt");
                    int iType  = GetDataPropertyInt(hArray, _DataPropertyInt::Data_Type); ErrChk("GetDataPropertyInt");

                    size_t total = (size_t)iSize1*(size_t)iComp*(size_t)iBytes;
                    if (total>0 && total<=((size_t)1<<30))
                    {
                        // Copia o conteúdo bruto e imprime os primeiros coeficientes para inspeção rápida
                        void* raw = new byte[total]; SDKErrChk(CopyDataContent(hArray, raw));
                        int show = std::min(iSize1, 16);
                        std::cout << "[WFS] Z[1.." << show << "]=";
                        for (int i=0;i<show;++i)
                        {
                            // Cada entrada corresponde ao coeficiente Z_i (convenção Noll no plugin)
                            double v=0;
                            if (iType == _DataType::Float)  { float* f=(float*)raw;  v=f[i*iComp]; }
                            else if (iType == _DataType::Double) { double* d=(double*)raw; v=d[i*iComp]; }
                            std::cout << (i?", ":"") << v;
                        }
                        std::cout << std::endl;
                        delete[] (byte*)raw;
                    }
                }
            }
            if (c=='p' || c=='P')
            {
                double pcx=0,pcy=0,pdx=0,pdy=0; int limit=0;
                SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilCenterX", &pcx));
                SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilCenterY", &pcy));
                SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilDiameterX", &pdx));
                SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilDiameterY", &pdy));
                SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "LimitToPupil", &limit));
                std::cout << "[WFS] Pupil: Center=(" << pcx << "," << pcy << ") Diam=(" << pdx << "," << pdy << ") LimitToPupil=" << limit << std::endl;
            }
        }

        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));
        if (use_ref)
        {
            SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotToReferenceDeviations"));
        }
        else
        {
            SDKErrChk(SetOutputByName(hSensorElement, "ArraySpotDeviations"));
            SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_GetSpotDeviations"));
        }
        // Compute wavefront first, then statistics
        SDKErrChk(SetOutputByName(hSensorElement, "ArrayWavefront"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefront"));
        SDKErrChk(SetOutputByName(hSensorElement, "WavefrontMax"));
        SDKErrChk(SetOutputByName(hSensorElement, "WavefrontMin"));
        SDKErrChk(SetOutputByName(hSensorElement, "WavefrontRMS"));
        SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefrontStatistics"));

        int sX=0, sY=0;
        SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "SpotsX", &sX));
        SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "SpotsY", &sY));
        double exp_ms=0.0; SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "ExposureTimeAct", &exp_ms));
        int wunit=0; GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "WavefrontUnitSel", &wunit); // best-effort
        // Force selection of outputs and catch
        SDKErrChk(CatchElementOutput(hSensorElement));
        double wfmax=0.0, wfmin=0.0, rms=0.0;
        SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontMax", &wfmax));
        SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontMin", &wfmin));
        SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontRMS", &rms));
        // Append to CSV (inclui coeficientes de Zernike z1..z15)
        if (csv.is_open())
        {
            SYSTEMTIME st; GetLocalTime(&st);
            char ts[32];
            sprintf_s(ts, "%04u-%02u-%02u %02u:%02u:%02u",
                      st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
            csv << ts << "," << iter << "," << std::setprecision(10) << rms << ","
                << (wunit == 0 ? "waves" : "um");

            // Coleta coeficientes de Zernike (amostra adaptativa)
            std::vector<double> zlog;
            {
                SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficients"));
                SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficientsRMS"));
                int ord_cur = 0; GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "ZernikeOrderSel", &ord_cur);
                int tryOrd = (ord_cur > 0 ? ord_cur : 8);
                bool okz = false;
                for (; tryOrd >= 2; --tryOrd)
                {
                    SetNumericParameter(hSensorElement, "ZernikeOrderSel", &tryOrd);
                    ExecuteApiFunction(hSensorElement, "WFS_ZernikeLsf");
                    if (!GetError(errStr, STRING_LENGTH_BUFFER_512)) { okz = true; break; }
                    std::string msg(errStr);
                    if (msg.find("Insufficient spots") == std::string::npos) { break; }
                }
                if (okz)
                {
                    SDKErrChk(CatchElementOutput(hSensorElement));
                    DataHandle hArray = GetDataObjectHandle(hSensorElement, "ArrayZernikeCoefficients"); ErrChk("GetDataObjectHandle");
                    int iSize1 = GetDataPropertyInt(hArray, _DataPropertyInt::Data_Size1); ErrChk("GetDataPropertyInt"); if (iSize1<=0) iSize1=1;
                    int iComp  = GetDataPropertyInt(hArray, _DataPropertyInt::Data_ComponentsPerData); ErrChk("GetDataPropertyInt"); if (iComp<=0) iComp=1;
                    int iBytes = GetDataPropertyInt(hArray, _DataPropertyInt::Data_BytesPerComponent); ErrChk("GetDataPropertyInt");
                    int iType  = GetDataPropertyInt(hArray, _DataPropertyInt::Data_Type); ErrChk("GetDataPropertyInt");
                    size_t total = (size_t)iSize1*(size_t)iComp*(size_t)iBytes;
                    if (total>0 && total<=((size_t)1<<30))
                    {
                        void* raw = new byte[total]; SDKErrChk(CopyDataContent(hArray, raw));
                        zlog.assign(iSize1, 0.0);
                        for (int i=0;i<iSize1;++i)
                        {
                            if (iType == _DataType::Float)  { float* f=(float*)raw;  zlog[i]=f[i*iComp]; }
                            else if (iType == _DataType::Double) { double* d=(double*)raw; zlog[i]=d[i*iComp]; }
                        }
                        delete[] (byte*)raw;
                    }
                }
            }
            for (int zi=0; zi<15; ++zi)
            {
                double v = (zi < (int)zlog.size() ? zlog[zi] : 0.0);
                csv << "," << v;
            }
            csv << std::endl;
        }
        double arr_min=0.0, arr_max=0.0, arr_rms=0.0;
        bool ok_arr = read_wavefront_stats_from_array(arr_min, arr_max, arr_rms);
        std::cout << std::fixed << std::setprecision(3)
                  << "[WFS] Spots=" << sX << "x" << sY
                  << ", Exp(ms)=" << exp_ms
                  << ", WF(min/max/RMS)=" << wfmin << "/" << wfmax << "/" << rms;
        if (ok_arr)
            std::cout << " | WF(Array) min/max/RMS=" << arr_min << "/" << arr_max << "/" << arr_rms;
        else
            std::cout << " | WF(Array) min/max=invalid";
        std::cout << std::endl;

        bool has_valid=false;
        read_spot_deviations_sample(has_valid);
        if (!has_valid)
        {
            std::cout << "[WFS] Aviso: desvios inválidos (NaN) — frente de onda pode estar sem dados válidos nesta iteração." << std::endl;
        }

        ++iter;
        Sleep(80);
    }

    closeControlSystem();
    if (csv.is_open()) { csv.flush(); csv.close(); }
    return Rtn_OK;
}
