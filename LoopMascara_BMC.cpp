#include "TestFunctions.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <conio.h>

/*
  LoopMascara_BMC.cpp — explicacoes e referencias

  Visao geral:
  - Gera fase turbulenta (modelo estatistico de Kolmogorov),
    decompõe nos primeiros K modos de Zernike (indexacao de Noll),
    reconstrói a fase na malha dos atuadores e mapeia para tensões
    do BMC 140 usando os limites de calibracao (offset/max).

  Principais relacoes e formulas:
  - Kolmogorov (estatistica da fase):
      D_phi(rho) = 6.88 * (rho / r0)^(5/3)
      Phi_phi(k) = 0.023 * r0^(-5/3) * |k|^(-11/3)  (PSD)
    A fase e sintetizada por generate_kolmogorov_phase_plane_waves a
    partir da soma de ondas planas ponderadas pela PSD.

  - Zernike e indexacao de Noll:
    j -> (n,m) via mapeamento de Noll; Z_j(r,theta) = Z_n^m(r,theta).
    A decomposicao em K modos equivale a projetar a fase b na base A
    e resolver (A^T A) c = A^T b (minimos quadrados, com Tikhonov leve),
    conforme implementado na versao Thor (ver LoopMascara.cpp).

  - Fase [rad], OPD [um] e atuadores:
      phi = (2*pi/lambda) * OPD  =>  OPD = phi * (lambda / 2*pi)
    Aqui reconstrui-se a fase (rad) sobre a grade de atuadores e mapeia-se
    diretamente para tensões em [offset..max] com saturacao suave (tanh).
    Caso seja necessario operar em unidade de OPD ou de deslocamento
    de superficie (h, com OPD = 2*h para espelho refletivo), inserir a
    conversao apropriada antes do mapeamento para tensoes.

  Referencias:
  - Noll, R. J. (1976). Zernike polynomials and atmospheric turbulence. JOSA.
  - Fried, D. L. (1966). Optical resolution through a randomly inhomogeneous medium. JOSA.
  - Goodman, J. W. (2000). Statistical Optics. Wiley.
  - Roddier, F. (1999). Adaptive Optics in Astronomy. Cambridge Univ. Press.
  - Born, M.; Wolf, E. (1999). Principles of Optics. Cambridge Univ. Press.
*/

// Conversão Noll j -> (n,m) (j>=1)
static inline void noll_to_nm(int j, int& n, int& m) {
    // Noll (1976): mapeia j (1-indexado) para par (n,m) dos polinomios de Zernike.
    // Usado em Z_j(r,theta) = Z_n^m(r,theta) para a decomposicao/reconstrucao.
    int j1 = j - 1;
    n = 0;
    while (j1 > n) { j1 -= (n + 1); ++n; }
    int k = j1;
    int s = (n & 1) ? 1 : 0;
    m = 2 * k - n;
    if (s) m = -m;
}

// Clamp utilitário
static inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

int LoopMascara_BMC()
{
    if (IsSimulate())
    {
        std::cout << "[SIM] LoopMascara_BMC (offline)" << std::endl;
        std::cout << "[SIM] Gerando máscaras Zernike sintéticas para BMC 140. Pressione 'x' para sair (no-op)." << std::endl;
        Sleep(300);
        return Rtn_OK;
    }

    if (create_BMC_DM_ControlSystem() == Rtn_ERROR) {
        std::cout << "\nNão foi possível criar o sistema com BMC Multi deformable mirror.\n";
        return Rtn_ERROR;
    }
    if (set_BMC_DM_Parameters() == Rtn_ERROR) return Rtn_ERROR;
    if (get_BMC_DM_DataObject() == Rtn_ERROR) return Rtn_ERROR;

    // Sinais/calibração do BMC: limites por atuador
    DataHandle hSignal = GetElementSignalObjectHandle(hControllerElement1);
    SDKErrChk(hSignal);
    double* offsetCommands = (double*)GetControllerSignal_ArrayPtr(hSignal, _ControllerSignal_InternalArray::ControllerSignal_Calibration_OffsetCommands);
    SDKErrChk(offsetCommands);
    double* maxCommands = (double*)GetControllerSignal_ArrayPtr(hSignal, _ControllerSignal_InternalArray::ControllerSignal_Calibration_MaxCommands);
    SDKErrChk(maxCommands);

    const int rows = iBMC_DM_DataSize1;
    const int cols = iBMC_DM_DataSize2;
    const int Nact = rows * cols;

    std::vector<double> dmData((size_t)Nact, 0.0);

    // Parâmetros de geração de fase/Zernike
    const int    Ngrid     = 32;
    // Ngrid: tamanho da malha para amostrar a pupila (disco unitario)
    const int    K         = 12;              // primeiros modos Zernike
    // K: quantidade de modos de Zernike na decomposicao
    const double lambda_um = 0.6328;          // HeNe típico (µm)
    const double r0_relD   = 0.2;             // turbulência relativa (ajuste conforme)
    const double numCycles = 20.0;
    const int    numModes  = 400;
    // numCycles/numModes: controle da sintese de ondas planas na geracao de fase
    const double amplitude_frac = 1;        // fração do range [offset..max]

    std::cout << "Aplicando máscara em loop no BMC 140. Pressione 'x' para parar." << std::endl;

    bool stop = false;
    while (!stop)
    {
        // 1) Gera tela de fase (rad) e decompõe nos K primeiros Zernike
        // Kolmogorov: D_phi(rho)=6.88*(rho/r0)^(5/3); PSD ~ |k|^(-11/3)
        // Referencias: Fried (1966); Goodman (2000); Roddier (1999)
        auto phase_rad = generate_kolmogorov_phase_plane_waves(Ngrid, numCycles, r0_relD, numModes);
        std::vector<double> c_rad;
        // Projecao em Zernike via minimos quadrados: (A^T A)c = A^T b
        if (!decompose_phase_firstK_using_zernikePolynomial(phase_rad, K, c_rad)) {
            std::cout << "Falha na decomposição em Zernike." << std::endl;
            break;
        }

        // 2) Recompõe a superfície em coordenadas dos atuadores e normaliza por RMS
        // Reconstrucao da fase (rad) na grade de atuadores; normalizacao por RMS
        const double cx = (cols - 1) * 0.5;
        const double cy = (rows - 1) * 0.5;
        const double Rn = std::min(cx, cy);

        std::vector<double> surf_rad((size_t)Nact, 0.0);
        int P = 0; double sum2 = 0.0;
        for (int iy = 0; iy < rows; ++iy) {
            for (int ix = 0; ix < cols; ++ix) {
                const int id = iy * cols + ix;
                // Coordenadas sobre disco unitario
                double X = (ix - cx) / Rn;
                double Y = (iy - cy) / Rn;
                double r = std::sqrt(X*X + Y*Y);
                if (r > 1.0) { surf_rad[id] = 0.0; continue; }
                double th = std::atan2(Y, X);

                double val = 0.0;
                for (int j = 1; j <= K; ++j) {
                    int n, m; noll_to_nm(j, n, m);
                    val += c_rad[j-1] * zernikePolynomial(n, m, r, th);
                }
                surf_rad[id] = val;
                sum2 += val * val; ++P;
            }
        }
        double rms = (P > 0) ? std::sqrt(sum2 / P) : 1.0; // normalizacao de energia (RMS)
        if (rms < 1e-12) rms = 1.0;

        // 3) Mapeia superfície -> tensões por atuador dentro [offset..max]
        //    Usa escala suave (tanh) para evitar saturação dura.
        SDKErrChk(LockElement(hControllerElement1));
        const double scale = 3.0 * rms; // ~3 sigma para tanh
        for (int i = 0; i < Nact; ++i) {
            double vmin = offsetCommands[i];
            double vmax = maxCommands[i];
            double vctr = 0.5 * (vmin + vmax);
            double dv   = 0.5 * amplitude_frac * (vmax - vmin);
            double norm = std::tanh(surf_rad[i] / scale); // [-1,1]
            dmData[i] = clamp(vctr + dv * norm, vmin, vmax);
        }
        // Envia ao driver: setar vetor -> aplicar (BMCSetArray)
        SDKErrChk(SetDataContent(hArrayActuatorVoltages, dmData.data()));
        SDKErrChk(ExecuteApiFunction(hControllerElement1, "BMCSetArray"));
        SDKErrChk(UnlockElement(hControllerElement1));

        if (_kbhit()) {
            char c = _getch();
            if (c == 'x' || c == 'X' || c == 27) stop = true;
        }
        Sleep(100);
    }
    // Ao sair com 'x': zerar coeficientes (estado inicial)
    // e restaurar os atuadores para a base (offsetCommands)
    SDKErrChk(LockElement(hControllerElement1));
    for (int i = 0; i < Nact; ++i) {
        dmData[i] = offsetCommands[i];
    }
    SDKErrChk(SetDataContent(hArrayActuatorVoltages, dmData.data()));
    SDKErrChk(ExecuteApiFunction(hControllerElement1, "BMCSetArray"));
    SDKErrChk(UnlockElement(hControllerElement1));

    closeControlSystem();
    return Rtn_OK;
}
