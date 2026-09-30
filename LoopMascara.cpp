#include "TestFunctions.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <conio.h>   // _kbhit, _getch

/*
  LoopMascara.cpp — explicacoes e referencias

  Visao geral:
  - Gera fase turbulenta (modelo estatistico de Kolmogorov),
    projeta em base de Zernike (indexacao de Noll) e envia os
    primeiros K coeficientes ao DMP40 via SDK.

  Principais relacoes e formulas:
  - Kolmogorov (estatistica da fase):
      D_phi(rho) = 6.88 * (rho / r0)^(5/3)
      Phi_phi(k) = 0.023 * r0^(-5/3) * |k|^(-11/3)  (PSD)
    A fase sintetizada aqui (generate_kolmogorov_phase_plane_waves)
    segue a ideia de somar ondas planas ponderadas pelo PSD.

  - Zernike e indexacao de Noll:
    j -> (n,m) via mapeamento de Noll; Z_j(r,theta) = Z_n^m(r,theta).
    Projetamos a fase amostrada b em A (base Zernike por pixel) via
    minimos quadrados: resolver (A^T A) c = A^T b (com Tikhonov leve).

  - Fase [rad] e OPD [um]:
      phi = (2*pi/lambda) * OPD  =>  OPD = phi * (lambda / 2*pi)
    Se o driver/espelho espera deslocamento de superficie h (espelho
    refletivo), vale OPD = 2*h. Neste codigo assumimos unidade em OPD
    ao converter radianos para micrometros.

  Referencias:
  - Noll, R. J. (1976). Zernike polynomials and atmospheric turbulence. JOSA.
  - Fried, D. L. (1966). Optical resolution through a randomly inhomogeneous medium. JOSA.
  - Goodman, J. W. (2000). Statistical Optics. Wiley.
  - Roddier, F. (1999). Adaptive Optics in Astronomy. Cambridge Univ. Press.
  - Born, M.; Wolf, E. (1999). Principles of Optics. Cambridge Univ. Press.
*/
// ===== Conversão Noll j -> (n,m) (j>=1) =====
static inline void noll_to_nm(int j, int& n, int& m) {
    // Noll (1976): mapeia j (1-indexado) para par (n,m) em Zernike.
    // Usado para avaliar Z_j(r,theta) = Z_n^m(r,theta) por pixel.
    int j1 = j - 1;
    n = 0;
    while (j1 > n) { j1 -= (n + 1); ++n; }
    int k = j1;
    int s = (n & 1) ? 1 : 0;     // paridade de n
    m = 2 * k - n;
    if (s) m = -m;
}

// ===== Cache de geometria, base Zernike e (A^T A) =====
struct ZernCache {
    bool ready = false;
    int N = 0;
    int K = 0;
    int P = 0;                                 // nº de pixels na pupila
    std::vector<int> pix;                      // índices lineares y*N + x
    std::vector<double> Zall;                  // tamanho P*K: Z(pixel p, modo j)
    std::vector<double> ATA;                   // K*K (simétrica)
};
static ZernCache g_cache;

// Pré-computa máscara da pupila, Zernike por pixel e ATA
static void precompute_cache(int N, int K) {
    // Coordenadas sobre disco unitario:
    // X = (x - c)/Rn, Y = (y - c)/Rn, r = sqrt(X^2+Y^2), theta = atan2(Y,X)
    // com c = 0.5*(N-1) e Rn=c. Base para avaliar Zernike por pixel.
    if (g_cache.ready && g_cache.N == N && g_cache.K == K) return;

    g_cache = ZernCache{};
    g_cache.N = N; g_cache.K = K;

    const double c  = 0.5 * (N - 1);
    const double Rn = c;

    // 1) lista de pixels da pupila (disco unitário)
    g_cache.pix.clear();
    g_cache.pix.reserve(N * N);
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            double X = (x - c) / Rn;
            double Y = (y - c) / Rn;
            if (X*X + Y*Y <= 1.0) {
                g_cache.pix.push_back(y * N + x);
            }
        }
    }
    g_cache.P = (int)g_cache.pix.size();

    // 2) Zernike por pixel (usa sua zernikePolynomial(n,m,r,theta))
    g_cache.Zall.assign((size_t)g_cache.P * K, 0.0);
    std::vector<double> z(K, 0.0);
    for (int p = 0; p < g_cache.P; ++p) {
        int idx = g_cache.pix[p];
        int y = idx / N, x = idx % N;
        double X = (x - c) / Rn;
        double Y = (y - c) / Rn;
        double r = std::sqrt(X*X + Y*Y);
        double th = std::atan2(Y, X);

        for (int j = 1; j <= K; ++j) {
            int n, m; noll_to_nm(j, n, m);
            z[j-1] = zernikePolynomial(n, m, r, th);
        }
        // copia p/ Zall
        std::copy(z.begin(), z.end(), g_cache.Zall.begin() + (size_t)p * K);
    }

    // 3) A^T A = sum_p z_p * z_p^T
    g_cache.ATA.assign((size_t)K * K, 0.0);
    for (int p = 0; p < g_cache.P; ++p) {
        const double* zp = &g_cache.Zall[(size_t)p * K];
        for (int a = 0; a < K; ++a) {
            double za = zp[a];
            for (int b = 0; b < K; ++b) {
                g_cache.ATA[(size_t)a * K + b] += za * zp[b];
            }
        }
    }

    g_cache.ready = true;
}

// ATb = sum_p z_p * b_p  (b = fase em radianos)
static void compute_ATb(const std::vector<std::vector<double>>& phase, std::vector<double>& ATb) {
    // Computa o termo direito das equacoes normais: ATb (projecao de b em A).
    const int N = g_cache.N;
    const int K = g_cache.K;

    ATb.assign((size_t)K, 0.0);

    for (int p = 0; p < g_cache.P; ++p) {
        int idx = g_cache.pix[p];
        int y = idx / N, x = idx % N;
        double b = phase[y][x];
        const double* zp = &g_cache.Zall[(size_t)p * K];
        for (int a = 0; a < K; ++a) {
            ATb[a] += zp[a] * b;
        }
    }
}

// Resolve (ATA + λI) c = ATb por eliminação gaussiana simples
static bool solve_normal_eq(const std::vector<double>& ATb, std::vector<double>& c_out) {
    // Resolve (ATA + lambda I) c = ATb com eliminacao gaussiana e pivoteamento parcial.
    // Tikhonov leve (lambda ~ 1e-8) melhora condicao numerica. Alternativas: Cholesky, QR, SVD.
    const int K = g_cache.K;
    // copia ATA do cache e aplica regularização leve
    std::vector<double> A = g_cache.ATA;
    const double lambda = 1e-8;
    for (int i = 0; i < K; ++i) A[(size_t)i * K + i] += lambda;

    c_out = ATb;

    // forward (pivoteamento parcial)
    for (int k = 0; k < K; ++k) {
        int piv = k;
        double best = std::fabs(A[(size_t)k * K + k]);
        for (int i = k + 1; i < K; ++i) {
            double v = std::fabs(A[(size_t)i * K + k]);
            if (v > best) { best = v; piv = i; }
        }
        if (best < 1e-14) return false;

        if (piv != k) {
            for (int j = k; j < K; ++j) std::swap(A[(size_t)k * K + j], A[(size_t)piv * K + j]);
            std::swap(c_out[k], c_out[piv]);
        }
        double diag = A[(size_t)k * K + k];
        for (int j = k; j < K; ++j) A[(size_t)k * K + j] /= diag;
        c_out[k] /= diag;

        for (int i = k + 1; i < K; ++i) {
            double f = A[(size_t)i * K + k];
            for (int j = k; j < K; ++j) A[(size_t)i * K + j] -= f * A[(size_t)k * K + j];
            c_out[i] -= f * c_out[k];
        }
    }
    // back-substitution
    for (int i = K - 1; i >= 0; --i) {
        for (int j = i + 1; j < K; ++j)
            c_out[i] -= A[(size_t)i * K + j] * c_out[j];
    }
    return true;
}

// buffer reutilizado para reduzir alocações
static std::vector<double> zern_prev;

int LoopMascara()
{
    // Loop principal:
    // - Gera fase turbulenta (Kolmogorov)
    // - Projeta em K modos de Zernike via (A^T A)c = A^T b
    // - Converte radianos -> micrometros (OPD) e envia ao DMP40.
    if (IsSimulate())
    {
        std::cout << "[SIM] LoopMascara (offline)" << std::endl;
        std::cout << "[SIM] Streaming synthetic Zernike masks. Press 'x' to stop (no-op in sim)." << std::endl;
        Sleep(300);
        return Rtn_OK;
    }
    if (create_DMP40_DM_ControlSystem() == Rtn_ERROR) {
        std::cout << "\nCan not create a control system with Thorlabs DMP40 deformable mirror\n";
        return Rtn_ERROR;
    }
    if (set_DMP40_DM_Parameters() == Rtn_ERROR) return Rtn_ERROR;
    if (get_DMP40_DM_Parameters() == Rtn_ERROR) return Rtn_ERROR;
    if (get_DMP40_DM_DataObject() == Rtn_ERROR) return Rtn_ERROR;

    // Parâmetros da máscara turbulenta
    const int    Ngrid     = 32;
    const int    K         = std::min(12, iZernikeCoefficientCount);
    const double lambda_um = 0.6328;    // HeNe típico, ajuste conforme seu sistema
    const double r0_relD   = 1;       // turbulência
    const double numCycles = 10.0;
    const int    numModes  = 100;
    // Ngrid: tamanho da malha para amostrar a pupila (disco unitario)
    // K: quantidade de modos de Zernike usados na projecao
    // lambda_um: comprimento de onda para conversao rad -> micrometros (OPD)
    // r0_relD: parametro de Fried relativo ao diametro (turbulencia)
    // numCycles/numModes: controle da sintese de ondas planas no gerador de fase

    // Pré-cálculo (cache persistente entre execuções)
    precompute_cache(Ngrid, K);

    // Prepara buffer para o vetor zernike completo do plugin
    if (zern_prev.size() != (size_t)iZernikeCoefficientCount)
        zern_prev.assign((size_t)iZernikeCoefficientCount, 0.0);

    bool stop = false;
    std::cout << "Máscara aplicada. Pressione 'x' para parar.\n";
    while (!stop) {
        // 1) Gera fase (rad) com espectro de Kolmogorov
        //    D_phi(rho) = 6.88 * (rho/r0)^(5/3)
        //    PSD: Phi_phi(k) ~ |k|^(-11/3)
        // Referencias: Fried (1966), Goodman (2000), Roddier (1999)
        auto phase_rad = generate_kolmogorov_phase_plane_waves(Ngrid, numCycles, r0_relD, numModes);

        // 2) ATb e resolve (A^TA)c=ATb
        std::vector<double> ATb, c_rad;
        compute_ATb(phase_rad, ATb);
        if (!solve_normal_eq(ATb, c_rad)) {
            std::cout << "Falha ao resolver normal equations.\n";
            break;
        }

        // 3) rad -> µm (ou mude para 'waves' se seu plugin usar ondas)
        // Converte fase [rad] em OPD [um]: OPD = phi * lambda / (2*pi)
        // Observacao: para deslocamento de superficie (espelho refletivo), OPD = 2*h.
        const double rad_to_um = lambda_um / (2.0 * M_PI);
        for (double& v : c_rad) v *= rad_to_um;

        // 4) Envia para o DMP40 (primeiros K coeficientes; demais = 0)
        std::vector<double> zern(iZernikeCoefficientCount, 0.0);
        std::copy(c_rad.begin(), c_rad.end(), zern.begin());
        // Sequencia do SDK: setar vetor -> calcular padrao -> aplicar tensoes

        SDKErrChk(LockElement(hControllerElement));
        SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, zern.data()));
        SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));
        SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));
        SDKErrChk(UnlockElement(hControllerElement));

        // recicla buffer
        zern_prev.swap(zern);

        
        if (_kbhit()) {
            char c = _getch();
            if (c == 'x' || c == 'X' || c == 27) stop = true; // ESC
        }

        Sleep(50); // ajuste conforme sua taxa desejada
    }

    closeControlSystem();
    return Rtn_OK;
}
