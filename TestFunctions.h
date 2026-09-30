#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <shlobj.h>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <thread>
#include <complex>

#include "C:\Program Files\Thorlabs\ThorAOControl\SDK\Inc\TL_ControlSystemSDK.h"
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <string>
#include <iostream>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


extern char errStr[STRING_LENGTH_BUFFER_512];

extern char str[STRING_LENGTH_BUFFER];
extern char str1[STRING_LENGTH_BUFFER];
extern int strLength;
extern int strLength1;

extern char c_szPath[STRING_LENGTH_BUFFER];

extern ModuleHandle *processor_ModuleHandles;
extern ModuleHandle *sensor_ModuleHandles;
extern ModuleHandle *controller_ModuleHandles;

extern ModuleHandle *procExt_ModuleHandles;

extern int numModules;

extern PluginHandle *deviceHandles;
extern int numDevices;

extern ExtensionHandle extensionHandle;

extern PluginHandle instanceHandle;

extern PluginHandle hDev;
extern PluginHandle hProcessor;
extern PluginHandle hSensor;
extern PluginHandle hWoofer;
extern PluginHandle hTweeter;
extern ExtensionHandle hProcessorExt;

extern CtrlSysHandle hCtrlSys;
extern CtrlNodeHandle hRootNode;
extern CtrlNodeHandle hCtrlNode;
extern CtrlElementHandle hProcessorElement;
extern CtrlElementHandle hSensorElement;
extern CtrlElementHandle hControllerElement;
extern CtrlElementHandle hControllerElement1;

extern DataHandle hSpotFieldImage;
extern DataHandle hWavefront;
extern DataHandle hSpotDeviations;

extern double minIntensity;
extern double maxIntensity;
extern double meanIntensity;
extern double RMSofIntensity;

extern DataHandle hArrayActuatorVoltages;

extern int iBMC_DM_DataSize1;
extern int iBMC_DM_DataSize2;

extern DataHandle hSegmentData;
extern DataHandle hSelectedZernikeTermAmplitudes;
extern int iSegmentCount;
extern int iZernikeCoefficientCount;

extern bool bStopClosedLoopControl;
extern bool bThreadTerminated;

#define Rtn_OK		0
#define Rtn_ERROR	-1

// For functions without return value
#define SDKErrChk(functionCall) functionCall; if (GetError(errStr, STRING_LENGTH_BUFFER_512)) {std::cout << "\nError: " << errStr << "\n\nPress any key to exit the testing.\n"; getchar(); return Rtn_ERROR-1;}

// For functions with return value
#define ErrChk(functionName) if (GetError(errStr, STRING_LENGTH_BUFFER_512)) {std::cout << "\nError: " << errStr << "\n\nPress any key to exit the testing.\n"; getchar(); return Rtn_ERROR;}

int ModuleAndDeviceExample();
int findPluginModules(_PluginType moduleType);
int findExtensionModules(_ExtensionType extModuleType);
int findDeviceHandles(_PluginType pluginType, ModuleHandle moduleHandle);
int getProcessorInstanceHandle(_PluginType pluginType, ModuleHandle moduleHandle, PluginHandle *instanceHandle);
int getExtensionHandle(ModuleHandle moduleHandle, ExtensionHandle *extensionHandles);

int Thorlabs_WavefrontSensor_Example();
int create_WFS_ControlSystem();
int set_WFS_Parameters();
int get_WFS_Parameters();
int set_WFS_Outputs();
int get_WFS_Outputs();
int get_WFS_DataObject();

int BMC_DeformableMirror_Example();
int create_BMC_DM_ControlSystem();
int set_BMC_DM_Parameters();
int get_BMC_DM_Parameters();
int get_BMC_DM_DataObject();
int send_BMC_DM_Signal();

int Thorlabs_DMP40_DeformableMirror_Example();
int create_DMP40_DM_ControlSystem();
int set_DMP40_DM_Parameters();
int get_DMP40_DM_Parameters();
int get_DMP40_DM_DataObject();
int send_DMP40_DM_Signal();

int WooferTweeter_LagrangeBased_Example();
int create_WT_LagrangeBased();
int calibrate_WT_LagrangeBased();
int run_WT_LagrangeBased_ClosedLoop();
int start_node();

int DataAndFileOperationExample();
int create_TLD_file();
int unzip_TLD_file();
int export_TLD_file();

int ExecuteDeviceFactoryApiFunctionExample();
int get_WFS_Info();
int config_WFS();
int read_WFS_Image();
int calculate_Spots_CentrDiaIntens();
int calculate_beam_centrDia();
int calculate_Spot_Deviations();
int calculate_Wavefront();
int calculate_Zernike();

int closeControlSystem();

// funções criadas
int apply_line_pattern_to_DMP40();
int Plotar_Linha();
int mascara();
int LoopMascara();
int LoopMascara_BMC();

// --- Funções extras de teste para DMP40 ---
// Aplica uma máscara de fase arbitrária, ativando os modos especificados
int apply_mask_to_DMP40(const std::vector<int>& modesOn, double amplitude);

// Aplica um padrão linear (liga os modos de firstMode até lastMode em sequência)
int apply_line_pattern_to_DMP40_range(int firstMode, int lastMode, double amplitude, int dwell_ms);

// Aplica uma máscara senoidal em um conjunto de modos (variação harmônica dos coeficientes)
int apply_sinusoidal_mask_to_DMP40(int firstMode, int lastMode, double amplitude_peak, double cycles);

// ==== Utilitários Zernike (baseados em Noll) ====
double zernikeRadial(int n, int m, double r);
double zernikePolynomial(int n, int m, double r, double theta);

// ---- Fechamento em malha com DMP40 + WFS ----
int WFS_DMP40_ClosedLoop_Example();
int create_WFS_DMP40_ControlSystem();
int run_WFS_DMP40_ZernikeClosedLoop(int maxModes, double gain, bool cancelTilt, int sleepMs);

// Closed-loop (DM40 + WFS) with on/off toggle for DM actuation
int WFS_DMP40_ClosedLoop_Toggle_Example();
int run_WFS_DMP40_ZernikeClosedLoop_Toggle(int maxModes, double gain, bool cancelTilt, int sleepMs);

// Sensor-only WFS monitor
int WFS_SensorOnly_Example();

// Teste: incrementa Z1 (Noll=1) em passos de 0.1 no DMP40
int dm40();

// Simulation (offline) mode
// Returns true if env var THOR_SIMULATE is set to 1/true/on/yes
//
bool IsSimulate();

// Geração de base de Zernike até N modos
std::vector<std::vector<double>> generateZernikeBasis(int N, int gridSize);

// Projeção de uma tela de fase W(x,y) nos polinômios de Zernike
std::vector<double> projectPhaseOntoZernike(
    const std::vector<std::vector<double>>& phaseScreen,
    const std::vector<std::vector<std::vector<double>>>& zernikeBasis);

// ==== Funções de aplicação no DMP40 ====
int apply_mask_to_DMP40(const std::vector<int>& modesOn, double amplitude);
int apply_line_pattern_to_DMP40_range(int firstMode, int lastMode,
                                      double amplitude, int dwell_ms);
int apply_sinusoidal_mask_to_DMP40(int firstMode, int lastMode,
                                   double amplitude_peak, int cycles);

                                   // --- Geração de tela Kolmogorov (fase em radianos, grid NxN) ---
std::vector<std::vector<double>> generate_kolmogorov_phase_plane_waves(
    int N, double numCycles, double r0_relD, int numModes);


// --- Decomposição da tela nos K primeiros Zernike (saída em radianos) ---
bool decompose_phase_firstK_using_zernikePolynomial(
    const std::vector<std::vector<double>>& phase_rad, // <<-- 2D
    int K,
    std::vector<double>& coeffs_rad);
// no topo do header: #include <vector> etc.
double zernikePolynomial(int n, int m, double r, double theta);
// === Implementações que casam com as assinaturas do .h ===
double zernikeRadial(int n, int m, double r); 

double zernikePolynomial(int n, int m, double r, double theta);

// ==== Utilitários Zernike (baseados em Noll) ====

// Radial function R_n^m(r)
double R_nm(int n, int m, double r);

// Zernike polynomial Z_n^m(r,theta)
double Z_nm(int n, int m, double r, double theta);

// Compatível com o restante do código
double zernikeRadial(int n, int m, double r);
double zernikePolynomial(int n, int m, double r, double theta);

int WFS_DMP40_ANSI_Calibrated_Example();
int run_WFS_DMP40_ANSI_Calibrated(int maxModes, double overallGain, int sleepMs);  