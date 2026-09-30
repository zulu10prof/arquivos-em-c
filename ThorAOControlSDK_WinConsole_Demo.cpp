// ThorAOControlSDK_WinConsole_Demo.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "C:\Program Files\Thorlabs\ThorAOControl\Examples\ThorAOControlSDK_WinConsole_Demo\TestFunctions.h"

char errStr[STRING_LENGTH_BUFFER_512];

char str[STRING_LENGTH_BUFFER];
char str1[STRING_LENGTH_BUFFER];
int strLength = STRING_LENGTH_BUFFER;
int strLength1 = STRING_LENGTH_BUFFER;

char c_szPath[STRING_LENGTH_BUFFER];

ModuleHandle *processor_ModuleHandles = NULL;
ModuleHandle *sensor_ModuleHandles = NULL;
ModuleHandle *controller_ModuleHandles = NULL;

ModuleHandle *procExt_ModuleHandles = NULL;

int numModules = 0;

PluginHandle *deviceHandles = NULL;
int numDevices = 0;

ExtensionHandle extensionHandle = NULL;

PluginHandle instanceHandle = NULL;

PluginHandle hDev;
PluginHandle hProcessor = NULL;
PluginHandle hSensor = NULL;
PluginHandle hWoofer = NULL;
PluginHandle hTweeter;
ExtensionHandle hProcessorExt;

CtrlSysHandle hCtrlSys = NULL;
CtrlNodeHandle hRootNode = NULL;
CtrlNodeHandle hCtrlNode = NULL;
CtrlElementHandle hProcessorElement = NULL;
CtrlElementHandle hSensorElement = NULL;
CtrlElementHandle hControllerElement = NULL;
CtrlElementHandle hControllerElement1 = NULL;

DataHandle hSpotFieldImage = NULL;
DataHandle hWavefront = NULL;
DataHandle hSpotDeviations = NULL;

double minIntensity = 0.0;
double maxIntensity = 0.0;
double meanIntensity = 0.0;
double RMSofIntensity = 0.0;

DataHandle hArrayActuatorVoltages = NULL;

int iBMC_DM_DataSize1;
int iBMC_DM_DataSize2;

DataHandle hSegmentData = NULL;
DataHandle hSelectedZernikeTermAmplitudes = NULL;
int iSegmentCount;
int iZernikeCoefficientCount;

bool bStopClosedLoopControl = false;
bool bThreadTerminated = true;

int main()
{
    if (IsSimulate())
    {
        std::cout << "[SIM] Running in simulation (offline) mode. No hardware required." << std::endl;
    }

	SDKErrChk(OpenPluginManager());
		
	SDKErrChk(GetSDKPropertyString(_SDKProperty::SDK_MajorVersion, str, &strLength));

	strLength = STRING_LENGTH_BUFFER;
	SDKErrChk(GetSDKPropertyString(_SDKProperty::SDK_MinorVersion, str1, &strLength));

	std::cout << "SDK Version: " << str << "." << str1 << std::endl;	
		
	strLength = STRING_LENGTH_BUFFER;
	SDKErrChk(GetControlSystemDirectory(str, &strLength));
	std::cout << "Software directory: " << str << std::endl;

	std::cout << "\nThe following tests are available: \n\n";
	std::cout << "a: Manage Plugin modules/devices\n";
	std::cout << "b: Create a sensor-only control system with Thorlabs wavefront sensor\n";
	std::cout << "c: Create a controller-only control system with Boston Micromachine Multi deformable mirror\n";
	std::cout << "d: Create a controller-only control system with Thorlabs DMP40 deformable mirror\n";
	std::cout << "e: Create a Lagrange algorithm based Woofer-Tweeter control system\n";
	std::cout << "f: Example of data and file operation with Thorlabs wavefront sensor\n"; 
	std::cout << "g: Example of calling device driver functions Thorlabs wavefront sensor\n";
	std::cout << "h: teste de plotar linha no espelho\n";
	std::cout << "i: teste de mascara de fase\n";
	std::cout << "j: Loop mascara de fase\n";
	std::cout << "k: Loop fechado DMP40 + WFS\n";
	std::cout << "l: Loop mascara BMC 140\n";
	std::cout << "m: Monitor WFS (somente sensor)\n";
	std::cout << "n: Apenas DM40 no projeto (teste Z1)\n";
	std::cout << "o: Loop fechado DM40 + WFS (toggle espelho)\n";
	std::cout << "p: Loop fechado ANSI calibrado (WFS->DM40)\n";
	std::cout << "x: Exit\n\n";

	char c = getchar();		// Waiting for choice and enter
	getchar();				// read enter key

	switch (c)
	{
	case 'a':
	{
		ModuleAndDeviceExample();
		break;
	}

	case 'b':
	{
		Thorlabs_WavefrontSensor_Example();
		break;
	}

	case 'c':
	{
		BMC_DeformableMirror_Example();
		break;
	}

	case 'd':
	{
		Thorlabs_DMP40_DeformableMirror_Example();
		break;
	}

	case 'e':
	{
		WooferTweeter_LagrangeBased_Example();
		break;
	}

	case 'f':
	{
		DataAndFileOperationExample();
		break;
	}

	case 'g':
	{
		ExecuteDeviceFactoryApiFunctionExample();
		break;
	}
	case 'h':
	{
		Plotar_Linha();
		break;
	}
	case 'i':
	{
		mascara();
		break;
	}
	case 'j':
	{
		LoopMascara();
		break;
	}
	case 'k':
	{
		WFS_DMP40_ClosedLoop_Example();
		break;
	}
	case 'l':
	{
		LoopMascara_BMC();
		break;
	}
	case 'm':
	{
		WFS_SensorOnly_Example();
		break;
	}
	case 'n':
	{
		dm40();
		break;
	}
	case 'o':
	{
		WFS_DMP40_ClosedLoop_Toggle_Example();
		break;
	}
	case 'p':
	{
		WFS_DMP40_ANSI_Calibrated_Example();
		break;
	}
	case 'x':
		break;

	default:
		std::cerr << "Function key not supported!" << std::endl;
		break;
	}

	SDKErrChk(ClosePluginManager());

	printf("\nTest is over. \nPress any key to exit the program...");
	getchar();

	return Rtn_OK;
}
