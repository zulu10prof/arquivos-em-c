#include "TestFunctions.h"
#include <cstdlib>
#include <cctype>

static bool g_simulate_inited = false;
static bool g_simulate = false;

bool IsSimulate()
{
    if (!g_simulate_inited)
    {
        g_simulate_inited = true;
        #ifdef _WIN32
        char* envBuf = nullptr;
        size_t envLen = 0;
        if (_dupenv_s(&envBuf, &envLen, "THOR_SIMULATE") == 0 && envBuf)
        {
            std::string s(envBuf);
            free(envBuf);
        #else
        const char* env = std::getenv("THOR_SIMULATE");
		}
        if (env)
        {
            std::string s(env);
        #endif
            for (auto& c : s) c = (char)tolower((unsigned char)c);
            if (s == "1" || s == "true" || s == "on" || s == "yes")
                g_simulate = true;
        }
    }
    return g_simulate;
	
}

int findPluginModules(_PluginType moduleType)
{
	if (moduleType == _PluginType::Processor)
		processor_ModuleHandles = (ModuleHandle *)GetAvailablePluginModules(moduleType, &numModules);
	else if (moduleType == _PluginType::Sensor)
		sensor_ModuleHandles = (ModuleHandle *)GetAvailablePluginModules(moduleType, &numModules);
	else if (moduleType == _PluginType::Controller)
		controller_ModuleHandles = (ModuleHandle *)GetAvailablePluginModules(moduleType, &numModules);
	else
		processor_ModuleHandles = (ModuleHandle *)GetAvailablePluginModules(moduleType, &numModules);

	ErrChk("GetAvailablePluginModules");

	return Rtn_OK;
}

int findExtensionModules(_ExtensionType extModuleType)
{
	if (extModuleType == _ExtensionType::ProcessorExtension)
		procExt_ModuleHandles = (ModuleHandle *)GetAvailableExtensionModules(extModuleType, &numModules);
	else
		procExt_ModuleHandles = (ModuleHandle *)GetAvailableExtensionModules(extModuleType, &numModules);

	ErrChk("GetAvailableExtensionModules");

	return Rtn_OK;
}

int findDeviceHandles(_PluginType pluginType, ModuleHandle moduleHandle)
{
	if (pluginType == _PluginType::Processor)
	{
		deviceHandles = (PluginHandle *)GetProcessorInstance(moduleHandle);
		ErrChk("GetProcessorInstance");
		numDevices = 1;
	}
	else
	{
		deviceHandles = (PluginHandle *)GetPluginDevices(moduleHandle, &numDevices);
		ErrChk("GetPluginDevices");
	}

	return Rtn_OK;
}

int getProcessorInstanceHandle(_PluginType pluginType, ModuleHandle moduleHandle, PluginHandle *instanceHandle)
{
	*instanceHandle = (PluginHandle)GetProcessorInstance(moduleHandle);
	ErrChk("GetProcessorInstance");

	return Rtn_OK;
}

int getExtensionHandle(ModuleHandle moduleHandle, ExtensionHandle *extensionHandle)
{
	*extensionHandle = (ExtensionHandle)GetExtensionInstance(moduleHandle);
	ErrChk("GetExtensionInstance");

	return Rtn_OK;
}

int create_WFS_ControlSystem()
{
	if (findPluginModules(_PluginType::Sensor) == Rtn_ERROR)
		return Rtn_ERROR;

	if (numModules > 0)
	{
		for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++)
		{
			char str[STRING_LENGTH_BUFFER];
			int strLength = STRING_LENGTH_BUFFER;
			SDKErrChk(GetModuleProperty(sensor_ModuleHandles[moduleIndex], _ModuleProperty::Module_Name, str, &strLength));
			if (strcmp(str, "Thorlabs Wavefront Sensor Plugin Module") == 0)
			{
				std::cout << "Getting plugin handle..." << std::endl;
				if (findDeviceHandles(_PluginType::Sensor, sensor_ModuleHandles[moduleIndex]) == Rtn_ERROR)
					return Rtn_ERROR;

				if (numDevices > 0)
				{
					hDev = deviceHandles[0];

					std::cout << "Creating control system..." << std::endl;
					hCtrlSys = CreateControlSystem("WFS_Only_ControlSystem");
					ErrChk("CreateControlSystem");

					hRootNode = GetRootControlNode(hCtrlSys);
					ErrChk("GetRootControlNode");

					std::cout << "Creating control node..." << std::endl;
					hCtrlNode = NewControlNode(hCtrlSys, hRootNode, "root");
					ErrChk("NewControlNode");

					std::cout << "Adding Thorlabs wavefront sensor plugin handle to the created control node..." << std::endl;
					hSensorElement = AddControlElement(hCtrlNode, hDev, "Wavefront_Sensor");
					ErrChk("AddControlElement");
				}
				else
					return Rtn_ERROR;
			}
		}
	}
	else
		return Rtn_ERROR;

	return Rtn_OK;
}

int set_WFS_Parameters()
{
	std::cout << std::endl;

	std::cout << "Setting wavefront sensor parameters..." << std::endl;
	int iValue = 2;
	SDKErrChk(SetNumericParameter(hSensorElement, "CamResolIndex", &iValue));

	iValue = 0;
	SDKErrChk(SetNumericParameter(hSensorElement, "NumImagesSkipped", &iValue));

	iValue = 3;
	SDKErrChk(SetNumericParameter(hSensorElement, "AverageCount", &iValue));

	// Enable auto exposure
	bool boolValue = true;
	SDKErrChk(SetBooleanParameter(hSensorElement, "ApplyAutoExposure", boolValue));

	double dpValue = 15.0;
	// SDKErrChk(SetNumericParameter(hSensorElement, "ExposureTimeSet", &dpValue));

	// dpValue = 1.0;
	// SDKErrChk(SetNumericParameter(hSensorElement, "MasterGainSet", &dpValue));

	// Disable Highspeed mode
	iValue = 0;
	SDKErrChk(SetNumericParameter(hSensorElement, "HighspeedMode", &iValue));

	// Disable Hardware trigger
	iValue = 0;
	SDKErrChk(SetNumericParameter(hSensorElement, "TriggerModeSel", &iValue));

	iValue = 7;
	SDKErrChk(SetNumericParameter(hSensorElement, "ZernikePolynomialOrderSel", &iValue));

	iValue = 0;
	SDKErrChk(SetNumericParameter(hSensorElement, "WavefrontUnitSel", &iValue));

	iValue = 1;
	SDKErrChk(SetNumericParameter(hSensorElement, "CancelWavefrontTilt", &iValue));

	dpValue = 0.0;
	SDKErrChk(SetNumericParameter(hSensorElement, "PupilCenterX", &dpValue));
	SDKErrChk(SetNumericParameter(hSensorElement, "PupilCenterY", &dpValue));

	dpValue = 3.0;
	SDKErrChk(SetNumericParameter(hSensorElement, "PupilDiameterX", &dpValue));
	SDKErrChk(SetNumericParameter(hSensorElement, "PupilDiameterY", &dpValue));

	iValue = 1;
	SDKErrChk(SetNumericParameter(hSensorElement, "LimitToPupil", &iValue));

	SDKErrChk(SetElementControlMode(hSensorElement, _ElementControlMode::Realtime));
	SDKErrChk(InitControlElement(hSensorElement));

	return Rtn_OK;
}

int get_WFS_Parameters()
{
	std::cout << std::endl;

	int iValue = 1;
	double dpValue = 0.0;
	bool boolValue = true;

	std::cout << "Getting wavefront sensor parameters..." << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "CamResolIndex", &iValue));
	std::cout << "Camera resolution index is set to " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "CamResolWidth", &iValue));
	std::cout << "Camera resolution width is " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "CamResolHeight", &iValue));
	std::cout << "Camera resolution height is " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "SpotsX", &iValue));
	std::cout << "Number of spots in X direction is " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "SpotsY", &iValue));
	std::cout << "Number of spots in Y direction is " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "AOIWidth", &dpValue));
	std::cout << "The width of AOI is " << dpValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "AOIHeight", &dpValue));
	std::cout << "The height of AOI is " << dpValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilDiameterX", &dpValue));
	std::cout << "The diameter of pupil in X direction is " << dpValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilDiameterY", &dpValue));
	std::cout << "The diameter of pupil in Y direction is " << dpValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "NumImagesSkipped", &iValue));
	std::cout << "Number of skipped images is set to " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "AverageCount", &iValue));
	std::cout << "Number of image average is set to " << iValue << std::endl;

	boolValue = GetBooleanParameter(hSensorElement, "ApplyAutoExposure");
	ErrChk("GetBooleanParameter");
	std::cout << "Auto exposure mode is set to " << boolValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "HighspeedMode", &iValue));
	std::cout << "High speed mode is set to " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "TriggerModeSel", &iValue));
	std::cout << "Trigger mode is set to " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "ZernikePolynomialOrderSel", &iValue));
	std::cout << "Zernike polynomial order is set to " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "WavefrontUnitSel", &iValue));
	std::cout << "Wavefront unit is set to " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "CancelWavefrontTilt", &iValue));
	std::cout << "Wavefront tilt cancellation is set to " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilCenterX", &dpValue));
	std::cout << "Pupil center in X direction is set to " << dpValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilCenterY", &dpValue));
	std::cout << "Pupil center in Y direction is set to " << dpValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilDiameterX", &dpValue));
	std::cout << "Pupil diameter in X direction is set to " << dpValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "PupilDiameterY", &dpValue));
	std::cout << "Pupil diameter in Y direction is set to " << dpValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "LimitToPupil", &iValue));
	boolValue = (iValue == 1) ? true : false;
	std::cout << "Wavefront calculation is limited to pupil: " << boolValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "ExposureTimeAct", &dpValue));
	std::cout << "Actual exposure time is set to " << dpValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "MasterGainAct", &dpValue));
	std::cout << "Actual master gain is set to " << dpValue << std::endl;

	return Rtn_OK;
}

int set_WFS_Outputs()
{
	std::cout << std::endl;

	std::cout << "Setting wavefront sensor outputs..." << std::endl;

	SDKErrChk(SetOutputByName(hSensorElement, "ArrayWavefront"));
	SDKErrChk(SetOutputByName(hSensorElement, "ArrayImageBuf"));
	SDKErrChk(SetOutputByName(hSensorElement, "ArraySpotDeviations"));
	SDKErrChk(SetOutputByName(hSensorElement, "WavefrontMax"));
	SDKErrChk(SetOutputByName(hSensorElement, "WavefrontMin"));
	SDKErrChk(SetOutputByName(hSensorElement, "WavefrontRMS"));

	return Rtn_OK;
}

int get_WFS_Outputs()
{
	std::cout << std::endl;

	std::cout << "Getting wavefront sensor outputs..." << std::endl;

	SDKErrChk(CatchElementOutput(hSensorElement));

	double dpOutput = 0.0;
	SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontMax", &dpOutput));
	std::cout << "Wavefront Max is " << dpOutput << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontMin", &dpOutput));
	std::cout << "Wavefront Min is " << dpOutput << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontRMS", &dpOutput));
	std::cout << "Wavefront RMS is " << dpOutput << std::endl;

	return Rtn_OK;
}

int get_WFS_DataObject()
{
	hSpotFieldImage = GetDataObjectHandle(hSensorElement, "ArrayImageBuf");
	ErrChk("GetDataObjectHandle");

	hWavefront = GetDataObjectHandle(hSensorElement, "ArrayWavefront");
	ErrChk("GetDataObjectHandle");

	hSpotDeviations = GetDataObjectHandle(hSensorElement, "ArraySpotDeviations");
	ErrChk("GetDataObjectHandle");

	return Rtn_OK;
}

int create_BMC_DM_ControlSystem()
{
	if (findPluginModules(_PluginType::Controller) == Rtn_ERROR)
		return Rtn_ERROR;

	if (numModules > 0)
	{
		for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++)
		{
			char str[STRING_LENGTH_BUFFER];
			int strLength = STRING_LENGTH_BUFFER;
			SDKErrChk(GetModuleProperty(controller_ModuleHandles[moduleIndex], _ModuleProperty::Module_Name, str, &strLength));

			if (strcmp(str, "BMC Mini/Multi Deformable Mirror Plugin Module") == 0)
			{
				std::cout << "Getting plugin handle..." << std::endl;
				if (findDeviceHandles(_PluginType::Controller, controller_ModuleHandles[moduleIndex]) == Rtn_ERROR)
					return Rtn_ERROR;

				if (numDevices > 0)
				{
					hDev = deviceHandles[0];

					std::cout << "Creating control system..." << std::endl;
					hCtrlSys = CreateControlSystem("BMC_DM_ControlSystem");
					ErrChk("CreateControlSystem");

					hRootNode = GetRootControlNode(hCtrlSys);
					ErrChk("GetRootControlNode");

					std::cout << "Creating control node..." << std::endl;
					hCtrlNode = NewControlNode(hCtrlSys, hRootNode, "root");
					ErrChk("NewControlNode");

					std::cout << "Adding BMC deformable mirror plugin handle to the created control node..." << std::endl;
					hControllerElement1 = AddControlElement(hCtrlNode, hDev, "BMC_DM");
					ErrChk("AddControlElement");
				}
				else
					return Rtn_ERROR;
			}
		}
	}
	else
		return Rtn_ERROR;

	return Rtn_OK;
}

int set_BMC_DM_Parameters()
{
	double VoltageToDisplacementCoeff0;
	double VoltageToDisplacementCoeff1;
	double VoltageToDisplacementCoeff2;
	double ActuatorMaxVoltage;

	std::cout << std::endl;
	std::cout << "Setting BMC Multi deformable mirror parameters..." << std::endl;

	// Fixed coefficients as requested: coef01=0.027, coef02=11.105, coef03=0
	VoltageToDisplacementCoeff0 = 0.027;
	VoltageToDisplacementCoeff1 = 11.105;
	VoltageToDisplacementCoeff2 = 0.0;

	std::cout << "\nUsing fixed Voltage_To_Displacement coefficients:" << std::endl;
	std::cout << "  Coeff0: " << VoltageToDisplacementCoeff0 << std::endl;
	std::cout << "  Coeff1: " << VoltageToDisplacementCoeff1 << std::endl;
	std::cout << "  Coeff2: " << VoltageToDisplacementCoeff2 << std::endl;

	SDKErrChk(SetNumericParameter(hControllerElement1, "VoltageToDisplacementCoeff0", &VoltageToDisplacementCoeff0));
	SDKErrChk(SetNumericParameter(hControllerElement1, "VoltageToDisplacementCoeff1", &VoltageToDisplacementCoeff1));
	SDKErrChk(SetNumericParameter(hControllerElement1, "VoltageToDisplacementCoeff2", &VoltageToDisplacementCoeff2));

	std::cout << "\n";

	double dpValue = 0.3;
	SDKErrChk(SetNumericParameter(hControllerElement1, "StrokeStartPercent", &dpValue));

	dpValue = 0.7;
	SDKErrChk(SetNumericParameter(hControllerElement1, "StrokeEndPercent", &dpValue));

	dpValue = -70;
	SDKErrChk(SetNumericParameter(hControllerElement1, "SignalCorrectionFactor", &dpValue));

	int iValue = 5;
	SDKErrChk(SetNumericParameter(hControllerElement1, "ActuatorSettleTime", &iValue));

	return Rtn_OK;
}

int get_BMC_DM_Parameters()
{
	double VoltageToDisplacementCoeff0;
	double VoltageToDisplacementCoeff1;
	double VoltageToDisplacementCoeff2;
	double ActuatorMaxVoltage;

	std::cout << std::endl;
	std::cout << "Getting BMC Multi deformable mirror parameters..." << std::endl;

	double dpRetValue = 0.0;
	SDKErrChk(GetNumericParameter(hControllerElement1, _NumericParameterProperty::Value, "StrokeStartPercent", &dpRetValue));
	std::cout << "The stroke start percentage is set to " << dpRetValue << std::endl;

	SDKErrChk(GetNumericParameter(hControllerElement1, _NumericParameterProperty::Value, "StrokeEndPercent", &dpRetValue));
	std::cout << "The stroke end percentage is set to " << dpRetValue << std::endl;

	SDKErrChk(GetNumericParameter(hControllerElement1, _NumericParameterProperty::Value, "SignalCorrectionFactor", &dpRetValue));
	std::cout << "The signal correction factor is set to " << dpRetValue << std::endl;

	int iRetValue = 0;
	SDKErrChk(GetNumericParameter(hControllerElement1, _NumericParameterProperty::Value, "ActuatorSettleTime", &iRetValue));
	std::cout << "The actuator settlement time is set to " << iRetValue << std::endl;

	return Rtn_OK;
}

int get_BMC_DM_DataObject()
{
	std::cout << std::endl;
	std::cout << "Getting BMC Multi deformable mirror data object..." << std::endl;

	// When use Boston MicroMachine Multi / Mini deformable mirror in the AO control system,
	// the actuator voltages is used as the control signal.
	// We need to get the data handle of "ArrayActuatorVoltages", which actually holds the actuator's
	// displacement in nm. the plugin internally will convert the displacements to the voltages and send them
	// to the deformable mirror.
	hArrayActuatorVoltages = GetDataObjectHandle(hControllerElement1, "ArrayActuatorVoltages");
	ErrChk("GetDataObjectHandle");
	iBMC_DM_DataSize1 = GetDataPropertyInt(hArrayActuatorVoltages, _DataPropertyInt::Data_Size1);
	ErrChk("GetDataPropertyInt");
	iBMC_DM_DataSize2 = GetDataPropertyInt(hArrayActuatorVoltages, _DataPropertyInt::Data_Size2);
	ErrChk("GetDataPropertyInt");

	std::cout << "Number of rows of Acutators: " << iBMC_DM_DataSize1 << std::endl;
	std::cout << "Number of columns of Actuators: " << iBMC_DM_DataSize2 << std::endl;

	return Rtn_OK;
}

int send_BMC_DM_Signal()
{
	std::cout << "Sending data to Boston MicroMachine deformable mirror...\n\n";

	// Get Offset and Max commands from the signal object of the BMC Multi deformable mirror
	DataHandle hSignal = GetElementSignalObjectHandle(hControllerElement1);
	ErrChk("GetElementSignalObjectHandle");
	double *offsetCommands = (double *)GetControllerSignal_ArrayPtr(hSignal, _ControllerSignal_InternalArray::ControllerSignal_Calibration_OffsetCommands);
	ErrChk("GetControllerSignal_ArrayPtr");
	double *maxCommands = (double *)GetControllerSignal_ArrayPtr(hSignal, _ControllerSignal_InternalArray::ControllerSignal_Calibration_MaxCommands);
	ErrChk("GetControllerSignal_ArrayPtr");

	double *dmData = new double[iBMC_DM_DataSize1 * iBMC_DM_DataSize2];

	for (int i = 0; i < iBMC_DM_DataSize1 * iBMC_DM_DataSize2; i++)
		dmData[i] = offsetCommands[i];

	SDKErrChk(LockElement(hControllerElement1));

	for (int i = 0; i < iBMC_DM_DataSize1 * iBMC_DM_DataSize2; i++)
	{
		if (i > 0)
			dmData[i - 1] = offsetCommands[i - 1];
		dmData[i] = maxCommands[i];
		SDKErrChk(SetDataContent(hArrayActuatorVoltages, dmData));
		SDKErrChk(ExecuteApiFunction(hControllerElement1, "BMCSetArray"));

		Sleep(50);
	}

	SDKErrChk(UnlockElement(hControllerElement1));

	delete dmData;

	return Rtn_OK;
}

int create_DMP40_DM_ControlSystem()
{
	if (findPluginModules(_PluginType::Controller) == Rtn_ERROR)
		return Rtn_ERROR;

	if (numModules > 0)
	{
		for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++)
		{
			char str[STRING_LENGTH_BUFFER];
			int strLength = STRING_LENGTH_BUFFER;
			SDKErrChk(GetModuleProperty(controller_ModuleHandles[moduleIndex], _ModuleProperty::Module_Name, str, &strLength));
			if (strcmp(str, "Thorlabs DMP40 Deformable Mirror Plugin Module") == 0)
			{
				std::cout << "Getting plugin handle..." << std::endl;
				if (findDeviceHandles(_PluginType::Controller, controller_ModuleHandles[moduleIndex]) == Rtn_ERROR)
					return Rtn_ERROR;

				if (numDevices > 0)
				{
					hDev = deviceHandles[0];

					std::cout << "Creating control system..." << std::endl;
					hCtrlSys = CreateControlSystem("DMP40_Only_ControlSystem");
					ErrChk("CreateControlSystem");

					hRootNode = GetRootControlNode(hCtrlSys);
					ErrChk("GetRootControlNode");

					std::cout << "Creating control node..." << std::endl;
					hCtrlNode = NewControlNode(hCtrlSys, hRootNode, "root");
					ErrChk("NewControlNode");

					std::cout << "Adding Thorlabs DMP40 deformable mirror plugin handle to the created control node..." << std::endl;
					hControllerElement = AddControlElement(hCtrlNode, hDev, "DMP40");
					ErrChk("AddControlElement");
				}
				else
					return Rtn_ERROR;
			}
		}
	}
	else
		return Rtn_ERROR;

	return Rtn_OK;
}

int set_DMP40_DM_Parameters()
{
	std::cout << std::endl;
	std::cout << "Setting DMP40 deformable mirror parameters..." << std::endl;
	bool boolValue = false;
	SDKErrChk(SetBooleanParameter(hControllerElement, "ArmsHysteresisCompensation", boolValue));
	SDKErrChk(SetBooleanParameter(hControllerElement, "SegmentsHysteresisCompensation", boolValue));
	double dpValue = 0.0;
	SDKErrChk(SetNumericParameter(hControllerElement, "TiltAmplitude", &dpValue));
	SDKErrChk(SetNumericParameter(hControllerElement, "TiltAngle", &dpValue));
	int iValue = 0;
	SDKErrChk(SetNumericParameter(hControllerElement, "FlipMode", &iValue));
	SDKErrChk(SetNumericParameter(hControllerElement, "RotationMode", &iValue));
	iValue = 10;
	SDKErrChk(SetNumericParameter(hControllerElement, "ActuatorSettleTime", &iValue));
	// SignalSel = 0: Use segment voltages as control signal
	// SignalSel = 1: Use zernike coefficients as control signal
	// In our AO control, we always use zernike coefficients as control signal
    iValue = 1;
    SDKErrChk(SetNumericParameter(hControllerElement, "SignalSel", &iValue));
    // Use positive correction factor; loop already applies negative sign to measured Zernike
    dpValue = 1.0;
    SDKErrChk(SetNumericParameter(hControllerElement, "SignalCorrectionFactor", &dpValue));
	SDKErrChk(SetElementControlMode(hControllerElement, _ElementControlMode::Realtime));

	return Rtn_OK;
}

int get_DMP40_DM_Parameters()
{
	std::cout << std::endl;
	std::cout << "Getting BMC Multi deformable mirror parameters..." << std::endl;

	bool boolRetValue = false;
	boolRetValue = GetBooleanParameter(hControllerElement, "ArmsHysteresisCompensation");
	ErrChk("GetBooleanParameter");
	std::cout << "Arm hysteresis compensation is set to " << boolRetValue << std::endl;
	boolRetValue = GetBooleanParameter(hControllerElement, "SegmentsHysteresisCompensation");
	ErrChk("GetBooleanParameter");
	std::cout << "Segment hysteresis compensation is set to " << boolRetValue << std::endl;

	double dpRetValue = 1.0;
	SDKErrChk(GetNumericParameter(hControllerElement, _NumericParameterProperty::Value, "TiltAmplitude", &dpRetValue));
	std::cout << "Tilt amplitude is set to " << dpRetValue << std::endl;
	SDKErrChk(GetNumericParameter(hControllerElement, _NumericParameterProperty::Value, "TiltAngle", &dpRetValue));
	std::cout << "Tilt angle is set to " << dpRetValue << std::endl;

	int iRetValue = 1;
	SDKErrChk(GetNumericParameter(hControllerElement, _NumericParameterProperty::Value, "FlipMode", &iRetValue));
	std::cout << "Flip mode is set to " << iRetValue << std::endl;
	SDKErrChk(GetNumericParameter(hControllerElement, _NumericParameterProperty::Value, "RotationMode", &iRetValue));
	std::cout << "Rotation mode is set to " << iRetValue << std::endl;

	SDKErrChk(GetNumericParameter(hControllerElement, _NumericParameterProperty::Value, "ActuatorSettleTime", &iRetValue));
	std::cout << "Actuator settle time is set to " << iRetValue << std::endl;

	// SignalSel = 0: Use segment voltages as control signal
	// SignalSel = 1: Use zernike coefficients as control signal
	// In our AO control, we always use zernike coefficients as control signal
	SDKErrChk(GetNumericParameter(hControllerElement, _NumericParameterProperty::Value, "SignalSel", &iRetValue));
	std::cout << "Signal selection is set to " << iRetValue << std::endl;

	SDKErrChk(GetNumericParameter(hControllerElement, _NumericParameterProperty::Value, "SignalCorrectionFactor", &dpRetValue));
	std::cout << "Signal correction factor is set to " << dpRetValue << std::endl;

	SDKErrChk(SetElementControlMode(hControllerElement, _ElementControlMode::Realtime));

	return Rtn_OK;
}

int get_DMP40_DM_DataObject()
{
	std::cout << std::endl;
	std::cout << "Getting DMP40 deformable mirror data object..." << std::endl;

	// When use Thorlabs DMP40 in the AO control system, we always choose zernike coefficients as control signal.
	// We need to get data handle of "ArraySelectedZernikeTermAmplitudes". After the new value is set to "ArraySelectedZernikeTermAmplitudes",
	// function "ExecuteApiFunction("TLDFMX_calculate_zernike_pattern")" is called to get the new segment voltages,
	// and then function "ExecuteApiFunction(hThorCtrlElement, "TLDFM_set_segment_voltages")" is called to send the voltages to the deformable mirror.
	hSegmentData = GetDataObjectHandle(hControllerElement, "ArraySegmentVoltages");
	ErrChk("GetDataObjectHandle");

	hSelectedZernikeTermAmplitudes = GetDataObjectHandle(hControllerElement, "ArraySelectedZernikeTermAmplitudes");
	ErrChk("GetDataObjectHandle");

	iSegmentCount = GetDataPropertyInt(hSegmentData, _DataPropertyInt::Data_Size1);
	ErrChk("GetDataPropertyInt");
	std::cout << "Number of segment: " << iSegmentCount << std::endl;

    iZernikeCoefficientCount = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_Size1);
    ErrChk("GetDataPropertyInt");
    std::cout << "Number of Zernike coefficients: " << iZernikeCoefficientCount << std::endl;

    // Report data layout (no globals retained)
    int dt  = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_Type); ErrChk("GetDataPropertyInt");
    int bpc = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_BytesPerComponent); ErrChk("GetDataPropertyInt");
    int cpd = GetDataPropertyInt(hSelectedZernikeTermAmplitudes, _DataPropertyInt::Data_ComponentsPerData); ErrChk("GetDataPropertyInt");
    std::cout << "DM Zernike DataType=" << dt << ", BytesPerComponent=" << bpc << ", ComponentsPerData=" << cpd << std::endl;

    return Rtn_OK;
}

int send_DMP40_DM_Signal()
{
	std::cout << "Sending data to Thorlabs DMP40 deformable mirror...\n\n";

	// Get Offset and Max commands from the signal object of the Thorlabs DMP40 deformable mirror
	double *zernikeCoefficients = new double[iZernikeCoefficientCount];

	DataHandle hSignal = GetElementSignalObjectHandle(hControllerElement);
	ErrChk("GetElementSignalObjectHandle");
	double *offsetCommands = (double *)GetControllerSignal_ArrayPtr(hSignal, _ControllerSignal_InternalArray::ControllerSignal_Calibration_OffsetCommands);
	ErrChk("GetControllerSignal_ArrayPtr");
	double *maxCommands = (double *)GetControllerSignal_ArrayPtr(hSignal, _ControllerSignal_InternalArray::ControllerSignal_Calibration_MaxCommands);
	ErrChk("GetControllerSignal_ArrayPtr");
	// Imprimir os valores de offsetCommands
	std::cout << "Offset Commands:\n";

	for (int i = 0; i < iZernikeCoefficientCount; i++)
		zernikeCoefficients[i] = offsetCommands[i];

	SDKErrChk(LockElement(hControllerElement));

	for (int i = 0; i < iZernikeCoefficientCount; i++)
	{
		if (i > 0)
			zernikeCoefficients[i - 1] = offsetCommands[i - 1];
		zernikeCoefficients[i] = maxCommands[i];
		SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, zernikeCoefficients));

		std::cout << "Calculating zernike pattern..." << std::endl;
		SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));

		std::cout << "Sending segment voltages to the deformable mirror..." << std::endl;
		SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));
		Sleep(50);
	}

	SDKErrChk(UnlockElement(hControllerElement));

	delete zernikeCoefficients;

	return Rtn_OK;
}

int closeControlSystem()
{
	std::cout << std::endl;

	if (hProcessorElement)
	{
		std::cout << "Removing the processor element..." << std::endl;
		SDKErrChk(RemoveControlElement(hCtrlNode, hProcessorElement));
	}

	if (hSensorElement)
	{
		std::cout << "Removing the sensor element..." << std::endl;
		SDKErrChk(RemoveControlElement(hCtrlNode, hSensorElement));
	}

	if (hControllerElement)
	{
		std::cout << "Removing the controller element 0..." << std::endl;
		SDKErrChk(RemoveControlElement(hCtrlNode, hControllerElement));
	}

	if (hControllerElement1)
	{
		std::cout << "Removing the controller element 1..." << std::endl;
		SDKErrChk(RemoveControlElement(hCtrlNode, hControllerElement1));
	}

	std::cout << "Deleting the control node..." << std::endl;
	SDKErrChk(DeleteControlNode(hCtrlNode));

	std::cout << "Closing the control system..." << std::endl;
	SDKErrChk(CloseControlSystem(hCtrlSys));

	return Rtn_OK;
}

// ============================================================
// WFS + DMP40 Closed-Loop (Zernike-based)
// - Cria sistema com WFS (sensor) e DMP40 (controlador)
// - Mede Zernike no WFS e aplica correção no DMP40
// - j=1 (piston), j=2..3 (tilts) podem ser cancelados
// ============================================================

int create_WFS_DMP40_ControlSystem()
{
    std::cout << "[TRACE] Enter create_WFS_DMP40_ControlSystem" << std::endl;
    if (IsSimulate())
    {
        std::cout << "[SIM] create_WFS_DMP40_ControlSystem (offline)" << std::endl;
        return Rtn_OK;
    }

    hProcessor = NULL;
    hSensor    = NULL;
    hWoofer    = NULL; // usaremos como DMP40 (controlador principal)
    hTweeter   = NULL;

    if (findPluginModules(_PluginType::Sensor) == Rtn_ERROR)
        return Rtn_ERROR;
    if (numModules > 0)
    {
        for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++)
        {
            strLength = STRING_LENGTH_BUFFER;
            SDKErrChk(GetModuleProperty(sensor_ModuleHandles[moduleIndex], _ModuleProperty::Module_Name, str, &strLength));
            if (strcmp(str, "Thorlabs Wavefront Sensor Plugin Module") == 0)
            {
                std::cout << "Getting Wavefront Sensor plugin handle..." << std::endl;
                if (findDeviceHandles(_PluginType::Sensor, sensor_ModuleHandles[moduleIndex]) == Rtn_ERROR)
                    return Rtn_ERROR;
                if (numDevices > 0)
                    hSensor = deviceHandles[0];
                else
                    return Rtn_ERROR;
            }
        }
    }
    else return Rtn_ERROR;

    if (findPluginModules(_PluginType::Controller) == Rtn_ERROR)
        return Rtn_ERROR;
    if (numModules > 0)
    {
        for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++)
        {
            strLength = STRING_LENGTH_BUFFER;
            SDKErrChk(GetModuleProperty(controller_ModuleHandles[moduleIndex], _ModuleProperty::Module_Name, str, &strLength));
            if (strcmp(str, "Thorlabs DMP40 Deformable Mirror Plugin Module") == 0)
            {
                std::cout << "Getting DMP40 plugin handle..." << std::endl;
                if (findDeviceHandles(_PluginType::Controller, controller_ModuleHandles[moduleIndex]) == Rtn_ERROR)
                    return Rtn_ERROR;
                if (numDevices > 0)
                    hWoofer = deviceHandles[0];
                else
                    return Rtn_ERROR;
            }
        }
    }
    else return Rtn_ERROR;

    if (hSensor != NULL && hWoofer != NULL)
    {
        std::cout << "Creating control system..." << std::endl;
        hCtrlSys = CreateControlSystem("WFS_DMP40_ClosedLoop_ControlSystem");
        ErrChk("CreateControlSystem");

        hRootNode = GetRootControlNode(hCtrlSys);
        ErrChk("GetRootControlNode");

        std::cout << "Creating control node..." << std::endl;
        hCtrlNode = NewControlNode(hCtrlSys, hRootNode, "root");
        ErrChk("NewControlNode");

        std::cout << "Adding Sensor to the created control node..." << std::endl;
        hSensorElement = AddControlElement(hCtrlNode, hSensor, "Sensor");
        ErrChk("AddControlElement");

        std::cout << "Adding DMP40 to the created control node..." << std::endl;
        hControllerElement = AddControlElement(hCtrlNode, hWoofer, "DMP40");
        ErrChk("AddControlElement");

        // Configurar elementos
        SDKErrChk(SetElementControlMode(hSensorElement, _ElementControlMode::Realtime));
        SDKErrChk(SetElementControlMode(hControllerElement, _ElementControlMode::Realtime));

        // Parâmetros WFS e DMP40
        if (set_WFS_Parameters() == Rtn_ERROR) return Rtn_ERROR;
        if (set_WFS_Outputs() == Rtn_ERROR)    return Rtn_ERROR;
        // Seleciona MLA, define pupil e referência interna do WFS
        if (config_WFS() == Rtn_ERROR)         return Rtn_ERROR;
        SDKErrChk(RunElement(hSensorElement));

        SDKErrChk(InitControlElement(hControllerElement));
        if (set_DMP40_DM_Parameters() == Rtn_ERROR) return Rtn_ERROR;
        if (get_DMP40_DM_DataObject() == Rtn_ERROR)  return Rtn_ERROR;

        return Rtn_OK;
    }
    else return Rtn_ERROR;
}


int create_WT_LagrangeBased()
{
	hProcessor = NULL;
	hSensor = NULL;
	hWoofer = NULL;
	hTweeter = NULL;
	hProcessorExt = NULL;

	if (findPluginModules(_PluginType::Processor) == Rtn_ERROR)
		return Rtn_ERROR;

	if (numModules > 0)
	{
		if (getProcessorInstanceHandle(_PluginType::Processor, processor_ModuleHandles[0], &instanceHandle) == Rtn_ERROR)
			return Rtn_ERROR;

		hProcessor = instanceHandle;
	}
	else
		return Rtn_ERROR;

	if (findExtensionModules(_ExtensionType::ProcessorExtension) == Rtn_ERROR)
		return Rtn_ERROR;

	if (numModules > 0)
	{
		for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++)
		{
			strLength = STRING_LENGTH_BUFFER;
			SDKErrChk(GetModuleProperty(procExt_ModuleHandles[moduleIndex], _ModuleProperty::Module_Name, str, &strLength));

			if (strcmp(str, "AOKit WooferTweeter Lagrange Based Processor Extension Module") == 0)
			{
				std::cout << "Getting Processor extension handle..." << std::endl;
				if (getExtensionHandle(procExt_ModuleHandles[moduleIndex], &extensionHandle) == Rtn_ERROR)
					return Rtn_ERROR;

				hProcessorExt = extensionHandle;
			}
		}
	}
	else
		return Rtn_ERROR;

	if (findPluginModules(_PluginType::Sensor) == Rtn_ERROR)
		return Rtn_ERROR;

	if (numModules > 0)
	{
		for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++)
		{
			strLength = STRING_LENGTH_BUFFER;
			SDKErrChk(GetModuleProperty(sensor_ModuleHandles[moduleIndex], _ModuleProperty::Module_Name, str, &strLength));

			if (strcmp(str, "Thorlabs Wavefront Sensor Plugin Module") == 0)
			{
				std::cout << "Getting Wavefront Sensor plugin handle..." << std::endl;
				if (findDeviceHandles(_PluginType::Sensor, sensor_ModuleHandles[moduleIndex]) == Rtn_ERROR)
					return Rtn_ERROR;

				if (numDevices > 0)
					hSensor = deviceHandles[0];
				else
					return Rtn_ERROR;
			}
		}
	}
	else
		return Rtn_ERROR;

	if (findPluginModules(_PluginType::Controller) == Rtn_ERROR)
		return Rtn_ERROR;

	if (numModules > 0)
	{
		for (int moduleIndex = 0; moduleIndex < numModules; moduleIndex++)
		{
			strLength = STRING_LENGTH_BUFFER;
			SDKErrChk(GetModuleProperty(controller_ModuleHandles[moduleIndex], _ModuleProperty::Module_Name, str, &strLength));

			if (strcmp(str, "Thorlabs DMP40 Deformable Mirror Plugin Module") == 0)
			{
				std::cout << "Getting DMP40 plugin handle..." << std::endl;
				if (findDeviceHandles(_PluginType::Controller, controller_ModuleHandles[moduleIndex]) == Rtn_ERROR)
					return Rtn_ERROR;

				if (numDevices > 0)
					hWoofer = deviceHandles[0];
				else
					return Rtn_ERROR;
			}

			if (strcmp(str, "BMC Mini/Multi Deformable Mirror Plugin Module") == 0)
			{
				std::cout << "Getting BMC deformable mirror plugin handle..." << std::endl;
				if (findDeviceHandles(_PluginType::Controller, controller_ModuleHandles[moduleIndex]) == Rtn_ERROR)
					return Rtn_ERROR;

				if (numDevices > 0)
					hTweeter = deviceHandles[0];
				else
					return Rtn_ERROR;
			}
		}
	}
	else
		return Rtn_ERROR;

	if (hProcessor != NULL &&
		hSensor != NULL &&
		hWoofer != NULL &&
		hTweeter != NULL &&
		hProcessorExt != NULL)
	{
		std::cout << "Creating control system..." << std::endl;
		hCtrlSys = CreateControlSystem("WT_LagrangeBased_ControlSystem");
		ErrChk("CreateControlSystem");

		hRootNode = GetRootControlNode(hCtrlSys);
		ErrChk("GetRootControlNode");

		std::cout << "Creating control node..." << std::endl;
		hCtrlNode = NewControlNode(hCtrlSys, hRootNode, "root");
		ErrChk("NewControlNode");

		std::cout << "Adding Processor to the created control node..." << std::endl;
		hProcessorElement = AddControlElement(hCtrlNode, hProcessor, "Processor");
		ErrChk("AddControlElement");

		SDKErrChk(SetUserExtension(hProcessorElement, hProcessorExt));

		std::cout << "Adding Sensor to the created control node..." << std::endl;
		hSensorElement = AddControlElement(hCtrlNode, hSensor, "Sensor");
		ErrChk("AddControlElement");

		std::cout << "Adding DMP40 to the created control node..." << std::endl;
		hControllerElement = AddControlElement(hCtrlNode, hWoofer, "Woofer");
		ErrChk("AddControlElement");

		std::cout << "Adding BMC deformable mirror to the created control node..." << std::endl;
		hControllerElement1 = AddControlElement(hCtrlNode, hTweeter, "Tweeter");
		ErrChk("AddControlElement");
	}
	else
		return Rtn_ERROR;

	return Rtn_OK;
}

int create_TLD_file()
{
	char fullFilePath[STRING_LENGTH_BUFFER];
	char fullSoptFieldImageStatisticsFilePath[STRING_LENGTH_BUFFER];

	strcpy_s(fullFilePath, c_szPath);
	strcat_s(fullFilePath, "WinConsole_Example.tld");

	strcpy_s(fullSoptFieldImageStatisticsFilePath, c_szPath);
	strcat_s(fullSoptFieldImageStatisticsFilePath, "SpotFieldImageStatistics.txt");

	std::cout << "Creating Thorlabs Data File (tld)..." << std::endl;
	FileHandle hFile = CreateCtrlSysFile(_FileOpenMode::Write, fullFilePath);
	ErrChk("CreateCtrlSysFile");

	std::cout << "Appending SpotFieldImage data to the tld file..." << std::endl;
	SDKErrChk(AppendData(hFile, "SpotFieldImage", hSpotFieldImage));

	std::cout << "Appending Wavefront data to the tld file..." << std::endl;
	SDKErrChk(AppendData(hFile, "Wavefront", hWavefront));

	std::cout << "Appending SpotDeviations data to the tld file..." << std::endl;
	SDKErrChk(AppendData(hFile, "SpotDeviations", hSpotDeviations));

	std::ofstream statisticsFile;
	statisticsFile.open(fullSoptFieldImageStatisticsFilePath);

	statisticsFile << "Spot field image statistics:" << std::endl;
	statisticsFile << "Minimum Intensity: " << minIntensity << std::endl;
	statisticsFile << "Maximum Intensity: " << maxIntensity << std::endl;
	statisticsFile << "Mean Intensity: " << meanIntensity << std::endl;
	statisticsFile << "RMS of Intensity: " << RMSofIntensity << std::endl;

	statisticsFile.close();

	std::cout << "Adding Spot field image statistics to the tld file..." << std::endl;
	SDKErrChk(AddFile(hFile, fullSoptFieldImageStatisticsFilePath));

	SDKErrChk(SaveCtrlSysFile(hFile));

	SDKErrChk(CloseCtrlSysFile(hFile));

	SDKErrChk(closeControlSystem());

	std::remove(fullSoptFieldImageStatisticsFilePath);

	return Rtn_OK;
}

int unzip_TLD_file()
{
	char fullFilePath[STRING_LENGTH_BUFFER];
	char unZippedPath[STRING_LENGTH_BUFFER];

	strcpy_s(fullFilePath, c_szPath);
	strcat_s(fullFilePath, "WinConsole_Example.tld");

	strcpy_s(unZippedPath, c_szPath);
	strcat_s(unZippedPath, "UnZipped\\");

	// Unzip files from the Thorlabs data file
	std::cout << "\nUnzipping files from tld data file..." << std::endl;
	FileHandle hFile = CreateCtrlSysFile(_FileOpenMode::Read, fullFilePath);
	ErrChk("CreateCtrlSysFile");

	int numFiles = GetNumZippedFiles(hFile);
	ErrChk("GetNumZippedFiles");
	for (int i = 1; i <= numFiles; i++)
	{
		strLength = STRING_LENGTH_BUFFER;
		SDKErrChk(GetZippedFilename(hFile, i, str, &strLength));
		SDKErrChk(UnzipFile(hFile, str, unZippedPath));
	}

	SDKErrChk(CloseCtrlSysFile(hFile));

	return Rtn_OK;
}

int export_TLD_file()
{
	char fullFilePath[STRING_LENGTH_BUFFER];
	char exportedDir[STRING_LENGTH_BUFFER];
	char exportFilePath[STRING_LENGTH_BUFFER];

	strcpy_s(fullFilePath, c_szPath);
	strcat_s(fullFilePath, "WinConsole_Example.tld");

	strcpy_s(exportedDir, c_szPath);
	strcat_s(exportedDir, "Exported\\");

	// Export data from the file
	std::cout << "\nExporting data from tld data file..." << std::endl;
	FileHandle hFile = CreateCtrlSysFile(_FileOpenMode::Read, fullFilePath);
	ErrChk("CreateCtrlSysFile");

	int numFiles = GetNumDataFiles(hFile);
	ErrChk("GetNumDataFiles");
	for (int i = 1; i <= numFiles; i++)
	{
		strLength = STRING_LENGTH_BUFFER;
		SDKErrChk(GetZippedDataFileInfoString(hFile, i, _DataFilePropertyString::DataFile_Name, str, &strLength));
		strcpy_s(exportFilePath, exportedDir);

		if (strcmp(str, "SpotFieldImage.dat") == 0)
		{
			std::cout << "Exporting " << str << " to image" << std::endl;
			strcat_s(exportFilePath, "SpotFieldImage.bmp");
			SDKErrChk(ExportDataFileToImage(hFile, _ExportImageType::Image_bmp, exportFilePath, i));
		}
		else if (strcmp(str, "Wavefront.dat") == 0)
		{
			std::cout << "Exportint " << str << " to text file" << std::endl;
			strcat_s(exportFilePath, "Wavefront.txt");
			SDKErrChk(ExportDataFileToFile(hFile, _ExportFileType::File_text, exportFilePath, i));
		}
		else
		{
			std::cout << "Exporting " << str << " to binary file" << std::endl;
			strcat_s(exportFilePath, str);
			SDKErrChk(ExportDataFileToFile(hFile, _ExportFileType::File_raw, exportFilePath, i));
		}
	}

	SDKErrChk(CloseCtrlSysFile(hFile));

	return Rtn_OK;
}

int calibrate_WT_LagrangeBased()
{
	std::cout << std::endl;
	std::cout << "Calibrating Woofer-Tweeter control system..." << std::endl;

	SDKErrChk(InitControlNode(hCtrlNode));

	_ProcessingStatus processingStatus = _ProcessingStatus::OK;
	int remainingSteps = -1;
	int remainingCommandsAtCurrentStep = 0;
	std::string processingName = "";
	std::string elementsInProcessing = "";
	int processingNameStrBufferLength = 0;
	int elementsInProcessingStrBufferLength = 0;

	while (processingStatus != _ProcessingStatus::Calibration_Failed &&
		   processingStatus != _ProcessingStatus::Calibration_Finished)
	{
		strLength = STRING_LENGTH_BUFFER;
		strLength1 = STRING_LENGTH_BUFFER;
		CalibrateControlNode(hCtrlNode, &processingStatus, &remainingSteps, &remainingCommandsAtCurrentStep, str, &strLength, str1, &strLength1);

		std::cout << "Remaining steps in " << str << ": " << remainingSteps << std::endl;
	}

	std::cout << "Calibration is done!" << std::endl;

	return Rtn_OK;
}

int start_node()
{
	while (!bStopClosedLoopControl)
	{
		_ProcessingStatus processingStatus = _ProcessingStatus::OK;
		strLength = STRING_LENGTH_BUFFER;
		strLength1 = STRING_LENGTH_BUFFER;
		SDKErrChk(StartNodeLoopControl(hCtrlNode, &processingStatus, str, &strLength, str1, &strLength1));

		std::cout << std::endl;
		std::cout << "Closed-loop control is running. Press 'x' key to stop..." << std::endl;
	}

	bThreadTerminated = true;

	return Rtn_OK;
}

int run_WT_LagrangeBased_ClosedLoop()
{
	std::cout << std::endl;
	std::cout << "Starting Woofer-Tweeter Closed-Loop control. Press 'x' to stop" << std::endl;

	SDKErrChk(SetControlNodeLoopMode(hCtrlNode, _LoopControlMode::closed));

	std::thread m = std::thread(start_node);
	m.detach();

	bThreadTerminated = false;
	char c = 'r';

	while (!bStopClosedLoopControl)
	{
		if (c == 'x')
			bStopClosedLoopControl = true;
		else
			c = _getch();
	}

	while (!bThreadTerminated)
	{
		Sleep(50);
	}

	std::cout << std::endl;
	std::cout << "Closed-Loop control is stopped." << std::endl;

	return Rtn_OK;
}

int get_WFS_Info()
{
	// Get the driver version information
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_RevisionQuery"));

	strLength = STRING_LENGTH_BUFFER;
	SDKErrChk(GetStringParameter(hSensorElement, "InstrumentDriverRevision", str, &strLength));
	std::cout << "WFS instrument driver version: " << str << std::endl;

	strLength = STRING_LENGTH_BUFFER;
	SDKErrChk(GetStringParameter(hSensorElement, "CamDriverRevision", str, &strLength));
	std::cout << "WFS camera driver version: " << str << std::endl;

	// Get the device information
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_GetInstrumentInfo"));
	strLength = STRING_LENGTH_BUFFER;
	SDKErrChk(GetStringParameter(hSensorElement, "ManufacturerName", str, &strLength));
	std::cout << "The manufacturer is: " << str << std::endl;

	strLength = STRING_LENGTH_BUFFER;
	SDKErrChk(GetStringParameter(hSensorElement, "InstrumentNameWFS", str, &strLength));
	std::cout << "The device name is: " << str << std::endl;

	strLength = STRING_LENGTH_BUFFER;
	SDKErrChk(GetStringParameter(hSensorElement, "SerialNumberWFS", str, &strLength));
	std::cout << "The device SN is: " << str << std::endl;

	strLength = STRING_LENGTH_BUFFER;
	SDKErrChk(GetStringParameter(hSensorElement, "SerialNumberCam", str, &strLength));
	std::cout << "The camera SN is: " << str << std::endl;

	// Initialize WFS
	// SDKErrChk(InitControlElement(hSensorElement));

	// Get MLA information from the WFS
	strLength = STRING_LENGTH_BUFFER;
	SDKErrChk(GetEnumParameterElements(hSensorElement, "MLA", str, &strLength));

	// If it contains multiple MLAs, ';' is used to seperate them.
	char *pch;
	pch = strchr(str, ';');
	int mla_cnt = 0;
	while (pch != NULL)
	{
		mla_cnt++;
		pch = strchr(pch + 1, ';');
	}

	double dpValue = 0.0;
	for (int i = 0; i < mla_cnt; i++)
	{
		SDKErrChk(SetNumericParameter(hSensorElement, "MLAIndexForDataQuery", &i));

		SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_GetMlaData"));
		strLength = STRING_LENGTH_BUFFER;
		SDKErrChk(GetStringParameter(hSensorElement, "MLAName", str, &strLength));
		std::cout << "MLA name:" << str << std::endl;

		SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "CamPitchm", &dpValue));
		std::cout << "CamPitchm = " << dpValue << std::endl;

		SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "LensletPitchm", &dpValue));
		std::cout << "LensletPitchm = " << dpValue << std::endl;

		SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "SpotOffsetX", &dpValue));
		std::cout << "SpotOffsetX = " << dpValue << std::endl;

		SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "SpotOffsetY", &dpValue));
		std::cout << "SpotOffsetY = " << dpValue << std::endl;

		SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "LensletFocalLength", &dpValue));
		std::cout << "LensletFocalLength = " << dpValue << std::endl;

		SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "GrdCorr0", &dpValue));
		std::cout << "GrdCorr0 = " << dpValue << std::endl;

		SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "GrdCorr45", &dpValue));
		std::cout << "GrdCorr45 = " << dpValue << std::endl;
	}

	return Rtn_OK;
}

int config_WFS()
{
	// Select MLA
	int iValue = 0;
	SDKErrChk(SetNumericParameter(hSensorElement, "MLAIndex", &iValue));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SelectMla"));

    // Configure the camera resolution of the WFS at 2 (align with set_WFS_Parameters)
    iValue = 2;
    SDKErrChk(SetNumericParameter(hSensorElement, "CamResolIndex", &iValue));
	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "SpotsX", &iValue));
	std::cout << "Spot size in X is: " << iValue << std::endl;

	SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "SpotsY", &iValue));
	std::cout << "Spot size in Y is: " << iValue << std::endl;

	// Set WFS internal reference plane
	iValue = 0;
	SDKErrChk(SetNumericParameter(hSensorElement, "ReferenceIndexSel", &iValue));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetReferencePlane"));

	// Define pupil
	double dpValue = 0.0;
	SDKErrChk(SetNumericParameter(hSensorElement, "PupilCenterX", &dpValue));
	SDKErrChk(SetNumericParameter(hSensorElement, "PupilCenterY", &dpValue));

    dpValue = 3.0;
    SDKErrChk(SetNumericParameter(hSensorElement, "PupilDiameterX", &dpValue));
    SDKErrChk(SetNumericParameter(hSensorElement, "PupilDiameterY", &dpValue));

	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_SetPupil"));

	return Rtn_OK;
}

int read_WFS_Image()
{
	int iValue = 0;
	double dpValue = 0.0;

	std::cout << std::endl;
	std::cout << "Reading Wavefront Sensor spot field image..." << std::endl;

	// Read camera image 10 times
	for (int i = 0; i < 10; i++)
	{
		SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_TakeSpotfieldImageAutoExpos"));

		SDKErrChk(SetOutputByName(hSensorElement, "DeviceStatus"));
		SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_GetStatus"));
		SDKErrChk(CatchElementOutput(hSensorElement));

		SDKErrChk(GetNumericOutput(hSensorElement, "DeviceStatus", &iValue));
		if (iValue & 2)
			std::cout << "Power too high!" << std::endl;
		else if (iValue == 4)
			std::cout << "Power too low!" << std::endl;
		else if (iValue == 8)
			std::cout << "High ambient light!" << std::endl;
		else
		{
			SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "ExposureTimeAct", &dpValue));
			std::cout << "The actual exposure time is: " << dpValue << std::endl;

			SDKErrChk(GetNumericParameter(hSensorElement, _NumericParameterProperty::Value, "MasterGainAct", &dpValue));
			std::cout << "The actual master gain is: " << dpValue << std::endl;
		}
	}

	if (iValue == 2 ||
		iValue == 4 ||
		iValue == 8)
	{
		std::cout << "Sample program will be closed because of unusable image quality." << std::endl;
		return Rtn_ERROR;
	}

	// Get spot field image
	SDKErrChk(SetOutputByName(hSensorElement, "ArrayImageBuf"));
	SDKErrChk(SetOutputByName(hSensorElement, "ImageRows"));
	SDKErrChk(SetOutputByName(hSensorElement, "ImageColumns"));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_GetSpotfieldImage"));

	SDKErrChk(CatchElementOutput(hSensorElement));

	SDKErrChk(GetNumericOutput(hSensorElement, "ImageRows", &iValue));
	std::cout << "Image rows is " << iValue << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "ImageColumns", &iValue));
	std::cout << "Image columns is " << iValue << std::endl;

	return Rtn_OK;
}

int calculate_Spots_CentrDiaIntens()
{
	std::cout << std::endl;
	std::cout << "Calculating all spots centroids, diameters and intensities..." << std::endl;

	// Calculate all spot centroid positions using dynamic noise cut option
	int iValue = 1;
	SDKErrChk(SetNumericParameter(hSensorElement, "DynamicNoiseCut", &iValue));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotsCentrDiaIntens"));

	SDKErrChk(CatchElementOutput(hSensorElement));

	// Get centroid result arrays
	SDKErrChk(SetOutputByName(hSensorElement, "ArraySpotCentroid"));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_GetSpotCentroids"));

	DataHandle hArraySpotCentroid = GetDataObjectHandle(hSensorElement, "ArraySpotCentroid");
	ErrChk("GetDataObjectHandle");

	int iSize1 = GetDataPropertyInt(hArraySpotCentroid, _DataPropertyInt::Data_Size1);
	ErrChk("GetDataPropertyInt");
	iSize1 = (iSize1 == 0) ? 1 : iSize1;

	int iSize2 = GetDataPropertyInt(hArraySpotCentroid, _DataPropertyInt::Data_Size2);
	ErrChk("GetDataPropertyInt");
	iSize2 = (iSize2 == 0) ? 1 : iSize2;

	int iSize3 = GetDataPropertyInt(hArraySpotCentroid, _DataPropertyInt::Data_Size3);
	ErrChk("GetDataPropertyInt");
	iSize3 = (iSize3 == 0) ? 1 : iSize3;

	int iSize4 = GetDataPropertyInt(hArraySpotCentroid, _DataPropertyInt::Data_Size4);
	ErrChk("GetDataPropertyInt");
	iSize4 = (iSize4 == 0) ? 1 : iSize4;

	int iBytesPerComponent = GetDataPropertyInt(hArraySpotCentroid, _DataPropertyInt::Data_BytesPerComponent);
	ErrChk("GetDataPropertyInt");

	int iComponentPerData = GetDataPropertyInt(hArraySpotCentroid, _DataPropertyInt::Data_ComponentsPerData);
	ErrChk("GetDataPropertyInt");

	int iDataType = GetDataPropertyInt(hArraySpotCentroid, _DataPropertyInt::Data_Type);
	ErrChk("GetDataPropertyInt");

	void *byteArray = new byte[iSize1 * iSize2 * iSize3 * iSize4 * iComponentPerData * iBytesPerComponent];
	SDKErrChk(CopyDataContent(hArraySpotCentroid, byteArray));

	std::cout << std::fixed;
	std::cout << std::setprecision(3);
	std::cout << "Centroid X position in pixels (first 5x5 elements)" << std::endl;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			if (iDataType == _DataType::Float)
				std::cout << ((float *)byteArray)[i * iSize1 * iComponentPerData + j * iComponentPerData] << "   ";
			else if (iDataType == _DataType::Double)
				std::cout << ((double *)byteArray)[i * iSize1 * iComponentPerData + j * iComponentPerData] << "   ";
		}

		std::cout << std::endl;
	}

	std::cout << "Centroid Y Positions in pixels (first 5x5 elements)" << std::endl;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			if (iDataType == _DataType::Float)
				std::cout << ((float *)byteArray)[i * iSize1 * iComponentPerData + (j * iComponentPerData + 1)] << "   ";
			else if (iDataType == _DataType::Double)
				std::cout << ((double *)byteArray)[i * iSize1 * iComponentPerData + (j * iComponentPerData + 1)] << "   ";
		}

		std::cout << std::endl;
	}

	delete byteArray;

	std::cout << "Press <ENTER> to proceed..." << std::endl;
	getchar();

	return Rtn_OK;
}

int calculate_beam_centrDia()
{
	std::cout << std::endl;
	std::cout << "Calculating beam centroids and diameters..." << std::endl;

	double dpValue;

	// Get centroid and diameter of the optical beam
	SDKErrChk(SetOutputByName(hSensorElement, "BeamCentroidX"));
	SDKErrChk(SetOutputByName(hSensorElement, "BeamCentroidY"));
	SDKErrChk(SetOutputByName(hSensorElement, "BeamDiameterX"));
	SDKErrChk(SetOutputByName(hSensorElement, "BeamDiameterY"));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcBeamCentroidDia"));

	SDKErrChk(CatchElementOutput(hSensorElement));

	SDKErrChk(GetNumericOutput(hSensorElement, "BeamCentroidX", &dpValue));
	std::cout << "Beam centroid in X is: " << dpValue << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "BeamCentroidY", &dpValue));
	std::cout << "Beam centroid in Y is: " << dpValue << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "BeamDiameterX", &dpValue));
	std::cout << "Beam diameter in X is: " << dpValue << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "BeamDiameterY", &dpValue));
	std::cout << "Beam diameter in Y is: " << dpValue << std::endl;

	std::cout << "Press <ENTER> to proceed..." << std::endl;
	getchar();

	return Rtn_OK;
}

int calculate_Spot_Deviations()
{
	std::cout << std::endl;
	std::cout << "Calculating spot deviations..." << std::endl;

	// Calculate spot deviations to internal reference
	int iValue = 1;
	SDKErrChk(SetNumericParameter(hSensorElement, "CancelWavefrontTilt", &iValue));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcSpotToReferenceDeviations"));

	// Get spot deviations
	SDKErrChk(SetOutputByName(hSensorElement, "ArraySpotDeviations"));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_GetSpotDeviations"));

	DataHandle hArraySpotDeviations = GetDataObjectHandle(hSensorElement, "ArraySpotDeviations");
	ErrChk("GetDataObjectHandle");

	int iSize1 = GetDataPropertyInt(hArraySpotDeviations, _DataPropertyInt::Data_Size1);
	ErrChk("GetDataPropertyInt");
	iSize1 = (iSize1 == 0) ? 1 : iSize1;

	int iSize2 = GetDataPropertyInt(hArraySpotDeviations, _DataPropertyInt::Data_Size2);
	ErrChk("GetDataPropertyInt");
	iSize2 = (iSize2 == 0) ? 1 : iSize2;

	int iSize3 = GetDataPropertyInt(hArraySpotDeviations, _DataPropertyInt::Data_Size3);
	ErrChk("GetDataPropertyInt");
	iSize3 = (iSize3 == 0) ? 1 : iSize3;

	int iSize4 = GetDataPropertyInt(hArraySpotDeviations, _DataPropertyInt::Data_Size4);
	ErrChk("GetDataPropertyInt");
	iSize4 = (iSize4 == 0) ? 1 : iSize4;

	int iComponentPerData = GetDataPropertyInt(hArraySpotDeviations, _DataPropertyInt::Data_ComponentsPerData);
	ErrChk("GetDataPropertyInt");

	int iBytesPerComponent = GetDataPropertyInt(hArraySpotDeviations, _DataPropertyInt::Data_BytesPerComponent);
	ErrChk("GetDataPropertyInt");

	// User needs to get the data type from the data object.
	int iDataType = GetDataPropertyInt(hArraySpotDeviations, _DataPropertyInt::Data_Type);
	ErrChk("GetDataPropertyInt");

	void *byteArray = new byte[iSize1 * iSize2 * iSize3 * iSize4 * iComponentPerData * iBytesPerComponent];
	SDKErrChk(CopyDataContent(hArraySpotDeviations, byteArray));

	std::cout << std::fixed;
	std::cout << std::setprecision(3);
	std::cout << "Deviation X position in pixels (first 5x5 elements)" << std::endl;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			if (iDataType == _DataType::Float)
				std::cout << ((float *)byteArray)[i * iSize1 * iComponentPerData + j * iComponentPerData] << "   ";
			else if (iDataType == _DataType::Double)
				std::cout << ((double *)byteArray)[i * iSize1 * iComponentPerData + j * iComponentPerData] << "   ";
		}

		std::cout << std::endl;
	}

	std::cout << "Deviation Y Positions in pixels (first 5x5 elements)" << std::endl;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			if (iDataType == _DataType::Float)
				std::cout << ((float *)byteArray)[i * iSize1 * iComponentPerData + j * iComponentPerData] << "   ";
			else if (iDataType == _DataType::Double)
				std::cout << ((double *)byteArray)[i * iSize1 * iComponentPerData + j * iComponentPerData] << "   ";
		}

		std::cout << std::endl;
	}

	std::cout << "Press <ENTER> to proceed..." << std::endl;
	getchar();

	delete byteArray;

	return Rtn_OK;
}

int calculate_Wavefront()
{
	std::cout << std::endl;
	std::cout << "Calculating wavefront and wavefront statistics..." << std::endl;

	// Calculate measured wavefront
	int iValue = 0;
	SDKErrChk(SetNumericParameter(hSensorElement, "WavefrontTypeSel", &iValue));
	SDKErrChk(SetNumericParameter(hSensorElement, "LimitToPupil", &iValue));
	SDKErrChk(SetOutputByName(hSensorElement, "ArrayWavefront"));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefront"));

	SDKErrChk(CatchElementOutput(hSensorElement));

	DataHandle hArrayWavefront = GetDataObjectHandle(hSensorElement, "ArrayWavefront");
	ErrChk("GetDataObjectHandle");

	int iSize1 = GetDataPropertyInt(hArrayWavefront, _DataPropertyInt::Data_Size1);
	ErrChk("GetDataPropertyInt");
	iSize1 = (iSize1 == 0) ? 1 : iSize1;

	int iSize2 = GetDataPropertyInt(hArrayWavefront, _DataPropertyInt::Data_Size2);
	ErrChk("GetDataPropertyInt");
	iSize2 = (iSize2 == 0) ? 1 : iSize2;

	int iSize3 = GetDataPropertyInt(hArrayWavefront, _DataPropertyInt::Data_Size3);
	ErrChk("GetDataPropertyInt");
	iSize3 = (iSize3 == 0) ? 1 : iSize3;

	int iSize4 = GetDataPropertyInt(hArrayWavefront, _DataPropertyInt::Data_Size4);
	ErrChk("GetDataPropertyInt");
	iSize4 = (iSize4 == 0) ? 1 : iSize4;

	int iComponentPerData = GetDataPropertyInt(hArrayWavefront, _DataPropertyInt::Data_ComponentsPerData);
	ErrChk("GetDataPropertyInt");

	int iBytesPerComponent = GetDataPropertyInt(hArrayWavefront, _DataPropertyInt::Data_BytesPerComponent);
	ErrChk("GetDataPropertyInt");

	// User needs to get the data type from the data object.
	int iDataType = GetDataPropertyInt(hArrayWavefront, _DataPropertyInt::Data_Type);
	ErrChk("GetDataPropertyInt");

	void *byteArray = new byte[iSize1 * iSize2 * iSize3 * iSize4 * iComponentPerData * iBytesPerComponent];
	SDKErrChk(CopyDataContent(hArrayWavefront, byteArray));

	std::cout << std::fixed;
	std::cout << std::setprecision(3);
	std::cout << "Wavefront in microns (first 5x5 elements)" << std::endl;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			if (iDataType == _DataType::Float)
				std::cout << ((float *)byteArray)[i * iSize1 * iComponentPerData + j * iComponentPerData] << "   ";
			else if (iDataType == _DataType::Double)
				std::cout << ((double *)byteArray)[i * iSize1 * iComponentPerData + j * iComponentPerData] << "   ";
		}

		std::cout << std::endl;
	}

	// Calculate wavefront statistics within defined pupil
	SDKErrChk(SetOutputByName(hSensorElement, "WavefrontMin"));
	SDKErrChk(SetOutputByName(hSensorElement, "WavefrontMax"));
	SDKErrChk(SetOutputByName(hSensorElement, "WavefrontDiff"));
	SDKErrChk(SetOutputByName(hSensorElement, "WavefrontMean"));
	SDKErrChk(SetOutputByName(hSensorElement, "WavefrontRMS"));
	SDKErrChk(SetOutputByName(hSensorElement, "WavefrontWeightedRMS"));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_CalcWavefrontStatistics"));

	SDKErrChk(CatchElementOutput(hSensorElement));

	double dpValue;
	SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontMin", &dpValue));
	std::cout << "Wavefront minimum value is " << dpValue << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontMax", &dpValue));
	std::cout << "Wavefront maximum value is " << dpValue << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontDiff", &dpValue));
	std::cout << "Difference between Maximum wavefront and Minimum wavefront is " << dpValue << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontMean", &dpValue));
	std::cout << "Wavefront mean value is " << dpValue << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontRMS", &dpValue));
	std::cout << "RMS value of Wavefront is " << dpValue << std::endl;

	SDKErrChk(GetNumericOutput(hSensorElement, "WavefrontWeightedRMS", &dpValue));
	std::cout << "Weighted RMS value of Wavefront is " << dpValue << std::endl;

	std::cout << "Press <ENTER> to proceed..." << std::endl;
	getchar();

	delete byteArray;

	return Rtn_OK;
}

int calculate_Zernike()
{
	std::cout << std::endl;
	std::cout << "Calculating Zernike coefficients. Zernike fit up to order 3, Zernike modes is 10." << std::endl;

	int iValue = 3;

	SDKErrChk(SetNumericParameter(hSensorElement, "ZernikeOrderSel", &iValue));
	SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficients"));
	SDKErrChk(SetOutputByName(hSensorElement, "ArrayZernikeCoefficientsRMS"));
	SDKErrChk(ExecuteApiFunction(hSensorElement, "WFS_ZernikeLsf"));

	SDKErrChk(CatchElementOutput(hSensorElement));

	DataHandle hArrayZernikeCoefficients = GetDataObjectHandle(hSensorElement, "ArrayZernikeCoefficients");
	ErrChk("GetDataObjectHandle");

	int iSize1 = GetDataPropertyInt(hArrayZernikeCoefficients, _DataPropertyInt::Data_Size1);
	ErrChk("GetDataPropertyInt");
	iSize1 = (iSize1 == 0) ? 1 : iSize1;

	int iSize2 = GetDataPropertyInt(hArrayZernikeCoefficients, _DataPropertyInt::Data_Size2);
	ErrChk("GetDataPropertyInt");
	iSize2 = (iSize2 == 0) ? 1 : iSize2;

	int iSize3 = GetDataPropertyInt(hArrayZernikeCoefficients, _DataPropertyInt::Data_Size3);
	ErrChk("GetDataPropertyInt");
	iSize3 = (iSize3 == 0) ? 1 : iSize3;

	int iSize4 = GetDataPropertyInt(hArrayZernikeCoefficients, _DataPropertyInt::Data_Size4);
	ErrChk("GetDataPropertyInt");
	iSize4 = (iSize4 == 0) ? 1 : iSize4;

	int iComponentPerData = GetDataPropertyInt(hArrayZernikeCoefficients, _DataPropertyInt::Data_ComponentsPerData);
	ErrChk("GetDataPropertyInt");

	int iBytesPerComponent = GetDataPropertyInt(hArrayZernikeCoefficients, _DataPropertyInt::Data_BytesPerComponent);
	ErrChk("GetDataPropertyInt");

	// User needs to get the data type from the data object.
	int iDataType = GetDataPropertyInt(hArrayZernikeCoefficients, _DataPropertyInt::Data_Type);
	ErrChk("GetDataPropertyInt");

	void *byteArray = new byte[iSize1 * iSize2 * iSize3 * iSize4 * iComponentPerData * iBytesPerComponent];
	SDKErrChk(CopyDataContent(hArrayZernikeCoefficients, byteArray));

	std::cout << std::fixed;
	std::cout << std::setprecision(3);

	for (int i = 0; i < 10; i++)
	{
		if (iDataType == _DataType::Float)
			std::cout << "Zernike coefficient " << i << ": " << ((float *)byteArray)[i] << std::endl;

		else if (iDataType == _DataType::Double)
			std::cout << "Zernike coefficient " << i << ": " << ((double *)byteArray)[i] << std::endl;
	}

	delete byteArray;

	return Rtn_OK;
}

int apply_line_pattern_to_DMP40()
{
	const int numZernike = iZernikeCoefficientCount;
double amplitude = 1.0;  // ou outro valor que gere deformação visível
double* zernikeCoefficients = new double[iZernikeCoefficientCount];
for (int mode = 0; mode < numZernike; mode++)
{
	// Zera todos os coeficientes
	for (int i = 0; i < numZernike; i++)
		zernikeCoefficients[i] = 0.0;

	// Ativa apenas o modo atual com uma amplitude definida
	zernikeCoefficients[mode] = amplitude;

	// Envia os coeficientes para o espelho
	SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, zernikeCoefficients));

	std::cout << "Aplicando modo Zernike " << (mode + 1) << " com amplitude " << amplitude << std::endl;

	SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));
	SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));

	std::cout << "Pressione Enter para continuar para o próximo modo...\n";
	getchar();  // espera o usuário ver o efeito no sistema óptico
}
delete[] zernikeCoefficients;
}

// ---------- Helpers e Aplicadores de padrões Zernike no DMP40 ----------



// --- máscara: liga apenas os modos informados (1-based) ---
int apply_mask_to_DMP40(const std::vector<int>& modesOn, double amplitude)
{
    

    std::vector<double> zernike(iZernikeCoefficientCount, 0.0);

    for (int m : modesOn) {
        int idx = m - 1;
        if (idx >= 0 && idx < iZernikeCoefficientCount) {
            zernike[idx] = amplitude;
        } else {
            std::cerr << "[Aviso] Modo " << m << " fora de [1.." << iZernikeCoefficientCount << "]\n";
        }
    }

    SDKErrChk(LockElement(hControllerElement));
    SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, zernike.data()));
    SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));
    SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));
    SDKErrChk(UnlockElement(hControllerElement));
    return Rtn_OK;
}

// --- varredura em linha: ativa sequencialmente first..last (1-based) ---
int apply_line_pattern_to_DMP40_range(int firstMode, int lastMode, double amplitude, int dwell_ms)
{
    

    const int numZ = iZernikeCoefficientCount;
    if (firstMode < 1) firstMode = 1;
    if (lastMode  > numZ) lastMode = numZ;
    if (firstMode > lastMode) std::swap(firstMode, lastMode);

    std::vector<double> zernike(numZ, 0.0);

    SDKErrChk(LockElement(hControllerElement));
    for (int m = firstMode; m <= lastMode; ++m) {
        std::fill(zernike.begin(), zernike.end(), 0.0);
        zernike[m - 1] = amplitude;

        SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, zernike.data()));
        SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));
        SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));

        std::cout << "Aplicado modo Zernike " << m << " (amp=" << amplitude << ")\n";
        if (dwell_ms > 0) Sleep(dwell_ms);
    }
    SDKErrChk(UnlockElement(hControllerElement));
    return Rtn_OK;
}

// --- padrão senoidal: distribui seno entre first..last (1-based) ---
int apply_sinusoidal_mask_to_DMP40(int firstMode, int lastMode, double amplitude_peak, double cycles)
{
   
    const int numZ = iZernikeCoefficientCount;
    if (firstMode < 1) firstMode = 1;
    if (lastMode  > numZ) lastMode = numZ;
    if (firstMode > lastMode) std::swap(firstMode, lastMode);

    std::vector<double> zernike(numZ, 0.0);
    const int count = lastMode - firstMode + 1;

    // evitar conflito com macro max do Windows:
    int denom = (count > 1 ? count - 1 : 1);
    for (int i = 0; i < count; ++i) {
        double phase = (2.0 * M_PI * cycles) * (static_cast<double>(i) / static_cast<double>(denom));
        zernike[(firstMode - 1) + i] = amplitude_peak * std::sin(phase);
    }
	

    SDKErrChk(LockElement(hControllerElement));
    SDKErrChk(SetDataContent(hSelectedZernikeTermAmplitudes, zernike.data()));
    SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFMX_calculate_zernike_pattern"));
    SDKErrChk(ExecuteApiFunction(hControllerElement, "TLDFM_set_segment_voltages"));
    SDKErrChk(UnlockElement(hControllerElement));
    return Rtn_OK;
}


#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

// ---------- Noll index j(1..K) -> (n,m) ----------
static void noll_to_nm(int j, int& n, int& m) {
    // Noll (1976): j>=1
    int j1 = j - 1;
    n = 0;
    while (j1 > n) { j1 -= (n + 1); n++; }
    int k = j1;
    int s = (n & 1) ? 1 : 0;         // paridade de n
    m = 2 * k - n;                   // |m| tem a mesma paridade de n
    if (s) m = -m;                   // alterna sinal conforme Noll
}

// ---------- fatorial simples p/ coeficientes radiais ----------
static inline double fact(int x){ double f=1.0; for(int i=2;i<=x;i++) f*=i; return f; }

double R_nm(int n, int m, double r) {
    m = std::abs(m);
    double sum = 0.0;
    for (int s = 0; s <= (n - m) / 2; ++s) {
        double c = (s % 2) ? -1.0 : 1.0;
        double num = fact(n - s);
        double den = fact(s) * fact((n + m)/2 - s) * fact((n - m)/2 - s);
        sum += c * (num / den) * std::pow(r, n - 2*s);
    }
    return sum;
}

double Z_nm(int n, int m, double r, double theta) {
    if (r > 1.0) return 0.0;
    double R = R_nm(n, m, r);
    if (m > 0)  return std::sqrt(2.0) * R * std::cos(m * theta);
    if (m < 0)  return std::sqrt(2.0) * R * std::sin(-m * theta);
    return R; // m == 0
}

double zernikeRadial(int n, int m, double r) {
    return R_nm(n, m, r);
}

double zernikePolynomial(int n, int m, double r, double theta) {
    return Z_nm(n, m, r, theta);
}


// ---------- converte pixel (x,y) -> coordenadas polares no disco unitário ----------
static inline void to_polar(double x, double y, double& r, double& th) {
    r = std::sqrt(x*x + y*y);
    th = std::atan2(y, x);
}

// ---------- constrói máscara de pupila circular centrada ----------
static void build_unit_disk_mask(int rows, int cols, std::vector<uint8_t>& mask) {
    mask.assign(rows*cols, 0);
    const double cx = (cols-1)*0.5, cy = (rows-1)*0.5;
    const double Rn = std::min(cx, cy);
    for (int iy=0; iy<rows; ++iy) {
        for (int ix=0; ix<cols; ++ix) {
            double x = (ix - cx)/Rn, y = (iy - cy)/Rn;
            if (x*x + y*y <= 1.0) mask[iy*cols + ix] = 1;
        }
    }
}

// ---------- decompõe fase em K Zernike por mínimos‑quadrados ----------
static bool decompose_phase_firstK(const std::vector<double>& phase, int rows, int cols,
                                   int K, std::vector<double>& coeffs)
{
    // monta A^T A e A^T b sem guardar A: acumula on-the-fly
    std::vector<double> ATA(K*K, 0.0);
    std::vector<double> ATb(K, 0.0);

    // máscara de pupila
    std::vector<uint8_t> mask; build_unit_disk_mask(rows, cols, mask);

    const double cx = (cols-1)*0.5, cy = (rows-1)*0.5;
    const double Rn = std::min(cx, cy);

    // varre pontos da pupila
    std::vector<double> z(K);
    int P = 0;
    for (int iy=0; iy<rows; ++iy) {
        for (int ix=0; ix<cols; ++ix) {
            if (!mask[iy*cols + ix]) continue;

            // coords normalizadas no disco
            double x = (ix - cx)/Rn, y = (iy - cy)/Rn;
            double r, th; to_polar(x,y,r,th);

            // z_j = Zernike_j(x,y), j=1..K
            for (int j=1; j<=K; ++j) {
                int n,m; noll_to_nm(j,n,m);
                z[j-1] = Z_nm(n,m,r,th);
            }

            double b = phase[iy*cols + ix];

            // acumula
            for (int a=0; a<K; ++a) {
                ATb[a] += z[a]*b;
                for (int b2=0; b2<K; ++b2)
                    ATA[a*K + b2] += z[a]*z[b2];
            }
            ++P;
        }
    }
    if (P == 0) return false;

    // resolve (ATA) c = ATb (eliminação gaussiana com pivoteamento parcial simples)
    coeffs = ATb;
    // forward
    for (int k=0; k<K; ++k) {
        // pivô
        int piv = k;
        double best = std::fabs(ATA[k*K + k]);
        for (int i=k+1;i<K;i++){
            double v = std::fabs(ATA[i*K + k]);
            if (v > best) { best = v; piv = i; }
        }
        if (best < 1e-14) return false;
        if (piv != k) {
            for (int j=k;j<K;j++) std::swap(ATA[k*K + j], ATA[piv*K + j]);
            std::swap(coeffs[k], coeffs[piv]);
        }
        double diag = ATA[k*K + k];
        for (int j=k;j<K;j++) ATA[k*K + j] /= diag;
        coeffs[k] /= diag;

        for (int i=k+1;i<K;i++){
            double f = ATA[i*K + k];
            for (int j=k;j<K;j++) ATA[i*K + j] -= f * ATA[k*K + j];
            coeffs[i] -= f * coeffs[k];
        }
    }
    // back
    for (int i=K-1;i>=0;i--){
        for (int j=i+1;j<K;j++) coeffs[i] -= ATA[i*K + j]*coeffs[j];
    }
    return true;
}
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// ============================================================
// 1) Tela de fase Kolmogorov por soma de ondas planas
//    - Grid NxN em coordenadas normalizadas à pupila circular de diâmetro D=1
//    - numCycles define f_max (em ciclos/diâmetro); f_min ~ 1 ciclo/diâmetro
//    - r0_relD = r0/D (quanto menor, mais forte a turbulência)
//    - numModes: nº de ondas planas somadas (200–2000 dá resultados bons)
//    Saída: W[y][x] em RADIANOS
// ============================================================
std::vector<std::vector<double>> generate_kolmogorov_phase_plane_waves(
    int N, double numCycles, double r0_relD, int numModes)
{
    std::vector<std::vector<double>> W(N, std::vector<double>(N, 0.0));
    if (N <= 0 || numCycles <= 0.0 || r0_relD <= 0.0 || numModes <= 0) return W;

    // RNG
    std::mt19937_64 rng{std::random_device{}()};
    std::uniform_real_distribution<double> U(0.0, 1.0);
    std::uniform_real_distribution<double> Uang(0.0, 2.0*M_PI);

    // Faixa espectral (em ciclos / diâmetro)
    const double fmin = 1.0;
    const double fmax = std::max(numCycles, fmin + 1.0);
    const double expo = 2.0; // controla densidade de amostragem, >1 concentra em baixas f

    struct PW { double fx, fy, A, phi0; };
    std::vector<PW> modes; modes.reserve(numModes);

    // Monta modos de ondas planas com espectro ~ |f|^{-11/6}
    for (int k = 0; k < numModes; ++k) {
        double u  = U(rng);
        double f  = std::pow(u, expo) * (fmax - fmin) + fmin; // enviesado p/ baixas f
        double th = Uang(rng);
        double fx = f * std::cos(th);
        double fy = f * std::sin(th);
        double A  = std::pow(f, -11.0/6.0);                   // Kolmogorov
        double ph = Uang(rng);
        modes.push_back({fx, fy, A, ph});
    }

    // Síntese no grid
    const double c  = 0.5 * (N - 1);
    const double Rn = c; // raio em pixels
    for (int iy = 0; iy < N; ++iy) {
        for (int ix = 0; ix < N; ++ix) {
            // Coordenadas normalizadas ao DIÂMETRO (D=1) => X,Y ∈ [-0.5,0.5]
            double X = (ix - c) / (2.0*Rn);
            double Y = (iy - c) / (2.0*Rn);
            if (X*X + Y*Y > 0.25) continue; // fora da pupila (raio 0.5)

            double phi = 0.0;
            for (const auto& m : modes) {
                double arg = 2.0*M_PI * (m.fx * X + m.fy * Y) + m.phi0;
                phi += m.A * std::cos(arg);
            }
            W[iy][ix] = phi; // radianos (escala relativa)
        }
    }

    // Normaliza variância ~ (D/r0)^{5/3} = (1/r0_relD)^{5/3}
    double mean = 0.0, var = 0.0; int P = 0;
    for (int iy = 0; iy < N; ++iy)
        for (int ix = 0; ix < N; ++ix)
            if (( (ix - c)/(2.0*Rn) ) * ( (ix - c)/(2.0*Rn) )
              + ( (iy - c)/(2.0*Rn) ) * ( (iy - c)/(2.0*Rn) ) <= 0.25)
            { mean += W[iy][ix]; ++P; }
    if (P > 0) mean /= P;

    for (int iy = 0; iy < N; ++iy)
        for (int ix = 0; ix < N; ++ix) {
            double X = (ix - c) / (2.0*Rn);
            double Y = (iy - c) / (2.0*Rn);
            if (X*X + Y*Y > 0.25) continue;
            double d = W[iy][ix] - mean;
            var += d*d;
        }
    if (P > 1) var /= (P - 1);

    const double target_var = std::pow(1.0 / r0_relD, 5.0/3.0);
    if (var > 1e-12) {
        double g = std::sqrt(target_var / var);
        for (int iy = 0; iy < N; ++iy)
            for (int ix = 0; ix < N; ++ix)
                W[iy][ix] = g * (W[iy][ix] - mean);
    }

    return W; // fase em radianos
}

// ============================================================
// 2) Decomposição da tela (rad) nos K primeiros Zernike (Noll 1..K)
//     - Usa sua zernikePolynomial(n,m,r,theta)
//     - Resolve mínimos-quadrados via (A^T A) c = A^T b
// ============================================================
bool decompose_phase_firstK_using_zernikePolynomial(
    const std::vector<std::vector<double>>& phase_rad,
    int K,
    std::vector<double>& coeffs_rad)
{
    const int N = (int)phase_rad.size();
    if (N == 0 || (int)phase_rad[0].size() != N || K <= 0) return false;

    std::vector<double> ATA(K*K, 0.0);
    std::vector<double> ATb(K,    0.0);
    std::vector<double> z(K, 0.0);

    const double c  = 0.5 * (N - 1);
    const double Rn = c;

    int P = 0;
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            double X = (x - c)/Rn, Y = (y - c)/Rn;
            double r = std::sqrt(X*X + Y*Y);
            if (r > 1.0) continue; // só pupila
            double th = std::atan2(Y, X);

            // base Zernike com sua função já existente
            for (int j = 1; j <= K; ++j) {
                int n, m; noll_to_nm(j, n, m);
                z[j-1] = zernikePolynomial(n, m, r, th); // <<-- SUA FUNÇÃO
            }

            double b = phase_rad[y][x]; // rad
            for (int a = 0; a < K; ++a) {
                ATb[a] += z[a] * b;
                for (int b2 = 0; b2 < K; ++b2)
                    ATA[a*K + b2] += z[a] * z[b2];
            }
            ++P;
        }
    }
    if (P == 0) return false;

    // Resolve (ATA) c = ATb (eliminação gaussiana com pivoteamento parcial)
    coeffs_rad = ATb;
    for (int k = 0; k < K; ++k) {
        int piv = k;
        double best = std::fabs(ATA[k*K + k]);
        for (int i = k + 1; i < K; ++i) {
            double v = std::fabs(ATA[i*K + k]);
            if (v > best) { best = v; piv = i; }
        }
        if (best < 1e-14) return false;

        if (piv != k) {
            for (int j = k; j < K; ++j) std::swap(ATA[k*K + j], ATA[piv*K + j]);
            std::swap(coeffs_rad[k], coeffs_rad[piv]);
        }
        double diag = ATA[k*K + k];
        for (int j = k; j < K; ++j) ATA[k*K + j] /= diag;
        coeffs_rad[k] /= diag;

        for (int i = k + 1; i < K; ++i) {
            double f = ATA[i*K + k];
            for (int j = k; j < K; ++j) ATA[i*K + j] -= f * ATA[k*K + j];
            coeffs_rad[i] -= f * coeffs_rad[k];
        }
    }
    for (int i = K - 1; i >= 0; --i) {
        for (int j = i + 1; j < K; ++j)
            coeffs_rad[i] -= ATA[i*K + j] * coeffs_rad[j];
    }
    return true;
}

