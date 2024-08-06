#include "CameraCardController.h"
#define DLLIMPORT extern "C" 

// Control methods
DLLIMPORT int SetExposureFps(unsigned int fps);
DLLIMPORT int AutoExposure(int* max_bright_band);
DLLIMPORT int GetExposureTime();
DLLIMPORT int SetGain(float gainVal);
DLLIMPORT float GetGain();

// LUT - Basic methods
DLLIMPORT int SetLineVoltages(int index);
DLLIMPORT int BuildCustomLUT(int* iRelevantIndexes, int iL);
DLLIMPORT int CaptureLUT(unsigned short* frames);
//DLLIMPORT HRESULT CaptureLUT(int iNumFramesForBands, unsigned short* frames);
DLLIMPORT int RestoreLUT();

// LUT - Advanced methods
DLLIMPORT void EnableLowPower(int is_power_low);
DLLIMPORT int GetLUTLine(int index, unsigned short* converted_voltages, unsigned short* fps, unsigned short* gain_value, unsigned short* cwl);
DLLIMPORT void LoadVoltages();

// Camera methods
DLLIMPORT int InitCam(int iCamNumber = -1);
DLLIMPORT void SkipFrames(int number_of_frames);
DLLIMPORT int GetFrameRaw(unsigned short* frame, bool isTake = true);
DLLIMPORT int GetFrame10(unsigned short* frame, bool isTake = true);
DLLIMPORT int GetFrame8(unsigned char* frame, bool isTake = true);
DLLIMPORT int GetPreview(unsigned char* frame, bool isTake = true);
DLLIMPORT void Release();
DLLIMPORT int GetCamIndex();
// Other methods
DLLIMPORT void PowerLed(int iState);
DLLIMPORT float GetTemperature();
DLLIMPORT unsigned int GetSerialNumber();
DLLIMPORT int GetFwVersion();
DLLIMPORT int GetApiVersion();
DLLIMPORT void Reset();