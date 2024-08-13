#include <memory>
#include <sstream>

#include "monarch_sdk/CameraCardController.h"

class CameraWrapper {
public:
  CameraWrapper(int fps, float gain);
  ~CameraWrapper();
  void save_png(const char *path, void *data, int width, int height);
  void CaptureToFile(std::string strFN, unsigned short *usBuff, int iL,
                     std::string pngName);
  void SaveCapturedLUT(unsigned short *bufFrames, int iLines, int iFrames,
                       std::string path);
  bool CaptureLUTFrames(std::string path);
  std::stringstream DisplayAllData();
  std::stringstream DisplayGainRatio();
  std::stringstream DisplayExposureTime();
  void SetGain(float gain);
  void SetFps(int fps);

  bool cameraFound = false;

private:
  std::stringstream DisplayACTD();
  std::stringstream DisplayTemperture();
  std::stringstream DisplaySerialNumber();
  std::stringstream DisplayFWVersion();
  std::stringstream DisplayAPIVersion();
  std::stringstream DisplayCoords();
  std::stringstream DisplayLutLine();
  std::stringstream DisplayOpenCVCameraStatus();

  std::unique_ptr<CameraCardController> myCamera;
  unsigned short bufRunLUTFrames[10 * 1024 * 1280];
};