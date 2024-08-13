#include <memory>
#include <sstream>

#include "monarch_sdk/CameraCardController.h"

class CameraWrapper {   
    public:
        CameraWrapper(int fps, float gain);
        ~CameraWrapper();
        void save_png(const char *path, void *data, int width, int height);
        void CaptureToFile(std::string strFN, unsigned short *usBuff, int iL, std::string pngName);
        void SaveCapturedLUT(unsigned short *bufFrames, int iLines, int iFrames);
        bool CaptureLUTFrames();
        std::stringstream DisplayACTD(CameraCardController *pmyCamera);
        std::stringstream DisplayGainRatio(CameraCardController *pmyCamera);
        std::stringstream DisplayExposureTime(CameraCardController *pmyCamera);
        std::stringstream DisplayTemperture(CameraCardController *pmyCamera);
        std::stringstream DisplaySerialNumber(CameraCardController *pmyCamera);
        std::stringstream DisplayFWVersion(CameraCardController *pmyCamera);
        std::stringstream DisplayAPIVersion(CameraCardController *pmyCamera);
        std::stringstream DisplayCoords(CameraCardController *pmyCamera);
        std::stringstream DisplayLutLine(CameraCardController *pmyCamera);
        std::stringstream DisplayOpenCVCameraStatus(CameraCardController *pmyCamera);
        std::stringstream DisplayAllData();

        bool cameraFound = false;

    private:
        std::unique_ptr<CameraCardController> myCamera;
        unsigned short bufRunLUTFrames[10 * 1024 * 1280];
        int bufRunLUTFramesSize = 10;
};