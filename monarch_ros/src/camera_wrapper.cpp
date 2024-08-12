#include <sys/stat.h>
#include <sys/types.h>
#include <png.h>

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "monarch_sdk/CameraCardController.h"

static ros::ServiceServer save_images_service;
static std::unique_ptr<CameraCardController> myCamera;
static unsigned short bufRunLUTFrames[10 * 1024 * 1280];
static int bufRunLUTFramesSize = 10;

void savePng(const char *path, void *data, int width, int height)
{
    FILE *f = fopen(path, "wb");
    png_structp png_ptr = NULL;
    png_infop info_ptr = NULL;
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr)
    {
        printf("png_create_write_struct error %d", 1);
    }
    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        png_destroy_write_struct(&png_ptr, NULL);
        printf("png_create_info_struct error  %d", 1);
    }
    png_init_io(png_ptr, f);
    png_set_IHDR(png_ptr, info_ptr,
                 width,
                 height,
                 8,
                 PNG_COLOR_TYPE_GRAY,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_BASE);
    png_set_packing(png_ptr);
    png_write_info(png_ptr, info_ptr);
    png_bytep row = (png_bytep)data;
    png_bytep png_row = (png_bytep)malloc(width * sizeof(png_byte));
    png_set_compression_level(png_ptr, 0);
    for (uint32_t i = 0; i < height; i++)
    {
        memcpy(png_row, row, width);
        png_write_row(png_ptr, png_row);
        row += width;
    }
    free(png_row);
    png_write_end(png_ptr, info_ptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(f);
}

void CaptureToFile(std::string strFN, unsigned short *usBuff, int iL, std::string pngName)
{
    //save raw
    FILE *fp = fopen(strFN.c_str(), "a");
    fwrite((char *)usBuff, 1, iL * 2, fp);
    fclose(fp);
    // unsigned char *data8;
    unsigned char data_png[1024 * 1280];
    //save png
    for (int j = 0; j < 1280 * 1024; j++)
    {
        data_png[j] = usBuff[j] >> 2;
    }
    savePng(pngName.c_str(), data_png, 1280, 1024);
}

void SaveCapturedLUT(unsigned short *bufFrames, int iLines, int iFrames)
{
    mkdir("./raw", S_IRWXO | S_IRWXG | S_IRWXU);
    int cwls[] = {713, 736, 759, 782, 805, 828, 851, 874, 897, 920};
    int *relCwls = myCamera->getIndexForCapture();
    int size = myCamera->GetLUTIndex() + 1;
    // cout << "capture end start save file " << endl;
    auto start = std::chrono::steady_clock::now();
    int i = 0; // Rows
    int j = 0; // Cols
    int k = 0; // Frames
    int l = 0; // Lines
    unsigned short c1[4];
    unsigned short fps;
    unsigned short gain;
    unsigned short cwl;
    int exp_time;
    int iBPos = 0;
    std::stringstream ss;
    std::string strFileName = "CapLut";
    std::string strFNSave;
    int iBlockL = 1280 * 1024;
    unsigned long iL = 0;
    unsigned short read_gain = myCamera->GetGain();
    int read_exp_time = myCamera->GetExposureTime();
    // cout << endl << "exposure just have been read" << endl << endl;
    //create cube folder
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string n = "cube_" + std::to_string(1900 + ltm->tm_year);
    std::string month = std::string(2 - std::to_string(1 + ltm->tm_mon).length(), '0') + std::to_string(1 + ltm->tm_mon);
    std::string day = std::string(2 - std::to_string(ltm->tm_mday).length(), '0') + std::to_string(ltm->tm_mday);
    std::string hour = std::string(2 - std::to_string(ltm->tm_hour).length(), '0') + std::to_string(ltm->tm_hour);
    std::string min = std::string(2 - std::to_string(ltm->tm_min).length(), '0') + std::to_string(ltm->tm_min);
    std::string sec = std::string(2 - std::to_string(ltm->tm_sec).length(), '0') + std::to_string(ltm->tm_sec);
    n = n + month + day + "_" + hour + min + sec;
    std::string fn = "./raw/" + n;
    // cout << "cube will be saved in: " << fn << endl;
    mkdir(fn.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    std::string name = fn + "/" + "ENVI_" + n + ".raw";
    std::string hdrName = fn + "/" + "ENVI_" + n + ".hdr";
    //save raw and png files
    std::string fPng = fn + "/png";
    mkdir(fPng.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    for (l = 0; l < iLines; l++)
    {
        for (k = 0; k < iFrames; k++)
        {
            myCamera->GetLUTLine(l, c1, &fps, &gain, &cwl);
            gain = read_gain;
            exp_time = read_exp_time;
            iL = iBPos * iBlockL;
            //create png file name
            std::string ff = fPng + "/" + "CWL_" + std::to_string(cwls[relCwls[l]]) + "_" +
                        "Gain_" + std::to_string(gain) + "_" + "Exposure_" + std::to_string(read_exp_time) + ".png";
            CaptureToFile(name, &bufFrames[iL], iBlockL, ff);
            iBPos++;
        }
    }
    //create hdr file
    std::string hdr = "";
    const char* intro =
    "ENVI\n"
    "file type = ENVI Standard\n"
    "sensor type = MONARCH\n"
    "wavelength units = nm\n"
    "data type = 12\n"
    "interleave = BSQ\n"
    "byte order = 0\n"
    "bit depth = 10\n"
    "header offset = 0\n"
    "wavelength = {\n";

    hdr.append(intro);

    for (int i = 0; i < size; i++)
    {
        hdr.append(std::to_string(cwls[relCwls[i]]));
        if (i < size -1)
        {
            hdr.append(",");
        }
        hdr.append("\n");
    }

    hdr.append("}\ngain = {\n)");

    for (int i = 0; i < size; i++)
    {
        hdr.append(std::to_string(gain));
        if (i < size -1)
        {
            hdr.append(",");
        }
        hdr.append("\n");
    }
    hdr.append("}\nexosure time = {\n");
    
    for (int i = 0; i < size; i++)
    {
        hdr.append(std::to_string(exp_time));
        if (i < size - 1)
        {
            hdr.append(",");
        }
        hdr.append("\n");
    }

    const char* outro = 
    "}\n"
    "exposure type = {\n"
    "radiometric-calibration}\n"
    "default bands = {\n"
    "713\n"
    "}\n"    
    "lines = 1024\n"
    "samples = 1280\n"
    "bands = ";
    
    hdr.append(outro);
    hdr.append(std::to_string(size));
    FILE *fp1 = fopen(hdrName.c_str(), "w");
    fwrite(hdr.c_str(), 1, hdr.length(), fp1);
    fclose(fp1);
}

bool CaptureLUTFrames(unsigned short *bufFrames, int &iFramesSize)
{
    int hr = 0;
    std::string newVal;
    try
    {
        iFramesSize = 1;
        hr = myCamera->CaptureLUT(iFramesSize, bufFrames);
        int iNL = 10;
        SaveCapturedLUT(bufFrames, iNL, iFramesSize);
    }
    catch (const std::exception &)
    {
      return false;
    }
  return true;
}

bool save_images_callback(std_srvs::TriggerRequest& req,
                             std_srvs::TriggerResponse& res) {
  
  if(CaptureLUTFrames(bufRunLUTFrames, bufRunLUTFramesSize)) {
    res.success = true;
    return true;
  } else {
    res.success = false;
    return false;
  }
}

void DisplayACTD(CameraCardController *pmyCamera)
{
    ROS_INFO_STREAM("ACTD = " << pmyCamera->GetACTD());
}
void DisplayGainRatio(CameraCardController *pmyCamera)
{
    ROS_INFO_STREAM("GainRatio = " << pmyCamera->GetGain());
}
void DisplayExposureTime(CameraCardController *pmyCamera)
{
    ROS_INFO_STREAM("ExposureTime = " << pmyCamera->GetExposureTime() << ",  fps = " << 1000 / pmyCamera->GetExposureTime());
}
void DisplayTemperture(CameraCardController *pmyCamera)
{
    ROS_INFO_STREAM("Tempreture = " << pmyCamera->GetTemperature());
}
void DisplaySerialNumber(CameraCardController *pmyCamera)
{
    ROS_INFO_STREAM("SerialNumber = " << pmyCamera->GetSerialNumber());
}
void DisplayFWVersion(CameraCardController *pmyCamera)
{
    ROS_INFO_STREAM("Firmware version = " << pmyCamera->GetFwVersion());
}
void DisplayAPIVersion(CameraCardController *pmyCamera)
{
    ROS_INFO_STREAM("Camera API version = " << pmyCamera->GetApiVersion());
}
void DisplayCoords(CameraCardController *pmyCamera)
{
    unsigned short c1[2];
    unsigned short c2[2];
    if (pmyCamera->GetCoords(c1, c2) == 0)
    {
        ROS_INFO_STREAM("Coords = "
             << "(" << (int)c1[0] << ", " << (int)c1[1] << ") -- (" << (int)c2[0] << ", " << (int)c2[1] << ")");
    }
}

void DisplayLutLine(CameraCardController *pmyCamera)
{
    std::string newVal;
    unsigned short c1[4];
    unsigned short fps;
    unsigned short gain;
    unsigned short cwl;
    int index;
    ROS_INFO("LUT Table:");
    for (index = 0; index < 10; index++)
    {
        pmyCamera->GetLUTLine(index, c1, &fps, &gain, &cwl);
        ROS_INFO_STREAM("LutLine(" << index << ") = (" << c1[0] << ", " << c1[1] << ", " << c1[2] << ", " << c1[3] << ") , " << fps << ", " << gain << ", " << cwl);
    }
}

void DisplayOpenCVCameraStatus(CameraCardController *pmyCamera)
{
    if (pmyCamera->IsCameraOpend())
    {
        ROS_INFO_STREAM("OpenCV Camera #: " << pmyCamera->m_iCamID << " - is open");
    }
    else
    {
        ROS_INFO_STREAM("OpenCV Camera #: " << pmyCamera->m_iCamID << " - is close");
    }
}

void DisplayAllData(CameraCardController *pmyCamera)
{
    ROS_INFO("------ Display All Data ------");
    DisplaySerialNumber(pmyCamera);
    DisplayGainRatio(pmyCamera);
    DisplayExposureTime(pmyCamera);
    DisplayTemperture(pmyCamera);
    DisplayACTD(pmyCamera);
    DisplayCoords(pmyCamera);
    DisplayLutLine(pmyCamera);
    DisplayOpenCVCameraStatus(pmyCamera);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "test_node");

  ros::NodeHandle nh;

  myCamera = std::make_unique<CameraCardController>();

  if (myCamera->IsCameraFound()) {
    ROS_INFO("CAMERA FOUND");
    myCamera->InitCam(myCamera->m_iCamID);
    myCamera->PowerLed(1);
    sleep(1);
    DisplayAllData(myCamera.get());
    myCamera->SetExposureFps(10);
    
    CaptureLUTFrames(bufRunLUTFrames, bufRunLUTFramesSize);
    
    save_images_service =
      nh.advertiseService("save_images", &save_images_callback);

    ros::spin();
  } else {
    ROS_INFO("CAMERA NOT FOUND");
  }

  return 0;
}