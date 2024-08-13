#include "utils.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <png.h>
#include <chrono>

CameraWrapper::CameraWrapper(int fps, float gain) {
    myCamera = std::make_unique<CameraCardController>();

    if(myCamera->IsCameraFound()) {   
        cameraFound = true;
        myCamera->InitCam(myCamera->m_iCamID);
        myCamera->PowerLed(1);
        sleep(1);
        myCamera->SetExposureFps(fps);
        myCamera->SetGain(gain);
    }
}

CameraWrapper::~CameraWrapper() {
    myCamera->Release();
    myCamera.reset();
}

void CameraWrapper::save_png(const char *path, void *data, int width, int height)
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

void CameraWrapper::CaptureToFile(std::string strFN, unsigned short *usBuff, int iL, std::string pngName)
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
    save_png(pngName.c_str(), data_png, 1280, 1024);
}

void CameraWrapper::SaveCapturedLUT(unsigned short *bufFrames, int iLines, int iFrames)
{
    mkdir("~/raw", S_IRWXO | S_IRWXG | S_IRWXU);
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

bool CameraWrapper::CaptureLUTFrames()
{
    int hr = 0;
    std::string newVal;
    try
    {
        bufRunLUTFramesSize = 1;
        hr = myCamera->CaptureLUT(bufRunLUTFramesSize, bufRunLUTFrames);
        int iNL = 10;
        SaveCapturedLUT(bufRunLUTFrames, iNL, bufRunLUTFramesSize);
    }
    catch (const std::exception &)
    {
      return false;
    }
  return true;
}

std::stringstream CameraWrapper::DisplayACTD(CameraCardController *pmyCamera)
{
    std::stringstream message;
    message << "ACTD = " << pmyCamera->GetACTD();
    return message;
}

std::stringstream CameraWrapper::DisplayGainRatio(CameraCardController *pmyCamera)
{
    std::stringstream message;
    message << "GainRatio = " << pmyCamera->GetGain();
    return message;
}

std::stringstream CameraWrapper::DisplayExposureTime(CameraCardController *pmyCamera)
{
    std::stringstream message;
    message << "ExposureTime = " << pmyCamera->GetExposureTime() << ",  fps = " << 1000 / pmyCamera->GetExposureTime();
    return message;
}

std::stringstream CameraWrapper::DisplayTemperture(CameraCardController *pmyCamera)
{
    std::stringstream message;
    message << "Tempreture = " << pmyCamera->GetTemperature();
    return message;
}

std::stringstream CameraWrapper::DisplaySerialNumber(CameraCardController *pmyCamera)
{
    std::stringstream message;
    message << "SerialNumber = " << pmyCamera->GetSerialNumber();
    return message;
}

std::stringstream CameraWrapper::DisplayFWVersion(CameraCardController *pmyCamera)
{
    std::stringstream message;
    message << "Firmware version = " << pmyCamera->GetFwVersion();
    return message;
}

std::stringstream CameraWrapper::DisplayAPIVersion(CameraCardController *pmyCamera)
{
    std::stringstream message;
    message << "Camera API version = " << pmyCamera->GetApiVersion();
    return message;
}

std::stringstream CameraWrapper::DisplayCoords(CameraCardController *pmyCamera)
{
    std::stringstream message("");
    unsigned short c1[2];
    unsigned short c2[2];
    if (pmyCamera->GetCoords(c1, c2) == 0)
    {
        message << "Coords = "
             << "(" << (int)c1[0] << ", " << (int)c1[1] << ") -- (" << (int)c2[0] << ", " << (int)c2[1] << ")";
    }
    return message;
}

std::stringstream CameraWrapper::DisplayLutLine(CameraCardController *pmyCamera)
{
    std::stringstream table;
    std::string newVal;
    unsigned short c1[4];
    unsigned short fps;
    unsigned short gain;
    unsigned short cwl;
    int index;

    table << "LUT Table:\n";
    for (index = 0; index < 10; index++)
    {
        pmyCamera->GetLUTLine(index, c1, &fps, &gain, &cwl);
        table << "LutLine(" << index << ") = (" << c1[0] << ", " << c1[1] << ", " << c1[2] << ", " << c1[3] << ") , " << fps << ", " << gain << ", " << cwl;
        if (index < 9)
            table << "\n";
    }
    return table;
}

std::stringstream CameraWrapper::DisplayOpenCVCameraStatus(CameraCardController *pmyCamera)
{
    
    std::stringstream message;
    if (pmyCamera->IsCameraOpend())
    {
        message << "OpenCV Camera #: " << pmyCamera->m_iCamID << " - is open";
    }
    else
    {
        message << "OpenCV Camera #: " << pmyCamera->m_iCamID << " - is close";
    }
    
    return message;
}

std::stringstream CameraWrapper::DisplayAllData()
{
    std::stringstream data;
    data << "------ Display All Data ------\n";
    data << DisplaySerialNumber(myCamera.get()).str() << "\n";
    data << DisplayGainRatio(myCamera.get()).str() << "\n";
    data << DisplayExposureTime(myCamera.get()).str() << "\n";
    data << DisplayTemperture(myCamera.get()).str() << "\n";
    data << DisplayACTD(myCamera.get()).str() << "\n";
    data << DisplayCoords(myCamera.get()).str() << "\n";
    data << DisplayLutLine(myCamera.get()).str() << "\n";
    data << DisplayOpenCVCameraStatus(myCamera.get()).str() << "\n";

    return data;
}
