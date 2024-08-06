#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <png.h>

#include "monarch_sdk/CameraCardController.h"
using namespace std;

#define myVersion "TestSDK V1.0.0"

int test1 = 0;
int videostream_state = 1; // videostream_state is on by default (turned on by cameracard with ENVIRONMENT_INITIALIZE 0x01 command)
int lowpower_state = 0;    // lowpower_state is off by default

unsigned short data_orig[1024][800];
unsigned short data_raw[1024][1280];
unsigned short data16[1024][1280];
unsigned char data8[1024][1280];

int numOfIndexes = 10;

// CameraCardController myCamera;
//////
void TestCheck(int HR, string msg);
void ListCommands();
int main();
void ChangeACTD(CameraCardController *pmyCamera);
void ChangeGainRatio(CameraCardController *pmyCamera);
void ChangeGainRatio(CameraCardController *pmyCamera);
void ChangeExposureTime(CameraCardController *pmyCamera);
void ChangeSerialNumber(CameraCardController *pmyCamera);
void ChangeCoords(CameraCardController *pmyCamera);
void ChangeCurrentLineIndex(CameraCardController *pmyCamera);
void ChangeLutLine(CameraCardController *pmyCamera);
void ChangeDacWrite(CameraCardController *pmyCamera);
void ChangeDacUpdate(CameraCardController *pmyCamera);
void SetBandVoltages(CameraCardController *pmyCamera);

void DisplayACTD(CameraCardController *pmyCamera);
void DisplayAllData(CameraCardController *pmyCamera);
void DisplayGainRatio(CameraCardController *pmyCamera);
void DisplayExposureTime(CameraCardController *pmyCamera);
void DisplayTemperture(CameraCardController *pmyCamera);
void DisplaySerialNumber(CameraCardController *pmyCamera);
void DisplayFWVersion(CameraCardController *pmyCamera);
void DisplayAPIVersion(CameraCardController *pmyCamera);
void DisplayCoords(CameraCardController *pmyCamera);
void DisplayLutLine(CameraCardController *pmyCamera);
void DisplayOpenCVCameraStatus(CameraCardController *pmyCamera);
/////
int OpenCameraCV(CameraCardController *pmyCamera);
int AutoExposure(CameraCardController *pmyCamera);
void CloseCameraCV(CameraCardController *pmyCamera);
void TurnONLed(CameraCardController *pmyCamera);
void TurnOFFLed(CameraCardController *pmyCamera);
void ToggleVideoStream(CameraCardController *pmyCamera);
void ToggleLowPower(CameraCardController *pmyCamera);
void CaptureLUTFrames(CameraCardController *pmyCamera, unsigned short *bufFrames, int &iFramesSize);
void SaveCaptureedLUT(CameraCardController *pmyCamera, unsigned short *bufFrames, int iLines, int iFrames);
void CaptureToFile(string strFN, unsigned short *usBuff, int iL, string pngName);
void savePng(const char *path, void *data, int width, int height);
void calculate_average_brightness(void *data, const int width,
                                  const int height,
                                  double *brightness);
typedef union tagSN
{
    unsigned char uctv[2];
    unsigned short ustv;
    short stv;
} temprVal;

temprVal TestTempr1(float tVal)
{
    short sT1 = (short)(tVal * 256.0);
    temprVal t1;
    t1.stv = sT1;
    string strT1;

    cout << " Test temr (float to bytes): " << tVal << " --- 0x" << std::hex << t1.ustv << endl;
    return t1;
}
float TestTempr2(temprVal tVal)
{
    float fTmpr = 0.0;
    fTmpr = (float)tVal.stv / (float)256.0;

    cout << " Test temr (bytes to float): "
         << "0x" << std::hex << tVal.ustv << " --- " << fTmpr << endl;
    return fTmpr;
}

int get_char()
{
    fd_set rfds;
    struct timeval tv;
    int ch = 0;
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10;
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        ch = getchar();
    }
    return ch;
}

void calculate_average_brightness(unsigned char *data, const int width,
                                  const int height,
                                  double *brightness)
{
    int x1 = 352;
    int x2 = 672;
    int y1 = 440;
    int y2 = 840;

    double sum = 0;
    int length = width * height;
    int allSize = 0;
    for (int m = x1; m < x2; ++m)
    {
        for (int n = y1; n < y2; ++n)
        {
            if (m * 1280 + n < length)
            {
                sum += data[m * 1280 + n];
                ++allSize;
            }
        }
    }
    *brightness = sum / allSize;
}

unsigned short bufRunLUTFrames[10 * 1024 * 1280] = {0};
unsigned short bufRunLUTFramesOrig[20 * 1024 * 800] = {0};
int iExit = 1;
int main()
{
    bool bskipRealPics = false;
    int iR = 0;
    char ch = 0;
    string newGain;
    float fnewGain = 1.0;
    int iCams = 0;
    unsigned char bid[3] = {0};
    unsigned long hr = 0;
    int maxBand = 0;
    int bufRunLUTFramesSize;

    int verbosity = 2;
    int iExposureTime_Start = 2;
    int iGain_Start = 10;
    int iSerial = 1234;

    if (test1 == 0)
        test1 = 1;

    cout << myVersion << endl;

    unsigned short usNums[4] = {16, 31, 33, 257};
    for (int i = 0; i < 4; i++)
    {
        cout << usNums[i] << ",";
    }
    cout << endl;

    int hrDAC = -1;

    bool bWithOpenCV = true;


    CameraCardController myCamera;
    if (myCamera.IsCameraFound())
    {
        // --- Get Frames from camera
        //
        if (bWithOpenCV)
        {
            myCamera.InitCam(myCamera.m_iCamID);
        }
        myCamera.PowerLed(1);
	    sleep(1);
        myCamera.SetExposureFps(10);
        DisplayAllData(&myCamera);
        bool displayAskForInput = true;
        int exp;
        double gain;
        bool restoreValues = false;

        cv::Mat myPic;
        myPic.create(1024, 1280, CV_8UC1);

        int vvv = 0;
        while (iExit == 1)
        {

            if (bWithOpenCV)
            {

                if (!myCamera.IsCameraOpend())
                {
                    myCamera.InitCam(myCamera.m_iCamID);
                }
                if (bskipRealPics)
                {

                    myCamera.GetPreview(&data8[0][0]);
                    cv::Mat img = cv::Mat(cv::Size(1280, 1024), CV_8UC1, data8);
                    cv::imshow("Unispectral", img);
                    double s = 0;
                    calculate_average_brightness(&data8[0][0], 1280, 1024, &s);
                    // std::cout << "mean is: " << s << std::endl;
                }
            }
            if (displayAskForInput)
            {
                cout << "Enter request:>";
                displayAskForInput = false;
            }
            cv::waitKey(1);

            iR = get_char();
            switch (iR)
            {
            case 27:
                iExit = 0;
                break;
            case 'F':
                DisplayFWVersion(&myCamera);
                break;
            case 'V':
                DisplayAPIVersion(&myCamera);
                break;
            case 'a':
                ChangeACTD(&myCamera);
                break;
            case 'A':
                DisplayACTD(&myCamera);
                break;
            case 'g':
                ChangeGainRatio(&myCamera);
                break;
            case 'G':
                DisplayGainRatio(&myCamera);
                break;
            case 'e':
                ChangeExposureTime(&myCamera);
                break;
            case 'E':
                DisplayExposureTime(&myCamera);
                break;
            case 'I':
                ChangeCurrentLineIndex(&myCamera);
                break;
            case 'T':
                DisplayTemperture(&myCamera);
                break;

            case 'r':
                cout << "r - Restore LUT Table";
                hr = myCamera.RestoreLUT();
                cout << "hr : " << hr << endl;
                if (hr == 0)
                {
                    numOfIndexes = 10;
                }
                TestCheck(hr, "RestoreLUT");
                break;
            case 'n':
                ChangeSerialNumber(&myCamera);
                ;
                break;
            case 'N':
                DisplaySerialNumber(&myCamera);
                break;
            case 'h':
            case 'H':
                ListCommands();
                break;
            case '1':
                DisplayAllData(&myCamera);
                break;
            case 'c':
                ChangeCoords(&myCamera);
                break;
            case 'C':
                DisplayCoords(&myCamera);
                break;
            case 'l':
                ChangeLutLine(&myCamera);
                break;
            case 'L':
                DisplayLutLine(&myCamera);
                break;
            case 'x':
                hr = myCamera.AutoExposure(&maxBand);
                break;
            case 'y':
                cout << "***** CaptureLUTFrames Start *****" << endl;
                CaptureLUTFrames(&myCamera, bufRunLUTFrames, bufRunLUTFramesSize);
                cout << "***** CaptureLUTFrames End *****" << endl;
                break;
            case 'b':
            {
                vector<int> idxList;
                string newVal;
                // TODO: get comma-seperated input >> convert it to vector
                cout << "b - BuildCustomLUT" << endl
                     << "Enter relevant indexes:";
                cin >> newVal;
                stringstream ss(newVal);
                for (int i; ss >> i;)
                {
                    idxList.push_back(i);
                    if (ss.peek() == ',')
                        ss.ignore();
                }
                numOfIndexes = idxList.size();
                int *idxListi = &idxList[0];
                cout << "b - BuildCustomLUT with indexes: " << newVal << endl;
                for (int i = 0; i < numOfIndexes; i++)
                {
                    cout << idxListi[i] << endl;
                }

                hr = myCamera.BuildCustomLUT(idxListi, numOfIndexes);
                if (hr == 0)
                {
                    cout << "BuildCustomLUT - OK " << endl;
                }
                else
                {
                    cout << "BuildCustomLUT - FAILED! " << endl;
                }
            }
            break;
            case 'w':
                ChangeDacWrite(&myCamera);
                break;
            case 'u':
                ChangeDacUpdate(&myCamera);
                break;
            case 'v':
                SetBandVoltages(&myCamera);
                break;
            case 'm':
                cout << "m - Open Camera" << endl;
                //myCamera.SetExposureFps(10);
                //exp = myCamera.GetExposureTime();
                // cout << endl << "exposure just have been read" << endl << endl;
                //gain = myCamera.GetGain();
                //restoreValues = true;
                bskipRealPics=true;
                bWithOpenCV = true;
                break;
            case 'M':
                cout << "M - Close Camera" << endl;
                // CloseCameraCV(&myCamera);
                //myCamera.Stop();
                //myCamera.Restart();
                bWithOpenCV = false;
                break;
            case 'd':
                TurnONLed(&myCamera);
                break;
            case 'D':
                TurnOFFLed(&myCamera);
                break;

            case 'S':
                ToggleVideoStream(&myCamera);
                break;

            case 'P':
                ToggleLowPower(&myCamera);
                break;
            case 'Q':
                iExit = 0;
                break;
            }
            iR = 0;
        }
    }

    myCamera.EnableLowPower(0);
    DisplayAllData(&myCamera);

    if (bWithOpenCV)
    {
        myCamera.camClose();
    }
    myCamera.SetExposureFps(iExposureTime_Start);

    myCamera.PowerLed(0);

    return 0;
}

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

void ChangeACTD(CameraCardController *pmyCamera)
{
    int hr;
    string newVal;
    int iVal;
    cout << "Enter new ACTD:";
    cin >> newVal;
    try
    {
        iVal = stoi(newVal);
        hr = pmyCamera->SetACTD(iVal);
        TestCheck(hr, "SetACTD");
    }
    catch (const std::exception &)
    {
    }
}

void ChangeGainRatio(CameraCardController *pmyCamera)
{
    int hr;
    string newVal;
    float fVal;
    cout << "Enter new GainRatio:";
    cin >> newVal;
    try
    {
        fVal = stof(newVal);
        hr = pmyCamera->SetGain(fVal);
        TestCheck(hr, "SetGain");
    }
    catch (const std::exception &)
    {
    }
}
void ChangeExposureTime(CameraCardController *pmyCamera)
{
    int hr;
    string newVal;
    int iVal;
    cout << "Enter new ExposureTime (fps):";
    cin >> newVal;
    try
    {
        iVal = stoi(newVal);
        hr = pmyCamera->SetExposureFps(iVal);
        TestCheck(hr, "SetExposureFps");
    }
    catch (const std::exception &)
    {
    }
}
void ChangeSerialNumber(CameraCardController *pmyCamera)
{
    int hr;
    string newVal;
    int iVal;
    cout << "Enter new  SerialNuber:";
    cin >> newVal;
    try
    {
        iVal = (unsigned int)stoul(newVal);
        hr = pmyCamera->SetSerialNumber(iVal);
        TestCheck(hr, "SetSerialNumber");
    }
    catch (const std::exception &)
    {
    }
}
void ChangeCoords(CameraCardController *pmyCamera)
{
    int hr;
    string newVal = "10 20 30 40";
    unsigned int c1[2];
    unsigned int c2[2];
    unsigned short s1[2];
    unsigned short s2[2];

    cout << "Enter Coords x1,y1,x2,y2 = ";
    cin >> newVal;
    sscanf(newVal.c_str(), "%d,%d,%d,%d", &c1[0], &c1[1], &c2[0], &c2[1]);
    s1[0] = c1[0];
    s1[1] = c1[1];
    s2[0] = c2[0];
    s2[1] = c2[1];
    hr = pmyCamera->SetCoords(s1, s2);
    TestCheck(hr, "SetCoords");
}
void DisplayACTD(CameraCardController *pmyCamera)
{
    cout << "ACTD = " << pmyCamera->GetACTD() << endl;
}
void DisplayGainRatio(CameraCardController *pmyCamera)
{
    cout << "GainRatio = " << pmyCamera->GetGain() << endl;
}
void DisplayExposureTime(CameraCardController *pmyCamera)
{
    cout << "ExposureTime = " << pmyCamera->GetExposureTime();
    // cout << endl << "exposure just have been read" << endl << endl;
    cout << ",  fps = " << 1000 / pmyCamera->GetExposureTime() << endl;
    // cout << endl << "exposure just have been read" << endl << endl;
}
void DisplayTemperture(CameraCardController *pmyCamera)
{
    cout << "Tempreture = " << pmyCamera->GetTemperature() << endl;
}
void DisplaySerialNumber(CameraCardController *pmyCamera)
{
    cout << "SerialNumber = " << pmyCamera->GetSerialNumber() << endl;
}
void DisplayFWVersion(CameraCardController *pmyCamera)
{
    cout << "FWVersion = " << pmyCamera->GetFwVersion() << endl;
}
void DisplayAPIVersion(CameraCardController *pmyCamera)
{
    cout << "APIVersion = " << pmyCamera->GetApiVersion() << endl;
}
void DisplayCoords(CameraCardController *pmyCamera)
{
    unsigned short c1[2];
    unsigned short c2[2];
    if (pmyCamera->GetCoords(c1, c2) == 0)
    {
        cout << "Coords = "
             << "(" << (int)c1[0] << ", " << (int)c1[1] << ") -- (" << (int)c2[0] << ", " << (int)c2[1] << ")" << endl;
    }
}
void DisplayAllData(CameraCardController *pmyCamera)
{
    cout << "------ Display All Data ------" << endl;
    DisplaySerialNumber(pmyCamera);
    DisplayGainRatio(pmyCamera);
    DisplayExposureTime(pmyCamera);
    DisplayTemperture(pmyCamera);
    DisplayACTD(pmyCamera);
    DisplayCoords(pmyCamera);
    DisplayLutLine(pmyCamera);
    DisplayOpenCVCameraStatus(pmyCamera);
}
void ChangeCurrentLineIndex(CameraCardController *pmyCamera)
{
    int hr;
    string newVal;
    int iVal;
    cout << "Enter new Current LUT Index:";
    cin >> newVal;
    try
    {
        iVal = stoi(newVal);
        hr = pmyCamera->SetLUTIndex(iVal);
        TestCheck(hr, "SetLUTIndex");
    }
    catch (const std::exception &)
    {
    }
}
void ChangeLutLine(CameraCardController *pmyCamera)
{
    int hr;
    string newVal;
    unsigned short c1[4];
    unsigned short fps;
    unsigned short gain;
    unsigned short cwl;
    int index;
    int ic1[4];
    int ifps;
    int igain;
    int icwl;

    cout << "Enter LUT Line index,v1,v2,v3,v4,fps,gain,cwl = ";
    cin >> newVal;
    sscanf(newVal.c_str(), "%d,%d,%d,%d,%d,%d,%d,%d", &index, &ic1[0], &ic1[1], &ic1[2], &ic1[3], &ifps, &igain, &icwl);
    for (int i = 0; i < 4; i++)
    {
        c1[i] = ic1[i];
    }
    fps = ifps;
    gain = igain;
    cwl = icwl;
    pmyCamera->SetLUTIndex(index);
    hr = pmyCamera->SetLUTLine(c1, fps, gain, cwl);
    TestCheck(hr, "SetLUTLine");
}

void DisplayLutLine(CameraCardController *pmyCamera)
{
    string newVal;
    unsigned short c1[4];
    unsigned short fps;
    unsigned short gain;
    unsigned short cwl;
    int index;
    cout << "L - Display LUT Table " << endl;
    for (index = 0; index < 10; index++)
    {
        pmyCamera->GetLUTLine(index, c1, &fps, &gain, &cwl);
        cout << "LutLine(" << index << ") = (" << c1[0] << ", " << c1[1] << ", " << c1[2] << ", " << c1[3] << ") , " << fps << ", " << gain << ", " << cwl << endl;
    }
}
void DisplayOpenCVCameraStatus(CameraCardController *pmyCamera)
{
    if (pmyCamera->IsCameraOpend())
    {
        cout << "OpenCV Camera #: " << pmyCamera->m_iCamID << " - is open" << endl;
    }
    else
    {
        cout << "OpenCV Camera #: " << pmyCamera->m_iCamID << " - is close" << endl;
    }
}


void TestCheck(int hr, string msg)
{
    if (hr == 0)
    {
        cout << "Test <" << msg << "> succceed." << endl;
    }
    else
    {
        cout << "Test <" << msg << "> failed!" << endl;
    }
}

void ChangeDacWrite(CameraCardController *pmyCamera)
{
    int hr;
    string newVal;
    int index;
    int idv;

    cout << "Enter for DacWrite: index, dav voltage(0-4095) = ";
    cin >> newVal;
    sscanf(newVal.c_str(), "%d,%d", &index, &idv);
    hr = pmyCamera->DacWrite(index, idv);
    TestCheck(hr, "DacWrite");
}
void ChangeDacUpdate(CameraCardController *pmyCamera)
{
    int hr;
    string newVal;
    int index;

    cout << "Enter for DacUpdate: index = ";
    cin >> newVal;
    sscanf(newVal.c_str(), "%d", &index);
    hr = pmyCamera->DacUpdate(index);
    TestCheck(hr, "DacUpdate");
}

void SetBandVoltages(CameraCardController *pmyCamera)
{
    int hr;
    string newVal;
    int index;

    cout << "Enter line index : ";
    cin >> newVal;
    sscanf(newVal.c_str(), "%d", &index);
    hr = pmyCamera->SetLineVoltages(index);
    TestCheck(hr, "SetLineVoltages");
}

int OpenCameraCV(CameraCardController *pmyCamera)
{
    return pmyCamera->InitCam(pmyCamera->m_iCamID);
}
void CloseCameraCV(CameraCardController *pmyCamera)
{
    pmyCamera->camClose();
    DisplayOpenCVCameraStatus(pmyCamera);
}

void TurnONLed(CameraCardController *pmyCamera)
{
    pmyCamera->PowerLed(1);
    cout << "d - Led set to ON" << endl;
}
void TurnOFFLed(CameraCardController *pmyCamera)
{
    pmyCamera->PowerLed(0);
    cout << "D - Led set to OFF" << endl;
}

void ToggleVideoStream(CameraCardController *pmyCamera)
{
    videostream_state = !videostream_state;
    pmyCamera->EnableVideoStream(videostream_state);
    cout << "S - videostream state : " << videostream_state << endl;
}
void ToggleLowPower(CameraCardController *pmyCamera)
{
    lowpower_state = !lowpower_state;
    cout << "lowpower_state " << lowpower_state << std::endl;
    pmyCamera->EnableLowPower(lowpower_state);
    cout << "P - lowpower state : " << lowpower_state << endl;
}

void CaptureLUTFrames(CameraCardController *pmyCamera, unsigned short *bufFrames, int &iFramesSize)
{
    int hr = 0;
    string newVal;
    try
    {
        iFramesSize = 1;
        hr = pmyCamera->CaptureLUT(iFramesSize, bufFrames);
        TestCheck(hr, "RunLUT(Frames)");
        int iNL = numOfIndexes;
        SaveCaptureedLUT(pmyCamera, bufFrames, iNL, iFramesSize);
    }
    catch (const std::exception &)
    {
    }
}

void ListCommands()
{
    cout << "h - Display Help" << endl;
    cout << "<Esc> - Exit Application" << endl;
    cout << "1 - Display data" << endl;
    cout << "F - Display FW version" << endl;
    cout << "V - Display API version" << endl;
    // cout << "a - Change ACDT Time" << endl;
    cout << "A - Display ACDT Time" << endl;
    cout << "e - Change Exposure Time" << endl;
    cout << "E - Display Exposure Time" << endl;
    cout << "g - Change Gain Ratio" << endl;
    cout << "G - Display Gain Ratio" << endl;
    cout << "T - Display Temperture" << endl;
    // cout << "n - Change Serial Number" << endl;
    cout << "N - Display Serial Number" << endl;
    // cout << "c - Change Coords" << endl;
    // cout << "C - Display Coords" << endl;
    // cout << "l - Change LUT Line" << endl;
    cout << "L - Display LUT Line" << endl;
    cout << "v - Set LUT Line Voltages" << endl;
    // cout << "x - RunLUT" << endl;
    cout << "y - Capture Cube" << endl;
    // cout << "s - SaveLUT" << endl;
    cout << "r - RestoreLUT" << endl;
    cout << "b - BuilsCustomLUT" << endl;
    // cout << "w - Dac write" << endl;
    cout << "u - Dac update" << endl;
    cout << "m - Turn on OpenCV camera" << endl;
    cout << "M - Turn off OpenCV camera" << endl;
    cout << "d - Turn on LED" << endl;
    cout << "D - Turn off LED" << endl;
}

void SaveCaptureedLUT(CameraCardController *pmyCamera, unsigned short *bufFrames, int iLines, int iFrames)
{
    mkdir("./raw", S_IRWXO);
    int cwls[] = {713, 736, 759, 782, 805, 828, 851, 874, 897, 920};
    int *relCwls = pmyCamera->getIndexForCapture();
    int size = pmyCamera->GetLUTIndex() + 1;
    cout << "capture end start save file " << endl;
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
    stringstream ss;
    string strFileName = "CapLut";
    string strFNSave;
    int iBlockL = 1280 * 1024;
    unsigned long iL = 0;
    unsigned short read_gain = pmyCamera->GetGain();
    int read_exp_time = pmyCamera->GetExposureTime();
    // cout << endl << "exposure just have been read" << endl << endl;
    //create cube folder
    time_t now = time(0);
    tm *ltm = localtime(&now);
    string n = "cube_" + to_string(1900 + ltm->tm_year);
    string month = string(2 - to_string(1 + ltm->tm_mon).length(), '0') + to_string(1 + ltm->tm_mon);
    string day = string(2 - to_string(ltm->tm_mday).length(), '0') + to_string(ltm->tm_mday);
    string hour = string(2 - to_string(ltm->tm_hour).length(), '0') + to_string(ltm->tm_hour);
    string min = string(2 - to_string(ltm->tm_min).length(), '0') + to_string(ltm->tm_min);
    string sec = string(2 - to_string(ltm->tm_sec).length(), '0') + to_string(ltm->tm_sec);
    n = n + month + day + "_" + hour + min + sec;
    string fn = "./raw/" + n;
    cout << "cube will be saved in: " << fn << endl;
    mkdir(fn.c_str(), S_IRWXO);
    std::string name = fn + "/" + "ENVI_" + n + ".raw";
    std::string hdrName = fn + "/" + "ENVI_" + n + ".hdr";
    //save raw and png files
    string fPng = fn + "/png";
    mkdir(fPng.c_str(), S_IRWXO);
    for (l = 0; l < iLines; l++)
    {
        for (k = 0; k < iFrames; k++)
        {
            pmyCamera->GetLUTLine(l, c1, &fps, &gain, &cwl);
            gain = read_gain;
            exp_time = read_exp_time;
            iL = iBPos * iBlockL;
            //create png file name
            string ff = fPng + "/" + "CWL_" + to_string(cwls[relCwls[l]]) + "_" +
                        "Gain_" + to_string(gain) + "_" + "Exposure_" + to_string(read_exp_time) + ".png";
            CaptureToFile(name, &bufFrames[iL], iBlockL, ff);
            iBPos++;
        }
    }
    //create hdr file
    string hdr = "ENVI";
    hdr.append("\n");
    hdr.append("file type = ENVI Standard");
    hdr.append("\n");
    hdr.append("sensor type = MONARCH");
    hdr.append("\n");
    hdr.append("wavelength units = nm");
    hdr.append("\n");
    hdr.append("data type = 12");
    hdr.append("\n");
    hdr.append("interleave = BSQ");
    hdr.append("\n");
    hdr.append("byte order = 0");
    hdr.append("\n");
    hdr.append("bit depth = 10");
    hdr.append("\n");
    hdr.append("header offset = 0");
    hdr.append("\n");
    hdr.append("wavelength = {");
    hdr.append("\n");
    for (int i = 0; i < size; i++)
    {
        hdr.append(to_string(cwls[relCwls[i]]));
        if (i < size -1)
        {
            hdr.append(",");
        }
        hdr.append("\n");
    }
    hdr.append("}");
    hdr.append("\n");
    hdr.append("gain = {");
    hdr.append("\n");
    for (int i = 0; i < size; i++)
    {
        hdr.append(to_string(gain));
        if (i < size -1)
        {
            hdr.append(",");
        }
        hdr.append("\n");
    }
    hdr.append("}");
    hdr.append("\n");
    hdr.append("exposure time = {");
    hdr.append("\n");
    for (int i = 0; i < size; i++)
    {
        hdr.append(to_string(exp_time));
        if (i < size - 1)
        {
            hdr.append(",");
        }
        hdr.append("\n");
    }
    hdr.append("}");
    hdr.append("\n");
    hdr.append("exposure type = {");
    hdr.append("\n");
    hdr.append("radiometric-calibration");
    hdr.append("}");
    hdr.append("\n");
    hdr.append("default bands = {");
    hdr.append("\n");
    hdr.append("713");
    hdr.append("\n");
    hdr.append("}");
    hdr.append("\n");
    hdr.append("lines = 1024");
    hdr.append("\n");
    hdr.append("samples = 1280");
    hdr.append("\n");
    hdr.append("bands = ");
    hdr.append(to_string(size));
    FILE *fp1 = fopen(hdrName.c_str(), "w");
    fwrite(hdr.c_str(), 1, hdr.length(), fp1);
    fclose(fp1);
    auto end = std::chrono::steady_clock::now();
    std::cout << "gain is " << gain << " exposure is " << exp_time << " save file time is : "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms" << std::endl;
    cout << "save success" << endl;
}

void CaptureToFile(string strFN, unsigned short *usBuff, int iL, string pngName)
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