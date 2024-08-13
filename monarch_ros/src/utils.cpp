#include "utils.hpp"

#include <png.h>
#include <sys/stat.h>
#include <sys/types.h>

CameraWrapper::CameraWrapper(int fps, float gain) {
  myCamera = std::make_unique<CameraCardController>();

  if (myCamera->IsCameraFound()) {
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

void CameraWrapper::save_png(const char *path, void *data, int width,
                             int height) {
  FILE *f = fopen(path, "wb");
  png_structp png_ptr = NULL;
  png_infop info_ptr = NULL;
  png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png_ptr) {
    printf("png_create_write_struct error %d", 1);
  }
  info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_write_struct(&png_ptr, NULL);
    printf("png_create_info_struct error  %d", 1);
  }
  png_init_io(png_ptr, f);
  png_set_IHDR(png_ptr, info_ptr, width, height, 8, PNG_COLOR_TYPE_GRAY,
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
               PNG_FILTER_TYPE_BASE);
  png_set_packing(png_ptr);
  png_write_info(png_ptr, info_ptr);
  png_bytep row = (png_bytep)data;
  png_bytep png_row = (png_bytep)malloc(width * sizeof(png_byte));
  png_set_compression_level(png_ptr, 0);
  for (uint32_t i = 0; i < height; i++) {
    memcpy(png_row, row, width);
    png_write_row(png_ptr, png_row);
    row += width;
  }
  free(png_row);
  png_write_end(png_ptr, info_ptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);
  fclose(f);
}

void CameraWrapper::CaptureToFile(std::string strFN, unsigned short *usBuff,
                                  int iL, std::string pngName) {
  // save raw
  FILE *fp = fopen(strFN.c_str(), "a");
  fwrite((char *)usBuff, 1, iL * 2, fp);
  fclose(fp);
  // unsigned char *data8;
  unsigned char data_png[1024 * 1280];
  // save png
  for (int j = 0; j < 1280 * 1024; j++) {
    data_png[j] = usBuff[j] >> 2;
  }
  save_png(pngName.c_str(), data_png, 1280, 1024);
}

void CameraWrapper::SaveCapturedLUT(unsigned short *bufFrames, int iLines,
                                    int iFrames, std::string path) {
  mkdir(path.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
  int cwls[] = {713, 736, 759, 782, 805, 828, 851, 874, 897, 920};
  int *relCwls = myCamera->getIndexForCapture();
  int size = myCamera->GetLUTIndex() + 1;

  unsigned short c1[4];
  unsigned short fps;
  unsigned short gain;
  unsigned short cwl;
  int exp_time;
  int iBPos = 0;
  int iBlockL = 1280 * 1024;
  unsigned long iL = 0;
  unsigned short read_gain = myCamera->GetGain();
  int read_exp_time = myCamera->GetExposureTime();

  // create cube folder
  time_t now = time(0);
  tm *ltm = localtime(&now);
  std::string n = "cube_" + std::to_string(1900 + ltm->tm_year);
  std::string month =
      std::string(2 - std::to_string(1 + ltm->tm_mon).length(), '0') +
      std::to_string(1 + ltm->tm_mon);
  std::string day =
      std::string(2 - std::to_string(ltm->tm_mday).length(), '0') +
      std::to_string(ltm->tm_mday);
  std::string hour =
      std::string(2 - std::to_string(ltm->tm_hour).length(), '0') +
      std::to_string(ltm->tm_hour);
  std::string min = std::string(2 - std::to_string(ltm->tm_min).length(), '0') +
                    std::to_string(ltm->tm_min);
  std::string sec = std::string(2 - std::to_string(ltm->tm_sec).length(), '0') +
                    std::to_string(ltm->tm_sec);
  n = n + month + day + "_" + hour + min + sec;
  std::string fn = path + "/" + n;
  // cout << "cube will be saved in: " << fn << endl;
  mkdir(fn.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
  std::string name = fn + "/" + "ENVI_" + n + ".raw";
  std::string hdrName = fn + "/" + "ENVI_" + n + ".hdr";
  // save raw and png files
  std::string fPng = fn + "/png";
  mkdir(fPng.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
  for (int l = 0; l < iLines; l++) {
    for (int k = 0; k < iFrames; k++) {
      myCamera->GetLUTLine(l, c1, &fps, &gain, &cwl);
      gain = read_gain;
      exp_time = read_exp_time;
      iL = iBPos * iBlockL;
      // create png file name
      std::string ff = fPng + "/" + "CWL_" + std::to_string(cwls[relCwls[l]]) +
                       "_" + "Gain_" + std::to_string(gain) + "_" +
                       "Exposure_" + std::to_string(read_exp_time) + ".png";
      CaptureToFile(name, &bufFrames[iL], iBlockL, ff);
      iBPos++;
    }
  }
  // create hdr file
  std::string hdr = "";
  const char *intro = "ENVI\n"
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

  for (int i = 0; i < size; i++) {
    hdr.append(std::to_string(cwls[relCwls[i]]));
    if (i < size - 1) {
      hdr.append(",");
    }
    hdr.append("\n");
  }

  hdr.append("}\ngain = {\n)");

  for (int i = 0; i < size; i++) {
    hdr.append(std::to_string(gain));
    if (i < size - 1) {
      hdr.append(",");
    }
    hdr.append("\n");
  }
  hdr.append("}\nexosure time = {\n");

  for (int i = 0; i < size; i++) {
    hdr.append(std::to_string(exp_time));
    if (i < size - 1) {
      hdr.append(",");
    }
    hdr.append("\n");
  }

  const char *outro = "}\n"
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

bool CameraWrapper::CaptureLUTFrames(std::string path) {
  int hr = 0;
  try {
    hr = myCamera->CaptureLUT(1, bufRunLUTFrames);
    SaveCapturedLUT(bufRunLUTFrames, 10, 1, path);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

std::stringstream CameraWrapper::DisplayACTD() {
  std::stringstream message;
  message << "ACTD = " << myCamera->GetACTD();
  return message;
}

std::stringstream CameraWrapper::DisplayGainRatio() {
  std::stringstream message;
  message << "GainRatio = " << myCamera->GetGain();
  return message;
}

std::stringstream CameraWrapper::DisplayExposureTime() {
  std::stringstream message;
  message << "ExposureTime = " << myCamera->GetExposureTime()
          << ",  fps = " << 1000 / myCamera->GetExposureTime();
  return message;
}

std::stringstream CameraWrapper::DisplayTemperture() {
  std::stringstream message;
  message << "Tempreture = " << myCamera->GetTemperature();
  return message;
}

std::stringstream CameraWrapper::DisplaySerialNumber() {
  std::stringstream message;
  message << "SerialNumber = " << myCamera->GetSerialNumber();
  return message;
}

std::stringstream CameraWrapper::DisplayFWVersion() {
  std::stringstream message;
  message << "Firmware version = " << myCamera->GetFwVersion();
  return message;
}

std::stringstream CameraWrapper::DisplayAPIVersion() {
  std::stringstream message;
  message << "Camera API version = " << myCamera->GetApiVersion();
  return message;
}

std::stringstream CameraWrapper::DisplayCoords() {
  std::stringstream message("");
  unsigned short c1[2];
  unsigned short c2[2];
  if (myCamera->GetCoords(c1, c2) == 0) {
    message << "Coords = "
            << "(" << (int)c1[0] << ", " << (int)c1[1] << ") -- (" << (int)c2[0]
            << ", " << (int)c2[1] << ")";
  }
  return message;
}

std::stringstream CameraWrapper::DisplayLutLine() {
  std::stringstream table;
  unsigned short c1[4];
  unsigned short fps;
  unsigned short gain;
  unsigned short cwl;

  table << "LUT Table:\n";
  for (int index = 0; index < 10; index++) {
    myCamera->GetLUTLine(index, c1, &fps, &gain, &cwl);
    table << "LutLine(" << index << ") = (" << c1[0] << ", " << c1[1] << ", "
          << c1[2] << ", " << c1[3] << ") , " << fps << ", " << gain << ", "
          << cwl;
    if (index < 9)
      table << "\n";
  }
  return table;
}

std::stringstream CameraWrapper::DisplayOpenCVCameraStatus() {

  std::stringstream message;
  if (myCamera->IsCameraOpend()) {
    message << "OpenCV Camera #: " << myCamera->m_iCamID << " - is open";
  } else {
    message << "OpenCV Camera #: " << myCamera->m_iCamID << " - is close";
  }

  return message;
}

std::stringstream CameraWrapper::DisplayAllData() {
  std::stringstream data;
  data << "------ Display All Data ------\n";
  data << DisplaySerialNumber().str() << "\n";
  data << DisplayGainRatio().str() << "\n";
  data << DisplayExposureTime().str() << "\n";
  data << DisplayTemperture().str() << "\n";
  data << DisplayACTD().str() << "\n";
  data << DisplayCoords().str() << "\n";
  data << DisplayLutLine().str() << "\n";
  data << DisplayOpenCVCameraStatus().str() << "\n";

  return data;
}

void CameraWrapper::SetFps(int fps) { myCamera->SetExposureFps(fps); }

void CameraWrapper::SetGain(float gain) { myCamera->SetGain(gain); }
