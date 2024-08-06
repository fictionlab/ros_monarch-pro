#include "UCamInterface.h"

#pragma once

#define UnsC extern "C"

UnsC int initCam(int iCamNumber = 0);
UnsC bool IsCameraOpend();
UnsC int camClose();
UnsC int MakeWorkingFramesFromFrame0(unsigned char *frame);
UnsC int skipFrames(int iN);
UnsC int takeFrame(int iMode);
UnsC int getFrame1(unsigned short *frame, int position = -1);
UnsC int getFrame2(unsigned char *frame, int position = -1);
UnsC int getPicture(void *ValMatPic);
UnsC void convert10bto8bX_ucam(unsigned short *frame_16, unsigned char *frame_8);
UnsC int waitKeyX(int i);
UnsC void listCameras();
UnsC void SetFirstLUTFrame(bool bVal);
UnsC void EnableFramesComparison(bool bVal);
UnsC void SwapBinary16(std::uint16_t &value);
UnsC void SwapBinary32(std::uint32_t &value);
UnsC int getFrame00(unsigned short *frame);
UnsC int getFrame0(unsigned short *frame, int position = -1);
UnsC void SwapBinary64(std::uint64_t &value);
UnsC void setSave(bool save);
UnsC void clearBuf();
UnsC void stop();
UnsC void restart();
UnsC void setSkiptime(int time);
UnsC void setCacheCount(int cache);