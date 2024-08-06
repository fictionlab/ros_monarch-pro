#pragma once

#define DLLIMPORT extern "C"

DLLIMPORT int InitCard(int = 0);
DLLIMPORT unsigned long  put_property(unsigned long id, unsigned long size, unsigned char value[]);
DLLIMPORT unsigned long get_property(unsigned long id, unsigned long size, unsigned char valueRet[]);
DLLIMPORT unsigned long get_propertySize(unsigned long id, unsigned long* size);
DLLIMPORT void getCameraList(int camIndex[], unsigned int cardISN[], char camNames[10][20], int* iCams);

