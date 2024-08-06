//基础控制类
#include <cstring>
#include <list>
#include <vector>
#include <map>
#include "base.h"
#pragma once
#define SAFE_RELEASE(x) \
    if (x)              \
    {                   \
        (x)->Release(); \
        (x) = NULL;     \
    }

typedef std::vector<std::string> dev_vec;
typedef std::map<std::string, std::string> dev_map;

class card_interface
{
private:
    int fd = -1;      //设备句柄
    int mediaFd = -1; //获取media id
    int fwVersion;
    //发现设备
    dev_map findDevices();
    bool is_v4l_dev(const char *name);
    void testCammend();
    int mi_get_media_fd(int fd, const char *bus_info = NULL);
    int mi_get_dev_t_from_fd(int fd, dev_t *dev);
    void setFwVersion();

public:
    card_interface();
    ~card_interface();
    //初始化
    int Init(int iCamSN = -1);
    void SetFd(int newFd);
    //获取命令
    int get_Property(ulong propertyId, ulong ulSize, BYTE pValue[]);
    //发送指令
    int put_Property(ulong propertyId, ulong ulSize, BYTE pValue[]);
    //获取属性大小
    int get_PropertySize(ulong propertyId, ulong *pulSize);
    int GetFWVersion(); // int iUNSCam
};
