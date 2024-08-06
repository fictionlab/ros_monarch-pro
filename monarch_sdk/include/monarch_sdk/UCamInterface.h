#pragma once

#include <string>
#include <bitset>
#include "base.h"

#pragma pack(push, 1)
typedef struct tagS
{
    union
    {
        struct
        {
            int i1 : 4;
            int i2 : 4;
            int i3 : 4;
            int i4 : 4;
        } b4;
        unsigned short s;
    } s1;
} sv;

typedef union tagU
{
    unsigned short s;
    unsigned char uc[2];
} us;
struct BUF_INFO
{
    int size;
    int line_size;
    int index;
    void *buf;
    char *rgbbuf;
    int width;
    int height;
    int position = -1;
};

#define WIDTH 800             // 图片的宽度
#define HEIGHT 1024           // 图片的高度
#define FMT V4L2_PIX_FMT_YUYV // 图片格式
#pragma pack(pop)
#define CLEAR(x) memset(&(x), 0, sizeof(x))

class UCamInterface
{
private:
    int count = 0;
    int setProperty();
    int mainloop();
    void process();
    bool read_frame();
    //转换算法
    int raw10_to_gray8(BUF_INFO *in, BUF_INFO *out);
    int yuyv_to_raw10(BUF_INFO *in, BUF_INFO *out);
    void free_buf(BUF_INFO *frame);
    void convert_10bit_to_8bit(uint8_t *in, uint32_t in_bytes, uint8_t *out);
    void convert_5w_to_10bit(uint8_t *in, uint32_t in_bytes, uint8_t *out);
    void swap_bytes_and_reverse_bits(uint8_t *in, uint32_t in_bytes, uint8_t *out);
    uint8_t reverse_bits(uint8_t in);
    BUF_INFO *allocate_buf(size_t data_bytes);
    void setFrame0(BUF_INFO *info, unsigned short *frame0);
    void setFrame4(BUF_INFO *info, unsigned short *frame4);
    void setFrame7(BUF_INFO *info, unsigned char *frame7);
    void getFrameFromCamImageX(unsigned char *buf, unsigned char *frame);
    unsigned char reverseBitsX(unsigned char num);
    void conv5Wto8short10Bt(); // const char* strMsg, unsigned short* frame2, int len, unsigned short frame3[], int& lenout)
    void makeFixedFrame(unsigned char *frame3, unsigned char *frame4);
    void convert10bto8bX(unsigned short *frame4, unsigned char *frame7);
    void uvc_subtract_black_level(BUF_INFO *in, uint16_t black_level_10bit);
    void initUSERPTR(unsigned int buffer_size);
    void initMMAP();
    int skipTime = 10;
    void calculate_average_brightness(unsigned char *data, const int width,
                                  const int height,
                                  double *brightness);


public:
    UCamInterface();
    ~UCamInterface();
    bool Init(int iCamNumber = 0);
    int camClose();
    int takeFrame(int iMode = 0);
    int getPicture(void *valMatPic);
    int waitKeyX(int i); // { return cv::waitKey(i); };
    void EnableFramesComparison(bool bVal);
    void getFrameFromCamImage(BUF_INFO *info, unsigned short *frame);
    int MakeWorkingFramesFromFrame0(unsigned char *frame0);
    int fd=-1;
    int cache_count = 2;
    time_t t_s, t_e;
    BUF_INFO *infoImg;
    BUF_INFO *infoPreview;
    bool FramesComparison; //是否进行图片比对
    int iVerbocity = 0;
    bool MyCompare(BUF_INFO *img1, BUF_INFO *img2);
    void setGrayDataX(BUF_INFO *info, unsigned char *frame7);
    // unsigned short *frame0;//16位
    // unsigned short *frame4;//10位
    // unsigned char *frame7;//8位

    BUF_INFO *info10, *info8;
    unsigned short frame0[1024][800];
    unsigned short frame4[1024][1280];
    unsigned char frame7[1024][1280];
    int lutMode = -1; //为1时代表使用 LUT MODE 模式拍照
    void setFrameRaw(int position);
    void setFrame10(int postion);
    void setFrame8(int position);
    void setSaved(bool save);
    bool saved = false;
    void clearBuf();
    void stop();
    void restart();
    void setSkiptime(int time);
    void setCacheCount(int cacheCount);
};
