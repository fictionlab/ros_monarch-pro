#include "card_interface.h"
#include "UCam.h"

enum
{
	ENVIRONMENT_INITIALIZE = 0x01,
	DAC124S085_POWER_CONTROL = 0x02,
	SENSOR_VIDEOSTREAM_CONTROL = 0x03,
	RESET_DAC_CONTROL = 0x04,
	UPDATE_DAC_CONTROL = 0x05,
	WRITE_DAC_CONTROL = 0x06,
	SENSOR_EXPOSURE_TIME_CONTROL = 0x09,
	SENSOR_GAIN_CONTROL = 0x0A,
	SENSOR_TEMPERATURE_CONTROL = 0x0B,
	SENSOR_LOW_POWER_CONTROL = 0x0C,
	SENSOR_OUTPUT_FRAME_NUMBER_CONTROL = 0x0D,
	SENSOR_LED_POWER_CONTROL = 0x0E,
	SET_LUT_INDEX_CONTROL = 0x0F,
	BUILD_LUT_CONTROL = 0x10,
	PERFORM_LUT_CONTROL = 0x11,
	SAVE_LUT_FLASH_CONTROL = 0x12,
	RESTORE_LUT_FLASH_CONTROL = 0x13,
	SET_ACTD = 0x14,
	COORDINATE_CONTROL = 0x15,
	SERIAL_NUMBER_CONTROL = 0x16,
};

class CameraCardController : public card_interface
{

private:
	void validate_stability();
	void switch_control();
	void switch_control_warm_up();
	void close_switch_control_warm_up();
	void is_up_to_date();
	bool check_connectivity();
	int mi_get_media_fd(int fd, const char *bus_info);
	void lut_thread(int size);
	int find_brightest_band(int* band);
	float get_ROI_maxPixel_rate();
public:
	CameraCardController();
	~CameraCardController();
	int InitController();
	bool IsCameraFound() { return (GetCamIndex() >= 0); };

	//设置指令
	void reset_exp_gain();
	void store_LUT_info();

	int SetExposureFps(unsigned int fps);

	int AutoExposure(int* max_bright_band);
	int GetExposureTime();
	int SetGain(float gainVal);
	float GetGain();
	void Reset();
	int GetCamIndex();
	float GetTemperature();
	void PowerLed(int iState);
	void EnableLowPower(int is_power_low);
	void EnableVideoStream(int is_videostream_on);
	int m_iCamID = -1; // ID of the UNS cameras
					   /// Data
	float exposure_time;
	float gain_value;
	float __max_voltage;
	float real_temp;
	// VOLTAGESDAC converted_voltages;

	// Interface to the Camera picture date
	int InitCam(int iCamNumber = -1);
	bool IsCameraOpend();
	int camClose();
	int TakeFrame(int iMode = 0);
	int MakeWorkingFramesFromFrame0(unsigned char *frame0);
	int getFrame00(unsigned short *frame);
	int GetFrameRaw(unsigned short *frame, bool isTake = true, int position = -1);
	int GetFrame10(unsigned short *frame, bool isTake = true, int position = -1);
	int GetFrame8(unsigned char *frame, bool isTake = true, int position = -1);
	int GetPreview(unsigned char *frame, bool isTake = true);
	int getPicture(void *valMatPic);
	void convert10bto8bX_controller(unsigned short *frame_16, unsigned char *frame_8);
	int waitKeyX(int i);
	void SetFirstLUTFrame(bool bVal);
	void EnableFramesComparison(bool bVal);

	//======================================================================================
	void get_card_id();
	int GetFwVersion();
	int GetApiVersion();
	unsigned long get_property_size(unsigned long pid);
	unsigned char get_property1(unsigned long pid, unsigned long size);
	unsigned short get_property2(unsigned long pid, unsigned long size);
	unsigned int get_property3(unsigned long pid, unsigned long size);
	int get_property3(unsigned long pid, unsigned long size, unsigned char *chVal);
	unsigned long get_property4(unsigned long pid, unsigned long size);

	void Release();
	void SkipFrames(int number_of_frames);
	void connect_controller(int config_data);

	void EnableLutMode(bool lutmode);

	// bool MyCompare(cv::Mat img1, cv::Mat img2);
	int SetLUTNumFrames(int num);
	int SetLUTIndex(int index);
	int GetLUTIndex();
	int SetLUTLine(unsigned short converted_voltages[4], unsigned short fps, unsigned short gain_value, unsigned short cwl);
	int GetLUTLine(int index, unsigned short *converted_voltages, unsigned short *fps, unsigned short *gain_value, unsigned short *cwl);
	int RunLUT();
	int GetRunLUTCurrentIndex();
	int SaveLUT();
	int RestoreLUT(bool isUpdateVars = true);
	int SetACTD(unsigned short time_ms);
	unsigned short GetACTD();
	int SetCoords(unsigned short *coord_1, unsigned short *coord_2);
	int GetCoords(unsigned short *coord_1, unsigned short *coord_2);
	int SetSerialNumber(unsigned int serial_number);
	unsigned int GetSerialNumber();
	int SetVoltages(int *voltagesVals, bool is_voltages_converted = true);
	// HRESULT is_voltages_valid(int* voltagesVals, const char* message);
	int SetLineVoltages(int index);
	// bool setVoltages(VOLTAGES voltages, bool is_voltages_converted = false);
	// bool is_voltages_valid(VOLTAGES voltages);
	// VOLTAGESDAC convert_voltages(VOLTAGES voltages);
	int DacPower(int is_power_on);
	int DacReset();
	int DacWrite(int index, int voltage);
	int DacUpdate(int index);
	// int get_current_low_power_state();
	// HRESULT CaptureLUT(int iVal);
	int BuildCustomLUT(int *iRelevantIndexes, int iL);
	int CaptureLUT(int iNumFramesForBands, unsigned short *frames);
	void setSave(bool saved = false);
	int low_power_state = 0;
	void Stop();
	void Restart();
	void SetCacheCount(int count);
	int *getIndexForCapture();
};
