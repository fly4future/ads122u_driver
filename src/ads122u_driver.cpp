/* Derived from https://github.com/yasir-shahzad/ADS122U04_ADC_Arduino_Library/tree/master */

/* includes //{ */
#include <chrono>
#include <thread>
/* each ros package must have these */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>
//}

#include <serial_port.h>


/* defines //{ */

#define ADS122U04_CONVERSION_TIMEOUT 75

// Define 2/3/4-Wire, Temperature and Raw modes
#define ADS122U04_4WIRE_MODE 0x0
#define ADS122U04_3WIRE_MODE 0x1
#define ADS122U04_2WIRE_MODE 0x2
#define ADS122U04_TEMPERATURE_MODE 0x3
#define ADS122U04_RAW_MODE 0x4
#define ADS122U04_4WIRE_HI_TEMP 0x5
#define ADS122U04_3WIRE_HI_TEMP 0x6
#define ADS122U04_2WIRE_HI_TEMP 0x7

// ADS122U04 Table 16 in Datasheet
#define ADS122U04_SYNC_HEAD 0x55      // Sync Header
#define ADS122U04_RESET_CMD 0x06      // 0000 011x      Reset
#define ADS122U04_START_CMD 0x08      // 0000 100x      Start/Sync
#define ADS122U04_POWERDOWN_CMD 0x02  // 0000 001x      PowerDown
#define ADS122U04_RDATA_CMD 0x10      // 0001 xxxx      RDATA
#define ADS122U04_RREG_CMD 0x20       // 0010 rrxx      Read REG rr= register address 00 to 11
#define ADS122U04_WREG_CMD 0x40       // 0100 rrxx      Write REG rr= register address 00 to 11

// #define ADS122U04_WRITE_CMD(reg)     (ADS122U04_WREG_CMD | (reg << 2))    //Shift is 2-bit in ADS122U04
#define ADS122U04_WRITE_CMD(reg) (ADS122U04_WREG_CMD | (reg << 1))  // Shift is 2-bit in ADS122U04
#define ADS122U04_READ_CMD(reg) (ADS122U04_RREG_CMD | (reg << 2))   // Shift is 2-bit in ADS122U04

// ADS122U04 Table 16 in Datasheet
#define ADS122U04_CONFIG_0_REG 0  // Configuration Register 0
#define ADS122U04_CONFIG_1_REG 1  // Configuration Register 1
#define ADS122U04_CONFIG_2_REG 2  // Configuration Register 2
#define ADS122U04_CONFIG_3_REG 3  // Configuration Register 3
#define ADS122U04_CONFIG_4_REG 4  // Configuration Register 4

// Unshifted register definitions
// The bit field register definitions will do the bit shifting

// Configuration Register 0
// ADS122U04 Table 19 in Datasheet

// Input Multiplexer Configuration
#define ADS122U04_MUX_AIN0_AIN1 0x0
#define ADS122U04_MUX_AIN0_AIN2 0x1
#define ADS122U04_MUX_AIN0_AIN3 0x2
#define ADS122U04_MUX_AIN1_AIN0 0x3
#define ADS122U04_MUX_AIN1_AIN2 0x4
#define ADS122U04_MUX_AIN1_AIN3 0x5
#define ADS122U04_MUX_AIN2_AIN3 0x6
#define ADS122U04_MUX_AIN3_AIN2 0x7
#define ADS122U04_MUX_AIN0_AVSS 0x8
#define ADS122U04_MUX_AIN1_AVSS 0x9
#define ADS122U04_MUX_AIN2_AVSS 0xa
#define ADS122U04_MUX_AIN3_AVSS 0xb
#define ADS122U04_MUX_REFPmREFN 0xc
#define ADS122U04_MUX_AVDDmAVSS 0xd
#define ADS122U04_MUX_SHORTED 0xe

// Gain Configuration
#define ADS122U04_GAIN_1 0x0
#define ADS122U04_GAIN_2 0x1
#define ADS122U04_GAIN_4 0x2
#define ADS122U04_GAIN_8 0x3
#define ADS122U04_GAIN_16 0x4
#define ADS122U04_GAIN_32 0x5
#define ADS122U04_GAIN_64 0x6
#define ADS122U04_GAIN_128 0x7

// PGA Bypass (PGA is disabled when the PGA_BYPASS bit is set)
#define ADS122U04_PGA_DISABLED 0x1
#define ADS122U04_PGA_ENABLED 0x0

// Configuration Register 1
// ADS122U04 Table 19 in Datasheet

// Data Rate
// Turbo mode = Normal mode * 2 (Samples per Second)
// Normal mode
#define ADS122U04_DATA_RATE_20SPS 0x0
#define ADS122U04_DATA_RATE_45SPS 0x1
#define ADS122U04_DATA_RATE_90SPS 0x2
#define ADS122U04_DATA_RATE_175SPS 0x3
#define ADS122U04_DATA_RATE_330SPS 0x4
#define ADS122U04_DATA_RATE_600SPS 0x5
#define ADS122U04_DATA_RATE_1000SPS 0x6

// Operating Mode
#define ADS122U04_OP_MODE_NORMAL 0x0
#define ADS122U04_OP_MODE_TURBO 0x1

// Conversion Mode
#define ADS122U04_CONVERSION_MODE_SINGLE_SHOT 0x0
#define ADS122U04_CONVERSION_MODE_CONTINUOUS 0x1

// Voltage Reference Selection
#define ADS122U04_VREF_INTERNAL 0x0      // 2.048V internal
#define ADS122U04_VREF_EXT_REF_PINS 0x1  // REFp and REFn external
#define ADS122U04_VREF_AVDD 0x2          // Analog Supply AVDD and AVSS
#define ADS122U04_VREF_AVDD1 0x3         // Analog Supply AVDD and AVSS

// Temperature Sensor Mode
#define ADS122U04_TEMP_SENSOR_OFF 0x0
#define ADS122U04_TEMP_SENSOR_ON 0x1

// Configuration Register 2
// ADS122U04 Table 22 in Datasheet

// Conversion Result Ready Flag (READ ONLY)

// Data Counter Enable
#define ADS122U04_DCNT_DISABLE 0x0
#define ADS122U04_DCNT_ENABLE 0x1

// Data Integrity Check Enable
#define ADS122U04_CRC_DISABLED 0x0
#define ADS122U04_CRC_INVERTED 0x1
#define ADS122U04_CRC_CRC16_ENABLED 0x2

// Burn-Out Current Source
#define ADS122U04_BURN_OUT_CURRENT_OFF 0x0
#define ADS122U04_BURN_OUT_CURRENT_ON 0x1

// IDAC Current Setting
#define ADS122U04_IDAC_CURRENT_OFF 0x0
#define ADS122U04_IDAC_CURRENT_10_UA 0x1
#define ADS122U04_IDAC_CURRENT_50_UA 0x2
#define ADS122U04_IDAC_CURRENT_100_UA 0x3
#define ADS122U04_IDAC_CURRENT_250_UA 0x4
#define ADS122U04_IDAC_CURRENT_500_UA 0x5
#define ADS122U04_IDAC_CURRENT_1000_UA 0x6
#define ADS122U04_IDAC_CURRENT_1500_UA 0x7

// Configuration Register 3
// ADS122U04 Table 23 in Datasheet

// IDAC1 Routing Configuration
#define ADS122U04_IDAC1_DISABLED 0x0
#define ADS122U04_IDAC1_AIN0 0x1
#define ADS122U04_IDAC1_AIN1 0x2
#define ADS122U04_IDAC1_AIN2 0x3
#define ADS122U04_IDAC1_AIN3 0x4
#define ADS122U04_IDAC1_REFP 0x5
#define ADS122U04_IDAC1_REFN 0x6

// IDAC2 Routing Configuration
#define ADS122U04_IDAC2_DISABLED 0x0
#define ADS122U04_IDAC2_AIN0 0x1
#define ADS122U04_IDAC2_AIN1 0x2
#define ADS122U04_IDAC2_AIN2 0x3
#define ADS122U04_IDAC2_AIN3 0x4
#define ADS122U04_IDAC2_REFP 0x5
#define ADS122U04_IDAC2_REFN 0x6

typedef struct
{
  uint8_t inputMux;
  uint8_t gainLevel;
  uint8_t pgaBypass;
  uint8_t dataRate;
  uint8_t opMode;
  uint8_t convMode;
  uint8_t selectVref;
  uint8_t tempSensorEn;
  uint8_t dataReadyEn;
  uint8_t dataCounterEn;
  uint8_t dataCRCen;
  uint8_t burnOutEn;
  uint8_t idacCurrent;
  uint8_t routeIDAC1;
  uint8_t routeIDAC2;
  uint8_t outputMODE;
  uint8_t gpio2DIR;
  uint8_t gpio1DIR;
  uint8_t gpio0DIR;
  uint8_t gpio2SEL;
  uint8_t gpio2DAT;
  uint8_t gpio1DAT;
  uint8_t gpio0DAT;
} ADS122U04_initParam;

struct CONFIG_REG_0
{
  uint8_t PGA_BYPASS : 1;  // 0
  uint8_t GAIN : 3;        // 1-3
  uint8_t MUX : 4;         // 4-7
};
union CONFIG_REG_0_U
{
  uint8_t             all;
  struct CONFIG_REG_0 bit;
};

//--------------Address 0x01---------------------------------
struct CONFIG_REG_1
{
  uint8_t TS : 1;     // 0
  uint8_t VREF : 2;   // 1-2
  uint8_t CMBIT : 1;  // 3
  uint8_t MODE : 1;   // 4
  uint8_t DR : 3;     // 5-7
};
union CONFIG_REG_1_U
{
  uint8_t             all;
  struct CONFIG_REG_1 bit;
};

//--------------Address 0x02---------------------------------
struct CONFIG_REG_2
{
  uint8_t IDAC : 3;     // 0-2
  uint8_t BCS : 1;      // 3
  uint8_t CRCbits : 2;  // 4-5
  uint8_t DCNT : 1;     // 6
  uint8_t DRDY : 1;     // 7
};
union CONFIG_REG_2_U
{
  uint8_t             all;
  struct CONFIG_REG_2 bit;
};

//--------------Address 0x03---------------------------------
struct CONFIG_REG_3
{
  uint8_t RESERVED : 1;  // 0-1
  uint8_t AUTO : 1;      // 0-1
  uint8_t I2MUX : 3;     // 2-4
  uint8_t I1MUX : 3;     // 5-7
};
union CONFIG_REG_3_U
{
  uint8_t             all;
  struct CONFIG_REG_3 bit;
};


//--------------Address 0x04---------------------------------
struct CONFIG_REG_4
{
  uint8_t GPIO0DAT : 1;  // 0-1
  uint8_t GPIO1DAT : 1;  // 0-1
  uint8_t GPIO2DAT : 1;  // 0-1
  uint8_t GPIO2SEL : 1;  // 0-1
  uint8_t GPIO0DIR : 1;  // 0-1
  uint8_t GPIO1DIR : 1;  // 0-1
  uint8_t GPIO2DIR : 1;  // 0-1
  uint8_t RESERVED : 1;  // 2-4
};
union CONFIG_REG_4_U
{
  uint8_t             all;
  struct CONFIG_REG_4 bit;
};

// All four registers
typedef struct ADS122U04Reg
{
  union CONFIG_REG_0_U reg0;
  union CONFIG_REG_1_U reg1;
  union CONFIG_REG_2_U reg2;
  union CONFIG_REG_3_U reg3;
  union CONFIG_REG_4_U reg4;
} ADS122U04Reg_t;

//}

namespace ads122u_driver
{

/* class Ads122uDriver //{ */

class Ads122uDriver : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  // | -------------------------- flags ------------------------- |

  std::string             portname_ = "test";
  int                     baudrate_ = 115200;
  serial_port::SerialPort serial_port_;

  bool              is_connected_   = false;
  std::atomic<bool> is_initialized_ = false;

  int serial_buffer_size_ = 256;

  ros::Time last_received_ = ros::Time::now();
  uint8_t   connectToSensor();
  bool      reset();
  bool      send_command(int8_t command);
  bool      configureADCmode(uint8_t wire_mode, uint8_t rate);
  bool      start();
  bool      ADS122U04_init(ADS122U04_initParam* param);
  bool      ADS122U04_writeReg(uint8_t reg, uint8_t writeValue);
  bool      ADS122U04_sendCommandWithValue(uint8_t command, uint8_t value);
  bool      getConversionData(uint32_t* conversionData);

  uint8_t        _wireMode = ADS122U04_RAW_MODE;
  ADS122U04Reg_t ADS122U04_Reg;

  ros::Timer main_timer_;
  void       callbackMainTimer([[maybe_unused]] const ros::TimerEvent& te);

  // | ---------------------- ROS service servers --------------------- |

  bool               callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_;

  // | ---------------------- ROS publishers --------------------- |
  ros::Publisher pub_raw_;
  ros::Publisher pub_voltage_;
};
//}

/* onInit() //{ */

void Ads122uDriver::onInit() {

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  nh.param("portname", portname_, std::string("/dev/ttyUSB0"));
  nh.param("baudrate", baudrate_, 115200);

  // | -------- initialize a publisher for UAV reference -------- |
  pub_raw_ = nh.advertise<std_msgs::UInt32>("raw_out", 1);
  pub_voltage_ = nh.advertise<std_msgs::Float64>("voltage_out", 1);

  // | -- initialize the main timer - main loop of the nodelet -- |
  main_timer_ = nh.createTimer(ros::Rate(20), &Ads122uDriver::callbackMainTimer, this);

  // | ---------------- initialize service server --------------- |
  srv_server_start_ = nh.advertiseService("start", &Ads122uDriver::callbackStart, this);

  connectToSensor();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  configureADCmode(ADS122U04_RAW_MODE, 20);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  start();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  is_initialized_ = true;
  ROS_INFO("[Ads122uDriver]: Initialized");
}

//}

/* callbackMainTimer() //{ */

void Ads122uDriver::callbackMainTimer([[maybe_unused]] const ros::TimerEvent& te) {

  uint32_t raw_data;

  if (!getConversionData(&raw_data)) {
    ROS_ERROR("[Ads122uDriver]: Could not read raw data");
    return;
  }

  int32_t raw_data_signed = int32_t(raw_data);
  double  voltage         = (double(raw_data_signed) / (16777215.0)) * 10;
  
  std_msgs::UInt32 raw_msg;
  std_msgs::Float64 voltage_msg;

  raw_msg.data = raw_data;
  voltage_msg.data = voltage;

  pub_raw_.publish(raw_msg);
  pub_voltage_.publish(voltage_msg);

  ROS_INFO_STREAM("Volts: " << std::to_string(voltage) << " raw: " << std::to_string(raw_data) << " raw signed: " << std::to_string(raw_data_signed));
}

//}

/* callbackStart() //{ */

bool Ads122uDriver::callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("[Ads122uDriver]: callback start");
  res.success = true;
  res.message = "Starting waypoint following.";

  return true;
}

//}

/*  configureADCmode() //{ */

bool Ads122uDriver::configureADCmode(uint8_t wire_mode, uint8_t rate) {
  ADS122U04_initParam initParams;  // Storage for the chip parameters

  if (wire_mode == ADS122U04_RAW_MODE)  // Raw mode : disable the IDAC and use the internal reference
  {
    initParams.inputMux      = ADS122U04_MUX_AIN0_AVSS;
    initParams.gainLevel     = ADS122U04_GAIN_1;
    initParams.pgaBypass     = ADS122U04_PGA_DISABLED;
    initParams.dataRate      = rate;                                   // Set the data rate (samples per second). Defaults to 20
    initParams.opMode        = ADS122U04_OP_MODE_NORMAL;               // Disable turbo mode
    initParams.convMode      = ADS122U04_CONVERSION_MODE_SINGLE_SHOT;  // Use single shot mode
    initParams.selectVref    = ADS122U04_VREF_EXT_REF_PINS;
    initParams.tempSensorEn  = ADS122U04_TEMP_SENSOR_OFF;  // Disable the temperature sensor
    initParams.dataReadyEn   = 0b0;
    initParams.dataCounterEn = ADS122U04_DCNT_DISABLE;          // Disable the data counter
    initParams.dataCRCen     = ADS122U04_CRC_DISABLED;          // Disable CRC checking
    initParams.burnOutEn     = ADS122U04_BURN_OUT_CURRENT_OFF;  // Disable the burn-out current
    initParams.idacCurrent   = ADS122U04_IDAC_CURRENT_OFF;      // Disable the IDAC current
    initParams.routeIDAC1    = ADS122U04_IDAC1_DISABLED;        // Disable IDAC1
    initParams.routeIDAC2    = ADS122U04_IDAC2_DISABLED;        // Disable IDAC2
    initParams.outputMODE    = 0b0;
    initParams.gpio2DIR      = 0b1;
    initParams.gpio1DIR      = 0b0;
    initParams.gpio0DIR      = 0b0;
    initParams.gpio2SEL      = 0b1;
    initParams.gpio2DAT      = 0b0;
    initParams.gpio1DAT      = 0b0;
    initParams.gpio0DAT      = 0b0;
    _wireMode                = ADS122U04_RAW_MODE;  // Update the wire mode
  } else {
    ROS_ERROR("[Ads122uDriver]: configureADCmode: unknown mode");
    return (false);
  }
  return (ADS122U04_init(&initParams));  // Configure the chip
}

//}

/* ADS122U04_init //{ */

bool Ads122uDriver::ADS122U04_init(ADS122U04_initParam* param) {

  ADS122U04_Reg.reg0.all = 0;  // Reset all five register values to the default value of 0x00
  ADS122U04_Reg.reg1.all = 0;
  ADS122U04_Reg.reg2.all = 0;
  ADS122U04_Reg.reg3.all = 0;
  ADS122U04_Reg.reg4.all = 0;

  ADS122U04_Reg.reg0.bit.MUX        = param->inputMux;
  ADS122U04_Reg.reg0.bit.GAIN       = param->gainLevel;
  ADS122U04_Reg.reg0.bit.PGA_BYPASS = param->pgaBypass;

  ADS122U04_Reg.reg1.bit.DR    = param->dataRate;
  ADS122U04_Reg.reg1.bit.MODE  = param->opMode;
  ADS122U04_Reg.reg1.bit.CMBIT = param->convMode;
  ADS122U04_Reg.reg1.bit.VREF  = param->selectVref;
  ADS122U04_Reg.reg1.bit.TS    = param->tempSensorEn;

  ADS122U04_Reg.reg2.bit.DRDY    = param->dataReadyEn;
  ADS122U04_Reg.reg2.bit.DCNT    = param->dataCounterEn;
  ADS122U04_Reg.reg2.bit.CRCbits = param->dataCRCen;
  ADS122U04_Reg.reg2.bit.BCS     = param->burnOutEn;
  ADS122U04_Reg.reg2.bit.IDAC    = param->idacCurrent;

  ADS122U04_Reg.reg3.bit.I1MUX = param->routeIDAC1;
  ADS122U04_Reg.reg3.bit.I2MUX = param->routeIDAC2;
  ADS122U04_Reg.reg3.bit.AUTO  = param->outputMODE;

  ADS122U04_Reg.reg4.bit.GPIO2DIR = param->gpio2DIR;
  ADS122U04_Reg.reg4.bit.GPIO1DIR = param->gpio1DIR;
  ADS122U04_Reg.reg4.bit.GPIO0DIR = param->gpio0DIR;
  ADS122U04_Reg.reg4.bit.GPIO2SEL = param->gpio2SEL;
  ADS122U04_Reg.reg4.bit.GPIO2DAT = param->gpio2DAT;
  ADS122U04_Reg.reg4.bit.GPIO1DAT = param->gpio1DAT;
  ADS122U04_Reg.reg4.bit.GPIO0DAT = param->gpio0DAT;

  bool ret_val = true;  // Flag to show if the four writeRegs were successful
  // (If any one writeReg returns false, ret_val will be false)


  ret_val &= ADS122U04_writeReg(ADS122U04_CONFIG_0_REG, ADS122U04_Reg.reg0.all);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ret_val &= ADS122U04_writeReg(ADS122U04_CONFIG_1_REG, ADS122U04_Reg.reg1.all);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ret_val &= ADS122U04_writeReg(ADS122U04_CONFIG_2_REG, ADS122U04_Reg.reg2.all);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ret_val &= ADS122U04_writeReg(ADS122U04_CONFIG_3_REG, ADS122U04_Reg.reg3.all);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ret_val &= ADS122U04_writeReg(ADS122U04_CONFIG_4_REG, ADS122U04_Reg.reg4.all);

  return (ret_val);
}

//}

/*  ADS122U04_writeReg() //{ */

bool Ads122uDriver::ADS122U04_writeReg(uint8_t reg, uint8_t writeValue) {
  uint8_t command = 0;
  command         = ADS122U04_WRITE_CMD(reg);
  return (ADS122U04_sendCommandWithValue(command, writeValue));
}

//}

/*  ADS122U04_sendCommandWithValue() //{ */

bool Ads122uDriver::ADS122U04_sendCommandWithValue(uint8_t command, uint8_t value) {
  serial_port_.sendChar(ADS122U04_SYNC_HEAD);
  serial_port_.sendChar(command);
  serial_port_.sendChar(value);
  return true;
}

//}

/* connectToSensor() //{ */

uint8_t Ads122uDriver::connectToSensor(void) {

  ROS_INFO_THROTTLE(1.0, "[%s]: Openning the serial port.", ros::this_node::getName().c_str());

  if (!serial_port_.connect(portname_, baudrate_)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not connect to sensor.", ros::this_node::getName().c_str());
    is_connected_ = false;
    return 0;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());
  is_connected_  = true;
  last_received_ = ros::Time::now();

  return 1;
}

//}

/* reset() //{ */

bool Ads122uDriver::reset(void) {
  return (send_command(ADS122U04_RESET_CMD));
}

//}

/*  start() //{ */

bool Ads122uDriver::start() {
  return send_command(ADS122U04_START_CMD);
}

//}

/*  send_command() //{ */

bool Ads122uDriver::send_command(int8_t command) {
  serial_port_.sendChar(ADS122U04_SYNC_HEAD);
  serial_port_.sendChar(command);
  return true;
}

//}

/*  getConversionData() //{ */

bool Ads122uDriver::getConversionData(uint32_t* conversionData) {
  uint8_t RXByte[3] = {0};


  serial_port_.sendChar(ADS122U04_SYNC_HEAD);
  serial_port_.sendChar(ADS122U04_RDATA_CMD);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));


  /* ROS_INFO_STREAM("[Ads122uDriver]: read " << bytes_read << " bytes"); */


  uint8_t read_buffer[256];
  int     bytes_read;

  bytes_read = serial_port_.readSerial(read_buffer, serial_buffer_size_);

  if (bytes_read != 3) {
    ROS_WARN_STREAM("[Ads122uDriver]: Incorrect number of bytes read during conversion - skipping. Read: " << bytes_read << " bytes");
    return (false);
  }

  *conversionData = (read_buffer[0] + read_buffer[1] * 256 + read_buffer[2] * 65536);
  return (true);
}

//}

}  // namespace ads122u_driver

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ads122u_driver::Ads122uDriver, nodelet::Nodelet);
