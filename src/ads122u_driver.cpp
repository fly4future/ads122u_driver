/* includes //{ */
#include <chrono>
#include <thread>
/* each ros package must have these */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>
//}

#include <serial_port.h>


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

namespace ads122u_driver
{

/* class Ads122uDriver //{ */

class Ads122uDriver : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  // | -------------------------- flags ------------------------- |

  std::string             portname_ = "/dev/ttyUSB0";
  int                     baudrate_ = 115200;
  serial_port::SerialPort serial_port_;

  bool              is_connected_   = false;
  std::atomic<bool> is_initialized_ = false;

  int serial_buffer_size_ = 1024;

  ros::Time last_received_ = ros::Time::now();
  uint8_t   connectToSensor();

  // | ---------------------- ROS publishers --------------------- |
  /* ros::Publisher pub_reference_; */

  // | ---------------------- ROS timers --------------------- |

  ros::Timer main_timer_;
  void       callbackMainTimer([[maybe_unused]] const ros::TimerEvent& te);

  // | ---------------------- ROS service servers --------------------- |

  bool               callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_;
};
//}

/* onInit() //{ */

void Ads122uDriver::onInit() {

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | -------- initialize a publisher for UAV reference -------- |
  /* pub_reference_ = nh.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1); */

  // | -- initialize the main timer - main loop of the nodelet -- |
  main_timer_ = nh.createTimer(ros::Rate(10), &Ads122uDriver::callbackMainTimer, this);

  // | ---------------- initialize service server --------------- |
  srv_server_start_ = nh.advertiseService("start", &Ads122uDriver::callbackStart, this);

  connectToSensor();


  is_initialized_ = true;
  ROS_INFO("[Ads122uDriver]: Initialized");
}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackMainTimer() //{ */

void Ads122uDriver::callbackMainTimer([[maybe_unused]] const ros::TimerEvent& te) {
  ROS_INFO("[Ads122uDriver]: Main timer spinning");


  serial_port_.sendChar(ADS122U04_SYNC_HEAD);
  serial_port_.sendChar(ADS122U04_RDATA_CMD);

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  uint8_t read_buffer[256];
  int     bytes_read;

  bytes_read = serial_port_.readSerial(read_buffer, serial_buffer_size_);

  ROS_INFO_STREAM("[Ads122uDriver]: read " << bytes_read << " bytes");
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackStart() //{ */

bool Ads122uDriver::callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("[Ads122uDriver]: callback start");
  res.success = true;
  res.message = "Starting waypoint following.";

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

/*  ADS122U04_getConversionData()//{ */

/* void Ads122uDriver::ADS122U04_getConversionData() */
/* { */
/*   uint8_t RXByte[3] = {0}; */

/*   _serialPort->write(ADS122U04_SYNC_HEAD); */
/*   _serialPort->write(ADS122U04_RDATA_CMD); */
/*   delay(10); */

/*   // Note: the next line will need to be changed if data integrity is enabled. */
/*   //       The code will need to request 5 bytes for CRC or 6 bytes for inverted data. */

/*   if (_serialPort->available() >= 3) */
/*   { */
/*     RXByte[0] = _serialPort->read(); // MSB */
/*       delayMicroseconds(10); */
/*     RXByte[1] = _serialPort->read(); */
/*       delayMicroseconds(10); */
/*     RXByte[2] = _serialPort->read(); // LSB */
/*     if (_serialPort->available() > 0) // Note: this _should_ be redundant */
/*     { */
/*       if (_printDebug == true) */
/*       { */
/*         ROS_ERROR("ADS122U04_getConversionData: excess bytes available. Maybe data integrity is enabled?"); */
/*       } */
/*       while (_serialPort->available() > 0) */
/*       { */
/*         _serialPort->read(); // Read and ignore excess bytes (presumably inverted data or CRC) */
/*       } */
/*     } */
/*   } */
/*   else */
/*   { */
/*     if (_printDebug == true) */
/*     { */
/*         ROS_ERROR("ADS122U04_getConversionData: requestFrom failed"); */
/*     } */
/*     return; */
/*   } */


/*   // *conversionData = ((uint32_t)RXByte[2]) | ((uint32_t)RXByte[1]<<8) | ((uint32_t)RXByte[0]<<16); */
/*   uint32_t conversionData =  (RXByte[0] + RXByte[1] * 256 + RXByte[2] * 65536); */


/*   return; */
/* } */

//}


}  // namespace ads122u_driver

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ads122u_driver::Ads122uDriver, nodelet::Nodelet);
