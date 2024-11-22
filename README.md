# ADS122U04 ADC ROS Driver

This repository contains a simple ROS driver for the **ADS122U04** ADC, derived from [this repository](https://github.com/yasir-shahzad/ADS122U04_ADC_Arduino_Library/tree/master).

## Configuration

The driver can be configured through the `config/adc.yaml` file, which contains the following parameters:

- **portname**:  
  Default: `/dev/adc`  
  The serial device representing the ADC.

- **baudrate**:  
  Default: `115200`  
  The serial baud rate for communication with the ADC.

- **reference_voltage**:  
  Default: `5.0`  
  The actual voltage of the ADC's external voltage reference. This should be measured using a trusted precision voltmeter.

## Running the Node

To launch the driver, use the provided `adc.launch` file. The driver will output the measured data at **20Hz** on the following topics:

- **`/UAV_NAME/adc/voltage`**:  
  Type: `Float64`  
  Description: The measured voltage in volts.

- **`/UAV_NAME/adc/raw_measurement`**:  
  Type: `Uint32`  
  Description: The raw ADC measured value.

## Over-voltage Handling

If the ADC experiences an over-voltage condition (when the measured voltage exceeds the reference voltage), it may continue outputting the maximum possible value even after the over-voltage condition is cleared.

To address this, a reset routine has been implemented. When an over-voltage condition is detected (i.e., the ADC measures the maximum possible value), the ADC will continuously reset until the over-voltage condition is cleared.


## Advanced tuning
To select a different ADC channel, or configure different data rates or other ADC parameters (gain, Turbo mode, PGA etc.), you have to modify the source code of the node.
