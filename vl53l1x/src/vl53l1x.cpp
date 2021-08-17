/*
 * STM VL53L1X ToF rangefinder driver for ROS
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under BSD 3-Clause License (available at https://opensource.org/licenses/BSD-3-Clause).
 *
 * Documentation used:
 * VL53L1X datasheet - https://www.st.com/resource/en/datasheet/vl53l1x.pdf
 * VL53L1X API user manual - https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/98/0d/38/38/5d/84/49/1f/DM00474730/files/DM00474730.pdf/jcr:content/translations/en.DM00474730.pdf
 *
 */

#include <string>
#include <vector>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <vl53l1x/msg/measurement_data.hpp>

#include "vl53l1_api.h"
#include "i2c.h"

#define xSTR(x) #x
#define STR(x) xSTR(x)

#define CHECK_STATUS(func) { \
	VL53L1_Error status = func; \
	if (status != VL53L1_ERROR_NONE) { \
		RCLCPP_WARN(this->get_logger(), "VL53L1X: Error %d on %s", status, STR(func)); \
	} \
}

class VL53L1X_Node: public rclcpp::Node {
	public:
		VL53L1X_Node();

	private:
		void poll();

		sensor_msgs::msg::Range range;
		vl53l1x::msg::MeasurementData data;

		int mode, i2c_bus, i2c_address;
		double poll_rate, timing_budget, offset;
		bool ignore_range_status;
		std::vector<int64_t> pass_statuses { VL53L1_RANGESTATUS_RANGE_VALID,
										VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL,
										VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE };

		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub;
		rclcpp::Publisher<vl53l1x::msg::MeasurementData>::SharedPtr data_pub;

		rclcpp::TimerBase::SharedPtr timer;

		VL53L1_Dev_t dev;
		VL53L1_Error dev_error;
};

VL53L1X_Node::VL53L1X_Node() :
	Node("vl53l1X_rangefinder",
		 "",
		 rclcpp::NodeOptions()
			.allow_undeclared_parameters(true)
			.automatically_declare_parameters_from_overrides(true)
	)
{
	this->get_parameter_or("mode", this->mode, 3);
	this->get_parameter_or("i2c_bus", this->i2c_bus, 1);
	this->get_parameter_or("i2c_address", this->i2c_address, 0x29);
	this->get_parameter_or("poll_rate", this->poll_rate, 100.0);
	this->get_parameter_or("ignore_range_status", this->ignore_range_status, false);
	this->get_parameter_or("timing_budget", this->timing_budget, 0.1);
	this->get_parameter_or("offset", this->offset, 0.0);
	this->get_parameter_or("frame_id", this->range.header.frame_id, std::string(""));
	this->get_parameter_or("field_of_view", this->range.field_of_view, 0.471239f); // 27 deg, source: datasheet
	this->get_parameter_or("min_range", this->range.min_range, 0.0f);
	this->get_parameter_or("max_range", this->range.max_range, 4.0f);
	try {
		this->pass_statuses = this->get_parameter("pass_statuses").as_integer_array();
	} catch (const rclcpp::ParameterTypeException& e) {
		RCLCPP_INFO(this->get_logger(), "Pass Statuses set as default.");
	}

	this->range_pub = this->create_publisher<sensor_msgs::msg::Range>("range", 20);
	this->data_pub = this->create_publisher<vl53l1x::msg::MeasurementData>("range_data", 20);

	if (this->timing_budget < 0.02 || this->timing_budget > 1) {
		RCLCPP_FATAL(this->get_logger(), "Error: timing_budget should be within 0.02 and 1 s (%g is set)", this->timing_budget);
		rclcpp::shutdown();
	}

	this->range.radiation_type = sensor_msgs::msg::Range::INFRARED;
	// 	// The minimum inter-measurement period must be longer than the timing budget + 4 ms (*)
	double inter_measurement_period = this->timing_budget + 0.004;

	// Setup I2C bus
	i2c_setup(this->i2c_bus, this->i2c_address);

	// Init sensor
	// VL53L1_Dev_t dev;
	// VL53L1_Error dev_error;
	VL53L1_software_reset(&this->dev);
	VL53L1_WaitDeviceBooted(&this->dev);
	VL53L1_DataInit(&this->dev);
	VL53L1_StaticInit(&this->dev);
	VL53L1_SetPresetMode(&this->dev, VL53L1_PRESETMODE_AUTONOMOUS);

	// Print device info
	VL53L1_DeviceInfo_t device_info;
	CHECK_STATUS(VL53L1_GetDeviceInfo(&this->dev, &device_info));
	RCLCPP_INFO(this->get_logger(), "VL53L1X: Device name: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.Name);
	RCLCPP_INFO(this->get_logger(), "VL53L1X: Device type: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.Type);
	RCLCPP_INFO(this->get_logger(), "VL53L1X: Product ID: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.ProductId);
	RCLCPP_INFO(this->get_logger(), "VL53L1X: Type: %u Version: %u.%u", device_info.ProductType,
	          device_info.ProductRevisionMajor, device_info.ProductRevisionMinor);

	// Setup sensor
	CHECK_STATUS(VL53L1_SetDistanceMode(&this->dev, mode));
	CHECK_STATUS(VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, round(timing_budget * 1e6)));

	double min_signal;
	if (this->get_parameter("min_signal", min_signal)) {
		CHECK_STATUS(VL53L1_SetLimitCheckValue(&this->dev, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, min_signal * 65536));
	}

	double max_sigma;
	if (this->get_parameter("max_sigma", max_sigma)) {
		CHECK_STATUS(VL53L1_SetLimitCheckValue(&this->dev, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, max_sigma * 1000 * 65536));
	}

	// Start sensor
	for (int i = 0; i < 100; i++) {
		CHECK_STATUS(VL53L1_SetInterMeasurementPeriodMilliSeconds(&this->dev, round(inter_measurement_period * 1e3)));
		this->dev_error = VL53L1_StartMeasurement(&this->dev);
		if (this->dev_error == VL53L1_ERROR_INVALID_PARAMS) {
			inter_measurement_period += 0.001; // Increase inter_measurement_period to satisfy condition (*)
		} else break;
	}

	// Check for errors after start
	if (this->dev_error != VL53L1_ERROR_NONE) {
		RCLCPP_FATAL(this->get_logger(), "VL53L1X: Can't start measurement: error %d", this->dev_error);
		rclcpp::shutdown();
	}

	RCLCPP_INFO(this->get_logger(), "VL53L1X: ranging");

	this->timer = this->create_wall_timer(
		std::chrono::duration<double>(this->poll_rate),
		std::bind(&VL53L1X_Node::poll, this)
	);

}

void VL53L1X_Node::poll() {
	VL53L1_RangingMeasurementData_t measurement_data;
	this->range.header.stamp = this->now();

	// Check the data is ready
	uint8_t data_ready = 0;
	VL53L1_GetMeasurementDataReady(&this->dev, &data_ready);
	if (!data_ready) {
		return;
	}

	// Read measurement
	VL53L1_GetRangingMeasurementData(&this->dev, &measurement_data);
	VL53L1_ClearInterruptAndStartMeasurement(&this->dev);

	// Publish measurement data
	this->data.header.stamp = this->range.header.stamp;
	this->data.signal = measurement_data.SignalRateRtnMegaCps / 65536.0;
	this->data.ambient = measurement_data.AmbientRateRtnMegaCps / 65536.0;
	this->data.effective_spad = measurement_data.EffectiveSpadRtnCount / 256;
	this->data.sigma = measurement_data.SigmaMilliMeter / 65536.0 / 1000.0;
	this->data.status = measurement_data.RangeStatus;
	this->data_pub->publish(this->data);

	// Check measurement for validness
	if (!this->ignore_range_status &&
		std::find(this->pass_statuses.begin(), this->pass_statuses.end(), measurement_data.RangeStatus) == this->pass_statuses.end()) {
		char range_status[VL53L1_MAX_STRING_LENGTH];
		VL53L1_get_range_status_string(measurement_data.RangeStatus, range_status);
		RCLCPP_DEBUG(this->get_logger(), "Range measurement status is not valid: %s", range_status);
	} else {
		// Publish measurement
		this->range.range = measurement_data.RangeMilliMeter / 1000.0 + this->offset;
		this->range_pub->publish(this->range);
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VL53L1X_Node>());
	rclcpp::shutdown();
	return 0;
}
