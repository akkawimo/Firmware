#include "integrated_accel.hpp"

IntegratedAccel::IntegratedAccel() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

bool IntegratedAccel::init()
{
	if (!_sensors_sub.registerCallback()) {
		PX4_ERR("sensor combined callback registration failed!");
		return false;
	}
	return true;
}

void IntegratedAccel::Run()
{

	if (should_exit()) {
		_sensors_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_status_sub.update(&vehicle_status)) {
		armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	if (_sensors_sub.update(&sensors) && armed) {

		integ_acc.timestamp = hrt_absolute_time();

		integ_acc.vel_m_s[0] += sensors.accelerometer_m_s2[0] * sensors.accelerometer_integral_dt * 1.e-6f;
		integ_acc.vel_m_s[1] += sensors.accelerometer_m_s2[1] * sensors.accelerometer_integral_dt * 1.e-6f;
		integ_acc.vel_m_s[2] += sensors.accelerometer_m_s2[2] * sensors.accelerometer_integral_dt * 1.e-6f;

		_vel_pub.publish(integ_acc);
	}
}


int IntegratedAccel::task_spawn(int argc, char *argv[])
{
	IntegratedAccel *instance = new IntegratedAccel();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int IntegratedAccel::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int IntegratedAccel::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int IntegratedAccel::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
while the aircraft is **armed**, integrates all of the accelerometer data over time used by
**EKF2** and publishes the integrated x, y, z velocity estimates to `integrated_accel`.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("integrate_accel", "custom_module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int integrated_accel_main(int argc, char *argv[])
{
	return IntegratedAccel::main(argc, argv);
}
