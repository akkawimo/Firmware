#pragma once

#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/integrated_accel.h>
#include <uORB/topics/vehicle_status.h>


class IntegratedAccel : public ModuleBase<IntegratedAccel>, public ModuleParams, public px4::WorkItem
{
public:
	IntegratedAccel();
	~IntegratedAccel() {};

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	bool armed = false;

	vehicle_status_s vehicle_status;
	sensor_combined_s sensors;
	integrated_accel_s integ_acc;

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionCallbackWorkItem _sensors_sub{this, ORB_ID(sensor_combined)};

	uORB::Publication<integrated_accel_s> _vel_pub{ORB_ID(integrated_accel)};

};
