#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/internal_resistance.h>

#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

class InternalRes : public ModuleBase<InternalRes>, public ModuleParams, public px4::WorkItem
{
public:
	InternalRes();
	~InternalRes() {};

	//typedef matrix::Vector<float, 4> Vector4f;

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;



private:
	void Run() override;

	internal_resistance_s inter_res;
	battery_status_s battery_status;

	void update_parameter_vector(float voltage_prediction_error);

	float update_voltage_prediction(hrt_abstime sampling_period);

	hrt_abstime battery_time_prev;
	hrt_abstime internal_res_time_prev;


	float p_0 = 1;
	float p_1 = (0.1 +0.05)/(5*0.05);
	float p_2 = 1/(5*0.05);
	float p_3 = 12.6/(5*0.05);

	struct battery_parameters
	{
	float r_steady_state;
	float r_transient;
	float capacitance;
	float voltage_open_circuit;
	};


	//struct battery_parameters extract_parameters();

	float voltage_b_prediction_prev;
	float current_a_prev;


	uORB::SubscriptionCallbackWorkItem _battery_sub{this, ORB_ID(battery_status)};

	uORB::Publication<internal_resistance_s> _internal_res_pub{ORB_ID(internal_resistance)};

	DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BAT1_R_INTERNAL>) _bat1_r_internal,
	(ParamInt<px4::params::INTER_RES_EN>) _inter_res_en
    	)

};
