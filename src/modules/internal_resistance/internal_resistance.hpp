#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/internal_resistance.h>

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

	void write_internal_resistance();

	float extract_parameters();

private:
	void Run() override;

	internal_resistance_s inter_res;
	battery_status_s battery_status;

	hrt_abstime battery_time_prev;

	hrt_abstime last_param_write_time;

	float p_0 = 0.0f;
	float p_1 = 0.0f;
	float p_2 = 0.0f;
	float p_3 = 0.0f;

	float adaptation_gain_0 = 0.00001f;
	float adaptation_gain_1 = 0.00001f;
	float adaptation_gain_2 = 0.00001f;
	float adaptation_gain_3 = 0.00001f;

	float voltage_b_prediction_prev = 20.0f;
	float v_dot_prediction_prev = 0;
	float current_a_prev;

	float best_prediction_error;

	bool best_prediction_error_reset = true;

	float best_prediction;

	float bat1_r_internal_prev = -1.0f;

	uORB::SubscriptionCallbackWorkItem _battery_sub{this, ORB_ID(battery_status)};

	uORB::Publication<internal_resistance_s> _internal_res_pub{ORB_ID(internal_resistance)};

	DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BAT1_R_INTERNAL>) _bat1_r_internal,
	(ParamInt<px4::params::INTER_RES_EN>) _inter_res_en,
	(ParamFloat<px4::params::RIN_UPDATE_TIME>) _inter_res_update_period
    	)

};
