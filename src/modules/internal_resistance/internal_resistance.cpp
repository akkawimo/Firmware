#include "internal_resistance.hpp"

#include<math.h>

InternalRes::InternalRes() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

bool InternalRes::init()
{
	if (!_battery_sub.registerCallback()) {
		PX4_ERR("battery status callback registration failed!");
		return false;
	}

	voltage_b_prediction_prev = 10; //TODO find good correct inital esimate
	current_a_prev = battery_status.current_a;;

	battery_time_prev = battery_status.timestamp;
	internal_res_time_prev = hrt_absolute_time();

	return true;
}

void InternalRes::Run()
{

	if (should_exit()) {
		_battery_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_battery_sub.update(&battery_status)) {

		const hrt_abstime time_now = hrt_absolute_time();

		hrt_abstime call_back_sampling_period = time_now - internal_res_time_prev;

		hrt_abstime battery_sampling_period = battery_status.timestamp - battery_time_prev;

		float s_0 = -(battery_status.current_a - current_a_prev)/(battery_sampling_period/1000000); //delta current/delta time
		float s_1 = -battery_status.current_a;
		float s_2 = -voltage_b_prediction_prev;
		float s_3 = 1.0f; //TODO only set this one time

		//Apply discretized thevenin battery model (first order system)

		p_2 = -p_2;

		float a_d =  exp (p_2*(battery_sampling_period/1000000));

		float b_d_0 = ((exp (p_2*battery_sampling_period/1000000)/p_2) - 1/p_2) * p_0;
		float b_d_1 = ((exp (p_2*battery_sampling_period/1000000)/p_2) - 1/p_2) * p_1;
		float b_d_2 = ((exp (p_2*battery_sampling_period/1000000)/p_2) - 1/p_2) * p_3;


		float voltage_prediction_error = battery_status.voltage_v + s_2;

		float voltage_b_prediction = a_d*s_2 + b_d_0*s_0 + b_d_1*s_1 + b_d_2*s_3  + 0.4f*voltage_prediction_error;

		p_2 = -p_2;

		//end of voltage prediction

		//TODO tune

		float adaptation_gain_0 = 2;
		float adaptation_gain_1 = 2;
		float adaptation_gain_2 = 2;
		float adaptation_gain_3 = 1;

		//PX4_INFO("in update_parameter_vector");

		// parameter_vector_estimate_updated
		p_0 += adaptation_gain_0*voltage_prediction_error*s_0;
		p_1 += adaptation_gain_1*voltage_prediction_error*s_1;
		p_2 += adaptation_gain_2*voltage_prediction_error*s_2;
		p_3 += adaptation_gain_3*voltage_prediction_error*s_3;

		//extract_parameters

		battery_parameters bp;

		bp.r_steady_state = p_0;
		bp.r_transient = (p_1/p_2) - p_0;
		bp.voltage_open_circuit = p_3/p_2;

		//Set px4 Internal resistance using simplified Rint Model
		inter_res.timestamp = hrt_absolute_time();
		inter_res.voltage_prediction_error = voltage_prediction_error;
		inter_res.r_transient = bp.r_transient;
		inter_res.r_steady_state = bp.r_steady_state;
		inter_res.voltage_open_circuit = bp.voltage_open_circuit;

		//calculate bat1_r_internal

		inter_res.bat1_r_internal = (battery_status.voltage_v - bp.voltage_open_circuit) / (-battery_status.current_a);

		//tmp for debugging

		inter_res.current = battery_status.current_a;
		inter_res.voltage = battery_status.voltage_v;
		inter_res.voltage_prediction = voltage_b_prediction;
		inter_res.current_dot = battery_status.current_a - current_a_prev;
		inter_res.current_prev = current_a_prev;
		inter_res.battery_sampling_period = battery_sampling_period;
		inter_res.call_back_sampling_period = call_back_sampling_period;

		inter_res.p_0 = p_0;
		inter_res.p_1 = p_1;
		inter_res.p_2 = p_2;
		inter_res.p_3 = p_3;

		inter_res.s_0 = s_0;
		inter_res.s_1 = s_1;
		inter_res.s_2 = s_2;
		inter_res.s_3 = s_3;

		inter_res.a_d = a_d;
		inter_res.b_d_0 = b_d_0;
		inter_res.b_d_1 = b_d_1;
		inter_res.b_d_2 = b_d_2;



		_internal_res_pub.publish(inter_res);

		//store current values for next iteration
		voltage_b_prediction_prev = voltage_b_prediction;
		current_a_prev = battery_status.current_a;


		// commit value only periodically and if voltage_prediction_error is within some range

		//TODO increment the value.

		//std::max(std::min(inter_res.bat1_r_internal, 0.1), -1.0);

		//_bat1_r_internal.set(inter_res.bat1_r_internal);
		//_bat1_r_internal.commit();


		battery_time_prev = battery_status.timestamp;

		internal_res_time_prev = time_now;



	}

}


int InternalRes::task_spawn(int argc, char *argv[])
{
	InternalRes *instance = new InternalRes();

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

int InternalRes::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int InternalRes::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int InternalRes::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
module to PX4 which uses ​ only the current and voltage data​ to periodically re-estimate the
battery internal resistance. The estimate should then be written to the BAT1_R_INTERNAL
parameter.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("internal_resistance", "custom_module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int internal_resistance_main(int argc, char *argv[])
{
	return InternalRes::main(argc, argv);
}
