#include "internal_resistance.hpp"

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

	return true;
}

void InternalRes::write_internal_resistance() {

	//BAT${i}_R_INTERNAL  increment: 0.01
	best_prediction = round(best_prediction*10)/10;

	//clamp BAT${i}_R_INTERNAL min: -1.0  max: 0.2
	//std::clamp(bat1_r_internal, -1.0, 0.2);
	/*
	if (bat1_r_internal > 0.1f){
	    bat1_r_internal = 0.1f;
	}
	if (bat1_r_internal < -1.0f){
	    bat1_r_internal = -1.0f;
	}
	*/

	//Set px4 Internal resistance using simplified Rint Model
	_bat1_r_internal.set(best_prediction);
	_bat1_r_internal.commit();
	last_param_write_time = hrt_absolute_time();
	best_prediction_error_reset = true;

}

float InternalRes::extract_parameters() {

	float r_steady_state = p_0;
	float r_transient = (p_1/p_2) - p_0;
	float voltage_open_circuit = p_3/p_2;

	//calculate bat1_r_internal
	float internal_resistance = (battery_status.voltage_v - voltage_open_circuit) / (-battery_status.current_a);

	//logging
	inter_res.r_transient = r_transient;
	inter_res.r_steady_state = r_steady_state;
	inter_res.voltage_open_circuit = voltage_open_circuit;

	return internal_resistance;

}



void InternalRes::Run()
{

	if (should_exit()) {
		_battery_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_battery_sub.update(&battery_status)) {

		if (battery_time_prev != 0 && battery_status.timestamp != battery_time_prev) {

		float battery_sampling_period = (battery_status.timestamp - battery_time_prev)/ 1e6f;

		float voltage_b_prediction = v_dot_prediction_prev*battery_sampling_period + voltage_b_prediction_prev;

        	float voltage_prediction_error = battery_status.voltage_v - voltage_b_prediction;

		//process signal
		float s_0 = -(battery_status.current_a - current_a_prev)/(battery_sampling_period);
		float s_1 = -battery_status.current_a;
		float s_2 = -voltage_b_prediction_prev;

		//update parameter vector estimate
		p_0 += adaptation_gain_0*voltage_prediction_error*s_0*(battery_sampling_period);
		p_1 += adaptation_gain_1*voltage_prediction_error*s_1*(battery_sampling_period);
		p_2 += adaptation_gain_2*voltage_prediction_error*s_2*(battery_sampling_period);
		p_3 += adaptation_gain_3*voltage_prediction_error*(battery_sampling_period);

		//Apply thevenin battery model (first order system)
		float v_dot_prediction = p_0*s_0 + p_1*s_1 + p_2*s_2 + p_3 + 0.4f*voltage_prediction_error;

		float bat1_r_internal = extract_parameters();

		if((abs(voltage_prediction_error)) <= abs(best_prediction_error))
		{
			best_prediction = bat1_r_internal_prev;
			best_prediction_error = voltage_prediction_error;
		} else if (best_prediction_error_reset){
			best_prediction = bat1_r_internal_prev;
			best_prediction_error = voltage_prediction_error;
			best_prediction_error_reset = false;
		}

		float time_since_param_write = (hrt_absolute_time()- last_param_write_time)/ 1e6f;

		// save bat1_r_internal only periodically
		if (time_since_param_write >= _inter_res_update_period.get()){
			write_internal_resistance();
		}

		//logging for debugging
		inter_res.timestamp = hrt_absolute_time();
		inter_res.voltage_prediction_error = abs(voltage_prediction_error);
		inter_res.battery_sampling_period = battery_sampling_period;
		inter_res.v_dot_prediction_prev = v_dot_prediction_prev;
		inter_res.time_since_param_write = time_since_param_write;
		inter_res.best_prediction = best_prediction;
		inter_res.best_prediction_error = abs(best_prediction_error);
		inter_res.best_prediction_error_reset = best_prediction_error_reset;
		inter_res.bat1_r_internal = bat1_r_internal;

		inter_res.current = battery_status.current_a;
		inter_res.voltage = battery_status.voltage_v;
		inter_res.voltage_prediction = voltage_b_prediction;
		inter_res.current_dot = (battery_status.current_a - current_a_prev)/(battery_sampling_period);
		inter_res.current_prev = current_a_prev;


		inter_res.p_0 = p_0;
		inter_res.p_1 = p_1;
		inter_res.p_2 = p_2;
		inter_res.p_3 = p_3;

		inter_res.s_0 = s_0;
		inter_res.s_1 = s_1;
		inter_res.s_2 = s_2;


		//store current values for next iteration
		voltage_b_prediction_prev = voltage_b_prediction;
		v_dot_prediction_prev = v_dot_prediction;
		bat1_r_internal_prev = bat1_r_internal;

		_internal_res_pub.publish(inter_res);

		}

		battery_time_prev = battery_status.timestamp;
		current_a_prev = battery_status.current_a;

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
