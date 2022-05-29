/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
* @file FailureDetector.cpp
*
* @author Mathieu Bresciani	<brescianimathieu@gmail.com>
*
*/

#include "FailureDetector.hpp"

using namespace time_literals;

FailureDetector::FailureDetector(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool FailureDetector::resetAttitudeStatus()
{

	int attitude_fields_bitmask = _status & (FAILURE_ROLL | FAILURE_PITCH | FAILURE_ALT | FAILURE_EXT);
	bool status_changed(false);

	if (attitude_fields_bitmask > FAILURE_NONE) {
		_status &= ~attitude_fields_bitmask;
		status_changed = true;
	}

	return status_changed;
}

bool
FailureDetector::update(const vehicle_status_s &vehicle_status)
{

	bool updated(false);

	if (isAttitudeStabilized(vehicle_status)) {
		updated |= updateAttitudeStatus();

		if (_param_fd_ext_ats_en.get()) {
			updated |= updateExternalAtsStatus();
		}

	} else {
		updated |= resetAttitudeStatus();
	}

	if (_sub_esc_status.updated()) {

		if (_param_escs_en.get()) {
			updated |= updateEscsStatus(vehicle_status);
		}

	}

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

		vehicle_local_position_s vehicle_local_position;

		home_position_s home_position;

		if (!got_home_position) {
			if (_sub_home_position.update(&home_position)) {
				home_z = home_position.z;
				got_home_position = true;
			}
		}

		if (_sub_vehicle_local_position.update(&vehicle_local_position) && !escaped_z_threshold && got_home_position) {

			escaped_z_threshold = (-_param_fd_escape_z.get() > (vehicle_local_position.z - home_z));

			if (escaped_z_threshold) {
				PX4_INFO("Lift Off Detected!");
			}
		}

		position_setpoint_triplet_s pos_sp_triplet;

		if (_sub_pos_sp_triplet.update(&pos_sp_triplet)) {
			in_takeoff = pos_sp_triplet.current.valid && pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
		}

		bool detect_flip = !escaped_z_threshold && in_takeoff;

		if (_param_flip_en.get() &&
		    detect_flip) {

			updated |= updateFlipStatus();

		}

	}



	return updated;
}

bool
FailureDetector::isAttitudeStabilized(const vehicle_status_s &vehicle_status)
{
	bool attitude_is_stabilized{false};
	const uint8_t vehicle_type = vehicle_status.vehicle_type;
	const uint8_t nav_state = vehicle_status.nav_state;

	if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		attitude_is_stabilized =  nav_state != vehicle_status_s::NAVIGATION_STATE_ACRO &&
					  nav_state != vehicle_status_s::NAVIGATION_STATE_RATTITUDE;

	} else if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		attitude_is_stabilized =  nav_state != vehicle_status_s::NAVIGATION_STATE_MANUAL &&
					  nav_state != vehicle_status_s::NAVIGATION_STATE_ACRO &&
					  nav_state != vehicle_status_s::NAVIGATION_STATE_RATTITUDE;
	}

	return attitude_is_stabilized;
}

bool
FailureDetector::updateAttitudeStatus()
{
	bool updated(false);
	vehicle_attitude_s attitude;

	if (_sub_vehicule_attitude.update(&attitude)) {

		const matrix::Eulerf euler(matrix::Quatf(attitude.q));
		const float roll(euler.phi());
		const float pitch(euler.theta());

		const float max_roll_deg = _param_fd_fail_r.get();
		const float max_pitch_deg = _param_fd_fail_p.get();
		const float max_roll(fabsf(math::radians(max_roll_deg)));
		const float max_pitch(fabsf(math::radians(max_pitch_deg)));

		const bool roll_status = (max_roll > 0.0f) && (fabsf(roll) > max_roll);
		const bool pitch_status = (max_pitch > 0.0f) && (fabsf(pitch) > max_pitch);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_roll_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_r_ttri.get()));
		_pitch_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_p_ttri.get()));
		_roll_failure_hysteresis.set_state_and_update(roll_status, time_now);
		_pitch_failure_hysteresis.set_state_and_update(pitch_status, time_now);

		// Update bitmask
		_status &= ~(FAILURE_ROLL | FAILURE_PITCH);

		if (_roll_failure_hysteresis.get_state()) {
			_status |= FAILURE_ROLL;
		}

		if (_pitch_failure_hysteresis.get_state()) {
			_status |= FAILURE_PITCH;
		}

		updated = true;
	}

	return updated;
}

bool
FailureDetector::updateFlipStatus()
{
	bool updated(false);

	rate_ctrl_status_s rate_ctrl_status;
	//mc_vel_ctrl_status_s mc_vel_ctrl_status;
	//_sub_mc_vel_ctrl_status.update(&mc_vel_ctrl_status);

	if (_sub_rate_ctrl_status.update(&rate_ctrl_status) && !escaped_z_threshold) {

		bool pr_speed_integ_saturating = (abs(rate_ctrl_status.pitchspeed_integ) >= _param_fd_fail_pri.get()
						  or abs(rate_ctrl_status.rollspeed_integ) >= _param_fd_fail_pri.get());

		if (pr_speed_integ_saturating && !(_status & FAILURE_FLIP)) {
			_status |= FAILURE_FLIP;
			PX4_INFO("Flip Detected!");
			updated = true;
		}

	}

	return updated;
}

bool
FailureDetector::updateExternalAtsStatus()
{
	pwm_input_s pwm_input;
	bool updated(false);

	if (_sub_pwm_input.update(&pwm_input)) {

		uint32_t pulse_width = pwm_input.pulse_width;
		bool ats_trigger_status = (pulse_width >= (uint32_t)_param_fd_ext_ats_trig.get()) && (pulse_width < 3_ms);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_ext_ats_failure_hysteresis.set_hysteresis_time_from(false, 100_ms); // 5 consecutive pulses at 50hz
		_ext_ats_failure_hysteresis.set_state_and_update(ats_trigger_status, time_now);

		_status &= ~FAILURE_EXT;

		if (_ext_ats_failure_hysteresis.get_state()) {
			_status |= FAILURE_EXT;
		}

		updated = true;
	}

	return updated;
}

bool
FailureDetector::updateEscsStatus(const vehicle_status_s &vehicle_status)
{
	hrt_abstime time_now = hrt_absolute_time();
	bool updated(false);

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

		esc_status_s esc_status{};
		_sub_esc_status.copy(&esc_status);

		int all_escs_armed = (1 << esc_status.esc_count) - 1;


		_esc_failure_hysteresis.set_hysteresis_time_from(false, 300_ms);
		_esc_failure_hysteresis.set_state_and_update(all_escs_armed != esc_status.esc_armed_flags, time_now);

		if (_esc_failure_hysteresis.get_state() && !(_status & FAILURE_ARM_ESCS)) {
			_status |= FAILURE_ARM_ESCS;
			updated = true;
		}

	} else {
		// reset ESC bitfield
		_esc_failure_hysteresis.set_state_and_update(false, time_now);

		if (_status & FAILURE_ARM_ESCS) {
			_status &= ~FAILURE_ARM_ESCS;
			updated = true;
		}
	}

	return updated;
}
