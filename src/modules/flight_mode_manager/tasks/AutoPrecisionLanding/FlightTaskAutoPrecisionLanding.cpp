/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoPrecisionLanding.cpp
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "FlightTaskAutoPrecisionLanding.hpp"

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

static constexpr const char *LOST_TARGET_ERROR_MESSAGE = "Lost landing target while landing";

bool FlightTaskAutoPrecisionLanding::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	// This looks wrong at first, but is a mean little trick to avoid discontinuous setpoints.
	// When this flight task is activated, _target might be outdated until the next navigator triplet
	// comes in. Until then pretend that the current position setpoint is the navigator's setpoint:
	_target = _position_setpoint;

	_search_cnt = 0;
	_last_slewrate_time = 0;

	_sp_pev = matrix::Vector2f(last_setpoint.x, last_setpoint.y);
	_sp_pev_prev = matrix::Vector2f(last_setpoint.x, last_setpoint.y);

	switch_to_state_start();

	return ret;
}

bool FlightTaskAutoPrecisionLanding::update()
{
	bool ret = FlightTaskAuto::update();

	// Fetch uorb
	if (_target_pose_sub.updated()) {
		_target_pose_sub.copy(&_target_pose);
	}

	// target pose can become invalid when the message timed out
	_target_pose_valid = (hrt_elapsed_time(&_target_pose.timestamp) / 1e6f) <= _param_pld_btout.get();

	switch (_state) {
	case PrecLandState::Start:
		run_state_start();
		break;

	case PrecLandState::HorizontalApproach:
		run_state_horizontal_approach();
		break;

	case PrecLandState::DescendAboveTarget:
		run_state_descend_above_target();
		break;

	case PrecLandState::FinalApproach:
		run_state_final_approach();
		break;

	case PrecLandState::Search:
		run_state_search();
		break;

	case PrecLandState::Fallback:
		run_state_fallback();
		break;

	default:
		// unknown state
		break;
	}

	// Publish status message for debugging
	precision_landing_status_s precision_landing_status{};
	precision_landing_status.timestamp = hrt_absolute_time();
	precision_landing_status.precland_state = (uint8_t) _state;
	_precision_landing_status_pub.publish(precision_landing_status);

	_constraints.want_takeoff = _checkTakeoff();

	return ret;
}

void
FlightTaskAutoPrecisionLanding::run_state_start()
{
	_position_setpoint = _target; // Follow navigator triplet

	// check if target visible and go to horizontal approach directly
	if (switch_to_state_horizontal_approach()) {
		return;

	} else if ((PrecLandMode)_param_rtl_pld_md.get() == PrecLandMode::Opportunistic) {

		// could not see the target immediately, so just fall back to normal landing
		switch_to_state_fallback();

	} else if (_type == WaypointType::land) {

		// Navigator already entered land stage. Take over with precision landing
		switch_to_state_search();
	}
}

void
FlightTaskAutoPrecisionLanding::run_state_horizontal_approach()
{
	float x = _target_pose.x_abs;
	float y = _target_pose.y_abs;
	slewrate(x, y);

	// Fly to target XY position, but keep navigator's altitude setpoint
	_position_setpoint(0) = x;
	_position_setpoint(1) = y;
	_position_setpoint(2) = _target(2);
	_velocity_setpoint(0) = _velocity_setpoint(1) = _velocity_setpoint(2) = NAN;

	// check if target visible, if not go to start
	if (!check_state_conditions(PrecLandState::HorizontalApproach)) {
		PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);

		// Stay at current position for searching for the landing target
		// TODO: This is not going to work after conversion to flight task, because
		// the position_setpoint will be overwritten at the next iteration!
		// Solution: An additional wait state
		_position_setpoint = _position;
		_velocity_setpoint(0) = _velocity_setpoint(1) = _velocity_setpoint(2) = NAN;

		if (!switch_to_state_start()) {
			switch_to_state_fallback();
		}

		return;
	}

	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!_point_reached_time) {
			_point_reached_time = hrt_absolute_time();
		}

		if (hrt_absolute_time() - _point_reached_time > 2000000) {
			// if close enough for descent above target go to descend above target
			if (switch_to_state_descend_above_target()) {

				return;
			}
		}

	}
}

void
FlightTaskAutoPrecisionLanding::run_state_descend_above_target()
{
	// Overwrite Auto setpoints in order to descend above target
	_position_setpoint(0) = _target_pose.x_abs;
	_position_setpoint(1) = _target_pose.y_abs;
	_position_setpoint(2) = NAN;
	_velocity_setpoint(0) = 0;
	_velocity_setpoint(1) = 0;
	_velocity_setpoint(2) = _param_mpc_land_speed.get();

	// check if target visible
	if (!check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!switch_to_state_final_approach()) {
			PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);

			// Stay at current position for searching for the target
			_position_setpoint = _position;

			if (!switch_to_state_start()) {
				switch_to_state_fallback();
			}
		}

		return;
	}
}

void
FlightTaskAutoPrecisionLanding::run_state_final_approach()
{
	// Overwrite Auto setpoints in order to land at target's last known location
	_position_setpoint(0) = _target_pose.x_abs;
	_position_setpoint(1) = _target_pose.y_abs;
	_position_setpoint(2) = NAN;
	_velocity_setpoint(0) = 0;
	_velocity_setpoint(1) = 0;
	_velocity_setpoint(2) = _param_mpc_land_speed.get();
}

void
FlightTaskAutoPrecisionLanding::run_state_search()
{
	// Overwrite Auto setpoints in order to hover at search altitude
	_position_setpoint = _target;
	_position_setpoint(2) = _sub_home_position.get().z - _param_pld_srch_alt.get();
	_velocity_setpoint(0) = _velocity_setpoint(1) = _velocity_setpoint(2) = NAN;

	// check if we can see the target
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		if (!_target_acquired_time) {
			// target just became visible. Stop climbing, but give it some margin so we don't stop too apruptly
			_target_acquired_time = hrt_absolute_time();
			_position_setpoint = _position;
			_position_setpoint(2) += 1.0f;
		}

	}

	// stay at that height for a second to allow the vehicle to settle
	if (_target_acquired_time && (hrt_absolute_time() - _target_acquired_time) > 1000000) {
		// try to switch to horizontal approach
		if (switch_to_state_horizontal_approach()) {
			return;
		}
	}

	// check if search timed out and go to fallback
	if (hrt_absolute_time() - _state_start_time > _param_pld_srch_tout.get()*SEC2USEC) {
		PX4_WARN("Search timed out");

		switch_to_state_fallback();
	}
}

void
FlightTaskAutoPrecisionLanding::run_state_fallback()
{
	// nothing to do, just listen to navigator
	_position_setpoint = _target;
	_velocity_setpoint(0) = 0;
	_velocity_setpoint(1) = 0;
	_velocity_setpoint(2) = _param_mpc_land_speed.get();
}

bool
FlightTaskAutoPrecisionLanding::switch_to_state_start()
{
	if (check_state_conditions(PrecLandState::Start)) {
		_search_cnt++;

		_point_reached_time = 0;

		_state = PrecLandState::Start;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
FlightTaskAutoPrecisionLanding::switch_to_state_horizontal_approach()
{
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		print_state_switch_message("horizontal approach");

		_point_reached_time = 0;

		_state = PrecLandState::HorizontalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
FlightTaskAutoPrecisionLanding::switch_to_state_descend_above_target()
{
	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		print_state_switch_message("descend");
		_state = PrecLandState::DescendAboveTarget;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
FlightTaskAutoPrecisionLanding::switch_to_state_final_approach()
{
	if (check_state_conditions(PrecLandState::FinalApproach)) {
		print_state_switch_message("final approach");
		_state = PrecLandState::FinalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

void
FlightTaskAutoPrecisionLanding::switch_to_state_search()
{
	PX4_INFO("Climbing to search altitude");

	_target_acquired_time = 0;

	_state = PrecLandState::Search;
	_state_start_time = hrt_absolute_time();
}

void
FlightTaskAutoPrecisionLanding::switch_to_state_fallback()
{
	print_state_switch_message("fallback");

	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();
}

void FlightTaskAutoPrecisionLanding::print_state_switch_message(const char *state_name)
{
	PX4_INFO("Precland: switching to %s", state_name);
}

bool FlightTaskAutoPrecisionLanding::check_state_conditions(PrecLandState state)
{
	switch (state) {
	case PrecLandState::Start:
		return _search_cnt <= _param_pld_max_srch.get();

	case PrecLandState::HorizontalApproach:

		// if we're already in this state, only want to make it invalid if we reached the target but can't see it anymore
		if (_state == PrecLandState::HorizontalApproach) {
			if (Vector2f(Vector2f(_target_pose.x_abs, _target_pose.y_abs) - _position.xy()).norm() <= _param_pld_hacc_rad.get()) {
				// we've reached the position where we last saw the target. If we don't see it now, we need to do something
				return _target_pose_valid && _target_pose.abs_pos_valid;

			} else {
				// We've seen the target sometime during horizontal approach.
				// Even if we don't see it as we're moving towards it, continue approaching last known location
				return true;
			}
		}

		// If we're trying to switch to this state, the target needs to be visible
		return _target_pose_valid && _target_pose.abs_pos_valid;

	case PrecLandState::DescendAboveTarget:

		// if we're already in this state, only leave it if target becomes unusable, don't care about horizontal offset to target
		if (_state == PrecLandState::DescendAboveTarget) {
			// if we're close to the ground, we're more critical of target timeouts so we quickly go into descend
			if (check_state_conditions(PrecLandState::FinalApproach)) {
				return hrt_absolute_time() - _target_pose.timestamp < 500000; // 0.5s  // TODO: Magic number!

			} else {
				return _target_pose_valid && _target_pose.abs_pos_valid;
			}

		} else {
			// if not already in this state, need to be above target to enter it
			return _target_pose.abs_pos_valid
			       && fabsf(_target_pose.x_abs - _position(0)) < _param_pld_hacc_rad.get()
			       && fabsf(_target_pose.y_abs - _position(1)) < _param_pld_hacc_rad.get();
		}

	case PrecLandState::FinalApproach:
		return _target_pose_valid && _target_pose.abs_pos_valid
		       && (_target_pose.z_abs - _position(2)) < _param_pld_fappr_alt.get();

	case PrecLandState::Search:
		return true;

	case PrecLandState::Fallback:
		return true;

	default:
		return false;
	}
}

void FlightTaskAutoPrecisionLanding::slewrate(float &sp_x, float &sp_y)
{
	matrix::Vector2f sp_curr(sp_x, sp_y);
	uint64_t now = hrt_absolute_time();

	float dt = (now - _last_slewrate_time);

	if (dt < 1) {
		// bad dt, can't divide by it
		return;
	}

	dt /= SEC2USEC;

	if (!_last_slewrate_time) {
		// running the first time since switching to precland

		// assume dt will be about 50000us
		dt = 50000 / SEC2USEC;

		// set a best guess for previous setpoints for smooth transition
		_sp_pev_prev(0) = _sp_pev(0) - _velocity(0) * dt;
		_sp_pev_prev(1) = _sp_pev(1) - _velocity(1) * dt;
	}

	_last_slewrate_time = now;

	// limit the setpoint speed to the maximum cruise speed
	matrix::Vector2f sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > _param_xy_vel_cruise.get()) {
		sp_vel = sp_vel.normalized() * _param_xy_vel_cruise.get();
		sp_curr = _sp_pev + sp_vel * dt;
	}

	// limit the setpoint acceleration to the maximum acceleration
	matrix::Vector2f sp_acc = (sp_curr - _sp_pev * 2 + _sp_pev_prev) / (dt * dt); // acceleration of the setpoints

	if (sp_acc.length() > _param_acceleration_hor.get()) {
		sp_acc = sp_acc.normalized() * _param_acceleration_hor.get();
		sp_curr = _sp_pev * 2 - _sp_pev_prev + sp_acc * (dt * dt);
	}

	// limit the setpoint speed such that we can stop at the setpoint given the maximum acceleration/deceleration
	float max_spd = sqrtf(_param_acceleration_hor.get() * ((matrix::Vector2f)(_sp_pev - matrix::Vector2f(sp_x,
			      sp_y))).length());
	sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > max_spd) {
		sp_vel = sp_vel.normalized() * max_spd;
		sp_curr = _sp_pev + sp_vel * dt;
	}

	_sp_pev_prev = _sp_pev;
	_sp_pev = sp_curr;

	sp_x = sp_curr(0);
	sp_y = sp_curr(1);
}
