/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
#include <iostream>
#include <drivers/drv_hrt.h>
#include <lib/ecl/attitude_fw/ecl_pitch_controller.h>
#include <lib/ecl/attitude_fw/ecl_roll_controller.h>
#include <lib/ecl/attitude_fw/ecl_wheel_controller.h>
#include <lib/ecl/attitude_fw/ecl_yaw_controller.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/acrobatic_control.h>

using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

class FixedwingAttitudeControl final : public ModuleBase<FixedwingAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedwingAttitudeControl();
	~FixedwingAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */

	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint */
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};			/**< battery status subscription */
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};		/**< global position subscription */
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};		/**< notification of manual control updates */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< notification of parameter updates */
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint */
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _vehicle_cmd_sub{ORB_ID(vehicle_command)}; //added by caosu
    uORB::Publication<acrobatic_control_s>      _acro_control_pub{ORB_ID(acrobatic_control)}; //added by caosu



	uORB::SubscriptionData<airspeed_s> _airspeed_sub{ORB_ID(airspeed)};

	uORB::Publication<actuator_controls_s>		_actuators_2_pub{ORB_ID(actuator_controls_2)};		/**< actuator control group 1 setpoint (Airframe) */
	uORB::Publication<vehicle_rates_setpoint_s>	_rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};		/**< rate setpoint publication */
	uORB::PublicationMulti<rate_ctrl_status_s>	_rate_ctrl_status_pub{ORB_ID(rate_ctrl_status)};	/**< rate controller status publication */

	orb_id_t	_attitude_setpoint_id{nullptr};
	orb_advert_t	_attitude_sp_pub{nullptr};	/**< attitude setpoint point */

	orb_id_t	_actuators_id{nullptr};		/**< pointer to correct actuator controls0 uORB metadata structure */
	orb_advert_t	_actuators_0_pub{nullptr};	/**< actuator control group 0 setpoint */

	actuator_controls_s			_actuators {};		/**< actuator control inputs */
	actuator_controls_s			_actuators_airframe {};	/**< actuator control inputs */
	manual_control_setpoint_s		_manual {};		/**< r/c channel data */
	vehicle_attitude_s			_att {};		/**< vehicle attitude setpoint */
	vehicle_attitude_setpoint_s		_att_sp {};		/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_vcontrol_mode {};	/**< vehicle control mode */
	vehicle_global_position_s		_global_pos {};		/**< global position */
	vehicle_rates_setpoint_s		_rates_sp {};		/* attitude rates setpoint */
	vehicle_status_s			_vehicle_status {};	/**< vehicle status */
    acrobatic_control_s      _acro_control {}; //added by caosu


	perf_counter_t	_loop_perf;			/**< loop performance counter */

	float _flaps_applied{0.0f};
	float _flaperons_applied{0.0f};

	float _airspeed_scaling{1.0f};

	bool _landed{true};

    bool _safe_to_acro{false};

    vehicle_command_s _vehicle_cmd = {};

	float _battery_scale{1.0f};

	bool _flag_control_attitude_enabled_last{false};

	bool _is_tailsitter{false};


    hrt_abstime now; //added by caosu
    hrt_abstime _time_first_acrobatic{0};
    float _pitch_first_acrobatic{0};
    float _roll_first_acrobatic{0};
    float _yaw_first_acrobatic{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_ACRO_X_MAX>) _param_fw_acro_x_max,
		(ParamFloat<px4::params::FW_ACRO_Y_MAX>) _param_fw_acro_y_max,
		(ParamFloat<px4::params::FW_ACRO_Z_MAX>) _param_fw_acro_z_max,

        (ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max, // min speed
        (ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min, // min speed
        (ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim, //cruise speed
		(ParamInt<px4::params::FW_ARSP_MODE>) _param_fw_arsp_mode,

		(ParamBool<px4::params::FW_BAT_SCALE_EN>) _param_fw_bat_scale_en,

		(ParamFloat<px4::params::FW_DTRIM_P_FLPS>) _param_fw_dtrim_p_flps,
		(ParamFloat<px4::params::FW_DTRIM_P_VMAX>) _param_fw_dtrim_p_vmax,
		(ParamFloat<px4::params::FW_DTRIM_P_VMIN>) _param_fw_dtrim_p_vmin,
		(ParamFloat<px4::params::FW_DTRIM_R_FLPS>) _param_fw_dtrim_r_flps,
		(ParamFloat<px4::params::FW_DTRIM_R_VMAX>) _param_fw_dtrim_r_vmax,
		(ParamFloat<px4::params::FW_DTRIM_R_VMIN>) _param_fw_dtrim_r_vmin,
		(ParamFloat<px4::params::FW_DTRIM_Y_VMAX>) _param_fw_dtrim_y_vmax,
		(ParamFloat<px4::params::FW_DTRIM_Y_VMIN>) _param_fw_dtrim_y_vmin,

		(ParamFloat<px4::params::FW_FLAPERON_SCL>) _param_fw_flaperon_scl,
		(ParamFloat<px4::params::FW_FLAPS_LND_SCL>) _param_fw_flaps_lnd_scl,
		(ParamFloat<px4::params::FW_FLAPS_SCL>) _param_fw_flaps_scl,
		(ParamFloat<px4::params::FW_FLAPS_TO_SCL>) _param_fw_flaps_to_scl,

		(ParamFloat<px4::params::FW_MAN_P_MAX>) _param_fw_man_p_max,
		(ParamFloat<px4::params::FW_MAN_P_SC>) _param_fw_man_p_sc,
        (ParamFloat<px4::params::FW_MAN_R_MAX>) _param_fw_man_r_max, //stabilizing, the max desired angle
		(ParamFloat<px4::params::FW_MAN_R_SC>) _param_fw_man_r_sc,
		(ParamFloat<px4::params::FW_MAN_Y_SC>) _param_fw_man_y_sc,

		(ParamFloat<px4::params::FW_P_RMAX_NEG>) _param_fw_p_rmax_neg,
		(ParamFloat<px4::params::FW_P_RMAX_POS>) _param_fw_p_rmax_pos,
		(ParamFloat<px4::params::FW_P_TC>) _param_fw_p_tc,
		(ParamFloat<px4::params::FW_PR_FF>) _param_fw_pr_ff,
		(ParamFloat<px4::params::FW_PR_I>) _param_fw_pr_i,
		(ParamFloat<px4::params::FW_PR_IMAX>) _param_fw_pr_imax,
		(ParamFloat<px4::params::FW_PR_P>) _param_fw_pr_p,
		(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,

		(ParamFloat<px4::params::FW_RATT_TH>) _param_fw_ratt_th,

		(ParamFloat<px4::params::FW_R_RMAX>) _param_fw_r_rmax,
        (ParamFloat<px4::params::FW_R_TC>) _param_fw_r_tc, //outer control P
        (ParamFloat<px4::params::FW_RLL_TO_YAW_FF>) _param_fw_rll_to_yaw_ff, //roll to yaw the feedforward
        (ParamFloat<px4::params::FW_RR_FF>) _param_fw_rr_ff, //speed to control surface feedforward
        (ParamFloat<px4::params::FW_RR_I>) _param_fw_rr_i, //inner control I
        (ParamFloat<px4::params::FW_RR_IMAX>) _param_fw_rr_imax, //inner control I, to anti wind-up
        (ParamFloat<px4::params::FW_RR_P>) _param_fw_rr_p, //inner control P
        (ParamFloat<px4::params::FW_RSP_OFF>) _param_fw_rsp_off, //the desired angle, offset

		(ParamBool<px4::params::FW_W_EN>) _param_fw_w_en,
		(ParamFloat<px4::params::FW_W_RMAX>) _param_fw_w_rmax,
		(ParamFloat<px4::params::FW_WR_FF>) _param_fw_wr_ff,
		(ParamFloat<px4::params::FW_WR_I>) _param_fw_wr_i,
		(ParamFloat<px4::params::FW_WR_IMAX>) _param_fw_wr_imax,
		(ParamFloat<px4::params::FW_WR_P>) _param_fw_wr_p,

		(ParamFloat<px4::params::FW_Y_RMAX>) _param_fw_y_rmax,
		(ParamFloat<px4::params::FW_YR_FF>) _param_fw_yr_ff,
		(ParamFloat<px4::params::FW_YR_I>) _param_fw_yr_i,
		(ParamFloat<px4::params::FW_YR_IMAX>) _param_fw_yr_imax,
		(ParamFloat<px4::params::FW_YR_P>) _param_fw_yr_p,

		(ParamFloat<px4::params::TRIM_PITCH>) _param_trim_pitch,
		(ParamFloat<px4::params::TRIM_ROLL>) _param_trim_roll,
		(ParamFloat<px4::params::TRIM_YAW>) _param_trim_yaw
	)

	ECL_RollController		_roll_ctrl;
	ECL_PitchController		_pitch_ctrl;
	ECL_YawController		_yaw_ctrl;
	ECL_WheelController		_wheel_ctrl;

	void control_flaps(const float dt);

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_attitude_setpoint_poll();
	void		vehicle_rates_setpoint_poll();
	void		vehicle_status_poll();
	void		vehicle_land_detected_poll();

	float 		get_airspeed_and_update_scaling();
};
