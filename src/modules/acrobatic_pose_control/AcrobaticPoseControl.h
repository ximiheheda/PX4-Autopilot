#ifndef ACROBATICPOSECONTROL_H
#define ACROBATICPOSECONTROL_H


//#include <iostream>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>
#include <matrix/math.hpp>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/acrobatic_cmd.h>
#include <uORB/topics/acrobatic_debug.h>
#include <uORB/topics/acrobatic_demo.h>
#include <uORB/topics/acrobaticL1_cmd.h>
//#include <uORB/topics/dmp_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
//#include <uORB/topics/model_iden_data.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_local_position.h>
//#include <uORB/topics/manual_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/acrobatic_setpoint.h>
#include <uORB/topics/sensor_combined.h>

//#include <vector>
#include <cmath>
#include <string.h>
#include <stdlib.h>
#include <containers/Array.hpp>
#include <parameters/param.h>
#include <matrix/Matrix.hpp>
#include <matrix/math.hpp>
#include <matrix/DualQuaternion.h>

using matrix::Eulerf;
using matrix::Quatf;
//using matrix::Vector;
//using matrix::Matrix;
using px4::Array;
using matrix::DualQuaternion;
using matrix::Quaternion;

using uORB::SubscriptionData;

#define PI 3.14159265

class AcrobaticPoseControl final : public ModuleBase<AcrobaticPoseControl>, public px4::WorkItem
{
public:
    AcrobaticPoseControl();
    ~AcrobaticPoseControl() override;


    void Run() override;
    bool init();

    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]); /**< static function does not need the object */
    int print_status() override;
    static int task_spawn(int argc, char *argv[]);
    static AcrobaticPoseControl *instantiate(int argc, char *argv[]);


private:
    perf_counter_t _loop_perf;  /**< loop performance counter */
    /**<--------------------------- Subscription -------------------------------------------*/

    //int _att_sub{-1};
    uORB::Subscription _vehicle_cmd_sub{ORB_ID(vehicle_command)};
    uORB::SubscriptionCallbackWorkItem  _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */
    uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
    uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< notification of parameter updates */
    uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)}; // Obtain the acceleration data contains the temperature(thrust indeed)
    //uORB::Subscription _man_sub{ORB_ID(manual_status)};
    uORB::Subscription _vstatus_sub{ORB_ID(vehicle_status)}; // Obtain the vehicle status and get the navigation status

    uORB::Subscription _sensor_com_sub{ORB_ID(sensor_combined)}; //Obtain the combined sensor data
    uORB::Subscription _acro_cmd_sub{ORB_ID(acrobatic_setpoint)}; //Obtain acrobatic setpoint
    uORB::Subscription _act_con_sub{ORB_ID(actuator_controls_0)};


    /**<--------------------------- Publication  ------------------------------------------*/

    uORB::Publication<acrobatic_cmd_s>      _acro_cmd_pub{ORB_ID(acrobatic_cmd)};
    uORB::Publication<acrobatic_debug_s>    _acro_debug_pub{ORB_ID(acrobatic_debug)};

    uORB::Publication<acrobatic_demo_s>     _acro_demo_pub{ORB_ID(acrobatic_demo)};
    //uORB::Publication<dmp_test_s>           _dmp_test_pub{ORB_ID(dmp_test)};        //for the dmp function test
    orb_advert_t _mavlink_log_pub{nullptr};
    //uORB::Publication<manual_status_s>          _man_pub{ORB_ID(manual_status)};


    /**<----------------------------- Message Structure -----------------------------------*/


    sensor_accel_s _sensor_accel{}; //The acceleration
    sensor_combined_s _sensor_com{}; //The combined sensor
    vehicle_attitude_s _vehicle_att{}; /**< vehicle attitude */
    acrobatic_cmd_s _acrobatic_cmd{}; /**< acrobatic cmd to fixedwing attitude module */
    acrobaticL1_cmd_s _acrobaticL1_cmd{}; // acrobatic_L1 cmd to fixed-wing attitude module
    acrobatic_setpoint_s _acro_cmd_setpoint{}; /**< acrobatic setpoint*/

    vehicle_attitude_s _att{};
    vehicle_command_s _vehicle_cmd{}; /**< vehicle command */
    vehicle_angular_velocity_s _vehicle_angular_vel; /**< vehicle angular velocity */
    vehicle_global_position_s _global_pos; /**< vehicle global position */
    vehicle_local_position_s _local_pos; /**< vehicle local position */
    //manual_status_s _man_status;
    vehicle_status_s _vstatus{};		/**< vehicle status */
    actuator_controls_s _actuator_controls{};

    /**<----------------------------- Variables ------------------------------------------*/
    Quatf _att_q{};
    Quatf _att_q_cmd{};
    void acc_data_read();
    uint16_t _finish_count{0}; /**< acrobatic finish count */
    uint16_t _start_count{0}; /**< acrobatic start count */

    hrt_abstime now;
    hrt_abstime _time_first_acrobatic{0};
    hrt_abstime time_prev;

    Quatf _quat_cmd; /**< quaternion command at present time now */
    Array<float, 3> _xyz_cmd; //xyz command at the present time new
    Quatf _quat_first_acro; /**< quaternion at the first acrobatic to integrate */
    Array<float, 3> _xyz_first_acro; // the position at the first acrobatic to integrate

    Quatf _quat_err; /**< quaternion error */
    float _tc = 0.3; //This value should be set in the QGC
    float _body_setpoint[3];
    float _alt_sp_acrobatic{0};   /**< altitude setpoint in acrobatic */
    float _alt_first_acrobatic{0};  /**< altitude first acrobatic */
    float _u_cal_tmp{0}, _v_cal_tmp{0}, _w_cal_tmp{0};
    float _pos_err_val_tmp[8];
    float _Hat_G_inv_tmp[64];

    float _v_real{0};       /**< real v velocity*/
    float _w_real{0};       /**< real w velocity*/
    float _u_real{0};       /**< real u velocity*/
    float _vel_total{0};     /**< velocity total*/

    struct{
        float fw_dq_w_i;
        float fw_dq_w_ff;
        float fw_dq_w_p;
        float fw_dq_v_i;
        float fw_dq_v_ff;
        float fw_dq_v_p;
        float fw_dq_p_i;
        float fw_dq_p_ff;
        float fw_dq_p_p;
        float fw_dq_delta_x;
        float fw_acro_q0_tc;
        float fw_acro_q1_tc;
        float fw_acro_q2_tc;
        float fw_acro_q3_tc;
    }_parameters{};			/**< local copies of interesting parameters */

    struct {
        param_t fw_dq_w_i;
        param_t fw_dq_w_ff;
        param_t fw_dq_w_p;
        param_t fw_dq_v_i;
        param_t fw_dq_v_ff;
        param_t fw_dq_v_p;
        param_t fw_dq_p_i;
        param_t fw_dq_p_ff;
        param_t fw_dq_p_p;
        param_t fw_dq_delta_x;
        param_t fw_acro_q0_tc;
        param_t fw_acro_q1_tc;
        param_t fw_acro_q2_tc;
        param_t fw_acro_q3_tc;
    } _parameter_handles{};		/**< handles for interesting parameters */

    /**<------------------------------ Functions -----------------------------------------*/
    /**
    * Update the local parameter cache.
    */
    int parameters_update();
    void vehicle_att_poll();
    void vehicle_cmd_poll();
    void vehicle_global_pos_poll();
    void vehicle_local_pos_poll();
    void sensor_accel_poll();
    void actuator_controls_poll();
    void vehicle_angular_rates_poll();
    void manual_status_poll();
    void vstatus_poll();
    void acro_setpoint_poll();

    /**
    * Several kinds of the acrobatic command generation in quaternion.
    */
    void acro_loop_cmd();
    void acro_immelman_cmd();

    /**
    * The function which is used for the model identification (added in 7.1.2021)
    */
    void mod_iden_data();

    /**
    * Realize the 1-dimensional interpolation, read from the data matrix
    */

    void pqr_uvw_acro_data_read();
    //vector<float> *InputData_To_Vector();

    void acro_safety_pre_check(); //The function to check if the aircraft is safe to perform the acrobatic

    // Obtain the desried quaternion and position
    void pqr2quat();
    void quat_uvw2xyz(); // this function can integrate the quaternion and xyz

    matrix::Matrix<float, 8, 8> Hat_G_mat(DualQuaternion<float> dq1) const;
    matrix::Matrix<float, 3, 3> inv_three_order(const matrix::Matrix<float, 3, 3> mat_A) const;

    matrix::Matrix<float, 8, 8> inv_Hat_G_mat_cal(const matrix::Matrix<float, 8, 8> Hat_G_mat) const;

    DualQuaternion<float> Convert_quat_xyz_2_DQ(const Array<float,3> _xyz, const Quaternion<float> _att_d);
    void Twist_Command_Gen(const DualQuaternion<float> _dual_quat_cmd, const float& _ref_twist) const;
    matrix::Matrix<float, 8, 1> Twist_Command_Gen(const DualQuaternion<float> _dual_quat_cmd,
                                                  const DualQuaternion<float> _dual_quat_val,
                                                  const matrix::Matrix<float, 8, 1> _ref_twist);
    void Twist_Demon_Storage();
    matrix::Matrix<float, 3, 1> Angular_rates_control(Quaternion<float> _quat_cmd,
                                                                            Quaternion<float> _att_q);
};

#endif // ACROBATICCOMMAND_H
