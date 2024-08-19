#ifndef ACROBATICCOMMANDER_H
#define ACROBATICCOMMANDER_H


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
#include <uORB/topics/acrobatic_setpoint.h>
#include <uORB/topics/vehicle_status.h>
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
#include <matrix/Vector3.hpp>

//hrt_abstime _last_run{0};
//float _rate_prev_filtered;
//float rate_d;
using matrix::Eulerf;
using matrix::Quatf;
//using matrix::Vector;
//using matrix::Matrix;
using px4::Array;
using matrix::DualQuaternion;
using matrix::Quaternion;

using uORB::SubscriptionData;

#define PI 3.14159265

class AcrobaticCommander final : public ModuleBase<AcrobaticCommander>, public px4::WorkItem
{
public:
    AcrobaticCommander();
    ~AcrobaticCommander() override;


    void Run() override;
    bool init();

    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]); /**< static function does not need the object */
    int print_status() override;
    static int task_spawn(int argc, char *argv[]);
    static AcrobaticCommander *instantiate(int argc, char *argv[]);


private:
    perf_counter_t _loop_perf;  /**< loop performance counter */

    //int _att_sub{-1};
    uORB::Subscription _vehicle_cmd_sub{ORB_ID(vehicle_command)};

    uORB::Publication<acrobatic_cmd_s>      _acro_cmd_pub{ORB_ID(acrobatic_cmd)};
    uORB::Publication<acrobatic_debug_s>    _acro_debug_pub{ORB_ID(acrobatic_debug)};

    uORB::Publication<acrobaticL1_cmd_s>    _acroL1_cmd_pub{ORB_ID(acrobaticL1_cmd)};
    uORB::Publication<acrobatic_demo_s>     _acro_demo_pub{ORB_ID(acrobatic_demo)};
    //uORB::Publication<dmp_test_s>           _dmp_test_pub{ORB_ID(dmp_test)};        //for the dmp function test
    uORB::SubscriptionCallbackWorkItem  _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */
    uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
    uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< notification of parameter updates */

    uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)}; // Obtain the acceleration data contains the temperature(thrust indeed)
    //uORB::Subscription _man_sub{ORB_ID(manual_status)};
    uORB::Subscription _vstatus_sub{ORB_ID(vehicle_status)}; // Obtain the vehicle status and get the navigation status

    uORB::Subscription _sensor_com_sub{ORB_ID(sensor_combined)}; //Obtain the combined sensor data
    sensor_accel_s _sensor_accel{}; //The acceleration
    sensor_combined_s _sensor_com{}; //The combined sensor
    //uORB::Publication<manual_status_s>          _man_pub{ORB_ID(manual_status)};
    uORB::Publication<acrobatic_setpoint_s>     _acro_setpoint_pub{ORB_ID(acrobatic_setpoint)};
    acrobatic_setpoint_s _acro_setpoint{};



    orb_advert_t _mavlink_log_pub{nullptr};

    const char *filepath_att; /**< file path of the acrobatic category in attitude */
    const char *filepath_pqr; /**< file path of the acrobatic category in pqr */
    const char *filepath_pqr_uvw; // file path for the acrobatic in pqr and uvw


    const char *filepath_acc;
    vehicle_attitude_s _vehicle_att{}; /**< vehicle attitude */
    acrobatic_cmd_s _acrobatic_cmd{}; /**< acrobatic cmd to fixedwing attitude module */
    acrobaticL1_cmd_s _acrobaticL1_cmd{}; // acrobatic_L1 cmd to fixed-wing attitude module
    //this variable contains the variables
    // in the dmp function
    //dmp_test_s _dmp_test{}; /* dmp test*/

    vehicle_attitude_s _att{};
    Quatf _att_q{};
    Quatf _att_q_cmd{};
    void acc_data_read();


    vehicle_command_s _vehicle_cmd{}; /**< vehicle command */
    vehicle_angular_velocity_s _vehicle_angular_vel; /**< vehicle angular velocity */
    vehicle_global_position_s _global_pos; /**< vehicle global position */
    vehicle_local_position_s _local_pos; /**< vehicle local position */
    //manual_status_s _man_status;
    vehicle_status_s _vstatus{};		/**< vehicle status */


    uint16_t _finish_count{0}; /**< acrobatic finish count */
    uint16_t _start_count{0}; /**< acrobatic start count */

    bool file_readed{false}; /**< the acrobatic file has been readed */
    Array<float,3> interp_1_d_acc();


    hrt_abstime now;
    hrt_abstime _time_first_acrobatic{0};
    hrt_abstime time_prev;

    struct quat_time
    {
        Quatf quat_v;
        hrt_abstime time_v;
    };

    Array<quat_time, 1000> _quat_time_l;

    struct pqr_time
    {
        float pqr_v[3];
        hrt_abstime time_v;
    };
    Array<pqr_time, 1000> _pqr_time_l;

    struct acc_time
    {
        Array<float, 3> acc_v;
        hrt_abstime time_v;
    };
    Array<acc_time, 1000> _acc_time_l;

    struct uvw_time
    {
        Array<float, 3> uvw_v;
        hrt_abstime time_v;
    };
    Array<uvw_time, 1000> _uvw_time_l;

    struct xyz_time
    {
        Array<float, 3> xyz_v;
        hrt_abstime time_v;
    };
    Array<xyz_time, 1000> _xyz_time_l;

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

    //Model identification
    //model_iden_data_s _mod_iden_data{};
    //uORB::Publication<model_iden_data_s>       _mod_iden_pub{ORB_ID(model_iden_data)};

    actuator_controls_s _actuator_controls{};
    uORB::Subscription _act_con_sub{ORB_ID(actuator_controls_0)};

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

    Quaternion<float> interp_1_d_quat(); //old version
    Array<float,3> interp_1_d_xyz(); //new version

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

    matrix::Matrix<float, 8, 1> interp_1_d_pqr_uvw();
    void Twist_Demon_Storage();
};

#endif // ACROBATICCOMMAND_H
