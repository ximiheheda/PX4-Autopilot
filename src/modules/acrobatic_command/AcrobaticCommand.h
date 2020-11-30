#ifndef ACROBATICCOMMAND_H
#define ACROBATICCOMMAND_H


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
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
//#include <vector>
//#include <cmath>
#include <string.h>
#include <stdlib.h>
#include <containers/Array.hpp>

using matrix::Eulerf;
using matrix::Quatf;
//using matrix::Vector;
using matrix::Matrix;
using px4::Array;

using uORB::SubscriptionData;

class AcrobaticCommand final : public ModuleBase<AcrobaticCommand>, public ModuleParams
{
public:
    AcrobaticCommand();
    ~AcrobaticCommand() override;


    void run() override;
    bool init();

    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]); /**< static function does not need the object */
    int print_status() override;
    static int task_spawn(int argc, char *argv[]);
    static AcrobaticCommand *instantiate(int argc, char *argv[]);



private:
    perf_counter_t _loop_perf;  /**< loop performance counter */

    //int _att_sub{-1};
    uORB::Subscription _vehicle_cmd_sub{ORB_ID(vehicle_command)};

    uORB::Publication<acrobatic_cmd_s>      _acro_cmd_pub{ORB_ID(acrobatic_cmd)};
    uORB::Subscription  _att_sub{ORB_ID(vehicle_attitude)};	/**< vehicle attitude */
    uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
    uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};

    const char *filepath; /**< file path of the acrobatic category */


    vehicle_attitude_s _vehicle_att{}; /**< vehicle attitude */
    acrobatic_cmd_s _acrobatic_cmd{}; /**< acrobatic cmd to fixedwing attitude module */

    vehicle_attitude_s _att{};
    Quatf _att_q{};
    Quatf _att_q_cmd{};

    vehicle_command_s _vehicle_cmd{}; /**< vehicle command */
    vehicle_angular_velocity_s _vehicle_angular_vel{}; /**< vehicle angular velocity */
    vehicle_global_position_s _global_pos; /**< vehicle global position */
    vehicle_local_position_s _local_pos; /**< vehicle local position */

    uint16_t _finish_count{0}; /**< acrobatic finish count */
    uint16_t _start_count{0}; /**< acrobatic start count */

    bool file_readed{false}; /**< the acrobatic file has been readed */


    hrt_abstime now;
    hrt_abstime _time_first_acrobatic{0};
    hrt_abstime time_prev;

    struct quat_time
    {
        Quatf quat_v;
        hrt_abstime time_v;
    };

    Array<quat_time, 1000> _quat_time_l;

    Quatf _quat_cmd; /**< quaternion command at present time now*/
    Quatf _quat_err; /**< quaternion error */
    float _tc = 0.3;
    float _body_setpoint[3];
    float _alt_sp_acrobatic{0};   /**< altitude setpoint in acrobatic */
    float _alt_first_acrobatic{0};  /**< altitude first acrobatic */

    /**
    * Update the local parameter cache.
    */
    int parameter_update();
    void vehicle_att_poll();
    void vehicle_cmd_poll();
    void vehicle_global_pos_poll();
    void vehicle_local_pos_poll();

    /**
    * Several kinds of the acrobatic command generation in quaternion.
    */
    void acro_loop_cmd();
    void acro_immelman_cmd();

    /**
    * Realize the 1-dimensional interpolation, read from the data matrix
    */
    Quatf interp_1_d();

    void acro_data_read();
    //vector<float> *InputData_To_Vector();

    struct{

    param_t q_tc;   /**< time constant for the p control in quaternion */

    }_parameter_handles{};





























};

#endif // ACROBATICCOMMAND_H
