#ifndef ACROBATICCOMMAND_H
#define ACROBATICCOMMAND_H

#endif // ACROBATICCOMMAND_H

#include <iostream>
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
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

class AcrobaticCommand final : public ModuleBase<AcrobaticCommand>, public ModuleParams,
        public px4::WorkItem
{
public:
    AcrobaticCommand();
    ~AcrobaticCommand() override;

    void Run() override;
    bool init();

private:
    perf_counter_t _loop_perf;  /**< loop performance counter */

    uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_cmd_sub{ORB_ID(vehicle_command)};


    vehicle_attitude_s _vehicle_att{}; /**< vehicle attitude */
    Quatf _att_q{};
    Quatf _att_q_cmd{};

    vehicle_command_s _vehicle_cmd{}; /**< vehicle command */
    vehicle_angular_velocity_s _vehicle_angular_vel{}; /**< vehicle angular velocity */


    hrt_abstime now;
    hrt_abstime _time_first_acrobatic{0};


    /**
    * Update the local parameter cache.
    */
    int parameter_update();
    void vehicle_att_poll();
    void vehicle_cmd_poll();

    /**
    * Several kinds of the acrobatic command generation in quaternion.
    */
    void acro_loop_cmd();
    void acro_immelman_cmd();

    /**
    * Realize the 1-dimensional interpolation, read from the data matrix
    */
    void interp_1_d(matrix::Matrix<float,10,10>);




























};
