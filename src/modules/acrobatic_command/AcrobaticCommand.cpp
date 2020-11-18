/**
 * Acrobatic Command
 * This module aims to generate the desired attitude command in quaternion and convert it to p,q,r(axis rotational rates)
 *
 *
 */
/**< Variables */

#include "AcrobaticCommand.h"
#include <px4_log.h>
#include <systemlib/mavlink_log.h>
using namespace time_literals;

AcrobaticCommand::AcrobaticCommand():
    ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
    /**< fetch initial parameter values*/

}

AcrobaticCommand::~AcrobaticCommand()
{
    perf_free(_loop_perf);
}

int
AcrobaticCommand::parameter_update()
{

    return PX4_OK;
}

void
AcrobaticCommand::vehicle_att_poll()
{
    if(_att_sub.update(&_vehicle_att))
    {
        _att_q(0) = _vehicle_att.q[0];
        _att_q(0) = _vehicle_att.q[1];
        _att_q(0) = _vehicle_att.q[2];
        _att_q(0) = _vehicle_att.q[3];
    }
}

void
AcrobaticCommand::vehicle_cmd_poll()
{
    _vehicle_cmd_sub.update(&_vehicle_cmd);
}

void
AcrobaticCommand::acro_loop_cmd()
{

}

void
AcrobaticCommand::interp_1_d(matrix::Matrix<float,10,10> acro_data)
{
    /**< In order to accelerate the calcualtion, we proposed two interpolate method.
    * The first method is
    * */
}



void
AcrobaticCommand::Run()
{
    if(should_exit()){
        return;
    }

    perf_begin(_loop_perf);

    if(_vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_ACROBATIC)
    {
        switch (_vehicle_cmd.acrobatic_name) {
        case 1:

        default: break;

        }



    }









}
































extern "C" __EXPORT int acrobatic_command_main(int argc, char *argv[]);


