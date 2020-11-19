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
using std::vector;
using matrix::Quatf;

AcrobaticCommand::AcrobaticCommand():
    ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
    /**< fetch initial parameter values*/
    _parameter_handles.q_tc = param_find("FW_P_TC");

}

AcrobaticCommand::~AcrobaticCommand()
{
    perf_free(_loop_perf);
}

bool AcrobaticCommand::init()
{
    return true;
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
    //intended blank
}

void
AcrobaticCommand::acro_data_read(const char *filepath) /**< This function needs to run in the init section */
{
    FILE *fp;
    fp = fopen(filepath, "rt");
    int ret;



    /**< Init the parser */
    //matrix::Matrix<float,100,5> acro_data;
    float time;
    Quatf q_temp;
    //float last_time;

    while (EOF != (ret = fscanf(fp, "%f,%f,%f,%f,%f",&time,&q_temp(0),&q_temp(1),&q_temp(2),&q_temp(3))))
    {
        if(ret <= 0){
            fclose(fp);
        }
        _time_v.push_back(time);
        _quat_v.push_back(q_temp);
    }

}

Quatf
AcrobaticCommand::interp_1_d()
{
    /**< In order to accelerate the calcualtion, we proposed two interpolate method.
    * The first method is the nearest approach, the second one is not decided yet.
    * */
    int index = 0;

    for(vector<float>::iterator iter = _time_v.begin(); iter != _time_v.end()-1; iter++, index++)
    {
        if(now < (*iter) && now >(*(iter+1)))
            break;
    }

    return _quat_v[index];

}



void
AcrobaticCommand::Run()
{
    if(should_exit()){
        return;
    }

    perf_begin(_loop_perf);

    /**< If we are not in the acrobatic mode, do nothing */
    if(_vehicle_cmd.command != vehicle_command_s::VEHICLE_CMD_DO_ACROBATIC)
    {
        perf_end(_loop_perf);
        return;
    }
    /**< read the acrobatic command data file */

    switch (_vehicle_cmd.acrobatic_name) {
    /**< loop maneuver */
    case 1: filepath = "loop_data.txt";
    /**< left flight maneuver */
    case 2: filepath = "left_flight_data.txt";
    /**< default read nothing, keep straight flight*/
    default: break;
    }
    PX4_INFO("_vehicle_cmd.acrobatic_name:%d", _vehicle_cmd.acrobatic_name);

    if(file_readed == false)
    {
        acro_data_read(filepath);
        file_readed = true;
    }

    /**< obtain the custom defined acrobatic motion command */
    now = hrt_absolute_time();
    _quat_cmd = interp_1_d();


    /**< Obtain the matrix Tf */
    vehicle_att_poll();
    _quat_err = _quat_cmd - _att_q;

    Quatf _quat_err_t = _quat_err / _tc;

    /**< Tranform matrix Tf
     * p = 2*q0*q1_dot - 2*q1*q0_dot - 2*q2*q3_dot + 2*q3*q2_dot
     * q = 2*q0*q2_dot - 2*q2*q0_dot + 2*q1*q3_dot - 2*q3*q1_dot
     * r = 2*q0*q3_dot - 2*q1*q2_dot + 2*q2*q1_dot - 2*q3*q0_dot
     * */
    _body_setpoint[0] = 2 * _att_q(0) * _quat_err_t(1) - 2 * _att_q(1) * _quat_err_t(0)
            - 2 * _att_q(2) * _quat_err_t(3) + 2 * _att_q(3) * _quat_err_t(2);
    _body_setpoint[1] = 2 * _att_q(0) * _quat_err_t(2) - 2 * _att_q(2) * _quat_err_t(0)
            + 2 * _att_q(1) * _quat_err_t(3) - 2 * _att_q(3) * _quat_err_t(1);
    _body_setpoint[2] = 2 * _att_q(0) * _quat_err_t(3) - 2 * _att_q(1) * _quat_err_t(2)
            + 2 * _att_q(2) * _quat_err_t(1) - 2 * _att_q(3) * _quat_err_t(0);





}




















int AcrobaticCommand::task_spawn(int argc, char *argv[])  /**< generate a task */
{
    AcrobaticCommand *instance = new AcrobaticCommand();

    if(instance){
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if(instance->init())
        {
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


int AcrobaticCommand::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}


int AcrobaticCommand::print_status()
{
    PX4_INFO("Running");
    perf_print_counter(_loop_perf);
    return 0;
}

int AcrobaticCommand::print_usage(const char *reason)
{
    if(reason){
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
                ### Description
                acrobatic_command is the fixed wing acrobatic command generator.

                )DESCR_STR"
                );
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_NAME("acrobatic_command", "controller");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}


extern "C" __EXPORT int acrobatic_command_main(int argc, char *argv[])
{
    return AcrobaticCommand::main(argc, argv);
}


