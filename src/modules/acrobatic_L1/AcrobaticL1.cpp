/**
 * Acrobatic Command
 * This module aims to generate the desired attitude command in quaternion and convert it to p,q,r(axis rotational rates)
 *
 *
 */
/**< Variables */

#include "AcrobaticL1.h"
#include <px4_log.h>
#include <systemlib/mavlink_log.h>
using namespace time_literals;
using matrix::Quatf;
//#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
//#define TEST_DATA_PATH "./test_data/"
//#else
//#endif

extern "C" __EXPORT int acrobatic_L1_main(int argc, char *argv[])
{
    return AcrobaticL1::main(argc, argv);
}

AcrobaticL1::AcrobaticL1():
    WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
    _loop_perf(perf_alloc(PC_ELAPSED, "acrobatic_L1: cycle"))
{
    //PX4_INFO("AcrobaticCommand::AcrobaticCommand");
    /**< fetch initial parameter values*/
    _acrobatic_cmd.acrobatic_finish = false;
    parameters_update();

}

AcrobaticL1::~AcrobaticL1()
{
    //PX4_INFO("AcrobaticCommand::~AcrobaticCommand");
    perf_free(_loop_perf);
}

bool AcrobaticL1::init()
{
    //PX4_INFO("init~~~");
    if(!_att_sub.registerCallback()){
        return false;
    }
    return true;
}

int
AcrobaticL1::parameters_update()
{
    //PX4_INFO("AcrobaticCommand::parameter_update");
    return PX4_OK;
}

void
AcrobaticL1::vehicle_cmd_poll()
{
    _vehicle_cmd_sub.update(&_vehicle_cmd);
}

void
AcrobaticL1::acro_loop_cmd()
{
    //intended blank
}


void
AcrobaticL1::vehicle_global_pos_poll()
{
    _global_pos_sub.update(&_global_pos);
}

void
AcrobaticL1::vehicle_local_pos_poll()
{
    _local_pos_sub.update(&_local_pos);
}

void
AcrobaticL1::sensor_accel_poll()
{
    _sensor_accel_sub.update(&_sensor_accel);
}
void
AcrobaticL1::actuator_controls_poll()
{
    _act_con_sub.update(&_actuator_controls);
}

void
AcrobaticL1::manual_status_poll()
{
    //_man_sub.update(&_man_status);
}
void
AcrobaticL1::vstatus_poll()
{
    _vstatus_sub.update(&_vstatus);
}
void
AcrobaticL1::acc_data_read() /**< This function needs to run in the init section */
{
    FILE *fp_acc = nullptr;
    int ret;
    unsigned long int time;

    mavlink_log_info(&_mavlink_log_pub, "reading file~~~");
    //PX4_INFO("filepath:%s",filepath);
    //filepath = "/fs/microsd/data/loopdata.txt";

    if((fp_acc = fopen(filepath_acc, "r"))==nullptr)
    {
        mavlink_log_info(&_mavlink_log_pub, "file open error%s",filepath_acc);
    }
    else
    {
        mavlink_log_info(&_mavlink_log_pub, "filepath %s open success", filepath_acc);
    }

    float acc_temp[3];

    acc_time acc_temp_t;

    while(EOF != (ret = fscanf(fp_acc, "%ld, \t%f, \t%f,\t%f", &time, &acc_temp[0], &acc_temp[1], &acc_temp[2])))
    {
        if(ret <= 0){
            fclose(fp_acc);
        }
        acc_temp_t.acc_v[0] = acc_temp[0];
        acc_temp_t.acc_v[1] = acc_temp[1];
        acc_temp_t.acc_v[2] = acc_temp[2];

        acc_temp_t.time_v = time;
        _acc_time_l.push_back(acc_temp_t);
    }
    fclose(fp_acc);
    mavlink_log_info(&_mavlink_log_pub, "acc readed succesfully! length: %d", _acc_time_l.size());

}

Array<float,3>
AcrobaticL1::interp_1_d_acc()
{
    size_t index;
    for(index = 0; index < _acc_time_l.size(); index++)
    {
        if(index == _acc_time_l.size()-1)
        {
            _finish_count++;
        }
        else if(((now-_time_first_acrobatic) >= (_acc_time_l[index].time_v)) && ((now-_time_first_acrobatic) < (_acc_time_l[index+1].time_v)))
        {
            break;
        }
    }
    return _acc_time_l[index].acc_v;
}


void
AcrobaticL1::Run()
{
        perf_begin(_loop_perf);
        //mavlink_log_info(&_mavlink_log_pub, "TEST_DATA_PATH:%s",TEST_DATA_PATH);
        //PX4_INFO("Running ~~~");
        //mavlink_log_info(&_mavlink_log_pub, "Running");
        if(_att_sub.update(&_att))
        {
            //PX4_INFO("Acrobatic Command~~~~~~~~~~");
            _start_count ++;
            vehicle_cmd_poll();

            _att_q(0) = _att.q[0]; _att_q(1) = _att.q[1];
            _att_q(2) = _att.q[2]; _att_q(3) = _att.q[3];

            vehicle_global_pos_poll();
            vehicle_local_pos_poll(); //read the current velocity
            //_sensor_com_sub.update(&_sensor_com);
            now = hrt_absolute_time();
            //PX4_INFO("Running ~~~");
            //mavlink_log_info(&_mavlink_log_pub, "accel_x:%f\taccel_y:%f\taccel_y%f\n",
            //                 (double)_sensor_com.accelerometer_m_s2[0],
            //                (double)_sensor_com.accelerometer_m_s2[1],
            //                (double)_sensor_com.accelerometer_m_s2[2]);

            if(_vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_ACROBATIC && _start_count > 0)
            {
                filepath_acc = "/fs/microsd/data/acc_cmd.txt";
                if(file_readed == false)
                {
                    //Obtain the acrobatic command
                    acc_data_read();
                    //pqr_uvw_acro_data_read();
                    file_readed = true;
                }

                if(_time_first_acrobatic == 0)
                {
                    _time_first_acrobatic = now;
                    //initial altitude
                    _alt_first_acrobatic = _global_pos.alt; //The altitude is z*-1 + alt_init
                    _alt_sp_acrobatic = _alt_first_acrobatic;
                }
                //_acrobaticL1_cmd.timestamp = hrt_absolute_time();
                _acrobatic_cmd.timestamp = hrt_absolute_time();
                //_acrobaticL1_cmd.acc_y_setpoint = 0;
                //_acrobaticL1_cmd.acc_z_setpoint = -9.8;
                //_acrobatic_cmd.acrobatic_finish = 0;
                //_finish_count ++;
                if(_finish_count > 500)
                {
                    //_acrobaticL1_cmd.acrobatic_finish = 1;
                    _acrobatic_cmd.acrobatic_finish = true;
                }
                float w = _local_pos.vz;

                /*The old version (altitude command)*/
                _alt_sp_acrobatic += -1 * (float)((now-time_prev)/1e6) * w; //transfer according to the frame
                //_acrobatic_cmd.alt_sp_acrobatic = _alt_sp_acrobatic;
                Array<float, 3> acc_temp;
                acc_temp = interp_1_d_acc();

                _acrobatic_cmd.accel_z_cmd = acc_temp[2];//float(-9.8 + 2*sin(hrt_absolute_time()/1e6));
                /*The new version (altitude command)*/
               // _alt_sp_acrobatic = _xyz_cmd[2];
               // _acrobatic_cmd.alt_sp_acrobatic = _alt_sp_acrobatic;
                _acrobatic_cmd.body_rates_cmd[0] = 0;
                _acrobatic_cmd.body_rates_cmd[1] = 0;
                _acrobatic_cmd.body_rates_cmd[2] = 0;

                _acrobatic_cmd.euler_cmd[0] = asinf(2*(_att_q(0)*_att_q(2)-_att_q(3)*_att_q(1)));




                //_acroL1_cmd_pub.publish(_acrobaticL1_cmd);
                _acro_cmd_pub.publish(_acrobatic_cmd);
                //PX4_INFO("-------------------------");
                //PX4_INFO("timestamp:%lld",_acrobatic_cmd.timestamp);

            }
            time_prev = now;
        }
        perf_end(_loop_perf);
    //}
}

int AcrobaticL1::task_spawn(int argc, char *argv[])  /**< generate a task */
{

    AcrobaticL1 *instance = new AcrobaticL1();

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

    return 0;

}

int AcrobaticL1::custom_command(int argc, char *argv[])
{
    //PX4_INFO("custom_command~~");
    return print_usage("unknown command");
}


int AcrobaticL1::print_status()
{
    //PX4_INFO("print_status");
    perf_print_counter(_loop_perf);
    return 0;
}

int AcrobaticL1::print_usage(const char *reason)
{
    if(reason){
        PX4_WARN("%s\n", reason);
    }
    //PX4_INFO("print_usage");

    PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
                ### Description
                acrobatic_L1 is the fixed wing acrobatic command generator.   

                )DESCR_STR"
                );
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_NAME("acrobatic_L1", "controller");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}
