/**
 * Acrobatic Command
 * This module aims to generate the desired attitude command in quaternion and convert it to p,q,r(axis rotational rates)
 *
 *
 */
/**< Variables */

#include "AcrobaticCommander.h"
#include <px4_log.h>
#include <systemlib/mavlink_log.h>
using namespace time_literals;
using matrix::Quatf;
//#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
//#define TEST_DATA_PATH "./test_data/"
//#else
//#endif

extern "C" __EXPORT int acrobatic_commander_main(int argc, char *argv[])
{
    return AcrobaticCommander::main(argc, argv);
}

AcrobaticCommander::AcrobaticCommander():
    WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
    _loop_perf(perf_alloc(PC_ELAPSED, "acrobatic_L1: cycle"))
{
    //PX4_INFO("AcrobaticCommand::AcrobaticCommand");
    /**< fetch initial parameter values*/
    _acrobatic_cmd.acrobatic_finish = false;
    parameters_update();

}

AcrobaticCommander::~AcrobaticCommander()
{
    //PX4_INFO("AcrobaticCommand::~AcrobaticCommand");
    perf_free(_loop_perf);
}

bool AcrobaticCommander::init()
{
    //PX4_INFO("init~~~");
    if(!_att_sub.registerCallback()){
        return false;
    }
    return true;
}

int
AcrobaticCommander::parameters_update()
{
    //PX4_INFO("AcrobaticCommand::parameter_update");
    return PX4_OK;
}

void
AcrobaticCommander::vehicle_cmd_poll()
{
    _vehicle_cmd_sub.update(&_vehicle_cmd);
}

void
AcrobaticCommander::acro_loop_cmd()
{
    //intended blank
}


void
AcrobaticCommander::vehicle_global_pos_poll()
{
    _global_pos_sub.update(&_global_pos);
}

void
AcrobaticCommander::vehicle_local_pos_poll()
{
    _local_pos_sub.update(&_local_pos);
}

void
AcrobaticCommander::sensor_accel_poll()
{
    _sensor_accel_sub.update(&_sensor_accel);
}
void
AcrobaticCommander::actuator_controls_poll()
{
    _act_con_sub.update(&_actuator_controls);
}

void
AcrobaticCommander::manual_status_poll()
{
    //_man_sub.update(&_man_status);
}
void
AcrobaticCommander::vstatus_poll()
{
    _vstatus_sub.update(&_vstatus);
}




void
AcrobaticCommander::pqr_uvw_acro_data_read()
{
    FILE *fp_pqr_uvw = nullptr;
    mavlink_log_info(&_mavlink_log_pub, "reading file~~~");
    //PX4_INFO("filepath:%s",filepath);
    //filepath = "/fs/microsd/data/loopdata.txt";

    if((fp_pqr_uvw = fopen(filepath_pqr_uvw, "r"))==nullptr)
    {
        mavlink_log_info(&_mavlink_log_pub, "file open error%s",filepath_pqr_uvw);
    }
    else
    {
        mavlink_log_info(&_mavlink_log_pub, "filepath %s open success", filepath_pqr_uvw);
    }
    int ret;
    unsigned long int time;
    float pqr_temp[3];
    float uvw_temp[3];
    pqr_time pqr_temp_t;
    uvw_time uvw_temp_t;
    while(EOF != (ret = fscanf(fp_pqr_uvw, "%ld\t%f\t%f\t%f\t%f\t%f\t%f", &time, &pqr_temp[0], &pqr_temp[1], &pqr_temp[2], &uvw_temp[0], &uvw_temp[1], &uvw_temp[2])))
    {
        if(ret <= 0){
            fclose(fp_pqr_uvw);
        }
        pqr_temp_t.pqr_v[0] = pqr_temp[0];
        pqr_temp_t.pqr_v[1] = pqr_temp[1];
        pqr_temp_t.pqr_v[2] = pqr_temp[2];

        uvw_temp_t.uvw_v[0] = uvw_temp[0];
        uvw_temp_t.uvw_v[1] = uvw_temp[1];
        uvw_temp_t.uvw_v[2] = uvw_temp[2];

        pqr_temp_t.time_v = time;
        uvw_temp_t.time_v = time;
        _pqr_time_l.push_back(pqr_temp_t);
        _uvw_time_l.push_back(uvw_temp_t);
    }
    fclose(fp_pqr_uvw);
}


void
AcrobaticCommander::quat_uvw2xyz()
{
    Quatf quat_temp = _quat_first_acro;
    Array<float, 3> xyz_temp;
    xyz_temp = _xyz_first_acro;

    float p,q,r;
    float u_body, v_body, w_body;
    float q1,q2,q3,q4;
    float x_body,y_body,z_body;
    float delta_t;
    //Initialize the integration
    q1 = quat_temp(0); q2 = quat_temp(1); q3 = quat_temp(2); q4 = quat_temp(3);
    //x_body = xyz_temp[0]; y_body = xyz_temp[1]; z_body = _alt_first_acrobatic; edited by caosu
    //x_body = xyz_temp[0]; y_body = xyz_temp[1]; z_body = xyz_temp[2]; edited by caosu, substitute the _xyz_first_acro with zeros
    x_body = 0; y_body = 0; z_body = 0; // added by caosu 20220302

    for(size_t index_temp=0; index_temp<_pqr_time_l.size()-1; index_temp++)
    {
        p = (float)_pqr_time_l[index_temp].pqr_v[0];
        q = (float)_pqr_time_l[index_temp].pqr_v[1];
        r = (float)_pqr_time_l[index_temp].pqr_v[2];

        delta_t = (float)((_pqr_time_l[index_temp+1].time_v - _pqr_time_l[index_temp].time_v)/(1e6));

        //Integrate the quaternion
        q1 += (float)0.5*(float)(-1*p*q2 - q*q3 - r*q4)*delta_t;
        q2 += (float)0.5*(float)(p*q1 + r*q3 - q*q4)*delta_t;
        q3 += (float)0.5*(float)(q*q1 - r*q2 + p*q4)*delta_t;
        q4 += (float)0.5*(float)(r*q1 + q*q2 - p*q3)*delta_t;

        u_body = (float)_uvw_time_l[index_temp].uvw_v[0];
        v_body = (float)_uvw_time_l[index_temp].uvw_v[1];
        w_body = (float)_uvw_time_l[index_temp].uvw_v[2];

        //Integrate the xyz (inertial frame) Z integrated from the first altitude
        x_body += (float)((q1*q1+q2*q2-q3*q3-q4*q4)*u_body + (2*(q2*q3-q1*q4))*v_body + (2*(q2*q4+q1*q3))*w_body)*delta_t;
        y_body += (float)((2*(q2*q3+q1*q4))*u_body + (q1*q1-q2*q2+q3*q3-q4*q4)*v_body + (2*(q3*q4-q1*q2))*w_body)*delta_t;
        //z_body -= (float)((2*(q2*q4-q1*q3))*u_body + (2*(q3*q4+q1*q2))*v_body + (q1*q1-q2*q2-q3*q3+q4*q4)*w_body)*delta_t; edited by caosu
        z_body += (float)((2*(q2*q4-q1*q3))*u_body + (2*(q3*q4+q1*q2))*v_body + (q1*q1-q2*q2-q3*q3+q4*q4)*w_body)*delta_t;

        quat_temp(0) = q1; quat_temp(1) = q2; quat_temp(2) = q3; quat_temp(3) = q4;
        xyz_temp[0] = x_body; xyz_temp[1] = y_body; xyz_temp[2] = z_body;

        //Store the attitude and position command
        quat_time quat_t_temp;
        quat_t_temp.quat_v = quat_temp;
        quat_t_temp.time_v = _pqr_time_l[index_temp].time_v;
        _quat_time_l.push_back(quat_t_temp);
        xyz_time xyz_t_temp;
        xyz_t_temp.xyz_v = xyz_temp;
        xyz_t_temp.time_v = _uvw_time_l[index_temp].time_v;
        _xyz_time_l.push_back(xyz_t_temp);
    }
}


Quaternion<float>
AcrobaticCommander::interp_1_d_quat()
{
    /**< In order to accelerate the calcualtio_quat_time_ln, we proposed two interpolate method.
    * The first method is the nearest approach, the second one is not decided yet.
    * */
    size_t index;
    hrt_abstime _now;
    _now = hrt_absolute_time();
    for(index = 0; index < _quat_time_l.size(); index++)
    {
        if(index == _quat_time_l.size()-1)
        {
            _finish_count ++;
            break;
        }
        else if(((_now-_time_first_acrobatic) >= (_quat_time_l[index].time_v)) && ((_now-_time_first_acrobatic) < (_quat_time_l[index+1].time_v)))
        {
            break;
        }

    }
    //mavlink_log_info(&_mavlink_log_pub, "index = %d", index);
    return _quat_time_l[index].quat_v ;
}

Array<float,3>
AcrobaticCommander::interp_1_d_xyz()
{
    size_t index;
    hrt_abstime _now;
    _now = hrt_absolute_time();
    for(index = 0; index < _xyz_time_l.size(); index++)
    {
        if(index == _xyz_time_l.size()-1)
        {
            //_finish_count++;
            break;
        }
        else if(((_now-_time_first_acrobatic) >= (_xyz_time_l[index].time_v)) && ((_now-_time_first_acrobatic) < (_xyz_time_l[index+1].time_v)))
        {
            break;
        }
    }
    return _xyz_time_l[index].xyz_v;
}

matrix::Matrix<float, 8, 1>
AcrobaticCommander::interp_1_d_pqr_uvw()
{
    size_t index;
    for(index=0; index<_pqr_time_l.size()-1; index++)
    {
        if(index==_pqr_time_l.size()-1)
        {
            break;
        }
        else if(((now-_time_first_acrobatic) >= (_pqr_time_l[index].time_v)) && ((now-_time_first_acrobatic) < (_pqr_time_l[index+1].time_v)))
        {
            break;
        }
    }
    matrix::Matrix<float, 8, 1> _pqr_uvw_com;
    for(size_t i=0; i<3; i++)
    {
        _pqr_uvw_com(i+1, 0) = float(_pqr_time_l[index].pqr_v[i]);
        _pqr_uvw_com(i+5, 0) = float(_uvw_time_l[index].uvw_v[i]);
    }
    return _pqr_uvw_com;
}


void
AcrobaticCommander::Run()
{
        perf_begin(_loop_perf);
        //mavlink_log_info(&_mavlink_log_pub, "TEST_DATA_PATH:%s",TEST_DATA_PATH);
        //PX4_INFO("Running ~~~");
        //mavlink_log_info(&_mavlink_log_pub, "Running");
        if(_att_sub.update(&_att))
        {
            now = hrt_absolute_time();
            _att_q(0) = _att.q[0];
            _att_q(1) = _att.q[1];
            _att_q(2) = _att.q[2];
            _att_q(3) = _att.q[3];
            _start_count ++;

            /**<----------------------    Poll the reference pose    ------------------------------*/
            //Obtain the vehicle command to know whether enters the acrobatic point
            vehicle_cmd_poll();
            //Obtain the global position of the vehicle at this time
            vehicle_global_pos_poll();
            //Obtain the local position of the vehicle at this time
            vehicle_local_pos_poll();

            /*<-----------------------     Enter acrobatic waypoint     --------------------------*/
            // Enter the acrobatic point
            if(_vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_ACROBATIC && _start_count > 0)
            {

                /**<                read the acrobatic command data file                    */

                //mavlink_log_info(&_mavlink_log_pub, "TEST_DATA_PATH");
                switch (_vehicle_cmd.acrobatic_name) {
                /**< loop maneuver */
                case 0:
                    //filepath_pqr_uvw = "/fs/microsd/data/level_pqr_uvw.txt";
                    filepath_pqr_uvw = "/fs/microsd/data/immelman_pqr_uvw.txt";
                    //filepath_pqr_uvw = "/fs/microsd/data/fast_climb_pqr_uvw.txt";
                    break;
                /**< Immelman maneuver */
                case 1:
                    filepath_pqr_uvw = "/fs/microsd/data/level_pqr_uvw.txt";
                    //filepath_pqr_uvw = "/fs/microsd/data/fast_climb_pqr_uvw.txt";
                    break;
                /**< default read nothing, keep straight flight*/
                default: break;
                }
                //PX4_INFO("_vehicle_cmd.acrobatic_name:%d", _vehicle_cmd.acrobatic_name);

                if(_time_first_acrobatic == 0)
                {
                    //initial altitude
                    _alt_first_acrobatic = _global_pos.alt; //The altitude is z*-1 + alt_init
                    _alt_sp_acrobatic = _alt_first_acrobatic;
                    //initial quaternion
                    _quat_first_acro = _att_q;
                    //initial position
                    _xyz_first_acro[0] = _local_pos.x;
                    _xyz_first_acro[1] = _local_pos.y;
                    _xyz_first_acro[2] = _local_pos.z;

                    /**<       Obtain the acrobatic command     **/
                    pqr_uvw_acro_data_read();
                    //pqr_uvw_acro_data_read();
                    quat_uvw2xyz();
                    _time_first_acrobatic = hrt_absolute_time(); //much earlier than the publishing time
                }

                if(file_readed == false)
                {
                    //quat_uvw2xyz();
                    file_readed = true;
                }



                /**<--------------------    Obtain the dq command   ---------------*/
                Quaternion<float> _quat_d;
                Array<float,3> _xyz_d;
                _quat_d = interp_1_d_quat();
                _xyz_d = interp_1_d_xyz();

                DualQuaternion<float> _dq_d;
                _dq_d = Convert_quat_xyz_2_DQ(_xyz_d, _quat_d);


                 /**<-------------------  Obtain the twist command  ---------------*/
                Quaternion<float> _quat_val;
                _quat_val = _att_q;
                Array<float, 3> _xyz_val;
                _xyz_val[0] = _local_pos.x - _xyz_first_acro[0];
                _xyz_val[1] = _local_pos.y - _xyz_first_acro[1];
                _xyz_val[2] = _local_pos.z - _xyz_first_acro[2];
                DualQuaternion<float> _dq_val;
                //_dq_val = Convert_quat_xyz_2_DQ(_xyz_val, _quat_val); edited by caosu
                //_dq_val = Convert_quat_xyz_2_DQ(_xyz_val, _quat_d);
                //Obtain the reference twist
                matrix::Matrix<float, 8, 1> _ref_twist;
                _ref_twist = interp_1_d_pqr_uvw();


                //Read into the acrobatic setpoint message
                _acro_setpoint.timestamp = hrt_absolute_time();
                _acro_setpoint.xyz_first_acro[0] = _xyz_first_acro[0];
                _acro_setpoint.xyz_first_acro[1] = _xyz_first_acro[1];
                _acro_setpoint.xyz_first_acro[2] = _xyz_first_acro[2];
                for(size_t i=0; i<4; i++)
                {
                    //_acro_setpoint.dq_d_real[i] = _dq_d.m_real(i);
                    //_acro_setpoint.dq_d_dual[i] = _dq_d.m_dual(i);
                    _acro_setpoint.quat_d[i] = _quat_d(i);
                }
                for(size_t i=0; i<3; i++)
                {
                    //_acro_setpoint.dq_d_real[i] = _dq_d.m_real(i);
                    //_acro_setpoint.dq_d_dual[i] = _dq_d.m_dual(i);
                    _acro_setpoint.xyz_d[i] = _xyz_d[i];
                }
                for(size_t i=0; i<8; i++)
                {
                    _acro_setpoint.ref_twist[i] = _ref_twist(i,0);
                }
                if(_finish_count >= 50)
                {
                     _acrobatic_cmd.acrobatic_finish = true;
                     _acro_setpoint.acrobatic_finish = true;
                }
                _acro_setpoint.setpoint_generated = true;
                _acro_setpoint_pub.publish(_acro_setpoint);


                //mavlink_log_info(&_mavlink_log_pub, "Twist Command(PQR): %.2lf\t%.2lf\t%.2lf\t%.2lf",
                //                (double)_com_twist(0,0),(double)_com_twist(1,0),
                //                (double)_com_twist(2,0),(double)_com_twist(3,0));
                //mavlink_log_info(&_mavlink_log_pub, "Twist Command(UVW): %.2lf\t%.2lf\t%.2lf",
                //                (double)_com_twist(5,0),(double)_com_twist(6,0),(double)_com_twist(7,0));
                //float w = _local_pos.vz;
                //_alt_sp_acrobatic += -1 * (float)((now-time_prev)/1e6) * w; //transfer according to the frame
                //_alt_sp_acrobatic = _alt_first_acrobatic - _xyz_d[2];
            }
            /*Storage the demonstration*/
            Twist_Demon_Storage();
            time_prev = now;
        }
        perf_end(_loop_perf);
    //}
}

void AcrobaticCommander::Twist_Demon_Storage()
{
    /*This function is to storage the twist during demonstration*/
    // Calculate the true
    // flight velocity v_sp and w_sp
    matrix::Vector3f ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
    //calculate the velocity in body frame
    // Velocity in body frame
    const matrix::Dcmf R_to_body(Quatf(_att.q).inversed());
    const matrix::Vector3f vel = R_to_body * matrix::Vector3f(ground_speed(0), ground_speed(1), ground_speed(2));

    // the linear velocities in the body frame
    float _u_real_storage = vel(0);
    float _v_real_storage = vel(1);
    float _w_real_storage = vel(2);

    // the angular velocities in the body frame
    //vehicle_angular_velocity_s angular_velocity{};
    vehicle_angular_velocity_s angular_velocity;
    _vehicle_rates_sub.copy(&angular_velocity);
    float rollspeed_storage = angular_velocity.xyz[0];
    float pitchspeed_storage = angular_velocity.xyz[1];
    float yawspeed_storage = angular_velocity.xyz[2];

    acrobatic_demo_s _acro_demo;
    _acro_demo.timestamp = hrt_absolute_time();
    _acro_demo.angular_velocity[0] = rollspeed_storage;
    _acro_demo.angular_velocity[1] = pitchspeed_storage;
    _acro_demo.angular_velocity[2] = yawspeed_storage;
    _acro_demo.linear_velocity[0] = _u_real_storage;
    _acro_demo.linear_velocity[1] = _v_real_storage;
    _acro_demo.linear_velocity[2] = _w_real_storage;
    _acro_demo_pub.publish(_acro_demo);
}



DualQuaternion<float> AcrobaticCommander::Convert_quat_xyz_2_DQ(const Array<float,3> _xyz, const Quaternion<float> _att_d){
    DualQuaternion<float> _hat_q_n_d;
    _hat_q_n_d.m_real = _att_d;
    Quaternion<float> _xyz_d;
    for(size_t i=0;i<3;i++){
        _xyz_d(i+1) = _xyz[i];
    }
    _xyz_d = _xyz_d * _att_d;
    _hat_q_n_d.m_dual = float(0.5) * _xyz_d;
    return _hat_q_n_d;
}

int AcrobaticCommander::task_spawn(int argc, char *argv[])  /**< generate a task */
{

    AcrobaticCommander *instance = new AcrobaticCommander();

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

int AcrobaticCommander::custom_command(int argc, char *argv[])
{
    //PX4_INFO("custom_command~~");
    return print_usage("unknown command");
}


int AcrobaticCommander::print_status()
{
    //PX4_INFO("print_status");
    perf_print_counter(_loop_perf);
    return 0;
}

int AcrobaticCommander::print_usage(const char *reason)
{
    if(reason){
        PX4_WARN("%s\n", reason);
    }
    //PX4_INFO("print_usage");

    PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
                ### Description
                AcrobaticCommander is the fixed wing acrobatic command generator.

                )DESCR_STR"
                );
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_NAME("AcrobaticCommander", "controller");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}
