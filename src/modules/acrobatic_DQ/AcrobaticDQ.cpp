/**
 * Acrobatic Command
 * This module aims to generate the desired attitude command in quaternion and convert it to p,q,r(axis rotational rates)
 *
 *
 */
/**< Variables */

#include "AcrobaticDQ.h"
#include <px4_log.h>
#include <systemlib/mavlink_log.h>
using namespace time_literals;
using matrix::Quatf;
//#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
//#define TEST_DATA_PATH "./test_data/"
//#else
//#endif

extern "C" __EXPORT int acrobatic_DQ_main(int argc, char *argv[])
{
    return AcrobaticDQ::main(argc, argv);
}

AcrobaticDQ::AcrobaticDQ():
    WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
    _loop_perf(perf_alloc(PC_ELAPSED, "acrobatic_L1: cycle"))
{
    //PX4_INFO("AcrobaticCommand::AcrobaticCommand");
    /**< fetch initial parameter values*/
    _acrobatic_cmd.acrobatic_finish = false;
    parameters_update();

}

AcrobaticDQ::~AcrobaticDQ()
{
    //PX4_INFO("AcrobaticCommand::~AcrobaticCommand");
    perf_free(_loop_perf);
}

bool AcrobaticDQ::init()
{
    //PX4_INFO("init~~~");
    if(!_att_sub.registerCallback()){
        return false;
    }
    return true;
}

int
AcrobaticDQ::parameters_update()
{
    //PX4_INFO("AcrobaticCommand::parameter_update");
    return PX4_OK;
}

void
AcrobaticDQ::vehicle_cmd_poll()
{
    _vehicle_cmd_sub.update(&_vehicle_cmd);
}

void
AcrobaticDQ::acro_loop_cmd()
{
    //intended blank
}


void
AcrobaticDQ::vehicle_global_pos_poll()
{
    _global_pos_sub.update(&_global_pos);
}

void
AcrobaticDQ::vehicle_local_pos_poll()
{
    _local_pos_sub.update(&_local_pos);
}

void
AcrobaticDQ::sensor_accel_poll()
{
    _sensor_accel_sub.update(&_sensor_accel);
}
void
AcrobaticDQ::actuator_controls_poll()
{
    _act_con_sub.update(&_actuator_controls);
}

void
AcrobaticDQ::manual_status_poll()
{
    //_man_sub.update(&_man_status);
}
void
AcrobaticDQ::vstatus_poll()
{
    _vstatus_sub.update(&_vstatus);
}




void
AcrobaticDQ::pqr_uvw_acro_data_read()
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
AcrobaticDQ::quat_uvw2xyz()
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
AcrobaticDQ::interp_1_d_quat()
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
AcrobaticDQ::interp_1_d_xyz()
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
AcrobaticDQ::interp_1_d_pqr_uvw()
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
AcrobaticDQ::Run()
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

            /* First we want to test the dual quaternion operator*/
            /*DualQuaternion<float> dq1, dq2;
            dq1.m_real(0) = 0.8816; dq1.m_real(1) = 0.3862; dq1.m_real(2) = 0.1853; dq1.m_real(3) = 0.1981;
            dq1.m_dual(0) = -1.9667; dq1.m_dual(1) = -2.6197; dq1.m_dual(2) = -2.3757; dq1.m_dual(3) = 16.0818;

            dq2.m_real(0) = 0.9093; dq2.m_real(1) = 0.2831; dq2.m_real(2) = 0.2969; dq2.m_real(3) = 0.0704;
            dq2.m_dual(0) = -7.6366; dq2.m_dual(1) = 9.5204; dq2.m_dual(2) = 12.9277; dq2.m_dual(3) = 5.8242;

            DualQuaternion<float> _hat_q_e;
            _hat_q_e = dq1.conjugate() * dq2;
            DualQuaternion<float> _pose_err;
            _pose_err = _hat_q_e.DQError_truepos();
            matrix::Matrix<float, 8, 8> _Hat_G_e;
            _Hat_G_e = Hat_G_mat(_hat_q_e);
            matrix::Matrix<float, 8, 8> _inv_Hat_G_e;
            _inv_Hat_G_e = inv_Hat_G_mat_cal(_Hat_G_e);
            matrix::Matrix<float, 8, 1> _pose_err_val;
            for(size_t i=0; i<4; i++){
               _pose_err_val(i,0) = _pose_err.m_real(i);
               _pose_err_val(i+4,0) = _pose_err.m_dual(i);
            }
            matrix::Matrix<float, 8, 1> _omega_e;
            matrix::Matrix<float, 8, 8> _mat_gain;
            _mat_gain(1,1) = -0.8;
            _mat_gain(2,2) = -0.8;
            _mat_gain(3,3) = -0.8;
            _mat_gain(5,5) = -0.1;
            _mat_gain(6,6) = -0.1;
            _mat_gain(7,7) = -0.1;
            _omega_e = _inv_Hat_G_e * (_mat_gain*_pose_err_val);
            mavlink_log_info(&_mavlink_log_pub, "Omega error: %.2lf\t%.2lf\t%.2lf\t%.2lf",
                            (double)_omega_e(4,0),(double)_omega_e(5,0),(double)_omega_e(6,0),(double)_omega_e(7,0));*/

            //DualQuaternion<float> dq_e, dq_test;
            //dq_test = dq1 + dq2;
            //dq_test = dq1.conjugate() * dq2;
            //dq_e = dq1.conjugate()*dq2;
            //dq_test = dq_e.DQError_truepos();
//            matrix::Matrix<float, 8, 1> dq_test;
//            for(size_t i=0; i<4; i++){
//                dq_test(i,0) = dq1.m_real(i);
//            }
//            for(size_t i=0; i<4; i++){
//                dq_test(i+4,0) = dq1.m_dual(i);
//            }
//            matrix::Matrix<float, 8, 8> hat_G_e;
//            hat_G_e = Hat_G_mat(dq1);
//            //out_S_q(0,0)=1; out_S_q(1,1)=2; out_S_q(2,2)=3; out_S_q(3,3)=4;
//            hat_G_e = inv_Hat_G_mat_cal(hat_G_e);

//            dq_test = hat_G_e * dq_test;

            // Print the value of q_test
            //mavlink_log_info(&_mavlink_log_pub, "q_test: %lf\t%lf\t%lf\t%lf",
            //                   double(out_S_q(1,0)),double(out_S_q(1,1)),double(out_S_q(1,2)),double(out_S_q(1,3)));
            //mavlink_log_info(&_mavlink_log_pub, "q_test: %lf\t%lf\t%lf",
            //                 double(mat_a(0,0)),double(mat_a(1,1)),double(mat_a(2,2)));
            //mavlink_log_info(&_mavlink_log_pub, "q_test: %lf\t%lf\t%lf\t%lf",
            //                 double(dq_test(4,0)),double(dq_test(5,0)),double(dq_test(6,0)),double(dq_test(7,0)));

            /**<                Poll the reference pose                                 */
            vehicle_cmd_poll();
            vehicle_global_pos_poll();
            // in the dmp function
            vehicle_local_pos_poll();
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
                    //Obtain the acrobatic command
                    pqr_uvw_acro_data_read();
                    //pqr_uvw_acro_data_read();
                    quat_uvw2xyz();
                    _time_first_acrobatic = hrt_absolute_time(); //much earlier than the publishing time
                }

                if(file_readed == false)
                {

                    //quat_uvw2xyz();
                    file_readed = true;

                    //mavlink_log_info(&_mavlink_log_pub, "Acrobatic Command Readed.");
                    //mavlink_log_info(&_mavlink_log_pub, "Acrobatic Target Attitude: %.2lf\t%.2lf\t%.2lf\t%.2lf",
                    //                 (double)_quat_time_l[_quat_time_l.size()-1].quat_v(0),(double)_quat_time_l[_quat_time_l.size()-1].quat_v(1),
                    //                 (double)_quat_time_l[_quat_time_l.size()-1].quat_v(2),(double)_quat_time_l[_quat_time_l.size()-1].quat_v(3));
                    //mavlink_log_info(&_mavlink_log_pub, "Acrobatic Target Position: %.2lf\t%.2lf\t%.2lf",
                    //                 (double)_xyz_time_l[_xyz_time_l.size()-1].xyz_v[0],
                    //                 (double)_xyz_time_l[_xyz_time_l.size()-1].xyz_v[1],
                    //                 (double)_xyz_time_l[_xyz_time_l.size()-1].xyz_v[2]);
                }

                if(_finish_count >= 50)
                {
                    _acrobatic_cmd.acrobatic_finish = true;
                }

                /*<                   Obtain the dq command                */
                Quaternion<float> _quat_d;
                Array<float,3> _xyz_d;
                _quat_d = interp_1_d_quat();
                _xyz_d = interp_1_d_xyz();

                DualQuaternion<float> _dq_d;
                _dq_d = Convert_quat_xyz_2_DQ(_xyz_d, _quat_d);

                 /*<                  Obtain the twist command             */
                Quaternion<float> _quat_val;
                _quat_val = _att_q;
                Array<float, 3> _xyz_val;
                _xyz_val[0] = _local_pos.x - _xyz_first_acro[0];
                _xyz_val[1] = _local_pos.y - _xyz_first_acro[1];
                _xyz_val[2] = _local_pos.z - _xyz_first_acro[2];
                DualQuaternion<float> _dq_val;
                //_dq_val = Convert_quat_xyz_2_DQ(_xyz_val, _quat_val); edited by caosu
                _dq_val = Convert_quat_xyz_2_DQ(_xyz_val, _quat_d);

                matrix::Matrix<float, 8, 1> _com_twist, _ref_twist;
                _ref_twist = interp_1_d_pqr_uvw();
                _com_twist = Twist_Command_Gen(_dq_d, _dq_val, _ref_twist);

                /*for(size_t i=0; i<8; i++)
                {
                    _acro_debug.dq_error[i] = _pos_err_val_tmp[i];
                }
                for(size_t i=0; i<64; i++)
                {
                    _acro_debug.hat_g_inv[i] = _Hat_G_inv_tmp[i];
                }*/

                //mavlink_log_info(&_mavlink_log_pub, "Twist Command(PQR): %.2lf\t%.2lf\t%.2lf\t%.2lf",
                //                (double)_com_twist(0,0),(double)_com_twist(1,0),
                //                (double)_com_twist(2,0),(double)_com_twist(3,0));
                //mavlink_log_info(&_mavlink_log_pub, "Twist Command(UVW): %.2lf\t%.2lf\t%.2lf",
                //                (double)_com_twist(5,0),(double)_com_twist(6,0),(double)_com_twist(7,0));
                //float w = _local_pos.vz;
                //_alt_sp_acrobatic += -1 * (float)((now-time_prev)/1e6) * w; //transfer according to the frame
                _alt_sp_acrobatic = _alt_first_acrobatic - _xyz_d[2];

                /*----------------------------Send Command TECS-----------------------------------------*/

                _acrobatic_cmd.timestamp = hrt_absolute_time();
                _acrobatic_cmd.do_acrobatic = true;
                _acrobatic_cmd.alt_sp_acrobatic = _alt_sp_acrobatic;//_xyz_d[2];
                _acrobatic_cmd.airsp_sp = 60;//_com_twist(5,0);
                _acrobatic_cmd.v_sp = 0;//_com_twist(6,0); edited by caosu
                //_acrobatic_cmd.w_sp = 1.5 + sin(float(now/(1*1e6)));//_com_twist(7,0); edited by caosu
                _acrobatic_cmd.w_sp = _com_twist(7,0);
                //_acrobatic_cmd.euler_cmd[0] = asinf(2*(_dq_d.m_real(0)*_dq_d.m_real(2)-_dq_d.m_real(3)*_dq_d.m_real(1)));
                _acrobatic_cmd.euler_cmd[0] = asinf(2*(_att_q(0)*_att_q(2)-_att_q(3)*_att_q(1)));

                _acrobatic_cmd.body_rates_cmd[0] = 0;//_com_twist(1,0);
                _acrobatic_cmd.body_rates_cmd[1] = 0;//_com_twist(2,0);
                _acrobatic_cmd.body_rates_cmd[2] = 0;//_com_twist(3,0);

                /*--------------------------------------------------------------------------------------*/

                // Calculate the true
                // flight velocity v_sp and w_sp
                matrix::Vector3f ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
                //calculate the velocity in body frame
                // Velocity in body frame
                const matrix::Dcmf R_to_body(Quatf(_att.q).inversed());
                const matrix::Vector3f vel = R_to_body * matrix::Vector3f(ground_speed(0), ground_speed(1), ground_speed(2));

                //const float _vel_x_real = vel(0);
                _v_real = vel(1);
                _w_real = vel(2);

                _acrobatic_cmd.v_real = _v_real;
                _acrobatic_cmd.w_real = _w_real;
                _acrobatic_cmd.u_sp_cal = _u_cal_tmp;
                _acrobatic_cmd.v_sp_cal = _v_cal_tmp;
                _acrobatic_cmd.w_sp_cal = _w_cal_tmp;
                _acrobatic_cmd.quaternion_cmd[0] = _dq_d.m_real(0);
                _acrobatic_cmd.quaternion_cmd[1] = _dq_d.m_real(1);
                _acrobatic_cmd.quaternion_cmd[2] = _dq_d.m_real(2);
                _acrobatic_cmd.quaternion_cmd[3] = _dq_d.m_real(3);
                _acro_cmd_pub.publish(_acrobatic_cmd);
                mavlink_log_info(&_mavlink_log_pub, "Publishing the acrobatic command ~~~");

            }
            /*Storage the demonstration*/
            Twist_Demon_Storage();
            time_prev = now;
        }
        perf_end(_loop_perf);
    //}
}

void AcrobaticDQ::Twist_Demon_Storage()
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


matrix::Matrix<float, 8, 1> AcrobaticDQ::Twist_Command_Gen(const DualQuaternion<float> _dual_quat_cmd,
                                                           const DualQuaternion<float> _dual_quat_val,
                                                           const matrix::Matrix<float, 8, 1> _ref_twist){ //verified
     DualQuaternion<float> _hat_q_e;
     _hat_q_e = _dual_quat_cmd.conjugate() * _dual_quat_val;
     DualQuaternion<float> _pose_err;
     _pose_err = _hat_q_e.DQError_truepos();
     matrix::Matrix<float, 8, 8> _Hat_G_e;
     _Hat_G_e = Hat_G_mat(_hat_q_e);
     matrix::Matrix<float, 8, 8> _inv_Hat_G_e;
     _inv_Hat_G_e = inv_Hat_G_mat_cal(_Hat_G_e);
     matrix::Matrix<float, 8, 1> _pose_err_val;
     for(size_t i=0; i<4; i++){
        _pose_err_val(i,0) = _pose_err.m_real(i);
        _pose_err_val(i+4,0) = _pose_err.m_dual(i);
     }
     matrix::Matrix<float, 8, 1> _omega_e;
     matrix::Matrix<float, 8, 8> _mat_gain;
     for(size_t i=0; i<8; i++)
     {
         _pos_err_val_tmp[i] = _pose_err_val(i,0);
     }
     _mat_gain(1,1) = -0.1;
     _mat_gain(2,2) = -0.1;
     _mat_gain(3,3) = -0.1;
     _mat_gain(5,5) = -0.1;
     _mat_gain(6,6) = -0.1;
     _mat_gain(7,7) = -0.1;
     for(size_t i=0; i<8; i++)
     {
         _pose_err_val(i,0) *= _mat_gain(i,i);
         _pose_err_val(i,0) *= -1;
     }
     for(size_t i=0; i<8; i++)
     {
         for(size_t j=0; j<8; j++)
         {
             _Hat_G_inv_tmp[i*8+j] = _inv_Hat_G_e(i,j);
         }
     }
     //_omega_e = float(1) * _inv_Hat_G_e * (_mat_gain*_pose_err_val);
     _omega_e = _inv_Hat_G_e * _pose_err_val;

     matrix::Matrix<float, 8, 1> _com_twist;
     //_com_twist = _ref_twist;//_omega_e + _ref_twist; //eddited by caosu
     //_com_twist = _ref_twist;
     _com_twist = _omega_e + _ref_twist;

     _u_cal_tmp = _omega_e(5,0); //obtain the u value
     _v_cal_tmp = _omega_e(6,0); //obtain the v value
     _w_cal_tmp = _omega_e(7,0); //obtain the w value
     return _com_twist;
}






DualQuaternion<float> AcrobaticDQ::Convert_quat_xyz_2_DQ(const Array<float,3> _xyz, const Quaternion<float> _att_d){
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



matrix::Matrix<float, 8, 8> AcrobaticDQ::Hat_G_mat(DualQuaternion<float> hat_q_e) const{ //verified
    matrix::Matrix<float, 8, 8> Hat_G_mat;
    matrix::Matrix<float, 4, 4> T_mat_q_mp, S_mat_q_mp;
    Quaternion<float> hat_q_e_1, hat_q_e_2;
    hat_q_e_1(0) = hat_q_e.m_real(0);
    hat_q_e_2 = hat_q_e.m_real;
    hat_q_e_2(0) = 0;
    T_mat_q_mp = hat_q_e.T_mat_q(hat_q_e_1);
    S_mat_q_mp = hat_q_e.S_mat_q(hat_q_e_2);
    for(size_t i=0; i<4; i++){
        for(size_t j=0; j<4; j++){
            if(hat_q_e.m_real(0)>0)
            {
                Hat_G_mat(i,j) = 2*T_mat_q_mp(i,j) + S_mat_q_mp(i,j);
            }
            else
            {
                Hat_G_mat(i,j) = -2*T_mat_q_mp(i,j) - S_mat_q_mp(i,j);
            }
        }
    }
    Quaternion<float> p_e_c;
    p_e_c = hat_q_e.quatConjugate(hat_q_e.m_real)*hat_q_e.m_dual;
    p_e_c *= 2;
    matrix::Matrix<float, 4, 4> S_mat_q_mp2;
    S_mat_q_mp2 = hat_q_e.S_mat_q(p_e_c);
    S_mat_q_mp2 *= -1;
    for(size_t i=0; i<4; i++){
        for(size_t j=0; j<4; j++){
            Hat_G_mat(i+4,j) = S_mat_q_mp2(i,j);
        }
    }
    Quaternion<float> quat_I;
    quat_I(0) = 1;
    matrix::Matrix<float, 4, 4> T_mat_q_mp2;
    T_mat_q_mp2 = hat_q_e.T_mat_q(quat_I);
    T_mat_q_mp2 *= 2;
    for(size_t i=0; i<4; i++){
        for(size_t j=0; j<4; j++){
            Hat_G_mat(i+4,j+4) = T_mat_q_mp2(i,j);
        }
    }

    Hat_G_mat = float(0.25)*Hat_G_mat;
    return Hat_G_mat;


}

matrix::Matrix<float, 3, 3> AcrobaticDQ::inv_three_order(const matrix::Matrix<float, 3, 3> mat_A) const{ // verified
    float a1, a2, a3, b1, b2, b3, c1, c2, c3;
    a1 = mat_A(0,0);
    a2 = mat_A(1,0);
    a3 = mat_A(2,0);
    b1 = mat_A(0,1);
    b2 = mat_A(1,1);
    b3 = mat_A(2,1);
    c1 = mat_A(0,2);
    c2 = mat_A(1,2);
    c3 = mat_A(2,2);

    float det_mat_A = float(float(1.0)/(float(1.0)*(a1*(b2*c3-c2*b3) - a2*(b1*c3-c1*b3) + a3*(b1*c2-c1*b2))));
    matrix::Matrix<float, 3, 3> inv_mat_A;
    inv_mat_A(0,0) = b2*c3 - c2*b3;
    inv_mat_A(0,1) = c1*b3 - b1*c3;
    inv_mat_A(0,2) = b1*c2 - c1*b2;
    inv_mat_A(1,0) = c2*a3 - a2*c3;
    inv_mat_A(1,1) = a1*c3 - c1*a3;
    inv_mat_A(1,2) = a2*c1 - a1*c2;
    inv_mat_A(2,0) = a2*b3 - b2*a3;
    inv_mat_A(2,1) = b1*a3 - a1*b3;
    inv_mat_A(2,2) = a1*b2 - a2*b1;

    if(fabs(det_mat_A) > 1e-5){
        inv_mat_A = inv_mat_A * det_mat_A;
    }

    return inv_mat_A;
}

matrix::Matrix<float, 8, 8> AcrobaticDQ::inv_Hat_G_mat_cal(const matrix::Matrix<float, 8, 8> Hat_G_mat) const{ // verified
    matrix::Matrix<float, 4, 4> inv_Hat_G_mat_1, inv_Hat_G_mat_2, inv_Hat_G_mat_3;
    matrix::Matrix<float, 3, 3> mat_temp, inv_mat_temp;
    inv_Hat_G_mat_1(0,0) = float(1.0) / Hat_G_mat(0,0);
    for(size_t i=0; i<3; i++)
        for(size_t j=0; j<3; j++)
        {
            mat_temp(i,j) = Hat_G_mat(i+1,j+1);
        }
    inv_mat_temp = inv_three_order(mat_temp);
    for(size_t i=0; i<3; i++)
        for(size_t j=0; j<3; j++)
        {
            inv_Hat_G_mat_1(i+1,j+1) = inv_mat_temp(i,j);
        }

    float inv_num_temp;
    inv_num_temp = float(1.0)/Hat_G_mat(6,6);
    inv_Hat_G_mat_3(0,0) = inv_num_temp;
    inv_Hat_G_mat_3(1,1) = inv_num_temp;
    inv_Hat_G_mat_3(2,2) = inv_num_temp;
    inv_Hat_G_mat_3(3,3) = inv_num_temp;

    for(size_t i=0; i<4; i++){
        for(size_t j=0; j<4; j++){
            inv_Hat_G_mat_2(i,j) = Hat_G_mat(i+4,j);
        }
    }
    inv_Hat_G_mat_2 = -float(1.0)*inv_Hat_G_mat_3*inv_Hat_G_mat_2;
    inv_Hat_G_mat_2 = inv_Hat_G_mat_2 * inv_Hat_G_mat_1;
    matrix::Matrix<float, 8, 8> inv_Hat_G_mat;
    for(size_t i=0; i<4; i++){
        for(size_t j=0; j<4; j++){
            inv_Hat_G_mat(i,j) = inv_Hat_G_mat_1(i,j);
            inv_Hat_G_mat(i+4,j) = inv_Hat_G_mat_2(i,j);
            inv_Hat_G_mat(i+4,j+4) = inv_Hat_G_mat_3(i,j);
        }
    }
    return inv_Hat_G_mat;
}



//matrix::Matrix<T, 8, 8> AcrobaticDQ::Hat_G_mat(DualQuaternion<T> dq1) const{}


int AcrobaticDQ::task_spawn(int argc, char *argv[])  /**< generate a task */
{

    AcrobaticDQ *instance = new AcrobaticDQ();

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

int AcrobaticDQ::custom_command(int argc, char *argv[])
{
    //PX4_INFO("custom_command~~");
    return print_usage("unknown command");
}


int AcrobaticDQ::print_status()
{
    //PX4_INFO("print_status");
    perf_print_counter(_loop_perf);
    return 0;
}

int AcrobaticDQ::print_usage(const char *reason)
{
    if(reason){
        PX4_WARN("%s\n", reason);
    }
    //PX4_INFO("print_usage");

    PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
                ### Description
                acrobatic_DQ is the fixed wing acrobatic command generator.

                )DESCR_STR"
                );
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_NAME("acrobatic_DQ", "controller");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}
