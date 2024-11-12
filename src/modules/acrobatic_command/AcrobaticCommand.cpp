/**
 * Acrobatic Command
 * This module aims to generate the desired attitude command in quaternion and convert it to p,q,r(axis rotational rates)
 *
 * This is version 2.0, for aggressive high-angle maneuver
 *
 *
 */
/**< Variables */

#include "AcrobaticCommand.h"
#include <px4_log.h>
#include <systemlib/mavlink_log.h>
#include <matrix/QuaternionMapping.h>
//#include <eigen3/Eigen/Dense>

using namespace time_literals;
using matrix::Quatf;
//#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
//#define TEST_DATA_PATH "./test_data/"
//#else
#define TEST_DATA_PATH "/fs/microsd/"
//#endif

extern "C" __EXPORT int acrobatic_command_main(int argc, char *argv[])
{
    return AcrobaticCommand::main(argc, argv);
}

AcrobaticCommand::AcrobaticCommand():
    WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
    _loop_perf(perf_alloc(PC_ELAPSED, "acrobatic_command: cycle"))
{
    //PX4_INFO("AcrobaticCommand::AcrobaticCommand");
    /**< fetch initial parameter values*/
    _parameter_handles.fw_acro_q0_tc = param_find("FW_ACRO_Q0_TC");
    _parameter_handles.fw_acro_q1_tc = param_find("FW_ACRO_Q1_TC");
    _parameter_handles.fw_acro_q2_tc = param_find("FW_ACRO_Q2_TC");
    _parameter_handles.fw_acro_q3_tc = param_find("FW_ACRO_Q3_TC");
    _acrobatic_cmd.acrobatic_finish = false;
    parameters_update();
    //_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    /*--------- dmp par related -----------*/
    //currClock.x = 1; //decay coefficient
    //currClock.t = 0;

}

AcrobaticCommand::~AcrobaticCommand()
{
    //PX4_INFO("AcrobaticCommand::~AcrobaticCommand");
    perf_free(_loop_perf);
}

bool AcrobaticCommand::init()
{
    //PX4_INFO("init~~~");
    if(!_att_sub.registerCallback()){
        return false;
    }
    return true;
}

int
AcrobaticCommand::parameters_update()
{
    //PX4_INFO("AcrobaticCommand::parameter_update");
    param_get(_parameter_handles.fw_acro_q0_tc, &(_parameters._fw_acro_q0_tc));
    param_get(_parameter_handles.fw_acro_q1_tc, &(_parameters._fw_acro_q1_tc));
    param_get(_parameter_handles.fw_acro_q2_tc, &(_parameters._fw_acro_q2_tc));
    param_get(_parameter_handles.fw_acro_q3_tc, &(_parameters._fw_acro_q3_tc));

    return PX4_OK;
}

void
AcrobaticCommand::vehicle_att_poll()
{
    //if(_att_sub.update(&_vehicle_att))
    //{
    //    _att_q(0) = _vehicle_att.q[0];
    //    _att_q(0) = _vehicle_att.q[1];
    //    _att_q(0) = _vehicle_att.q[2];
    //    _att_q(0) = _vehicle_att.q[3];
    //}
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
AcrobaticCommand::vehicle_global_pos_poll()
{
    _global_pos_sub.update(&_global_pos);
}

void
AcrobaticCommand::vehicle_local_pos_poll()
{
    _local_pos_sub.update(&_local_pos);
}

void
AcrobaticCommand::sensor_accel_poll()
{
    _sensor_accel_sub.update(&_sensor_accel);
}
void
AcrobaticCommand::actuator_controls_poll()
{
    _act_con_sub.update(&_actuator_controls);
}
//void
//AcrobaticCommand::vehicle_angular_rates_poll()
//{
//    _vehicle_rates_sub.copy(&_vehicle_angular_vel);
//    //_vehicle_rates_sub.update(&_vehicle_angular_vel);
//}
void
AcrobaticCommand::manual_status_poll()
{
    //_man_sub.update(&_man_status);
}
void
AcrobaticCommand::vstatus_poll()
{
    _vstatus_sub.update(&_vstatus);
}


void
AcrobaticCommand::acro_data_read() /**< This function needs to run in the init section */
{
    FILE *fp_pqr = nullptr;
    mavlink_log_info(&_mavlink_log_pub, "reading file~~~");
    //PX4_INFO("filepath:%s",filepath);
    //filepath = "/fs/microsd/data/loopdata.txt";

    /**< Init the parser */
    int ret;
    unsigned long int time;

    if((fp_pqr = fopen(filepath_pqr, "r"))==nullptr)
    {
        mavlink_log_info(&_mavlink_log_pub, "file open error%s",filepath_pqr);
    }
    else
    {
        mavlink_log_info(&_mavlink_log_pub, "filepath %s open success", filepath_pqr);
    }

    float pqr_temp[3];

    pqr_time pqr_temp_t;

    while(EOF != (ret = fscanf(fp_pqr, "%ld,\t%f,\t%f,\t%f", &time, &pqr_temp[0], &pqr_temp[1], &pqr_temp[2])))
    {
        if(ret <= 0){
            fclose(fp_pqr);
        }
        pqr_temp_t.pqr_v[0] = pqr_temp[0];
        pqr_temp_t.pqr_v[1] = pqr_temp[1];
        pqr_temp_t.pqr_v[2] = pqr_temp[2];

        pqr_temp_t.time_v = time;
        _pqr_time_l.push_back(pqr_temp_t);
        mavlink_log_info(&_mavlink_log_pub, "pqr reading time: %d", time);
    }
    fclose(fp_pqr);
    mavlink_log_info(&_mavlink_log_pub, "attitude and pqr readed", _pqr_time_l.size());

}

void
AcrobaticCommand::pqr_uvw_acro_data_read()
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

Quatf
AcrobaticCommand::interp_1_d_quat()
{
    /**< In order to accelerate the calcualtio_quat_time_ln, we proposed two interpolate method.
    * The first method is the nearest approach, the second one is not decided yet.
    * */
    size_t index;
    for(index = 0; index < _quat_time_l.size(); index++)
    {
        if(index == _quat_time_l.size()-1)
        {
            _finish_count ++;
            break;
        }
        else if(((now-_time_first_acrobatic) >= (_quat_time_l[index].time_v)) && ((now-_time_first_acrobatic) < (_quat_time_l[index+1].time_v)))
        {
            break;
        }

    }
    //mavlink_log_info(&_mavlink_log_pub, "index = %d", index);
    return _quat_time_l[index].quat_v ;
}

Array<float,3>
AcrobaticCommand::interp_1_d_xyz()
{
    size_t index;
    for(index = 0; index < _xyz_time_l.size(); index++)
    {
        if(index == _xyz_time_l.size()-1)
        {
            //_finish_count++;
            break;
        }
        else if(((now-_time_first_acrobatic) >= (_xyz_time_l[index].time_v)) && ((now-_time_first_acrobatic) < (_xyz_time_l[index+1].time_v)))
        {
            break;
        }
    }
    return _xyz_time_l[index].xyz_v;
}
//-----------PYF---High AOA--------
//---------------------------------
//parameters update
//float m, qbar, sref, Iyy;
//float alpha, alpha_dot, gamma_ref, gamma;
//Eigen::Matrix<float,5,1> thetaHat2;
//Eigen::Matrix<float,5,1> thetaHat3;
//Eigen::MatrixXf Gamma2(5,5);
//Gamma2 << 1, 0, 0, 0, 0,
//          0, 1, 0, 0, 0,
//          0, 0, 1, 0, 0,
//          0, 0, 0, 1, 0,
//          0, 0, 0, 0, 1;

//Eigen::MatrixXf Gamma3(5,5);
//Gamma3 << 1, 0, 0, 0, 0,
//          0, 1, 0, 0, 0,
//          0, 0, 1, 0, 0,
//          0, 0, 0, 1, 0,
//          0, 0, 0, 0, 1;

//Eigen::MatrixXf I5(5,5);
//I5 <<1, 0, 0, 0, 0,
//        0, 1, 0, 0, 0,
//        0, 0, 1, 0, 0,
//        0, 0, 0, 1, 0,
//        0, 0, 0, 0, 1;

//Eigen::VectorXf eta(5);
//eta << 1, alpha, alpha*alpha, alpha_dot, alpha_dot*alpha_dot;

//float e_g = gamma_ref - gamma;
//float e_q;
//Eigen::VectorXf est2 = -qbar*sref*Gamma2*eta*e_g/m;
//Eigen::VectorXf est3 = -qbar*sref*Gamma2*eta*e_q/Iyy;
//float g2 = square((2*thetaHat2.transpose()*eta + 0 - 2)/2) -1;
//float g3 = square((2*thetaHat3.transpose()*eta + 0 - 2)/2) -1;

//Eigen::VectorXf gra2 = 2*((2*thetaHat2.transpose()*eta + 0 - 2)/2)*2*eta/2;
//Eigen::VectorXf gra3 = 2*((2*thetaHat3.transpose()*eta + 0 - 2)/2)*2*eta/2;

//float pro2 = gra2*est2;
//int flag2;
//if (abs(g2)<0 && pro2 <=0){
//    flag2 = 1;
//}else{
//    flag2 = 0;
//}

//float pro3 = gra3*est3;
//int flag3;
//if (abs(g3)<0 && pro3 <=0){
//    flag3 = 1;
//}else{
//    flag3 = 0;
//}

//Eigen::VectorXf thetaHatDot2;
//if (abs(g2)<0 || flag2==1){
//    thetaHatDot2 = est2;
//}else{
//    thetaHatDot2 = (I5 - (gra2 * gra2.transpose()));
//}

//Eigen::VectorXf thetaHatDot3;
//if (abs(g2)<0 || flag3==1){
//    thetaHatDot3 = est3;
//}else{
//    thetaHatDot3 = (I5 - (gra3 * gra3.transpose()));
//}

//controller for maneuver
//float ct = 1, cq = 1, cg = 1;
//float alpha_ref, theta, q, Vt;

//float uth = 1/sin(alpha)*(-qbar*sref*thetaHat2.transpose()*eta + m*Vt*cg*(-gamma));
//float uq = ct*(alpha_ref + gamma - theta);
//float ude = cq*(uq - q) - thetaHat3*eta;//attention!!!

//float h_ref = 0;

void
AcrobaticCommand::pqr2quat()
{
    Quatf quat_temp = _quat_first_acro;

    float p,q,r;
    float q1,q2,q3,q4;
    float delta_t;
    q1 = quat_temp(0); q2 = quat_temp(1); q3 = quat_temp(2); q4 = quat_temp(3);

    for(size_t index_temp=0; index_temp<_pqr_time_l.size()-1; index_temp++)
    {
        p = (float)_pqr_time_l[index_temp].pqr_v[0];
        q = (float)_pqr_time_l[index_temp].pqr_v[1];
        r = (float)_pqr_time_l[index_temp].pqr_v[2];

        delta_t = (float)((_pqr_time_l[index_temp+1].time_v - _pqr_time_l[index_temp].time_v)/(1e6));

        q1 += (float)0.5*(float)(-1*p*q2 - q*q3 - r*q4)*delta_t;
        q2 += (float)0.5*(float)(p*q1 + r*q3 - q*q4)*delta_t;
        q3 += (float)0.5*(float)(q*q1 - r*q2 + p*q4)*delta_t;
        q4 += (float)0.5*(float)(r*q1 + q*q2 - p*q3)*delta_t;

        quat_temp(0) = q1;
        quat_temp(1) = q2;
        quat_temp(2) = q3;
        quat_temp(3) = q4;

        quat_time quat_t_temp;
        quat_t_temp.quat_v = quat_temp;
        quat_t_temp.time_v = _pqr_time_l[index_temp].time_v;
        _quat_time_l.push_back(quat_t_temp);
    }
    mavlink_log_info(&_mavlink_log_pub, "quaternion length: %d", _quat_time_l.size());
}

void
AcrobaticCommand::quat_uvw2xyz()
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
    x_body = xyz_temp[0]; y_body = xyz_temp[1]; z_body = _alt_first_acrobatic;

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
        z_body -= (float)((2*(q2*q4-q1*q3))*u_body + (2*(q3*q4+q1*q2))*v_body + (q1*q1-q2*q2-q3*q3+q4*q4)*w_body)*delta_t;

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

void
AcrobaticCommand::mod_iden_data()
{

    /*-------------        Sensor data subscription       ----------------*/

}


/*
void
AcrobaticCommand::acro_safety_pre_check()
{
    bool att_safe;
    bool velocity_safe;

    uint16_t vel_u; //tangential velocity
    uint16_t vel_con_top; //tangential velocity constrain during the top location of maneuver

}
*/

//quat_uvw2xyz();

template <typename T>
DualQuaternion<T> AcrobaticCommand::DQuatProduct(const DualQuaternion<T> dq1,
                                                  const DualQuaternion<T> dq2)
{
    // in the dmp function
    // dual quaternion multiplication
    return DualQuaternion<float>(dq1.m_real * dq2.m_real, dq1.m_real * dq2.m_dual + dq1.m_dual*dq2.m_real);
}

DualQuaternion<float>
AcrobaticCommand::DQuatIntegral(DualQuaternion<float> dq, DualQuaternion<float> twist, float dt)
{
    return DQuatProduct(dq, DQuatExponential(twist/2 * dt));
}


void
AcrobaticCommand::Run()
{
    //quat_uvw2xyz();
    //int i = 0;manual_status_poll
    //int loop_count = 0;alt
    //_vehicle_rates_sub.copy(&_vehicle_angular_vel);
    //while(!should_exit()){

    //    if(i==0 && loop_count < 1000)
    //    {
    //        mavlink_log_info(&_mavlink_log_pub, "testing~~~");
    //        continue;

    //   }
    //    else return;
        perf_begin(_loop_perf);
        //mavlink_log_info(&_mavlink_log_pub, "TEST_DATA_PATH:%s",TEST_DATA_PATH);

        if(_att_sub.update(&_att))
        {
            //PX4_INFO("Acrobatic Command~~~~~~~~~~");
            now = hrt_absolute_time();
            _att_q(0) = _att.q[0];
            _att_q(1) = _att.q[1];
            _att_q(2) = _att.q[2];
            _att_q(3) = _att.q[3];

            _start_count ++;

            //mavlink_log_info(&_mavlink_log_pub, "time:%lf", now);
            /* ---------------- Subscription -----------------*/
            // Obtain the current command
            vehicle_cmd_poll();
            vehicle_global_pos_poll();
            // in the dmp function
            vehicle_local_pos_poll();

            // only update parameters if they changed
            bool params_updated = _parameter_update_sub.updated();

            // check for
            //quat_uvw2xyz(); parameter updates
            if (params_updated) {
                // clear update
                parameter_update_s pupdate;
                _parameter_update_sub.copy(&pupdate);

                // update parameters from storage
                parameters_update();
            }
            // Test the current navigation status
            //mavlink_log_info(&_mavlink_log_pub, "Current Navigation Status: %d", _vstatus.nav_state);

            /**< If we are not in the acrobatic mode, do nothing */
            if(_vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_ACROBATIC && _start_count > 0)
            {

                if(_time_first_acrobatic == 0)
                {
                    _time_first_acrobatic = now;
                    //initial altitude
                    _alt_first_acrobatic = _global_pos.alt; //The altitude is z*-1 + alt_init
                    //During the pugachev maneuver, the attitude is assumed to be unchanged
                    _alt_sp_acrobatic = _alt_first_acrobatic;
                    //initial quaternion
                    _quat_first_acro = _att_q;
                    //initial position(local_position)
                    _xyz_first_acro[0] = _local_pos.x;
                    _xyz_first_acro[1] = _local_pos.y;
                    _xyz_first_acro[2] = _local_pos.z;
                }

                /**< read the acrobatic command data file */

                //mavlink_log_info(&_mavlink_log_pub, "TEST_DATA_PATH");
                switch (_vehicle_cmd.acrobatic_name) {
                /**< pugachev maneuver */
                case 0:
                    filepath_att = "/fs/microsd/data/high_angle_att.txt";
                    filepath_pqr = "/fs/microsd/data/high_angle_pqr.txt";
                    //filepath_pqr_uvw = "/fs/microsd/data/pugachev_pqr_uvw.txt";
                    filepath_pqr_uvw = "/fs/microsd/data/high_angle_pqr_uvw.txt";
                    break;
                /**< Immelman maneuver */
                case 1:
                    filepath_att = "/fs/microsd/data/half_cuban_eight_att.txt";
                    filepath_pqr = "/fs/microsd/data/half_cuban_eight_pqr.txt";
                    //filepath_pqr_uvw = "/fs/microsd/data/high_angle_pqr_uvw.txt";
                    //filepath_pqr_uvw = "/fs/microsd/data/high_angle_pqr_uvw.txt";
                    break;
                    /**< default read nothing, keep straight flight*/
                default: break;
                }
                //PX4_INFO("_vehicle_cmd.acrobatic_name:%d", _vehicle_cmd.acrobatic_name);

                if(file_readed == false)
                {
                    //Obtain the acrobatic command
                    acro_data_read();
                    file_readed = true;
                    //pqr_uvw_acro_data_read();
                    pqr2quat(); //The attitude command needs to be integrated
                    //quat_uvw2xyz();
                    mavlink_log_info(&_mavlink_log_pub, "file readed");
                }

                /* Obtain the custom defined acrobatic motion command*/

                //mavlink_log_info(&_mavlink_log_pub, "_quat_time_l[index].quat_v%.6lf",(double)_quat_time_l[1].quat_v(0));

                /**< obtain the custom defined acrobatic motion command */
                //_quat_cmd = interp_1_d();

                _quat_cmd = interp_1_d_quat();
                //_xyz_cmd = interp_1_d_xyz();

                //PX4_INFO("_quat_cmd:%f,%f,%f,%f",(double)_quat_cmd(0),(double)_quat_cmd(1),(double)_quat_cmd(2),(double)_quat_cmd(3));

//                //-----------PYF---High AOA--------
//                //---------------------------------
//                //parameters update
//                float m, qbar, sref, Iyy;
//                float alpha, alpha_dot, gamma_ref, gamma;
//                Eigen::Matrix<float,5,1> thetaHat2;
//                Eigen::Matrix<float,5,1> thetaHat3;
//                Eigen::MatrixXf Gamma2(5,5);
//                Gamma2 << 1, 0, 0, 0, 0,
//                          0, 1, 0, 0, 0,
//                          0, 0, 1, 0, 0,
//                          0, 0, 0, 1, 0,
//                          0, 0, 0, 0, 1;

//                Eigen::MatrixXf Gamma3(5,5);
//                Gamma3 << 1, 0, 0, 0, 0,
//                          0, 1, 0, 0, 0,
//                          0, 0, 1, 0, 0,
//                          0, 0, 0, 1, 0,
//                          0, 0, 0, 0, 1;

//                Eigen::MatrixXf I5(5,5);
//                I5 <<1, 0, 0, 0, 0,
//                        0, 1, 0, 0, 0,
//                        0, 0, 1, 0, 0,
//                        0, 0, 0, 1, 0,
//                        0, 0, 0, 0, 1;

//                Eigen::VectorXf eta(5);
//                eta << 1, alpha, alpha*alpha, alpha_dot, alpha_dot*alpha_dot;

//                float e_g = gamma_ref - gamma;
//                float e_q;
//                Eigen::VectorXf est2 = -qbar*sref*Gamma2*eta*e_g/m;
//                Eigen::VectorXf est3 = -qbar*sref*Gamma2*eta*e_q/Iyy;
//                float g2 = square((2*thetaHat2.transpose()*eta + 0 - 2)/2) -1;
//                float g3 = square((2*thetaHat3.transpose()*eta + 0 - 2)/2) -1;

//                Eigen::VectorXf gra2 = 2*((2*thetaHat2.transpose()*eta + 0 - 2)/2)*2*eta/2;
//                Eigen::VectorXf gra3 = 2*((2*thetaHat3.transpose()*eta + 0 - 2)/2)*2*eta/2;

//                float pro2 = gra2*est2;
//                int flag2;
//                if (abs(g2)<0 && pro2 <=0){
//                    flag2 = 1;
//                }else{
//                    flag2 = 0;
//                }

//                float pro3 = gra3*est3;
//                int flag3;
//                if (abs(g3)<0 && pro3 <=0){
//                    flag3 = 1;
//                }else{
//                    flag3 = 0;
//                }

//                Eigen::VectorXf thetaHatDot2;
//                if (abs(g2)<0 || flag2==1){
//                    thetaHatDot2 = est2;
//                }else{
//                    thetaHatDot2 = (I5 - (gra2 * gra2.transpose()));
//                }

//                Eigen::VectorXf thetaHatDot3;
//                if (abs(g2)<0 || flag3==1){
//                    thetaHatDot3 = est3;
//                }else{
//                    thetaHatDot3 = (I5 - (gra3 * gra3.transpose()));
//                }

//                //controller for maneuver
//                float ct = 1, cq = 1, cg = 1;
//                float alpha_ref, theta, q, Vt;

//                float uth = 1/sin(alpha)*(-qbar*sref*thetaHat2.transpose()*eta + m*Vt*cg*(-gamma));
//                float uq = ct*(alpha_ref + gamma - theta);
//                float ude = cq*(uq - q) - thetaHat3*eta;//attention!!!

//                float h_ref = 0;




                float _rollspeed = _vehicle_angular_vel.xyz[0];
                float _pitchspeed = _vehicle_angular_vel.xyz[1];
                float _yawspeed = _vehicle_angular_vel.xyz[2];


                /**< Obtain the matrix Tf */
                _fw_acro_q0_tc = _parameters._fw_acro_q0_tc;
                _fw_acro_q1_tc = _parameters._fw_acro_q1_tc;
                _fw_acro_q2_tc = _parameters._fw_acro_q2_tc;
                _fw_acro_q3_tc = _parameters._fw_acro_q3_tc;

                _quat_err(0) = (_quat_cmd(0) - _att_q(0))/_fw_acro_q0_tc;
                _quat_err(1) = (_quat_cmd(1) - _att_q(1))/_fw_acro_q1_tc;
                _quat_err(2) = (_quat_cmd(2) - _att_q(2))/_fw_acro_q2_tc;
                _quat_err(3) = (_quat_cmd(3) - _att_q(3))/_fw_acro_q3_tc;

                //Quatf _quat_err_t = _quat_err / _tc;

                /**< Tranform matrix Tf
                * p = 2*q0*q1_dot - 2*q1*q0_dot - 2*q2*q3_dot + 2*q3*q2_dot
                * q = 2*q0*q2_dot - 2*q2*q0_dot + 2*q1*q3_dot - 2*q3*q1_dot
                * r = 2*q0*q3_dot - 2*q1*q2_dot + 2*q2*q1_dot - 2*q3*q0_dot
                * */
                _body_setpoint[0] = 2 * _att_q(0) * _quat_err(1) - 2 * _att_q(1) * _quat_err(0)
                        - 2 * _att_q(2) * _quat_err(3) + 2 * _att_q(3) * _quat_err(2);
                _body_setpoint[1] = 2 * _att_q(0) * _quat_err(2) - 2 * _att_q(2) * _quat_err(0)
                        + 2 * _att_q(1) * _quat_err(3) - 2 * _att_q(3) * _quat_err(1);
                _body_setpoint[2] = 2 * _att_q(0) * _quat_err(3) - 2 * _att_q(1) * _quat_err(2)
                        + 2 * _att_q(2) * _quat_err(1) - 2 * _att_q(3) * _quat_err(0);

                _acrobatic_cmd.timestamp = hrt_absolute_time();
                //This is the actual desired anglular rates
                _acrobatic_cmd.body_rates_cmd[0] = _body_setpoint[0];
                _acrobatic_cmd.body_rates_cmd[1] = _body_setpoint[1];
                _acrobatic_cmd.body_rates_cmd[2] = _body_setpoint[2];

                _acrobatic_cmd.quaternion_cmd[0] = _quat_cmd(0);
                _acrobatic_cmd.quaternion_cmd[1] = _quat_cmd(1);
                _acrobatic_cmd.quaternion_cmd[2] = _quat_cmd(2);
                _acrobatic_cmd.quaternion_cmd[3] = _quat_cmd(3);

                _acrobatic_cmd.angular_velocity[0] = _rollspeed;
                _acrobatic_cmd.angular_velocity[1] = _pitchspeed;
                _acrobatic_cmd.angular_velocity[2] = _yawspeed;

                _acrobatic_cmd.do_acrobatic = true;

                /**< count if the acrobatic is finished*/
                // Add another condition: vehicle status is changed to manual or return. Time 20210712
                if(_finish_count >= 50)
                {
                    _acrobatic_cmd.acrobatic_finish = true;
                }

                /**< send the desired vehicle altitude to the tecs module */
                //_alt_sp_acrobatic += (float)((now-time_prev)/1e6)
                //float u = _local_pos.vx;
                //float v = _local_pos.vy;
                float w = _local_pos.vz;

                /*_alt_first_acrobatic += (float)((now-time_prev)/1e6) *
                        (2*(_att_q(1)*_att_q(3)-_att_q(0)*_att_q(2))*u +
                         2*(_att_q(2)*_att_q(3)+_att_q(0)*_att_q(1))*v +
                         (_att_q(0)*_att_q(0)-_att_q(1)*_att_q(1)-_att_q(2)*_att_q(2)+_att_q(3)*_att_q(3))*w);*/

                /*The old version (altitude command)*/
                _alt_sp_acrobatic += -1 * (float)((now-time_prev)/1e6) * w; //transfer according to the frame
                //The altitude command will not change during pugachev maneuver
                _acrobatic_cmd.airsp_sp = 20;
                //_alt_sp_acrobatic = _alt_first_acrobatic;

                _acrobatic_cmd.alt_sp_acrobatic = _alt_sp_acrobatic;

                /*The new version (altitude command)*/
               // _alt_sp_acrobatic = _xyz_cmd[2];
               // _acrobatic_cmd.alt_sp_acrobatic = _alt_sp_acrobatic;

                //_acrobatic_cmd.euler_cmd[0] = asinf(2*(_att_q(0)*_att_q(2)-_att_q(3)*_att_q(1)));

                //PX4_INFO("-------------------------");
                //PX4_INFO("timestamp:%lld",_acrobatic_cmd.timestamp);
                //PX4_INFO("_quat_cmd:%f,%f,%f,%f",(double)_quat_cmd(0),(double)_quat_cmd(1),(double)_quat_cmd(2),(double)_quat_cmd(3));
                //PX4_INFO("_quat:%f,%f,%f,%f",(double)_att_q(0),(double)_att_q(1),(double)_att_q(2),(double)_att_q(3));

                _acro_cmd_pub.publish(_acrobatic_cmd);

                //_dmp_test_pub.publish(_dmp_test); //the dmp information publication
                //PX4_INFO("publishing time:%ld", now);
            }
            //mod_iden_data(); //added in 01.07.2021
            time_prev = now;
        }
        perf_end(_loop_perf);
    //}
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

    /*
    _task_id = px4_task_spawn_cmd("acrobatic_command",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1200,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);
    if(_task_id < 0){
        _task_id = -1;
        return -errno;
    }
    */
    //PX4_INFO("_task_id:%d",_task_id);
    return 0;

}
/*
AcrobaticCommand *AcrobaticCommand::instantiate(int argc, char *argv[])
{
    AcrobaticCommand *instance = new AcrobaticCommand();

    if(instance == nullptr){
        PX4_ERR("alloc failed");
    }
    return instance;
}*/

int AcrobaticCommand::custom_command(int argc, char *argv[])
{
    //PX4_INFO("custom_command~~");
    return print_usage("unknown command");
}


int AcrobaticCommand::print_status()
{
    //PX4_INFO("print_status");
    perf_print_counter(_loop_perf);
    return 0;
}

int AcrobaticCommand::print_usage(const char *reason)
{
    if(reason){
        PX4_WARN("%s\n", reason);
    }
    //PX4_INFO("print_usage");

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




