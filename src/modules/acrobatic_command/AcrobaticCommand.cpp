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
#include <matrix/QuaternionMapping.h>
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
    FILE *fp_att = nullptr;
    FILE *fp_pqr = nullptr;
    mavlink_log_info(&_mavlink_log_pub, "reading file~~~");
    //PX4_INFO("filepath:%s",filepath);
    //filepath = "/fs/microsd/data/loopdata.txt";

    if((fp_att = fopen(filepath_att, "r"))==nullptr)
    {
        mavlink_log_info(&_mavlink_log_pub, "file open error%s",filepath_att);
    }
    else
    {
        mavlink_log_info(&_mavlink_log_pub, "filepath %s open success", filepath_att);
    }

    /**< Init the parser */
    int ret;
    unsigned long int time;
    Quatf q_temp;
    quat_time q_t_temp;

    while (EOF != (ret = fscanf(fp_att, "%ld, \t%f, \t%f, \t%f, \t%f", &time, &q_temp(0), &q_temp(1), &q_temp(2), &q_temp(3))))
    {
        if(ret <= 0){
            fclose(fp_att);
        }
        //q_t_temp.quat_v = q_temp;
        //q_t_temp.time_v = time;

        //_quat_time_l.push_back(q_t_temp);
        //_time_v.push_back(time);
        //_quat_v.push_back(q_temp);
        //PX4_INFO("-----------------------------------------");
        //PX4_INFO("time:%ld",time);
        //PX4_INFO("q_temp:%f,%f,%f,%f",(double)q_temp(0),(double)q_temp(1),(double)q_temp(2),(double)q_temp(3));
    }

    fclose(fp_att);

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

    while(EOF != (ret = fscanf(fp_pqr, "%ld, \t%f, \t%f,\t%f", &time, &pqr_temp[0], &pqr_temp[1], &pqr_temp[2])))
    {
        if(ret <= 0){
            fclose(fp_pqr);
        }
        pqr_temp_t.pqr_v[0] = pqr_temp[0];
        pqr_temp_t.pqr_v[1] = pqr_temp[1];
        pqr_temp_t.pqr_v[2] = pqr_temp[2];

        pqr_temp_t.time_v = time;
        _pqr_time_l.push_back(pqr_temp_t);
    }
    fclose(fp_pqr);
    mavlink_log_info(&_mavlink_log_pub, "attitude and pqr readed succesfully! length: %d", _pqr_time_l.size());

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
    // Acceleration
    /*
    _mod_iden_data.axyz_body[0] = _sensor_accel.x;
    _mod_iden_data.axyz_body[1] = _sensor_accel.y;
    _mod_iden_data.axyz_body[2] = _sensor_accel.z;
    // Engine Thrust
    _mod_iden_data.engine_thrust = _sensor_accel.temperature;
    // Angular velocity
    _vehicle_rates_sub.copy(&_vehicle_angular_vel);
//    _mod_iden_data.pqr_body[0] = _vehicle_angular_vel.xyz[0];
//    _mod_iden_data.pqr_body[1] = _vehicle_angular_vel.xyz[1];
//    _mod_iden_data.pqr_body[2] = _vehicle_angular_vel.xyz[2];
    _mod_iden_data.pqr_body[0] = _man_status.angular_rate_filt[0];
    _mod_iden_data.pqr_body[1] = _man_status.angular_rate_filt[1];
    _mod_iden_data.pqr_body[2] = _man_status.angular_rate_filt[2];
    mavlink_log_info(&_mavlink_log_pub, "_mod_iden_data.pqr_body[0]:%lf", (double)_vehicle_angular_vel.xyz[0]);
    // Control inputs dedadr
    _mod_iden_data.dadedr_def[0] = _actuator_controls.control[_actuator_controls.INDEX_ROLL];
    _mod_iden_data.dadedr_def[1] = _actuator_controls.control[_actuator_controls.INDEX_PITCH];
    _mod_iden_data.dadedr_def[2] = _actuator_controls.control[_actuator_controls.INDEX_YAW];
    // Body frame velocity
    // Rotation from inertial frame to body frame

    matrix::Matrix3f dcm;
    dcm.setZero();
    dcm(0, 0) = _att_q(0)*_att_q(0) + _att_q(1)*_att_q(1) - _att_q(2)*_att_q(2) - _att_q(3)*_att_q(3);
    dcm(0, 1) = 2*(_att_q(1)*_att_q(2) + _att_q(0)*_att_q(3));
    dcm(0, 2) = 2*(_att_q(1)*_att_q(3) - _att_q(0)*_att_q(2));

    dcm(1, 0) = 2*(_att_q(1)*_att_q(2) - _att_q(0)*_att_q(3));
    dcm(1, 1) = _att_q(0)*_att_q(0) - _att_q(1)*_att_q(1) + _att_q(2)*_att_q(2) - _att_q(3)*_att_q(3);
    dcm(1, 2) = 2*(_att_q(2)*_att_q(3) + _att_q(0)*_att_q(1));

    dcm(2, 0) = 2*(_att_q(1)*_att_q(3) + _att_q(0)*_att_q(2));
    dcm(2, 1) = 2*(_att_q(2)*_att_q(3) - _att_q(0)*_att_q(1));
    dcm(2, 2) = _att_q(0)*_att_q(0) - _att_q(1)*_att_q(1) - _att_q(2)*_att_q(2) + _att_q(3)*_att_q(3);

    matrix::Vector3f vxyz(_local_pos.vx, _local_pos.vy, _local_pos.vz);
    matrix::Vector3f uvw_body = dcm * vxyz;

    _mod_iden_data.uvw_body[0] = uvw_body(0);
    _mod_iden_data.uvw_body[1] = uvw_body(1);
    _mod_iden_data.uvw_body[2] = uvw_body(2);
    //Angular acceleration filtered
    */

    /*-------------        Sensor data Publication       ----------------*/
    //_mod_iden_data.timestamp = hrt_absolute_time();
    //_mod_iden_pub.publish(_mod_iden_data);
    //PX4_INFO("-------------------------");
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

//void
//AcrobaticCommand::DMP_calculate(dmpPar_str dmpPar)
//{
//    matrix::Dcmf _R_att;
//    _R_att = matrix::Dcm<float>(_att_q);

//    /*-------------- Initial DMP state ------------*/
//    currState.DQuat = dmpPar.InitDQ;
//    currState.Twist = dmpPar.InitTW;
//    //Have not add the degration factor yet
//    dmpState.push_back(currState);

//    int32_t itNum = 1;
//    matrix::DualQuaternion<float> dqe;
//    dqe = DQuatError(dmpPar.GoalDQ, currState.DQuat);
//    matrix::DualQuaternionf dquaterror;
//    dquaterror = dqe.DQError_truepos();
//    // in the dmp function
//    float posErrNorm = dquaterror.m_dual.norm();
//    float quaterrNorm = dquaterror.m_real.norm();

//    /*--------------- Intialize to zero -------------*/

//    /*---------------- Loop and calculate the dmp ---------------*/
//    while((posErrNorm>posErr || quaterrNorm>quatErr) && itNum<maxIter)
//    {
//        computeNextStateDQuatDMP(dmpPar);
//        currState = nextState;
//        currClock = nextClock;
//        itNum += 1;

//        // Store DMP states
//        dmpState.push_back(currState);
//        // in the dmp function
//        x_dq1.push_back(x_dq);
//        gausst_dq1.push_back(gausst_dq);
//        gauss_dq1.push_back(gauss_dq);
//        psi_dq1.push_back(psi_dq);
//        gaussW_dq1.push_back(gaussW_dq);

//        //Recalculate distance to the goal
//        dqe = DQuatError(dmpPar.GoalDQ, currState.DQuat);
//        matrix::DualQuaternionf dquatErr;
//        dquatErr = dqe.DQError_truepos();
//        posErrNorm = dquatErr.m_dual.norm();
//        quaterrNorm = dquatErr.m_real.norm();
//    }
//}

void
AcrobaticCommand::dmpPar_init(void)
{
    dmpPar_val.alphaDQuat = 0.05; //Decay coefficientalt
    dmpPar_val.tauDQuat = 1; //time scale
    dmpPar_val.sigmaDQuat = 0.1;
    // in the dmp function
    //Set the gain
    dmpPar_val.KDQuat_q = 1; // Attitude gain
    dmpPar_val.dDQuat_q = 10*sqrt(dmpPar_val.KDQuat_q*dmpPar_val.tauDQuat); // Attitude damping
    dmpPar_val.kDQuat_p = 1; // Position gain
    dmpPar_val.dDQuat_p = 10*sqrt(dmpPar_val.kDQuat_p*dmpPar_val.tauDQuat); // Position damping
    dmpPar_val.dtDQuat = 0.01;
}

//void
//AcrobaticCommand::computeNextStateDQuatDMP(const dmpPar_str dmpPar)
//{
//    /*------------------ Get DMP parameters --------------------*/
//    float alphaDQuat = dmpPar.alphaDQuat;
//    float tauDQuat = dmpPar.tauDQuat;
//    float kDQuat_q = dmpPar.KDQuat_q;
//    float dDQuat_q = dmpPar.dDQuat_q;
//    float kDQuat_p = dmpPar.kDQuat_p;alt
//    float dDQuat_p =dmpPar.dDQuat_p;
//    matrix::DualQuaternionf goalDQuat = dmpPar.GoalDQ;
//    matrix::DualQuaternionf initDQuat = dmpPar.InitDQ;
//    float dtDQuat = dmpPar.dtDQuat;
//    Array<float,2> kDQuat;
//    kDQuat[0] = kDQuat_q; kDQuat[1] = kDQuat_p;
//    Array<float,2> dDQuat;
//    dDQuat[0] = dDQuat_q; dDQuat[1] = dDQuat_p;

//    float x1 = currClock.x;
//    float t1 = currClock.t;

//    /*------------------- Update DMP state -----------------------*/
//    x1 = x1 + (-1*alphaDQuat*x1)*dtDQuat ;
//    nextClock.x = x1;
//    nextClock.t = t1 + dmpPar.dtDQuat;

//    matrix::DualQuaternionf dqe;
//    dqe = DQuatError(goalDQuat, currState.DQuat);
//    matrix::DualQuaternionf dquatErr;
//    dquatErr = dqe.DQError_truepos();
//    dqe = DQuatError(goalDQuat, initDQuat);
//    matrix::DualQuaternionf dquatDis;
//    dquatDis = dqe.DQError_truepos()*x1;


//    // Calculate the twist acceleration
//    matrix::DualQuaternionf TwistAcc;
//    TwistAcc = ((dquatErr - dquatDis) * kDQuat - currState.Twist * dDQuat)/tauDQuat;
//    //Compute nonlinear forcing term
//    dmpNonlinearForce(dmpPar.dquatForceW,
//                                             dmpPar.dquatCenter,
//                                             dmpPar.dquatAmp,
//                                             dmpPar.kDQuat,
//                                             x1, t1);
//    TwistAcc = TwistAcc +_dmpnon_linear_force;
//    nextState.TwistAcc = TwistAcc;
//    nextState.Twist = currState.Twist + TwistAcc*dtDQuat;
//    nextState.DQuat = DQuatIntegral(currState.DQuat, nextState.Twist, dtDQuat);
//    x_dq = x1;
//}

//void
//AcrobaticCommand::dmpNonlinearForce(matrix::Matrix<float, 8, nCompDQuat> forceW,
//                       matrix::Matrix<float, 1, nCompDQuat> center,
//                       matrix::Matrix<float, 1, nCompDQuat> amplitude,
//                       matrix::Matrix<float, 8, 8> K,
//                       float clockSignal, float t)
//{
//    float nComponents = nCompDQuat;
//    float psi_sum = 0;
//    for(int i=0; i<nComponents; i++)
//    {
//        psi_dq(i,0) = gaussPDF(clockSignal, center(i,0), amplitude(i,0));
//        gausst_dq(i,0) = gaussPDF(t, center(i,0), amplitude(i,0));
//        gauss_dq(i,0) = psi_dq(i,0);
//        psi_sum += psi_dq(i,0);
//    }
//    psi_dq = psi_dq/psi_sum;
//    //Compute forcing term
//    _dmpnon_linear_force = K*forceW*psi_dq*clockSignal;
//}

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

//DualQuaternion<float>
//AcrobaticCommand::DQuatError(matrix::DualQuaternion<float> dq1, matrix::DualQuaternion<float> dq2)
//{
//    return DQuatProduct(dq2.conjugate(), dq1);
//}


//float
//AcrobaticCommand::gaussPDF(float Data, float Mu, float Sigma)
//{
//    float Data_temp;alt
//    Data_temp = Data - Mu;
//    float prob_temp;
//    float prob;
//    prob_temp = Data_temp * Data_temp / Sigma;
//    // Removed the realmin in matlab
//    prob = exp(-float(0.5)*prob_temp) / sqrt(2*float(PI)*(abs(Sigma)));
//    return prob;
//}







void
AcrobaticCommand::Run()
{
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

            mavlink_log_info(&_mavlink_log_pub, "time:%lf", now);
            /* ---------------- Subscription -----------------*/
            vehicle_cmd_poll();
            vehicle_global_pos_poll();
            // in the dmp function
            vehicle_local_pos_poll();
            // sensor_accel_poll();
            //actuator_controls_poll();
            //manual_status_poll();
            //vstatus_poll();
            //vehicle_angular_rates_poll();
            //PX4_INFO("_vehicle_cmd.command:%d",_vehicle_cmd.command);

            /*----------------- Initialize the dmp -------------*/
            dmpPar_init();
            // only update parameters if they changed
            bool params_updated = _parameter_update_sub.updated();

            // check for parameter updates
            if (params_updated) {
                // clear update
                parameter_update_s pupdate;
                _parameter_update_sub.copy(&pupdate);

                // update parameters from storage
                parameters_update();
            }
            // Test the current navigation status
            mavlink_log_info(&_mavlink_log_pub, "Current Navigation Status: %d", _vstatus.nav_state);

            /**< If we are not in the acrobatic mode, do nothing */
            if(_vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_ACROBATIC && _start_count > 0)
            {

                if(_time_first_acrobatic == 0)
                {
                    _time_first_acrobatic = now;
                    //initial altitude
                    _alt_first_acrobatic = _global_pos.alt; //The altitude is z*-1 + alt_init
                    _alt_sp_acrobatic = _alt_first_acrobatic;
                    //initial quaternion
                    _quat_first_acro = _att_q;
                    //initial position
                    _xyz_first_acro[0] = _local_pos.x;
                    _xyz_first_acro[1] = _local_pos.y;
                    _xyz_first_acro[2] = _local_pos.z;
                }

                /**< read the acrobatic command data file */

                //mavlink_log_info(&_mavlink_log_pub, "TEST_DATA_PATH");
                switch (_vehicle_cmd.acrobatic_name) {
                /**< loop maneuver */
                case 0:
                    filepath_att = "/fs/microsd/data/loop_att.txt";
                    filepath_pqr = "/fs/microsd/data/loop_pqr.txt";
                    //filepath_pqr_uvw = "/fs/microsd/data/immelman_pqr_uvw.txt";
                    filepath_pqr_uvw = "/fs/microsd/data/fast_climb_pqr_uvw.txt";
                    break;
                    /**< Immelman maneuver */
                case 1:
                    filepath_att = "/fs/microsd/data/immelman_att.txt";
                    filepath_pqr = "/fs/microsd/data/immelman_pqr.txt";
                    //filepath_pqr_uvw = "/fs/microsd/data/immelman_pqr_uvw.txt";
                    filepath_pqr_uvw = "/fs/microsd/data/fast_climb_pqr_uvw.txt";
                    break;
                    /**< default read nothing, keep straight flight*/
                default: break;
                }
                //PX4_INFO("_vehicle_cmd.acrobatic_name:%d", _vehicle_cmd.acrobatic_name);

                if(file_readed == false)
                {
                    //Obtain the acrobatic command
                    acro_data_read();
                    //pqr_uvw_acro_data_read();
                    pqr2quat();
                    //quat_uvw2xyz();
                    file_readed = true;
                }

                /* Obtain the custom defined acrobatic motion command*/

                //mavlink_log_info(&_mavlink_log_pub, "_quat_time_l[index].quat_v%.6lf",(double)_quat_time_l[1].quat_v(0));

                /**< obtain the custom defined acrobatic motion command */
                //_quat_cmd = interp_1_d();

                _quat_cmd = interp_1_d_quat();
                _xyz_cmd = interp_1_d_xyz();

                //PX4_INFO("_quat_cmd:%f,%f,%f,%f",(double)_quat_cmd(0),(double)_quat_cmd(1),(double)_quat_cmd(2),(double)_quat_cmd(3));


                float _rollspeed = _vehicle_angular_vel.xyz[0];
                float _pitchspeed = _vehicle_angular_vel.xyz[1];
                float _yawspeed = _vehicle_angular_vel.xyz[2];


                /**< Obtain the matrix Tf */
                //vehicle_att_poll();
                //_quat_err = (_quat_cmd - _att_q)/_tc * 2;
                //_quat_err = _quat_cmd - _att_q;
                //_quat_err = (_quat_cmd - _att_q)/_tc /1.5;
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
                _acrobatic_cmd.alt_sp_acrobatic = _alt_sp_acrobatic;

                /*The new version (altitude command)*/
               // _alt_sp_acrobatic = _xyz_cmd[2];
               // _acrobatic_cmd.alt_sp_acrobatic = _alt_sp_acrobatic;

                _acrobatic_cmd.euler_cmd[0] = asinf(2*(_att_q(0)*_att_q(2)-_att_q(3)*_att_q(1)));

                //PX4_INFO("-------------------------");
                //PX4_INFO("timestamp:%lld",_acrobatic_cmd.timestamp);
                //PX4_INFO("_quat_cmd:%f,%f,%f,%f",(double)_quat_cmd(0),(double)_quat_cmd(1),(double)_quat_cmd(2),(double)_quat_cmd(3));
                //PX4_INFO("_quat:%f,%f,%f,%f",(double)_att_q(0),(double)_att_q(1),(double)_att_q(2),(double)_att_q(3));

                _acro_cmd_pub.publish(_acrobatic_cmd);

                //prepare and publish the dmp test value
                //_dmp_test.timestamp = hrt_absolute_time();
                //_dmp_test.quaternion_integrated[0] = _quat_cmd(0);
                //_dmp_test.quaternion_integrated[1] = _quat_cmd(1);
                //_dmp_test.quaternion_integrated[2] = _quat_cmd(2);
                //_dmp_test.quaternion_integrated[3] = _quat_cmd(3);
                //_dmp_test.xyz_integrated[0] = _xyz_cmd[0];
                //_dmp_test.xyz_integrated[1] = _xyz_cmd[1];
                //_dmp_test.xyz_integrated[2] = _xyz_cmd[2];

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




