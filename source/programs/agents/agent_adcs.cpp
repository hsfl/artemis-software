// Important TBDs
// *    When there is no GPS lock, the position type is none and the position and velocity
//      values will stall, must use the propagator to continue the position fix ...
//      otherwise we won't be able to point the satellite?
// * validate control code for detumble and LVLH
// * ST quaternion conversion
// * check ST return code value
// * decouple the GPS code and create and agent_gps? this will help the cpu load, in case we don't want agent_adcs to work in power safe mode
// * add logging capability (every action the agent_adcs does should be logged, even knowing the time it starts is a good thing)
// * decouple the simulation part, create another agent called agent_adcs_sim
// * every 10 minutes or so log the sensor status/data and GPS status/data
// * add control log every minute (same as gps log)

// TBDs for later
// * implement RW functionality

// TBDs done
// * Test GPS time (done)
// * calibrate IMU (done)
// * remove simulated segments from the code (done)

// Usage
// $ agent hiakasat adcs turn_on_actuators

// $ agent hiakasat adcs 'getvalue {"device_mtr_mom_000","device_mtr_mom_001","device_mtr_mom_002"}'
// $ agent hiakasat adcs 'getvalue {"device_mtr_rmom_000","device_mtr_rmom_001","device_mtr_rmom_002"}'

// $ agent hiakasat adcs 'setvalue {"device_mtr_rmom_000":20}'
// $ agent hiakasat adcs "control_mode manual 30 -4 3" // manual mode, values are the desired moments for the MTR (Am2)
// $ agent hiakasat adcs "control_mode off" // control off

// $ agent hiakasat adcs "control_mode pd kp kd" // pd control with gains kp and kd
// $ agent hiakasat adcs "control_mode pd -0.003 -5"

// for simulation
// $ agent hiakasat adcs 'setvalue {"device_imu_omega_000":[0,0,0.0275]}'
// $ agent hiakasat adcs 'getvalue {"device_imu_omega_000"}'

// Operation mode for agent_adcs
// * always start in detumble mode
// * check if ST omega is valid, if so use it, otherwise use IMU omega
// * if omega is less than 1 deg/sec then change to nominal/poiting mode (LVLH pointing)

#include "support/configCosmos.h"

//#include "vmt35_lib.h"
//#include "vn100_lib.h"
//#include "oemv_lib.h"
//#include "sinclair_lib.h"
#include "agent/agentclass.h"
#include "support/convertlib.h"
#include "support/jsonlib.h"
#include "library1/controllib.h"
#include "math/mathlib.h"
#include "support/print_utils.h"
#include <sys/time.h>

Agent *agent;

// these are controlled by request
bool debug_gps = false;
bool debug_control = false;
bool debug_print = false;
bool debug_st = false;

// add the node name by default
//char node_name[COSMOS_MAX_NAME] = "hiakasat";
string node_name = "hiakasat";
string agent_name = "adcs";

//vmt35_handle vmt35handle;
//vn100_handle vn100handle;
//oemv_handle oemvhandle;
//sinclair_state stthandle;

mutex sensor_lock;

#define AVERAGE_COUNT 10

LsFit lastposition(AVERAGE_COUNT, 2.);
LsFit lastvelocity(AVERAGE_COUNT, 2.);
LsFit lastimuomega(AVERAGE_COUNT, 2.);
LsFit lastbodymag(AVERAGE_COUNT, 2.);
LsFit lastatt(AVERAGE_COUNT, 2.);
LsFit laststtomega(AVERAGE_COUNT, 2.);
LsFit lasticrfmag(AVERAGE_COUNT, 2.);

double torque_lag = 50.;

enum
{
    CONTROL_MODE_BDOT,
    CONTROL_MODE_LVLH_HOLD,
    CONTROL_MODE_POINT,
    CONTROL_MODE_PD,
    CONTROL_MODE_OFF,
    CONTROL_MODE_AUTO,
    CONTROL_MODE_MANUAL,

    //    // delete?
    //    CONTROL_MODE_LEFT,
    //    CONTROL_MODE_MINUS,
    //    CONTROL_MODE_PLUS,
    //    CONTROL_MODE_RATE,
    //    CONTROL_MODE_RIGHT,
    //    CONTROL_MODE_ROTVAL,
    //    CONTROL_MODE_STOP,
    //    CONTROL_MODE_TEST,
    //    CONTROL_MODE_ADOT,
};

// agent requests
int32_t request_control_mode(string &request, string &response, Agent *agent);
int32_t request_log(string &request, string &response, Agent *agent);
int32_t request_turn_on_actuators(string &request, string &response, Agent *agent);
int32_t request_debug(string &request, string &response, Agent *agent);
int32_t request_reset_gps(string &request, string &response, Agent *agent);

// other functions
int32_t turn_on_actuators();
int32_t turn_on_gps();
int32_t turn_on_st();

int get_physics();
int get_motion();
int set_simulated();

void sensor_loop();
void gps_loop();

// Simulated variable
double romg, ralp, mtr[3];

#define MAXREQUESTENTRY 6
#define MAXBUFFERSIZE 1000
#define REQBUFSIZE 2000

beatstruc pbeat, mbeat;
string request, result;
//jstring rjstring={0,0,0};


double log_period=0.;
// default control mode, >> maybe should be bdot? or whoelse is activating bdot after the satellite ir released?
int control_mode=CONTROL_MODE_BDOT;
// start with automatic mode by default
// this can be overidden by a request
bool automatic_mode = true;

bool torque_allowed=false;
double timetillstoptorque=0;
double timetilltorque=0.;
double rotval;
rvector rotrate;
double pd_kp=0.;
double pd_kd=0.;
rvector momvec, temp_mom;

beatstruc iscbeat;

// for the log file of this program
double utc_agent_start = 0;


void log_agent(string log_entry){
    //bool debug_print = false;
    string log_string = "";

    //log_string += "ADCS@";
    // compute elapsed seconds since program started
    //log_string += to_string((int)round((currentmjd()-utc_agent_start)*86400));
    log_string += ": ";
    log_string += log_entry;

    //if(debug_print){cout << log_string << endl;}

    log_write(node_name, agent_name, utc_agent_start, "agent", "log", log_string);
}

int main(int argc, char *argv[])
{

    // >> check if it's ok to have a few counts?
    // add debug condition

    string log_string;

    utc_agent_start = currentmjd();

    log_string = "Starting ADCS agent v1.0";
    cout << log_string << endl;

    log_agent(log_string);

    int iretn;
    int count = 0;
    int iretnSystem = 0;

    //char buffer[255] = {'\0'};
    string response="";
    char buffer_string[300] = {'\0'};

    // for debugging
    PrintUtils print;

    // Check for other instance of this agent

    // collect node_name from user input
    // if the user wants to use another node_name other than the default
    if (argc == 2){
        //strcpy(node_name,argv[1]);
        node_name = argv[1];
    }

    // Initialize the Agent

  agent = new Agent(node_name.c_str(), agent_name.c_str(), 0.2);
    

    if ((iretn = agent->wait()) < 0)
    {
        agent->debug_error.Printf("%.4f %s Failed to start Agent %s on Node %s Dated %s : %s\n",currentmjd(), mjd2iso8601(currentmjd()).c_str(), agent->getAgent().c_str(), agent->getNode().c_str(), utc2iso8601(data_ctime(argv[0])).c_str(), cosmos_error_string(iretn).c_str());
        exit(iretn);
    }
    else
    {
        agent->debug_error.Printf("%.4f %s Started Agent %s on Node %s Dated %s\n",currentmjd(), mjd2iso8601(currentmjd()).c_str(), agent->getAgent().c_str(), agent->getNode().c_str(), utc2iso8601(data_ctime(argv[0])).c_str());
    }

    agent->debug_error.Printf("Node: %s Agent: %s - Established\n", agent->nodeName.c_str(), agent->agentName.c_str());


    // look up for agent_isc
    // if (iscbeat.utc == 0)
    // {
    //     iscbeat = agent_find_server(cdata, node_name.c_str(), "isc", 5.);
    // }


    // Add additional agent requests
    if ((iretn=agent->add_request("control_mode",request_control_mode,"", "gets the control mode")))
        exit (iretn);
    if ((iretn=agent->add_request("log",request_log, "", "gets the log")))
        exit (iretn);
    if ((iretn=agent->add_request("turn_on_actuators",request_turn_on_actuators, "", "actuators turned on")))
         exit (iretn);
     if ((iretn=agent->add_request("debug",request_debug, "", "turn on debugging on or off")))
         exit (iretn);
     if ((iretn=agent->add_request("reset_gps",request_reset_gps)))
         exit (iretn);

    


    // Set SOH String

    char sohstring[2000] = "{\"node_loc_utc\","
            "\"node_loc_pos_eci\","
            "\"node_loc_att_icrf\","
            "\"node_loc_bearth\"";

     // for (uint16_t i=0; i<cdata->devspec.rw_cnt; ++i)
     // {
     //    sprintf(&sohstring[strlen(sohstring)], ",\"device_rw_utc_%03d\",\"device_rw_alp_%03d\",\"device_rw_omg_%03d\",\"device_rw_ralp_%03d\",\"device_rw_romg_%03d\"",i,i,i,i,i);
     // }

//     for (uint16_t i=0; i<cdata->devspec.mtr_cnt; ++i)
//     {
//         sprintf(&sohstring[strlen(sohstring)], ",\"device_mtr_utc_%03d\",\"device_mtr_mom_%03d\",\"device_mtr_rmom_%03d\"",i,i,i);
//     }

//     for (uint16_t i=0; i<cdata->devspec.tcu_cnt; ++i)
//     {
//         sprintf(&sohstring[strlen(sohstring)], ",\"device_tcu_utc_%03d\",\"device_tcu_temp_%03d\"",i,i);
//     }

//     // Define IMU SOH
//     for (uint16_t i=0; i<cdata->devspec.imu_cnt; ++i)
//     {
//         sprintf(&sohstring[strlen(sohstring)], ",\"device_imu_utc_%03d\",\"device_imu_temp_%03d\",\"device_imu_mag_%03d\",\"device_imu_bdot_%03d\",\"device_imu_omega_%03d\",\"device_imu_accel_%03d\"",i,i,i,i,i,i);
//     }

//     // Define GPS SOH
//     for (uint16_t i=0; i<cdata->devspec.gps_cnt; ++i)
//     {
//         sprintf(&sohstring[strlen(sohstring)], ",\"device_gps_utc_%03d\",\"device_gps_position_%03d\",\"device_gps_velocity_%03d\",\"device_gps_geo_%03d\",\"device_gps_heading_%03d\",\"device_gps_nSats_%03d\",\"device_gps_time_status_%03d\",\"device_gps_position_type_%03d\",\"device_gps_solution_status_%03d\"",i,i,i,i,i,i,i,i,i);
//     }

//     for (uint16_t i=0; i<cdata->devspec.stt_cnt; ++i)
//     {
//         sprintf(&sohstring[strlen(sohstring)], ",\"device_stt_utc_%03d\",\"device_stt_temp_%03d\",\"device_stt_att_%03d\",\"device_stt_omega_%03d\"",i,i,i,i);
//     }

//     strcat(sohstring, "}");
//     agent_set_sohstring(cdata, sohstring);

//     //    //Attempt to contact physics agent // >> for simulation? then remove
//     //    if ((iretn=agent_get_server(cdata,node_name,(char *)"physics",3,&pbeat)) <= 0)
//     //    {
//     //        pbeat.utc = 0.;
//     //    }

//     //    //Attempt to contact motion agent
//     //    if ((iretn=agent_get_server(cdata,node_name,(char *)"motion",3,&mbeat)) <= 0)
//     //    {
//     //        mbeat.utc = 0.;
//     //    }


//     iretn = turn_on_gps();

//     // Open TCU+RW
//     iretn = turn_on_actuators();

//     // >> create function turn_on_imu()
//     // Open IMU device
//     cdata->devspec.imu[0].flag = DEVICE_FLAG_OFF;
//     if((iretn=vn100_connect(cdata->port[cdata->devspec.imu[0].portidx].name, &vn100handle)) == 0)
//     {
//         if(debug_print){
//             cout << "IMU connected" << endl;
//         }
//         //        cdata->devspec.imu[0].flag &= ~DEVICE_FLAG_SIMULATED;
//         cdata->devspec.imu[0].flag |= DEVICE_FLAG_ACTIVE;
//     }
//     else
//     {
//         // >> add log entry here
//         if(debug_print){
//             cout << "Error: Can't connect to IMU, not sure what to do since the IMU is always on (with the OBC). Maybe reboot OBC?" << endl;
//         }
//         //cdata->devspec.imu[0].flag |= DEVICE_FLAG_SIMULATED;
//     }

//     iretn = turn_on_st();


//     // Initialize devices and start threads
//     thread sensor_thread(sensor_loop);
//     thread gps_thread(gps_loop);

//     if(debug_print){
//         cout << "Threads initialized" << endl;
//     }

//     // agent body
//     float npolys[3][7], ppolys[3][7];
//     for (uint16_t i=0; i<3; ++i)
//     {
//         for (uint16_t j=0; j<7; ++j)
//         {
//             npolys[i][j] = cdata->devspec.mtr[i].npoly[j];
//             ppolys[i][j] = cdata->devspec.mtr[i].ppoly[j];
//         }
//     }
//     double imjd = currentmjd(0.);;
//     double logmjd = imjd;
//     rvector fitposition;
//     rvector fitvelocity;
//     quaternion fitatt;
//     quaternion slopeatt;
//     rvector fitsttomega;
//     rvector slopesttomega;
//     rvector fitimuomega;
//     rvector slopeimuomega;
//     rvector fitbodymag;
//     rvector slopebodymag;
//     rvector fiticrfmag;

//     // end result
//     rvector omega = {0,0,0};

//     //	rvector slopeicrfmag;
//     FILE* logfp=NULL;
//     qatt tatt; //MN: target attitude?


//     double utc_start_gps    = currentmjd();
//     double utc_start_st     = currentmjd();
//     double utc_start_tcu    = currentmjd();

//     double utc_start_debug  = currentmjd();

//     // ST flags
//     bool st_return_code_good = false;
//     bool st_q_norm_good = true;
//     bool st_use = false;
//     bool st_omega_norm_good = false;

//     // IMU flags
//     bool imu_omega_norm_good = false;
//     bool imu_use = false;


//     if(debug_print){
//         cout << "starting main loop" << endl;
//     }

//     qatt current_attitude;
// //    rvector mtorque;
// //    rvector torque;
//     rvector cbdot;

//     // --------------------------------------------------------------
//     // main adcs loop
//     while(agent_running(cdata)) {
//         count++;
//         double utc_now = currentmjd(0.);
//         tatt.s = q_zero();// >> must change to a request, and nominal is q_zero (identity) with orbital frame

//         // Get simulated data if available
//         // >> Remove?
//         //        get_physics();
//         //        if (mbeat.utc)
//         //        {
//         //            get_motion();
//         //        }

//         // Update control loop
//         sensor_lock.lock();

//         // Calculate latest estimated values
//         loc_clear(&cdata->node.loc);

//         // smoothing of the position
//         fitposition = lastposition.evalrvector(utc_now);
//         cdata->node.loc.pos.geoc.s = fitposition;
//         // smoothing of the velocity
//         fitvelocity = lastvelocity.evalrvector(utc_now);

//         cdata->node.loc.pos.geoc.v = fitvelocity;
//         cdata->node.loc.pos.geoc.utc = utc_now;
//         cdata->node.loc.pos.geoc.pass++;

//         // synchronize all frames (IRCF, etc.) with observed geocentric frame
//         pos_geoc(&cdata->node.loc);

//         // smoothing of the att
//         fitatt = lastatt.evalquaternion(utc_now);
//         cdata->node.loc.att.icrf.s = fitatt;
//         // // smoothing of the omega
//         fitsttomega = laststtomega.evalrvector(utc_now);

//         cdata->node.loc.att.icrf.v = fitsttomega;
//         cdata->node.loc.att.icrf.utc = utc_now;
//         cdata->node.loc.att.icrf.pass++;

//         // synchronize all attitude frames (IRCF, etc.) with observed geocentric frame
//         att_icrf(&cdata->node.loc);

//         fitimuomega = lastimuomega.evalrvector(utc_now);
//         fitbodymag = lastbodymag.evalrvector(utc_now);
//         cdata->node.loc.bearth = rotate_q(cdata->node.loc.att.geoc.s, fitbodymag);
//         fiticrfmag = transform_q(fitatt, lasticrfmag.evalrvector(utc_now));

//         // ----------------------------------------------------------
//         // check if we can use ST data or not
//         // >> someone needs to check this solution
//         if (cdata->devspec.stt[0].retcode == 5503){ //0x7F
//             // 7F = 0111 1111 (all return code bits are set to 1 so the solution is valid)
//             st_return_code_good = true;
//         } else {
//             st_return_code_good = false;
//         }

//         // check validity of quaternion from ST
//         double st_q_norm = sqrt(fitatt.d.x*fitatt.d.x + fitatt.d.y*fitatt.d.y + fitatt.d.z*fitatt.d.z + fitatt.w*fitatt.w);
//         if (st_q_norm > 0.9 && st_q_norm < 1.1){
//             st_q_norm_good = true;
//         } else {
//             // then we have a problem with the quaternion so don't use ST data
//             st_q_norm_good = false;
//         }

//         // check validity of ST omega
//         double st_omega_norm = sqrt(fitsttomega.col[0]*fitsttomega.col[0] + fitsttomega.col[1]*fitsttomega.col[1] + fitsttomega.col[2]*fitsttomega.col[2]);
//         if (st_omega_norm < 4*PI/180.){
//             st_omega_norm_good = true;
//         } else {
//             // then we have a problem with the omega so don't use ST data
//             st_omega_norm_good = false;
//         }

//         // decide wheter or not to use ST data
//         if( st_return_code_good && st_q_norm_good && st_omega_norm_good) {
//             st_use = true;
//         } else {
//             st_use = false;
//         }

//         // now decide which omega to use (ST or IMU)
//         // it a better world we would use a kalman filter to merge these two measurments
//         if (st_use){
//             // use omega from ST
//             omega = fitsttomega;
//         } else {
//             // use omega from IMU
//             omega = fitimuomega;
//         }


//         // check validity of IMU omega
//         double imu_omega_norm = sqrt(fitimuomega.col[0]*fitimuomega.col[0] + fitimuomega.col[1]*fitimuomega.col[1] + fitimuomega.col[2]*fitimuomega.col[2]);
//         if (imu_omega_norm < 10){
//             imu_omega_norm_good = true;
//         } else {
//             // then we have a problem with the omega so don't use IMU data
//             imu_omega_norm_good = false;
//         }

//         // decide wheter or not to use IMU data, later can add more conditions
//         if( imu_omega_norm_good ) {
//             imu_use = true;
//         } else {
//             imu_use = false;
//         }



//         // unlock threads
//         sensor_lock.unlock();

//         //        // print averaged mag values
//         //        sprintf(buffer,"[%f, %f, %f](%f)",
//         //                fitbodymag.col[0]*1e6,
//         //                fitbodymag.col[1]*1e6,
//         //                fitbodymag.col[2]*1e6,
//         //                1e6*sqrt(fitbodymag.col[0]*fitbodymag.col[0] + fitbodymag.col[1]*fitbodymag.col[1] + fitbodymag.col[2]*fitbodymag.col[2])
//         //                );
//         //        cout << buffer << endl;



//         switch (control_mode)
//         {

//         case CONTROL_MODE_BDOT:{
//             // B Dot, first mode after deployment
//             // >> check with Eric
//             // >> must add "-" to create moment in the right direction to break
//             // then ADOT must be "+"
//             // Logic:
//             // imagine omega = [0,0,1], B = [1,0,0]
//             // we want to break so need to produce a torque T = [0,0,-1]
//             // cbdot = B x omega = [0,-1,0]
//             // momvec = -cbdot = [0,1,0]
//             // torque = momvec x B = [0,0,-1], ok!

//             //>> what exactly is this product cbdot?


//             //if omega is small, say less than 1deg/sec = 0.017452778 rad sec then don't even bother to detumble
//             //>> if the ST omega is valid we should use that instead

//             // >> use fake mag for now, 1deg/sec around z-axis
//             //fitbodymag = {30e-6,0,0};
//             //fitimuomega = {0,0,1*PI/180.};
//             if (automatic_mode){
//                 if (length_rv(omega) >= 1.0*PI/180.){
//                     // >> Add logic to test if we're spinning up, if we are then kill the control or reverse
//                     // we're still spinning fast try to detumble
//                     cbdot = rv_smult(1e9, rv_cross(fitbodymag, omega));
//                 } else {
//                     // we're not spinning so much anymore, then change to nominal ponting mode (LVLH)
//                     control_mode = CONTROL_MODE_LVLH_HOLD;
//                     cbdot = rv_zero();
//                     break;
//                 }
//             } else {
//                 // we;re not in automatic mode, someone made a request
//                 cbdot = rv_smult(1e9, rv_cross(fitbodymag, omega));
//             }

//             rvector rmag = {0,0,0};

//             // in Bdot typically we shouldn't use the ST. That would be a special feature to add if needed later
//             // if the IMU is ok then we can use BDOT otherwise don't
//             if (imu_use){
//                 rmag = rv_smult(-1./50., cbdot);
//             } else {
//                 rmag = {0,0,0};
//             }
//             for (uint16_t i=0; i<3; ++i)
//             {
//                 momvec.col[i] = rmag.col[i];
//             }
//             //print.vector(momvec);
//         }
//             break;


//         case CONTROL_MODE_LVLH_HOLD:

//             // nominal attitude is LVLH

//             // we can't control the SC in nominal mode if the ST is not working (because it's the only source for attitude data)
//             // later this could be improved ... in case there is no ST, we have the Mag information wich combined with position can tell us roughly what attitude we have
//             // also the solar panels could provide some rough current attitude

//             // also if we don't have a GPS lock (or some other means to get position information)
//             // we can't compute the target attitude for LVLH

//             // gps solution_status = 0 is SOL_COMPUTED
//             if (!st_use && !(cdata->devspec.gps[0].solution_status == 0)){
//                 momvec = {0,0,0};
//                 break;
//             }


//             current_attitude.utc = utc_now;

//             // get pos in ECI
//             //cdata->node.loc.pos.eci.s


//             // >> we are getting the attitude in what frame? inertial I presume
//             // does it need to be changed to body frane?
//             current_attitude.s = fitatt;
//             current_attitude.v = fitsttomega;

//             // >> Eric, please make sure we are getting the right values here
//             // this should be the Orbital frame attitude with respect to inertial
//             // the LVLH attitude is coupled with the position and time
//             tatt.s = cdata->node.loc.att.lvlh.s;
//             tatt.v = cdata->node.loc.att.lvlh.v;

//             q_normalize(&tatt.s);

//             rvector moi;
//             // >> should this be the 2.5 kg.m2?
//             moi = {{6.,6.,6.}};
//             rvector torque;
//             torque = calc_control_torque_b(torque_lag, tatt, current_attitude, moi);
//             if (length_rv(torque))
//             {
//                 torque = rv_smult(1.+.0005/length_rv(torque), torque);
//             }
//             torque = transform_q(current_attitude.s,torque);
//             rvector mtorque;
//             calc_magnetic_torque(torque, &mtorque, &momvec, fitbodymag);
//             momvec = rv_smult(-1., momvec);

//             break;

//         case CONTROL_MODE_POINT:
//             // this mode lets the user point the SC anywhere
//             //momvec = rv_zero();
//             break;

//         case CONTROL_MODE_PD:

//             // Lab tuning with ACSTB
//             //kp = -0.003
//             //kd = -5


//             // current attitude
//             current_attitude.utc = utc_now;
//             current_attitude.s = fitatt;
//             current_attitude.v = fitimuomega;

//             // target attitude
//             tatt.utc = utc_now + 1/86400.;
//             tatt.s = q_eye();
//             tatt.v = rv_zero();
//             //rvector moi;
//             //moi = {{6.,6.,6.}};


//             torque = calc_control_torque_pd(pd_kp, pd_kd, tatt, current_attitude);
//             if (length_rv(torque))
//             {
//                 torque = rv_smult(1.+.0005/length_rv(torque), torque);
//             }
//             torque = transform_q(current_attitude.s,torque);

//             calc_magnetic_torque(torque, &mtorque, &momvec, fitbodymag);
//             momvec = rv_smult(-1., momvec);
//             break;

//         case CONTROL_MODE_OFF:
//             momvec = rv_zero();
//             break;

//         case CONTROL_MODE_AUTO:
//             // just makes sure that we can return to detumble/lvlh hold automatically
//             control_mode = CONTROL_MODE_BDOT;
//             break;

//         case CONTROL_MODE_MANUAL:
//             // control is totally in the user's hands
//             // go joystick!
//             for (uint16_t i=0; i<3; ++i)
//             {
//                 // >> assign requested moment
//                 // momvec.col[i] = cdata->devspec.mtr[i].rmom;
//                 momvec.col[i] = temp_mom.col[i];

//                 // at this moment it's working direclty from the request
//                 // so we can assign rmom instead
//                 cdata->devspec.mtr[i].rmom = momvec.col[i];
//             }
//             break;

//             //        case CONTROL_MODE_RATE:
//             //        case CONTROL_MODE_ROTVAL:
//             //        case CONTROL_MODE_TEST:
//             //        case CONTROL_MODE_PLUS:
//             //        case CONTROL_MODE_MINUS:
//             //        case CONTROL_MODE_LEFT:
//             //        case CONTROL_MODE_RIGHT:
//             //        case CONTROL_MODE_STOP:
//             //        {
//             //            qatt catt;
//             //            catt.utc = utc_now;
//             //            catt.s = fitatt;
//             //            catt.v = fitimuomega;

//             //            tatt.utc = utc_now + 1/86400.;
//             //            switch (control_mode)
//             //            {
//             //            case CONTROL_MODE_TEST:
//             //                rotval = fmod(tatt.utc * 86400./30., 1.) * D2PI;
//             //                tatt.s = q_mult(q_change_around_y(-RADOF(10)), q_change_around_z(-rotval));
//             //                tatt.v = rv_unitz(D2PI/100.);
//             //                break;
//             //            case CONTROL_MODE_PLUS:
//             //                rotval = fmod(tatt.utc*86400./15., 1.) * D2PI;
//             //                // Transform is negative rotation
//             //                tatt.s = q_change_around_z(-rotval);
//             //                tatt.v = rv_unitz(D2PI/15.);
//             //                break;
//             //            case CONTROL_MODE_MINUS:
//             //                rotval = fmod(tatt.utc*86400./15., 1.) * D2PI;
//             //                // Transform is negative rotation
//             //                tatt.s = q_change_around_z(rotval);
//             //                tatt.v = rv_unitz(-D2PI/15.);
//             //                break;
//             //            case CONTROL_MODE_LEFT:
//             //                rotval = RADOF(45.);
//             //                tatt.s = q_change_around_z(-rotval);
//             //                tatt.v = rv_zero();
//             //                break;
//             //            case CONTROL_MODE_RATE:
//             //                tatt.s = q_zero();
//             //                tatt.v = rotrate;
//             //                break;
//             //            case CONTROL_MODE_RIGHT:
//             //                rotval = RADOF(45.);
//             //                tatt.s = q_change_around_z(rotval);
//             //                tatt.v = rv_zero();
//             //                break;
//             //            case CONTROL_MODE_ROTVAL:
//             //                tatt.s = q_change_around_z(rotval);
//             //                tatt.v = rv_zero();
//             //                break;
//             //            case CONTROL_MODE_STOP:
//             //                tatt.s = q_eye();
//             //                tatt.v = rv_zero();
//             //                break;
//             //            }
//             //            q_normalize(&tatt.s);
//             //            //			printf("[%f %f %f %f] [%f %f %f]\n",catt.s.w,catt.s.d.x,catt.s.d.y,catt.s.d.z,catt.v.col[0],catt.v.col[1],catt.v.col[2]);
//             //            //			printf("tatt: [%f %f %f %f] [%f %f %f]\n",tatt.s.w,tatt.s.d.x,tatt.s.d.y,tatt.s.d.z,tatt.v.col[0],tatt.v.col[1],tatt.v.col[2]);
//             //            rvector moi;
//             //            moi = {{6.,6.,6.}};
//             //            rvector torque;
//             //            torque = calc_control_torque_b(torque_lag, tatt, catt, moi);
//             //            if (length_rv(torque))
//             //            {
//             //                torque = rv_smult(1.+.0005/length_rv(torque), torque);
//             //            }
//             //            torque = transform_q(catt.s,torque);
//             //            rvector mtorque;
//             //            calc_magnetic_torque(torque, &mtorque, &momvec, fitbodymag);
//             //            momvec = rv_smult(-1., momvec);
//             //        }
//             //            break;
//             //        case CONTROL_MODE_ADOT:
//             //            // A Dot (negative? to accelerate)
//             //        {
//             //            rvector rmag = rv_smult(-1./50., cbdot);
//             //            for (uint16_t i=0; i<3; ++i)
//             //            {
//             //                momvec.col[i] = rmag.col[i];
//             //            }
//             //        }
//             //            break;
//         }


//         //send control torque

//         if (cdata->devspec.tcu[0].flag & DEVICE_FLAG_ACTIVE)
//         {
//             //if (!(cdata->devspec.tcu[0].flag & DEVICE_FLAG_SIMULATED))
//             //{

//             // when we are in manual control we want full control without interruption of the torque_allowed flag
//             if (control_mode != CONTROL_MODE_MANUAL){

//                 if (torque_allowed)
//                 {
//                     //>> check thi for manual mode, not needed?
//                     // Less than .5 seconds from timetillstopstorque, stop early
//                     if (86400.*(timetillstoptorque-utc_now) <= .5)
//                     {
//                         momvec = rv_zero();
//                     }
//                 }
//                 else
//                 {
//                     momvec = rv_zero();
//                 }
//             }



//             // Convert Moment from Body Frame to Torque Rod frame
//             // the MTRs are oriented towards the negative axis of the body
//             // to achieve the desired effect momvec must be inverted
//             // MTR_X is reversed in Body_X
//             // MTR_Y is reversed in Body_Y
//             // MTR_Z is aligned with Body_Z
//             //momvec = rv_smult(-1,momvec);
//             momvec.col[0] = -momvec.col[0];
//             momvec.col[1] = -momvec.col[1];
//             momvec.col[2] = momvec.col[2];

//             iretn = vmt35_set_moments(&vmt35handle, momvec, npolys, ppolys);

//             // >> check sign
//             momvec.col[0] = -vmt35_calc_moment(vmt35handle.telem.dac[0]/1.e6, cdata->devspec.mtr[0].npoly, cdata->devspec.mtr[0].ppoly);
//             momvec.col[1] = -vmt35_calc_moment(vmt35handle.telem.dac[1]/1.e6, cdata->devspec.mtr[1].npoly, cdata->devspec.mtr[1].ppoly);
//             momvec.col[2] = vmt35_calc_moment(vmt35handle.telem.dac[2]/1.e6, cdata->devspec.mtr[2].npoly, cdata->devspec.mtr[2].ppoly);

//             //            if(debug_print){
//             //                //            cout << " | momvec cc: " << momvec << endl;
//             //            }

//             // copy data to cosmos struct
//             cdata->devspec.mtr[0].mom = momvec.col[0];
//             cdata->devspec.mtr[1].mom = momvec.col[1];
//             cdata->devspec.mtr[2].mom = momvec.col[2];


//             // >> debug print every second

//             if(debug_control){

//                 if( ((utc_now - utc_start_debug)*86400) > 0.5){
//                     cout << "Torque allowed: " << torque_allowed << " ";
//                     cout << "Control mode: " << control_mode << " "; // << endl;

//                     //print.vector("cbdot: ",cbdot, 3);
//                     print.vectorScaled("mag: ",fitbodymag, 1e6, 3); // in micro-Tesla
//                     print.vectorScaled("omega: ",fitimuomega, 180./PI, 3);
//                     print.vector("Moment: ",momvec);
//                     print.vector("Torque: ",rv_cross(momvec, fitbodymag), 6);
//                     print.end();

//                     //cout << "body mag " << fitbodymag << " | ";
//                     //cout << "omega " << fitimuomega << " | ";

//                     //cout << "Control Moment: " << momvec << endl;

//                     utc_start_debug = currentmjd();
//                 }

//             }

//         }

//         //cout << "Mtr moment set : " << endl;

//         // log stuff
//         if (log_period)
//         {
//             if (utc_now > logmjd)
//             {
//                 sensor_lock.lock();
//                 slopeimuomega = rv_smult(1./86400., lastimuomega.slopervector(utc_now));
//                 slopebodymag = rv_smult(1./86400., lastbodymag.slopervector(utc_now));
//                 slopesttomega = rv_smult(1./86400., laststtomega.slopervector(utc_now));
//                 slopeatt = q_smult(1./86400., lastatt.slopequaternion(utc_now));
//                 rvector torque = rv_cross(rv_smult(-1., momvec), fitbodymag);
//                 rvector domega = rv_sub(tatt.v, fitimuomega);
//                 quaternion dsq2 = q_mult(tatt.s,q_conjugate(fitatt));
//                 rvector distance = rv_quaternion2axis(dsq2);
//                 double domg = length_rv(distance);
//                 if (domg > DPI)
//                 {
//                     dsq2 = q_smult(-1.,dsq2);
//                     distance = rv_quaternion2axis(dsq2);
//                 }
//                 printf("Mode: %d Time: %7.2f %7.2f %7.2f Mom: %6.1f %6.1f %6.1f dTheta: %8.5f %8.5f %8.5f dOmega: %8.5f %8.5f %8.5f Torque: %8.5f %8.5f %8.5f\n"
//                        "Att: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f STTOmega: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f IMUOmega: %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f\n"
//                        "Mag: %7.0f %7.0f %7.0f %7.0f %7.0f %7.0f %7.0f %7.0f %7.0f Bdot: %7.0f %7.0f %7.0f\n"
//                        "Pos: %9.0f %9.0f %9.0f [%9.4f %9.4f %9.0f] Vel: %8.2f %8.2f %8.2f\n"
//                        ,control_mode
//                        ,(utc_now - imjd)*86400.,(timetilltorque-utc_now)*86400.,(timetillstoptorque-utc_now)*86400.
//                        ,momvec.col[0], momvec.col[1], momvec.col[2]
//                         ,distance.col[0], distance.col[1], distance.col[2]
//                         ,domega.col[0], domega.col[1], domega.col[2]
//                         ,torque.col[0], torque.col[1], torque.col[2]
//                         ,fitatt.w,fitatt.d.x,fitatt.d.y,fitatt.d.z
//                         ,slopeatt.w,slopeatt.d.x,slopeatt.d.y,slopeatt.d.z
//                         ,fitsttomega.col[0],fitsttomega.col[1],fitsttomega.col[2]
//                         ,slopesttomega.col[0],slopesttomega.col[1],slopesttomega.col[2]
//                         ,fitimuomega.col[0],fitimuomega.col[1],fitimuomega.col[2]
//                         ,slopeimuomega.col[0],slopeimuomega.col[1],slopeimuomega.col[2]
//                         ,1e9*fiticrfmag.col[0],1e9*fiticrfmag.col[1],1e9*fiticrfmag.col[2]
//                         ,1e9*fitbodymag.col[0],1e9*fitbodymag.col[1],1e9*fitbodymag.col[2]
//                         ,1e9*slopebodymag.col[0],1e9*slopebodymag.col[1],1e9*slopebodymag.col[2]
//                         ,cbdot.col[0],cbdot.col[1],cbdot.col[2]
//                         ,fitposition.col[0], fitposition.col[1], fitposition.col[2], DEGOF(cdata->node.loc.pos.geod.s.lat), DEGOF(cdata->node.loc.pos.geod.s.lon), cdata->node.loc.pos.geod.s.h, fitvelocity.col[0], fitvelocity.col[1], fitvelocity.col[2]
//                         );
//                 logfp = fopen("adcs_log.txt","a");
//                 if (logfp != NULL)
//                 {
//                     fprintf(logfp, "Mode:\t%d\tTime:\t%.8g\t%.8g\t%.8g\tMom:\t%.8g\t%.8g\t%.8g\tdTheta:\t%.8g\t%.8g\t%.8g\tdOmega:\t%.8g\t%.8g\t%.8g\tTorque:\t%.8g\t%.8g\t%.8g\tAtt:\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\tSTTOmega:\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\tIMUOmega:\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\tMag:\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\t%.8g\tBdot:\t%.8g\t%.8g\t%.8g\n"
//                             ,control_mode
//                             ,(utc_now - imjd)*86400.,(timetilltorque-utc_now)*86400.,(timetillstoptorque-utc_now)*86400.
//                             ,momvec.col[0], momvec.col[1], momvec.col[2]
//                             ,distance.col[0], distance.col[1], distance.col[2]
//                             ,domega.col[0], domega.col[1], domega.col[2]
//                             ,torque.col[0], torque.col[1], torque.col[2]
//                             ,fitatt.w,fitatt.d.x,fitatt.d.y,fitatt.d.z
//                             ,slopeatt.w,slopeatt.d.x,slopeatt.d.y,slopeatt.d.z
//                             ,fitsttomega.col[0],fitsttomega.col[1],fitsttomega.col[2]
//                             ,slopesttomega.col[0],slopesttomega.col[1],slopesttomega.col[2]
//                             ,fitimuomega.col[0],fitimuomega.col[1],fitimuomega.col[2]
//                             ,slopeimuomega.col[0],slopeimuomega.col[1],slopeimuomega.col[2]
//                             ,1e9*fiticrfmag.col[0],1e9*fiticrfmag.col[1],1e9*fiticrfmag.col[2]
//                             ,1e9*fitbodymag.col[0],1e9*fitbodymag.col[1],1e9*fitbodymag.col[2]
//                             ,1e9*slopebodymag.col[0],1e9*slopebodymag.col[1],1e9*slopebodymag.col[2]
//                             ,cbdot.col[0],cbdot.col[1],cbdot.col[2]
//                             );
//                     fclose(logfp);
//                 }
//                 sensor_lock.unlock();
//                 logmjd = utc_now + log_period/86400.;
//             }
//         }

//         //cout << "Log period done" << endl;


//         // let's make sure the GPS is always on
//         // as far as there is no request to shut it down
//         // check every 10 minutes
//         if( ((currentmjd() - utc_start_gps)*86400) > 600){

//             // turn on GPS (pdu_switch 10) for 15 minutes, then it's the job of the ADCS agent to keep the GPS on as needed
//             // >> enable requests to agent ISC when it works without crashing
//             if (0){
//                 if (iscbeat.utc != 0) {
//                     iretn = agent->send_request(iscbeat, "pdu_switch 10 900", response, 300, 2.);
//                 }
//             } else {

//                 // alternative cause agent_isc is crashing with agent requests
//                 iretn =system ("control_isc pdu_switch 10 900");

//                 COSMOS_SLEEP(1);

//                 // >> check if device bus 3 volt and current is > 0, then probably it is on
//                 if (iretn!=0){
//                     if(debug_print){
//                         cout << "Error: isc-control command failed to turn on GPS" << endl;
//                     }
//                     //exit (iretn);
//                 }
//             }

//             utc_start_gps = currentmjd();
//         }

//         // keep the ST always on while in control mode
//         // check every 10 minutes
//         if( ((currentmjd() - utc_start_st)*86400) > 600){

//             // turn on ST (pdu_switch 3) for 15 minutes, then it's the job of the ADCS agent to keep the actuators on as needed
//             // >> enable requests to agent ISC when it works without crashing
//             if (0){
//                 if (iscbeat.utc != 0) {
//                     iretn = agent->send_request(iscbeat, "pdu_switch 4 900", response, 300, 2.);
//                 }
//             }
//             else {

//                 iretnSystem =system ("control_isc pdu_switch 4 900");

//                 COSMOS_SLEEP(1);

//                 // >> check if device bus 4 volt and current is > 0, then probably it is on
//                 if (iretnSystem!=0){
//                     if(debug_print){
//                         cout << "Error: isc-control command failed to turn on ST" << endl;
//                     }
//                     //exit (iretn);
//                 }

//             }
//             utc_start_st = currentmjd();
//         }


//         // keep the actuators always on while in control mode
//         // check every 10 minutes
//         if( ((currentmjd() - utc_start_tcu)*86400 > 600) && (control_mode != CONTROL_MODE_OFF)){

//             // turn on TCU (pdu_switch 3) for 15 minutes, then it's the job of the ADCS agent to keep the TCU on as needed
//             // >> enable requests to agent ISC when it works without crashing
//             if (0){
//                 if (iscbeat.utc != 0) {
//                     iretn = agent->send_request(iscbeat, "pdu_switch 3 900", response, 300, 2.);
//                 }
//             } else {
//                 iretn =system ("control_isc pdu_switch 3 900");

//                 COSMOS_SLEEP(1);

//                 // >> check if device bus 3, volt and current is > 0, then probably it is on
//                 if (iretn!=0){
//                     if(debug_print){
//                         cout << "Error: isc-control command failed to turn on TCU" << endl;
//                     }
//                     //exit (iretn);
//                 }
//             }

//             utc_start_tcu = currentmjd();
//         }


//         //		fflush(stdout);
//         COSMOS_USLEEP(10000);
//     } // end while(agent_running(cdata))

//     // Wait for all threads to stop
//     sensor_thread.join();
//     gps_thread.join();

//     vmt35_disconnect(&vmt35handle);
//     vn100_disconnect(&vn100handle);
//     sinclair_disconnect(&stthandle);

//     agent_shutdown_server(cdata);

//     exit(0);
// }

// void gps_loop()
// {
//     bool newvalue;
//     double utc_now;
//     int32_t iretn;
//     double ts_position = .2;
//     double ts_velocity = .2;
//     char buffer[255] = {'\0'};
//     double utc = 0;

//     // define the ground stations available for increased beacon frequency
//     gvector kcc;
//     kcc.lat = 21.3928;
//     kcc.lon = -157.984;

//     gvector fairbanks;
//     fairbanks.lat = 64.8378;
//     fairbanks.lon = -147.716;

//     gvector surrey;
//     surrey.lat = 51.2433;
//     surrey.lon = 0.589496;

//     bool beacon_frequency_high = false;
//     bool beacon_frequency_high_previous = false;

//     string log_string = "";

//     // Open GPS device
//     cdata->devspec.gps[0].flag |= DEVICE_FLAG_ACTIVE;
//     //cdata->devspec.gps[0].flag &= ~DEVICE_FLAG_SIMULATED;

//     double gps_last_log_utc = currentmjd();

//     while (agent_running(cdata))
//     {
//         // Get GPS values
//         newvalue = false;
//         if (cdata->devspec.gps[0].flag & DEVICE_FLAG_ACTIVE)
//         {


//             // Check time and update if necessary

//             // get the time
//             if ((iretn=oemv_time(&oemvhandle)) == 0)
//             {
//                 utc = cal2mjd2(oemvhandle.message.time.utc_year,
//                                oemvhandle.message.time.utc_month,
//                                // compute day and fraction of day
//                                (oemvhandle.message.time.utc_day +
//                                 oemvhandle.message.time.utc_hour/24. +
//                                 oemvhandle.message.time.utc_minute/1440. +
//                                 oemvhandle.message.time.utc_ms/86400000.)
//                                );


//                 utc_now = currentmjd(0.);
//                 newvalue = true;
//             }
//             else
//             {
//                 // try to connect to GPS
//                 // this step might not be needed because the ISC keeps the GPS alive
//                 // and agent ADCS also
//                 if((iretn=oemv_connect(cdata->port[cdata->devspec.gps[0].portidx].name,
//                                        &oemvhandle)) != 0)
//                 {
//                     cdata->devspec.gps[0].flag &= ~DEVICE_FLAG_CONNECTED;
//                 }
//                 else
//                 {
//                     cdata->devspec.gps[0].flag |= DEVICE_FLAG_CONNECTED;
//                 }
//             }

//             // ------------------------------------------------------
//             // System time update

//             // make sure the time status is larger than 0 (0 = unknown time)
//             // all other values for time status indicate that the GPS has gotten some time
//             // can only update time when
//             // time status > 0 (time status = 0 is unknown)
//             if (newvalue && (oemvhandle.message.header.time_status > 0))
//             {
//                 double deltat = 86400.*(utc - utc_now);
//                 if (fabs(deltat) > 1.)
//                 {
//                     // Gross adjustment to system clock
//                     if(debug_print){
//                         //cout << "- Gross adjustment to system clock" << end;
//                     }
// #if defined(COSMOS_WIN_OS)
//                     SYSTEMTIME newtime;
//                     SetSystemTime(&newtime);
// #else
//                     struct timeval newtime = utc2unix(utc);
//                     // >> check with Eric if this is the right way to set the time?
//                     settimeofday(&newtime, NULL);
// #endif
//                     //printf("- Gross adjustment: %f to %f [%d %d] %d\n", utc_now, utc, newtime.tv_sec, newtime.tv_usec, errno);
//                 }
//                 else
//                 {

//                     // Fine adjustment using adjtime()
//                     if (fabs(deltat) > .001)
//                     {
// #if defined(COSMOS_WIN_OS)
//                         double newdelta;
//                         newdelta = deltat * 1e7;
//                         SetSystemTimeAdjustment(newdelta,false);
// #else

//                         struct timeval newdelta, olddelta;
//                         newdelta.tv_sec = deltat;
//                         newdelta.tv_usec = 100000. * (deltat - newdelta.tv_sec) + .5;
//                         adjtime(&newdelta, &olddelta);
// #endif
//                         // printf("- Fine adjustment: %d seconds %d useconds\n", newdelta.tv_sec, newdelta.tv_usec);

//                     }
//                 }
//             }

//             // ------------------------------------------------------
//             // System position/velocity update

//             // Check position and velocity
//             utc_now = currentmjd(0.);
//             double dt_position = (utc_now-lastposition.lastx())*86400.;
//             double dt_velocity = (utc_now-lastvelocity.lastx())*86400.;

//             // >> why is this condition here?
//             if (dt_position > ts_position ||  dt_velocity > ts_velocity)
//             {
//                 // create temporary container for the gps information before copying it into the cosmos structure
//                 gpsstruc newgps = *(cdata->devspec.gps[0]);

//                 newgps.time_status       = -1; // clear the status information
//                 newgps.solution_status   = -1; // clear the status information
//                 newgps.position_type     = -1; // clear the status information
//                 newgps.n_sats_used       = -1;
//                 newgps.n_sats_visible    = -1;

//                 // get gpgga log to get the LLA and number of satellites in use
//                 if ( (iretn =oemv_gpgga(&oemvhandle)) == 0){
//                     // got good message
//                     newgps.geo.lat = oemvhandle.message.geo.lat;
//                     newgps.geo.lon = oemvhandle.message.geo.lon;
//                     newgps.geo.h = oemvhandle.message.geo.h;

//                 } else {
//                     // GPGGA error
//                     log_string = "GPGGA error";
//                     if (debug_gps){
//                         cout << log_string << endl;
//                     }
//                     log_agent(log_string);
//                 }

//                 // get gpgsv log to check the number of satellites in view
//                 if ( (oemv_gpgsv(&oemvhandle)) == 0){
//                     // got good message
//                     newgps.n_sats_used       = oemvhandle.message.n_sats_used;
//                     newgps.n_sats_visible    = oemvhandle.message.n_sats_visible;
//                 } else {
//                     // GPGSV error
//                     log_string = "GPGSV error";
//                     if (debug_gps){
//                         cout << log_string << endl;
//                     }
//                     log_agent(log_string);
//                 }

//                 // if we have a good position, collect the information
//                 if ((iretn=oemv_bestxyz(&oemvhandle)) == 0)
//                 {

//                     newgps.utc = gps2utc(week2gps(oemvhandle.message.header.gps_week,
//                                                   oemvhandle.message.header.gps_second));
//                     // >> position in what coordinates? ECEF?
//                     newgps.position.col[0] = oemvhandle.message.bestxyz.position_x;
//                     newgps.position.col[1] = oemvhandle.message.bestxyz.position_y;
//                     newgps.position.col[2] = oemvhandle.message.bestxyz.position_z;
//                     newgps.position_sd.col[0] = oemvhandle.message.bestxyz.position_x_sd;
//                     newgps.position_sd.col[1] = oemvhandle.message.bestxyz.position_y_sd;
//                     newgps.position_sd.col[2] = oemvhandle.message.bestxyz.position_z_sd;

//                     // >> velocity in what coordinates? ECEF?
//                     newgps.velocity.col[0] = oemvhandle.message.bestxyz.velocity_x;
//                     newgps.velocity.col[1] = oemvhandle.message.bestxyz.velocity_y;
//                     newgps.velocity.col[2] = oemvhandle.message.bestxyz.velocity_z;
//                     newgps.velocity_sd.col[0] = oemvhandle.message.bestxyz.velocity_x_sd;
//                     newgps.velocity_sd.col[1] = oemvhandle.message.bestxyz.velocity_y_sd;
//                     newgps.velocity_sd.col[2] = oemvhandle.message.bestxyz.velocity_z_sd;


//                     newvalue = true;
//                 } else {
//                     // try to reconnect the GPS

//                     // oemv_bestxyz error
//                     log_string = "oemv_bestxyz error: ";
//                     log_string += iretn;
//                     if (debug_gps){
//                         cout << log_string << endl;
//                     }
//                     log_agent(log_string);

//                     if((iretn=oemv_connect(cdata->port[cdata->devspec.gps[0].portidx].name,
//                                            &oemvhandle)) != 0)
//                     {
//                         cdata->devspec.gps[0].flag &= ~DEVICE_FLAG_CONNECTED;
//                     }
//                     else
//                     {
//                         cdata->devspec.gps[0].flag |= DEVICE_FLAG_CONNECTED;

//                         // oemv_bestxyz error
//                         log_string = "oemv_bestxyz error: ";
//                         log_string += iretn;
//                         if (debug_gps){
//                             cout << log_string << endl;
//                         }
//                         log_agent(log_string);

//                     }
//                 }

//                 // keep checking the status
//                 newgps.time_status       = oemvhandle.message.header.time_status;
//                 newgps.solution_status   = oemvhandle.message.bestxyz.position_status;
//                 newgps.position_type     = oemvhandle.message.bestxyz.position_type;


//                 //

//                 // clear the status information for next time
//                 // >> check if this is the right place to do it
//                 oemvhandle.message.header.time_status       = -1;
//                 oemvhandle.message.bestxyz.position_status  = -1;
//                 oemvhandle.message.bestxyz.position_type    = -1;

//                 //sprintf(buffer,"Status [%d %d %d]",
//                 //        newgps.time_status, newgps.solution_status,  newgps.position_type);
//                 //cout << buffer << endl;

//                 // can only update position when
//                 // solution status = 0 (sol computed)
//                 // and position type > 0 (pos type = 0 means no solution)
//                 //&& (newgps.solution_status==0) && (newgps.position_type>0)

//                 // if we get a new value we still have usefull info even if it's not yet a computed solution
//                 if (newvalue)
//                 {


//                     sprintf(buffer,"Status [%d %d %d] | UTC: %f (%s) | Sat Vis: %d Sat Use: %d | LLA : [%.3f %.3f %.3f] | Pos :[%.3f, %.3f, %.3f] Vel [%.3f %.3f %.3f]",
//                             // status
//                             newgps.time_status,
//                             newgps.solution_status,
//                             newgps.position_type,
//                             // Time
//                             newgps.utc,
//                             mjd2human(newgps.utc).c_str(),
//                             // n sats
//                             newgps.n_sats_visible,
//                             newgps.n_sats_used,
//                             // LLA
//                             newgps.geo.lat,
//                             newgps.geo.lon,
//                             newgps.geo.h,
//                             // state
//                             newgps.position.col[0]/1.e3,
//                             newgps.position.col[1]/1.e3,
//                             newgps.position.col[2]/1.e3,
//                             newgps.velocity.col[0]/1.e3,
//                             newgps.velocity.col[1]/1.e3,
//                             newgps.velocity.col[2]/1.e3);

//                     if (debug_gps){
//                         cout << buffer << endl;
//                     }

//                     sensor_lock.lock();

//                     // if we have a good lock then we can control the beacon frequency
//                     // when we are newar Hawaii, Fairbanks and Surrey
//                     if ((newgps.solution_status == 0) && (newgps.position_type > 0)){

//                         // if LLA is within KCC radius increase beacon frequency: Lat 21.3928 Lon -157.984

//                         // default is 30 seconds
//                         beacon_frequency_high = false;

//                         // if we're less than 20 deg then increase frequency
//                         if ( (fabs(newgps.geo.lat - kcc.lat) < 20) &&  (fabs(newgps.geo.lon - kcc.lon) < 20) ){
//                             log_string = "near KCC, increase beacon frequency";

//                             beacon_frequency_high = true;
//                             // >> command isc to increase beacon frequency
//                         }

//                         // if LLA is within Fairbanks radius increase beacon frequency: Lat 64.8378 Lon -147.716,

//                         // if we're less than 20 deg then increase frequency
//                         if ( (fabs(newgps.geo.lat - fairbanks.lat) < 20) &&  (fabs(newgps.geo.lon - fairbanks.lon) < 20) ){
//                             log_string = "near Fairbanks, increase beacon frequency";

//                             beacon_frequency_high = true;
//                             // >> command isc to increase beacon frequency
//                         }

//                         // if LLA is within Surrey radius increase beacon frequency: Lat 51.2433 Lon 0.589496,


//                         // if we're less than 20 deg then increase frequency
//                         if ( (fabs(newgps.geo.lat - surrey.lat) < 20) &&  (fabs(newgps.geo.lon - surrey.lon) < 20) ){
//                             log_string = "near Surrey, increase beacon frequency";

//                             beacon_frequency_high = true;
//                             // >> command isc to increase beacon frequency
//                         }

//                         if ((beacon_frequency_high_previous == false) && (beacon_frequency_high == true)){
//                             // changed status on beacon frequency to high frequency (10 sec)

//                             // save the log string wiht the GS location
//                             cout << log_string << endl;
//                             log_agent(log_string);

//                             log_string = "changed beacon frequency to 10 sec";
//                             cout << log_string << endl;

//                             log_agent(log_string);
//                             iretn =system ("control_isc beacon_interval 10");

//                             // set flag so we don't have to repeat the command every loop
//                             beacon_frequency_high_previous = true;
//                         }

//                         if ((beacon_frequency_high_previous == true) && (beacon_frequency_high == false)){
//                             // changed status on beacon frequency to normal (30 sec)
//                             log_string = "changed beacon frequency to 30 sec";
//                             cout << log_string << endl;

//                             log_agent(log_string);
//                             // >> change to agent request if it's working at Kirtland
//                             iretn =system ("control_isc beacon_interval 30");

//                             // set flag so we don't have to repeat the command every loop
//                             beacon_frequency_high_previous = false;
//                         }


//                     }


//                     // >> Eric, please exaplain this averaging better?
//                     if (dt_position > ts_position)
//                     {
//                         lastposition.update(newgps.utc, newgps.position);
//                         if (ts_position < 8. &&
//                                 (lastposition.stdevy.a4[0] < 5*lastposition.resolution &&
//                                  lastposition.stdevy.a4[1] < 5*lastposition.resolution &&
//                                  lastposition.stdevy.a4[2] < 5*lastposition.resolution))
//                         {
//                             ts_position *= 2.;
//                         }
//                         else
//                         {
//                             if (ts_position > .1 &&
//                                     (lastposition.stdevy.a4[0] > 15*lastposition.resolution ||
//                                      lastposition.stdevy.a4[1] > 15*lastposition.resolution ||
//                                      lastposition.stdevy.a4[2] > 15*lastposition.resolution))
//                             {
//                                 ts_position /= 2.;
//                             }
//                         }
//                     }

//                     if (dt_velocity > ts_velocity)
//                     {
//                         lastvelocity.update(newgps.utc, newgps.velocity);
//                         if (ts_velocity < 8. &&
//                                 (lastvelocity.stdevy.a4[0] < 5*lastvelocity.resolution &&
//                                  lastvelocity.stdevy.a4[1] < 5*lastvelocity.resolution &&
//                                  lastvelocity.stdevy.a4[2] < 5*lastvelocity.resolution))
//                         {
//                             ts_velocity *= 2.;
//                         }
//                         else
//                         {
//                             if (ts_velocity > .1 &&
//                                     (lastvelocity.stdevy.a4[0] > 15*lastvelocity.resolution ||
//                                      lastvelocity.stdevy.a4[1] > 15*lastvelocity.resolution ||
//                                      lastvelocity.stdevy.a4[2] > 15*lastvelocity.resolution))
//                             {
//                                 ts_velocity /= 2.;
//                             }
//                         }
//                     }

//                     // if solution status is bad then propagate using gauss jackson propagator
//                     // from last time and state vector
//                     // >> implement

//                     if ((newgps.solution_status > 0) && (newgps.position_type == 0)){

//                         //                        int order = 6;
//                         //                        int mode = 1;
//                         //                        float dt = 1.;

//                         //                        gauss_jackson_init_eci(order,
//                         //                                               mode,
//                         //                                               dt,
//                         //                                               initState.utc,
//                         //                                               initState.pos.eci,
//                         //                                               initState.att.icrf,
//                         //                                               cdata);

//                         //                        // propagate
//                         //                        // >> maybe should also check if time is valid
//                         //                        gauss_jackson_propagate(cdata, currentmjd());
//                     }

//                     *(cdata->devspec.gps[0]) = newgps;

//                     sensor_lock.unlock();
//                 }
//             }
//         }

//         // every 1 minute log the status on the GPS
//         if ((currentmjd() - gps_last_log_utc)*86400 > 60 ){
//             log_agent(buffer);
//             gps_last_log_utc = currentmjd();
//         }

//         COSMOS_USLEEP(100000);
//     }

//     oemv_disconnect(&oemvhandle);
}

// ------------------------------------------------------------------
// collect date from IMU and ST
// >> break this into two loops, one for each sensor
void sensor_loop()
{
    int32_t iretn;
//    char buffer_imu[255] = {'\0'};
    char buffer_st[255] = {'\0'};

    float vmt35adjust[3][2][3]={{{0.,0.,0.},{0.,0.,0.}}
                                ,{{0.,0.,0.},{0.,0.,0.}}
                                ,{{0.,0.,0.},{0.,0.,0.}}};
    float vn100magscale[3]={0.,0.,0.};
    float vn100magbias[3]={0.,0.,0.};
    float vn100omegabias[3]={.0035,.0001,-.0065}; //this is actually read from the .ini file now, but I think I'll leave it here for now, just in case.

    // temp values to apply sensor transformation
    //double temp_x, temp_y, temp_z;

    // initiate DCM class for later use in sensor transformation
    DCM dcm;

    // define body frame base
    basisOrthonormal frame_body;
    frame_body.i = cv_unitx();
    frame_body.j = cv_unity();
    frame_body.k = cv_unitz();

    // define IMU frame
    // IMU+X is aligned with BODY+Y
    // IMU+Y is aligned with BODY+X
    // IMU+Z is aligned with BODY-Z
    basisOrthonormal frame_imu;
    frame_imu.i = {0,1,0};
    frame_imu.j = {1,0,0};
    frame_imu.k = {0,0,-1};

    // get the DCM to convert IMU vector into body vector
    cmatrix R_body_from_imu = dcm.base1_from_base2(frame_body,frame_imu);

    // define ST frame
    // >> must check with Lance
    basisOrthonormal frame_st;
    // ST+X (direction of connector) is aligned with BODY-X
    // ST+Y is triad between X and Z
    // ST+Z (direction of outward boresight) is aligned with BODY-Z
    frame_st.i = {-1,0,0};
    frame_st.j = {0,1,0};
    frame_st.k = {0,0,-1};

    // get the DCM to convert ST vector into body vector
    cmatrix R_body_from_st = dcm.base1_from_base2(frame_body,frame_st);

    // initiate RowVector class for later use
    //RowVector rv;

    bool newvalue;

    // Open calibration file for effect of VMT35
    FILE *fp;
//     string cnodedir = get_cnodedir(agent->cinfo->node.name);
//     string filename = cnodedir + "/vmt35.ini";
//     if ((fp=fopen(filename.c_str(),"r")) != NULL)
//     {
//         for (uint16_t i=0; i<3; ++i)
//         {
//             iretn = fscanf(fp, "%f %f %f %f %f %f\n",
//                            &vmt35adjust[i][0][0],
//                     &vmt35adjust[i][0][1],
//                     &vmt35adjust[i][0][2],
//                     &vmt35adjust[i][1][0],
//                     &vmt35adjust[i][1][1],
//                     &vmt35adjust[i][1][2]);
//         }
//         fclose(fp);
//     }

//     // Open calibration file for scaling of VN100
//     filename = cnodedir + "/vn100.ini";
//     if ((fp=fopen(filename.c_str(),"r")) != NULL)
//     {
//         iretn = fscanf(fp, "%f %f %f\n", &vn100magscale[0], &vn100magscale[1], &vn100magscale[2]);
//         iretn = fscanf(fp, "%f %f %f\n", &vn100magbias[0], &vn100magbias[1], &vn100magbias[2]);
//         iretn = fscanf(fp, "%f %f %f\n", &vn100omegabias[0], &vn100omegabias[1], &vn100omegabias[2]); // in rad/sec
//         fclose(fp);
//     }

//     timetilltorque = timetillstoptorque = currentmjd(0.);
//     double ts_imuomega = .2;
//     double ts_mag = .2;
//     double ts_att = .2;
//     double ts_sttomega = .2;
//     //	double lmjd = currentmjd(0.);
//     double nmjd = currentmjd(0.);

//     //sinclair_stt_result_operational result;

//     double st_last_log_utc = currentmjd();
// //    double imu_last_log_utc = currentmjd();

//     while (agent_running(cdata)){

//         double utc_now = currentmjd(0.);
//         //		printf("%f %.15g %.15g\n",86400.*(utc_now-lmjd), utc_now, nmjd);
//         //		lmjd = utc_now;
//         if (utc_now < nmjd)
//         {
//             //			printf("Sleep: %d\n", (uint32_t)(86400000000L*(nmjd - utc_now)));
//             COSMOS_USLEEP((uint32_t)(86400000000L*(nmjd - utc_now)));
//             utc_now = nmjd;
//         }

//         // >> broken for the moment
//         // Get Star Tracker values
//         newvalue = false;
//         if (cdata->devspec.stt[0].flag & DEVICE_FLAG_ACTIVE)
//         {
//             double utc_now = currentmjd(0.);
//             double dt_att = (utc_now-lastatt.lastx())*86400.;
//             double dt_sttomega = (utc_now-laststtomega.lastx())*86400.;
//             if (dt_att > ts_att ||  dt_sttomega > ts_sttomega){
//                 sttstruc newstt = *(cdata->devspec.stt[0]);
//                 //                if (cdata->devspec.stt[0].flag & DEVICE_FLAG_SIMULATED)
//                 //                {
//                 //                    if ((pbeat.utc || mbeat.utc) && agent->cinfo->devspec.stt[0].utc != 0.)
//                 //                    {
//                 //                        newstt.utc = agent->cinfo->devspec.stt[0].utc;
//                 //                        newstt.utc = currentmjd(0.);;
//                 //                        newstt.omega = agent->cinfo->devspec.stt[0].omega;
//                 //                        newstt.att = agent->cinfo->devspec.stt[0].att;
//                 //                        newstt.temp = agent->cinfo->devspec.stt[0].temp;
//                 //                        newstt.volt = agent->cinfo->devspec.stt[0].volt;

//                 //                        newvalue = true;
//                 //                    }
//                 //                }
//                 //                else
//                 //                {
//                 //if (stthandle.serial >= 0)
//                 if (stthandle.serial->fd >= 0)
//                 {
//                     //COSMOS_SLEEP(1);
//                     sinclair_stt_result_operational result;
//                     // >> segmentation fault problem
//                     iretn = sinclair_stt_combo(&stthandle, &result); // Sinclair STT command to read data from combo buffer
//                     if (iretn >= 0)
//                     {
//                         // >> MN: when the ST is on I'm getting a consisten return error of 48 =
//                         // if we assume this is 0x48 (0100 1000) then it should mean:
//                         // bit4 = full processing
//                         // bit6 = consistent image solutions
//                         // if we assume this is 48 decimal (0011 0000)
//                         // bit5 = detector image
//                         // bit6 = consistent image solutions
//                         // >> oh my! what does 48 mean?!?!??! it should be something like 0x7F = 127 for a sucessfull image
//                         // or zero, for no good attitude info. But not 48 ... what is 48?
//                         newstt.retcode = result.return_code;
//                         newstt.temp = result.htelem.detector_temperature;
//                         newstt.volt = result.htelem.vdd_core;
//                         newstt.utc = currentmjd(0.);
//                         newstt.omega = result.omega;
//                         newstt.att = result.attitude;
//                         // one other what to check the consistency of the data of the ST is by
//                         // looking at the quaternion, if the norm is zero then it's because the
//                         // solution is invalid.

//                         // >> convert the quaternion values from ST sensor frame to body frame

//                         // convert the omega values from ST sensor frame to body frame
//                         newstt.omega = rv_mmult(rm_from_cm(R_body_from_st),newstt.omega);

//                         //cout << newstt.omega << endl;

//                         newvalue = true;
//                     }
//                 }
//                 //                }

//                 if (newvalue)
//                 {
//                     sprintf(buffer_st,"UTC:%f ST RetCode: %d Temp: %f Volt: %f | Omega: %f %f %f | Quat: %f %f %f %f",
//                             newstt.utc,
//                             // status
//                             newstt.retcode,
//                             newstt.temp,
//                             newstt.volt,
//                             // Omega
//                             newstt.omega.col[0],
//                             newstt.omega.col[1],
//                             newstt.omega.col[2],
//                             // Att
//                             newstt.att.d.x,
//                             newstt.att.d.y,
//                             newstt.att.d.z,
//                             newstt.att.w
//                             );

//                     if (debug_st){
//                         cout << buffer_st << endl;
//                     }

//                     sensor_lock.lock();

//                     if (dt_att > ts_att)
//                     {
//                         lastatt.update(newstt.utc, newstt.att);
//                         if (ts_att < 8. &&
//                                 (lastatt.stdevy.a4[0] < 5*lastatt.resolution &&
//                                  lastatt.stdevy.a4[1] < 5*lastatt.resolution &&
//                                  lastatt.stdevy.a4[2] < 5*lastatt.resolution &&
//                                  lastatt.stdevy.a4[3] < 5*lastatt.resolution))
//                         {
//                             ts_att *= 2.;
//                         }
//                         else
//                         {
//                             if (ts_att > .1 &&
//                                     (lastatt.stdevy.a4[0] > 15*lastatt.resolution ||
//                                      lastatt.stdevy.a4[1] > 15*lastatt.resolution ||
//                                      lastatt.stdevy.a4[2] > 15*lastatt.resolution ||
//                                      lastatt.stdevy.a4[3] > 15*lastatt.resolution))
//                             {
//                                 ts_att /= 2.;
//                             }
//                         }
//                     }

//                     if (dt_sttomega > ts_sttomega)
//                     {
//                         laststtomega.update(newstt.utc, newstt.omega);
//                         newstt.alpha = laststtomega.slopervector(newstt.utc);
//                         if (ts_sttomega < 8. &&
//                                 (laststtomega.stdevy.a4[0] < 5*laststtomega.resolution &&
//                                  laststtomega.stdevy.a4[1] < 5*laststtomega.resolution &&
//                                  laststtomega.stdevy.a4[2] < 5*laststtomega.resolution))
//                         {
//                             ts_sttomega *= 2.;
//                         }
//                         else
//                         {
//                             if (ts_sttomega > .1 &&
//                                     (laststtomega.stdevy.a4[0] > 15*laststtomega.resolution ||
//                                      laststtomega.stdevy.a4[1] > 15*laststtomega.resolution ||
//                                      laststtomega.stdevy.a4[2] > 15*laststtomega.resolution))
//                             {
//                                 ts_sttomega /= 2.;
//                             }
//                         }
//                     }

//                     *(cdata->devspec.stt[0]) = newstt;

//                     sensor_lock.unlock();
//                 }
//             }
//         }

//         // Get IMU values
//         newvalue = false;
//         if (cdata->devspec.imu[0].flag & DEVICE_FLAG_ACTIVE)
//         {
//             double utc_now = currentmjd(0.);
//             double dt_mag = (utc_now-lastbodymag.lastx())*86400.;
//             double dt_imuomega = (utc_now-lastimuomega.lastx())*86400.;

//             if (dt_mag > ts_mag ||  dt_imuomega > ts_imuomega)
//             {
//                 imustruc newimu = *(cdata->devspec.imu[0]);
//                 //                if (cdata->devspec.imu[0].flag & DEVICE_FLAG_SIMULATED)
//                 //                {
//                 //                    if (pbeat.utc && agent->cinfo->devspec.imu[0].utc != 0.)
//                 //                    {
//                 //                        newimu.utc = agent->cinfo->devspec.imu[0].utc;
//                 //                        newimu.omega = agent->cinfo->devspec.imu[0].omega;
//                 //                        newimu.mag = agent->cinfo->devspec.imu[0].mag;
//                 //                        newimu.accel = agent->cinfo->devspec.imu[0].accel;
//                 //                        newimu.temp = agent->cinfo->devspec.imu[0].temp;
//                 //                        newvalue = true;
//                 //                    }
//                 //                }
//                 //                else
//                 //                {
//                 if ((iretn=vn100_measurements(&vn100handle)) >= 0)
//                 {
//                     newimu.utc = currentmjd(0.);
//                     // get temp data
//                     newimu.temp = vn100handle.imu.temp/10.;

//                     //-------------------------------------
//                     // get gyro data
//                     //if(0){//>> just temporary to simulate
//                     newimu.omega = vn100handle.imu.omega;
//                     // apply gyro bias
//                     newimu.omega.col[0] -= vn100omegabias[0];
//                     newimu.omega.col[1] -= vn100omegabias[1];
//                     newimu.omega.col[2] -= vn100omegabias[2];
//                     newimu.omega = rv_mmult(rm_from_cm(R_body_from_imu),newimu.omega);
//                     //}
//                     //-------------------------------------
//                     // get acceleration data
//                     newimu.accel = vn100handle.imu.accel;
//                     //cout << "Raw:" << newimu.accel << endl;
//                     newimu.accel = rv_mmult(rm_from_cm(R_body_from_imu),newimu.accel);
//                     //cout << "New:" << newimu.accel << endl;

//                     //-------------------------------------
//                     // get mag data from handle
//                     newimu.mag = vn100handle.imu.mag;
//                     // apply mag bias
//                     newimu.mag.col[0] -= vn100magbias[0];
//                     newimu.mag.col[1] -= vn100magbias[1];
//                     newimu.mag.col[2] -= vn100magbias[2];
//                     // apply mag scale
//                     newimu.mag.col[0] *= vn100magscale[0];
//                     newimu.mag.col[1] *= vn100magscale[1];
//                     newimu.mag.col[2] *= vn100magscale[2];

//                     // apply sensor body transformation so the readings are given in the body frame

//                     //                    temp_x = newimu.mag.col[0];
//                     //                    temp_y = newimu.mag.col[1];
//                     //                    temp_z = newimu.mag.col[2];

//                     //                        // IMU+X is aligned with BODY+Y
//                     //                        newimu.mag.col[1] = temp_x;

//                     //                        // IMU+Y is aligned with BODY+X
//                     //                        newimu.mag.col[0] = temp_y;

//                     //                        // IMU+Z is aligned with BODY-Z
//                     //                        newimu.mag.col[2] = -temp_z;

//                     // convert the mag values from sensor frame to body frame
//                     newimu.mag = rv_mmult(rm_from_cm(R_body_from_imu),newimu.mag);


//                     // >> try making the same transfosmation usign the alginment quaternion
//                     //rz = transform_q(cdata->devspec.rw[0].align,rv_unitz());


//                     //                    sprintf(buffer,"[%f, %f, %f](%f) [%f, %f, %f](%f)",
//                     //                            newimu.mag.col[0]*1e6,
//                     //                            newimu.mag.col[1]*1e6,
//                     //                            newimu.mag.col[2]*1e6,
//                     //                            1e6*sqrt(newimu.mag.col[0]*newimu.mag.col[0] + newimu.mag.col[1]*newimu.mag.col[1] + newimu.mag.col[2]*newimu.mag.col[2]),
//                     //                            newimu.omega.col[0]*180./PI,
//                     //                            newimu.omega.col[1]*180./PI,
//                     //                            newimu.omega.col[2]*180./PI,
//                     //                            180./PI*sqrt(newimu.omega.col[0]*newimu.omega.col[0] + newimu.omega.col[1]*newimu.omega.col[1] + newimu.omega.col[2]*newimu.omega.col[2])
//                     //                            );

//                     //                    cout << buffer << endl;

//                     newvalue = true;
//                 }
//                 //                }

//                 if (newvalue)
//                 {
//                     sensor_lock.lock();

//                     if (dt_imuomega > ts_imuomega)
//                     {
//                         lastimuomega.update(newimu.utc, newimu.omega);
//                         newimu.alpha = rv_smult(1./86400., lastimuomega.slopervector(newimu.utc));
//                         /*
//                                 double domega =  lastimuomega.resolution / ts_imuomega;
//                                 if (ts_imuomega < .8 && (lastimuomega.stdevy.a4[0] < lastimuomega.resolution && lastimuomega.stdevy.a4[1] < lastimuomega.resolution && lastimuomega.stdevy.a4[2] < lastimuomega.resolution))
//                                 {
//                                     ts_imuomega *= 2.;
//                                 }
//                                 if (ts_imuomega > .2 && (fabs(newimu.alpha.col[0]) > domega || fabs(newimu.alpha.col[1]) > domega || fabs(newimu.alpha.col[2]) > domega))
//                                 {
//                                     ts_imuomega /= 2.;
//                                 }
//                                 */
//                     }

//                     if (control_mode == CONTROL_MODE_OFF || (!torque_allowed && newimu.utc < timetilltorque))
//                     {
//                         if (dt_mag > ts_mag)
//                         {
//                             lastbodymag.update(newimu.utc, newimu.mag);
//                             //							quaternion tq = lastatt.evalquaternion(newimu.utc);
//                             //							printf("%f %f %f %f\n", tq.w, tq.d.x, tq.d.y, tq.d.z);
//                             lasticrfmag.update(newimu.utc, rotate_q(lastatt.evalquaternion(newimu.utc),newimu.mag));
//                             newimu.bdot = lastbodymag.slopervector(newimu.utc);
//                             if (ts_mag < 8. &&
//                                     (lastbodymag.stdevy.a4[0] < 5*lastbodymag.resolution &&
//                                      lastbodymag.stdevy.a4[1] < 5*lastbodymag.resolution &&
//                                      lastbodymag.stdevy.a4[2] < 5*lastbodymag.resolution))
//                             {
//                                 ts_mag *= 2.;
//                             }
//                             else
//                             {
//                                 if (ts_mag > .1 &&
//                                         (lastbodymag.stdevy.a4[0] > 15*lastbodymag.resolution ||
//                                          lastbodymag.stdevy.a4[1] > 15*lastbodymag.resolution ||
//                                          lastbodymag.stdevy.a4[2] > 15*lastbodymag.resolution))
//                                 {
//                                     ts_mag /= 2.;
//                                 }
//                             }
//                         }
//                     }
//                     // Estimate mag because IMU value is suspect
//                     else
//                     {
//                         // No torque allowed but exceeded timetilltorque
//                         if (!torque_allowed)
//                         {
//                             // Allow torque
//                             torque_allowed = true;
//                             // Set timetillstoptorque
//                             timetillstoptorque = newimu.utc + 3.2/86400.;
//                         }
//                         else
//                         {
//                             // Torque allowed but exceeded timetillstoptorque
//                             if (newimu.utc >= timetillstoptorque)
//                             {
//                                 // Disallow torque
//                                 torque_allowed = false;
//                                 // Set timetilltorque
//                                 timetilltorque = newimu.utc + .8/86400.;
//                             }

//                         }
//                     }

//                     // copy newimu back to cosmos data structure
//                     *(cdata->devspec.imu[0]) = newimu;

//                     sensor_lock.unlock();
//                 }
//             }
//             else
//             {
//                 // Estimate mag because IMU value is suspect
//                 if (control_mode != CONTROL_MODE_OFF)
//                 {
//                     // No torque allowed but exceeded timetilltorque
//                     if (!torque_allowed && utc_now >= timetilltorque)
//                     {
//                         // Allow torque
//                         torque_allowed = true;
//                         // Set timetillstoptorque
//                         timetillstoptorque = utc_now + 4./86400.;
//                     }
//                     else
//                     {
//                         // Torque allowed but exceeded timetillstoptorque
//                         if (torque_allowed && utc_now >= timetillstoptorque)
//                         {
//                             // Disallow torque
//                             torque_allowed = false;
//                             // Set timetilltorque
//                             timetilltorque = utc_now + 1./86400.;
//                         }

//                     }
//                 }
//             }

//             //			printf("%f %f [%f %f %f] ", ts_mag, lastbodymag.resolution, lastbodymag.stdevy.a4[0]/lastbodymag.resolution, lastbodymag.stdevy.a4[1]/lastbodymag.resolution, lastbodymag.stdevy.a4[2]/lastbodymag.resolution);
//             //printf("%f %f [%f %f %f] [%f %f %f]\n", ts_imuomega, lastimuomega.resolution/ts_imuomega, lastimuomega.stdevy.a4[0]/ts_imuomega, lastimuomega.stdevy.a4[1]/ts_imuomega, lastimuomega.stdevy.a4[2]/ts_imuomega, fabs(cdata->devspec.imu[0].alpha.col[0]), fabs(cdata->devspec.imu[0].alpha.col[1]), fabs(cdata->devspec.imu[0].alpha.col[2]));
//         }
//         nmjd = utc_now + (.05 / 86400.);

//         // every 1 minute log the status on the ST
//         // >> do same for IMU
//         if ((currentmjd() - st_last_log_utc)*86400 > 60 ){
//             log_agent(buffer_st);
//             st_last_log_utc = currentmjd();
//         }
//     }
}






int32_t turn_on_actuators(){

    //>> add request later when ISC is able to get requests
    int32_t iretn = -1;
    // turn on TCU (pdu_switch 3) for 15 minutes, then it's the job of the ADCS agent to keep the actuators on as needed
    iretn =system ("control_isc pdu_switch 3 900 >> NULL");

    // wait a second for the TCU to come up to life
    COSMOS_SLEEP(1);

    // >> check if device bus 3 volt and current is > 0, then probably it is on
    if (iretn!=0){
        log_agent("Error: isc-control command failed to turn on TCU");
        //exit (iretn);
    }

    // attempt to connect
    //cdata->devspec.tcu[0].flag = DEVICE_FLAG_OFF;
    // if ((iretn=vmt35_connect(cdata->port[cdata->devspec.tcu[0].portidx].name, &vmt35handle)) >= 0)
    // {
    //     cdata->devspec.tcu[0].flag |= DEVICE_FLAG_ACTIVE;
    //     for (uint32_t i=0; i<3; ++i)
    //     {
    //         cdata->devspec.mtr[i].flag |= DEVICE_FLAG_ACTIVE;
    //     }
    //     log_agent("TCU connected");
    //     vmt35_enable(&vmt35handle);
    // } else {
    //     log_agent("Error: Can't connect to TCU, going to run without TCU. Only sensors available");
    //     return -1;

    // }

    return 0;
}


//
int32_t turn_on_st(){

    int32_t iretn = -1;
    char isc_response[300] = {'\0'};

    // agent isc still breaks when it gets a request
    // if(0){
    //     if (iscbeat.utc != 0) {
    //         iretn = agent->send_request(iscbeat, "pdu_switch 4 900", isc_response, 300, 2.);
    //     }
    // }

    // // turn on ST (pdu_switch 3) for 15 minutes, then it's the job of the ADCS agent to keep the actuators on as needed
    // iretn =system ("control_isc pdu_switch 4 900 >> NULL");

    // // wait a second for the ST to come up to life
    // COSMOS_SLEEP(1);

    // // >> check if device bus 4 volt and current is > 0, then probably it is on
    // if (iretn!=0){
    //     log_agent("Error: isc-control command failed to turn on ST");
    //     //exit (iretn);
    // }


    // // >> create function turn_on_st()
    // // Open STT devices
    // cdata->devspec.stt[0].flag = DEVICE_FLAG_OFF;
    // if ((iretn = sinclair_stt_connect(cdata->port[cdata->devspec.stt[0].portidx].name,(uint8_t)0x11,(uint8_t)0x0E, &stthandle)) >= 0)
    // {
    //     log_agent("ST connected");
    //     cdata->devspec.stt[0].flag |= DEVICE_FLAG_ACTIVE;
    // }
    // else
    // {

    //     log_agent("Error: Can't connect to ST, going to try to turn on ST via isc-control");
    //     return -1;
    // }


    return 0;

}

// >> make the logic similar to turn_on_actuators()
int32_t turn_on_gps(){

    int32_t iretn = -1;

    //char isc_response[300] = {'\0'};
    string isc_response = "\0";
    string s = "pdu_switch 10 900";

    // agent isc still breaks when it gets a request
     if(0){
         if (iscbeat.utc != 0) {
             iretn = agent->send_request(iscbeat, s, isc_response, 300, 2.);
         }
     }

     // turn on GPS (pdu_switch 10) for 15 minutes, then it's the job of the ADCS agent to keep the GPS on as needed
     iretn =system ("control_isc pdu_switch 10 900 >> NULL");

     // wait a second for the ST to come up to life
     COSMOS_SLEEP(1);

     // >> check if device bus 3 volt and current is > 0, then probably it is on
     if (iretn!=0){
         log_agent("Error: isc-control command failed to turn on GPS");
         //exit (iretn);
     }


//     // Open GPS device
//     //cdata->devspec.gps[0].flag = DEVICE_FLAG_OFF;
//     if((iretn=oemv_connect(cdata->port[cdata->devspec.gps[0].portidx].name,
//                            &oemvhandle)) == 0) {
//         log_agent("GPS connected");

//         cdata->devspec.gps[0].flag |= DEVICE_FLAG_ACTIVE;
//     } else {
//         // check if the GPS is actually working already
//         // get the time

//         log_agent("Error: Can't connect to GPS, going to try to turn on GPS via isc-control");

//         // attempt to connect again
//         if((iretn=oemv_connect(cdata->port[cdata->devspec.gps[0].portidx].name,
//                                &oemvhandle)) != 0)
//         {
//             cdata->devspec.gps[0].flag |= DEVICE_FLAG_ACTIVE;
//             log_agent("GPS connected");

//         } else {

//             log_agent("Error: Can't connect to GPS, going to run without GPS.");

//             return -1;

//         }
//     }
    return 0;
}














// for simulation using motion tracker
int get_motion()
{

    memset((void *)result.c_str(),0,REQBUFSIZE);
    std::ostringstream buffer;
    buffer << "getvalue {"
             "\"device_stt_utc_000\","
             "\"device_stt_att_000\","
             "\"device_stt_omega_000\"}";
    string my_request = buffer.str();
    agent->send_request(mbeat,my_request,result,REQBUFSIZE,5);
    json_parse(result,agent->cinfo);

    return (0);
}

int set_simulated()
{
    std::ostringstream buffer;
    buffer << "setvalue {\"device_rw_romg_000\":" << romg << "},"
             "{\"device_rw_ralp_000\":" << ralp << "},"
             "{\"device_mtr_rmom_000\":" << mtr[0] << "},"
             "{\"device_mtr_rmom_001\":" << mtr[1] << "},"
             "{\"device_mtr_rmom_002\":" << mtr[2] << "},";
    string my_request = buffer.str();
    agent->send_request(pbeat,my_request,result,REQBUFSIZE,5);

    return (0);
}


// get physics for simulation
int get_physics()
{
    int32_t iretn = -1;

    agent->cinfo->devspec.gps[0].utc = agent->cinfo->devspec.imu[0].utc = agent->cinfo->devspec.stt[0].utc = 0;
    if (pbeat.utc)
    {
        request="getvalue {"
                "\"node_utcoffset\","
                "\"device_rw_utc_000\","
                "\"device_rw_omg_000\","
                "\"device_rw_alp_000\","
                "\"device_mtr_utc_000\","
                "\"device_mtr_mom_000\","
                "\"device_mtr_utc_001\","
                "\"device_mtr_mom_001\","
                "\"device_mtr_utc_002\","
                "\"device_mtr_mom_002\","
                "\"device_imu_utc_000\","
                "\"device_imu_mag_000\","
                "\"device_imu_omega_000\","
                "\"device_imu_accel_000\","
                "\"device_gps_utc_000\","
                "\"device_gps_pos_000\","
                "\"device_gps_vel_000\","
                "\"device_stt_utc_000\","
                "\"device_stt_att_000\","
                "\"device_stt_omega_000\"}";
        iretn = agent->send_request(pbeat,request,result,REQBUFSIZE,5);
        if (iretn > 0)
        {
            json_parse(result,agent->cinfo);
            agent->cinfo->node.utcoffset = agent->cinfo->node.utcoffset;
        }
    }

    if (iretn < 0)
    {
        agent->cinfo->devspec.gps[0].utc = agent->cinfo->devspec.imu[0].utc = agent->cinfo->devspec.stt[0].utc = currentmjd(0.);
        agent->cinfo->devspec.gps[0].geocs = GEOC_HONOLULU;
        agent->cinfo->devspec.gps[0].geocv = rv_zero();
    }

    return (0);
}






// ------------------------------------------------------------------

int32_t request_control_mode(string &request, string &response, Agent *agent)
{
    // colect up to 4 values (for point mode with quaternion)
    double value[4];
    sscanf(request.c_str(),"%*s %s %lf %lf %lf %lf",response,&value[0],&value[1],&value[2],&value[3]);

    // by default every time there is a control_mode request turn off the automatic mode
    automatic_mode = false; // this gives the user more control

    string control_mode_requested = string(response);

    if (control_mode_requested == "bdot"){
        cout << "Control mode changed to Bdot" << endl;
        control_mode = CONTROL_MODE_BDOT;
    }

    if (control_mode_requested == "lvlh"){
        cout << "Control mode changed to LVLH" << endl;
        control_mode = CONTROL_MODE_LVLH_HOLD;
    }

    if (control_mode_requested == "point"){
        cout << "Control mode changed to POINT" << endl;
        control_mode = CONTROL_MODE_POINT;
    }

    if (control_mode_requested == "pd"){
        cout << "Control mode changed to PD" << endl;
        control_mode = CONTROL_MODE_PD;

        pd_kp = value[0];
        pd_kd = value[1];
    }

    if (control_mode_requested == "off"){
        cout << "Control mode changed to OFF" << endl;
        control_mode = CONTROL_MODE_OFF;
    }

    if (control_mode_requested == "auto"){
        cout << "Control mode changed to AUTO" << endl;
        control_mode = CONTROL_MODE_AUTO;
        // we must turn on the automatic_mode flag
        automatic_mode = true;
    }

    if (control_mode_requested == "manual"){

        // >> add values
        log_agent("Control mode changed to MANUAL");

        control_mode = CONTROL_MODE_MANUAL;
        temp_mom.col[0] =  value[0];
        temp_mom.col[1] =  value[1];
        temp_mom.col[2] =  value[2];

        // >> how can I copy these values directly into cdata?
        //cdata->devspec.mtr[i].rmom.col[0] = value[0];
        //cdata->devspec.mtr[i].rmom.col[1] = value[1];
        //cdata->devspec.mtr[i].rmom.col[2] = value[2];

    }









    //    switch(response[0])
    //    {
    //    case 'a':
    //        control_mode = CONTROL_MODE_ADOT;
    //        break;
    //    case 'b':
    //        cout << "Control mode changed to Bdot" << endl;
    //        control_mode = CONTROL_MODE_BDOT;
    //        break;
    //    case 'l':
    //        control_mode = CONTROL_MODE_LEFT;
    //        break;
    //    case 'm':
    //        switch (response[1])
    //        {
    //        case 'a': // ok, MN confirmed, it works as expected from a request command
    //            control_mode = CONTROL_MODE_MANUAL;
    //            temp_mom.col[0] =  value[0];
    //            temp_mom.col[1] =  value[1];
    //            temp_mom.col[2] =  value[2];

    //            automatic_mode = false;

    //            // >> how can I copy these values directly into cdata?
    //            //cdata->devspec.mtr[i].rmom.col[0] = value[0];
    //            //cdata->devspec.mtr[i].rmom.col[1] = value[1];
    //            //cdata->devspec.mtr[i].rmom.col[2] = value[2];
    //            break;
    //        case 'i':
    //            control_mode = CONTROL_MODE_MINUS;
    //            break;
    //        }
    //        break;
    //    case 'n':
    //        control_mode = CONTROL_MODE_NOMINAL;
    //        break;
    //    case 'o':
    //        control_mode = CONTROL_MODE_OFF;
    //        break;
    //    case 'p':
    //        switch (response[1])
    //        {
    //        case 'd':
    //            control_mode = CONTROL_MODE_PD;
    //            pd_kp = value[0];
    //            pd_kd = value[1];
    //            break;
    //        case 'l':
    //            control_mode = CONTROL_MODE_PLUS;
    //            break;
    //        }
    //        break;
    //    case 'r':
    //        switch (response[1])
    //        {
    //        case 'a':
    //            control_mode = CONTROL_MODE_RATE;
    //            rotrate.col[0] = value[0];
    //            rotrate.col[1] = value[1];
    //            rotrate.col[2] = value[2];
    //            break;
    //        case 'i':
    //            control_mode = CONTROL_MODE_RIGHT;
    //            break;
    //        case 'o':
    //            control_mode = CONTROL_MODE_ROTVAL;
    //            rotval = RADOF(value[0]);
    //            break;
    //        }
    //        break;
    //    case 's':
    //        control_mode = CONTROL_MODE_STOP;
    //        break;
    //    case 't':
    //        control_mode = CONTROL_MODE_TEST;
    //        break;
    //    default:
    //        control_mode = CONTROL_MODE_BDOT;
    //        automatic_mode = true;
    //        break;
    //    }

    return (0);
}

int32_t request_log(char *request, char *response, void *cdata)
{
    sscanf(request,"%*s %lf",&log_period);

    return (0);
}

int32_t request_debug(char *request, char *response, void *cdata)
{

    char in1[100];
    int status;
    sscanf(request,"%*s %s %d",in1,&status);

    string debug_flag = string(in1);

    if (debug_flag == "gps"){
        debug_gps = status;
    }

    if (debug_flag == "st"){
        debug_st = status;
    }

    if (debug_flag == "control"){
        debug_control = status;
    }

    // delete this one
    if (debug_flag == "print"){
        debug_print = status;
    }

    return (0);
}


int32_t request_reset_gps(string &request, string &response, Agent *agent)
{

    int32_t iretn;
    iretn =system ("control_isc pdu_auto_on_disable 10");
    COSMOS_SLEEP(0.5);
    iretn =system ("control_isc pdu_switch 10 0");
    COSMOS_SLEEP(0.5);
    iretn =system ("control_isc pdu_switch 10 900");
    COSMOS_SLEEP(0.5);
    iretn =system ("control_isc pdu_auto_on_enable 10");

	return iretn;
}



int32_t request_turn_on_actuators(char *request, char* response, void *cdata){

    turn_on_actuators();
    return 0;
}
