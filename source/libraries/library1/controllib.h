#ifndef _CTLLIB_H
#define _CTLLIB_H 1

/*! \file controllib.h
	\brief controllib include file
	A library of functions to be used in support of Attitude control.
*/

//! \ingroup support
//! \defgroup controllib Attitude Control Library
//! \brief Attitude Control support library.
//! Various functions for the calculation of both torques and hardware settings necessary to
//! maintain a given attitude.

#include "controllib.h"
#include "support/jsonlib.h"
#include "physics/physicslib.h"
#include "support/datalib.h"
//#include "nodelib.h"
#include "math/mathlib.h"
using namespace Convert;


//! \ingroup controllib
//! \defgroup controllib_functions Attitude Control Library functions
//! @{

rvector calc_control_torque(double gain, qatt tatt, qatt catt, rvector moi);
rvector calc_control_torque_b(double gain, qatt tatt, qatt catt, rvector moi);
void calc_magnetic_torque(rvector torque, rvector* mtorque, rvector* magmom, rvector bbody);
void calc_hardware_torque(rvector torque, rvector *rtorque, rvector *mtorque, double *ralp, double *mtrx, double *mtry, double *mtrz, cosmosstruc *cosmos_data);
void calc_hardware_torque_x(rvector torque, rvector *rtorque, rvector *mtorque, double *ralp, double *mtrx, double *mtry, double *mtrz, cosmosstruc *cosmos_data);
rvector calc_control_torque_pd(double kp, double kd, qatt tatt, qatt catt);

//! @}

#endif
