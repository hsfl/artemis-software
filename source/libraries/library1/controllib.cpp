#include <stdlib.h>
#include <cstring>
#include "controllib.h"

//extern cosmosstruc cdata;

//! \addtogroup controllib_functions
//! @{

//! Calculate control torque - positional
/*! Calculate the control torque that should be applied to reach a target attitude from a current attitude.
    \param gain Portion of change to be achieved per time step.
    \param tatt Target ICRF attitude quaternion.
    \param catt Current ICRF attitude quaternion.
    \moi Principle moments of inertia.
*/
rvector calc_control_torque(double gain, qatt tatt, qatt catt, rvector moi)
{
    rmatrix mom, rm;
    double dalp, domg, dt;
    rvector alpha, omega, torque;
    rvector distance;
    quaternion dsq2, endatt, startatt;

    if ((dt=(tatt.utc-catt.utc)*86400.) > 0.)
    {
        // Conversion from ICRF to Body
        rm = rm_quaternion2dcm(catt.s);
        // Moment of Inertia in Body
        mom = rm_diag(moi);
        // Moment of Inertia in ICRF
        mom = rm_mmult(rm_transpose(rm),rm_mmult(mom,rm));

        // Correct for where we are going
        dsq2 = q_axis2quaternion_rv(rv_smult(dt,catt.v));
        startatt = q_mult(dsq2,catt.s);
        q_normalize(&startatt);
        //	startatt = catt.s;

        endatt = tatt.s;

        // Difference in position
        dsq2 = q_mult(endatt,q_conjugate(startatt));
        distance = rv_quaternion2axis(dsq2);
        domg = length_rv(distance);
        if (domg > DPI)
        {
            dsq2 = q_smult(-1.,dsq2);
            distance = rv_quaternion2axis(dsq2);
        }

        // Match velocity
        omega = rv_smult(gain/dt,distance);
        omega = rv_add(omega,tatt.v);
        alpha = rv_sub(omega,catt.v);
        dalp = gain * length_rv(alpha) / dt;
        alpha = rv_smult(dalp,rv_normal(alpha));

        torque = rv_mmult(mom,alpha);
        return (torque);
    }
    else
    {
        return (rv_zero());
    }
}

//! Calculate control torque - angular attitude and rate based
/*! Calculate the control torque that should be applied to reach a target attitude from a current attitude.
    \param gain Portion of change to be achieved per time step.
    \param tatt Target ICRF attitude quaternion.
    \param catt Current ICRF attitude quaternion.
    \moi Principle moments of inertia.
*/
rvector calc_control_torque_b(double lag, qatt tatt, qatt catt, rvector moi)
{
    rmatrix mom, rm;
    double dalp, domg, dt;
    rvector alpha, omega, torque;
    rvector distance;
    quaternion dsq2;

    if ((dt=(tatt.utc-catt.utc)*86400.) > 0.)
    {

        // Calculate adjustment for ICRF distance

        if (length_q(tatt.s))
        {
            dsq2 = q_mult(tatt.s,q_conjugate(catt.s));
            distance = rv_quaternion2axis(dsq2);
            domg = length_rv(distance);
            if (domg > DPI)
            {
                dsq2 = q_smult(-1.,dsq2);
                distance = rv_quaternion2axis(dsq2);
            }

            omega = rv_smult(1./lag,distance);
        }
        // If Target quaternion is zero, adjust only for velocity
        else
        {
            omega = rv_zero();
        }

        // Match velocity
        omega = rv_add(omega,tatt.v);
        alpha = rv_sub(omega,catt.v);
        dalp = length_rv(alpha) / lag;
        alpha = rv_smult(dalp,rv_normal(alpha));

        // Conversion from ICRF to Body
        rm = rm_quaternion2dcm(catt.s);
        // Moment of Inertia in Body
        mom = rm_diag(moi);
        // Moment of Inertia in ICRF
        mom = rm_mmult(rm_transpose(rm),rm_mmult(mom,rm));
        torque = rv_mmult(mom,alpha);

        return (torque);
    }
    else
    {
        return (rv_zero());
    }
}

//! Turn desired torque into magnetic moments.
/*! Partition a desired torque in the body frame into magnetic moments x, y, and z.
 * \param torque Desired torque.
 * \param mtorque Achieved magnetic torque.
 * \param magmom Primary moments of inertia.
 * \param bbody Earth's magnetic field in the body frame.
 */
void calc_magnetic_torque(rvector torque, rvector* mtorque, rvector* magmom, rvector bbody)
{
    rvector dtorque;

    // Transform bearth in to body frame
    double db, bangle;
    db = length_rv(bbody);
    dtorque = torque;

    bangle = sep_rv(dtorque,bbody);

    // Determine mag torque
    if ((bangle > RADOF(45.) && bangle < RADOF(135.)))
    {
        double mt_tau;
        mt_tau = length_rv(dtorque) * sin(bangle);
        *magmom = rv_smult(mt_tau/db,rv_normal(rv_cross(bbody,dtorque)));
        *mtorque = rv_cross(*magmom,bbody);
    }
    else
    {
        *magmom = rv_zero();
        *mtorque = rv_zero();
    }
}

void calc_hardware_torque_x(rvector torque, rvector *rtorque, rvector *mtorque, double *ralp, double *mtrx, double *mtry, double *mtrz, cosmosstruc *cdata)
{
    rvector dtorque, bearth, mmoment, rz;

    // Transform bearth in to body frame
    bearth = transform_q(cdata->node.loc.att.geoc.s,cdata->node.loc.bearth);
    double db, bangle, banglen;
    db = length_rv(bearth);
    dtorque = torque;

    bangle = sep_rv(dtorque,bearth);

    // Determine mag torque
    if ((bangle > RADOF(45.) && bangle < RADOF(135.)))
    {
        double mt_tau;
        mt_tau = length_rv(dtorque) * sin(bangle);
        if (mt_tau > cdata->devspec.mtr[0]->mxmom * db)
            mt_tau = cdata->devspec.mtr[0]->mxmom * db;
        if (mt_tau < -cdata->devspec.mtr[0]->mxmom * db)
            mt_tau = -cdata->devspec.mtr[0]->mxmom * db;
        mmoment = rv_smult(mt_tau/db,rv_normal(rv_cross(bearth,dtorque)));
        *mtorque = rv_cross(mmoment,bearth);

        // Hunt for best contribution of Reaction Wheel
        double scale, scalestep;

        bangle = sep_rv(*mtorque,torque);
        scalestep = length_rv(*mtorque) / 33.;
        scale = scalestep;
        rz = transform_q(cdata->devspec.rw[0]->align,rv_unitz());
        *rtorque = rv_smult(scale, rz);
        banglen = sep_rv(rv_add(*mtorque,*rtorque),torque);
        if (fabs(banglen) < fabs(bangle))
        {
            do
            {
                bangle = banglen;
                scale += scalestep;
                *rtorque = rv_smult(scale, rz);
                banglen = sep_rv(rv_add(*mtorque,*rtorque),torque);
            } while (scale <= 11. * scalestep && fabs(banglen) < fabs(bangle));
            *rtorque = rv_smult(scale-scalestep, rz);
        }
        else
        {
            banglen = sep_rv(rv_sub(*mtorque,*rtorque),torque);
            if (banglen < bangle)
            {
                do
                {
                    bangle = banglen;
                    scale += scalestep;
                    *rtorque = rv_smult(scale, rz);
                    banglen = sep_rv(rv_sub(*mtorque,*rtorque),torque);
                } while (scale <= 11. * scalestep && fabs(banglen) < fabs(bangle));
                *rtorque = rv_smult(-(scale-scalestep), rz);
            }
        }
        *ralp = length_rv(*rtorque) / cdata->devspec.rw[0]->mom.col[2];
    }
    else
    {
        mmoment = rv_zero();
        *mtorque = rv_zero();
        *rtorque = rv_zero();
        *ralp = 0;
    }

    *mtrx = mmoment.col[0];
    *mtry = mmoment.col[1];
    *mtrz = mmoment.col[2];
}

void calc_hardware_torque(rvector torque, rvector *rtorque, rvector *mtorque, double *ralp, double *mtrx, double *mtry, double *mtrz, cosmosstruc *cdata)
{
    rvector dtorque, bearth, mmoment, rz;
    double rangle, tangle, rw_tau, db, mt_tau;

    // Transform bearth in to body frame
    bearth = transform_q(cdata->node.loc.att.geoc.s,cdata->node.loc.bearth);
    db = length_rv(bearth);
    dtorque = torque;

    // Correct for angular momentum due to satellite rotation
    // Velocity of reaction wheel in ICRF
    rz = transform_q(cdata->devspec.rw[0]->align,rv_unitz());

    // MOI of reaction wheel in ICRF
    //	rm = rm_quaternion2dcm(cdata->devspec.rw[0]->align);
    //	mom = rm_mmult(rm,rm_mmult(rm_diag(cdata->devspec.rw[0]->mom),rm_transpose(rm)));

    // Delta MOI in ICRF
    //	tskew = rm_skew(loc.att.icrf.v);
    //	di = rm_mmult(loc.att.extra.b2j,rm_mmult(mom,rm_mmult(rm_smult(-1.,tskew),loc.att.extra.j2b)));
    //	di = rm_add(di,rm_mmult(tskew,rm_mmult(loc.att.extra.b2j,rm_mmult(mom,loc.att.extra.j2b))));

    // Angle between reaction wheel torque and B field in body frame
    rangle = sep_rv(rz,bearth);
    //	tangle = sep_rv(dtorque,bearth);

    // Only use reaction wheel for fine adjustments
    if (length_rv(torque) < 1.e-4)
    {
        // Maximum RW torque
        rw_tau = *ralp = 0.;

        if (rangle/DPI2 >= .8 && rangle/DPI2 <= 1.2)
        {
            if (cdata->devspec.rw[0]->omg < 350.)
            {
                rw_tau = .1 * cdata->devspec.rw[0]->mom.col[0];
                *ralp = .1;
            }
            if (cdata->devspec.rw[0]->omg > 450.)
            {
                rw_tau = -.1 * cdata->devspec.rw[0]->mom.col[0];
                *ralp = -.1;
            }
        }
        *rtorque = transform_q(cdata->devspec.rw[0]->align,rv_smult(rw_tau,rv_unitz()));
    }
    else
    {
        rw_tau = *ralp = 0;
        *rtorque = rv_zero();
    }

    // Angle between remaining torque and B field in body frame
    dtorque = rv_sub(dtorque,*rtorque);
    tangle = sep_rv(dtorque,bearth);

    // Maximum MTR torque
    mt_tau = length_rv(dtorque) * sin(tangle);
    if (mt_tau > cdata->devspec.mtr[0]->mxmom * db)
        mt_tau = cdata->devspec.mtr[0]->mxmom * db;
    if (mt_tau < -cdata->devspec.mtr[0]->mxmom * db)
        mt_tau = -cdata->devspec.mtr[0]->mxmom * db;
    if (mt_tau)
    {
        mmoment = rv_smult(mt_tau/db,rv_normal(rv_cross(bearth,dtorque)));
        *mtorque = rv_cross(mmoment,bearth);
    }
    else
    {
        mmoment = rv_zero();
        *mtorque = rv_zero();
    }
    db = 0;
    //	*ralp = cdata->devspec.rw[0]->ralp;
    *mtrx = mmoment.col[0];
    *mtry = mmoment.col[1];
    *mtrz = mmoment.col[2];
}


//! Calculate control torque - Miguel (PD)
/*! bla
    \param kp Proportional Gain.
    \param kd Derivative Gain.
    \param tatt Target ICRF attitude quaternion.
    \param catt Current ICRF attitude quaternion.
    \moi Principle moments of inertia.
*/
rvector calc_control_torque_pd(double kp, double kd, qatt tatt, qatt catt){

    // qatt contains {attitude, omega, alpha}
    static quaternion q_error_last = {{0,0,0},0}; // att
    quaternion q_error;
    quaternion q_derror;

    q_error = q_mult(tatt.s,catt.s); //q_target
    q_derror = q_smult(1/((tatt.utc-catt.utc)*86400.), q_sub(q_error,q_error_last));

    q_error_last = q_error;

    rvector error = rv_quaternion2axis(q_error);
    double domg = length_rv(error);
    if (domg > DPI)
    {
        q_error = q_smult(-1.,q_error);
        error = rv_quaternion2axis(q_error);
    }

    //	rvector derror = rv_sub(tatt.v, catt.v);

    rvector derror;
    derror.col[0] = q_derror.d.x;
    derror.col[1] = q_derror.d.y;
    derror.col[2] = q_derror.d.z;

    //printf("\nerror: %f, %f, %f\n", error.col[0], error.col[1], error.col[2]);
    //printf("\nderror: %f, %f, %f\n", derror.col[0], derror.col[1], derror.col[2]);

    rvector torque;

    torque.col[0] = kp*error.col[0] + kd*derror.col[0];
    torque.col[1] = kp*error.col[1] + kd*derror.col[1];
    torque.col[2] = kp*error.col[2] + kd*derror.col[2];

    // convert to inertial frame?
    torque = rotate_q(catt.s,torque);

    return torque;

}

//! @}





