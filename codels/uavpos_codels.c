/*
 * Copyright (c) 2015-2018 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *                                      Anthony Mallet on Thu May 31 2018
 */
#include "acuavpos.h"

#include <sys/time.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "uavpos_c_types.h"
#include "codels.h"


/* --- Attribute set_emerg ---------------------------------------------- */

/** Validation codel uavpos_set_emerg of attribute set_emerg.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
uavpos_set_emerg(uavpos_ids_servo_s_emerg_s *emerg,
                 const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  emerg->dx = emerg->dx * emerg->dx / 9.;
  emerg->dv = emerg->dv * emerg->dv / 9.;
  return genom_ok;
}


/* --- Function set_state ----------------------------------------------- */

/** Codel uavpos_set_state of function set_state.
 *
 * Returns genom_ok.
 */
genom_event
uavpos_set_state(const or_t3d_pos *pos, const or_t3d_att *att,
                 const or_t3d_vel *vel, const or_t3d_avel *avel,
                 const or_t3d_acc *acc, const or_t3d_aacc *aacc,
                 const or_t3d_jerk *jerk, const or_t3d_snap *snap,
                 or_rigid_body_state *reference,
                 const genom_context self)
{
  (void)self; /* -Wunused-parameter */
  struct timeval tv;

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  if (isnan(pos->x))
    reference->pos._present = false;
  else {
    reference->pos._present = true;
    reference->pos._value = *pos;
  }

  if (isnan(att->qw))
    reference->att._present = false;
  else {
    reference->att._present = true;
    reference->att._value = *att;
  }

  if (isnan(vel->vx))
    reference->vel._present = false;
  else {
    reference->vel._present = true;
    reference->vel._value = *vel;
  }

  if (isnan(avel->wx))
    reference->avel._present = false;
  else {
    reference->avel._present = true;
    reference->avel._value = *avel;
  }

  if (isnan(acc->ax))
    reference->acc._present = false;
  else {
    reference->acc._present = true;
    reference->acc._value = *acc;
  }

  if (isnan(aacc->awx))
    reference->aacc._present = false;
  else {
    reference->aacc._present = true;
    reference->aacc._value = *aacc;
  }

  if (isnan(jerk->jx))
    reference->jerk._present = false;
  else {
    reference->jerk._present = true;
    reference->jerk._value = *jerk;
  }

  if (isnan(snap->sx))
    reference->snap._present = false;
  else {
    reference->snap._present = true;
    reference->snap._value = *snap;
  }

  return genom_ok;
}


/* --- Function set_position -------------------------------------------- */

/** Codel uavpos_set_position of function set_position.
 *
 * Returns genom_ok.
 */
genom_event
uavpos_set_position(double x, double y, double z, double yaw,
                    or_rigid_body_state *reference,
                    const genom_context self)
{
  (void)self; /* -Wunused-parameter */
  struct timeval tv;

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  reference->pos._present = true;
  reference->pos._value.x = x;
  reference->pos._value.y = y;
  reference->pos._value.z = z;

  reference->att._present = true;
  reference->att._value.qw = cos(yaw/2.);
  reference->att._value.qx = 0.;
  reference->att._value.qy = 0.;
  reference->att._value.qz = sin(yaw/2.);

  reference->vel._present = true;
  reference->vel._value.vx = 0.;
  reference->vel._value.vy = 0.;
  reference->vel._value.vz = 0.;

  reference->avel._present = true;
  reference->avel._value.wx = 0.;
  reference->avel._value.wy = 0.;
  reference->avel._value.wz = 0.;

  reference->acc._present = true;
  reference->acc._value.ax = 0.;
  reference->acc._value.ay = 0.;
  reference->acc._value.az = 0.;

  reference->aacc._present = true;
  reference->aacc._value.awx = 0.;
  reference->aacc._value.awy = 0.;
  reference->aacc._value.awz = 0.;

  reference->jerk._present = true;
  reference->jerk._value.jx = 0.;
  reference->jerk._value.jy = 0.;
  reference->jerk._value.jz = 0.;

  reference->snap._present = true;
  reference->snap._value.sx = 0.;
  reference->snap._value.sy = 0.;
  reference->snap._value.sz = 0.;

  return genom_ok;
}


/* --- Function stop ---------------------------------------------------- */

/** Codel uavpos_servo_stop of function stop.
 *
 * Returns genom_ok.
 */
genom_event
uavpos_servo_stop(or_rigid_body_state *reference,
                  const genom_context self)
{
  (void)self; /* -Wunused-parameter */
  struct timeval tv;

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  reference->pos._present = false;
  reference->att._present = false;
  reference->vel._present = false;
  reference->avel._present = false;
  reference->acc._present = false;
  reference->aacc._present = false;
  reference->jerk._present = false;
  reference->snap._present = false;

  return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel uavpos_log of function log.
 *
 * Returns genom_ok.
 * Throws uavpos_e_sys.
 */
genom_event
uavpos_log(const char path[64], uint32_t decimation,
           uavpos_log_s **log, const genom_context self)
{
  int fd;

  fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
  if (fd < 0) return uavpos_e_sys_error(path, self);

  if (write(fd, uavpos_log_header_fmt "\n", sizeof(uavpos_log_header_fmt)) < 0)
    return uavpos_e_sys_error(path, self);

  if ((*log)->req.aio_fildes >= 0) {
    close((*log)->req.aio_fildes);

    if ((*log)->pending)
      while (aio_error(&(*log)->req) == EINPROGRESS)
        /* empty body */;
  }
  (*log)->req.aio_fildes = fd;
  (*log)->pending = false;
  (*log)->skipped = false;
  (*log)->decimation = decimation < 1 ? 1 : decimation;
  (*log)->missed = 0;
  (*log)->total = 0;

  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel uavpos_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
uavpos_log_stop(uavpos_log_s **log, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  if (*log && (*log)->req.aio_fildes >= 0)
    close((*log)->req.aio_fildes);
  (*log)->req.aio_fildes = -1;

  return genom_ok;
}


/* --- Function log_info ------------------------------------------------ */

/** Codel uavpos_log_info of function log_info.
 *
 * Returns genom_ok.
 */
genom_event
uavpos_log_info(const uavpos_log_s *log, uint32_t *miss,
                uint32_t *total, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  *miss = *total = 0;
  if (log) {
    *miss = log->missed;
    *total = log->total;
  }
  return genom_ok;
}
