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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "uavpos_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */


/** Codel uavpos_main_start of task main.
 *
 * Triggered by uavpos_start.
 * Yields to uavpos_control.
 */
genom_event
uavpos_main_start(uavpos_ids *ids, const uavpos_uav_input *uav_input,
                  const genom_context self)
{
  or_uav_input *input_data;

  ids->body = (uavpos_ids_body_s){
    /* mikrokopter quadrotors defaults */
    .J = {
      0.015,    0.,    0.,
      0.,    0.015,    0.,
      0.,       0., 0.015
    },

    .mass = 1.0,
  };

  ids->servo = (uavpos_ids_servo_s){
    .sat = { .x = 0.10, .v = 0.1, .ix = 0.10 },
    .gain = {
      .Kpxy = 14., .Kvxy = 7., .Kpz = 20., .Kvz = 10.,
      .Kixy = 0., .Kiz = 0.
    },

    .emerg = {
      .descent = .1,
      .dx = 0.05 * 0.05 /9.,
      .dv = 0.2 * 0.2 /9.,
    }
  };

  ids->reference = (or_rigid_body_state){
    .ts = { .sec = 0, .nsec = 0 },
    .intrinsic = false,
    .pos._present = false,
    .att._present = false,
    .vel._present = false,
    .avel._present = false,
    .acc._present = false,
    .aacc._present = false,
    .jerk._present = false,
    .snap._present = false,
  };

  input_data = uav_input->data(self);
  *input_data = (or_uav_input){
    .ts = { .sec = 0, .nsec = 0 },
    .intrinsic = false,
    .thrust = { ._present = false },
    .att = { ._present = false },
    .avel = { ._present = false },
    .aacc = { ._present = false }
  };
  uav_input->write(self);


  /* init logging */
  ids->log = malloc(sizeof(*ids->log));
  if (!ids->log) abort();
  *ids->log = (uavpos_log_s){
    .req = {
      .aio_fildes = -1,
      .aio_offset = 0,
      .aio_buf = ids->log->buffer,
      .aio_nbytes = 0,
      .aio_reqprio = 0,
      .aio_sigevent = { .sigev_notify = SIGEV_NONE },
      .aio_lio_opcode = LIO_NOP
    },
    .pending = false, .skipped = false,
    .decimation = 1, .missed = 0, .total = 0
  };

  return uavpos_control;
}


/** Codel uavpos_main_control of task main.
 *
 * Triggered by uavpos_control.
 * Yields to uavpos_pause_control.
 */
genom_event
uavpos_main_control(const uavpos_ids_body_s *body,
                    uavpos_ids_servo_s *servo,
                    const uavpos_state *state,
                    or_rigid_body_state *reference, uavpos_log_s **log,
                    const uavpos_uav_input *uav_input,
                    const genom_context self)
{
  const or_pose_estimator_state *state_data = NULL;
  or_uav_input *input_data;
  struct timeval tv;
  int s;

  /* publish only upon reception of the first valid position */
  if (reference->ts.sec == 0) return uavpos_pause_control;

  input_data = uav_input->data(self);
  if (!input_data) return uavpos_pause_control;
  gettimeofday(&tv, NULL);

  /* reset control by default */
  input_data->thrust._present = false;
  input_data->att._present = false;
  input_data->avel._present = false;
  input_data->aacc._present = false;

  /* current state */
  if (state->read(self) || !(state_data = state->data(self)))
    goto output;
  if (tv.tv_sec + 1e-6 * tv.tv_usec >
      0.5 + state_data->ts.sec + 1e-9 * state_data->ts.nsec)
    goto output;

  /* deal with obsolete reference */
  if (tv.tv_sec + 1e-6 * tv.tv_usec >
      0.5 + reference->ts.sec + 1e-9 * reference->ts.nsec) {
    reference->vel._present = false;
    reference->avel._present = false;
    reference->acc._present = false;
    reference->aacc._present = false;
    reference->jerk._present = false;
    reference->snap._present = false;
  }

  /* position controller */
  s = uavpos_controller(body, servo, state_data, reference, *log, input_data);
  if (s) return uavpos_pause_control;

  /* output */
output:
  if (state_data) {
    input_data->ts = state_data->ts;
  } else {
    input_data->ts.sec = tv.tv_sec;
    input_data->ts.nsec = tv.tv_usec * 1000;
  }
  input_data->intrinsic = false;

  uav_input->write(self);

  return uavpos_pause_control;
}


/** Codel mk_main_stop of task main.
 *
 * Triggered by uavpos_stop.
 * Yields to uavpos_ether.
 */
genom_event
mk_main_stop(const uavpos_uav_input *uav_input,
             const genom_context self)
{
  or_uav_input *input_data;
  struct timeval tv;

  input_data = uav_input->data(self);
  if (!input_data) return uavpos_ether;

  gettimeofday(&tv, NULL);
  input_data->ts.sec = tv.tv_sec;
  input_data->ts.nsec = tv.tv_usec * 1000;
  input_data->intrinsic = false;

  input_data->thrust = (optional_or_rb3d_force){
    ._present = true,
    ._value = { .x = 0., .y = 0., .z = 0. }
  };
  input_data->att._present = false;
  input_data->avel._present = false;
  input_data->aacc._present = false;

  uav_input->write(self);

  return uavpos_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel uavpos_servo_loop of activity servo.
 *
 * Triggered by uavpos_start.
 * Yields to uavpos_pause_start, uavpos_ether.
 * Throws uavpos_e_input.
 */
genom_event
uavpos_servo_loop(const uavpos_reference *in,
                  or_rigid_body_state *reference,
                  const genom_context self)
{
  const or_rigid_body_state *in_data;

  if (in->read(self)) return uavpos_e_input(self);
  in_data = in->data(self);
  if (!in_data) return uavpos_e_input(self);

  /* check if timestamps have changed */
  if (reference->ts.nsec != in_data->ts.nsec ||
      reference->ts.sec != in_data->ts.sec)
    *reference = *in_data;

  return uavpos_pause_start;
}


/* --- Activity set_current_position ------------------------------------ */

/** Codel uavpos_set_current_position of activity set_current_position.
 *
 * Triggered by uavpos_start.
 * Yields to uavpos_ether.
 * Throws uavpos_e_input.
 */
genom_event
uavpos_set_current_position(const uavpos_state *state,
                            or_rigid_body_state *reference,
                            const genom_context self)
{
  const or_pose_estimator_state *state_data;
  double qw, qx, qy, qz;
  double yaw;

  if (state->read(self)) return uavpos_e_input(self);
  state_data = state->data(self);
  if (!state_data) return uavpos_e_input(self);
  if (!state_data->pos._present) return uavpos_e_input(self);
  if (!state_data->att._present) return uavpos_e_input(self);

  qw = state_data->att._value.qw;
  qx = state_data->att._value.qx;
  qy = state_data->att._value.qy;
  qz = state_data->att._value.qz;
  yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

  reference->ts = state_data->ts;
  reference->intrinsic = false;

  reference->pos._present = true;
  reference->pos._value.x = state_data->pos._value.x;
  reference->pos._value.y = state_data->pos._value.y;
  reference->pos._value.z = state_data->pos._value.z;

  reference->att._present = true;
  reference->att._value.qw = cos(yaw/2.);
  reference->att._value.qx = 0.;
  reference->att._value.qy = 0.;
  reference->att._value.qz = sin(yaw/2.);

  reference->vel._present = false;
  reference->avel._present = false;
  reference->acc._present = false;
  reference->aacc._present = false;
  reference->jerk._present = false;
  reference->snap._present = false;

  return uavpos_ether;
}
