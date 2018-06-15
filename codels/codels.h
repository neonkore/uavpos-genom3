/*
 * Copyright (c) 2016-2018 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Anthony Mallet on Thu May 31 2018
 */
#ifndef H_UAVPOS_CODELS
#define H_UAVPOS_CODELS

#include <aio.h>
#include <errno.h>
#include <string.h>

#include "uavpos_c_types.h"

#ifdef __cplusplus
extern "C" {
#endif

  int	uavpos_controller(const uavpos_ids_body_s *body,
                          const uavpos_ids_servo_s *servo,
                          const or_pose_estimator_state *state,
                          const or_rigid_body_state *desired,
                          uavpos_log_s *log,
                          or_uav_input *uav_input);

#ifdef __cplusplus
}
#endif

static inline genom_event
uavpos_e_sys_error(const char *s, genom_context self)
{
  uavpos_e_sys_detail d;
  size_t l = 0;

  d.code = errno;
  if (s) {
    strncpy(d.what, s, sizeof(d.what) - 3);
    l = strlen(s);
    strcpy(d.what + l, ": ");
    l += 2;
  }
  if (strerror_r(d.code, d.what + l, sizeof(d.what) - l)) {
    /* ignore error*/;
  }
  return uavpos_e_sys(&d, self);
}

struct uavpos_log_s {
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define uavpos_g	" %g "
# define uavpos_log_header_fmt                                          \
  "ts delay "                                                           \
  "fx fy fz "                                                           \
  "xd yd zd rolld pitchd yawd "                                         \
  "vxd vyd vzd wxd wyd wzd "                                            \
  "axd ayd azd "                                                        \
  "e_x e_y e_z e_vx e_vy e_vz e_rx e_ry e_rz e_wx e_wy e_wz"
# define uavpos_log_fmt                                                 \
  "%d.%09d " uavpos_g                                                   \
  uavpos_g uavpos_g uavpos_g                                            \
  uavpos_g uavpos_g uavpos_g uavpos_g uavpos_g uavpos_g                 \
  uavpos_g uavpos_g uavpos_g uavpos_g uavpos_g uavpos_g                 \
  uavpos_g uavpos_g uavpos_g                                            \
  uavpos_g uavpos_g uavpos_g uavpos_g uavpos_g uavpos_g                 \
  uavpos_g uavpos_g uavpos_g uavpos_g uavpos_g uavpos_g
};

#endif /* H_UAVPOS_CODELS */
