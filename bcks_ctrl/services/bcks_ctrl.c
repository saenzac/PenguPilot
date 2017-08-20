#include <math.h>

#include <simple_thread.h>
#include <service.h>
#include <threadsafe_types.h>
#include <msgpack_reader.h>
#include <gyro.h>
#include <pp_prio.h>
#include <pid.h>
#include <mot_state.h>

#include "piid.h"

#define RATE_CTRL_PITCH 0
#define RATE_CTRL_ROLL  1
#define RATE_CTRL_YAW   2

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static tsfloat_t rs_ctrl_sp_p;
static tsfloat_t rs_ctrl_sp_r;
static tsfloat_t rs_ctrl_sp_y;
static int oe = 1; //by default is enable, the only way to disable is call the corresponding function in flight_logic -> ctrl_api

/* reads output enable: */
MSGPACK_READER_BEGIN(rs_ctrl_oe_reader)
   MSGPACK_READER_LOOP_BEGIN(rs_ctrl_oe_reader)
      oe = root.via.i64;
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

/* reads desired pitch position: */
MSGPACK_READER_BEGIN(rp_ctrl_sp_p_reader)
   MSGPACK_READER_LOOP_BEGIN(rp_ctrl_sp_p_reader)
      tsfloat_set(&rp_ctrl_sp_p, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

/* reads desired roll position: */
MSGPACK_READER_BEGIN(rp_ctrl_sp_r_reader)
   MSGPACK_READER_LOOP_BEGIN(rp_ctrl_sp_r_reader)
      tsfloat_set(&rp_ctrl_sp_r, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

/* reads desired yaw rate: */
MSGPACK_READER_BEGIN(rs_ctrl_sp_y_reader)
   MSGPACK_READER_LOOP_BEGIN(rs_ctrl_sp_y_reader)
      tsfloat_set(&rs_ctrl_sp_y, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

/* thread that reads the orientations: */
tsfloat_t orientation[3];
MSGPACK_READER_BEGIN(orientation_reader)
   MSGPACK_READER_LOOP_BEGIN(orientation_reader)
   if (root.type == MSGPACK_OBJECT_ARRAY)
   { 
   FOR_N(i, 3)
      tsfloat_set(&orientation[i], root.via.array.ptr[i].via.dec);
   }
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

/* thread that reads the integrator enable: */
/* MSGPACK_READER_BEGIN(int_en_reader)
   MSGPACK_READER_LOOP_BEGIN(int_en_reader)
   piid_int_enable(root.via.i64);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END */

/* thread that reads the motor state for integrator reset: */
/*MSGPACK_READER_BEGIN(mot_state_reader)
   MSGPACK_READER_LOOP_BEGIN(mot_state_reader)
   if (root.via.i64 != MOTORS_RUNNING)
   {
      pthread_mutex_lock(&mutex);
      piid_reset();
      pthread_mutex_unlock(&mutex);
   }
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END */

SERVICE_MAIN_BEGIN("bcks_ctrl", PP_PRIO_1)
{
   /* safe initial values: */
   tsfloat_init(&rp_ctrl_sp_p, 0.0);
   tsfloat_init(&rp_ctrl_sp_r, 0.0);
   tsfloat_init(&rs_ctrl_sp_y, 0.0);
   FOR_N(i, 3)
     tsfloat_init(&torques[i]);

   /* initialize msgpack: */
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
  
   /* initialize SCL: */
   void *gyro_socket = scl_get_socket("gyro", "sub");
   THROW_IF(gyro_socket == NULL, -EIO);
   void *orientation_socket = scl_get_socket("orientation", "sub");
   THROW_IF(orientation_socket == NULL, -EIO);
   void *torques_socket = scl_get_socket("torques_p", "push");
   THROW_IF(torques_socket == NULL, -EIO);

   MSGPACK_READER_START(rs_ctrl_oe_reader, "rs_ctrl_oe", PP_PRIO_1, "pull");
   MSGPACK_READER_START(rp_ctrl_sp_p_reader, "rp_ctrl_sp_p", PP_PRIO_1, "sub");
   MSGPACK_READER_START(rp_ctrl_sp_r_reader, "rp_ctrl_sp_r", PP_PRIO_1, "sub");
   MSGPACK_READER_START(rs_ctrl_sp_y_reader, "rs_ctrl_sp_y", PP_PRIO_1, "sub");
   MSGPACK_READER_START(orientation_reader, "orientation", PP_PRIO_1, "sub");
   //MSGPACK_READER_START(int_en_reader, "int_en", PP_PRIO_1, "sub");
   //MSGPACK_READER_START(mot_state_reader, "mot_state", PP_PRIO_1, "sub");
 
   //const float sample_dt = 0.005;
   //piid_init(sample_dt);
   //LOG(LL_INFO, "entering main loop");
   float _thrust_torques[4]={0.0, 0.0, 0.0, 0.0};
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(gyro)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         /* read synchronous gyro data: */
         // GYRO_X_AXIS, etc defined in gyro_cal/shared/gyro.h
         float gyro[3];
         gyro[GYRO_Y_AXIS] = root.via.array.ptr[GYRO_Y_DIR].via.dec; //dphi - pitch
         gyro[GYRO_X_AXIS] = root.via.array.ptr[GYRO_X_DIR].via.dec; //dtheta - roll
         gyro[GYRO_Z_AXIS] = root.via.array.ptr[GYRO_Z_DIR].via.dec; //dpsi - yaw

         /* read orientation data */
         float angles[3];
         angles[0]=tsfloat_get(&orientation[1]); //phi - pitch
         angles[1]=tsfloat_get(&orientation[2]); //theta -roll
         angles[2]=tsfloat_get(&orientation[0]); //psi - yaw

         /* retrieve asynchronous setpoints: */
         float setpoints[3] = {tsfloat_get(&rp_ctrl_sp_p),
                               tsfloat_get(&rp_ctrl_sp_r),
                               tsfloat_get(&rs_ctrl_sp_y)};
         
         /* run rate controller: */
         float torques[3];
         /* run backstepping controller: */
         pthread_mutex_lock(&mutex);
         //piid_run(torques, gyro, rates, sample_dt);
         



         torques[0]=u2*L;
         torques[1]=u3*L;
         torques[2]=u4;
         pthread_mutex_unlock(&mutex);

         /* send synchronous torques: */
         if (oe)
         {
            msgpack_sbuffer_clear(msgpack_buf);
            msgpack_pack_array(pk, 3);
            PACKFV(torques, 3);
            scl_copy_send_dynamic(torques_socket, msgpack_buf->data, msgpack_buf->size);
            
            msgpack_sbuffer_clear(msgpack_buf);
            msgpack_pack_array(pk, 3);
            FOR_N(i, 3)
               PACKF(rates[i] - gyro[i]);
            scl_copy_send_dynamic(err_socket, msgpack_buf->data, msgpack_buf->size);
 
         }
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

