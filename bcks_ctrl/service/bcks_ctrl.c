#include <math.h>
#include <simple_thread.h>
#include <service.h>
#include <scl.h>
#include <threadsafe_types.h>
#include <msgpack_reader.h>
#include <gyro.h>
#include <pp_prio.h>
#include <float.h>
#include <time.h>
#include <mot_state.h>

#define RATE_CTRL_PITCH 0
#define RATE_CTRL_ROLL  1
#define RATE_CTRL_YAW   2

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

static tsfloat_t rp_ctrl_sp_p;
static tsfloat_t rp_ctrl_sp_r;
static tsfloat_t rs_ctrl_sp_y;
static int oe = 1;

static float L=0.2025;
static float g=9.81;
static float m=1.26;
static float Ix=0.035;// 0.0223; 
static float Iy=0.05;//0.0222; 
static float Iz=0.0442;//0.0442;

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

static tsint_t integrator_enable;
/* thread that reads the integrator enable: */
MSGPACK_READER_BEGIN(int_en_reader)
   tsint_init(&integrator_enable, 0);
   MSGPACK_READER_LOOP_BEGIN(int_en_reader)
   tsint_set(&integrator_enable, root.via.i64);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END 

static tsfloat_t z2sum;
static tsfloat_t z4sum;
/* thread that reads the motor state for integrator reset: */
MSGPACK_READER_BEGIN(mot_state_reader)
   tsfloat_init(&z2sum, 0.0f);
   tsfloat_init(&z4sum, 0.0f);
   MSGPACK_READER_LOOP_BEGIN(mot_state_reader)
   if (root.via.i64 != MOTORS_RUNNING)
   {
      pthread_mutex_lock(&mutex);
      tsfloat_set(&z2sum, 0.0);
      tsfloat_set(&z4sum, 0.0);
      pthread_mutex_unlock(&mutex);
   }
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

SERVICE_MAIN_BEGIN("bcks_ctrl", PP_PRIO_1)
{
   /* Safe initial values: */
   tsfloat_init(&rp_ctrl_sp_p, 0.0);
   tsfloat_init(&rp_ctrl_sp_r, 0.0);
   tsfloat_init(&rs_ctrl_sp_y, 0.0);
   FOR_N(i, 3)
     tsfloat_init(&orientation[i], 0.0);

   /* Initialize SCL */
   MSGPACK_PACKER_DECL_INFUNC();
   /* Start of connection with the sockets */
   void *gyro_socket = scl_get_socket("gyro", "sub");
   THROW_IF(gyro_socket == NULL, -EIO);
   void *torques_socket = scl_get_socket("torques_p", "push");
   THROW_IF(torques_socket == NULL, -EIO);

   MSGPACK_READER_START(rs_ctrl_oe_reader, "rs_ctrl_oe", PP_PRIO_1, "pull");
   MSGPACK_READER_START(rp_ctrl_sp_p_reader, "rp_ctrl_sp_p", PP_PRIO_1, "sub");
   MSGPACK_READER_START(rp_ctrl_sp_r_reader, "rp_ctrl_sp_r", PP_PRIO_1, "sub");
   MSGPACK_READER_START(rs_ctrl_sp_y_reader, "rs_ctrl_sp_y", PP_PRIO_1, "sub");
   MSGPACK_READER_START(orientation_reader, "orientation", PP_PRIO_1, "sub");
   MSGPACK_READER_START(int_en_reader, "int_en", PP_PRIO_1, "sub");
   MSGPACK_READER_START(mot_state_reader, "mot_state", PP_PRIO_1, "sub");
 
   /* Sampling time */
   const float dt_ms = 10; // in miliseconds
   const float dt = dt_ms / 1000.0; //in seconds
   /* Constant variables */
   float u2,u3,u4;
   float a1=(Iy-Iz)/Ix;
   float a2=(L/Ix);
   float a3=(Iz-Ix)/Iy;
   float a4=(L/Iy);
   float a5=(Ix-Iy)/Iz;
   float a6=(1/Iz);
   float inv_a2 = (1/a2);
   float inv_a4 = (1/a4);
   float k1=9;
   float k2=9;
   float k3=9;
   float k4=9;
   float k5=62;
   float rho1 = 0;
   float rho2 = 0;
   float gamma1 = 60;
   float gamma2 = 60;
   float max_sum_error = 7;
   float max_torque = 1; // in Newton-meter
   float z1,z2,z3,z4,z5,z6,x1d_ant,x3d_ant,dx1d,dx3d;
   z1=z2=z3=z4=z5=z6=x1d_ant=x3d_ant=dx1d=dx3d=0;

/* Control algorithm start */
MSGPACK_READER_SIMPLE_LOOP_BEGIN(gyro)
{
   if (root.type == MSGPACK_OBJECT_ARRAY)
   {
      /* Read synchronous gyro data: */
      float gyro[3];
      gyro[GYRO_Y_AXIS] = root.via.array.ptr[GYRO_Y_DIR].via.dec;
      gyro[GYRO_X_AXIS] = root.via.array.ptr[GYRO_X_DIR].via.dec;
      gyro[GYRO_Z_AXIS] = root.via.array.ptr[GYRO_Z_DIR].via.dec;
      /* Read orientation data */
      float angles[3];
      angles[0]=tsfloat_get(&orientation[1]);
      angles[1]=tsfloat_get(&orientation[2]);
      angles[2]=tsfloat_get(&orientation[0]);
      /* Retrieve asynchronous setpoints: */
      float setpoints[3] = {tsfloat_get(&rp_ctrl_sp_p),
                            tsfloat_get(&rp_ctrl_sp_r),
                            tsfloat_get(&rs_ctrl_sp_y)};
      /* Output torques */
      float torques[3];

      pthread_mutex_lock(&mutex);

         // Pitch controller
         z1 = setpoints[0] - angles[0];
         dx1d = (setpoints[0] - x1d_ant)/dt;
         z2 = dx1d + k1*z1 - gyro[0];
         u2 = 0;
         if (tsint_get(&integrator_enable) == 1)
         {
            float sum_error = tsfloat_get(&z2sum);
            sum_error += gamma1*z2 * dt;
            sum_error = sym_limit(sum_error, max_sum_error);
            u2 = inv_a2*sum_error;
            tsfloat_set(&z2sum,sum_error);
         }
         u2 += inv_a2*(k1*(z2-k1*z1)-gyro[1]*gyro[2]*a1+z1 + k2*z2);
         x1d_ant = setpoints[0];
         
         // Roll controller
         z3 = setpoints[1] - angles[1];
         dx3d = (setpoints[1] - x3d_ant)/dt;
         z4 = dx3d + k3*z3 - gyro[1];
         u3 = 0;
         if (tsint_get(&integrator_enable) == 1)
         {
            float sum_error = tsfloat_get(&z4sum);
            sum_error += gamma2*z4 * dt;
            sum_error = sym_limit(sum_error, max_sum_error);
            u3 = inv_a4*sum_error;
            tsfloat_set(&z4sum,sum_error);
         }
         u3 += inv_a2*(k3*(z4-k3*z3)-gyro[0]*gyro[2]*a3+z3 + k4*z4);
         x3d_ant = setpoints[1];

         // Yaw controller
         z5 = setpoints[2] - gyro[2];
         u4 = (1/a6)*(k5*z5-gyro[0]*gyro[1]*a5);

         // Torques
         torques[0]=sym_limit(u2*L,max_torque);
         torques[1]=sym_limit(u3*L,max_torque);
         torques[2]=sym_limit(u4,max_torque);

      pthread_mutex_unlock(&mutex);

      /* Send synchronous torques: */
      if (oe)
      {
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 3);
         PACKFV(torques, 3);
         scl_copy_send_dynamic(torques_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
}
/* Control algorithm end */
MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END


// u2 += inv_a2*(k1*(z2-k1*z1)-gyro[1]*gyro[2]*a1+z1 + k2*z2 + rho1*z2);
// u3 += inv_a2*(k3*(z4-k3*z3)-gyro[0]*gyro[2]*a3+z3 + k4*z4 + rho2*z4);


