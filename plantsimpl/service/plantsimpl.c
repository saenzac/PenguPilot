
#include <stdio.h>
#include <opcd_interface.h>
#include <service.h>
#include <scl.h>
#include <msgpack_reader.h>
#include <util.h>
#include <threadsafe_types.h>
#include <pp_prio.h>
#include <float.h>
#include <math.h>
#include <mot_state.h>
#include <time.h>
#include <stdlib.h>

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//Constant values
static float L=0.2025;
static float g=9.81;
static float m=1.26;
static float Ix= 0.0098; 
static float Iy= 0.0142; 
static float Iz= 0.0082;

/*static tsint_t motstate;

MSGPACK_READER_BEGIN(mot_state_reader)
   MSGPACK_READER_LOOP_BEGIN(mot_state_reader)
   tsint_set(&motstate, root.via.i64); //!= MOTORS_RUNNING)
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END*/


/* thread that reads the torques: */
tsfloat_t torques[3];
MSGPACK_READER_BEGIN(torques_reader)
   MSGPACK_READER_LOOP_BEGIN(torques_reader)
   if (root.type == MSGPACK_OBJECT_ARRAY)
   {
    
   FOR_N(i, 3)
      tsfloat_set(&torques[i], root.via.array.ptr[i].via.dec);

   LOG(LL_INFO, "torques[0]: %f, [1]: %f, [2]: %f",  tsfloat_get(&torques[0]),tsfloat_get(&torques[1]),tsfloat_get(&torques[2])); 
   }
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


/* thread that reads the thrust: */
tsfloat_t thrust;
MSGPACK_READER_BEGIN(thrust_reader)
   MSGPACK_READER_LOOP_BEGIN(thrust_reader)
      tsfloat_set(&thrust, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

/* thread that reads the thrust maximum: */
tsfloat_t thrust_max;
MSGPACK_READER_BEGIN(thrust_max_reader)
   MSGPACK_READER_LOOP_BEGIN(thrust_max_reader)
      tsfloat_set(&thrust_max, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

SERVICE_MAIN_BEGIN("plantsimpl", PP_PRIO_1)
{
  tsfloat_init(&thrust, 0.0f); // 0N force; safe to start with
  tsfloat_init(&thrust_max, 12); // allow maximum thrust, can be limited later on

  /* initialize SCL: */
  void *rc_socket = scl_get_socket("rc", "sub");
  THROW_IF(rc_socket == NULL, -EIO);
  void *gyro_socket = scl_get_socket("gyro", "pub");
  THROW_IF(gyro_socket == NULL, -EIO);
  void *orientation_socket = scl_get_socket("orientation", "pub");
  THROW_IF(orientation_socket == NULL, -EIO);

  MSGPACK_READER_START(torques_reader, "torques", PP_PRIO_1, "sub");
  MSGPACK_READER_START(thrust_reader, "thrust", PP_PRIO_1, "sub");
  MSGPACK_READER_START(thrust_max_reader, "thrust_max", PP_PRIO_1, "sub");
  //MSGPACK_READER_START(mot_state_reader, "mot_state", PP_PRIO_1, "sub");

  const float Tms = 10; //T in ms
  const float T = Tms / 1000.0; //T in seconds
  srand(time(NULL));  

  //float u1;
  float u2;
  float u3;
  float u4;

  float a1=(Iy-Iz)/Ix;
  float a2=(L/Ix);
  float a3=(Iz-Ix)/Iy;
  float a4=(L/Iy);
  float a5=(Ix-Iy)/Iz;
  float a6=(1/Iz);

  float x1ant,x2ant,x3ant,x4ant,x5ant,x6ant,x7ant,x8ant,x9ant,x10ant,x11ant,x12ant;
  //float u1ant,u2ant,u3ant,u4ant;
  //float x1ant2, x3ant2,x5ant2; x1ant2=x3ant2=x5ant2=0;
  //u1ant=u2ant=u3ant=u4ant=0;
  float x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12;
  
  //initial values
  x1ant=x2ant=x3ant=x4ant=x5ant=x6ant=x7ant=x8ant=x9ant=x10ant=x11ant=x12ant=0; 
  x1=x2=x3=x4=x5=x6=x7=x8=x9=x10=x11=x12=0;

  //set-up msgpack packer: 
  MSGPACK_PACKER_DECL_INFUNC();

  MSGPACK_READER_SIMPLE_LOOP_BEGIN(rc)
  { 
    /*if (tsint_get(&motstate) == MOTORS_STOPPED){
      x1ant=0;
      x2ant=0;
      x3ant=0;
      x4ant=0;
      x5ant=0;
      x6ant=0;
      x7ant=0;
      x8ant=0;
      x9ant=0;
      x10ant=0;
      x11ant=0;
      x12ant=0;
    }*/

    float _thrust_torques[4]={0.0, 0.0, 0.0, 0.0};
    if (root.type == MSGPACK_OBJECT_ARRAY)
    {
      _thrust_torques[0] = fmin(tsfloat_get(&thrust_max), tsfloat_get(&thrust));
      FOR_N(i, 3)
        _thrust_torques[i+1] = tsfloat_get(&torques[i]);

      pthread_mutex_lock(&mutex);
      //u1=_thrust_torques[0];
      u2=_thrust_torques[1]/L; //with original controller we must divide by L
      u3=_thrust_torques[2]/L; // "            "             "             "
      u4=_thrust_torques[3]; 
      //LOG(LL_INFO, "u1: %f, u2: %f, u3: %f, u4: %f", u1,u2,u3,u4);

      //u1ant=u1;
      //u2ant=u2;
      //u3ant=u3;
      //u4ant=u4;
      
      // Este tambien funciona , requiere anteriores de anteriores: k-2
      /*
      x2= (x1ant - x1ant2)/T;
      x4= (x3ant - x3ant2)/T;
      x6= (x5ant - x5ant2)/T;  

      x1 = (T*T) * ( ((x3ant-x3ant2)/T)*((x5ant-x5ant2)/T)*a1 + a2*u2 ) + 2*x1ant - x1ant2;
      x3 = (T*T) * ( ((x1ant-x1ant2)/T)*((x5ant-x5ant2)/T)*a3 + a4*u3 ) + 2*x3ant - x3ant2;
      x5 = (T*T) * ( ((x1ant-x1ant2)/T)*((x3ant-x3ant2)/T)*a5 + a6*u4 ) + 2*x5ant - x5ant2;
      
      x1ant2 = x1ant;
      x3ant2 = x3ant;
      x5ant2 = x5ant;
      // fin ,
      */
      // --- este es el mejor funciona en velocidad si se parece con T entre mseg ( 0.3 y 0.5)
      x1 = x2ant*T + x1ant; //phi - pitch
      x3 = x4ant*T + x3ant; //theta - roll
      x5 = x6ant*T + x5ant; //psi - yaw

      x2 = T*( x4ant*x6ant*a1 + a2*u2 ) + x2ant; // dphi
      x4 = T*( x2ant*x6ant*a3 + a4*u3 ) + x4ant; // dtheta
      x6 = T*( x2ant*x4ant*a5 + a6*u4 ) + x6ant; // dpsi
      // -- end

      x1ant = x1;
      x2ant = x2;

      x3ant = x3;
      x4ant = x4;

      x5ant = x5;
      x6ant = x6;

      msgpack_sbuffer_clear(msgpack_buf);
      msgpack_pack_array(pk, 3);
      PACKF(x4); // dtheta
      //PACKF(x4 + ((rand() % 100)/1000.0)); // dtheta
      PACKF(x2); // dphi
      PACKF(x6); // dpsi
      scl_copy_send_dynamic(gyro_socket, msgpack_buf->data, msgpack_buf->size);

      msgpack_sbuffer_clear(msgpack_buf);
      msgpack_pack_array(pk, 3);
      PACKF(x5); // psi
      PACKF(x1); // phi
      PACKF(x3); // theta
      scl_copy_send_dynamic(orientation_socket, msgpack_buf->data, msgpack_buf->size);

      msleep(Tms);
      pthread_mutex_unlock(&mutex);

    }
  }
  MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END
