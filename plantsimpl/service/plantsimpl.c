#include <math.h>

#include <simple_thread.h>
#include <service.h>
#include <threadsafe_types.h>
#include <msgpack_reader.h>
#include <gyro.h>
#include <pp_prio.h>
#include <pid.h>


/* thread that reads the thrust: */
tsfloat_t thrust;
MSGPACK_READER_BEGIN(thrust_reader)
   MSGPACK_READER_LOOP_BEGIN(thrust_reader)
      tsfloat_set(&thrust, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


SERVICE_MAIN_BEGIN("plantsimpl", PP_PRIO_1)
{

MSGPACK_READER_START(thrust_reader, "thrust", PP_PRIO_1, "sub");
/* initialize SCL: */
void *gyro_socket = scl_get_socket("gyro", "pub");
THROW_IF(gyro_socket == NULL, -EIO);
void *orientation_socket = scl_get_socket("orientation", "pub");
THROW_IF(orientation_socket == NULL, -EIO);
void *torques_socket = scl_get_socket("torques", "sub");
THROW_IF(torques_socket == NULL, -EIO);

float u1;
float u2;
float u3;
float u4;
float T=0.01;


float L=0.2025;
float g=9.81;
float m=1.26;

float Ix= 0.0098; 
float Iy= 0.0142; 
float Iz= 0.0082;

float a1= (Iy-Iz)/Ix;
float a2= L/Ix;
float a3= (Iz-Ix)/Iy;
float a4= L/Iy;
float a5= (Ix-Iy)/Iz;
float a6= 1/Iz;

float x1ant=0;
float x2ant=0;
float x3ant=0;
float x4ant=0;
float x5ant=0;
float x6ant=0;
float x7ant=0;
float x8ant=0;
float x9ant=0;
float x10ant=0;
float x11ant=0;
float x12ant=0;
float x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12;


MSGPACK_READER_SIMPLE_LOOP_BEGIN(torques)
{
   //set-up msgpack packer: 
   MSGPACK_PACKER_DECL_INFUNC();
 
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(torques)
   {  
   	  float torques[3];
      float thrust;
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         thrust = fmin(tsfloat_get(&thrust_max), tsfloat_get(&thrust));
         FOR_N(i, 3)
            torques[i + 1] = root.via.array.ptr[i].via.dec;
       
 				 u1=thrust;
 				 u2=torques[0]/L;
 				 u3=torques[1]/L;
 				 u4=torques[2]/L;

				x1 = x2ant*T + x1ant;
				x2 = T* ( ((x3-x3ant)/T)*((x5-x5ant)/T)*a1 + a2*u2 ) + x2ant;
				x3 = x4ant*T + x3ant;
				x4 = T*( ((x1-x1ant)/T)*((x5-x5ant)/T)*a3 + a4*u3 ) + x4ant;
				x5 = x6ant*T + x5ant;
				x6 = T*( ((x1-x1ant)/T)*((x3-x3ant)/T)*a5 + a6*u4 ) + x6ant;
				x7 = x8ant*T + x7ant;
				x8 = T*(-g + (cos(x1)*cos(x3)/m)*u1) + x8ant;
				x9 = x10ant*T + x9ant;
				x10 = T*((sin(x1)*sin(x5)+cos(x1)*cos(x5)*sin(x3))/m)*u1 + x10ant;
				x11 = x12ant*T + x11ant;
				x12 = T*( (cos(x1)*sin(x5)*sin(x3)-cos(x5)*sin(x1))/m )*u1 + x12ant;

        msgpack_sbuffer_clear(msgpack_buf);
        msgpack_pack_array(pk, 3);
        PACKF(x2);
        PACKF(x4);
        PACKF(x6);
        scl_copy_send_dynamic(gyro_socket, msgpack_buf->data, msgpack_buf->size);

        msgpack_sbuffer_clear(msgpack_buf);
        msgpack_pack_array(pk, 3);
        PACKF(x1);
        PACKF(x3);
        PACKF(x5);
        scl_copy_send_dynamic(orientation_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END
