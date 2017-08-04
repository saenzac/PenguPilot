#include <msgpack.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <logger.h>
#include <msgpack_reader.h>
#include <service.h>
#include <pp_prio.h>
#include <stdio.h>
#include <stdlib.h>

static int actt=0;
static int goout=0;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

/* reads output enable: */
MSGPACK_READER_BEGIN(simu_oe_reader)
   MSGPACK_READER_LOOP_BEGIN(simu_oe_reader)
      actt = root.via.i64;
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

SERVICE_MAIN_BEGIN("simulation", PP_PRIO_1)
{
   /* set-up msgpack packer: */
  MSGPACK_PACKER_DECL_INFUNC();
 
   /* initialize SCL: */
  void *rc_socket = scl_get_socket("rc", "pub");
  THROW_IF(rc_socket == NULL, -EIO);
 
  MSGPACK_READER_START(simu_oe_reader, "simu_oe", PP_PRIO_1, "pull");

  FILE* fp = fopen("/Volumes/data/GoogleDrive/Data/Maestria/Investigation/tesis/PenguPilot/test.txt", "r");
  if (fp == NULL)
    exit(EXIT_FAILURE);

  char *line=NULL; size_t len =0; 
  float rc16[6];
  int rc1;

  while(1)
  {
    pthread_mutex_lock(&mutex);
    //LOG(LL_INFO, "goout=%d, c=%d",goout,c);  
    if (actt)
    {
      int read = getline(&line, &len, fp);
      //LOG(LL_INFO, "read=%d",read);
      if (read == -1)
      {
        //LOG(LL_INFO, "read <= -1");
        break;
      }
      sscanf(line,"%d,%f,%f,%f,%f,%f,%f", &rc1, &rc16[0], &rc16[1], &rc16[2], &rc16[3], &rc16[4],&rc16[5]);
      /* send the channels: */
      msgpack_sbuffer_clear(msgpack_buf);
      msgpack_pack_array(pk, 6 + 1);
      //package only the valid field.
      PACKI(rc1);    /* index 0: valid */
      //package in a vector all the remaining 6 fields of 'rc'
      PACKFV(rc16, 6); /* index 1, .. : channels */
      //Send the valid field plus the remaining fields,
      //then in total we send a vector of 7 elements (1+6).
      scl_copy_send_dynamic  (rc_socket, msgpack_buf->data, msgpack_buf->size);

      msleep(10);
      //printf("rc[0]:%d rc[1]:%.13f rc[2]:%.13f rc[3]:%.13f rc[4]:%.13f rc[5]:%.13f rc[6]:%.13f \n",rc1,rc16[0],rc16[1],rc16[2],rc16[3],rc16[4],rc16[5]);
    }
    else
    {
      //LOG(LL_INFO, "simulation deactivated");
      msleep(1000);
    }

    pthread_mutex_unlock(&mutex);
  } 
  //LOG(LL_INFO, "out of while , closing FILE*");

  fclose(fp);

}
SERVICE_MAIN_END

