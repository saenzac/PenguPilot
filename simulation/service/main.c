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

static int act = 0;

/*MSGPACK_READER_BEGIN(startsimu_reader)
   MSGPACK_READER_LOOP_BEGIN(startsimu_reader)
      act = root.via.i64;
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END
*/

SERVICE_MAIN_BEGIN("simulation", PP_PRIO_1)
{
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   /* initialize SCL: */
   void *rc_socket = scl_get_socket("rc", "pub");
   THROW_IF(rc_socket == NULL, -EIO);
 
   //MSGPACK_READER_START(startsimu_reader, "startsimu", PP_PRIO_3, "sub");

   FILE* fp = fopen("/home/johnnysaenz/workspace/PenguPilot/test.txt", "r");
   if (fp == NULL)
    exit(EXIT_FAILURE);

   char *line=NULL; size_t len =0; ssize_t read;
   float rc16[6];
   int rc1;

    while ((read = getline(&line, &len, fp)) != -1) {
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


   fclose(fp);
}
SERVICE_MAIN_END

