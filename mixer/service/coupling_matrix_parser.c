
#include <stdio.h>
#include <assert.h>

#include <logger.h>
#include <util.h>
#include <fs_parser.h>

#include "coupling_matrix_parser.h"


void coupling_matrix_print(int n_motors, float mixer[FORCES_AND_MOMENTS][MAX_MOTORS])
{
   LOG(LL_DEBUG, "normalized inverse coupling matrix:");
   FOR_N(i, n_motors)
   {
      LOG(LL_DEBUG, "%.1f\t%.1f\t%.1f\t%.1f", mixer[0][i], mixer[1][i], mixer[2][i], mixer[3][i]);
   }
  /*LOG(LL_DEBUG, "mixer[0:3][0]:%.1f\t%.1f\t%.1f\t%.1f ---- mixer[0:3][1]:%.1f\t%.1f\t%.1f\t%.1f --- mixer[0:3][2]:%.1f\t%.1f\t%.1f\t%.1f ---mixer[0:3][3]:%.1f\t%.1f\t%.1f\t%.1f", 
    mixer[0][0], mixer[1][0], mixer[2][0], mixer[3][0], 
    mixer[0][1], mixer[1][1], mixer[2][1], mixer[3][1],
    mixer[0][2], mixer[1][2], mixer[2][2], mixer[3][2],
    mixer[0][3], mixer[1][3], mixer[2][3], mixer[3][3]);*/
}


int coupling_matrix_parser_run(char *buffer, float mixer[FORCES_AND_MOMENTS][MAX_MOTORS])
{
   fs_parser_t parser;
   fs_parser_init(&parser);
   unsigned int n = fs_parser_count(&parser, buffer);
   int motors = n / FORCES_AND_MOMENTS;
   assert(motors < MAX_MOTORS);
   if ((n % FORCES_AND_MOMENTS) == 0)
   {
      for (int motor = 0; motor < motors; motor++)
      {
         for (int i = 0; i < FORCES_AND_MOMENTS; i++)
         {
             while (1)
             {
                fs_parser_run(&parser, *buffer++);
                if (fs_parser_ready(&parser))
                   break;
             }
             mixer[i][motor] = fs_parser_val(&parser);
         }
      }
   }
   return motors;
}

