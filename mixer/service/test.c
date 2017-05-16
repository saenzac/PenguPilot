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

#include "inv_coupling.h"
#include "coupling_matrix_parser.h"

int main()
{
   float mixer[FORCES_AND_MOMENTS][MAX_MOTORS];

   opcd_param_t params[] =
   {
      {"quad_matrix", &matrix_def},
      {"imtx1", &imtx1},
      {"imtx2", &imtx2},
      {"imtx3", &imtx3},
      {"f_c", &f_c},
      OPCD_PARAMS_END
   };
   opcd_params_apply(".", params);
   int n_motors = coupling_matrix_parser_run(matrix_def, mixer);
   coupling_matrix_print(n_motors, mixer);
   FOR_N(i, n_motors)
   {
      mixer[0][i] *= tsfloat_get(&imtx1); /* gas */
      mixer[1][i] *= tsfloat_get(&imtx2); /* pitch */
      mixer[2][i] *= tsfloat_get(&imtx2); /* roll */
      mixer[3][i] *= tsfloat_get(&imtx3); /* yaw */
   }
   inv_coupling_init(n_motors, mixer);

}