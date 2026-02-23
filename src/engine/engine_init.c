// Copyright 2025 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "engine/engine_init.h"

#include <string.h>

#include <simcore/SIM_model.h>
#include <simcore/SIM_tnum.h>
#include "engine/engine_util_blas.h"

#ifdef _MSC_VER
  #pragma warning (disable: 4305)  // disable MSVC warning: truncation from 'double' to 'float'
#endif



//------------------------------- solver parameters ------------------------------------------------

// set default solver parameters
void sim_defaultSolRefImp(sim_scalar_t* solref, sim_scalar_t* solimp) {
  if (solref) {
    solref[0] = 0.02;       // timeconst
    solref[1] = 1;          // dampratio
  }

  if (solimp) {
    solimp[0] = 0.9;        // dmin
    solimp[1] = 0.95;       // dmax
    solimp[2] = 0.001;      // width
    solimp[3] = 0.5;        // midpoint
    solimp[4] = 2;          // power
  }
}


//------------------------------- SIM_Option ---------------------------------------------------------

// set model options to default values
void sim_defaultOption(SIM_Option* opt) {
  // fill opt with zeros in case struct is padded
  memset(opt, 0, sizeof(SIM_Option));

  // timing parameters
  opt->timestep           = 0.002;

  // solver parameters
  opt->impratio           = 1;
  opt->tolerance          = 1e-8;
  opt->ls_tolerance       = 0.01;
  opt->noslip_tolerance   = 1e-6;
  opt->ccd_tolerance      = 1e-6;

  // sleep settings
  opt->sleep_tolerance    = 1e-4;

  // physical constants
  opt->gravity[0]         = 0;
  opt->gravity[1]         = 0;
  opt->gravity[2]         = -9.81;
  opt->wind[0]            = 0;
  opt->wind[1]            = 0;
  opt->wind[2]            = 0;
  opt->magnetic[0]        = 0;
  opt->magnetic[1]        = -0.5;
  opt->magnetic[2]        = 0;
  opt->density            = 0;
  opt->viscosity          = 0;

  // solver overrides
  opt->o_margin           = 0;
  sim_defaultSolRefImp(opt->o_solref, opt->o_solimp);
  opt->o_friction[0] = 1;
  opt->o_friction[1] = 1;
  opt->o_friction[2] = 0.005;
  opt->o_friction[3] = 0.0001;
  opt->o_friction[4] = 0.0001;

  // discrete options
  opt->integrator         = SIM_INT_EULER;
  opt->cone               = SIM_CONE_PYRAMIDAL;
  opt->jacobian           = SIM_JAC_AUTO;
  opt->solver             = SIM_SOL_NEWTON;
  opt->iterations         = 100;
  opt->ls_iterations      = 50;
  opt->noslip_iterations  = 0;
  opt->ccd_iterations     = 35;
  opt->disableflags       = 0;
  opt->enableflags        = 0;
  opt->disableactuator    = 0;

  // sdf collisions
  opt->sdf_initpoints     = 40;
  opt->sdf_iterations     = 10;
}


//------------------------------- SIM_Statistic ------------------------------------------------------

// set statistics to default values; compute later in compiler
void sim_defaultStatistic(SIM_Statistic* stat) {
  sim_math_zero_3(stat->center);
  stat->extent = 2;
  stat->meaninertia = 1;
  stat->meanmass = 1;
  stat->meansize = 0.2;
}


//------------------------------ SIM_LROpt -----------------------------------------------------------

// set default options for length range computation
void sim_defaultLROpt(SIM_LROpt* opt) {
  opt->mode           = SIM_LRMODE_MUSCLE;
  opt->useexisting    = 1;
  opt->uselimit       = 0;

  opt->accel          = 20;
  opt->maxforce       = 0;
  opt->timeconst      = 1;
  opt->timestep       = 0.01;
  opt->inttotal       = 10;
  opt->interval       = 2;
  opt->tolrange       = 0.05;
}
