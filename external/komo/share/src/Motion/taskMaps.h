#pragma once

#include "motion.h"

//===========================================================================

struct PositionTaskMap:public TaskMap {
  int i, j;               ///< which shapes does it refer to?
  ors::Vector ivec, jvec; ///< additional position or vector

  PositionTaskMap(ors::Shape *a=NULL, const ors::Vector& ivec=NoVector,
                  ors::Shape *b=NULL, const ors::Vector& jvec=NoVector):i(-1), j(-1){
    i(-1), j(-1)
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

