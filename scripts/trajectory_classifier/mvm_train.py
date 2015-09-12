######################
## Version 0.1 #######
## /**********************************************************************
##   Copyright 2014, Sandor Szedmak  
##   email: sandor.szedmak@uibk.ac.at
##
##   This file is part of Maximum Margin Multi-valued Regression code(MMMVR).
##
##   MMMVR is free software: you can redistribute it and/or modify
##   it under the terms of the GNU Lesser General Public License as published by
##   the Free Software Foundation, either version 3 of the License, or
##   (at your option) any later version. 
##
##   MMMVR is distributed in the hope that it will be useful,
##   but WITHOUT ANY WARRANTY; without even the implied warranty of
##   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##   GNU Lesser General Public License for more details.
##
##   You should have received a copy of the GNU Lesser General Public License
##   along with MMMVR.  If not, see <http://www.gnu.org/licenses/>.
##
## ***********************************************************************/
######################
import time
## ####################
## import mvm_classes
from mvm_build_kernel import mvm_build_kernel
from mvm_solver import mvm_solver
## ####################
def mvm_train(xdatacls,params):
####################################################
# generate kernels and solves the optimization problem
####################################################
# inputs:
#     xdatacls            see in mvm_classes
#     params              optimization parameters
# output:
#     xalpha              dual variables of the optimization problem
####################################################
  ## print('Generate kernels')
  mvm_build_kernel(xdatacls,params)
  ## print('Solve optimization problem')

  ## t0=time.clock()
  xalpha=mvm_solver(xdatacls,params)
  ## print('Solver time: ',time.clock()-t0)
  
  return(xalpha)
  
  
  
  
