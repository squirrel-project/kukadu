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
import numpy as np

## ####################
## import mvm_classes
from mvm_kernel_eval import mvm_kernel
from mvm_kernel_eval import mvm_kernel_sparse
from mmr_normalization_new import mmr_normalization
## ####################
def mvm_build_kernel(xdatacls,params):
########################################################
# computes the full input and output kernels
#########################################################
# inputs:
#     xdatacls         see in mvm_classes
#     params           see in mvm_setparams
# outputs:
#########################################################

# input componnets kernel
  ## nrow=xdatacls.ncol
  ## ncol=xdatacls.nrow

# input relation kernel
  ## if xdatacls.kmode==0:
  ##   xdatacls.KXfull=np.zeros((nrow,ncol))
  ## elif xdatacls.kmode==1:
  ##   xdatacls.KXfull=np.ones((nrow,ncol))

  isymmetric=1  
  xdatacls.KXfull=mvm_kernel_sparse(xdatacls,isymmetric,params,params.input[0])

  ## for iview in xdatacls.lview:
  ##   xrow=xdatacls.xdata_feature_train[iview]
  ##   (xrow,yy,opar)=mmr_normalization(params.input[1].ilocal,
  ##                                    params.input[1].iscale,
  ##                                    xrow,np.array([]),0)
  ##   KXfull=mvm_kernel(xrow,xrow,params,params.input[1])

  ##   KXfull+=params.input[0].bias
  ##   if xdatacls.kmode==0:
  ##     xdatacls.KXfull+=KXfull
  ##   elif xdatacls.kmode==1:
  ##     xdatacls.KXfull*=KXfull


## output kernel
  ymax=params.output.ymax
  ymin=params.output.ymin
  ystep=params.output.ystep
  yinterval=np.arange(ymin,ymax+ystep,ystep)
# output kernel
  xdatacls.KYfull=mvm_kernel(yinterval.T,np.array([]),params,params.output)  

  return
  
# ------------------------------------------------------------

