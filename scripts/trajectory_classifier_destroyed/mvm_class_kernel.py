######################
## Version 0.1 #######
######################

import numpy as np
## ###########################################################
from mmr_normalization_new import mmr_normalization
from mvm_kernel_eval import mvm_kernel_sparse
## class definitions
## ##################################################
class cls_dataitem:

  ## ---------------------------------------
  def __init__(self,xdata1,xdata2,nrow,ncol):

    self.xdata_tra=xdata1
    self.xdata_tes=xdata2
    self.nrow=nrow
    self.ncol=ncol
    

## ##################################################
class cls_kernel:

  def __init__(self,ikernelstruct=0):
    """
    ikernelstruct       =0 monolith
                        =1 learner wise

    """
    self.ikernelstruct=ikernelstruct
    self.Kmonolith=None
    self.dkernel=None
    self.dsubsets=None
    self.index2subset=None

  ## -------------------------------------------------
  def create_monolith(self,xdatacls,isymmetric,params_spec,norm_spec):

    self.Kmonolith=mvm_kernel_sparse(xdatacls,isymmetric, \
                                     params_spec,norm_spec)

    return
  ## -------------------------------------------------
  def load_subsets(self,dsubsets):
    """
    dsubsets partitions the row space of xdata
    dsubset={1:set(<partition1>),2:set(<partition2>), ...}
    """
    
    self.dsubsets=dsubsets
    self.index2subset={}

    for ikey,ssubset in self.dsubsets.items():
      for irow in ssubset:
        if irow not in self.index2subset:
          self.index2subset[irow]=set([ikey])
        else:
          self.index2subset[irow].add([ikey])

    return
  ## -------------------------------------------------
  def create_diagblocks(self,xdatacls,isymmetric,params_spec,norm_spec):

    xdata_1=xdatacls.xdata_tra
    xdata_2=xdatacls.xdata_tes
    n=xdata_1.shape[1]

    xranges_1=xdatacls.xranges_rel
    xranges_2=xdatacls.xranges_rel_test
    
    nrow=xdatacls.nrow
    ncol=xdatacls.ncol

    cxdata=cls_dataitem()
    X1=None
    X2=None
   
    for ikey,ssubset in self.dsubsets.items():
      nlength1=0
      nlength2=0
      for irow in ssubset:
        nlength1+=xranges_1[irow,0]
        nlength2+=xranges_2[irow,0]
      X1=np.zeros((nlength1,n))
      X2=np.zeros((nlength2,n))
      i1=0
      i2=0
      for irow in ssubset:
        (istart,nlength)=xranges_1[irow,0]
        X1[i1,:]=xdata_1[istart,istart:istart+nlength]
        i1+=nlength
        (istart,nlength)=xranges_2[irow,0]
        X2[i2,:]=xdata_2[istart,istart:istart+nlength]
        i2+=nlength
      cxdata.xdata_tra=X1
      cxdata.xdata_tes=X2
      cxdata.nrow=len(ssubset)
      cxdata.ncol=ncol
      self.dkernel[ikey]=mvm_kernel_sparse(cxdata,isymmetric, \
                                     params_spec,norm_spec)

    return

  ## -------------------------------------------------
  
  
    
    
