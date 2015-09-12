######################
## Version 0.1 #######
######################
import time
import numpy as np
## ##########################################
## from mmr_normalization_new import mmr_normalization
## from mmr_kernel_eval import kernel_eval_kernel
## import mmr_subspace
## import objedge_prekernel
## #####################
def mmr_kernel(cMMR,itrain,itest,ioutput=0,itraintest=0,itraindata=0,itensor=0):

  mtra=len(itrain)
  mtes=len(itest)

  d1=None
  d2=None
  if ioutput==0:
    ## input training kernel
    nkernel=len(cMMR.XKernel)
    KK=cMMR.XKernel[0].get_kernel(itrain,itest,itraintest=itraintest, \
                                  itraindata=itraindata)[0]
    for ikernel in range(1,nkernel):
      rkernel=cMMR.XKernel[ikernel]
      K=rkernel.get_kernel(itrain,itest,itraintest=itraintest, \
                           itraindata=itraindata)[0]
      if itensor==0:
        KK+=K
      else:
        KK*=K

    (mtra,mtes)=KK.shape
    return(KK+cMMR.xbias*np.ones((mtra,mtes)),d1,d2)
  elif ioutput==1:
  ## output kernel
    rkernel=cMMR.YKernel
    (KK,d1,d2)=rkernel.get_kernel(itrain,itest,ioutput=ioutput, \
                                  itraintest=itraintest,itraindata=itraindata)
    
  return(KK,d1,d2)
