######################
## Version 0.1 #######
######################
import sys
import numpy as np
## ###########################################################
import mmr_mmr_cls
import mmr_base_classes
from mmr_test import inverse_knn
from mmr_kernel import mmr_kernel
from mmr_eval import mmr_eval_binvector
from mmr_kernel_explicit import cls_feature
## ###########################################################

def mmr_validation(cMMR, params, evaluateSpecific):

  np.set_printoptions(precision=4)
  
  mtrain=cMMR.mtrain

  best_param=mmr_base_classes.cls_empty_class()
  best_param.c=0.0
  best_param.d=0.0
  best_param.par1=0.0
  best_param.par2=0.0
  xparam=mmr_base_classes.cls_empty_class()

  cMMRVal=mmr_mmr_cls.cls_mmr(cMMR.ninputview, evaluateSpecific)
  cMMRVal.XKernel=[None]*cMMR.ninputview
  cMMR.copy(cMMRVal,cMMR.itrain)
  ## params.validation.rkernel=cMMRVal.XKernel[0].title
  
  if params.validation.rkernel in cMMRVal.dkernels:
    rkernel=cMMRVal.dkernels[params.validation.rkernel]
  else:
    rkernel=cMMRVal.XKernel[0]

  kernel_type=rkernel.kernel_params.kernel_type
  kinput=rkernel.crossval

  if kernel_type==0:
    ip1min=0.0
    ip1max=0.0
    ip2min=0.0
    ip2max=0.0
    ip1step=1.0
    ip2step=1.0
  elif kernel_type in (1,11,12,2,51):
    ip1min=kinput.par1min
    ip1max=kinput.par1max
    ip2min=kinput.par2min
    ip2max=kinput.par2max
    ip1step=kinput.par1step
    ip2step=kinput.par2step
  elif kernel_type in (3,31,32,41,53):
    if kinput.nrange>1:
      if kinput.par1max>kinput.par1min:
        dpar= np.power(kinput.par1max/kinput.par1min,1/(kinput.nrange-1))
        ip1max=kinput.nrange
      else:
        dpar=1.0
        ip1max=1.0
    else:
      ip1max=1.0
      dpar=1.0
    ip1min=1.0
    ip2min=kinput.par2min
    ip2max=kinput.par2max
    ip1step=1.0
    ip2step=kinput.par2step
  else: 
    ip1min=1.0
    ip1max=1.0
    ip2min=1.0
    ip2max=1.0
    ip1step=1.0
    ip2step=1.0

## Validation 

## number of validation folds
  if params.validation.vnfold<2:
    params.validation.vnfold=2
  vnfold=params.validation.vnfold ## number of validation folds
  vxsel=np.floor(np.random.random(cMMRVal.mdata)*vnfold)
  vxsel=vxsel-(vxsel==vnfold)
## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if params.itest==1:
    vxsel=np.zeros(cMMRVal.mdata)
    for i in range(0,cMMRVal.mdata,2):
      vxsel[i]=1
## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   
  vpredtr=np.zeros(vnfold)     ## valid
  vpred=np.zeros(vnfold)     ## train
  print('C,D,par1,par2,traning accuracy,validation test accuracy')    

  ## scanning the parameter space
  if cMMR.ieval_type in (0,10):
    xxmax=-np.inf
  else:
    xxmax=np.inf
    
  penalty=cMMRVal.penalty.crossval
  crange=np.arange(penalty.par1min,penalty.par1max+penalty.par1step/2, \
                   penalty.par1step)
  drange=np.arange(penalty.par2min,penalty.par2max+penalty.par2step/2, \
                   penalty.par2step)

  p1range=np.arange(ip1min,ip1max+ip1step/2,ip1step)
  p2range=np.arange(ip2min,ip2max+ip2step/2,ip2step)
  
  for iC in crange:
    for iD in drange:
      for ip1 in p1range:
        for ip2 in p2range:
          if kernel_type in (3,31,32,41,53): 
            dpar1=kinput.par1min*dpar**(ip1-1)
            dpar2=ip2
          else:
            dpar1=ip1
            dpar2=ip2
             
          cMMRVal.penalty.c=iC;
          cMMRVal.penalty.d=iD;
          rkernel.kernel_params.ipar1=dpar1;
          rkernel.kernel_params.ipar2=dpar2;

          for vifold in range(vnfold):
            
            cMMRVal.split_train_test(vxsel,vifold)
            cMMRVal.compute_kernels()
            cMMRVal.Y0=cMMRVal.YKernel.get_Y0(cMMRVal.itrain)

            cOptDual=cMMRVal.mmr_train(params)
## validation training         
            cPredictValTrain=cMMRVal.mmr_test(cOptDual,params,itraindata=0)
## counts the proportion the ones predicted correctly    
## ##############################################
            if cMMRVal.itestmode==2:
              ypred=inverse_knn(cMMRVal.YKernel.get_Y0(cMMRVal.itrain), \
                                cPredictValTrain)
            else:
              ypred=cPredictValTrain.zPred
            cEvaluationValTrain= \
                mmr_eval_binvector(cMMRVal.YKernel.get_train(cMMRVal.itrain), \
                                   ypred)    
            vpredtr[vifold]=cEvaluationValTrain.f1
              
## ##############################################
## validation test
            cPredictValTest=cMMRVal.mmr_test(cOptDual,params,itraindata=1)
              
## counts the proportion the ones predicted correctly    
## ##############################################
            if cMMRVal.itestmode==2:
              ypred=inverse_knn(cMMRVal.YKernel.get_Y0(cMMRVal.itrain), \
                                cPredictValTest)
            else:
              ypred=cPredictValTest.zPred
            cEvaluationValTest= \
                mmr_eval_binvector(cMMRVal.YKernel.get_test(cMMRVal.itest), \
                                   ypred)

            ## if params.hier_multic==1:
            vpred[vifold]=cEvaluationValTest.f1
            ## else:
              ## vpred[vifold]=cEvaluationValTest.accuracy              
          
## ##############################################
          np.set_printoptions(precision=4)
          print('%9.5g'%iC,'%9.5g'%iD,'%9.5g'%dpar1,'%9.5g'%dpar2, \
                '%9.5g'%(np.mean(vpredtr)),'%9.5g'%(np.mean(vpred)))
##          print(array((iC,iD,dpar1,dpar2,mean(vpredtr),mean(vpred))))
##          print(iC,iD,dpar1,dpar2,mean(vpredtr),mean(vpred))
## searching for the best configuration in validation
          mvpred=np.mean(vpred)

          if cMMR.ieval_type in (0,10):
            if mvpred>xxmax:
              xxmax=mvpred
              xparam.c=iC
              xparam.d=iD
              xparam.par1=dpar1
              xparam.par2=dpar2
              print('The best:',xxmax)
          else:
            if mvpred<xxmax:
              xxmax=mvpred
              xparam.c=iC
              xparam.d=iD
              xparam.par1=dpar1
              xparam.par2=dpar2
              print('The best:',xxmax)
              
          sys.stdout.flush()  
  validationScore = xxmax
  best_param=xparam

  return( {'vs' : validationScore, 'bp' : best_param})

