######################
## Version 0.1 #######
######################

import time
import numpy as np
## ###########################################################
## from mmr_normalization_new import mmr_normalization
import mmr_base_classes as base
from mmr_kernel import mmr_kernel 
## from mmr_kernel_eval import kernel_operator_valued
from mmr_solver import mmr_solver
from mmr_tools import mmr_perceptron_primal, mmr_perceptron_dual
import mmr_kernel_explicit
import mmr_kernel_mvm_y 
import mmr_kernel_mvm_x 
import mvm_test_orig
import mvm_solver
import mvm_prepare
from mmr_initial_params import cls_initial_params

## class definitions
## ##################################################
class cls_mvm(base.cls_data):

  def __init__(self,ninputview=1):

    base.cls_data.__init__(self,ninputview)

    self.ninputview=ninputview
    self.XKernel=[ None ]*ninputview
    self.YKernel=None
    self.KX=None
    self.KXCross=None
    self.KY=None
    self.d1x=None
    self.d2x=None
    self.d1x=None
    self.d2xcross=None
    self.d1ycross=None
    self.d2y=None
    ## mvm specific

    ## ============================
    ## self.xdata_rel=None
    ## self.rel_dim=None
    ## self.xdata_tra=None
    ## self.xdata_tes=None
    ## self.xranges_rel=None
    ## self.xranges_rel_test=None
    ## self.nrow=None
    ## self.ncol=None
    ## self.KXvar=None
    ## self.glm_model=None
    ## self.largest_class=None
    ## self.category=0     ## =0 rank cells =1 category cells =2 {-1,0,+1}^n
    ## self.categorymax=0
    ## self.ndata=0

    self.lxdata=[ base.cls_mvm_view() for i in range(ninputview)]

    self.penalty=base.cls_penalty() ## setting C,D penelty term paramters
    ## perceptron
    self.iperceptron=0
    self.perceptron=base.cls_perceptron_param()
    ## other classes
    self.dual=None
    self.predict=None
    self.evaluation=None

    self.xbias=0.2
    self.iybias=0

    self.kmode=0    ## =0 additive (feature concatenation)
                    ## =1 multiplicative (fetaure tensor product)

    self.ifixtrain=None
    self.ifixtest=None

    self.crossval_mode=0  ## =0 random cross folds =1 fixtraining
    self.itestmode=1      ## 0 active learning 1,2 random subsets
    self.ibootstrap=2    ## !!!!! =0 random =1 worst =2 best =3 worst+random  
    self.nrepeat=1
    self.nfold=2
    self.ieval_type=1  ## 
    self.testknn=10
    self.iperceptron=0
    self.ibinary=0        ## =1 Y0=[-1,+1], =0 [0,1,...,categorymax-1]

    ## test
    self.verbose=0

    ## row-column exchange
    self.rowcol=0       ## =0 row-col order =1 col-row order
    
## ---------------------------------------------------------
  ## def load_data(self,xdata,ncategory,nrow,ncol,Y0):
  def load_data(self,xdatalist,Y0):

    ntdata=len(xdatalist)
    iview=0
    for tdata in xdatalist:
      (xdata,ncategory,nrow,ncol)=tdata
      nldata=len(xdata)
      self.lxdata[iview].xdata_rel=[None]*nldata
      self.lxdata[iview].xdata_rel[0]=xdata[0]
      self.lxdata[iview].xdata_rel[1]=xdata[1]
      self.lxdata[iview].ncol=ncol
      self.lxdata[iview].nrow=nrow
      self.lxdata[iview].xdata_rel[2]=xdata[2]
      self.lxdata[iview].categorymax=ncategory
      self.lxdata[iview].ndata=len(xdata[0])
      iview+=1
    
    self.YKernel=mmr_kernel_mvm_y.cls_feature()
    self.XKernel[0]=mmr_kernel_mvm_x.cls_feature()
    cparams=cls_initial_params()
    
    self.YKernel=mmr_kernel_mvm_y.cls_feature(ifeature=0)
    self.YKernel.kernel_params.set(cparams.get_yparams('kernel',0))
    self.YKernel.crossval.set(cparams.get_yparams('cross',0))
    self.YKernel.norm.set(cparams.get_yparams('norm',0))

    ## setting input parameters
    iview=0   ## we have only one kernel
    for iview in range(self.ninputview):
      self.XKernel[iview]=mmr_kernel_mvm_x.cls_feature(ifeature=0)
      self.XKernel[iview].kernel_params.set(cparams.get_xparams('kernel',iview))
      self.XKernel[iview].crossval.set(cparams.get_xparams('cross',iview))
      self.XKernel[iview].norm.set(cparams.get_xparams('norm',iview))
    
    ## self.Y0=np.arange(self.categorymax)
    self.Y0=Y0
    
    self.penalty.set_crossval()
    
## ---------------------------------------------------------
  def set_validation(self):

    self.dkernels={}
    self.dkernels[self.YKernel.title]=self.YKernel
    nkernel=len(self.XKernel)
    for ikernel in range(nkernel):
      self.dkernels[self.XKernel[ikernel].title]=self.XKernel[ikernel]  

## ---------------------------------------------------------
  def split_train_test(self,xselector,ifold):

    if self.itestmode==0:   # active learning
      itest=np.where(xselector==ifold)[0]
      itrain=np.where(xselector!=ifold)[0]
    elif self.itestmode==1:   # random subset of rank data
      itest=np.where(xselector==ifold)[0]
      itrain=np.where(xselector!=ifold)[0]
    elif self.itestmode==2:   # random subset of rank data
      itest=np.where(xselector==0)[0]
      itrain=np.where(xselector!=0)[0]

    self.itrain=itrain
    self.itest=itest
    self.mtrain=len(itrain)
    self.mtest=len(itest)

## ---------------------------------------------------------
## ########################################################
  def mvm_datasplit(self):
    """
    splitting the full data into training and test
    xdata_rel -> xdata_train, xdata_test

    Input:
    xdatacls        data class
    itrain       indexs of training items in xdata_rel 
    itest        indexs of test items in xdata_rel 
    """
    xdata_rel=self.lxdata[0].xdata_rel
    nitem=len(xdata_rel)
    self.lxdata[0].xdata_tra=[ None for i in range(nitem) ]
    self.lxdata[0].xdata_tes=[ None for i in range(nitem) ]
    for i in range(nitem):
      self.lxdata[0].xdata_tra[i]=self.lxdata[0].xdata_rel[i][self.itrain]
      self.lxdata[0].xdata_tes[i]=self.lxdata[0].xdata_rel[i][self.itest]

    return
# ## ###########################################################
#   def mvm_ranges(self,nitem):
#     """
#     Creates the ranges of the data to each column index
# 
#     Input:
#     xdata       data, for example xdatacls.xdata_tra
#     nitem       number of rows, for example  xdatacls.nrow
#     Output:
#     xranges     the range matrix with two coluns (starting point, length)
#     """
# 
#     xranges=np.zeros((nitem+1,2),dtype=int)
# 
#     mdata=self[0].shape[0]
#     for idata in range(mdata):
#       iitem=self.xdata[0][idata]
#       xranges[iitem,1]+=1       ## counts the row items to one column
#                                 ## to get the length
# 
#     xranges[:,0]=np.cumsum(xranges[:,1])-xranges[:,1] ## compute starting points
# 
#     return(xranges)
## ###########################################################
  
  def compute_kernels(self):

    ## print('Kernel computation')
    if self.category==2:
      self.YKernel.compute_prekernel(self)
    self.YKernel.compute_kernel(self)
    nkernel=len(self.XKernel)
    for ikernel in range(nkernel):
      self.XKernel[ikernel].compute_kernel(self)

## ---------------------------------------------------------
  def mvm_train(self,params):

    ## print('Generate kernels')
    time0=time.time()
    self.compute_kernels()
    if self.verbose==1:
      print('Kernel computation:',time.time()-time0)
    ## print('Solve optimization problem')
    time0=time.time()
    self.KX=mmr_kernel(self,self.itrain,self.itrain,ioutput=0, \
                            itraintest=0, itensor=self.kmode)[0]
    self.KY=mmr_kernel(self,self.itrain,self.itrain,ioutput=1)[0]
    if self.verbose==1:
      print('Kernel merge computation:',time.time()-time0)
    ## self.solvertime=time.time()-time0

    ## t0=time.clock()
    cOptDual=base.cls_dual(None,None)
    self.dual=cOptDual
    time0=time.time()
    self.dual.alpha=mvm_solver.mvm_solver(self,params)
    self.solvertime=time.time()-time0
    ## print('Solver computation:',time.time()-time0)
    ## if self.verbose==1:
    ##   print('Solver computation:',time.time()-time0)
    ## print('Solver time: ',time.clock()-t0)

    return(cOptDual)

## ------------------------------------------------------------------
  def mvm_test(self,cOptDual,params,itraintest=1):
    #####################################################
    # select the potential test method
    #####################################################
    # wrapper around the possible test evaluations of different losses
    # inputs:
    #       xdatacls        data class
    #       xalpha          optimal dual variables of optimization
    #       params          global parameters
    # outputs:
    #       Zrow       row wise prediction for the test relations between
    #                         rows and cols      
    #####################################################
    cPredict=base.cls_predict()
    itest_method=0   # =0 orig 
    if itest_method==0:
      # matrix completition test
      cPredict.Zrow=mvm_test_orig.mvm_test_orig(self,self.dual.alpha,params)
    
    return(cPredict)
## ------------------------------------------------------------------
  def glm_norm_in(self,X):

    (m,n)=X.shape
    self.colmean=np.mean(X,axis=0)
    self.rowmean=np.mean(X,axis=1)
    self.totalmean=np.mean(self.colmean)
    
    Xres=X-np.outer(np.ones(m),self.colmean) \
          -np.outer(self.rowmean,np.ones(n))+self.totalmean

    return(Xres)
## ------------------------------------------------------------------
  def glm_norm_out(self,Xres):

    (m,n)=Xres.shape
    X=Xres+np.outer(np.ones(m),self.colmean) \
          +np.outer(self.rowmean,np.ones(n))-self.totalmean

    return(X)
## ------------------------------------------------------------------
  def copy(self,new_obj):

    nkernel=len(self.XKernel)
    ndata=len(self.xdata_rel)
    new_obj.xdata_rel=[None]*ndata
    for i in range(ndata):
      new_obj.xdata_rel[i]=self.xdata_rel[i][self.itrain]

    for ikernel in range(nkernel):
      new_obj.XKernel[ikernel]=self.XKernel[ikernel].copy() 
    new_obj.YKernel=self.YKernel.copy()
      
    new_obj.set_validation()

    new_obj.penalty=base.cls_penalty()
    new_obj.penalty.c=self.penalty.c
    new_obj.penalty.d=self.penalty.d
    new_obj.penalty.crossval=self.penalty.crossval

    new_obj.glm_model=self.glm_model

    new_obj.nrow=self.nrow
    new_obj.ncol=self.ncol
    new_obj.itestmode=self.itestmode
    new_obj.kmode=self.kmode
    new_obj.ieval_type=self.ieval_type
    new_obj.categorymax=self.categorymax
    new_obj.Y0=self.Y0
    new_obj.rowcol=self.rowcol
    
## #######################################################################
  

    
