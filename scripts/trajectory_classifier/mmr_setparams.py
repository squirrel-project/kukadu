######################
## Version 0.1 #######
######################
## import os, string, re, math, pickle, random
## import math, time
import numpy as np
## ################################
## import pylab as lab
from mmr_base_classes import cls_empty_class
## #####################
class cls_params:

  def __init__(self):

    self.ninputkernel=3
    self.ninputview=1
    self.verbose=1      ## =0 no messsages =1 debug messages
    self.nrepeat=1   ## number of the repetation of the full experiment
    self.nfold=5     ## number of folds >1
    self.itest=0
    self.itestmode=0  ## precision,recall of positive elements
    self.crossval_mode=1  ## =0 random =1 fix training and test
    self.output_subspace=1  ## =0 simple kernel =1 subspace
    self.testknn=10    ## test neighbors
    self.iperceptron=0  ## batch =1 primal =2 dual

##  params=cls_empty_class()
    
    self.general=cls_empty_class()
    self.output=cls_empty_class()
    self.validation=cls_empty_class()
    self.solver=cls_empty_class()
    self.output1=cls_empty_class()
    self.output2=cls_empty_class()
    self.inputraw=cls_empty_class()

  def setvalidation(self):
## parameters to validation
    self.validation.vnfold=2    ## number folds(>1) in validation
    self.validation.ivalid=1    ## =0 no validation =1 validation
    self.validation.ikernel=0  ## index of kernel to be validated
    self.validation.rkernel='mvm_x'  ## reference kernel to be validated
    self.validation.icommon=1   ## validation on C,D =1 otherwise =0
    self.validation.report=0

  def setsolver(self):
## solver parameters
    self.solver.niter=1000   ## maximum iteration
    self.solver.normx1=1  ## normalization within the kernel by this power
    self.solver.normy1=1  ## normalization within the kernel by this power
    self.solver.normx2=1  ## normalization of duals bound
    self.solver.normy2=1  ## normalization of duals bound
    self.solver.ilabel=0  ## 1 explicit labels, 0 implicit labels 
    self.solver.ibias=0   ## 0 no bias considered, 1 bias is computed by solver 
    self.solver.ibias_estim=0  ## estimated bias =0 no =1 computed 
    self.solver.i_l2_l1=1   ## =1 l2 norm =0 l1 norm regularization   
    self.solver.report=0

## parameters to cross validation  
  
  ## general  params e.g. C,D
  ## output params
  ## input params
    
  ## ################################################################
  ## genaral params
  ## general common parameters
  def setgeneral(self):  
  ## range for C the trade off parameter between regularization and error
  ## upper bound on the dual variables
  ## C=0.01 has been tried  
    self.general.cmin=1.0
    self.general.cmax=1.0
    self.general.cstep=0.2
  ## current value
    self.general.c=self.general.cmin
  ## range for D the trade off parameter between regularization and error
  ## lower bound on the dual variables, generally it assumed to be zero
    self.general.dmin=0.0
    self.general.dmax=0.0
    self.general.dstep=0.1
  ## current value
    self.general.d=self.general.dmin
  ##

    self.general.kmode=0  ## =0 additive =1 multiplicative


## ################################################################
  ## Output kernel
  def setoutput(self):  

## ilabmode:  labeling mode of multiclass
##               =0 indicators
##               =1 class mean
##               =2 class median
##               =3 tetrahedron
##               =31 weighted simplex

    self.output.ilabmode=31
    self.output.ndepth=3

    self.output.nclass=2
  ## new style of centralization and normaliation
    self.output.ilocal=2 ## centralization type see mmr_normalization_new.py
    self.output.iscale=0 ## normalization type see mmr_normalization_new.py
    self.output.offset=0.01

    self.output.nboro=10
    self.output.neig=5    ## number of greatest eigenvalues taken
    self.output.ioperator_valued=0  ## =0 scalar kernel, =1 operator valued
    self.output.col_dim=5        ## dimension of column representation
    
    self.output.kernel_type=0  ## 0 linear, 1 polynomial, 3 Gaussian

    if self.output.kernel_type==32:
      self.output.iyfeature=1     ## =0 explicit y feature =1 kernel
      self.output.itestmode=0    ## =0 dot(Y0,Zw), =1 max(Zw,axis=1) items
      self.output.nmax=4         ## number of largest values in Zw considered
      self.output.preimage=1      ## =0 testing agains training
                                ## =1 approxiamte dynamic programming
    elif self.output.kernel_type in (50,51,53,61):
      self.output.iyfeature=1     ## =0 explicit y feature =1 kernel
      self.output.itestmode=2    ## =0 dot(Y0,Zw), =1 max(Zw,axis=1) items
      self.output.preimage=0
    else:
      if self.output_subspace==0:
        self.output.iyfeature=0     ## =0 explicit y feature =1 kernel
      else:
        self.output.iyfeature=1     ## =0 explicit y feature =1 kernel
      self.output.itestmode=2    ## =0 dot(Y0,Zw), =1 max(Zw,axis=1) items
      self.output.nmax=4         ## number of largest values in Zw considered
      self.output.preimage=0      ## =0 testing agains training
                                ## =1 approxiamte dynamic programming
  
    
  ## possible parameter ranges scanned in the validation
    if self.output.kernel_type==1:
  ## polynomial kernel par1=degree and par2=constant
      self.output.par1min=1.0
      self.output.par1max=5.0
      self.output.par2min=0.00
      self.output.par2max=0.1
      self.output.par1step=0.5 
      self.output.par2step=0.02 
    elif self.output.kernel_type==2:
  ## sigmoid kernel par1=factor and par2=constant
      self.output.par1min=0.01 
      self.output.par1max=0.1 
      self.output.par2min=0 
      self.output.par2max=0.1 
      self.output.par1step=0.01 
      self.output.par2step=0.01 
    elif self.output.kernel_type==3:
  ## Gaussian kernel par1=variance(width) 
      self.output.par1min=0.04
      self.output.par1max=0.04 ## ~10 
      self.output.par2min=0 
      self.output.par2max=0
      self.output.par1step=1 
      self.output.par2step=1 
      self.output.nrange=7 
    elif self.output.kernel_type==31:
  ## PolyGaussian kernel par1=variance(width) 
      self.output.par1min=0.01 
      self.output.par1max=10 ## ~10 
      self.output.par2min=1.5 
      self.output.par2max=2.5
      self.output.par1step=1 
      self.output.par2step=0.5 
      self.output.nrange=20 
      self.output.dpower=1.0
      self.output.spower=1.0
    elif self.output.kernel_type==32:
  ## Gaussian with full covariance 
      self.output.par1min=np.sqrt(0.5)*0.25
      self.output.par1max=np.sqrt(0.5)*0.25 ## ~10 
      self.output.par2min=1.5 
      self.output.par2max=2.5
      self.output.par1step=1 
      self.output.par2step=0.5 
      self.output.nrange=20 
    elif self.output.kernel_type==41:
  ## PolyLaplace kernel par1=variance(width) 
      self.output.par1min=0.3 
      self.output.par1max=0.8 ## ~10 
      self.output.par2min=0 
      self.output.par2max=0
      self.output.par1step=1 
      self.output.par2step=1 
      self.output.nrange=5
    elif self.output.kernel_type==50:   ## cross table kernel
      self.output.par1min=0 
      self.output.par1max=0  
      self.output.par2min=0 
      self.output.par2max=0
      self.output.par1step=1 
      self.output.par2step=1       
    elif self.output.kernel_type==51:
  ## polynomial kernel par1=degree and par2=constant
      self.output.par1min=1
      self.output.par1max=1 
      self.output.par2min=0.1 
      self.output.par2max=0.1 
      self.output.par1step=1 
      self.output.par2step=1 
    elif self.output.kernel_type==53:
  ## Gaussian kernel par1=variance(width) 
      self.output.par1min=0.3
      self.output.par1max=0.3 ## ~10 
      self.output.par2min=0 
      self.output.par2max=0
      self.output.par1step=1 
      self.output.par2step=1 
      self.output.nrange=7 
    elif self.output.kernel_type==61:
  ## polynomial kernel par1=degree and par2=constant
      self.output.par1min=1
      self.output.par1max=1 
      self.output.par2min=0.1 
      self.output.par2max=0.1 
      self.output.par1step=1 
      self.output.par2step=1 
    else:
      self.output.par1min=0 
      self.output.par1max=0  
      self.output.par2min=0 
      self.output.par2max=0
      self.output.par1step=1 
      self.output.par2step=1 


    self.output.dpower=1.0

    self.output.ipar1=self.output.par1min
    self.output.ipar2=self.output.par2min
    


  ## ################################################################
  ## input params

  def getinput(self,ikernel):

    kparams=self.input[0]
    ## if ikernel<self.ninputkernel:
    ##   kparams=self.input[ikernel]
    ## else:  
    ##   kparams=self.input[0]
      
    return(kparams)
## ------------------------------------------------------
  def setinput(self):
    
    self.input=[ cls_empty_class() for i in range(self.ninputkernel) ]

  ## new style of centralization and normaliation
    self.input[0].ilocal=2 ## centralization type see mmr_normalization_new.py
    self.input[0].iscale=0 ## normalization type see mmr_normalization_new.py
    
    self.input[0].kernel_type=0 ## 0 linear, 1 polynomial(multinomila,
                                  ## 3 Gaussian
                                  ## 11 poly-uniform 
    self.input[0].bias=0.0001    ## bias factor for input
    
    self.input[0].prekernel=1   ## =0 no prekernel =1 compute prekernel
    
  ## possible parameter ranges scanned in the validation
    if self.input[0].kernel_type==1:
  ## polynomial multinomial kernel par1=degree and par2=constant
      self.input[0].par1min=2.0
      self.input[0].par1max=2.0
      self.input[0].par2min=0.04
      self.input[0].par2max=0.04
      self.input[0].par1step=0.5
      self.input[0].par2step=0.02
    elif self.input[0].kernel_type==11:
  ## polynomial uniform kernel par1=degree and par2=constant, 
      self.input[0].par1min=1
      self.input[0].par1max=10
      self.input[0].par2min=1
      self.input[0].par2max=1 
      self.input[0].par1step=1
      self.input[0].par2step=1
    elif self.input[0].kernel_type==12:
  ## trigonometric polynomial kernel par1=maximum frequency  and par2=constant, 
      self.input[0].par1min=50
      self.input[0].par1max=50
      self.input[0].par2min=1
      self.input[0].par2max=1 
      self.input[0].par1step=1
      self.input[0].par2step=1
    elif self.input[0].kernel_type==2:
  ## sigmoid kernel par1=factor and par2=constant
      self.input[0].par1min=0.01 
      self.input[0].par1max=0.11 
      self.input[0].par2min=0.01 
      self.input[0].par2max=0.01 
      self.input[0].par1step=0.01 
      self.input[0].par2step=0.01 
    elif self.input[0].kernel_type==3:
  ## Gaussian kernel par1=variance(width) 
      self.input[0].par1min=0.5
      self.input[0].par1max=0.5   ## ~10 
      self.input[0].par2min=0 
      self.input[0].par2max=0
      self.input[0].par1step=1 
      self.input[0].par2step=1 
      self.input[0].nrange=10
    elif self.input[0].kernel_type==31:
  ## PolyGaussian kernel par1=variance(width) 
      self.input[0].par1min=0.4 
      self.input[0].par1max=0.8 ## ~10 
      self.input[0].par2min=1.0
      self.input[0].par2max=4.0
      self.input[0].par1step=1.0 
      self.input[0].par2step=0.5
      self.input[0].nrange=10 
      self.input[0].dpower=1.0
      self.input[0].spower=1.0
    elif self.input[0].kernel_type==41:
  ## PolyLaplace kernel par1=variance(width) 
      self.input[0].par1min=0.3 
      self.input[0].par1max=0.8 ## ~10 
      self.input[0].par2min=0 
      self.input[0].par2max=0
      self.input[0].par1step=1 
      self.input[0].par2step=1 
      self.input[0].nrange=5 
    else:
      self.input[0].par1min=0 
      self.input[0].par1max=0  
      self.input[0].par2min=0 
      self.input[0].par2max=0
      self.input[0].par1step=1 
      self.input[0].par2step=1 

    self.input[0].ipar1=self.input[0].par1min
    self.input[0].ipar2=self.input[0].par2min

## ----------------------------------------------------  
## ## ##################################################################
##  ## new style of centralization and normaliation
    self.input[1].ilocal=2 ## centralization see mmr_normalization_new.py
    self.input[1].iscale=0 ## normalization see mmr_normalization_new.py

    self.input[1].kernel_type=1  ## 0 linear, 1 polynomial, 3 Gaussian
  ## possible parameter ranges scanned in the validation
    if self.input[1].kernel_type==1:
  ## polynomial kernel par1=degree and par2=constant
      self.input[1].par1min=2.0
      self.input[1].par1max=2.0
      self.input[1].par2min=0.00
      self.input[1].par2max=0.00
      self.input[1].par1step=0.5 
      self.input[1].par2step=0.02 
    elif self.input[1].kernel_type==2:
  ## sigmoid kernel par1=factor and par2=constant
      self.input[1].par1min=0.01 
      self.input[1].par1max=0.1 
      self.input[1].par2min=0.0 
      self.input[1].par2max=0.1 
      self.input[1].par1step=0.01 
      self.input[1].par2step=0.01 
    elif self.input[1].kernel_type==3:
  ## Gaussian kernel par1=variance(width) 
      self.input[1].par1min=0.74
      self.input[1].par1max=0.74
      self.input[1].par2min=0.0 
      self.input[1].par2max=0.0
      self.input[1].par1step=1.0 
      self.input[1].par2step=1.0 
      self.input[1].nrange=10
    elif self.input[1].kernel_type==31:
  ## PolyGaussian kernel par1=variance(width) 
      self.input[1].par1min=2.5
      self.input[1].par1max=2.5 ## ~10 
      self.input[1].par2min=1.5 
      self.input[1].par2max=2.5
      self.input[1].par1step=1.0 
      self.input[1].par2step=0.5 
      self.input[1].nrange=20 
    elif self.input[1].kernel_type==41:
  ## PolyLaplace kernel par1=variance(width) 
      self.input[1].par1min=0.3 
      self.input[1].par1max=0.8 ## ~10 
      self.input[1].par2min=0.0 
      self.input[1].par2max=0.0
      self.input[1].par1step=1.0 
      self.input[1].par2step=1.0 
      self.input[1].nrange=5 
    else:
      self.input[1].par1min=0.0 
      self.input[1].par1max=0.0 
      self.input[1].par2min=0.0 
      self.input[1].par2max=0.0
      self.input[1].par1step=1.0 
      self.input[1].par2step=1.0 

    self.input[1].ipar1=self.input[1].par1min
    self.input[1].ipar2=self.input[1].par2min
## ## ##################################################################
##  ## new style of centralization and normaliation
    self.input[2].ilocal=2 ## centralization see mmr_normalization_new.py
    self.input[2].iscale=0 ## normalization see mmr_normalization_new.py

    self.input[2].kernel_type=1  ## 0 linear, 1 polynomial, 3 Gaussian
  ## possible parameter ranges scanned in the validation
    if self.input[2].kernel_type==1:
  ## polynomial kernel par1=degree and par2=constant
      self.input[2].par1min=3.0
      self.input[2].par1max=3.0 
      self.input[2].par2min=0.02 
      self.input[2].par2max=0.02
      self.input[2].par1step=1.0 
      self.input[2].par2step=0.1 
    elif self.input[2].kernel_type==2:
  ## sigmoid kernel par1=factor and par2=constant
      self.input[2].par1min=0.01 
      self.input[2].par1max=0.1 
      self.input[2].par2min=0.0 
      self.input[2].par2max=0.1 
      self.input[2].par1step=0.01 
      self.input[2].par2step=0.01 
    elif self.input[2].kernel_type==3:
  ## Gaussian kernel par1=variance(width) 
      self.input[2].par1min=0.4
      self.input[2].par1max=1.6
      self.input[2].par2min=0.0 
      self.input[2].par2max=0.0
      self.input[2].par1step=1.0 
      self.input[2].par2step=1.0 
      self.input[2].nrange=10
    elif self.input[2].kernel_type==31:
  ## PolyGaussian kernel par1=variance(width) 
      self.input[2].par1min=0.01 
      self.input[2].par1max=10 ## ~10 
      self.input[2].par2min=1.5 
      self.input[2].par2max=2.5
      self.input[2].par1step=1.0 
      self.input[2].par2step=0.5 
      self.input[2].nrange=20 
    elif self.input[2].kernel_type==41:
  ## PolyLaplace kernel par1=variance(width) 
      self.input[2].par1min=0.3 
      self.input[2].par1max=0.8 ## ~10 
      self.input[2].par2min=0.0 
      self.input[2].par2max=0.0
      self.input[2].par1step=1.0 
      self.input[2].par2step=1.0 
      self.input[2].nrange=5 
    else:
      self.input[2].par1min=0.0 
      self.input[2].par1max=0.0 
      self.input[2].par2min=0.0 
      self.input[2].par2max=0.0
      self.input[2].par1step=1.0 
      self.input[2].par2step=1.0 

    self.input[2].ipar1=self.input[2].par1min
    self.input[2].ipar2=self.input[2].par2min
  
## ----------------------------------------------------  
  def setinputraw(self):
    
  ## new style of centralization and normaliation
    self.inputraw.ilocal=2 ## centralization type see mmr_normalization_new.py
    self.inputraw.iscale=0 ## normalization type see mmr_normalization_new.py
    
    self.inputraw.kernel_type=3 ## 0 linear, 1 polynomial(multinomila,
                                  ## 3 Gaussian
                                  ## 11 poly-uniform 
    self.inputraw.bias=0.0001    ## bias factor for input
    
    
  ## possible parameter ranges scanned in the validation
    if self.inputraw.kernel_type==1:
  ## polynomial multinomial kernel par1=degree and par2=constant
      self.inputraw.par1min=2.0
      self.inputraw.par1max=2.0
      self.inputraw.par2min=0.04
      self.inputraw.par2max=0.04
      self.inputraw.par1step=0.5
      self.inputraw.par2step=0.02
    elif self.inputraw.kernel_type==11:
  ## polynomial uniform kernel par1=degree and par2=constant, 
      self.inputraw.par1min=1
      self.inputraw.par1max=10
      self.inputraw.par2min=1
      self.inputraw.par2max=1 
      self.inputraw.par1step=1
      self.inputraw.par2step=1
    elif self.inputraw.kernel_type==12:
  ## trigonometric polynomial kernel par1=maximum frequency  and par2=constant, 
      self.inputraw.par1min=50
      self.inputraw.par1max=50
      self.inputraw.par2min=1
      self.inputraw.par2max=1 
      self.inputraw.par1step=1
      self.inputraw.par2step=1
    elif self.inputraw.kernel_type==2:
  ## sigmoid kernel par1=factor and par2=constant
      self.inputraw.par1min=0.01 
      self.inputraw.par1max=0.11 
      self.inputraw.par2min=0.01 
      self.inputraw.par2max=0.01 
      self.inputraw.par1step=0.01 
      self.inputraw.par2step=0.01 
    elif self.inputraw.kernel_type==3:
  ## Gaussian kernel par1=variance(width) 
      self.inputraw.par1min=0.5
      self.inputraw.par1max=0.5   ## ~10 
      self.inputraw.par2min=0 
      self.inputraw.par2max=0
      self.inputraw.par1step=1 
      self.inputraw.par2step=1 
      self.inputraw.nrange=10
    elif self.inputraw.kernel_type==31:
  ## PolyGaussian kernel par1=variance(width) 
      self.inputraw.par1min=0.4 
      self.inputraw.par1max=0.8 ## ~10 
      self.inputraw.par2min=1.0
      self.inputraw.par2max=4.0
      self.inputraw.par1step=1.0 
      self.inputraw.par2step=0.5
      self.inputraw.nrange=10 
      self.inputraw.dpower=1.0
      self.inputraw.spower=1.0
    elif self.inputraw.kernel_type==41:
  ## PolyLaplace kernel par1=variance(width) 
      self.inputraw.par1min=0.3 
      self.inputraw.par1max=0.8 ## ~10 
      self.inputraw.par2min=0 
      self.inputraw.par2max=0
      self.inputraw.par1step=1 
      self.inputraw.par2step=1 
      self.inputraw.nrange=5 
    else:
      self.inputraw.par1min=0 
      self.inputraw.par1max=0  
      self.inputraw.par2min=0 
      self.inputraw.par2max=0
      self.inputraw.par1step=1 
      self.inputraw.par2step=1 

    self.inputraw.ipar1=self.inputraw.par1min
    self.inputraw.ipar2=self.inputraw.par2min

## ##################################################################
  ## Output subspace kernel base
  def setoutput1(self):  

## ilabmode:  labeling mode of multiclass
##               =0 indicators
##               =1 class mean
##               =2 class median
##               =3 tetrahedron
##               =31 weighted simplex

    self.output1.ilabmode=31
    self.output1.ndepth=3

    self.output1.nclass=2
  ## new style of centralization and normaliation
    self.output1.ilocal=0 ## centralization type see mmr_normalization_new.py
    self.output1.iscale=4 ## normalization type see mmr_normalization_new.py
    self.output1.offset=0.01

    self.output1.nboro=10
    self.output1.neig=5    ## number of greatest eigenvalues taken
    self.output1.ioperator_valued=0  ## =0 scalar kernel, =1 operator valued
    self.output1.col_dim=5        ## dimension of column representation
    
    self.output1.kernel_type=1  ## 0 linear, 1 polynomial, 3 Gaussian

    if self.output1.kernel_type==32:
      self.output1.iyfeature=1     ## =0 explicit y feature =1 kernel
      self.output1.itestmode=1    ## =0 dot(Y0,Zw), =1 max(Zw,axis=1) items
      self.output1.nmax=4         ## number of largest values in Zw considered
      self.output1.preimage=1      ## =0 testing agains training
                                ## =1 approxiamte dynamic programming
    else:
      self.output1.iyfeature=0     ## =0 explicit y feature =1 kernel
      self.output1.itestmode=0    ## =0 dot(Y0,Zw), =1 max(Zw,axis=1) items
      self.output1.nmax=4         ## number of largest values in Zw considered
      self.output1.preimage=0      ## =0 testing agains training
                                ## =1 approxiamte dynamic programming
    
  ## possible parameter ranges scanned in the validation
    if self.output1.kernel_type==1:
  ## polynomial kernel par1=degree and par2=constant
      self.output1.par1min=2
      self.output1.par1max=2
      self.output1.par2min=0.04
      self.output1.par2max=0.04
      self.output1.par1step=0.5 
      self.output1.par2step=0.02
    elif self.output1.kernel_type==2:
  ## sigmoid kernel par1=factor and par2=constant
      self.output1.par1min=0.01 
      self.output1.par1max=0.1 
      self.output1.par2min=0 
      self.output1.par2max=0.1 
      self.output1.par1step=0.01 
      self.output1.par2step=0.01 
    elif self.output1.kernel_type==3:
  ## Gaussian kernel par1=variance(width) 
      self.output1.par1min=1.0
      self.output1.par1max=3.0 ## ~10 
      self.output1.par2min=0 
      self.output1.par2max=0
      self.output1.par1step=1 
      self.output1.par2step=1 
      self.output1.nrange=10 
    elif self.output1.kernel_type==31:
  ## PolyGaussian kernel par1=variance(width) 
      self.output1.par1min=0.01 
      self.output1.par1max=10 ## ~10 
      self.output1.par2min=1.5 
      self.output1.par2max=2.5
      self.output1.par1step=1 
      self.output1.par2step=0.5 
      self.output1.nrange=20 
      self.output1.dpower=1.0
      self.output1.spower=1.0
    elif self.output1.kernel_type==32:
  ## Gaussian with full covariance 
      self.output1.par1min=np.sqrt(0.5)*0.25
      self.output1.par1max=np.sqrt(0.5)*0.25 ## ~10 
      self.output1.par2min=1.5 
      self.output1.par2max=2.5
      self.output1.par1step=1 
      self.output1.par2step=0.5 
      self.output1.nrange=20 
    elif self.output1.kernel_type==41:
  ## PolyLaplace kernel par1=variance(width) 
      self.output1.par1min=0.3 
      self.output1.par1max=0.8 ## ~10 
      self.output1.par2min=0 
      self.output1.par2max=0
      self.output1.par1step=1 
      self.output1.par2step=1 
      self.output1.nrange=5 
    else:
      self.output1.par1min=0 
      self.output1.par1max=0  
      self.output1.par2min=0 
      self.output1.par2max=0
      self.output1.par1step=1 
      self.output1.par2step=1 


    self.output1.dpower=1.0

    self.output1.ipar1=self.output1.par1min
    self.output1.ipar2=self.output1.par2min
## ################################################################
  ## Output subspace kernel items
  def setoutput2(self):  

## ilabmode:  labeling mode of multiclass
##               =0 indicators
##               =1 class mean
##               =2 class median
##               =3 tetrahedron
##               =31 weighted simplex

    self.output2.ilabmode=31
    self.output2.ndepth=3

    self.output2.nclass=2
  ## new style of centralization and normaliation
    self.output2.ilocal=-1 ## centralization type see mmr_normalization_new.py
    self.output2.iscale=-1 ## normalization type see mmr_normalization_new.py
    self.output2.offset=0.01

    self.output2.nboro=10
    self.output2.neig=5    ## number of greatest eigenvalues taken
    self.output2.ioperator_valued=0  ## =0 scalar kernel, =1 operator valued
    self.output2.col_dim=5        ## dimension of column representation
    
    self.output2.kernel_type=0  ## 0 linear, 1 polynomial, 3 Gaussian

    if self.output2.kernel_type==32:
      self.output2.iyfeature=1     ## =0 explicit y feature =1 kernel
      self.output2.itestmode=1    ## =0 dot(Y0,Zw), =1 max(Zw,axis=1) items
      self.output2.nmax=4         ## number of largest values in Zw considered
      self.output2.preimage=1      ## =0 testing agains training
                                ## =1 approxiamte dynamic programming
    else:
      self.output2.iyfeature=1     ## =0 explicit y feature =1 kernel
      self.output2.itestmode=0    ## =0 dot(Y0,Zw), =1 max(Zw,axis=1) items
      self.output2.nmax=4         ## number of largest values in Zw considered
      self.output2.preimage=0      ## =0 testing agains training
                                ## =1 approxiamte dynamic programming
    
  ## possible parameter ranges scanned in the validation
    if self.output2.kernel_type==1:
  ## polynomial kernel par1=degree and par2=constant
      self.output2.par1min=1
      self.output2.par1max=1 
      self.output2.par2min=1 
      self.output2.par2max=1 
      self.output2.par1step=1 
      self.output2.par2step=1 
    elif self.output2.kernel_type==2:
  ## sigmoid kernel par1=factor and par2=constant
      self.output2.par1min=0.01 
      self.output2.par1max=0.1 
      self.output2.par2min=0 
      self.output2.par2max=0.1 
      self.output2.par1step=0.01 
      self.output2.par2step=0.01 
    elif self.output2.kernel_type==3:
  ## Gaussian kernel par1=variance(width) 
      self.output2.par1min=0.02
      self.output2.par1max=0.02 ## ~10 
      self.output2.par2min=0 
      self.output2.par2max=0
      self.output2.par1step=1 
      self.output2.par2step=1 
      self.output2.nrange=7 
    elif self.output2.kernel_type==31:
  ## PolyGaussian kernel par1=variance(width) 
      self.output2.par1min=0.01 
      self.output2.par1max=10 ## ~10 
      self.output2.par2min=1.5 
      self.output2.par2max=2.5
      self.output2.par1step=1 
      self.output2.par2step=0.5 
      self.output2.nrange=20 
      self.output2.dpower=1.0
      self.output2.spower=1.0
    elif self.output2.kernel_type==32:
  ## Gaussian with full covariance 
      self.output2.par1min=np.sqrt(0.5)*0.25
      self.output2.par1max=np.sqrt(0.5)*0.25 ## ~10 
      self.output2.par2min=1.5 
      self.output2.par2max=2.5
      self.output2.par1step=1 
      self.output2.par2step=0.5 
      self.output2.nrange=20 
    elif self.output2.kernel_type==41:
  ## PolyLaplace kernel par1=variance(width) 
      self.output2.par1min=0.3 
      self.output2.par1max=0.8 ## ~10 
      self.output2.par2min=0 
      self.output2.par2max=0
      self.output2.par1step=1 
      self.output2.par2step=1 
      self.output2.nrange=5 
    else:
      self.output2.par1min=0 
      self.output2.par1max=0  
      self.output2.par2min=0 
      self.output2.par2max=0
      self.output2.par1step=1 
      self.output2.par2step=1 


    self.output2.dpower=1.0

    self.output2.ipar1=self.output2.par1min
    self.output2.ipar2=self.output2.par2min
## ################################################################



