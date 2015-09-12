######################
## Version 0.1 #######
######################

## import sys
import numpy as np
## ###########################################################
from mmr_normalization_new import mmr_normalization
from mmr_kernel_eval import kernel_eval_kernel, tanimoto, cross_table_kernel
from mmr_solver import mmr_solver
## import mmr_subspace
## ###########################################################

## ##################################################
class cls_edgekernel:

  def __init__(self):

    self.dalpha={}
    self.dxy={}
    self.ddiag=None

    
## ----------------------------------------
  def edge_prekernel(self,X,params_spec):

    (m,n)=X.shape   ## xdataraw[0]
    dalpha={}
    dxy={}
    ialpha=1
    ny=2

    self.ddiag=np.zeros(m)

    for iview in range(m):
      lxy=[]
      i=ny
      while i<n: 
        edge=X[iview,i:i+3]
        if edge[0]!=0:
          lxy.append(tuple(edge))
        else:
          break
        i+=3

      xy=np.array(lxy)
      x=xy[:,:2]
      y=xy[:,2]

      y=np.vstack((np.cos(y),np.sin(y))).T
      ilocal=-1
      iscale=-1
      (xnorm,xdummy,opar)=mmr_normalization(ilocal,iscale,x,None,0)
      dxy[iview]=[xnorm,y]
      if ialpha==1:
        Ky=np.dot(y,y.T)
        xdata=[xnorm,None]
        (Kx,dx1,dx2)=kernel_eval_kernel(xdata,None,None,params_spec)
        C=1
        D=0
        xalpha=mmr_solver(Kx,Ky,C,D,1,1,1,1)
        dalpha[iview]=xalpha
      else:
        xalpha=np.ones(len(y))/len(y)
        dalpha[iview]=xalpha
      
      self.ddiag[iview]=np.dot(xalpha,np.dot(Kx*Ky,xalpha))

    return
## ----------------------------------------
  def prekernel(self,Xdata,itrain,itest,kernel_params,prekernel_params):

    mtrain=len(itrain)
    mtest=len(itest)
    K=np.zeros((mtrain,mtest))
    for i in itrain:
      for j in itest:
        xdata=[self.dxy[i][0],self.dxy[j][0]]
        (Kx,dx1,dx2)=kernel_eval_kernel(xdata,None,None,kernel_params)
        Ky=np.dot(self.dxy[i][1],self.dxy[j][1].T)
        K[i,j]=np.dot(self.dalpha[i],np.dot((Ky*Kx),self.dalpha[j]))
        
    d1=self.ddiag[itrain]
    d2=self.ddiag[itest]
    
    return(K,d1,d2)
## ############################################################
class cls_oddskernel:
  """
  Only for output kernel based on the training
  """
  def __init__(self):
    return

  ## -------------------------------------------------  

  def prekernel(self,Xdata,itrain,itest,kernel_params,prekernel_params):

    m=Xdata.shape[0]
    if kernel_params.prekernel_type==61:
      K=tanimoto(Xdata)
    else:
      K=cross_table_kernel(Xdata)
    dKY=np.diag(K)
    dKY=np.sqrt(dKY*(dKY>=0))
    dKY=dKY+(dKY==0)
    K=K/np.outer(dKY,dKY)
    d1=np.ones(m)
    d2=d1
    
    return(K,d1,d2)

## ############################################################
