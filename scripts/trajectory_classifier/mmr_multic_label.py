######################
## Version 0.1 #######
######################
## import os, string, re, math, pickle, random
## import math, time
## import numpy
## import pylab as lab
from numpy import eye, zeros, where, median, sqrt, ones, linalg, diag
from numpy import dot, argmin

## #################################################################
def mmr_multic_label(ilabmode,Y,X,kk,lpar):
## It labels the outputs for multiclass classification
## Input:    ilabmode  labeling mode
##                 =0 indicators
##                 =1 class mean
##                 =2 class median
##                 =3 tetrahedron
##                 =31 weighted simplex  
##           Y     output categories 
##                 column vector with components =1,...,kk  
##           X     corresponding input vectors, 
##                 the rows contain the input vectors
##           kk    number of possible categories
##           lpar  optional parameter used by method 31
## Output:   YL  label vectors in its rows to all sample items
##           Y0  all possible labels, it has kk rows and in the rows the
##               possible labels  

## number of items and input dimension  
  (m,nx)=X.shape
  if ilabmode==0:
## the indicator case for multiclass learning
    Y0=eye(kk)
    ## setting the label vectors 
    YL=zeros((m,kk))
    for i in range(m):
      YL[i,Y[i]]=1
  elif ilabmode==1:
## class mean
    Y0=zeros((kk,nx))
    xmm=zeros((kk,nx))
    xnn=zeros(kk)
    for i in range(m):
      iy=Y[i]
      xmm[iy,:]=xmm[iy,:]+X[i,:]
      xnn[iy]+=1
    for k in range(kk):
      if xnn[k]>0:
        Y0[k,:]=xmm[k,:]/xnn[k]
    YL=zeros((m,nx))
    for i in range(m):
      YL[i,:]=Y0[Y[i],:]
  elif ilabmode==2:
## class median
    Y0=zeros((kk,nx))
    for k in range(kk):
      inx=where(Y==k)[0]
      if len(inx)>0:
        xmm=median(X[inx,:],axis=0)
        Y0[k,:]=xmm/sqrt(sum(xmm**2))
    YL=zeros((m,nx))
    for i in range(m):
      YL[i,:]=Y0[Y[i],:]
  elif ilabmode==3:
## tetrahedron, minimum correlation
    Y0=eye(kk)
    Y0=Y0+Y0/(kk-1)-ones((kk,kk))/(kk-1)
    (S,U)=linalg.eigh(Y0)
    SS=dot(U,diag(sqrt(abs(S))))
    ix=argmin(S)
    Y0=zeros((kk,kk-1))
    j=0
    for k in range(kk):
      if k!=ix:
        Y0[:,j]=SS[:,k]
        j+=1
    YL=zeros((m,kk-1))
    for i in range(m):
      YL[i,:]=Y0[Y[i],:]
  elif ilabmode==31:
    if kk>1:
      lpar=float(1)/(kk-1)
    else:
      lpar=1.0
    Y0=(1+lpar)*eye(kk)-lpar*ones((kk,kk))
    YL=zeros((m,kk))
    for i in range(m):
      YL[i,:]=Y0[Y[i,0],:]
  else:
    pass
  
  return(YL,Y0)

        
        
