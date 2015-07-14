######################
## Version 0.1 #######
######################
## import os, string, re, math, pickle, random
## import math, time
from numpy import sqrt, diag, zeros, dot, ones, outer, abs
## import pylab as lab
## #####################
 
## #######################################################################
def mmr_solver(Kx,Ky,C,D,normy1,normy2,normx1,normx2):
## solve an unbiased mmr
  err_tolerance=0.001
  maxiter=100
  xeps=10**(-4)

## input output norms
  dx=diag(Kx)
  dy=diag(Ky)
  dx=dx+(abs(dx)+xeps)*(dx<=0)
  dy=dy+(abs(dy)+xeps)*(dy<=0)
  dKx=sqrt(dx)
  dKy=sqrt(dy)

  dKxy1=dKx**normx1*dKy**normy1   ## norm based scale of the margin
  dKxy2=dKx**normx2*dKy**normy2   ## norm based scale of the loss

  dKxy2+=1.0*(dKxy2==0)    ## to avoid zero

  lB=float(D)/(dKxy2)               ## scale the ranges
  uB=float(C)/(dKxy2)

  Bdiff=uB-lB
  z_eps=err_tolerance*sqrt(sum(Bdiff*Bdiff))

  qs=-dKxy1

  Kxy=Kx*Ky
  [m,n]=Kxy.shape
## scaling by diagonal elements  
  dKxy=diag(Kx)
  ## dKxy=dKxy+(dKxy==0)
  ## Kxy=Kxy/outer(dKxy,dKxy)
  ## qs=qs/dKxy
  
##  xalpha=zeros(m)
  xalpha=lB
  xalpha0=xalpha.copy()
  for irepeat in range(maxiter):
    for irow in range(m):
      t=(-qs[irow]-dot(Kxy[irow],xalpha0))/Kxy[irow,irow]
      ## t=-qs[irow]-dot(Kxy[irow],xalpha0)
      xnew=xalpha0[irow]+t
      lbi=lB[irow]
      ubi=uB[irow]
      if lbi<xnew:
        if ubi>xnew:
          xalpha0[irow]=xnew
        else:
          xalpha0[irow]=ubi
      else:
        xalpha0[irow]=lbi
    xdiff=xalpha0-xalpha
    zerr=sqrt(sum(xdiff*xdiff))     ## L2 norm error
##     zerr=max(abs(xdiff))     ## L_infty norm error
    xalpha=xalpha0.copy()
    if zerr<z_eps:
##       print irepeat
      break
## xalpha the dual solution
  return(xalpha)
## #######################################################################
def mmr_solver_bias(Kx,Ky,C,D,normy1,normy2,normx1,normx2):
## solve an unbiased mmr

##   C=1.0
##   D=0.0

  cgrowmax=4.0    ## growing factor of the penalty constant
  ck=2            ## initial penalty
  maxiterout=20   ## max iteration of the Lagrangien
  maxiterin=100   ## maximum iteration of the inner loop
  err_tolerance_out=0.0001;  ## error tolerance of the Lagrangian of the dual
  err_tolerance_in=0.00001;  ## error tolerance of the dual variables
  ilambda=1       ## Lagrangian is considered
  xeps=10**(-4)
    
  cgrow=cgrowmax
  
## input output norms
  dKx=sqrt(diag(Kx))
  dKy=sqrt(diag(Ky))

  dKxy1=dKx**normx1*dKy**normy1   ## norm based scale of the margin
  dKxy2=dKx**normx2*dKy**normy2   ## norm based scale of the loss

  dKxy2+=1.0*(dKxy2==0)    ## to avoid zero

  lB=float(D)/(dKxy2)               ## scale the ranges
  uB=float(C)/(dKxy2)

  Bdiff=uB-lB
  z_eps=err_tolerance_in*sqrt(sum(Bdiff*Bdiff))
  herr=10**10

  Kxy=Kx*Ky
  (m,n)=Kxy.shape           ## kernel size

  xlambda=zeros(m)          ## initialization of the Lagrangien
  xlambda0=xlambda.copy()

  Ky2=dot(Ky,Ky)
  qs0=-dKxy1

  xalpha=zeros(m)
  for iouter in range(maxiterout):
    xalpha0=xalpha.copy()
    if ilambda==1:              ## augmented Lagrangian
      qs1=dot(Ky,xlambda)
      qs=qs0+qs1
      KK=Kxy+ck*Ky2
    else:
      qs=qs0
      KK=Kxy

## scaling by diagonal elements  
    dKxy=diag(KK)
    dKxy=dKxy+xeps*(dKxy==0)
    KK=KK+diag(dKxy)
    ## KK=KK/outer(dKxy,ones(m))
    ## qs=qs/dKxy
 
    xalpha0=xalpha.copy()
    for irepeat in range(maxiterin):
      for irow in range(m):
        t=(-qs[irow]-dot(KK[irow],xalpha0))/KK[irow,irow]
        xnew=xalpha0[irow]+t
        if lB[irow]<xnew:
          if uB[irow]>xnew:
            xalpha0[irow]=xnew
          else:
            xalpha0[irow]=uB[irow]
        else:
          xalpha0[irow]=lB[irow]
      xdiff=xalpha0-xalpha
      zerr=sqrt(sum(xdiff*xdiff))     ## L2 norm error
  ##     zerr=max(abs(xdiff))     ## L_infty norm error
      xalpha=xalpha0.copy()
      if zerr<z_eps:
  ##       print irepeat
        break

    h=dot(Ky.transpose(),xalpha)
    xlambda0=xlambda.copy()
    xlambda+=ck*h/cgrow
    ck=cgrow*ck
    xdiff_lambda=xlambda-xlambda0
    lerr=sqrt(sum(xdiff_lambda*xdiff_lambda))
    herr0=herr
    herr=sum(h*h)
    if herr<err_tolerance_out or abs(herr/herr0)-1<err_tolerance_out:
      break
    herr0=herr
      
## xalpha the dual solution
  return(xalpha,xlambda)
## ########################################################################
  

