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
from numpy import ones, zeros, where, round,   \
     int, random, dot
from numpy import min as np_min
from numpy import sum as np_sum
from numpy import sqrt as np_sqrt
## from scipy import sparse
## ####################
        
## ######################################################################
def mvm_solver(xdatacls,params):
  """
  It solves the maximum margin solution to the relation learning problem
  
  Input:
  xdatacls      data class
  params        global parameters
  Output:
  xalpha        optimal dual variables
  """
  KXfull=xdatacls.KX
  KYfull=xdatacls.KY
  KXvar=xdatacls.KXvar
  xranges=xdatacls.xranges_rel
  xdata=xdatacls.xdata_tra
  xdata1=xdata[1]
  xdata2=xdata[2]

  txdim=xdata2.shape
  if len(txdim)==1:
    nxdim=1
  else:
    nxdim=txdim[1]
  
  nrow=xdatacls.nrow
  ncol=xdatacls.ncol
  ndata=xdata[0].shape[0]

  ycategory=xdatacls.category   ## cells are categorical variables
  
  ymax=xdatacls.YKernel.ymax
  ymin=xdatacls.YKernel.ymin
  ystep=xdatacls.YKernel.ystep

  C=xdatacls.penalty.c
  
  ## xnrow=zeros(nrow)
  
  niter=params.solver.niter

  lcols={}           ## dictionary for ncol
  if params.solver.report==1:  
    print('Preparing the solver input')
  xncols=zeros(ncol)
  for idata in range(ndata):
    icol=xdata1[idata]
    xncols[icol]+=1

  for icol in range(ncol):
    lcols[icol]=zeros(xncols[icol],dtype=int)
  
  xpcols=zeros(ncol)
  for idata in range(ndata):
    icol=xdata1[idata]
    lcols[icol][xpcols[icol]]=idata
    xpcols[icol]+=1

  xweight_row=ones(nrow)
  xweight_col=ones(ncol)*C
  
  xalpha=zeros(ndata)
  tau=0
  ixalpha_star=zeros((ncol,3),dtype=int)-1
  xnabla0_prev=zeros(ndata)
  for irow in range(nrow):
    istart=xranges[irow,0]
    nlength=xranges[irow,1]
    if nlength>0:
      xnabla0_prev[istart:istart+nlength]=(-1)*xweight_row[irow]

  xtime=zeros(5)    
  
##  print('Solving optimization problem') 
## conditional gradient iteration
  for iiter in range(niter):
    t0=time.clock()
## current gradient
    xnabla0=zeros(ndata)
    for irow in range(nrow):
      istart=xranges[irow,0]
      nlength=xranges[irow,1]
      if nlength>0:
        xnabla0_prev_s=xnabla0_prev[istart:istart+nlength]
        icol_index=where(ixalpha_star[:,1]==irow)[0]
        if len(icol_index)>0:
          icols=ixalpha_star[icol_index,2]
          ixrange=xdata1[istart:istart+nlength]
          ixsubrange=ixrange[icols]

          if ycategory in (0,3):
            if nxdim==1:
              iyrange=xdata2[istart:istart+nlength]
            else:
              iyrange=xdata2[istart:istart+nlength][:,0]              
            iyrange=round((iyrange-ymin)/ystep).astype(int)
            iysubrange=iyrange[icols]
            ly=len(iyrange)
            KKY=KYfull[iyrange.reshape((ly,1)),iysubrange]
            for i in range(1,nxdim):
              iyrange=xdata2[istart:istart+nlength][:,i]
              iyrange=round((iyrange-ymin)/ystep).astype(int)
              iysubrange=iyrange[icols]
              ly=len(iyrange)
              KKY+=KYfull[iyrange.reshape((ly,1)),iysubrange]
          elif ycategory in (1,2):
            iyrange=xdata2[istart:istart+nlength]
            iysubrange=iyrange[icols]
            ly=len(iyrange)
            KKY=KYfull[iyrange.reshape((ly,1)),iysubrange]

          lx=len(ixrange)
          KKX=KXfull[ixrange.reshape((lx,1)),ixsubrange]
          KKZ=KKX*KKY
          if KXvar is not None:
            KKZ*=KXvar[irow,irow] ## including row scale
          
          ## xnabla_star=C*dot(KKZ,xweight_col[icols])
          xnabla_star=C*dot(KKZ,ones(len(icols))*C)

          xnabla0[istart:istart+nlength]+=tau*xnabla_star

        xnabla0[istart:istart+nlength]+=(1-tau)*xnabla0_prev_s-tau

    xnabla0_prev=xnabla0

    t1=time.clock()
    xtime[0]+=t1-t0
## optimum solution of subproblem    
    ixalpha_star=zeros((ncol,3),dtype=int)-1
    for icol in range(ncol):
      irows=lcols[icol]
      if len(irows)>0:
          vm=np_min(xnabla0[irows])
          imall=where(xnabla0[irows]==vm)[0]
          try:
            imp=random.randint(0,len(imall),1)[0]
          except:
            print(imall)
          im=imall[imp]
          if vm<0:
            iglob=irows[im]
            ixalpha_star[icol,0]=iglob
    ## binary search for row index
            irow=nrow>>1
            ibegin=0
            iend=nrow-1
            istat=1
            while istat==1:
              istart=xranges[irow,0]
              nlength=xranges[irow,1]
              if iglob>=istart:
                if iglob<istart+nlength:
                  ixalpha_star[icol,1]=irow
                  ixalpha_star[icol,2]=iglob-istart
                  istat=0
                else:
                  ibegin=irow+1
                  irow=(ibegin+iend)>>1
                  ## if irow==ibegin:
                  ##   irow=iend
              else:
                iend=irow-1
                irow=(ibegin+iend)>>1
                ## if irow==iend:
                ##   irow=ibegin

## find best convex combination (tau) of old and new
    t2=time.clock()
    xtime[1]+=t2-t1
    
    xdelta=-xalpha
    for icol in range(ncol):
      if ixalpha_star[icol,0]>=0:
        ## xdelta[ixalpha_star[icol,0]]=xdelta[ixalpha_star[icol,0]] \
        ##                                  +C*xweight_col[icol]
        xdelta[ixalpha_star[icol,0]]=xdelta[ixalpha_star[icol,0]] \
                                         +C*C
##    xnumerator=-sum(xnabla0*xdelta)+sum(xdelta)
    xnumerator=-np_sum(xnabla0*xdelta)
    xdenominator=0

    for irow in range(nrow):
      istart=xranges[irow,0]
      nlength=xranges[irow,1]
      if nlength>0:
        xdeltas=xdelta[istart:istart+nlength]
        inzero=where(xdeltas!=0)[0]
        if len(inzero)>0:
          ixsubrange=xdata1[istart+inzero]
          if ycategory in (0,3):
            if nxdim==1:
              iysubrange=xdata2[istart+inzero]
            else:
              iysubrange=xdata2[istart+inzero][:,0]
            iysubrange=round((iysubrange-ymin)/ystep).astype(int)
            ly=len(iysubrange)
            KKY=KYfull[iysubrange.reshape((ly,1)),iysubrange]
            for i in range(1,nxdim):
              iysubrange=xdata2[istart+inzero][:,i]
              iysubrange=round((iysubrange-ymin)/ystep).astype(int)
              ly=len(iysubrange)
              KKY+=KYfull[iysubrange.reshape((ly,1)),iysubrange]
          elif ycategory in (1,2):
            iysubrange=xdata2[istart+inzero]
            ly=len(iysubrange)
            KKY=KYfull[iysubrange.reshape((ly,1)),iysubrange]
          
          lx=len(ixsubrange)
          KKX=KXfull[ixsubrange.reshape((lx,1)),ixsubrange]
          KKZ=KKX*KKY
          
          if KXvar is not None:
            KKZ*=KXvar[irow,irow] ## including row scale

          xdeltas=xdeltas[inzero]
          xdenominator=xdenominator+ \
                        dot(xdeltas,dot(KKZ,xdeltas))
          
      if (irow+1) % 1000==0:
##        print(iiter,irow)
        pass

      if xdenominator!=0.0:
        tau=xnumerator/xdenominator
      else:
        tau=1.0

    if tau<0:
      tau=0
    if tau>1:
      tau=1
    xalpha=xalpha+tau*xdelta

    t3=time.clock()
    xtime[2]+=t3-t2
    
    if (iiter+1)%10==0:
      xerr=np_sqrt(np_sum((tau*xdelta)**2))
      if params.solver.report==1:  
        print('%4d'%iiter, '%6.4f'%tau, '%12.6f'%xerr,'%12.6f'%(-xnumerator))
        ## print(xtime)

  if params.solver.report==1:  
    print(xtime)
  
  return(xalpha)
                 
