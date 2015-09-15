## ##################################################
import numpy as np
import scipy.linalg as sp_linalg

## ###################################################
## Solve the kernel pca problem
##
## input:
## K        2d-array is expected to be centralized kernel
## neig     number of greatest eigenvalues
## output:
## seig     neig greatest eigenvalues
## veig     neig eigen vectors belonging to seig
## #######################################################
def kpca(K,neig):
  m=K.shape[0]
## centralization
  J=np.ones((m,m))
  K=K-np.dot(J,K)/m-np.dot(K,J)/m+np.dot(J,np.dot(K,J))/(m**2)

  hi=m-1
  lo=m-1-neig+1
  if lo<0:
    lo=0
  (seig,Veig)=sp_linalg.eigh(K/m,b=None,eigvals=(lo,hi))

  iseig=np.argsort(-seig)
  Veig=Veig[:,iseig]
  seig=seig[iseig]

  iz=np.where(seig<0)[0]
  seig[iz]=0.0
  
  vnorm=np.sqrt(seig*m)
  vnorm=vnorm+(vnorm==0)
  Veig=Veig/np.tile(vnorm,(m,1))

  return(seig,Veig)

  

  
  
  

  
