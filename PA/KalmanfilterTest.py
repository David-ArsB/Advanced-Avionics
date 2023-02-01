# -*- coding: utf-8 -*-
"""
Created on Wed Jan 18 10:26:33 2023

@author: Catherine
"""
import numpy as np
from Kalmanfilter_fct import * 
from GPSPoller import GpsPoller
from LSM6DSL import LSM6DSL

#initial state

x = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T
#print(x, x.shape)
n=x.size # States

#initial uncertainty

P = np.diag([100.0, 100.0, 10.0, 10.0, 1.0, 1.0])
#print(P, P.shape)

#dynamic matrix

dt = 0.1 # Time Step between Filter Steps

A = np.matrix([[1.0, 0.0, dt, 0.0, 1/2.0*dt**2, 0.0],
              [0.0, 1.0, 0.0, dt, 0.0, 1/2.0*dt**2],
              [0.0, 0.0, 1.0, 0.0, dt, 0.0],
              [0.0, 0.0, 0.0, 1.0, 0.0, dt],
              [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
#print(A, A.shape)

#Measurement matrix

H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
#print(H, H.shape)

#Measurement noise covariance 

ra = 10.0**2   # Noise of Acceleration Measurement
rp = 100.0**2  # Noise of Position Measurement
R = np.matrix([[rp, 0.0, 0.0, 0.0],
               [0.0, rp, 0.0, 0.0],
               [0.0, 0.0, ra, 0.0],
               [0.0, 0.0, 0.0, ra]])
#print(R, R.shape)

# processs noise covariance matrix

sj=0.1

Q = np.matrix([[(dt**6)/36, 0, (dt**5)/12, 0, (dt**4)/6, 0],
               [0, (dt**6)/36, 0, (dt**5)/12, 0, (dt**4)/6],
               [(dt**5)/12, 0, (dt**4)/4, 0, (dt**3)/2, 0],
               [0, (dt**5)/12, 0, (dt**4)/4, 0, (dt**3)/2],
               [(dt**4)/6, 0, (dt**3)/2, 0, (dt**2),0],
               [0, (dt**4)/6, 0, (dt**3)/2, 0, (dt**2)]]) *sj**2

#print(Q, Q.shape)

#I matrix

I = np.eye(n)
#print(I, I.shape)

#tests
cd1 = [45.5147478972148, -73.7745783865117]
cd2 = [75.48154597970112, -154.3539584108125]





# GPS readings (position)
m=500
GPS=np.ndarray(m,dtype=bool)
GPS[0]=True
mpx=np.array([0])
mpy=np.array([0])

coords=np.array([[0,0]])
dists=np.empty(0)
ori=np.empty(0)
varx=0
vary=0

for i in range(1,m):
    
   if i%10==0:
        GPS[i]=True
        gpsp=GpsPoller()
        gpsp.start()
    
        lat=gpsp.gpsd.fix.latitude
        lon=gpsp.gpsd.fix.longitude
        cord=np.array([lat,lon])
        print(cord)

        coords=np.append(coords,cord)
        
        xy=get_xy(coords)
   else:
        mpx=np.append(mpx,mpx[i-1])
        mpy=np.append(mpy,mpy[i-1])
        GPS[i]=False


#get acceleration data 
#mx=np.empty(0)
#my=np.empty(0)
#ax=readACCx()
#ay=readACCy()
#mx=np.append(mx,ax)
#my=np.append(my,ay)

#measurements=np.vstack(mpx,mpy,mx,my)
#print(measurements.shape)

#filter 
#for filterstep in range(m):
    
    # Time Update (Prediction)
    # ========================
    # Project the state ahead
 #   x = A*x
    
    # Project the error covariance ahead
  #  P = A*P*A.T + Q    
    
    
    # Measurement Update (Correction)
    # ===============================
    # if there is a GPS Measurement
   # if GPS[filterstep]:
        # Compute the Kalman Gain
    #    S = H*P*H.T + R
     #   K = (P*H.T) * np.linalg.pinv(S)
    
        
        # Update the estimate via z
      #  Z = measurements[:,filterstep].reshape(H.shape[0],1)
       # y = Z - (H*x)                            # Innovation or Residual
        #x = x + (K*y)
        
        # Update the error covariance
        #P = (I - (K*H))*P

   
    
    # Save states for Plotting
    #savestates(x, Z, P, K)
