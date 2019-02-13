function [mu, Sigma, predMu, predSigma, zhat, G, R, H, K ] = ekfUpdate( ...
    mu, Sigma, u, deltaT, M, z, Q, markerId)

% NOTE: The header is not set in stone.  You may change it if you like.
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(markerId);
landmark_y = FIELDINFO.MARKER_Y_POS(markerId);

stateDim=3;
motionDim=3;
observationDim=2;
%-----------------------------------------------
%Define variables 
% -------------------------------------------
%KNOWN
% mu         mean of state 
% Sigma     Variance of state
% u,         [drot1, dtrans, drot2]' Odometry control command
    % x' = x + ˆ?trans cos(? + ˆ?rot1)
    % y' = y + ˆ?trans sin(? + ˆ?rot1)
    % ?' = ? + ˆ?rot1 + ˆ?rot2
% deltaT     Step size between filter updates, can be less than 1.
% M,         Motion noise
% z          bearing_noise observation 
% Q          Observation noise
% markerId   landmark id

%OUTPUT
% H          Jaxobian of h w.r.t location 
% K          Kalman gain
% G          Jacobian of g w,r,t location 
% V          Jacobian of g w.r.t control
% R          transfromed motion noise covariance
% zhat,     prediction measurement mean

% --------------------------------------------
% Prediction step
% --------------------------------------------

% EKF prediction of mean and covariance
 mu(3)=minimizedAngle(mu(3));
  G=[ 1, 0, -u(2)*sin(u(1)+mu(3)); 
      0,1,u(2)*cos(mu(3)+u(1));
      0,0,1];
  V=[ -u(2)*sin(u(1)+mu(3)),cos(u(1)+mu(3)),0;
      u(2)*cos(mu(3)+u(1)),sin(u(1)+mu(3)),0;
      1,0,1];  
  predMu=prediction(mu,u);
  R=V*M*V';
  predSigma=G*Sigma*G'+R;
  
%--------------------------------------------------------------
% Correction step
%--------------------------------------------------------------

% Compute expected observation and Jacobian
obs=observation(predMu,markerId);
zhat=obs(1);
q=(landmark_x-predMu(1))^2+(landmark_y-predMu(2))^2;
H=[-(landmark_x-predMu(1))/sqrt(q),-(landmark_y-predMu(2))/sqrt(q),0;
    (landmark_y-predMu(2))/q,-(landmark_x-predMu(1))/q,-1;
    0,0,0];
% Innovation / residual covariance
S=H(2,:)*predSigma*H(2,:)'+Q;
% Kalman gain
K=predSigma*H(2,:)'*inv(S);
% Correction
mu=predMu+K*minimizedAngle(z-zhat);
mu(3)=minimizedAngle(mu(3));
I=eye(length(mu));
Sigma=(I-K*H(2,:))*predSigma*(I-K*H(2,:))'+K*Q*K'; %Joseph form

