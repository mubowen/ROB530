function [mu, Sigma, predMu, predSigma, zhat] = ukfUpdate( ...
    mu, Sigma, u,  M, z, Q, markerId)

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
% z_hat,     prediction measurement mean

% --------------------------------------------
% Setup UKF
% --------------------------------------------

% UKF params
a=1; %sigma point scaling
k=2; %scalar tuning parameter
n=length(mu(:,1));
lamda=a^2*(n+k)-n;

% Augmented state

% Sigma points
gamma=sqrt(n+lamda);
%Sigma_a = blkdiag(Sigma, M, Q)
L=chol(Sigma,'lower');
x_pre=[mu, mu(:,ones(1,n))+gamma*L,mu(:,ones(1,n))-gamma*L];
% Weights
b=5;
w_m=lamda/(n+lamda);
w_c1=w_m+(1-a^2+b);
w=zeros(2*n+1,1);
w_c=zeros(2*n+1,1);
w(1:1)=w_m;
w(2:end)=1/(2*(n+lamda));
w_c(1:1)=w_c1;
w_c(2:end)=1/(2*(n+lamda));
% --------------------------------------------
% Prediction step
% --------------------------------------------

% UKF prediction of mean and covariance
x_bar=prediction(x_pre,u);
predMu=x_bar*w;
predSigma=(x_bar-predMu)*diag(w_c)*(x_bar-predMu)'+M;
%--------------------------------------------------------------
% Correction step
%--------------------------------------------------------------
z_hat=zeros(1,length(x_pre(1,:)));
for j=1:length(x_bar(1,:))
   obs=observation(x_bar(:,j),markerId);
    Z=obs(1);
    z_hat(j)=Z;
end
zhat=minimizedAngle(z_hat*w);
Z=minimizedAngle(z_hat);
% UKF correction of mean and covariance
%S=(Z-zhat)'*diag(w)*(Z-zhat)+Q;
S=minimizedAngle((Z-zhat))*diag(w_c)*minimizedAngle(Z-zhat)'+Q;
% x_r=x_bar-predMu;
% zl=Z-zhat;
% for i=1:(2*n+1);
%   Cov_xz(:,i)=w(i)*x_r(:,i)*zl(i);
% end
Cov_xz=(x_bar-predMu)*diag(w_c)*minimizedAngle(Z-zhat)';
K=Cov_xz*inv(S);
mu=predMu+K*minimizedAngle(z-zhat);
mu(3)=minimizedAngle(mu(3));
Sigma=predSigma-K*S*K';



