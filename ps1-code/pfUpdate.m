function [samples, weight, predMu, predSigma, zHat] = pfUpdate( ...
    samples, weight, numSamples, u, alphas, z, Q, markerId)

% NOTE: The header is not set in stone.  You may change it if you like
global FIELDINFO;
landmark_x = FIELDINFO.MARKER_X_POS(markerId);
landmark_y = FIELDINFO.MARKER_Y_POS(markerId);

stateDim=3;
motionDim=3;
observationDim=2;


% ----------------------------------------------------------------
% Prediction step
% ----------------------------------------------------------------

% some stuff goes here
for i=1:numSamples
    %Generate new samples
    samples(:,i)=sampleOdometry(u, samples(:,i), alphas);
    obs=observation(samples(:,i),markerId);
    zHat=obs(1);
    v=minimizedAngle(z-zHat);
    w(i)=mvnpdf(v,0,Q);
end
    % update and normalize weights
    weight = weight.* w; % since we used motion model to sample
    weight = weight/sum(weight);
    % compute effective number of particles
    Neff = 1 / sum(weight.^2);
    % main loop; iterate over the measurements
    if Neff<numSamples/5
       [samples, weight] = resample(samples, weight);
    end
 
    
    
% Compute mean and variance of estimate. Not really needed for inference.
[predMu, predSigma] = meanAndVariance(samples, numSamples);


% ----------------------------------------------------------------
% Correction step
% ----------------------------------------------------------------
