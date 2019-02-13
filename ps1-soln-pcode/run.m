% RUN  PS2 Feature-Based Localization Simulator
%   RUN(ARG)
%   RUN(ARG, PAUSLEN)
%   RUN(ARG,PAUSELEN,FILTER)
%      ARG - is either the number of time steps, (e.g. 100 is a complete
%            circuit) or a data array from a previous run.
%      PAUSELEN - set to `inf`, to manually pause, o/w # of seconds to wait
%                 (e.g., 0.3 is the default)
%      FILTER - one or more of {'ekf','ukf','pf'} as a cell array
%               'pfcov' and 'pfsamp' show covariance statistics or samples
%               only from the particle filter; 'pf' shows both.
% 
%   DATA = RUN(ARG,PAUSELEN,FILTER, OPTIONS)
%      OPTIONS may be:
%      'openLoop' or {'openloop', 0 or 1}
%      'globalLocalization' or {'globalLocalization, 0 or 1}
%      'pfTrajectory' or {'pfTrajectory', 0 or 1} PF trajectory evolution
%      {'pmNoiseFactor', scalar} process noise scale factor
%      {'omNoiseFactor', scalar} observation noise scale factor
%      {'nParticles', scalar} number of particles in PF
%      {'nSigma', scalar} e.g, 1, 2, or 3 sigma covariance plots
% 
%      DATA - is an optional output and contains the data array generated
%             and/or used during the simulation.
run(100);