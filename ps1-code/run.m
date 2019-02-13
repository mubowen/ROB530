function varargout = run(stepsOrData, pauseLen, makeVideo)
% RUN  PS2 Feature-Based Localization Simulator
%   RUN(ARG)
%   RUN(ARG, PAUSLEN, MAKEVIDEO)
%      ARG - is either the number of time steps, (e.g. 100 is a complete
%            circuit) or a data array from a previous run.
%      PAUSELEN - set to `inf`, to manually pause, o/w # of seconds to wait
%                 (e.g., 0.3 is the default)
%      MAKEVIDEO - boolean specifying whether to record a video or not
%
%   DATA = RUN(ARG,PAUSELEN)
%      DATA - is an optional output and contains the data array generated
%             and/or used during the simulation.

%   (c) 2009-2015
%   Ryan M. Eustice
%   University of Michigan
%   eustice@umich.edu

if ~exist('pauseLen','var') || isempty(pauseLen)
    pauseLen = 0.3; % seconds
end
if ~exist('makeVideo','var') || isempty(makeVideo)
    makeVideo = false;
end

%--------------------------------------------------------------
% Graphics
%--------------------------------------------------------------

NOISEFREE_PATH_COL = 'green';
ACTUAL_PATH_COL = 'blue';

NOISEFREE_BEARING_COLOR = 'cyan';
OBSERVED_BEARING_COLOR = 'red';

GLOBAL_FIGURE = 1;

if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', min(5, 1/pauseLen));
        open(vo);
    end
end

%--------------------------------------------------------------
% Initializations
%--------------------------------------------------------------

initialStateMean = [180 50 0]'; %[X,Y,sita]

% Motion noise (in odometry space, see Table 5.5, p.134 in book).
alphas = [0.05 0.001 0.05 0.01].^2; % variance of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
beta = deg2rad(20);

% Step size between filter updates, can be less than 1.
deltaT=0.1;

persistent data numSteps;
if isempty(stepsOrData) % use dataset from last time
    if isempty(data)
        numSteps = 100;
        data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
    end
elseif isscalar(stepsOrData)
    % Generate a dataset of motion and sensor info consistent with
    % noise models.
    numSteps = stepsOrData;
    data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
else
    % use a user supplied dataset from a previous run
    data = stepsOrData;
    numSteps = size(data, 1);
    global FIELDINFO;
    FIELDINFO = getfieldinfo;
end


% TODO: provide proper initialization for your filters here
% You can set the initial mean and variance of the EKF to the true mean and
% some uncertainty.

mu=zeros(3,length(numSteps));  % mean of the state
mu(:,1)=initialStateMean;
Sigma=eye(3) ;% variance of the state
Sigma_pf=zeros(3);


predMu=zeros(3,length(numSteps));
predMu(:,1)=initialStateMean;  % predict mean
Q=1*beta^2;              %Measure covariance Observation noise
% Q=0
predSigma=0;
zhat=zeros(length(numSteps));

%partial filter
numSamples=300;
L_init=chol(Sigma,'lower');
A=initialStateMean;
B=ones(1,numSamples);
samples=kron(A,B);
%samples(3,:)=minimizedAngle(samples(3,:))
weight_pf=B/numSamples;

%Bonus kidnapped robot
% location_x=300;
% location_y=300;
% % mu(:,1) = rand(3,1).*[location_x;location_y;2*pi] + [0;0;-pi];
% %for pf
% samples = rand(3,numSamples).*[location_x;location_y;2*pi] + [0;0;-pi];

%plot parameter
Sigma_p = zeros(3,3,numSteps);



% Call ekfUpdate, ukfUpdate and pfUpdate in every iteration of this loop.
% You might consider putting in a switch yard so you can select which
% algorithm does the update
results = [];

for t = 1:numSteps

    %=================================================
    % data available to your filter at this time step
    %=================================================
    motionCommand = data(t,3:5)'; % [drot1, dtrans, drot2]' noisefree control command
    observation = data(t,1:2)';   % [bearing, landmark_id]' noisy observation

    %=================================================
    % data *not* available to your filter, i.e., known
    % only by the simulator, useful for making error plots
    %=================================================
    % actual position (i.e., ground truth)
    x = data(t,8);
    y = data(t,9);
    theta = data(t,10);

    % noisefree observation
    noisefreeBearing = data(t, 6);

    %=================================================
    % graphics
    %=================================================
    figure(GLOBAL_FIGURE); clf; hold on; plotfield(observation(2));

    % draw actual path and path that would result if there was no noise in
    % executing the motion command
    plot([initialStateMean(1) data(1,8)], [initialStateMean(2) data(1,9)], 'Color', ACTUAL_PATH_COL);
    plot([initialStateMean(1) data(1,11)], [initialStateMean(2) data(1,12)], 'Color', NOISEFREE_PATH_COL);

    % draw actual path (i.e., ground truth)
    plot(data(1:t,8), data(1:t,9), 'Color', ACTUAL_PATH_COL);
    plotrobot( x, y, theta, 'black', 1, 'cyan');

    % draw noise free motion command path
    plot(data(1:t,11), data(1:t,12), 'Color', NOISEFREE_PATH_COL);
    plot(data(t,11), data(t,12), '*', 'Color', NOISEFREE_PATH_COL);

    % indicate observed angle relative to actual position
    plot([x x+cos(theta+observation(1))*100], [y y+sin(theta+observation(1))*100], 'Color', OBSERVED_BEARING_COLOR);

    % indicate ideal noise-free angle relative to actual position
    plot([x x+cos(theta+noisefreeBearing)*100], [y y+sin(theta+noisefreeBearing)*100], 'Color', NOISEFREE_BEARING_COLOR);

    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================

    markerId=observation(2);
    z=observation(1);
    u=motionCommand;
    M=[alphas(1)*abs(motionCommand(1)^2)+alphas(2)*abs(motionCommand(2)^2),0,0;...
        0,alphas(3)*abs(motionCommand(2)^2)+alphas(4)*abs(motionCommand(1)^2+motionCommand(3)^2),0;
        0,0,alphas(1)*abs(motionCommand(3)^2)+alphas(2)*abs(motionCommand(2)^2)];
     %M=M/100;
     %choose the filter to run 0 for EKF,1 for UKF,2 for PF
    filter =2;
    switch filter 
        case 0
            [mu(:,t+1), Sigma, predMu(:,t+1), predSigma, zhat(t+1), ~, ~, ~, ~ ] = ekfUpdate( ...
        mu(:,t), Sigma, u, deltaT, M, z, Q, markerId);
           Sigma_p(:,:,t+1) = Sigma;
           %ekf   
 
            plotcov2d(mu(1,t+1), mu(2,t+1), Sigma(1:2,1:2), 'r', false, 'r', 1 , 3);
  plotmarker(mu(1:2,t+1), 'r');

        case 1
            [mu(:,t+1), Sigma, predMu(:,t+1), predSigma, zhat(t+1) ] = ukfUpdate( ...
        mu(:,t), Sigma, u, M, z, Q, markerId);
            Sigma_p(:,:,t+1) = Sigma;
            %  %ukf
              plotcov2d(mu(1,t+1), mu(2,t+1), Sigma(1:2,1:2), 'r', false, 'r', 1 , 3);
  plotmarker(mu(1:2,t+1), 'r');
        case 2
             [samples, weight_pf, mu(:,t+1),Sigma, zHat] = pfUpdate( ...
     samples, weight_pf, numSamples, u, alphas, z, Q, markerId);
           Sigma_p(:,:,t+1) = Sigma;
           %   pf
             plotSamples(samples)
             red = [0, 0.5, 0];
%         plotcov2d(mu(1,t+1), mu(2,t+1), predSigma(1:2,1:2), 'r' , false, 'r', 1 , 3);
%         plotmarker(mu(1:2,t+1), red);
    end
  
 
        
    drawnow;
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end

    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
end
  
    %=================================================
    %TODO: plot and evaluate filter results here
    %=================================================
 
%         
%-------------------------------------------------------
  
   x=data(:,8)';
   y=data(:,9)';
   theta=data(:,10)';
  
   x_error=mu(1,2:end)-x;
   y_error=mu(2,2:end)-y;
   theta_error=mu(3,2:end)-theta;
   for k=1:length(theta_error)
       theta_error(k)=minimizedAngle(theta_error(k));
   end
   x_sigma=zeros(size(x_error));
   y_sigma=zeros(size(y_error));
   theta_sigma=zeros(size(theta_error));
   
   for i=2:size(mu,2)
        x_sigma(i)=sqrt(Sigma_p(1,1,i));
        y_sigma(i)=sqrt(Sigma_p(2,2,i));
        theta_sigma(i)=sqrt(Sigma_p(3,3,i));
   end
   fig=1;
   figure(fig)
   
   subplot(3,1,1)
   plot(x_error,'b')
   hold on
   plot(3*x_sigma,'r')
   plot(-3*x_sigma,'r')
   title('Pose error vs. time for pf kidnapped robot')
   ylabel('x (cm)')
   hold off
 
     subplot(3,1,2)
     plot(y_error,'b')
   hold on
   plot(3*y_sigma,'r')
   plot(-3*y_sigma,'r')
   hold off
   ylabel('y (cm)')
 
   subplot(3,1,3)
   plot(theta_error,'b')
   hold on
   plot(3*theta_sigma,'r')
   plot(-3*theta_sigma,'r')
   hold off
   ylabel('theta (rad)')
   xlabel('time (s)')
   
%------------------------------------

if nargout >= 1
    varargout{1} = data;
end
if nargout >= 2
    varargout{2} = results;
    
 end
% if nargout >= 3
%     varargout{3} = Sigma;
% end
if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end
