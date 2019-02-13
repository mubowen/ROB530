%generateScript: simulates the trajectory of the robot using square
%                path given by generate motion
%
%data=generateScript(initialstatemean,numSteps,alphas,betas)
%     generates data of the form
%     [realObservation', noisefreeMotion', noisefreeObservation',
%     realRobot', noisefreeRobot']
%
%realObservation and noisefreeMotion is the only data available to
%filter.  All other data for debugging/display purposes.
%
%alphas are the 4-d noise for robot motion
%beta: noise for observations
%
%right now, observation ids not based on relationship between robot and marker
%
% TODO: update generateScript so that observations are for the
% closest marker

function data = generateScript(initialStateMean, numSteps, alphas, ...
                               beta, deltaT)

%--------------------------------------------------------------
% Initializations
%--------------------------------------------------------------

motionDim = 3;
observationDim = 2; % observation size (bearing, marker ID)

realRobot = initialStateMean;
noisefreeRobot = initialStateMean;

% soccer field
global FIELDINFO;
FIELDINFO = getfieldinfo();

data = zeros(numSteps, 13);
for n = 1:numSteps
    % --------------------------------------------
    % Simulate motion
    % --------------------------------------------

    t=n*deltaT;
    noisefreeMotion = generateMotion(t,deltaT);

    % Noise-free robot
    prevNoisefreeRobot = noisefreeRobot;
    noisefreeRobot = sampleOdometry(noisefreeMotion, noisefreeRobot, [0 0 0 0]);


    % Move robot
    realRobot = sampleOdometry(noisefreeMotion, realRobot, alphas);

    %--------------------------------------------------------------
    % Simulate observation
    %--------------------------------------------------------------

    % n / 2 causes each landmark to be viewed twice
    markerId = mod(floor(n / 2), FIELDINFO.NUM_MARKERS) + 1;

    noisefreeObservation = observation(realRobot, markerId);

    % Observation noise
    Q = [beta^2     0;
         0          0];

    observationNoise = sample(zeros(observationDim,1), Q);
    realObservation = noisefreeObservation + observationNoise;


    data(n,:) = [realObservation', noisefreeMotion', noisefreeObservation', realRobot', noisefreeRobot'];
end
