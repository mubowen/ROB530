function demo(arg)

load task3;
%sod = data;
sod = 100;
plen = 0.0;

global results;

switch arg
    %======OPEN LOOP===================================================
  case 0, disp('PF Trajectory Open Loop');
    [data,results] = run(sod,plen,'pf', ...
                         {'nParticles',1000},{'pmNoiseFactor',1},{'openLoop',1});

  case 1, disp('PF Trajectory Open Loop Large PM Noise');
    [data,results] = run(sod,plen,'pf', ...
                         {'nParticles',1000},{'pmNoiseFactor',10},{'openLoop',1});

  case 2, disp('PF, UKF, EKF Open Loop');
    [data, results] = run(sod,plen,{'ekf','ukf','pf'}, ...
                          {'nParticles',1000},{'pmNoiseFactor',1},{'openLoop',1});

  case 3, disp('PF, UKF, EKF Open Loop Large PM Noise');
    [data, results] = run(sod,plen,{'ekf','ukf','pf'}, ...
                          {'nParticles',1000},{'pmNoiseFactor',10},{'openLoop',1});

  case 4, disp('PF, UKF, EKF Open Loop Huge PM Noise');
    [data, results] = run(sod,plen,{'ekf','ukf','pf'}, ...
                          {'nParticles',1000},{'pmNoiseFactor',100},{'openLoop',1});

    %======CLOSED LOOP=================================================
  case 10, disp('PF Trajectory');
    [data,results] = run(sod,plen,'pf', ...
                         {'nParticles',1000},{'pmNoiseFactor',1},{'openLoop',0});

  case 11, disp('PF Trajectory Large PM Noise');
    [data,results] = run(sod,plen,'pf', ...
                         {'nParticles',1000},{'pmNoiseFactor',10},{'openLoop',0});

  case 12, disp('PF, UKF, EKF');
    [data, results] = run(sod,plen,{'ekf','ukf','pf'}, ...
                          {'nParticles',1000},{'pmNoiseFactor',1},{'openLoop',0});

  case 13, disp('PF, UKF, EKF Large PM Noise');
    [data, results] = run(sod,plen,{'ekf','ukf','pf'}, ...
                          {'nParticles',1000},{'pmNoiseFactor',10},{'openLoop',0});

  case 14, disp('PF, UKF, EKF Huge PM Noise');
    [data, results] = run(sod,plen,{'ekf','ukf','pf'}, ...
                          {'nParticles',1000},{'pmNoiseFactor',100},{'openLoop',0});

  case 15, disp('EKF');
    [data, results] = run(sod,plen,{'ekf'});

    %=======PF TRAJ DEPLETION==========================================
  case 20, disp('PF Trajectory Good');
    [data, results] = run(sod,plen,'pfsamp', ...
                          {'nParticles',50},{'pmNoiseFactor',1},{'pfTrajectory',1});

  case 21, disp('PF Trajectory Depletion');
    [data, results] = run(sod,plen,'pfsamp', ...
                          {'nParticles',50},{'pmNoiseFactor',100},{'pfTrajectory',1});


    %======GLOBAL LOCALIZATION=================================================
  case 30, disp('PF Global Localization');
    [data,results] = run(sod,plen,'pfsamp', ...
                         {'nParticles',1000},{'pmNoiseFactor',1},{'globalLocalization',1});

  case 32, disp('PF Global Localization (Trajectory)');
    [data,results] = run(90,plen,'pfsamp', ...
                         {'nParticles',1000},{'pfTrajectory',1},{'globalLocalization',1});


  case 33, disp('PF Global Localization Large PM Noise');
    [data,results] = run(sod,plen,'pfsamp', ...
                         {'nParticles',1000},{'pmNoiseFactor',10},{'globalLocalization',1});

  case 34, disp('PF Global Localization Large OM Noise');
    [data,results] = run(sod,plen,'pfsamp', ...
                         {'nParticles',1000},{'omNoiseFactor',10},{'globalLocalization',1});

  otherwise
    error('unrecognized case');
end

%figure(2);
%analyze(results);

%==========================================================================
function R = myinput(str)

r = input([str, '[Y] '],'s');

if isempty(r)
    R = true;
elseif lower(r(1)) == 'n'
    R = false;
else
    R = true;
end

