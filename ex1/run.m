function trackErrorNorm=run(simConst,estConst,doplot,seed)
% run()
%
% Main function for Extended Kalman Filter programming exercise.
% It is used to simulate the true model, call the estimator and show the 
% results.
%
% Inputs:
%   simConst    constants used for the simulation (as in SimulationConstants.m)
%   estConst    estimator constants (as in EstimatorConstants.m), constants
%               used by the estimator
%   doplot      if doplot=true      plots are generated
%               if doplot=false     no plots are generated
%   seed        seed for the random number generator for reproducable results, 
%               if seed = 0 no seed is spedicfied
%
% Output:
%   trackErrorNorm  the tracking error e, as defined in the exercise sheet
%
% You can run the function simply with "run()" if you want to use the
% default parameters as provided by EstimatorConstants.m and
% SimulationConstants.m, and generate the plots.
%
%
%
% Class:
% Recursive Estimation
% Spring 2017
% Programming Exercise 1
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Raffaello D'Andrea, Michael Muehlebach, Lukas Hewing
% michaemu@ethz.ch
% lhewing@ethz.ch
%
% --
% Revision history
% [15.04.11, ST]    first version by Sebastian Trimpe
% [30.04.12, PR]    2012 version, added unknown wheel radius
% [06.05.13, MH]    2013 version
% [24.04.15, MM]    2015 version
% [14.04.15, MM]    2016 version
% [05.05.17, LH]    2017 version

% clear command window, close figures
clc;
close all;

if nargin==0
   % Define the simulation constants that are used in simulation, but not 
   % accessible to the estimator.  
   simConst = SimulationConstants();
   
   % Define the constants accessible to the estimator.
   estConst = EstimatorConstants();
   
   % Generate plots by default.
   doplot=true;
   
   % use random seed
   seed = 0;
end



%% Setup

% Set the random number generator state.
% Uncomment to make results reproducable. This setting was used to generate
% the plot in the problem description.
if seed ~= 0
    rand('seed',seed);
    randn('seed',seed);
end


%% Simulation
% The function 'Simulator' simulates the robot kinematics and generates
% measurements.
[tm, loc, drift, input, sense] = Simulator( simConst );


%% Run the Estimator
N=simConst.N;

% Initialize the estimator.  
estState = [];
posEst = zeros(N,2);
oriEst = zeros(N,1);
driftEst = zeros(N,1);
posVar = zeros(N,2);
oriVar = zeros(N,1);
driftVar = zeros(N,1);
[posEst(1,:),oriEst(1),driftEst(1),posVar(1,:),oriVar(1),driftVar(1),estState] = ...
    Estimator(estState,zeros(1,2),zeros(1,3),0,estConst);

% Call the estimator for each time step.
for n = 2:N
    [posEst(n,:),oriEst(n),driftEst(n),posVar(n,:),oriVar(n),driftVar(n),estState] = ...
        Estimator(estState,input(n,:),sense(n,:),tm(n),estConst);
end



%% The results
% Plots of the results.

% Calculate the total tracking error.
% Replace the following:
e = 0;
for n = 1:N
   e = e + (loc(n,1)-posEst(n,1))^2 + (loc(n,2)-posEst(n,2))^2;
end
trackErrorNorm = (e/N)^(1/2);

if doplot
    % Add your plots here to debug the estimator and verify your
    % implementation.
    subplot(2,3,1);
    plot(tm, loc(:,1),'b');
    hold on;
    plot(tm, posEst(:,1),'r');
    plot(tm, posEst(:,1)+sqrt(posVar(:,1)),'r--');
    plot(tm, posEst(:,1)-sqrt(posVar(:,1)),'r--');
    title('Estimation of X position')
    xlabel('Time [s]') % x-axis label
    ylabel('Distance in X [m]') % y-axis label
    legend('Ground Truth','Estimation')
    
    subplot(2,3,2);
    plot(tm, loc(:,2),'b');
    hold on;
    plot(tm, posEst(:,2),'r');
    plot(tm, posEst(:,2)+sqrt(posVar(:,2)),'r--');
    plot(tm, posEst(:,2)-sqrt(posVar(:,2)),'r--');
    title('Estimation of Y position')
    xlabel('Time [s]') % x-axis label
    ylabel('Distance in Y [m]') % y-axis label
    legend('Ground Truth','Estimation')
    
    subplot(2,3,3);
    plot(tm, loc(:,3),'b');
    hold on;
    plot(tm, oriEst(:,1),'r');
    plot(tm, oriEst(:,1)+sqrt(oriVar(:,1)),'r--');
    plot(tm, oriEst(:,1)-sqrt(oriVar(:,1)),'r--');
    title('Estimation of orientation')
    xlabel('Time [s]') % x-axis label
    ylabel('Orientation [rad]') % y-axis label
    legend('Ground Truth','Estimation')
    
    subplot(2,3,4);
    plot(tm, drift(:,1),'b');
    hold on;
    plot(tm, driftEst(:,1),'r');
    plot(tm, driftEst(:,1)+sqrt(driftVar(:,1)),'r--');
    plot(tm, driftEst(:,1)-sqrt(driftVar(:,1)),'r--');
    title('Estimation of drift')
    xlabel('Time [s]') % x-axis label
    ylabel('Orientation [rad]') % y-axis label
    legend('Ground Truth','Estimation')
    
    subplot(2,3,5);
    plot(tm, (loc(:,1).^2 + loc(:,2).^2).^(1/2),'b');
    hold on;
    plot(tm, (posEst(:,1).^2 + posEst(:,2).^2).^(1/2),'r');
    title('Estimation of Distance')
    xlabel('Time [s]') % x-axis label
    ylabel('Distance [m]') % y-axis label
    legend('Ground Truth','Estimation')
    
end
    
return;

    