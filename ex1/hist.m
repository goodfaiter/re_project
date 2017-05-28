clear all;
clc;
close all;

simConst = SimulationConstants();

% Define the constants accessible to the estimator.
estConst = EstimatorConstants();
estConst.VelocityInputPSD = 1;

% Generate plots by default.
doplot=false;

% use random seed
seed = 0;
   
for i = 1:50
    e(i) = run(simConst, estConst, doplot, seed);
end
edges = [0:0.1:3];
histogram(e,edges);
strmax = ['Histogram of 50 simulation with Qv = ',num2str(estConst.VelocityInputPSD)];
title(strmax)
xlabel('Error [m]') % x-axis label
ylabel('Number of Simulations') % y-axis label
strmax1 = ['Mean = ',num2str(mean(e))];
text(2,4,strmax1);
strmax2 = ['Var = ',num2str(var(e))];
text(2,3.5,strmax2);
mean(e)
var(e)
