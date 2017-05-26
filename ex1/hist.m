simConst = SimulationConstants();

% Define the constants accessible to the estimator.
estConst = EstimatorConstants();

% Generate plots by default.
doplot=false;

% use random seed
seed = 0;
   
for i = 1:50
    e(i) = run(simConst, estConst, doplot, seed);
end
edges = [0:0.1:3];
histogram(e,edges);
