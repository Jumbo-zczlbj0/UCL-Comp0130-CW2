% This script runs Q2(d)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Disable the GPS and enable the laser for pure SLAM.
configuration.enableGPS = false;
configuration.enableLaser = true;

% For this part of the coursework, this should be set to zero.
configuration.perturbWithNoise = false;

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
configuration.maximumStepNumber = 1200; % modify this value from 1200 to 1230

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_d');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(Inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times.
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
hold on

% Plot the error curves.
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
legend('x','y','psi')
hold on

% Plot errors.
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x','y','psi')
hold on

% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();
NumberOfVehiclePoses = 0;
for i = 1:length(allVertices)
    a = isa(allVertices{i}, 'drivebot.VehicleStateVertex');
    NumberOfVehiclePoses = NumberOfVehiclePoses + a;
end % NumberOfVehiclePoses = 100
LandmarksList = [];
for j = 1:length(allVertices)
    if (isa(allVertices{j}, 'drivebot.LandmarkStateVertex'))
        LandmarksList = [LandmarksList, j];
    end
end % LandmarksList = [3  4]
edges4 = edges(allVertices{4});
for k = 1:length(edges(allVertices{4}))
    temp1 = edges4(k);
    temp2 = temp1{1};
    k;
    temp2.Omega;
end % Always same

results{1}.vehicleCovarianceHistory



    

