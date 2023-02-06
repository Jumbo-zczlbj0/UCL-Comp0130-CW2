% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef DriveBotSLAMSystem < minislam.slam.SLAMSystem
    
    properties(Access = public, Constant)
        % Platform state dimension
        NP = 3;
        
        % Landmark dimension
        NL = 2;
        
        % Initial cache size; might help a bit with performance
        INITIAL_CACHE_SIZE = 10000;
    end
    
    properties(Access = protected)
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;
        
        % The set of all prediction edges. These are removed from the graph
        % afterwards if we don't use prediction
        processModelEdges;
        numProcessModelEdges;
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarkIDStateVectorMap;
        
        % How often we recommend running the optimization
        recommendOptimizationPeriod;
        
        % Flag to show if we should prune the edges. This is needed for
        % question Q3a
        removePredictionEdgesFromGraph;
        keepFirstPredictionEdge;
        
    end
    
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = DriveBotSLAMSystem(configuration)
            
            % Call the base class constructor
            this = this@minislam.slam.SLAMSystem(configuration);
            
            % Preallocate for convenience
            this.vehicleVertices = cell(1, this.INITIAL_CACHE_SIZE);
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 0;
            
            % The set of prediction edges, initially empty
            this.processModelEdges = cell(1, this.INITIAL_CACHE_SIZE);
            this.numProcessModelEdges = 0;
            
            % Allocate the landmark map
            this.landmarkIDStateVectorMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            % By default, run very infrequently
            this.recommendOptimizationPeriod = inf;
            
            this.removePredictionEdgesFromGraph = false;
            this.keepFirstPredictionEdge = false;
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. The logic we have here just recommends
        % an optimization if a fixed number of steps have been completed.
        
        function recommendation = recommendOptimization(this)
            
            % This is how to do it after every 100 steps
            recommendation = rem(this.stepNumber, ...
                this.recommendOptimizationPeriod) == 0;
        end
        
        % Set the value of how often recommend optimization should return
        % true
        function setRecommendOptimizationPeriod(this, newRecommendOptimizationPeriod)
            this.recommendOptimizationPeriod = newRecommendOptimizationPeriod;
        end
        
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = platformEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
        end
        
        % Returns the entire history of the platform estimates. Suppose
        % there are n vehicle vertices. T is a 1 by N dimensional vector of
        % timesteps. X is a 3 by N dimensional vector of vehicle state (x,
        % y, theta). P is a 3 by N dimensional vector where the nth column
        % are the diagonals from the covariance matrix.
        function [T, X, P] = platformEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            
            % Create the output array
            X = zeros(this.NP, this.vehicleVertexId);
            P = zeros(this.NP, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this)
            
            landmarkVertices = values(this.landmarkIDStateVectorMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(this.NL, numberOfLandmarks);
            P = NaN(this.NL, this.NL, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
        
        % We overload the optimize method so that you can add additional
        % logic here
        function chi2 = optimize(this, maximumNumberOfOptimizationSteps)
            
%             Remove the prediction edges if requested.
            if (this.removePredictionEdgesFromGraph == true)
                this.deleteVehiclePredictionEdges();
            end

            % Now call the actual optimizer. Let it handle the default if
            % no steps are specified.
            this.graph.initializeOptimization(this.validateGraphOnInitialization);
            if (nargin > 1)
                chi2 = optimize@minislam.slam.SLAMSystem(this, ...
                    maximumNumberOfOptimizationSteps);
            else
                chi2 = optimize@minislam.slam.SLAMSystem(this);
            end
%             chi2 = 0;
        end
        
        function setRemovePredictionEdges(this, removeEdges, keepFirst)
            this.removePredictionEdgesFromGraph = removeEdges;
            this.keepFirstPredictionEdge = keepFirst;
            
        end
    end
    
    % These are the methods you will need to overload
    methods(Access = protected)
        
        % Handle the initial condition
        
        function handleInitialConditionEvent(this, event)
            
            % Create the first vertex, set its estimate to the initial
            % value and add it to the graph.
            this.currentVehicleVertex = drivebot.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.graph.addVertex(this.currentVehicleVertex);
            
            % Set the book keeping for this initial vertex.
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
            
            % If the covariance is 0, the vertex is known perfectly and so
            % we set it as fixed. If the covariance is non-zero, add a
            % unary initial prior condition edge instead. This adds a soft
            % constraint on where the state can be.
            if (det(event.covariance) < 1e-6)
                this.currentVehicleVertex.setFixed(true);
            else
                initialPriorEdge = drivebot.InitialPriorEdge();
                initialPriorEdge.setMeasurement(event.data);
                initialPriorEdge.setInformation(pinv(event.covariance));
                initialPriorEdge.setVertex(this.currentVehicleVertex);
                this.graph.addEdge(initialPriorEdge);
            end
        end
        
        function handleNoPrediction(~)
            % Nothing to do
        end
        
        function handleHeartbeatEvent(this, ~)
            % Nothing to do
        end
        
        function handlePredictToTime(this, time, dT)
            
            % Create the next vehicle vertex and add it to the graph
            
            this.currentVehicleVertex = drivebot.VehicleStateVertex(time);
            this.graph.addVertex(this.currentVehicleVertex);
            
            % Q1b:
            % Implement prediction code here
            k = this.vehicleVertexId; %% previous vertex in graph
            Q = this.uCov;
            omegaQ = pinv(Q); 
            odometry=this.u;         
            processModelEdge = drivebot.VehicleKinematicsEdge(dT);
            processModelEdge.setVertex(1, this.vehicleVertices{k});
            processModelEdge.setVertex(2, this.currentVehicleVertex);
            processModelEdge.setMeasurement(odometry);
            processModelEdge.setInformation(omegaQ);
            this.graph.addEdge(processModelEdge);

            priorX = this.vehicleVertices{k}.estimate(); % [xk, yk, psi_k]
            c = cos(priorX(3));
            s = sin(priorX(3));

            M = [c, -s, 0;... % Rotation matrix
                s, c, 0;...
                0, 0, 1];

            predictedX = priorX;
            predictedX = predictedX + dT * M * (odometry); % Process model
            predictedX(3) = g2o.stuff.normalize_theta(predictedX(3));   % Wrap the heading to -pi to pi
            this.currentVehicleVertex.setEstimate(predictedX);
            
%             warning('drivebotslam:handlepredicttotime:unimplemented', ...
%                 'Implement the rest of this method for Q1b.');
            
            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)
            % Create a GPS measurement edge
            % Q1c:
            % Implement prediction code here
            k = this.vehicleVertexId;
            e = drivebot.GPSMeasurementEdge();
    
            % Link it so that it connects to the vertex we want to estimate
            e.setVertex(1, this.vehicleVertices{k});

            % Set the measurement value and the measurement covariance
            omegaR = pinv(event.covariance);
            e.setMeasurement(event.data);
            e.setInformation(omegaR); %(event.covariance);

            % Add the edge to the graph
            this.graph.addEdge(e);
%             warning('drivebotslam:handlegpsobservationevent:unimplemented', ...
%                 'Implement the rest of this method for Q1c.');
       
        end
        
        function handleLandmarkObservationEvent(this, event)
            
            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);
                
                % Add the measurement edge
                e = drivebot.LandmarkRangeBearingEdge();
                e.setVertex(1, this.currentVehicleVertex);
                e.setVertex(2, landmarkVertex);
                e.setMeasurement(z);
                e.setInformation(pinv(event.covariance));
%                 this.graph.addEdge(e); 
                % Finish to implement Q2b
                
%                 warning('drivebotslamsystem:handlelandmarkobservationevent:unimplemented', ...
%                     'Implement the rest of this method for Q2b.');
                
                
                if (newVertexCreated == true)
                    e.initialize();
                end
                
                this.graph.addEdge(e);
            end
        end
        
        function deleteVehiclePredictionEdges(this)
            % Q3a:
%             if (this.keepFirstPredictionEdge == true)
                % We protect the first prediction edge
%             end
            disp('number of initial vertices and edges:')
            disp(length(this.graph.vertices()));
            disp(length(this.graph.edges()));
            
            EdgeArray = this.graph.edges();
            NumberOfEdges = length(EdgeArray);
            VehicleEdgeCount = 0;
            for i = 1:NumberOfEdges
                edge = EdgeArray{i};
                if class(edge) == "drivebot.VehicleKinematicsEdge"
                    VehicleEdgeCount = VehicleEdgeCount + 1;
                    if (VehicleEdgeCount == 1 && this.keepFirstPredictionEdge == true)
                        disp("First Edge kept");
                    else
                        this.graph.removeEdge(edge);
%                         disp(["#", num2str(VehicleEdgeCount), " Vehicle Edge Removed"]);
                    end
                end
            end
            % Process of Deleting prediction edges
%             disp(this.vehicleVertexId) % Step size = setRecommendOptimizationPeriod
%             NumberOfDeteledEdges = 0;
% 
%             for i = 1:this.vehicleVertexId % could be improved 
%                 vehicleVertex = this.vehicleVertices{i};
%                 EdgesConnectedToVehicleVertex = vehicleVertex.edges();
%                 NumberOfEdges = length(EdgesConnectedToVehicleVertex);
%                 
% %                 NumberOfEdgesWithLandmark = 0;
% %           For ith vehicle vertex
%                 for j = 1:NumberOfEdges
%                     SpecificEdge = EdgesConnectedToVehicleVertex{j};
%                     if class(SpecificEdge) == "drivebot.VehicleKinematicsEdge"
%                         % Detele the edge
%                         this.graph.removeEdge(SpecificEdge);
%                         NumberOfDeteledEdges = NumberOfDeteledEdges + 1;
% %                         NumberOfEdgesWithLandmark = NumberOfEdgesWithLandmark + 1; % For ith vehicleVertex, it has NumberOfEdgesWithLandmark edges connected to landmarks
%                     end
%                 end
%                 
%                 if (isempty(EdgesConnectedToVehicleVertex))
%                     this.graph.removeVertex(vehicleVertex)
%                 end
                
                
            
            
            % After Delete
%             disp("Number of Deleted Edges:")
%             disp(NumberOfDeteledEdges)

            disp("number of remaining vertices and edges:")
            disp(length(this.graph.vertices()));
            disp(length(this.graph.edges()));
%             warning('drivebotslam:deletevehiclepredictionedges:unimplemented', ...
%                 'Implement the rest of this method for Q3a.');
        end
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)
            
            % If the landmark exists already, return it
            if (isKey(this.landmarkIDStateVectorMap, landmarkId) == true)
                landmarkVertex = this.landmarkIDStateVectorMap(landmarkId);
                newVertexCreated = false;
                return
            end
            
            fprintf('Creating landmark %d\n', landmarkId);
            
            % Create the new landmark add it to the graph
            landmarkVertex = drivebot.LandmarkStateVertex(landmarkId);
            this.landmarkIDStateVectorMap(landmarkId) = landmarkVertex;
            
            this.graph.addVertex(landmarkVertex);
            
            newVertexCreated = true;
        end
        
        function storeStepResults(this)
            % Nothing
        end
        
    end
end
