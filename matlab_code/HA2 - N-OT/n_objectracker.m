classdef n_objectracker
    %N_OBJECTRACKER is a class containing functions to track n object in
    %clutter. 
    %Model structures need to be called:
    %sensormodel: a structure specifies the sensor parameters
    %           P_D: object detection probability --- scalar
    %           lambda_c: average number of clutter measurements per time
    %           scan, Poisson distributed --- scalar 
    %           pdf_c: clutter (Poisson) intensity --- scalar
    %           intensity_c: clutter (Poisson) intensity --- scalar
    %motionmodel: a structure specifies the motion model parameters
    %           d: object state dimension --- scalar
    %           F: function handle return transition/Jacobian matrix
    %           f: function handle return predicted object state
    %           Q: motion noise covariance matrix
    %measmodel: a structure specifies the measurement model parameters
    %           d: measurement dimension --- scalar
    %           H: function handle return transition/Jacobian matrix
    %           h: function handle return the observation of the object
    %           state 
    %           R: measurement noise covariance matrix
    
    properties
        gating      %specify gating parameter
        reduction   %specify hypothesis reduction parameter
        density     %density class handle
    end
    
    methods
        
        function obj = initialize(obj,density_class_handle,P_G,m_d,w_min,merging_threshold,M)
            %INITIATOR initializes n_objectracker class
            %INPUT: density_class_handle: density class handle
            %       P_D: object detection probability
            %       P_G: gating size in decimal --- scalar
            %       m_d: measurement dimension --- scalar
            %       wmin: allowed minimum hypothesis weight --- scalar
            %       merging_threshold: merging threshold --- scalar
            %       M: allowed maximum number of hypotheses --- scalar
            %OUTPUT:  obj.density: density class handle
            %         obj.gating.P_G: gating size in decimal --- scalar
            %         obj.gating.size: gating size --- scalar
            %         obj.reduction.w_min: allowed minimum hypothesis
            %         weight in logarithmic scale --- scalar 
            %         obj.reduction.merging_threshold: merging threshold
            %         --- scalar 
            %         obj.reduction.M: allowed maximum number of hypotheses
            %         used in TOMHT --- scalar 
            obj.density = density_class_handle;
            obj.gating.P_G = P_G;
            obj.gating.size = chi2inv(obj.gating.P_G,m_d);
            obj.reduction.w_min = log(w_min);
            obj.reduction.merging_threshold = merging_threshold;
            obj.reduction.M = M;
        end
        
        function estimates = GNNfilter(obj, states, Z, sensormodel, motionmodel, measmodel)
            %GNNFILTER tracks n object using global nearest neighbor
            %association 
            %INPUT: obj: an instantiation of n_objectracker class
            %       states: structure array of size (1, number of objects)
            %       with two fields: 
            %                x: object initial state mean --- (object state
            %                dimension) x 1 vector 
            %                P: object initial state covariance --- (object
            %                state dimension) x (object state dimension)
            %                matrix  
            %       Z: cell array of size (total tracking time, 1), each
            %       cell stores measurements of size (measurement
            %       dimension) x (number of measurements at corresponding
            %       time step)  
            %OUTPUT:estimates: cell array of size (total tracking time, 1),
            %       each cell stores estimated object state of size (object
            %       state dimension) x (number of objects)
            
            %Calculate total tracking time
            K = size(Z,1);
            
            %Initialize estimates cell array
            estimates = cell(K,1);
            
            %Number of state dimensions
            n_dim = length(states(1).x);
            
            for i = 1:K
                
                %Calculate number of objects
                n = length(states);
                
                %Calculate number of measurements
                m = size(Z{i},2);
                
                % Create Misdetection cost matrix
                % Dimensions: [number_objects] * [number_objects]
                
                % Set non-diagonal elements to infinity and diagonal to the
                % likelihood of misdetection
                misdetect_cost = Inf * ones(n,n);
                diag_index = eye(size(misdetect_cost)) == 1;
                misdetect_cost(diag_index) = -log(1 - sensormodel.P_D);
                
                % Create Detection cost matrix
                % Dimensions: [number_objects] * [number_measurements]
                detect_cost = Inf * ones(n,m);
                
                for o = 1:n % For each object
                    %Perform Gating
                    [~,ind] = obj.density.ellipsoidalGating(states(o), Z{i}, measmodel, obj.gating.size);
                    z_ingate = Z{i}(:,ind);
                    
                    if isempty(z_ingate) == 0
                        %Calculate the predicted likelihood for each measurement in the gate
                        predicted_likelihood = obj.density.predictedLikelihood(states(o),z_ingate,measmodel);
                        %Update detection cost matrix
                        detect_cost(o,ind) = -log(sensormodel.P_D)+log(sensormodel.intensity_c)-predicted_likelihood;
                    end
                end
                
                % Create Cost Matrix
                C = [detect_cost misdetect_cost];
                
                % Solve Assignment Problem using Gibbs Sampling
                numIteration = 10;
                k = 1;
                [col4row,~]= assign2DByGibbs(C,numIteration,k);
                
                %[col4row,~,~] = assign2D(C);
                %[col4row, ~, ~, ~, ~] = assign2DByCol(C,100,1);
                
                % For each object 
                for o = 1:n
                    %Perform Kalman update if it was assigned with a measurement
                    if col4row(o) < m
                        states(o) = obj.density.update(states(o), Z{i}(:,col4row(o)), measmodel);
                    end
                    
                    %Store the state estimation
                    estimates{i}(:,o) = states(o).x;
                    
                    %Perform Kalman prediction 
                    states(o) = obj.density.predict(states(o), motionmodel);
                end                            
            end
        end
        
        function estimates = JPDAfilter(obj, states, Z, sensormodel, motionmodel, measmodel)
            %JPDAFILTER tracks n object using joint probabilistic data
            %association
            %INPUT: obj: an instantiation of n_objectracker class
            %       states: structure array of size (1, number of objects)
            %       with two fields: 
            %                x: object initial state mean --- (object state
            %                dimension) x 1 vector 
            %                P: object initial state covariance --- (object
            %                state dimension) x (object state dimension)
            %                matrix  
            %       Z: cell array of size (total tracking time, 1), each
            %       cell stores measurements of size (measurement
            %       dimension) x (number of measurements at corresponding
            %       time step)  
            %OUTPUT:estimates: cell array of size (total tracking time, 1),
            %       each cell stores estimated object state of size (object
            %       state dimension) x (number of objects)

            %Calculate total tracking time
            K = size(Z,1);

            %Initialize estimates cell array
            estimates = cell(K,1);

            %Number of state dimensions
            n_dim = length(states(1).x); 
            
            for i = 1:K
                
                %Calculate number of objects
                n = length(states);
                
                %Calculate number of measurements
                m = size(Z{i},2);
                
                % Create Misdetection cost matrix
                % Dimensions: [number_objects] * [number_objects]
                
                % Set non-diagonal elements to infinity and diagonal to the
                % likelihood of misdetection
                misdetect_cost = Inf * ones(n,n);
                diag_index = eye(size(misdetect_cost)) == 1;
                misdetect_cost(diag_index) = -log(1 - sensormodel.P_D);
                
                % Create Detection cost matrix
                % Dimensions: [number_objects] * [number_measurements]
                detect_cost = Inf * ones(n,m);
                
                for o = 1:n % For each object
                    %Perform Gating
                    [~,ind] = obj.density.ellipsoidalGating(states(o), Z{i}, measmodel, obj.gating.size);
                    z_ingate = Z{i}(:,ind);
                    
                    if isempty(z_ingate) == 0
                        %Calculate the predicted likelihood for each measurement in the gate
                        predicted_likelihood = obj.density.predictedLikelihood(states(o),z_ingate,measmodel);
                        %Update detection cost matrix
                        detect_cost(o,ind) = -log(sensormodel.P_D)+log(sensormodel.intensity_c)-predicted_likelihood;
                    end
                end
                
                % Create Cost Matrix
                C = [detect_cost misdetect_cost];
                
                % Solve Assignment Problem using Gibbs Sampling
                numIteration = 100;
                k = obj.reduction.M;
                [col4rowBest,gainBest] = assign2DByGibbs(C,numIteration,k);
                %[col4rowBest,row4colBest,gainBest] = kBest2DAssign(C,k);
                
                %Normalise the weights of different data association hypotheses
                w = normalizeLogWeights(-gainBest);
                
                % Prune assignment matrices 
                [w,ind] = hypothesisReduction.prune(w, 1:1:length(w), obj.reduction.w_min);
                
                % Renormalise the weights
                w = normalizeLogWeights(w); 
                
                % Initialize local hypothesis cell
                new_states = cell(n,1);

                for o = 1:n % For all objects
                    for h = 1:length(ind) % For all hypothesis
                        
                        if col4rowBest(o,ind(h)) > m 
                            %Add missing detection hypothesis if not pruned
                            new_states{o} = [new_states{o} states(o)];
                            
                        else
                            % Kalman Update Step
                            new_states{o} = [new_states{o}...
                                obj.density.update(states(o), Z{i}(:,col4rowBest(o,ind(h))), measmodel)];
                            
                        end
                    end
                    
                    % Merge local hypotheses by moment matching
                    states(o) = obj.density.momentMatching(w, new_states{o});
                    
                    % Extract object state estimates
                    estimates{i}(:,o) = states(o).x;
                    
                    % Predict each local hypothesis
                    states(o) = obj.density.predict(states(o), motionmodel);
                    
                end
            end          
        end
        
        function estimates = TOMHT(obj, states, Z, sensormodel, motionmodel, measmodel)
            %TOMHT tracks n object using track-oriented multi-hypothesis tracking
            %INPUT: obj: an instantiation of n_objectracker class
            %       states: structure array of size (1, number of objects)
            %       with two fields: 
            %                x: object initial state mean --- (object state
            %                dimension) x 1 vector 
            %                P: object initial state covariance --- (object
            %                state dimension) x (object state dimension)
            %                matrix  
            %       Z: cell array of size (total tracking time, 1), each
            %       cell stores measurements of size (measurement
            %       dimension) x (number of measurements at corresponding
            %       time step)  
            %OUTPUT:estimates: cell array of size (total tracking time, 1),
            %       each cell stores estimated object state of size (object
            %       state dimension) x (number of objects)
            
            %Calculate total tracking time
            K = size(Z,1);

            %Initialize estimates cell array
            estimates = cell(K,1);

            %Number of state dimensions
            n_dim = length(states(1).x); 
            
            % Number of objects
            n = length(states);
            
            % Initialize global lookup table
            lookup_table = 1:n;
                
            % Initialize weights of global hypotheses
            w = 0;
            
            for t = 1:K
                
                % Number of global hypothesis
                n_global_hyp = size(lookup_table,1);
                
                % Number of measurements
                m = length(Z{t});
                
                % Re-initialize new set of lookup table and weights
                new_lookup_table = {};
                new_w = {};
                
                for h = 1:n_global_hyp %For each global hypothesis    
                    
                    %Set non-diagonal elements to infinity and diagonal to the
                    % likelihood of misdetection
                    misdetect_cost = Inf * ones(n,n);
                    diag_index = eye(size(misdetect_cost)) == 1;
                    misdetect_cost(diag_index) = - log(1 - sensormodel.P_D);

                    % Create Detection cost matrix
                    % Dimensions: [number_objects] * [number_measurements]
                    detect_cost = Inf * ones(n,m);
                    
                    for o = 1:n %For each object   
                        
                        %Perform Gating
                        [~,ind] = obj.density.ellipsoidalGating(states(lookup_table(h,o)), Z{t}, measmodel, obj.gating.size);
                        z_ingate = Z{t}(:,ind);
                        
                        if isempty(z_ingate) == 0                           
                            %Create object detection hypotheses for each detection inside the gate
                            predicted_likelihood = obj.density.predictedLikelihood(states(lookup_table(h,o)), z_ingate, measmodel);
                            
                            %Update detection cost matrix
                            detect_cost(o,ind) = - log(sensormodel.P_D) ...
                                + log(sensormodel.intensity_c)...
                                - predicted_likelihood;                  
                        end                         
                    end
                    
                    % Create Cost Matrix
                    C = [detect_cost misdetect_cost];
                    
                    % Solve Assignment Problem
                    numIteration = 100;
                    k = ceil(exp(w(h))*obj.reduction.M);
                    [col4rowBest,gainBest] = assign2DByGibbs(C,numIteration,k);
                    %[col4rowBest,~,gainBest] = kBest2DAssign(C,k);
                    
                    % Assign all misdetections to zero and update global
                    % lookup table
                    col4rowBest(col4rowBest > m) = 0;
                    new_lookup_table{h} = col4rowBest';
                    
                    % Update weights of global hypotheses 
                    new_w{h} = -gainBest + w(h);
                end
                
                % Normalize hypothesis weights across all global hypothesis
                [new_w,~] = normalizeLogWeights(cell2mat(new_w'));
                
                % Initialize new global hypothesis cell
                new_states = struct('x',[],'P',[]);
                final_lookup_table = [];
                
                s = 1;
                % Kalman Update
                for h = 1:n_global_hyp %For each global hypothesis
                    
                    index_lookup_table = new_lookup_table{h};
                    
                    for o = 1:n
                        
                        indexes = unique(new_lookup_table{h}(:,o));
                        
                        for i = 1:length(indexes) 
                            % Kalman Update
                            if indexes(i) == 0 % Misdetection
                                new_states(s) = states(lookup_table(h,o));
                            else    
                                % Create updated local hypothesis
                                new_states(s) = obj.density.update(states(lookup_table(h,o)), Z{t}(:,indexes(i)), measmodel);
                            end
                            
                            % Re-index lookup table
                            rows = new_lookup_table{h}(:,o) == indexes(i);
                            index_lookup_table(rows,o) = s;
                            
                            s = s + 1;
                        end  
                        
                    end  
                    
                    % Update final lookup table 
                    final_lookup_table = [final_lookup_table;index_lookup_table];
                    
                end
                
                % Prune lookup table 
                [new_w,ind] = hypothesisReduction.prune(new_w, 1:1:length(new_w), obj.reduction.w_min);
                final_lookup_table = final_lookup_table(ind,:);
                
                % Cap lookup table
                [new_w,ind] = hypothesisReduction.cap(new_w, 1:1:length(new_w), obj.reduction.M);
                final_lookup_table = final_lookup_table(ind,:);
                
                % Re-index lookup table
                new_index_states = struct('x',[],'P',[]);
                reindex_lookup_table = final_lookup_table;
                
                [~,~,ic] = unique([new_states.x]','rows');
                for i = 1:max(max(final_lookup_table))
                    reindex_lookup_table(final_lookup_table == i) = ic(i);
                    new_index_states(ic(i)) = new_states(i);
                end
                
                % Re-Normalize hypothesis weights across all global hypothesis
                [final_w,~] = normalizeLogWeights(new_w);
                
                % Extract object estimate 
                [~,bestEstimate] = max(final_w);
                for o = 1:n
                    estimates{t}(:,o) = new_index_states(reindex_lookup_table(bestEstimate,o)).x;
                end
                
                % Predict each local hypothesis
                for h = 1:length(new_index_states)
                    if ~isempty(new_index_states(h).x)
                        new_index_states(h) = obj.density.predict(new_index_states(h), motionmodel);
                    end
                end  
                
                % Update variables for next iteration
                states = new_index_states;
                lookup_table = reindex_lookup_table;
                w = final_w;
                
            end
        end
    end
end