classdef singleobjectracker
    %SINGLEOBJECTRACKER is a class containing functions to track a single
    %object in clutter. 
    %Model structures need to be called:
    %sensormodel: a structure specifies the sensor parameters
    %           P_D: object detection probability --- scalar
    %           lambda_c: average number of clutter measurements per time
    %           scan, Poisson distributed --- scalar 
    %           pdf_c: clutter (Poisson) density --- scalar
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
            %INITIATOR initializes singleobjectracker class
            %INPUT: density_class_handle: density class handle
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
            %         --- scalar 
            
            obj.density = density_class_handle;
            obj.gating.P_G = P_G;
            obj.gating.size = chi2inv(obj.gating.P_G,m_d);
            obj.reduction.w_min = log(w_min);
            obj.reduction.merging_threshold = merging_threshold;
            obj.reduction.M = M;
        end
        
        function estimates = nearestNeighbourFilter(obj, state, Z, sensormodel, motionmodel, measmodel)
            %NEARESTNEIGHBOURFILTER tracks a single object using nearest
            %neighbor association 
            %INPUT: state: a structure with two fields:
            %                x: object initial state mean --- (object state
            %                dimension) x 1 vector 
            %                P: object initial state covariance --- (object
            %                state dimension) x (object state dimension)
            %                matrix  
            %       Z: cell array of size (total tracking time, 1), each
            %       cell stores measurements of  
            %            size (measurement dimension) x (number of
            %            measurements at corresponding time step) 
            %OUTPUT:estimates: cell array of size (total tracking time, 1),
            %       each cell stores estimated object state of size (object
            %       state dimension) x 1 
            
            %Calculate total tracking time
            K = size(Z,1);
            
            %Initialize estimates cell array
            estimates = cell(K,1);
            
            for i = 1:K
                
                %Gating
                [z_ingate,~] = obj.density.ellipsoidalGating(state, Z{i}, measmodel, obj.gating.size);
                
                if isempty(z_ingate) == 0
                    %Calculate the predicted likelihood for each measurement in the gate
                    predicted_likelihood = obj.density.predictedLikelihood(state,z_ingate,measmodel);
                
                    %Find the nearest neighbour measurement
                    [~,index] = max(predicted_likelihood);
            
                    %Compare the weight of the missed detection hypothesis and the 
                    %weight of the object detection hypothesis created using the 
                    %nearest neighbour measurement
                    w_missed = log(1 - sensormodel.P_D);
                    w_detected = log(sensormodel.P_D)-log(sensormodel.intensity_c)+predicted_likelihood(index);
                    
                    %If the object detection hypothesis using the nearest neighbour 
                    %measurement has the highest weight, perform Kalman update
                    if w_detected >= w_missed
                        state = obj.density.update(state, z_ingate(:,index), measmodel);
                    end
                    
                end
                
                %Extract object state estimate
                estimates{i} = state.x;
            
                %Kalman prediction
                state = obj.density.predict(state, motionmodel);

            end

        end
        
        
        function estimates = probDataAssocFilter(obj, state, Z, sensormodel, motionmodel, measmodel)
            %PROBDATAASSOCFILTER tracks a single object using probalistic
            %data association 
            %INPUT: state: a structure with two fields:
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
            %       state dimension) x 1  
            
            %Calculate total tracking time
            K = size(Z,1);
            
            %Initialize estimates cell array
            estimates = cell(K,1);
            
            for i = 1:K
                
                %Gating
                [z_ingate,~] = obj.density.ellipsoidalGating(state, Z{i}, measmodel, obj.gating.size);
                
                %Create missed detection hypothesis
                w_missed = log(1 - sensormodel.P_D);
                
                if isempty(z_ingate) == 0
                    %Create object detection hypotheses for each detection inside the gate
                    predicted_likelihood = obj.density.predictedLikelihood(state,z_ingate,measmodel);
                    w_detected = (log(sensormodel.P_D)-log(sensormodel.intensity_c))+predicted_likelihood;
                    
                    %Normalise hypothesis weights
                    w = [w_missed;w_detected];
                    w = normalizeLogWeights(w);
                    
                    %Prune hypotheses with small weights, and then re-normalise the weights
                    [w,ind] = hypothesisReduction.prune(w,0:1:length(w)-1, obj.reduction.w_min);
                    w = normalizeLogWeights(w);
                    
                    %Kalman Update Step
                    new_states = [];
                    
                    for h = 1:length(ind) % For all hypothesis
                        if ind(h) == 0 %Add missing detection hypothesis if not pruned
                            new_states = [new_states state];
                        else
                            new_states = [new_states obj.density.update(state, z_ingate(:,ind(h)), measmodel)];
                        end
                    end
                    
                    %Perform Moment Matching
                    state = obj.density.momentMatching(w, new_states);
                end
                
                %Extract object state estimate
                estimates{i} = state.x;
            
                %Kalman prediction
                state = obj.density.predict(state, motionmodel);
                
            end

        end
        
        function estimates = GaussianSumFilter(obj, state, Z, sensormodel, motionmodel, measmodel)
            %GAUSSIANSUMFILTER tracks a single object using Gaussian sum
            %filtering
            %INPUT: state: a structure with two fields:
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
            %       state dimension) x 1  
            
            %Calculate total tracking time
            K = size(Z,1);
            
            %Initialize estimates cell array
            estimates = cell(K,1);
            
            %Initialize states hypothesis array with initial state
            states = state;
            w = 0;
            
            for i = 1:K
                
                % Initialize log-weights array with -Inf
                % Dimensions: [number_hypothesis] * [number of
                % measurements + 1]
                % First column corresponds to missed detections
                n_hyp = length(states);
                m = length(Z{i});
                log_w = -Inf * ones(n_hyp,m+1);
                
                for h = 1:n_hyp %For each hypothesis
                    
                    %Create Missed Detection hypothesis
                    log_w(h,1) = w(h) + log(1 - sensormodel.P_D);
                    
                    %Perform Gating
                    [~,ind] = obj.density.ellipsoidalGating(states(h), Z{i}, measmodel, obj.gating.size);
                    z_ingate = Z{i}(:,ind);
                    
                    if isempty(z_ingate) == 0
                        %Create object detection hypotheses for each detection inside the gate
                        predicted_likelihood = obj.density.predictedLikelihood(states(h), z_ingate, measmodel);
                        %Compute weights
                        log_w(h,[false;ind]) = (log(sensormodel.P_D) - log(sensormodel.intensity_c))...
                            + predicted_likelihood + w(h);
                    end
                    
                end          
                
                %Normalize hypothesis weights 
                [~,log_sum] = normalizeLogWeights(reshape(log_w,1,[]));
                log_w = log_w - log_sum;
                
                %Prune hypotheses with small weights, and then re-normalise the weights
                log_w(log_w <= obj.reduction.w_min) = -Inf;
                [~,log_sum] = normalizeLogWeights(reshape(log_w,1,[]));
                log_w = log_w - log_sum;
                
                % Create non-pruned new detection hypothesis
                new_states = [];
                w = [];
                [row,col] = find(log_w ~= -Inf);
                
                for h = 1:length(row)
                    if col(h) == 1 %If missed detection don't to Kalman update
                        new_states = [new_states states(row(h))];
                    else
                        new_states = [new_states...
                            obj.density.update(states(row(h)), Z{i}(:,col(h)-1), measmodel)];
                    end
                    % Compute weights
                    w(h,1) = log_w(row(h),col(h));       
                end
                    
                %Merge hypothesis within small Mahalanobis distance
                [w,new_states] = hypothesisReduction.merge(w,new_states,...
                        obj.reduction.merging_threshold,obj.density);
                    
                %Cap the number of the hypotheses, and then re-normalise the weights
                [w,new_states] = hypothesisReduction.cap(w,new_states,obj.reduction.M);
                w = normalizeLogWeights(w);
                
                %Extract object state estimate
                [~,index] = max(w);
                estimates{i} = new_states(index).x;
            
                %For each new hypothesis perform Kalman prediction and
                %store weights             
                for h = 1:length(w)
                    new_states(h) = obj.density.predict(new_states(h), motionmodel);
                end
                states = new_states;
            end
        end
        
    end
end
