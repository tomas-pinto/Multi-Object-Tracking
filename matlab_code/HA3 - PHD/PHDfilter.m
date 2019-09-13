classdef PHDfilter
    %PHDFILTER is a class containing necessary functions to implement the
    %PHD filter 
    %Model structures need to be called:
    %    sensormodel: a structure which specifies the sensor parameters
    %           P_D: object detection probability --- scalar
    %           lambda_c: average number of clutter measurements per time scan, 
    %                     Poisson distributed --- scalar
    %           pdf_c: value of clutter pdf --- scalar
    %           intensity_c: Poisson clutter intensity --- scalar
    %       motionmodel: a structure which specifies the motion model parameters
    %           d: object state dimension --- scalar
    %           F: function handle return transition/Jacobian matrix
    %           f: function handle return predicted object state
    %           Q: motion noise covariance matrix
    %       measmodel: a structure which specifies the measurement model parameters
    %           d: measurement dimension --- scalar
    %           H: function handle return transition/Jacobian matrix
    %           h: function handle return the observation of the target state
    %           R: measurement noise covariance matrix
    %       birthmodel: a structure array which specifies the birth model (Gaussian
    %       mixture density) parameters --- (1 x number of birth components)
    %           w: weights of mixture components (in logarithm domain)
    %           x: mean of mixture components
    %           P: covariance of mixture components
    
    properties
        density %density class handle
        paras   %parameters specify a PPP
    end
    
    methods
        function obj = initialize(obj,density_class_handle,birthmodel)
            %INITIATOR initializes PHDfilter class
            %INPUT: density_class_handle: density class handle
            %OUTPUT:obj.density: density class handle
            %       obj.paras.w: weights of mixture components --- vector
            %                    of size (number of mixture components x 1)
            %       obj.paras.states: parameters of mixture components ---
            %                    struct array of size (number of mixture
            %                    components x 1) 
            
            obj.density = density_class_handle;
            obj.paras.w = [birthmodel.w]';
            obj.paras.states = rmfield(birthmodel,'w')';
        end
        
        function obj = predict(obj,motionmodel,P_S,birthmodel)
            %PREDICT performs PPP prediction step
            %INPUT: P_S: object survival probability
            
            % Number of pre-existing states
            n_states = length(obj.paras.states);
            
            % Update weights of pre-existing objects
            obj.paras.w = log(P_S) + obj.paras.w;
            
            % Predict Poisson intensity for pre-existing objects
            for i = 1:n_states
                obj.paras.states(i) = obj.density.predict(obj.paras.states(i), motionmodel);
            end
            
            % Incorporate Poisson birth intensity into Poisson intensity for pre-existing objects
            for i = 1:length(birthmodel)              
                obj.paras.w(i+n_states) = birthmodel(i).w;
                obj.paras.states(i+n_states).x = birthmodel(i).x;
                obj.paras.states(i+n_states).P = birthmodel(i).P;
            end
            
        end
        
        function obj = update(obj,z,measmodel,intensity_c,P_D,gating)
            %UPDATE performs PPP update step and PPP approximation
            %INPUT: z: measurements --- matrix of size (measurement dimension 
            %          x number of measurements)
            %       intensity_c: Poisson clutter intensity --- scalar
            %       P_D: object detection probability --- scalar
            %       gating: a struct with two fields: P_G, size, used to
            %               specify the gating parameters
            
            n_states = length(obj.paras.states);
            
            %Construct update components resulted from missed detection
            missed = obj.paras;
            missed.w = log(1-P_D) + missed.w;
            
            index = 1;
            for h = 1:n_states
                
                % Perform ellipsoidal gating 
                [~,ind] = obj.density.ellipsoidalGating(obj.paras.states(h), z, measmodel, gating.size);
                z_ingate = z(:,ind);
                look_ind = find(ind == 1);
                
                if isempty(z_ingate) == 0
                    for i = 1:size(z_ingate,2)
                        
                        % Kalman Update for measurements inside gate                       
                        w(index) = log(P_D) + obj.paras.w(h) + ...
                            obj.density.predictedLikelihood(obj.paras.states(h), z_ingate(:,i), measmodel);
                        
                        lookup_table(index) = look_ind(i);
                        
                        %if sum(obj.paras.states(h).x == 4)
                            %detected(index) = z_ingate(:,i);
                        %else
                        detected(index) = obj.density.update(obj.paras.states(h), z_ingate(:,i), measmodel);
                        %end
                        
                        index = index + 1;
                    end
                end              
            end
            
            % Compute the log weights
            a = unique(lookup_table);
            for i = 1:length(a)
                [~, log_sum_w] = normalizeLogWeights([w(lookup_table == a(i)) log(intensity_c)]); 
                w(lookup_table == a(i)) = w(lookup_table == a(i)) - log_sum_w;
            end
            
            % Update object states and weights
            obj.paras.states = [];
            
            % Include missed detection
            obj.paras.states = [detected'; missed.states];
            obj.paras.w = [w'; missed.w];
        end
        
        function obj = componentReduction(obj,reduction)
            %COMPONENTREDUCTION approximates the PPP by representing its
            %intensity with fewer parameters
            
            %Pruning
            [obj.paras.w, obj.paras.states] = hypothesisReduction.prune(obj.paras.w, obj.paras.states, reduction.w_min);
            %Merging
            %if length(obj.paras.w) > 1
                %[obj.paras.w, obj.paras.states] = hypothesisReduction.merge(obj.paras.w, obj.paras.states, reduction.merging_threshold, obj.density);
            %end
            %Capping
            [obj.paras.w, obj.paras.states] = hypothesisReduction.cap(obj.paras.w, obj.paras.states, reduction.M);
        end
        
        function estimates = PHD_estimator(obj)
            %PHD_ESTIMATOR performs object state estimation in the GMPHD filter
            %OUTPUT:estimates: estimated object states in matrix form of
            %                  size (object state dimension) x (number of
            %                  objects) 
            
            estimates = [];
            
            n = min(size(obj.paras.states,1),round(sum(exp(obj.paras.w))));
            
            [~,ind] = sort(obj.paras.w,'descend');
            
            for i = 1:n
                estimates(:,i) = obj.paras.states(ind(i)).x;
            end

        end
        
    end
    
end
