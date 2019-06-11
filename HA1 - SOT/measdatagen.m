%Model structures need to be called:
    %objectdata: a structure specifies the object trajectories
    %           X: (K x 1) cell array, each cell stores object states
    %           of size (object state dimension) x (number of objects at
    %           corresponding time step)  
    %           N:  (K x 1) cell array, each cell stores the number of
    %           objects at corresponding time step 
    %sensormodel: a structure specifies the sensor parameters
    %           P_D: object detection probability --- scalar
    %           lambda_c: average number of clutter measurements per time
    %           scan, Poisson distributed --- scalar 
    %           range_c: range of surveillance area --- if 2D model: 2
    %           x 2 matrix of the form [xmin xmax;ymin ymax]; if 1D
    %           model: 1 x 2 vector of the form [xmin xmax] 
    %           pdf_c: clutter (Poisson) density --- scalar
    %           intensity_c: clutter (Poisson) intensity --- scalar
    %measmodel: a structure specifies the measurement model parameters
    %           d: measurement dimension --- scalar
    %           H: function handle return transition/Jacobian matrix
    %           h: function handle return the observation of the object
    %           state 
    %           R: measurement noise covariance matrix
function measdata = measdatagen(objectdata, sensormodel, measmodel)
%MEASDATAGEN generates object-generated measurements and clutter
%INPUT:     objectdata: a structure contains object data
%           sensormodel: a structure specifies sensor model parameters
%           measmodel: a structure specifies the measurement model
%           parameters 
%OUTPUT:    measdata: cell array of size (total tracking time, 1), each
%           cell stores measurements of size (measurement dimension) x
%           (number of measurements at corresponding time step) 

% We strongly recommend you to make use of the code snippet provided below,
% such that the random numbers generated in your code are reproducible. Otherwise,
% you may fail to pass the tests even if your implementation is correct.

%Initialize memory
K = length(objectdata.X);
measdata = cell(K,1);

%Generate measurements
for k = 1:K
    if objectdata.N(k) > 0
        idx = rand(objectdata.N(k),1) <= sensormodel.P_D;
        %Only generate object-originated observations for detected objects
        true_states = [];
        for i = 1:length(idx)
            if idx(i) ~= 0
                true_states(:,sum(idx(1:i))) = measmodel.h(objectdata.X{k}(:,i));
            end
        end
        
        for i = 1:size(true_states,2)
            measdata{k} = [measdata{k} mvnrnd(true_states(:,i),measmodel.R)'];
        end
    end
    
    %Number of clutter measurements
    N_c = poissrnd(sensormodel.lambda_c);
    
    %Generate clutter
    C = (sensormodel.range_c(:,2)-sensormodel.range_c(:,1)).*rand(measmodel.d,N_c) + sensormodel.range_c(:,1);
    
    %Total measurements are the union of object detections and clutter
    measdata{k} = [measdata{k} C];                                                                                                                                    
end

end
