function [object_tracks] = trackgen(K,measmodel,motionmodel,sensormodel,birthmodel,P_S)
%TRACKGEN draws samples from RFS of objects (birth model, dynamic equations) 
%to simulate object trajectories. Assume that a 2D measurement model is used. 
%INPUT: K: total tracking time --- scalar
%       sensormodel: a structure which specifies the sensor parameters
%           P_D: object detection probability --- scalar
%           lambda_c: average number of clutter measurements per time scan, 
%                     --- scalar
%           range_c: range of surveillance area --- if 2D model: 2
%                    x 2 matrix of the form [xmin xmax;ymin ymax]; if 1D
%                    model: 1 x 2 vector of the form [xmin xmax]
%           pdf_c: clutter (Poisson) intensity --- scalar
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
%           w: weights of mixture components
%           x: mean of mixture components
%           P: covariance of mixture components
%       P_S: object survival probability --- scalar
%OUTPUT:object_tracks: a structure array specifies object tracks ---
%       (number of tracks x 1) 
%           tbirth: track initiate (object appears) time --- scalar
%           tdeath: track end time (object disappears) time --- scalar
%           x: object trajectory --- (object state dimension x time steps 
%              object exists in the scene)
%Note that if the number of tracks is zero, set the output to empty

object_tracks = struct();
n_el = size(birthmodel,2);
log_w = zeros(size(birthmodel,2),1);

for n = 1:n_el
    log_w(n) = birthmodel(n).w;
end

[log_w,log_sum_w] = normalizeLogWeights(log_w);

n_tracks = 0;

for t = 1:K %time step

    n_obj = poissrnd(exp(log_sum_w));
    
    for i = 1:n_obj %for each newborn object
        
        n_tracks = n_tracks + 1;
        
        % Sample a birth component
        index = find(rand < cumsum(exp(log_w)), 1, 'first');
        
        % Sample a kinematic state
        xk = mvnrnd(birthmodel(index).x, birthmodel(index).P)';
        object_tracks(n_tracks,1).x = xk;
        
        % Sample a measurement state
        meas = mvnrnd(measmodel.h(xk), measmodel.R)';
        
        % Set the time of birth
        object_tracks(n_tracks,1).tbirth = t;
        
        ifalive = true;
        time = t;
        
        while (ifalive == true) && (time + 1 <= K) ...
                && meas(1) > sensormodel.range_c(1,1) ...
                && meas(1) < sensormodel.range_c(1,2) ...
                && meas(2) > sensormodel.range_c(2,1) ...
                && meas(2) < sensormodel.range_c(2,2) 
            
            time = time + 1;
            
            % Simulate the object's kinematic state at next time step
            xk =  mvnrnd(motionmodel.f(xk), motionmodel.Q)';
            
            % Sample a measurement state
            meas = mvnrnd(measmodel.h(xk), measmodel.R)';
            
            % Update object track
            object_tracks(n_tracks,1).x = [object_tracks(n_tracks).x xk];
            
            % Determine whether the object is still alive
            if rand > P_S
                ifalive = false;
            end            
        end 
        
        % Update object death time
        object_tracks(n_tracks,1).tdeath = time;
    end
end

end

