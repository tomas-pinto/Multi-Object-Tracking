%Model structures need to be called:
    %groundtruth: a structure specifies the parameters to generate the ground truth
    %           nbirths: number of objects hypothesised to exist from
    %           time step 1 to time step K --- scalar 
    %           xstart: object initial state --- (object state
    %           dimension) x nbirths matrix 
    %           tbirth: object birth (appearing) time --- (total number
    %           of objects existed in the scene) x 1 vector  
    %           tdeath: the last time the object exists ---  (total number
    %           of objects existed in the scene) x 1 vector 
    %           K: total tracking time --- scalar
    %motionmodel: a structure specifies the motion model parameters
    %           d: object state dimension --- scalar
    %           F: function handle return transition/Jacobian matrix
    %           f: function handle return predicted object state
    %           Q: motion noise covariance matrix
function objectdata = objectdatagen(groundtruth,motionmodel,ifnoisy)
%TARGETDATA generates groundtruth object data
%INPUT:  groundtruth specifies the parameters used to generate groundtruth
%        motionmodel: a structure specifies the motion model parameters
%        ifnoisy: boolean value indicating whether to generate noisy object
%        state sequence or not 
%OUTPUT: objectdata.X:  (K x 1) cell array, each cell stores object states
%        of size (object state dimension) x (number of objects at
%        corresponding time step)  
%        objectdata.N:  (K x 1) vector, each element stores the number of
%        objects at corresponding time step 

objectdata.X = cell(groundtruth.K,1);
objectdata.N = zeros(groundtruth.K,1);

%Try to switch time step and object
for i = 1:groundtruth.K %time step
    for j = 1:groundtruth.nbirths %object
        if ((groundtruth.tbirth(j) <= i) && (groundtruth.tdeath(j) >= i))
            objectdata.N(i) = objectdata.N(i) + 1;
            if ifnoisy == false
                groundtruth.xstart(:,j) = motionmodel.f(groundtruth.xstart(:,j)); %update state
            else
                groundtruth.xstart(:,j) = mvnrnd(motionmodel.f(groundtruth.xstart(:,j)),motionmodel.Q)'; %update state w/noise
            end        
            objectdata.X{i} = [objectdata.X{i} groundtruth.xstart(:,j)];  
        end
    end
end
    
end

