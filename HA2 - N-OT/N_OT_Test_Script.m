%rng(r);
%% Choose object detection probability
P_D = 0.7;
%Choose clutter rate
lambda_c = 500;

%Choose linear or nonlinear scenario
scenario_type = 'linear';

%Create sensor model
range_c = [-1000 1000;-1000 1000];
sensor_model = modelgen.sensormodel(P_D,lambda_c,range_c);
        
%Creat linear motion model
T = 1;
sigma_q = 5;
motion_model = motionmodel.cvmodel(T,sigma_q);
        
%Create linear measurement model
sigma_r = 10;
meas_model = measmodel.cvmeasmodel(sigma_r);
        
%Creat ground truth model
nbirths = 5;
K = 20;
tbirth = zeros(nbirths,1);
tdeath = zeros(nbirths,1);
        
initial_state = repmat(struct('x',[],'P',eye(motion_model.d)),[1,nbirths]);
        
initial_state(1).x = [0; 0; 0; -10];        tbirth(1) = 1;   tdeath(1) = K;
initial_state(2).x = [400; -600; -10; 5];   tbirth(2) = 1;   tdeath(2) = K;
initial_state(3).x = [-800; -200; 20; -5];  tbirth(3) = 1;   tdeath(3) = K;
initial_state(4).x = [0; 0; 7.5; -5];       tbirth(4) = 1;   tdeath(4) = K;
initial_state(5).x = [-200; 800; -3; -15];  tbirth(5) = 1;   tdeath(5) = K;

%% Generate true object data (noisy or noiseless) and measurement data
ground_truth = modelgen.groundtruth(nbirths,[initial_state.x],tbirth,tdeath,K);
ifnoisy = 1;
my_objectdata = objectdatagen(ground_truth,motion_model,ifnoisy);
my_measdata = measdatagen(my_objectdata,sensor_model,meas_model);

%% N-object tracker parameter setting
P_G = 0.999;            %gating size in percentage
w_min = 1e-3;           %hypothesis pruning threshold
merging_threshold = 2;  %hypothesis merging threshold
M = 100;                %maximum number of hypotheses kept in TOMHT
density_class_handle = feval(@GaussianDensity);    %density class handle
tracker = n_objectracker();
tracker = tracker.initialize(density_class_handle,P_G,meas_model.d,w_min,merging_threshold,M);

%Global Nearest Neighbour Estimation
my_GNNestimates = GNNfilter(tracker, initial_state, my_measdata, sensor_model, motion_model, meas_model);
GNN_RMSE = RMSE_n_objects(my_GNNestimates,my_objectdata.X);

% Joint Probabilistic Data Association Estimation
my_JPDAestimates = JPDAfilter(tracker, initial_state, my_measdata, sensor_model, motion_model, meas_model);
JPDA_RMSE = RMSE_n_objects(my_JPDAestimates,my_objectdata.X);

% TO-MHT Estimation
my_TOMHTestimates = TOMHT(tracker, initial_state, my_measdata, sensor_model, motion_model, meas_model);
TOMHT_RMSE = RMSE_n_objects(my_TOMHTestimates,my_objectdata.X);

X = sprintf('Root mean square error: GNN: %.3f; JPDA: %.3f; TOMHT: %.3f.'...
    ,GNN_RMSE,JPDA_RMSE,TOMHT_RMSE);
disp(X)

figure
hold on
grid on

for i = 1:nbirths
    h1 = plot(cell2mat(cellfun(@(x) x(1,i), my_objectdata.X, 'UniformOutput', false)), ...
        cell2mat(cellfun(@(x) x(2,i), my_objectdata.X, 'UniformOutput', false)), 'g', 'Linewidth', 2);
    h2 = plot(cell2mat(cellfun(@(x) x(1,i), my_GNNestimates, 'UniformOutput', false)), ...
        cell2mat(cellfun(@(x) x(2,i), my_GNNestimates, 'UniformOutput', false)), 'r-s', 'Linewidth', 1);
    h3 = plot(cell2mat(cellfun(@(x) x(1,i), my_JPDAestimates, 'UniformOutput', false)), ...
        cell2mat(cellfun(@(x) x(2,i), my_JPDAestimates, 'UniformOutput', false)), 'm-o', 'Linewidth', 1);
    h4 = plot(cell2mat(cellfun(@(x) x(1,i), my_TOMHTestimates, 'UniformOutput', false)), ...
        cell2mat(cellfun(@(x) x(2,i), my_TOMHTestimates, 'UniformOutput', false)), 'b-d', 'Linewidth', 1);
end

xlabel('x'); ylabel('y')

xlim([-1000 1000])
ylim([-1000 1000])

legend([h1 h2 h3 h4],'Ground Truth','GNN Estimates','JPDA Estimates', 'TOMHT Estimates', 'Location', 'best')

set(gca,'FontSize',12) 