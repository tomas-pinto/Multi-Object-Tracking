%% Probability Estimation
%Choose object detection probability
P_D = 0.9;
%Choose object survival probability
P_S = 0.99;

%% Sensor Model
range_c = [-100 100;-100 100];
%Choose clutter rate
lambda_c = 300;
sensor_model = modelgen.sensormodel(P_D,lambda_c,range_c);

%% Motion Model
%Create linear motion model
T = 0.1;
sigma_q = 5;
motion_model = motionmodel.cvmodel(T,sigma_q);

%% Measurement Model
%Create linear measurement model
sigma_r = 1;
meas_model = measmodel.cvmeasmodel(sigma_r);

%% Birth Model        
birth_model = repmat(struct('w',log(0.03),'x',[],'P',400*eye(motion_model.d)),[1,1]);
birth_model(1).x = [ 0; 0; 0; 0];

%% Object tracker parameter setting
P_G = 0.99;            %gating size in percentage
w_min = 1e-3;           %hypothesis pruning threshold
merging_threshold = 0.1;  %hypothesis merging threshold
M = 50;                %maximum number of hypotheses kept in PHD
density_class_handle = feval(@GaussianDensity);    %density class handle

%% GM-PHD
objectdata = read_log('ground_truth_log.txt');
measdata = read_log('percept_log.txt');

tracker = multiobjectracker();
tracker = tracker.initialize(density_class_handle,P_S,P_G,meas_model.d,w_min,merging_threshold,M);

%GM-PHD filter
tic
GMPHDestimates = GMPHDfilter(tracker, birth_model, measdata, sensor_model, motion_model, meas_model);
toc

%% Plot Results
%Trajectory plot
true_state = cell2mat(objectdata');
GMPHD_estimated_state = cell2mat(GMPHDestimates');

figure
hold on
grid on

h1 = plot(true_state(1,:), true_state(2,:), 'bo', 'Linewidth', 1);
h2 = plot(GMPHD_estimated_state(1,:), GMPHD_estimated_state(2,:), 'r+', 'Linewidth', 1);

xlabel('x (m)'); ylabel('y (m)')
legend([h1, h2],'Ground Truth','PHD Estimates', 'Location', 'best')
set(gca,'FontSize',12) 