clear

import gtsam.*

% Options
NUM_FRAMES = 0; % 0 for all
ADD_NOISE = 1;
ITERATIONS = 100;
blenddir = strcat(fileparts(mfilename('fullpath')), '/../blender/');

% Load data 
camera_gt = dlmread(strcat(blenddir, 'camera_poses.txt'));
features_gt = dlmread(strcat(blenddir, 'tracks_dist.txt'));
landmarks_gt = dlmread(strcat(blenddir, 'landmarks_3d.txt'));
landmarks_used = zeros(size(landmarks_gt,1),1);
if NUM_FRAMES < 1
    NUM_FRAMES = size(camera_gt, 1);
end
calib = Cal3_S2( ...
    634.8, ... % focal
    634.8, ... % focal
    0, ... % skew
    480,... % center
    270); % center

%% Setup noise
measurementNoiseSigma = 3;
pointNoiseSigma = 0.1;
rotationSigma = 0.2;
positionSigma = 3;
poseNoiseSigmas = [ positionSigma positionSigma positionSigma ...
                    rotationSigma rotationSigma rotationSigma]';
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);

%% Add noise to input data
if ADD_NOISE == 1
    disp('Adding noise...')
    
    for i=1:NUM_FRAMES
        
        % 2D features
        f = 1;
        while f < size(features_gt, 2) && features_gt(i,f) > 0
            features_gt(i,f+1:f+2) = features_gt(i,f+1:f+2) + measurementNoiseSigma * randn(1,2);
            f = f + 4;
        end
        
        % Camera Poses
        rot = Rot3.Quaternion( camera_gt(i,8), ...
                               camera_gt(i,5), ...
                               camera_gt(i,6), ...
                               camera_gt(i,7)).xyz;
        
        rot = rot + rotationSigma * randn(3,1);
        rotq = Rot3.RzRyRx(rot).quaternion();
        camera_gt(i,8) = rotq(1);
        camera_gt(i,5) = rotq(2);
        camera_gt(i,6) = rotq(3);
        camera_gt(i,7) = rotq(4);        
        camera_gt(i,2:4) = camera_gt(i,2:4) + positionSigma * randn(1,3);       
    end
    
    % 3D landmarks
    landmarks_gt = landmarks_gt + randn(size(landmarks_gt));  
end

graph = NonlinearFactorGraph;
initialEstimate = Values;

% simplify measurment
% features_gt = features_gt(:,1:360);

%% Add factors for all measurements
for i=1:NUM_FRAMES
    
    fprintf('Adding frame %d to graph...\n', i)
    
    cam_pose = Pose3(  Rot3.Quaternion(camera_gt(i,8), camera_gt(i,5), camera_gt(i,6), camera_gt(i,7)), ...
                       Point3(camera_gt(i,2), camera_gt(i,3), camera_gt(i,4)));
	
    initialEstimate.insert(symbol('p', i), cam_pose);
                   
    f = 1; % column of current feature ID
    while f < size(features_gt, 2) && features_gt(i,f) > 0
        feature_id = features_gt(i,f);
        graph.add(GenericProjectionFactorCal3_S2(Point2(features_gt(i,f+1), features_gt(i,f+2)), measurementNoise, symbol('p', i), symbol('f', feature_id), calib));
        % Initialise the point near ground-truth
        if landmarks_used(feature_id,1) < 1
            rep = [(features_gt(i,f+1) - calib.px) / calib.fx, (features_gt(i,f+2) - calib.py) / calib.fy, 1] * features_gt(i,f+3);
                    rep = cam_pose.matrix * [rep'; 1];
                    feature_pos = Point3(rep(1),rep(2),rep(3));
                    
             %Exclude some of the features
                    
            graph.add(PriorFactorPoint3(symbol('f',feature_id), feature_pos, pointPriorNoise));
            initialEstimate.insert(symbol('f',feature_id), feature_pos);
            
            landmarks_used(feature_id,1) = 1;
        end
        f = f + 4;
    end
end

optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
for i=1:ITERATIONS
    fprintf('Starting iteration %d...\n', i);
    optimizer.iterate();
end

disp('Retrieving results...');
result = optimizer.values();
fprintf('Initial error: %f\n', graph.error(initialEstimate));
fprintf('Final error: %f\n', graph.error(result));

%% Output noisy input data and optimised result
disp('Exporting results...');
dlmwrite('input_poses.txt',camera_gt(1:NUM_FRAMES,:),'delimiter','\t','precision',6);
output_poses = zeros(NUM_FRAMES,size(camera_gt,2));
for p = 1:NUM_FRAMES
    pose = result.at(symbol('p', p));
    pos = pose.translation();
    quat = pose.rotation().quaternion();
    output_poses(p,:) = [camera_gt(p,1) pos.x pos.y pos.z quat(4) quat(2) quat(3) quat(1)];
end
dlmwrite('output_poses.txt',output_poses,'delimiter','\t','precision',6);

%% 2.1 Plot result
figure
plot3(camera_gt(:,2), camera_gt(:,3), camera_gt(:,4), 'gx');
hold on
plot3(output_poses(:,2),output_poses(:,3),output_poses(:,4),'rx');

%% 2.2 Plot covariance
mar = gtsam.Marginals(graph, result);
feaCov = zeros([3,size(landmarks_used)]);
% get the diagonals of the covariance matrices
for i = 1:size(landmarks_used)
    if landmarks_used(i) > 0
        feaCov(:,i) = diag(mar.marginalCovariance(gtsam.symbol('f',i)));
    end
end
% plot the covariances of the different values of the features
figure
k = 0;
for i = 1:size(landmarks_used)
    if landmarks_used(i) > 0
        scatter3(landmarks_gt(i,1),landmarks_gt(i,2),landmarks_gt(i,3),5,norm(feaCov(:,i)));
        hold on;
        k = k+1;
    end
end
colorbar %show color scale
