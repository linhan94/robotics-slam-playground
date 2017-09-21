% This scripts assumes that python and the follownig packages are
% installed (available for each platform): argparse numpy
% If you have python and package manager pip installed, you can simply
% install the packages using: pip install argparse numpy
%
% Also, check that the relative paths are correct.

% windows
% system('python external/tum-evaluation/evaluate_ate.py blender/camera_poses.txt src/output/vo_output_poses4020.txt --plot example_ate.pdf')
% system('python external/tum-evaluation/evaluate_rpe.py blender/camera_poses.txt src/output/vo_output_poses4020.txt --plot example_rpe.pdf --fixed_delta')
% 

% Using Reference implementation
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py blender/camera_poses.txt src/output/vo_output_poses4020.txt --plot example_ate4020.pdf');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py blender/camera_poses.txt src/output/vo_output_poses4020.txt --plot example_rpe4020.pdf --fixed_delta');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py blender/camera_poses.txt src/output/vo_output_poses6030.txt --plot example_ate6030.pdf');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py blender/camera_poses.txt src/output/vo_output_poses6030.txt --plot example_rpe6030.pdf --fixed_delta');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py blender/camera_poses.txt src/output/vo_output_poses8040.txt --plot example_ate8040.pdf');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py blender/camera_poses.txt src/output/vo_output_poses8040.txt --plot example_rpe8040.pdf --fixed_delta');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py blender/camera_poses.txt src/output/vo_output_poses106.txt --plot example_ate106.pdf');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py blender/camera_poses.txt src/output/vo_output_poses106.txt --plot example_rpe106.pdf --fixed_delta');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py blender/camera_poses.txt src/output/vo_output_poses52.txt --plot example_ate52.pdf');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py blender/camera_poses.txt src/output/vo_output_poses52.txt --plot example_rpe52.pdf --fixed_delta');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py blender/camera_poses.txt src/output/vo_output_poses53.txt --plot example_ate53.pdf');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py blender/camera_poses.txt src/output/vo_output_poses53.txt --plot example_rpe53.pdf --fixed_delta');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py blender/camera_poses.txt src/output/vo_output_poses2010.txt --plot example_ate2010.pdf');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py blender/camera_poses.txt src/output/vo_output_poses2010.txt --plot example_rpe2010.pdf --fixed_delta')
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py blender/camera_poses.txt src/output/vo_output_poses106GPS.txt --plot example_ate106GPS.pdf');
system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py blender/camera_poses.txt src/output/vo_output_poses106GPS.txt --plot example_rpe106GPS.pdf --fixed_delta')


%The RTE and ATE computed over	just the length	of	the	sliding	window.
input_poses = dlmread('blender/camera_poses.txt');
output_poses = dlmread('src/output/vo_output_poses4020.txt');

lengthOfWindows = 40;

ATE_window = zeros(length(input_poses) - lengthOfWindows,1);
RTE_window = zeros(length(input_poses) - lengthOfWindows,1);
for i = 1:length(input_poses) - lengthOfWindows
    for j = i:lengthOfWindows
        dlmwrite('temp_input_poses.txt',input_poses(j,:),'delimiter','\t','precision',6);
        dlmwrite('temp_output_poses.txt',output_poses(j,:),'delimiter','\t','precision',6);
        system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py temp_input_poses.txt temp_output_poses.txt --save ate_out.txt');
        %system('python external/tum-evaluation/evaluate_ate.py temp_input_poses.txt temp_output_poses.txt --save ate_out.txt');
        ATE_temp = dlmread('ate_out.txt');
        system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py temp_input_poses.txt temp_output_poses.txt --save rpe_out.txt');
        %system('python external/tum-evaluation/evaluate_rpe.py temp_input_poses.txt temp_output_poses.txt --save rpe_out.txt --fixed_delta');
        RTE_temp = dlmread('rpe_out.txt');
        ATE_window(i,:) = ATE_window(i,:) + ATE_temp(:,2);
        RTE_window(i,:) = RTE_window(i,:) + RTE_temp(:,5);
    end
end
figure
plot(1:length(input_poses) - lengthOfWindows, ATE_window);
title('ATE computed over	just the length	of	the	sliding	window');
figure
plot(1:length(input_poses) - lengthOfWindows, RTE_window);
title('RPE computed over	just the length	of	the	sliding	window');

    
    
% The RTE and ATE computed from	the start of the run to	the	current	time
ATE_all = zeros(length(input_poses) ,1);
RTE_all = zeros(length(input_poses),1);
for i = 1:length(input_poses)
    for j = 1:i
        dlmwrite('temp_input_poses.txt',input_poses(j,:),'delimiter','\t','precision',6);
        dlmwrite('temp_output_poses.txt',output_poses(j,:),'delimiter','\t','precision',6);
        system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_ate.py temp_input_poses.txt temp_output_poses.txt --save ate_out.txt');
        %system('python external/tum-evaluation/evaluate_ate.py temp_input_poses.txt temp_output_poses.txt --save ate_out.txt');
        ATE_temp = dlmread('ate_out.txt');
        system('C:\Users\linha\Documents\MATLAB\compgx04-coursework-02-materials-20170322-rev1\venv\Scripts\python.exe external/tum-evaluation/evaluate_rpe.py temp_input_poses.txt temp_output_poses.txt --save rpe_out.txt');
        %system('python external/tum-evaluation/evaluate_rpe.py temp_input_poses.txt temp_output_poses.txt --save rpe_out.txt --fixed_delta');
        RTE_temp = dlmread('rpe_out.txt');
        ATE_all(i,:) = ATE_all(i,:) + ATE_temp(:,2);
        RTE_all(i,:) = RTE_all(i,:) + RTE_temp(:,5);
    end
end
figure
plot(1:length(input_poses), ATE_all);
title('ATE computed from the start of the run to	the	current	time');
figure
plot(1:length(input_poses), RTE_all);
title('% RTE computed from	the start of the run to	the	current	time')

% Optimizing Performance of the algorithm
ate1 = [18.060636, 16.015757, 14.531238, 11.234455, 6.847573, 4.382276, 3.700761];
rpe1 = [0.346481458813, 0.286454007093, 0.23889744855, 0.199147103836, 0.126686917347, 0.086001088259, 0.0742144133169];
time = [1.85, 1.93, 2.14, 2.52, 3.92, 5.26, 5.56];
size = [2, 3, 6, 10, 20, 30, 40];

figure
plot(size,ate1,'b');
hold on
plot(size,time,'r');
hold on
plot(size,ate1./time,'g');
title('ATE');
figure
plot(time,rpe1);
plot(size,rpe1.*10,'b');
hold on
plot(size,time,'r');
hold on
plot(size,rpe1./time.*10,'g');
title('RPE');
