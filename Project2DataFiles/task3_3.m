
%% Task 3.3  Triangulation Example
clc; clear; close all;

% --- Load the data you were given ---
load('Parameters_V1_1.mat'); % Camera 1 parameters
cam1 = Parameters;           % Store as cam1 structure
load('Parameters_V2_1.mat'); % Camera 2 parameters
cam2 = Parameters;           % Store as cam2 structure
load('mocapPoints3D.mat');   % 3D motion capture points

% For now just see what's inside:
% Display the calibration paramter structures for each camera
disp('Camera 1 parameters:');
disp(cam1);

disp('Camera 2 parameters:');
disp(cam2);

%Display first few mocap points to verify successful load
disp('First few mocap points:');
disp(pts3D(:,1:5));

% --- Simple "result display" example ---
% (This just plots a few of the mocap 3D points)
figure;                     % open a new window
scatter3(pts3D(1,:), pts3D(2,:), pts3D(3,:), 'filled');

title('Original 3D Mocap Points');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; axis equal;        % Equal scaling for all axes

