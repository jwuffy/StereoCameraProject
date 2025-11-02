%% Task 3.4  Triangulation Measurements (No Toolbox Version)
% CMPEN/EE 454 – Project 2
% Author: Michael Cichocki
% Purpose: Use triangulation to make 3-D measurements (doorway, person, camera)
% Uses ginput instead of cpselect

clc; clear; close all;

% -------------------------------------------------------------------------
% 1. Load camera parameters and images
% -------------------------------------------------------------------------
load('Parameters_V1_1.mat'); cam1 = Parameters;
load('Parameters_V2_1.mat'); cam2 = Parameters;
load('mocapPoints3D.mat');  % optional, for reference

I1 = imread('im1corrected.jpg');
I2 = imread('im2corrected.jpg');

fprintf('Loaded calibration for two cameras.\n');

% -------------------------------------------------------------------------
% 2. Select matching points manually (no cpselect required)
% -------------------------------------------------------------------------
fprintf('\nClick corresponding points in BOTH images.\n');
fprintf('1️⃣ First click all points in the LEFT image (e.g., floor, door, head, tripod).\n');
fprintf('2️⃣ Then click the SAME physical points in the RIGHT image in the SAME order.\n');
fprintf('Press ENTER after you finish selecting points in each image.\n\n');

figure;
subplot(1,2,1);
imshow(I1);
title('LEFT image – click points, then press ENTER');
[x1, y1] = ginput;   % left image

subplot(1,2,2);
imshow(I2);
title('RIGHT image – click SAME points, then press ENTER');
[x2, y2] = ginput;   % right image

fixedPoints  = [x1 y1];   % left image
movingPoints = [x2 y2];   % right image

fprintf('\nYou selected %d corresponding points.\n', size(fixedPoints,1));

% -------------------------------------------------------------------------
% 3. Triangulate 3-D points
% -------------------------------------------------------------------------
% --- Extract intrinsic parameters robustly ---
% Extract intrinsics directly
K1 = cam1.Kmat;
K2 = cam2.Kmat;

% Extract rotation and translation
R1 = cam1.Rmat; 
R2 = cam2.Rmat;

% The translation vectors (column) from camera centers
t1 = -R1 * cam1.position(:);
t2 = -R2 * cam2.position(:);

% Projection matrices
P1 = K1 * [R1, t1];
P2 = K2 * [R2, t2];

% Normalized directions (pixel -> ray)
fx1 = K1(1,1); fy1 = K1(2,2);
cx1 = K1(1,3); cy1 = K1(2,3);
fx2 = K2(1,1); fy2 = K2(2,2);
cx2 = K2(1,3); cy2 = K2(2,3);

n1 = [(fixedPoints(:,1) - cx1)/fx1, (fixedPoints(:,2) - cy1)/fy1, ones(size(fixedPoints,1),1)];
n2 = [(movingPoints(:,1) - cx2)/fx2, (movingPoints(:,2) - cy2)/fy2, ones(size(movingPoints,1),1)];

pts3D = zeros(3,size(n1,1));
for i = 1:size(n1,1)
    A = [ n1(i,1)*P1(3,:) - P1(1,:);
          n1(i,2)*P1(3,:) - P1(2,:);
          n2(i,1)*P2(3,:) - P2(1,:);
          n2(i,2)*P2(3,:) - P2(2,:) ];
    [~,~,V] = svd(A);
    X = V(:,end);
    pts3D(:,i) = X(1:3)./X(4);
end
% -------------------------------------------------------------------------
% 4. Fit planes (floor and wall)
% -------------------------------------------------------------------------
fprintf('\nNow define which points belong to FLOOR and WALL.\n');
fprintf('Example: floor_idx = [1 2 3]; wall_idx = [4 5 6];\n');
fprintf('You can edit these in the Command Window after the figure opens.\n');

disp(pts3D');  % display coordinates for reference

% (pause here so user can define indices)
keyboard

% Fit plane ax + by + cz + d = 0 using null space
Xf = pts3D(:,floor_idx)'; 
v = null([Xf, ones(size(Xf,1),1)]); 
a = v(1); b = v(2); c = v(3); d = v(4);
fprintf('\nFloor plane: %.3fx + %.3fy + %.3fz + %.3f = 0\n', a,b,c,d);

Xw = pts3D(:,wall_idx)'; 
v = null([Xw, ones(size(Xw,1),1)]); 
aw = v(1); bw = v(2); cw = v(3); dw = v(4);
fprintf('Wall plane:  %.3fx + %.3fy + %.3fz + %.3f = 0\n', aw,bw,cw,dw);

% -------------------------------------------------------------------------
% 5. Measure heights (door, person, camera)
% -------------------------------------------------------------------------
fprintf('\nNow assign indices for door and person points (example below):\n');
fprintf('door_bottom_idx = 7; door_top_idx = 8; person_head_idx = 9;\n');
keyboard

door_height   = abs(pts3D(3,door_top_idx) - pts3D(3,door_bottom_idx));
person_height = abs(pts3D(3,person_head_idx) - mean(pts3D(3,floor_idx)));
fprintf('\nDoor height ≈ %.2f m\n', door_height);
fprintf('Person height ≈ %.2f m\n', person_height);

% -------------------------------------------------------------------------
% 6. Estimate camera on tripod
% -------------------------------------------------------------------------
fprintf('\nIf you clicked a tripod camera point, assign its index:\n');
fprintf('camera_idx = 10;\n');
keyboard

fprintf('Tripod camera center (X,Y,Z) ≈ [%.2f, %.2f, %.2f]\n', ...
    pts3D(1,camera_idx), pts3D(2,camera_idx), pts3D(3,camera_idx));

fprintf('\n✅ All done – you now have physical 3-D scene measurements!\n');
