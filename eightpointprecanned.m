
%test eight point algorithm

% precanned version using previously carefully clicked points!


%read in precanned points
load savepoints.mat
x1 = savx1; y1 = savy1; x2 = savx2; y2 = savy2;

im = imread('oldmain6.jpg');
im2 = imread('oldmain5.jpg');

F = eightpoint(im,im2,[x1 y1],[x2 y2]);

