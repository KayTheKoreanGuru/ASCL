%% Clear workspace
clc
clear all
close all
%% Input
global LiDARedge RGBedge LiDARx LiDARy RGBx RGBy
LiDARx = 100;       % desired pixel values for 2D projection
LiDARy = 100;

num_feature = 3;
MaxD = 1000;
zfar = 1;
x0 = ones(1,9)';
%% Read LiDAR image


Path = 'C:\Users\escap\Box\Documents\Lab\Calibration\Pointcloud\';

File = dir(fullfile(Path,'*.ply'));                     %save path for the point cloud file
FileNames = {File.name}';                               %call filename
%% 
LiDAR3D = char(strcat(Path, FileNames(2)));             %pointcloud filepath

%% 
masterinfo = pc2master(LiDAR3D,LiDARx,LiDARy,MaxD);      %read ptCloud 3D

%% Read RGB Image

File = dir(fullfile(Path,'*.png'));                     %save path for the RGB file
FileNames = {File.name}';                               %call filename
Camera = char(strcat(Path, FileNames(1)));             %pointcloud filepath
RGB = imread(Camera);                                   %Read LiDAR RGB camera image
RGB = imresize(RGB,[LiDARy,LiDARx]);                        %resize image
RGBx = size(RGB,2);
RGBy = size(RGB,1);

%% Convert LiDAR information to IMG format

LiDARimg = zeros(1,1, 'uint8');
for i = 1:size(masterinfo,1)
    LiDARimg(masterinfo(i,2),masterinfo(i,1),1) = masterinfo(i,7); 
    LiDARimg(masterinfo(i,2),masterinfo(i,1),2) = masterinfo(i,8);
    LiDARimg(masterinfo(i,2),masterinfo(i,1),3) = masterinfo(i,9); 
end

%% Canny Edge Detection

LiDARimg = im2gray(LiDARimg);
LiDARedge = edge(LiDARimg,'Canny',0.4);             %canny edge detection
LiDARedge = LiDARedge*1;
LiDARedge = imgaussfilt(LiDARedge,0.1);           %apply gaussian blur with threshold
figure()
imshowpair(LiDARimg,LiDARedge,'montage')

RGB = im2gray(RGB);

RGBedge = edge(RGB,'Canny',0.1);                %canny edge detection
RGBedge = RGBedge*1;
RGBedge = imgaussfilt(RGBedge,0.1);             %apply gaussian blur
RGBedge = RGBedge.*4;
figure()
imshowpair(RGB,RGBedge,'montage')

%% Optimization
Aineq = [];
Bineq = [];
Aeq = [];
Beq = [];
lb = [-50;-50;-10;-10;-10;0;0;0.3];
ub = [50;50;10;10;10;1;1;1];
%% Genetric Algorithm
x = ga(@cannyopt,9,[],[],[],[],lb,ub,[],x0);

%% Transformation Matrix Generation 

% Optimized parameters are assigned and transformation matrix is created
translation = x(1:2);
translation_x = translation(1);
translation_y = translation(2);
translation_z = 0;
translation_matrix = [1 0 0 translation_x; 0 1 0 translation_y; 0 0 1 translation_z; 0 0 0 1];

rotation = x(3:5);
rotation_x = rotation(1);
rotation_y = rotation(2);
rotation_z = rotation(3);
rotation_x = deg2rad(rotation_x);
rotation_y = deg2rad(rotation_y);
rotation_z = deg2rad(rotation_z);
rotation_matrixx = [1 0 0 0; 0 cos(rotation_x) -sin(rotation_x) 0; 0 sin(rotation_x) cos(rotation_x) 0; 0 0 0 1];
rotation_matrixy = [cos(rotation_y) 0 sin(rotation_y) 0; 0 1 0 0; -sin(rotation_y) 0 cos(rotation_y) 0; 0 0 0 1];
rotation_matrixz = [cos(rotation_z) -sin(rotation_z) 0 0; sin(rotation_z) cos(rotation_z) 0 0; 0 0 1 0; 0 0 0 1];

shear = x(6:7);
shxy = shear(1);
shxz = 0;
shyx = shear(2);
shyz = 0;
shzx = 0;
shzy = 0;
yzshear = [1 0 0 0; shxy 1 0 0; shxz 0 1 0; 0 0 0 1];
xzshear = [1 shyx 0 0; 0 1 0 0; 0 shyz 1 0; 0 0 0 1];
xyshear = [1 0 shzx 0; 0 1 shzy 0; 0 0 1 0; 0 0 0 1];

scale = x(8:9);
scale_x = scale(1);
scale_y = scale(2);
scale_z = 1;

scale_matrix = [scale_x 0 0 0; 0 scale_y 0 0; 0 0 scale_z 0; 0 0 0 1];

p = translation_matrix*rotation_matrixx*rotation_matrixy*rotation_matrixz*yzshear*xzshear*xyshear*scale_matrix;     
%transformation matrix
newRGB = zeros(LiDARy,LiDARx,'uint8');
%% Pixelwise transformation using the transformation matrix

for i = 1:RGBx
    for j = 1:RGBy
        coordinates = [i j 0 1]';
        
        newcoordinates = ceil(p*coordinates);
        if newcoordinates(1) <= 0
            newcoordinates(1) = 1;
        end
        if newcoordinates(2) <= 0
            newcoordinates(2) = 1;
        end
        xcoordinate = newcoordinates(1,1);
        ycoordinate = newcoordinates(2,1);

        newRGB(ycoordinate,xcoordinate) = RGB(j,i);     %assign new coordinates
    end
end
figure()
imshow(newRGB)

%% Overlay

overlay = (newRGB(1:100,1:100).*0.5+LiDARimg.*0.5);
figure()
imshow(overlay)


