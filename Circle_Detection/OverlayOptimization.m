%% clear workspace
clc
clear all
close all

%% Set global variables and initial points

global imagePoints imagePoints1
x0 = zeros(1,10)';



%% Read Image from the directory

imds = imageDatastore("Camera1\*.jpg"); %store image data from file
I = imds.Files{1,1};                    %save image file name

Icamera = imread(I);                    %read image

SizeCamera = size(Icamera);             %determination of Image Size
pixelx = SizeCamera(1,2);               %pixel size X
pixely = SizeCamera(1,1);               %pixel size Y
blankimage = zeros(SizeCamera(1,1),SizeCamera(1,2), 'uint8');     %create blank image of same size
blankimage2 = zeros(SizeCamera(1,1),SizeCamera(1,2), 'uint8');

phoneimds = imageDatastore("PhoneCam\*.jpg");       %store phone camera data
phone = phoneimds.Files{1,1};

Iphone = imread(phone);                 %read image
Iphone = rgb2gray(Iphone);              %grayscale conversion

%% Detect Checkerboard in Image

[imagePoints,boardSize] = detectCheckerboardPoints(Icamera);        %detect checkerboard corners
IcameraCheckerboard = insertText(Icamera,imagePoints,1:size(imagePoints,1));
IcameraCheckerboard = insertMarker(IcameraCheckerboard,imagePoints,'o','Color','red','Size',5);
figure()
imshow(IcameraCheckerboard);

[imagePoints1,boardSize1] = detectCheckerboardPoints(Iphone);       %detect checkerboard corners
IphoneCheckerboard = insertText(Iphone,imagePoints1,1:size(imagePoints1,1));
IphoneCheckerboard = insertMarker(IphoneCheckerboard,imagePoints1,'o','Color','red','Size',5);
figure()
imshow(IphoneCheckerboard);

%% Constraint
Aineq = [];
Bineq = [];
Aeq = [];
Beq = [];
lb = [-pixelx;-pixely;-10;-10;-10;0;0];
ub = [pixelx;pixely;10;10;10;0;0];

%% Fmincon

% Optimization options, use 'active-set' algorithm
options = optimoptions('fmincon','Display','iter', ...
    'Algorithm','active-set','SpecifyConstraintGradient',true,'PlotFcns',{@optimplotfval,@optimplotfirstorderopt});

% Save optimal 'a' and function value
[xopt,fval] = fmincon(@obj,x0,Aineq,Bineq,Aeq,Beq,lb,ub,[],options);

%% Optimized Extrinsic Parameter Assign

translation_x = xopt(1,1);          %x translation
translation_y = xopt(2,1);          %y translation  
translation_z = 0;                  %z translation set to 0

scale_x = xopt(8,1);                %x scale
scale_y = xopt(9,1);                %y scale
scale_z = 1;                        %nominal z scale value

rotation_x = xopt(3,1);             %x rotation in degrees
rotation_y = xopt(4,1);             %y rotation in degrees
rotation_z = xopt(5,1);             %z rotation in degrees
rotation_x = deg2rad(rotation_x);   %degree to radian conversion
rotation_y = deg2rad(rotation_y);
rotation_z = deg2rad(rotation_z);

% Nominal value 0 is assigned for unused variables
shxy = xopt(6,1);                   %XY shear
shxz = 0;               
shyx = xopt(7,1);                   %YX shear
shyz = 0;
shzx = 0;
shzy = 0;

zfar = 99;              %distance to obj
znear = 100;             %focal point

%% Image Transformation Matrix generation

translation_matrix = [1 0 0 translation_x; 0 1 0 translation_y; 0 0 1 translation_z; 0 0 0 1];
rotation_matrixx = [1 0 0 0; 0 cos(rotation_x) -sin(rotation_x) 0; 0 sin(rotation_x) cos(rotation_x) 0; 0 0 0 1];
rotation_matrixy = [cos(rotation_y) 0 sin(rotation_y) 0; 0 1 0 0; -sin(rotation_y) 0 cos(rotation_y) 0; 0 0 0 1];
rotation_matrixz = [cos(rotation_z) -sin(rotation_z) 0 0; sin(rotation_z) cos(rotation_z) 0 0; 0 0 1 0; 0 0 0 1];
yzshear = [1 0 0 0; shxy 1 0 0; shxz 0 1 0; 0 0 0 1];
xzshear = [1 shyx 0 0; 0 1 0 0; 0 shyz 1 0; 0 0 0 1];
xyshear = [1 0 shzx 0; 0 1 shzy 0; 0 0 1 0; 0 0 0 1];
scale_matrix = [scale_x 0 0 0; 0 scale_y 0 0; 0 0 scale_z 0; 0 0 0 1];

% Individual matrix are multiplied to generate the transformation matrix
p = translation_matrix*rotation_matrixx*rotation_matrixy*rotation_matrixz*yzshear*xzshear*xyshear*scale_matrix;

for i = 1:pixelx
    for j = 1:pixely
        coordinates = [i j zfar 1];
        newcoordinates = p*coordinates';
        if round(newcoordinates(1,1)) <= 0
            xcoordinate = 1;
        else
             xcoordinate = round(newcoordinates(1,1));
        end

        if round(newcoordinates(2,1))<=0
            ycoordinate = 1;
        else

        ycoordinate = round(newcoordinates(2,1));
        end
        if round(newcoordinates(3,1))<=0
            zcoordinate = 1;
        else
            zcoordinate = round(newcoordinates(3,1));
        end
    
       
        
        blankimage(ycoordinate,xcoordinate)=Icamera(j,i);   % assign intensity on a blank image

    end
end
figure()
imshow(blankimage);


%% Crop

%images are cropped because MatLab allows only same size images to be
%overlayed
IphoneGray = Iphone;

IphoneGray = [IphoneGray zeros(size(IphoneGray,1), size(blankimage,2)-size(IphoneGray,2))];
IphoneGray = [IphoneGray; zeros(size(blankimage,1)-size(IphoneGray,1), size(IphoneGray,2))];

figure()
imshow(IphoneGray);
%% Overlay

newimage = IphoneGray.*(1/2)+blankimage.*(1/2);
figure()
imshow(newimage)