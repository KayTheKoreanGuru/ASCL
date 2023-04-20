function [weight] = cannyopt(x)
% Objective function of the optimization problem
% Inputs:
%   x - Design parameter, [R_inner R_outer]
%
% Outputs:
%   weight - weight of the spar for given geometry information
%
% Weight of the spar is calculated through the calculation of volume.
% Volume is then multiplied with the density (rho) to obtain mass
%--------------------------------------------------------------------------
global LiDARedge RGBedge LiDARx LiDARy RGBx RGBy

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
newRGBedge = zeros(RGBy,RGBx);

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

        newRGBedge(ycoordinate,xcoordinate) = RGBedge(j,i);
    end
end

weight = -sum(LiDARedge.*newRGBedge(1:LiDARy,1:LiDARx),'all');
