function [] = calculations()
% Models the kinematics of a 3-arm Delta robot.

% Design Parameters (inches)
global w; % Width of tool equilateral traingle
global W; % Width of base equilateral triangle
global l; % Length of short arm
global L; % length of long arm
w = 4;
W = 10;
l = 5;
L = 12;

radtodeg(inverse_kinematics([0, 0, -8]))

    function [angles] = inverse_kinematics(T)
    % Returns the inverse kinematics 'angles' = [tt1, tt2, tt3] in response
    % to a desired tool position 'T' = [Tx, Ty, Tz].
    [tt1, tt2, tt3] = deal([]);
    syms tt1 tt2 tt3;
    lim = degtorad(80); % tt1, tt2, tt3 must be in [-80 deg, 80 deg].
    C1 = [T(1) + w/sqrt(3), T(2), T(3)];
    C2 = [T(1) - w/(2 * sqrt(3)), T(2) + w/2, T(3)];
    C3 = [T(1) - w/(2 * sqrt(3)), T(2) - w/2, T(3)];
    
    B1 = [W/sqrt(3) + l * cos(tt1), 0, l * sin(tt1)];
    B2 = [-W/(2 * sqrt(3))-l * cos(tt2)/2, ...
           W/2 + l * sqrt(3) * cos(tt2) / 2, l * sin(tt2)];
    B3 = [-W/(2 * sqrt(3))-l * cos(tt3)/2, ...
          -W/2 - l * sqrt(3) * cos(tt3) / 2, l * sin(tt3)];
      
    eqn1 = L == norm(C1 - B1);
    eqn2 = L == norm(C2 - B2);
    eqn3 = L == norm(C3 - B3);
    
    S = vpasolve([eqn1, eqn2, eqn3], [tt1, tt2, tt3], ...
        [-lim lim; -lim lim; -lim lim]);
    try
        angles = transpose([S.tt1, S.tt2, S.tt3]);
    catch err
        angles = transpose([NaN, NaN, NaN]); % Outside the workspace        
    end

    end
end
