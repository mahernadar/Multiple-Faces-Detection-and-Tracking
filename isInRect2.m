function [PT_ARRAY_FINAL] = isInRect2(A, B, C, D, PT_ARRAY)
% CV 2015 short project
%% Dimitri LECA - Maher NADAR
% input : 4 points A, B, C, D and an array of points.
% The goal of this function is to return an array of points with ONLY the
% points which are inside the polygon (here a rectangle) formed by A, B, C
% and D.

firstPointDetected = false;

% For each edge, we compute a, b, c such as the cartesian line equation is
% a*x + b*y + c = 0;

vec_ab = [B(1)-A(1),B(2)-A(2)];
vec_bc = [C(1)-B(1),C(2)-B(2)];
vec_cd = [D(1)-C(1),D(2)-C(2)];
vec_da = [A(1)-D(1),A(2)-D(2)];

% Computing a
a_ab = -vec_ab(2);
a_bc = -vec_bc(2);
a_cd = -vec_cd(2);
a_da = -vec_da(2);

% Computing b
b_ab = vec_ab(1);
b_bc = vec_bc(1);
b_cd = vec_cd(1);
b_da = vec_da(1);

% Computing c
c_ab = -a_ab*A(1) - b_ab*A(2);
c_bc = -a_bc*B(1) - b_bc*B(2);
c_cd = -a_cd*C(1) - b_cd*C(2);
c_da = -a_da*D(1) - b_da*D(2);

% We also compute the distance AB, BC, CD and DA
AB = sqrt((A(1)-B(1))^2 + (A(2)-B(2))^2);
BC = sqrt((C(1)-B(1))^2 + (C(2)-B(2))^2);
CD = sqrt((C(1)-D(1))^2 + (C(2)-D(2))^2);
DA = sqrt((D(1)-A(1))^2 + (D(2)-A(2))^2);

% To determine is a point is inside the rectangle, we will use the area
% method :
% If the sum of the areas of the triangle ABX, BCX, CDX, ADX (X is our
% point) is greater than the area of the rectangle ABCD, it means that the
% point IS NOT inside the rectangle.

areaRectangle = AB*BC;

PT_ARRAY_FINAL = [0, 0];

for i= 1:size(PT_ARRAY,1) % For each points 
    
    PT = PT_ARRAY(i,:);
    % We compute the area of each triangle, using the formula of the
    % distance from a point to the line.
    % (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line)
    
    ab_area = (abs(a_ab*PT(1) + b_ab*PT(2) + c_ab)/sqrt(a_ab^2 + b_ab^2))*AB/2;
    bc_area = (abs(a_bc*PT(1) + b_bc*PT(2) + c_bc)/sqrt(a_bc^2 + b_bc^2))*BC/2;
    cd_area = (abs(a_cd*PT(1) + b_cd*PT(2) + c_cd)/sqrt(a_cd^2 + b_cd^2))*CD/2;
    da_area = (abs(a_da*PT(1) + b_da*PT(2) + c_da)/sqrt(a_da^2 + b_da^2))*DA/2;
    
    % Computing the total area
    areaPoint = ab_area + bc_area + cd_area + da_area;

    if (areaPoint < areaRectangle + 1) % We add '+1' to create a kind of margin error;
        if(firstPointDetected == false)
            PT_ARRAY_FINAL = PT;
            firstPointDetected = true;
        else
            
            PT_ARRAY_FINAL = cat(1, PT_ARRAY_FINAL, PT); % If our point is in the rectangle, we add it to the final vector.
        end
    end
    
end



end

