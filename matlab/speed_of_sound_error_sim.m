v_max = 1.5;
v = v_max/2; % assume travelling half of max speed
T = 60; % 1 min between readings
d = T*v; % distance travelled between readings

% assume B drift is negligible
B1_true = [0,0];
B2_true = [0,0];
B3_true = [0,0];

% A positions
Ax = 100;
A1_true = [Ax,0];
A2_true = [Ax+d/sqrt(2),d/sqrt(2)];
A3_true = [Ax+2*d/sqrt(2),0];

% error
e_DVL = 0.001; % m/s
e_compass = deg2rad(0.5); % rad
v_sound = 1500; 

% calculate A vectors
A1_A2_true = A2_true - A1_true;
A2_A3_true = A3_true - A2_true;

iterations = 1000;
max_c_error = 50;
e = zeros(iterations,max_c_error);
for e_speed_of_sound = 0:max_c_error
for x = 1:iterations
    % add error
    t1 = norm(A1_true - B1_true)/normrnd(v_sound,e_speed_of_sound);
    t2 = norm(A2_true - B2_true)/normrnd(v_sound,e_speed_of_sound);
    t3 = norm(A3_true - B3_true)/normrnd(v_sound,e_speed_of_sound);
    A1_A2 = addError(A1_A2_true, e_compass, e_DVL*T);
    A2_A3 = addError(A2_A3_true, e_compass, e_DVL*T);

    D1 = v_sound*t1;
    D2 = v_sound*t2;
    D3 = v_sound*t3;

    % calculate vector positions
    [A11, A12] = vectorInCircles(A1_A2, D1, B1_true, D2, B2_true);
    [A21, A22] = vectorInCircles(A2_A3, D2, B2_true, D3, B3_true);

    % the solution is where the tip of the first arrow most closely matches
    % the base of the second arrow
    A1 = [A11; A12];
    A2 = [A21; A22];
    diffs = zeros(2);

    for i = 1:2
        for j = 1:2
            diffs(i,j) = norm((A1(i,:) + A1_A2) - A2(j,:));
        end
    end

    [error, idx] = min(diffs,[],'all','linear');
    [row,col] = ind2sub(size(diffs),idx);

    A3_error = norm(A2(col,:) + A2_A3 - A3_true);
    e(x,e_speed_of_sound+1) = A3_error;
    
%     % plotting
%     figure(1);
%     clf(1);
%     viscircles([B1_true; B2_true; B3_true],[D1, D2, D3],'LineWidth',1);
%     axis equal
%     hold on;
%     plot([B1_true(1) B2_true(1) B3_true(1)],[B1_true(2) B2_true(2) B3_true(2)],'go');
%     plot(A1_true(1), A1_true(2),'ro');
%     plot(A2_true(1), A2_true(2),'ro');
%     plot(A3_true(1), A3_true(2),'ro');
%     
%     quiver(A11(1), A11(2), A1_A2(1), A1_A2(2), 0,'b')
%     quiver(A12(1), A12(2), A1_A2(1), A1_A2(2), 0,'b')
%     
%     quiver(A21(1), A21(2), A2_A3(1), A2_A3(2), 0,'g')
%     quiver(A22(1), A22(2), A2_A3(1), A2_A3(2), 0,'g')
%     
%     plot(A1(row,1),A1(row,2),'kx')
%     plot(A2(col,1),A2(col,2),'kx')
%     pause();
end
end

plot(0:max_c_error, mean(e))

function ve = addError(v, rotation_error, translation_error)
[v_theta, v_r] = cart2pol(v(1), v(2));
v_theta = normrnd(v_theta, rotation_error);
v_r = normrnd(v_r, translation_error);
[vx, vy] = pol2cart(v_theta, v_r);
ve = [vx, vy];
end

function [solution1, solution2] = vectorInCircles(v, d1, c1, d2, c2)
v = v - (c2 - c1);
theta = acos((d1^2 + norm(v)^2 - d2^2)/(2*d1*norm(v)));
vector_angle = atan2(v(2), v(1));
[x1, y1] = pol2cart(theta+vector_angle, d1);
[x2, y2] = pol2cart(-theta+vector_angle, d1);
solution1 = [-x1, -y1] + c1; % first solution for A1
solution2 = [-x2, -y2] + c1; % second solution for A1
end