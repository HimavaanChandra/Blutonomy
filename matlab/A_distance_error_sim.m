v = 2.057778; % assume travelling at 4 knots
T = 60; % 1 min between readings
d = T*v; % distance travelled between readings

% assume B does not move
B1_true = [0,0];
B2_true = [0,0];
B3_true = [0,0];

% error
e_DVL = 0.001; % m/s
e_compass = deg2rad(1); % rad
e_temp = 1;
temp = 20;
salinity = 35;
depth = 5;
c_base = speed_of_sound(temp,salinity,depth);

iterations = 1000;
max_Ax = 2000;
e = zeros(iterations, max_Ax);
for Ax = 1:max_Ax
    Ax
    % A positions
    A1_true = [Ax,0];
    A2_true = [Ax+d/sqrt(2),d/sqrt(2)];
    A3_true = [Ax+2*d/sqrt(2),0];

    % calculate A vectors
    A1_A2_true = A2_true - A1_true;
    A2_A3_true = A3_true - A2_true;

    for x = 1:iterations
        v_sound = [speed_of_sound(normrnd(temp,e_temp),salinity,depth),...
        speed_of_sound(normrnd(temp,e_temp),salinity,depth),...
        speed_of_sound(normrnd(temp,e_temp),salinity,depth)];
        
        % add error
        t1 = norm(A1_true - B1_true)/v_sound(1);
        t2 = norm(A2_true - B2_true)/v_sound(2);
        t3 = norm(A3_true - B3_true)/v_sound(3);
        A1_A2 = addError(A1_A2_true, e_compass, e_DVL*T);
        A2_A3 = addError(A2_A3_true, e_compass, e_DVL*T);

        D1 = c_base*t1;
        D2 = c_base*t2;
        D3 = c_base*t3;

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

        A1_error = norm(A1(row,:) - A1_true);
        e(x,Ax) = A1_error;
    end
end

%%
plot(1:max_Ax, mean(e),'b')
title("Localisation error vs distance between vehicles (1°C std dev temperature error)");
xlabel("Starting distance between Vehicle B to Vehicle A (m)");
ylabel("Average error (m)");
%%
hold on;
onepercent = sort(e,'descend');
onepercent = onepercent(1:0.01*iterations,:);
plot(1:max_Ax, mean(onepercent),'b:')
legend(["Mean error","1% worst case error"]);

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
