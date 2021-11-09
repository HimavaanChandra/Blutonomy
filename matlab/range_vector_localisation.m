B1_true = [0,0];
B2_true = [0,0.5];
B3_true = [0,1];

A1_true = [0.75,1];
A2_true = [2, 2];
A3_true = [2, 4];

A1_A2 = A2_true - A1_true;
A2_A3 = A3_true - A2_true;
D1 = norm(A1_true - B1_true);
D2 = norm(A2_true - B2_true);
D3 = norm(A3_true - B3_true);

% subtract movement of B from movement of A to get net vector
%v1 = A1_A2 - (B2_true - B1_true)
[A11, A12] = localiseVectors(A1_A2, D1, B1_true, D2, B2_true)

% same for second pair
% v2 = A2_A3 - (B3_true - B2_true)
[A21, A22] = localiseVectors(A2_A3, D2, B2_true, D3, B3_true)
% shift result by B2
% A21 = A21 + B2_true;
% A22 = A22 + B2_true;

% the solution is where the tip of the first arrow most closely matches
% the base of the second arrow
A1 = [A11; A12];
A2 = [A21; A22];
diffs = zeros(2);

for i = 1:2
    for j = 1:2
        diffs(i,j) = norm((A1(i,:) + A1_A2) - A2(j,:))
    end
end

[error, idx] = min(diffs,[],'all','linear')
[row,col] = ind2sub(size(diffs),idx)

% plotting
figure(1);
viscircles([B1_true; B2_true; B3_true],[D1, D2, D3],'LineWidth',1);
axis equal
hold on;
plot([B1_true(1) B2_true(1) B3_true(1)],[B1_true(2) B2_true(2) B3_true(2)],'go');
% plot(A1_true(1), A1_true(2),'ro');
% plot(A2_true(1), A2_true(2),'ro');
% plot(A3_true(1), A3_true(2),'ro');
% 
% quiver(A11(1), A11(2), A1_A2(1), A1_A2(2), 0,'b')
% quiver(A12(1), A12(2), A1_A2(1), A1_A2(2), 0,'b')
% 
% quiver(A21(1), A21(2), A2_A3(1), A2_A3(2), 0,'g')
% quiver(A22(1), A22(2), A2_A3(1), A2_A3(2), 0,'g')
% 
% plot(A1(row,1),A1(row,2),'kx')
% plot(A2(col,1),A2(col,2),'kx')

function [solution1, solution2] = localiseVectors(v, d1, b1, d2, b2)
v = v - (b2 - b1); % net movement vector
phi = acos((d1^2 + norm(v)^2 - d2^2)/(2*d1*norm(v))); % angle between v and B1
vector_angle = atan2(v(2), v(1));
[x1, y1] = pol2cart(phi+vector_angle, d1); % first solution for B1 wrt A1
[x2, y2] = pol2cart(-phi+vector_angle, d1); % second solution for B1 wrt A1
solution1 = [-x1, -y1] + b1; % first solution for A1 wrt B1
solution2 = [-x2, -y2] + b1; % second solution for A1 wrt B1
end
