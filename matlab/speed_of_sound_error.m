clear all

% Empiracal Speed of Sound Calculation
% Constants
a1 = 1448.96;
a2 = 4.591;
a3 = -5.304 * 10^-2;
a4 = 2.374*10^-4;
a5 = 1.34;
a6 = 1.630 * 10^-2;
a7 = 1.675 * 10^-7;
a8 = -1.025 * 10^-2;
a9 = -7.139 * 10^-13;

%%

T1 = 23; % Water Temperature in degrees celcius
S1 = 37.2; % Parts per thousand
z1 = 0; % Depth in meters

c1 = a1 + a2*T1 + a3*T1^2 + a5*(S1-35) + a6*z1 + a7*z1^2 + a8*T1*(S1-35) + a9*T1*z1^3 % Speed of Sound Calculation

T2 = 28; % Water Temperature in degrees celcius
S2 = 37.2; % Parts per thousand
z2 = 0; % Depth in meters

c2 = a1 + a2*T2 + a3*T2^2 + a5*(S2-35) + a6*z2 + a7*z2^2 + a8*T2*(S2-35) + a9*T2*z2^3 % Speed of Sound Calculation

% distance = 10:1:1000
time = 0:0.1:1;

% for i = 1:numel(distance)

for i = 1:numel(time)

distance1(i) = c1*time(i);
distance2(i) = c2*time(i);
distance_error(i) = abs(distance2(i)-distance1(i));

error_percent(i) = (distance_error(i)/distance2(i))*100;

end

figure(1);
plot(time,distance_error)
title('Distance Error vs Time of Flight')
xlabel('Time (Seconds)')
ylabel('Distance Error (Meters)')

figure(2);
plot(time,error_percent)
ylim([0, 1])
title('Distance Error Percent vs Time of Flight')
xlabel('Time (Seconds)')
ylabel('Distance Error (Percentage)')

%%
error_percent = [];
distance = 1000

% depth = 0:100:distance;
depth = 0;
iter = 0;

for z = depth
T1 = 23; % Water Temperature in degrees celcius
S1 = 37.2; % Parts per thousand
z1 = z; % Depth in meters

c1 = a1 + a2*T1 + a3*T1^2 + a5*(S1-35) + a6*z1 + a7*z1^2 + a8*T1*(S1-35) + a9*T1*z1^3 % Speed of Sound Calculation

T2 = 23:1:28; % Water Temperature in degrees celcius
S2 = 37.2; % Parts per thousand
z2 = z; % Depth in meters

% distance = 100
% time1 = distance/c1;
% error1(:) = [];
%  error2(:) = [];

time = 1;

for i = 1:numel(T2)
c2 = a1 + a2*T2(i) + a3*T2(i)^2 + a5*(S2-35) + a6*z2 + a7*z2^2 + a8*T2(i)*(S2-35) + a9*T2(i)*z2^3 % Speed of Sound Calculation

distance1(i) = c1*time;
distance2(i) = c2*time;
distance_error(i) = abs(distance2(i)-distance1(i));
error_percent(i) = (distance_error(i)/distance2(i))*100;

end

hold on;
figure(3);
plot(T2,error_percent)
title('Distance Error vs Varying Ocean Temperature Difference Between Vehicles')
xlabel('Change in Temperature Between AUVs (Degrees C)')
ylabel('Error Percentage')
depth = z;
depth = sprintf('Depth: %.1f',depth);
legend(depth);
hold off;

end

