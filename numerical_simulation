%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File: numerical_simulation.m
% Author: Wenyu
% Date: 06/16/2019
% Description: simulate the kinematic motion of four wheel vehicle by a
% two-wheel differential mobile vehicle with sine inputs
% Version: v1.0 [06/16/2019][Wenyu]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% init parameters
% init vehicle parameters
dt = 0.01;
L = 5;
s = 20;

% init controller gains
kx = 0.798;
ky = 0.001;
kth = 0.653;

% init 4 wheel vehicle pose
x4 = 0;
y4 = 0;
theta4 = 0;

% init 2 wheel vehicle pose
x2 = 0;
y2 = 0;
theta2 = 0;

% init real pose
x = 0;
y = 0;
theta = 0;

% init iteration times
amount = 1000;
t = 1 : amount;
TIME = t * dt;

% init input function
VR = 15 + 15 * sin(t / amount * 5 * pi);
PHI = pi / 6 * sin(t / amount * 5 * pi + pi / 2);

%% simulation
for i = 1 : amount
    disp(i);
    %% input
    x4 = x;
    y4 = y;
    theta4 = theta;
    
    vr = VR(i);
    phi = PHI(i);
    
    %% expected 4 wheel pose derivatives
    dth4 = vr * tan(phi) / L;
    dx4 = vr * cos(theta4) - s * dth4 * sin(theta4);
    dy4 = vr * sin(theta4) + s * dth4 * cos(theta4);
    
    % accumulated pose
    x4 = x4 + dx4 * dt;
    y4 = y4 + dy4 * dt;
    theta4 = theta4 + dth4 * dt;
    
    %% transform into 2 wheel
    % errors
    dx2 = x4 - x;
    dy2 = y4 - y;
    dth2 = theta4 - theta;
    
    % nonlinear controller
    v = vr * cos(dth2) + kx * dx2;
    w = vr * tan(phi) / L + ky * vr * dy2 + kth * sin(dth2);
    
    % accumulated pose
    x2 = x2 + v * cos(theta2) * dt;
    y2 = y2 + v * sin(theta2) * dt;
    theta2 = theta2 + w * dt;
    
    %% output
    % real pose
    x = x2;
    y = y2;
    theta = theta2;
    
    % record results
    Rx(i) = x;
    Ry(i) = y;
    Rth(i) = theta;
    
%     while Rth(i) > pi
%         Rth(i) = Rth(i) - 2 * pi;
%     end
%     
%     while Rth(i) < -pi
%         Rth(i) = Rth(i) + 2 * pi;
%     end
    
    Rx4(i) = x4;
    Ry4(i) = y4;
    Rth4(i) = theta4;
    
%     while Rth4(i) > pi
%         Rth4(i) = Rth4(i) - 2 * pi;
%     end
%     
%     while Rth4(i) < -pi
%         Rth4(i) = Rth4(i) + 2 * pi;
%     end
    
    Rxe(i) = x2 - x4;
    Rye(i) = y2 - y4;
    Rthe(i) = theta2 - theta4;

end

%% Illustrate results figures
% inputs
figure;
hold on;

subplot(2, 1, 1);
plot(TIME, VR, 'b');
xlabel('Time (s)');
ylabel('v_r (m/s)');

subplot(2, 1, 2);
plot(TIME, PHI, 'r');
xlabel('Time (s)')
ylabel('phi (rad)')

% X and Y direction and errors
figure;
hold on;

subplot(2, 1, 1);
hold on;
plot(TIME, Rx, 'r');
plot(TIME, Rx4, 'b');
plot(TIME, Rxe, 'k');
xlabel('Time (s)')
ylabel('X (m)')

subplot(2, 1, 2);
hold on;
plot(TIME, Ry, 'r');
plot(TIME, Ry4, 'b');
plot(TIME, Rye, 'k');
xlabel('Time (s)')
ylabel('Y (m)')

% theta and error
figure;
subplot(2, 1, 1);
hold on;
plot(TIME, Rth, 'r');
plot(TIME, Rth4, 'b');
plot(TIME, Rthe, 'k');
xlabel('Time (s)')
ylabel('\theta (rad)')
