%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File: numerical_simulation.m
% Author: Wenyu
% Date: 06/16/2019
% Description: simulate the kinematic motion of four wheel vehicle by a
% two-wheel differential mobile vehicle with sine inputs
% Version: v1.0 [06/16/2019][Wenyu]
%          v1.1 [06/19/2019][Zhenyu] add PID control 
%          v1.2 [06/21/2019][Zhenyu] add robort control
%          v1.3 [06/21/2019][Zhenyu] add a comparison of the three methods
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

% init 4 wheel vehicle pose for nonlinear controller
x4_non = 0;
y4_non = 0;
theta4_non = 0;

% init 2 wheel vehicle pose for nonlinear controller
x2_non = 0;
y2_non = 0;
theta2_non = 0;

% init real pose for nonlinear controller
x_non = 0;
y_non = 0;
theta_non = 0;

%init PID

Kp_v=[100 100 100];
Ki_v=[0.5 0.5 0.5];
Kd_v=[2 2 2];
Kp_w=[25 25 25];
Ki_w=[0.5 0.5 0.5];
Kd_w=[2 2 2];

% init 4 wheel vehicle pose for PID
x4_PID = 0;
y4_PID = 0;
theta4_PID = 0;

% init 2 wheel vehicle pose for PID
x2_PID = 0;
y2_PID = 0;
theta2_PID = 0;

% init real pose for PID
x_PID = 0;
y_PID = 0;
theta_PID = 0;

% initialize pid data
dx_before = x4_PID - x_PID;
dy_before = y4_PID - y_PID;
dth_before = theta4_PID - theta_PID;
dx_add = dx_before;
dy_add = dy_before;
dth_add = dth_before;


% init robort parameters
kalpha = 5;
krou = 6;

% init 4 wheel vehicle pose for robot control
x4_robo = 0;
y4_robo = 0;
theta4_robo = 0;

% init 2 wheel vehicle pose for robot control
x2_robo = 0;
y2_robo = 0;
theta2_robo = 0;

% init real pose for robot control
x_robo = 0;
y_robo = 0;
theta_robo = 0;

% init iteration times
amount = 1000;
t = 1 : amount;
TIME = t * dt;

% init input function (simple)
VR = 8.5 + 8.5 * sin(t / amount * 2 * pi);
PHI = pi / 6 * sin(t / amount * 2 * pi + pi / 2);

% init input function (extreme)
%VR = 15 + 15 * sin(t / amount * 5 * pi);
%PHI = pi / 6 * sin(t / amount * 5 * pi + pi / 2);
%% simulation for nonlinear controller, PID controller and robotic controller
for i = 1 : amount
    disp(i);
    %% input
    x4_non = x_non;
    y4_non = y_non;
    theta4_non = theta_non;
    
    vr_non = VR(i);
    phi_non = PHI(i);
    
    %% expected 4 wheel pose derivatives
    dth4_non = vr_non * tan(phi_non) / L;
    dx4_non = vr_non * cos(theta4_non) - s * dth4_non * sin(theta4_non);
    dy4_non = vr_non * sin(theta4_non) + s * dth4_non * cos(theta4_non);
    
    % accumulated pose
    x4_non = x4_non + dx4_non * dt;
    y4_non = y4_non + dy4_non * dt;
    theta4_non = theta4_non + dth4_non * dt;
    
    %% transform into 2 wheel
    % errors
    dx2_non = x4_non - x_non;
    dy2_non = y4_non - y_non;
    dth2_non = theta4_non - theta_non;
    % nonlinear controller
    v_non = vr_non * cos(dth2_non) + kx * dx2_non;
    w_non = vr_non * tan(phi_non) / L + ky * vr_non * dy2_non + kth * sin(dth2_non);
    % accumulated pose
    x2_non = x2_non + v_non * cos(theta2_non) * dt;
    y2_non = y2_non + v_non * sin(theta2_non) * dt;
    theta2_non = theta2_non + w_non * dt;
    
    %% output
    % real pose
    x_non = x2_non;
    y_non = y2_non;
    theta_non = theta2_non;
    
    % record results
    Rx_non(i) = x_non;
    Ry_non(i) = y_non;
    Rth_non(i) = theta_non;
    
%     while Rth(i) > pi
%         Rth(i) = Rth(i) - 2 * pi;
%     end
%     
%     while Rth(i) < -pi
%         Rth(i) = Rth(i) + 2 * pi;
%     end
    
    Rx4_non(i) = x4_non;
    Ry4_non(i) = y4_non;
    Rth4_non(i) = theta4_non;
    
%     while Rth4(i) > pi
%         Rth4(i) = Rth4(i) - 2 * pi;
%     end
%     
%     while Rth4(i) < -pi
%         Rth4(i) = Rth4(i) + 2 * pi;
%     end
    
    Rxe_non(i) = x2_non - x4_non;
    Rye_non(i) = y2_non - y4_non;
    Rthe_non(i) = theta2_non - theta4_non;
    
end
for i = 1 : amount
    disp(i);
    %% input
    x4_PID = x_PID;
    y4_PID = y_PID;
    theta4_PID = theta_PID;
    
    vr_PID = VR(i);
    phi_PID = PHI(i);
    
    %% expected 4 wheel pose derivatives
    dth4_PID = vr_PID * tan(phi_PID) / L;
    dx4_PID = vr_PID * cos(theta4_PID) - s * dth4_PID * sin(theta4_PID);
    dy4_PID = vr_PID * sin(theta4_PID) + s * dth4_PID * cos(theta4_PID);
    
    % accumulated pose
    x4_PID = x4_PID + dx4_PID * dt;
    y4_PID = y4_PID + dy4_PID * dt;
    theta4_PID = theta4_PID + dth4_PID * dt;
    
    %% transform into 2 wheel
    % errors
    dx2_PID = x4_PID - x_PID;
    dy2_PID = y4_PID - y_PID;
    dth2_PID = theta4_PID - theta_PID;
    dP = [dx2_PID ; dy2_PID ; dth2_PID];
    dD = [dx2_PID - dx_before ; dy2_PID - dy_before ; dth2_PID - dth_before];
    dI = [dx_add ; dy_add ; dth_add];
    % PID controller
    v_PID = Kp_v * dP + Kd_v * dD + Ki_v * dI;
    w_PID = Kp_w * dP + Kd_w * dD + Ki_w * dI;    
    % accumulated pose
    x2_PID = x2_PID + v_PID * cos(theta2_PID) * dt;
    y2_PID = y2_PID + v_PID * sin(theta2_PID) * dt;
    theta2_PID = theta2_PID + w_PID * dt;
    
    %% output
    % real pose
    x_PID = x2_PID;
    y_PID = y2_PID;
    theta_PID = theta2_PID;
    
    % record results
    Rx_PID(i) = x_PID;
    Ry_PID(i) = y_PID;
    Rth_PID(i) = theta_PID;
    
%     while Rth(i) > pi
%         Rth(i) = Rth(i) - 2 * pi;
%     end
%     
%     while Rth(i) < -pi
%         Rth(i) = Rth(i) + 2 * pi;
%     end
    
    Rx4_PID(i) = x4_PID;
    Ry4_PID(i) = y4_PID;
    Rth4_PID(i) = theta4_PID;
    
%     while Rth4(i) > pi
%         Rth4(i) = Rth4(i) - 2 * pi;
%     end
%     
%     while Rth4(i) < -pi
%         Rth4(i) = Rth4(i) + 2 * pi;
%     end
    
    Rxe_PID(i) = x2_PID - x4_PID;
    Rye_PID(i) = y2_PID - y4_PID;
    Rthe_PID(i) = theta2_PID - theta4_PID;
    
    dx_before = dx2_PID;
    dy_before = dy2_PID;
    dth_before = dth2_PID;

    dx_add = dx_add + dx_before;
    dy_add = dy_add + dy_before;
    dth_add = dth_add + dth_before;


end
for i = 1 : amount
    disp(i);
    %% input
    x4_robo = x_robo;
    y4_robo = y_robo;
    theta4_robo = theta_robo;
    
    vr_robo = VR(i);
    phi_robo = PHI(i);
    
    %% expected 4 wheel pose derivatives
    dth4_robo = vr_robo * tan(phi_robo) / L;
    dx4_robo = vr_robo * cos(theta4_robo) - s * dth4_robo * sin(theta4_robo);
    dy4_robo = vr_robo * sin(theta4_robo) + s * dth4_robo * cos(theta4_robo);
    
    % accumulated pose
    x4_robo = x4_robo + dx4_robo * dt;
    y4_robo = y4_robo + dy4_robo * dt;
    theta4_robo = theta4_robo + dth4_robo * dt;
    
    %% transform into 2 wheel
    % errors
    dx2_robo = x4_robo - x_robo;
    dy2_robo = y4_robo - y_robo;
    dth2_robo = theta4_robo - theta_robo;
    % robot controller
    rou = sqrt(dx2_robo * dx2_robo + dy2_robo * dy2_robo);
    alpha = abs(dth2_robo);
    v_robo = krou * rou;
    w_robo = kalpha * alpha;
    
    % accumulated pose
    x2_robo = x2_robo + v_robo * cos(theta2_robo) * dt;
    y2_robo = y2_robo + v_robo * sin(theta2_robo) * dt;
    theta2_robo = theta2_robo + w_robo * dt;
    
    %% output
    % real pose
    x_robo = x2_robo;
    y_robo = y2_robo;
    theta_robo = theta2_robo;
    
    % record results
    Rx_robo(i) = x_robo;
    Ry_robo(i) = y_robo;
    Rth_robo(i) = theta_robo;
    
%     while Rth(i) > pi
%         Rth(i) = Rth(i) - 2 * pi;
%     end
%     
%     while Rth(i) < -pi
%         Rth(i) = Rth(i) + 2 * pi;
%     end
    
    Rx4_robo(i) = x4_robo;
    Ry4_robo(i) = y4_robo;
    Rth4_robo(i) = theta4_robo;
    
%     while Rth4(i) > pi
%         Rth4(i) = Rth4(i) - 2 * pi;
%     end
%     
%     while Rth4(i) < -pi
%         Rth4(i) = Rth4(i) + 2 * pi;
%     end
    
    Rxe_robo(i) = x2_robo - x4_robo;
    Rye_robo(i) = y2_robo - y4_robo;
    Rthe_robo(i) = theta2_robo - theta4_robo;
    

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

%  X, Y and theta errors while red line corresponds to nonlinear controller, 
%  blue line corresponds to PID controller, 
%  black line corresponds to robotic controller
figure;
hold on;

subplot(2, 1, 1);
hold on;
plot(TIME, Rxe_non, 'r');
plot(TIME, Rxe_PID, 'b');
plot(TIME, Rxe_robo, 'k');
xlabel('Time (s)')
ylabel('X (m)')

subplot(2, 1, 2);
hold on;
plot(TIME, Rye_non, 'r');
plot(TIME, Rye_PID, 'b');
plot(TIME, Rye_robo, 'k');
xlabel('Time (s)')
ylabel('Y (m)')

figure;
subplot(2, 1, 1);
hold on;
plot(TIME, Rthe_non, 'r');
plot(TIME, Rthe_PID, 'b');
plot(TIME, Rthe_robo, 'k');
xlabel('Time (s)')
ylabel('\theta (rad)')