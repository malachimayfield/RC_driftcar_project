% Runs a drift simulation without a controller

close all; clc;

% Parameters of RC car in paper, probably a good place to start
m = 2.04;       % Mass [kg]
Lr = .1087;     % Distance from CM to rear axle [m]
Lf = .1513;     % Distance from CM to front axle [m]
Iz = 0.03;      % Yaw moment of inertia [kg/m^2]
Cr = 127.77;    % Rear tire cornering stiffness
Cf = 47.86;     % Front tire cornering stiffness
mu_r = 0.33;    % Rear tire coefficient of friction
mu_f = 0.35;    % Front tire coefficient of friction
g = 9.81;       % Acceleration due to gravity [m/s^2]

% Initial conditions (delta=-15deg equilibrium point)
vx_init     = 1.5 - 0.3;
vy_init     = -0.5699 + 0.1;
r_init      = 1.9604 + 0.1;
x_pos_init  = 0;
y_pos_init  = 0;
angle_init  = pi/2;

duration = 5;

% Integrate
state_init = [ vx_init; vy_init; r_init; x_pos_init; y_pos_init; angle_init];

sim_result = sim('closed_loop_sim', duration);
states = sim_result.states.Data;
u = sim_result.u.Data;

t_vec = sim_result.tout;

% Extract state
vx = squeeze(states(1,1,:));
vy = squeeze(states(2,1,:));
r  = squeeze(states(3,1,:));
size(u)
delta = squeeze(u(:,1));
throttle = squeeze(u(:,2));

% Sideslip angle
beta = atan(vy./vx);

V_body = [vx,vy]';
V_inertial = zeros(2,length(r));
pos = zeros(2,length(r));
theta = zeros(length(r),1);
theta(1) = pi/2;

dt = t_vec(2)-t_vec(1);

% Manually integrate body velocity and yaw rate to get poition and angle
for i = 2:length(r)

    theta(i) = theta(i-1)+r(i)*dt;
    
    % Rotation matrix from body to intrtial
    T = [cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))];
   
    V_inertial(:,i) = T*V_body(:,i);

    pos(:,i) = pos(:,i-1) + V_inertial(:,i) * dt;

end

figure(1)
ax1 = subplot(5,1,1);
hold on;
plot(t_vec, vx, "DisplayName", "Vx")
plot(t_vec, vy, "DisplayName", "Vy")
legend()
ylabel("Velocity (m/s)")

ax2 = subplot(5,1,2);
plot(t_vec, rad2deg(r), "DisplayName", "r")
ylabel("Yaw rate (deg/s)")

ax3 = subplot(5,1,3);
plot(t_vec, rad2deg(beta), "DisplayName", "Beta")
ylabel("Sideslip (deg)")

ax4 = subplot(5,1,4);
plot(sim_result.u.Time, rad2deg(delta), "DisplayName", "delta")
ylabel("Steering Angle (deg)")

ax5 = subplot(5,1,5);
plot(sim_result.u.Time, throttle, "DisplayName", "throttle")
ylabel("Throttle (Fxr) (N)")

xlabel("Time (sec)")


rad2deg(delta)

linkaxes([ax1,ax2,ax3,ax4,ax5],'x')

figure(2)
hold on
plot(pos(1,:), pos(2,:))

% How big to draw car
scale = 1;
% Number of cars to draw
num_lines = 100;

step_size = round(length(r)/num_lines);
for i = 1:step_size:length(r)
    plot([pos(1,i) - scale*Lr*cos(theta(i)), pos(1,i) + scale*Lf*cos(theta(i))], [pos(2,i)- scale*Lr*sin(theta(i)), pos(2,i)+ scale*Lf*sin(theta(i))], 'k')
end

xlim([-2,4])
ylim([-1,5])

