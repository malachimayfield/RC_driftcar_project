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
vx_init = 1.5;
vy_init = -0.5699;
r_init = 1.9604;

% Control input for delta=-15deg drift
u = [ -15*pi/180; 1.6071];

t_vec = linspace(0,10,1000);

% Integrate
state_init = [ vx_init; vy_init; r_init ];
[~, states] = ode45( @(t,y) drift_eom(t,y,u,g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f), t_vec, state_init);           

% Extract state
vx = states(:,1);
vy = states(:,2);
r  = states(:,3);

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
ax1 = subplot(3,1,1);
hold on;
plot(t_vec, vx, "DisplayName", "Vx")
plot(t_vec, vy, "DisplayName", "Vy")
legend()
ylabel("Velocity (m/s)")

ax2 = subplot(3,1,2);
plot(t_vec, rad2deg(r), "DisplayName", "r")
ylabel("Yaw rate (deg/s)")

ax3 = subplot(3,1,3);
plot(t_vec, rad2deg(beta), "DisplayName", "Beta")
ylabel("Sideslip (deg)")
xlabel("Time (sec)")

linkaxes([ax1,ax2,ax3],'x')

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

