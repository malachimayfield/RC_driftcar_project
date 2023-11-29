% Finds and plots the drifting equilibrium points

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

% Search for points with a constant Vx
vx_eq = 0.5;

options = optimset('Display','off');
figure();
hold on
% Find points at different constant steer angles
for delta_eq = linspace(-20,20,50)*pi/180
    
    % Oversteer equilibrum point
    eq_pt = fsolve(@(y) drift_eom(0, [vx_eq; y(1); y(2)], [delta_eq, y(3)], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f), [-.8, 2, 3], options);
    vy_eq = eq_pt(1);
    r_eq = eq_pt(2);
    throttle = eq_pt(3);
    
    beta = atan(vy_eq/vx_eq);
    subplot(2,2,1);
    hold on
    plot(delta_eq*180/pi, vy_eq, 'bo', "MarkerFaceColor",'b')
    subplot(2,2,2);
    hold on
    plot(delta_eq*180/pi, beta*180/pi, 'bo', "MarkerFaceColor",'b')
    subplot(2,2,3);
    hold on
    plot(delta_eq*180/pi, r_eq , 'bo', "MarkerFaceColor",'b')
    subplot(2,2,4);
    hold on
    plot(delta_eq*180/pi, throttle , 'bo', "MarkerFaceColor",'b')

    % Understeer equilibrum point
    eq_pt = fsolve(@(y) drift_eom(0, [vx_eq; y(1); y(2)], [delta_eq, y(3)], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f), [.8, -2, 3], options);
    vy_eq = eq_pt(1);
    r_eq = eq_pt(2);
    throttle = eq_pt(3);
    
    beta = atan(vy_eq/vx_eq);
    subplot(2,2,1);
    hold on
    plot(delta_eq*180/pi, vy_eq, 'yo', "MarkerFaceColor",'y')
    subplot(2,2,2);
    hold on
    plot(delta_eq*180/pi, beta*180/pi, 'yo', "MarkerFaceColor",'y')
    subplot(2,2,3);
    hold on
    plot(delta_eq*180/pi, r_eq , 'yo', "MarkerFaceColor",'y')
    subplot(2,2,4);
    hold on
    plot(delta_eq*180/pi, throttle , 'yo', "MarkerFaceColor",'y')


    % Normal cornering equilibrum point
    eq_pt = fsolve(@(y) drift_eom(0, [vx_eq; y(1); y(2)], [delta_eq, y(3)], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f), [0, 0, 0], options);
    vy_eq = eq_pt(1);
    r_eq = eq_pt(2);
    throttle = eq_pt(3);
    
    beta = atan(vy_eq/vx_eq);
    subplot(2,2,1);
    hold on
    plot(delta_eq*180/pi, vy_eq, 'co', "MarkerFaceColor",'c')
    subplot(2,2,2);
    hold on
    plot(delta_eq*180/pi, beta*180/pi, 'co', "MarkerFaceColor",'c')
    subplot(2,2,3);
    hold on
    plot(delta_eq*180/pi, r_eq , 'co', "MarkerFaceColor",'c')
    subplot(2,2,4);
    hold on
    plot(delta_eq*180/pi, throttle , 'co', "MarkerFaceColor",'c')

end

delta_eq = -15*pi/180;
eq_pt = fsolve(@(y) drift_eom(0, [vx_eq; y(1); y(2)], [delta_eq, y(3)], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f), [0, 0, 0], options);
vy_eq = eq_pt(1);
r_eq = eq_pt(2);
throttle = eq_pt(3);
beta = atan(vy_eq/vx_eq);
fprintf("delta = -15deg, normal cornering: vy=%.3f\t r=%.3f\t Fx=%.3f\t beta=%.3f\n", vy_eq, r_eq, throttle, beta);


delta_eq = -15*pi/180;
eq_pt = fsolve(@(y) drift_eom(0, [vx_eq; y(1); y(2)], [delta_eq, y(3)], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f), [.8, -2, 3], options);
vy_eq = eq_pt(1);
r_eq = eq_pt(2);
throttle = eq_pt(3);
beta = atan(vy_eq/vx_eq);
fprintf("delta = -15deg, understeer:       vy=%.3f\t r=%.3f\t Fx=%.3f\t beta=%.3f\n", vy_eq, r_eq, throttle, beta);


delta_eq = -15*pi/180;
eq_pt = fsolve(@(y) drift_eom(0, [vx_eq; y(1); y(2)], [delta_eq, y(3)], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f), [-.8, 2, 3], options);
vy_eq = eq_pt(1);
r_eq = eq_pt(2);
throttle = eq_pt(3);
beta = atan(vy_eq/vx_eq);

fprintf("delta = -15deg, oversteer:        vy=%.3f\t r=%.3f\t Fx=%.3f\t beta=%.3f\n", vy_eq, r_eq, throttle, beta);

subplot(2,2,1);
hold on
plot(delta_eq*180/pi, vy_eq, "pentagram", "MarkerFaceColor",'r')
subplot(2,2,2);
hold on
plot(delta_eq*180/pi, beta*180/pi, "pentagram", "MarkerFaceColor",'r')
subplot(2,2,3);
hold on
plot(delta_eq*180/pi, r_eq , "pentagram", "MarkerFaceColor",'r')
subplot(2,2,4);
hold on
plot(delta_eq*180/pi, throttle , "pentagram", "MarkerFaceColor",'r')



subplot(2,2,1);

ylabel('Vy')

subplot(2,2,2);
% ylim([-40,40])
ylabel('beta')

subplot(2,2,3);
% ylim([-2.5,2.5])
ylabel('r')

subplot(2,2,4);
% ylim([-2.5,2.5])
ylabel('Fx')