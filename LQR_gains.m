

% Parameters of RC car in paper, probably a good place to start
m = 1.86;       % Mass [kg]
Lr = .100;     % Distance from CM to rear axle [m]
Lf = .160;     % Distance from CM to front axle [m]
Iz = 0.03;      % Yaw moment of inertia [kg/m^2]
Cr = 127.77;    % Rear tire cornering stiffness
Cf = 47.86;     % Front tire cornering stiffness
mu_r = 0.33;    % Rear tire coefficient of friction
mu_f = 0.35;    % Front tire coefficient of friction
g = 9.81;       % Acceleration due to gravity [m/s^2]

% Search for points with a constant Vx
vx_eq = 1.5;

options = optimset('Display','off');

delta_eq = -10*pi/180;
eq_pt = fsolve(@(y) drift_eom(0, [vx_eq; y(1); y(2)], [delta_eq, y(3)], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f), [-.8, 2, 3], options);
vy_eq = eq_pt(1);
r_eq = eq_pt(2);
throttle = eq_pt(3);
beta = atan(vy_eq/vx_eq);

fprintf("delta = -15deg, oversteer:        vy=%.3f\t r=%.3f\t Fx=%.3f\t beta=%.3f\n", vy_eq, r_eq, throttle, beta);

A=zeros(3,3);
B=zeros(3,2);
h=0.0000001;
A(:,1) = (drift_eom(0,[vx_eq+h; vy_eq; r_eq], [delta_eq; throttle], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f)-drift_eom(0,[vx_eq-h; vy_eq; r_eq], [delta_eq; throttle], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f))/(2*h);

A(:,2) = (drift_eom(0,[vx_eq; vy_eq+h; r_eq], [delta_eq; throttle], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f)-drift_eom(0,[vx_eq; vy_eq-h; r_eq], [delta_eq; throttle], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f))/(2*h);

A(:,3) = (drift_eom(0,[vx_eq; vy_eq; r_eq+h], [delta_eq; throttle], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f)-drift_eom(0,[vx_eq; vy_eq; r_eq-h], [delta_eq; throttle], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f))/(2*h);

B(:,1) = (drift_eom(0,[vx_eq; vy_eq; r_eq], [delta_eq+h; throttle], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f)-drift_eom(0,[vx_eq; vy_eq; r_eq], [delta_eq-h; throttle], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f))/(2*h);

B(:,2) = (drift_eom(0,[vx_eq; vy_eq; r_eq], [delta_eq; throttle+h], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f)-drift_eom(0,[vx_eq; vy_eq; r_eq], [delta_eq; throttle-h], g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f))/(2*h);

Q = diag([.1,.1,1]);
R=diag([1, 0.5]);
lqr(A,B,Q,R,0)