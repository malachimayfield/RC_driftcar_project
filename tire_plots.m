% Generates tire force vs tire sideslip plots at differnt throttle levels

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
        
% Use rear tire parameters
Fz = Lr*m*g/(Lf+Lr);
alpha = deg2rad(linspace(-30,30,1000));
C = Cr;
mu = mu_r;

for Fx = linspace(0 , mu*Fz,10)
    Fy = arrayfun(@(a) fiala_tire(Fx, Fz, a, C, mu), alpha);
    plot(alpha*180/pi, Fy, "DisplayName","Fx = "+string(round(Fx,2)))
    hold on
end

xlabel("Tire Sideslip (deg)")
ylabel("Tire Force (N)")
legend()