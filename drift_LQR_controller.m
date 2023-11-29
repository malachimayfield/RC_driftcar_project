function u = drift_LQR_controller(state)
% drift_LQR_controller TODO

%     vx = state(1);
%     vy = state(2);
%     r  = state(3);
    max_steering_angle = 30*pi/180;

    u_eq = [ -15*pi/180; 1.6071];

    state_eq = [1.5; -0.57; 1.96];

    K_LQR = [-0.9277,   -1.7791,    1.0831;    1.3237,   -0.5485,    0.0787];

    u = u_eq-K_LQR * (state-state_eq);
    
    u(1) = min(max(u(1),-max_steering_angle), max_steering_angle);
end

