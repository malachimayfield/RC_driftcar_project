function u = drift_LQR_controller(state)
% drift_LQR_controller TODO

%     vx = state(1);
%     vy = state(2);
%     r  = state(3);
    max_steering_angle = 20*pi/180;

    u_eq = [-15*pi/180; 1.51];

    state_eq = [1.5; -0.566; 1.971];

    K_LQR = [-0.3046   -1.4142    1.0229;
    1.2731   -0.6590    0.1152];

    u = u_eq-K_LQR * (state-state_eq);
    
    u(1) = min(max(u(1),-max_steering_angle), max_steering_angle);
end

