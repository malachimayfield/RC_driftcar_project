function state_dot = drift_eom(t,state,u,g, Lf, Lr, m, Iz, Cr, Cf, mu_r, mu_f)
    % drift_eom calculates the longitudinal acceleration, lateral
    % acceleration, and the yaw angular acceleration
    

    % Longitudinal, lateral, and yaw velocity
    vx = state(1);
    vy = state(2);
    r  = state(3);
 
    v = sqrt(vx^2 +vy^2);

    % Steer angle and throttle
    delta = u(1);
    throttle = u(2);
    
    % Slip angle of front and rear
    if abs(v)>0
        alpha_r = atan(vy-r*Lf/ vx);
        alpha_f = atan(vy+r*Lf/ vx) - delta;
    else
        alpha_r = 0;
        alpha_f = 0;
    end

    % Need to figure out throttle to rear wheel force map
    Fxr = throttle;

    % Normal force of front and rear tires
    Fzr = Lf*m*g/(Lf+Lr);
    Fzf = Lr*m*g/(Lf+Lr);

    % Lateral force on the tires
    Fyr = fiala_tire(Fxr, Fzr, alpha_r, Cr, mu_r);
    Fyf = fiala_tire(0, Fzf, alpha_f, Cf, mu_f);

    % Total force and moment
    Fx = Fxr-Fyf*sin(delta);
    Fy = Fyr+Fyf*cos(delta);
    Mz = Lf*Fyf*cos(delta) - Lr*Fyr;

    vx_dot = r*vy + Fx/m;
    vy_dot = -r*vx + Fy/m;
    r_dot  = Mz / Iz;

    state_dot = [vx_dot; vy_dot; r_dot];
end

