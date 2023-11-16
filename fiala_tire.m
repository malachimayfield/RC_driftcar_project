function Fy = fiala_tire(Fx, Fz, alpha, C, mu)
    % fiala_tire calculates the lateral force on a tire given the normal
    % force, longitudinal force, and tire/surface parameters

    % If the tire isn't saturated use a factor defined by friction circle
    if (mu*Fz)^2-Fx^2 > 0
        f = sqrt( ((mu*Fz)^2-Fx^2) / ((mu*Fz)^2)) ;
    else
        f=0;
    end
    
    % Calculate slip angle
    alpha_slip = atan(3*mu*Fz*f/C);
       
    % Differnt lateral loads whether the tire is slipping or not
    if abs(alpha) < alpha_slip
        Fy = -C*tan(alpha)+(C^2 / (3*mu*Fz*f))* abs(tan(alpha))*tan(alpha) - (C^3 / (27*mu^2*f^2*Fz^2))*tan(alpha)^3;
    else
        Fy = -mu*Fz*sign(alpha)*f;
    end
    
    
end

