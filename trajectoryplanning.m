function k = trajectoryplanning(q0,qf,tf)
 %returns the constants for zeros initial and finishing velocity
 % and zero acceleretation 
 %initial time is considered 0
    k(1) = q0;
    k(2) = 0;
    k(3) = 0;
    k(4) = 10*(qf-q0)/tf^3;
    k(5) = -15*(qf-q0)/tf^4;
    k(6) = 6*(qf-q0)/tf^5;
end