function jointvelocitiesfiltered  = discretelowpass(jointvelocitiesbeforefilter,jointvelocitiesprevious,timestep)
    %qdot limits in rad/sec
    qdotlimits = (pi/180)*[110 110 128 128 204 184 184];
    maxaccel  = 200; % accel in rad/S^2
    %at firstchecking if the desired velocities are acceptable
    %for the given limits
    limitedvelocities = zeros(length(qdotlimits),1);
    for i = 1:length(qdotlimits)
        if abs(jointvelocitiesbeforefilter(i)) <= qdotlimits(i)
            limitedvelocities(i) = jointvelocitiesbeforefilter(i);
        else 
            limitedvelocities(i) = sign(jointvelocitiesbeforefilter(i))*qdotlimits(i);
        end
    end   
    %jointvelocitiesbeforefilter
  
    %for limited velocities now check the acceleration
    for i = 1:length(limitedvelocities)
        tempaccel = (limitedvelocities(i) - jointvelocitiesprevious(i))/timestep;
        if abs(tempaccel)<= maxaccel
            jointvelocitiesfiltered(i) = limitedvelocities(i);
           
        else 
            if tempaccel > 0
                jointvelocitiesfiltered(i) = jointvelocitiesprevious(i) + maxaccel*timestep;
            else 
                jointvelocitiesfiltered(i) = jointvelocitiesprevious(i) - maxaccel*timestep; 
            end
        end 
    end
      % jointvelocitiesfiltered
   
end
