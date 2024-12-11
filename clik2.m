function  velocity = clik2(currentpos,vendeffectornow,ballxyz,vballxyz,rotationwanted,stage,stepsinstage1,kappas)   
    zwanted = [0;0;-1];
    rotationnow = currentpos.R;
    zcurrent = rotationnow(:,3);
    %getting the unitary vector of rotation
    vectorofRotation = cross(zcurrent,zwanted);
    vectorofRotation = vectorofRotation/norm(vectorofRotation);
    thetainrad= -acos(dot(zcurrent,zwanted)); 
    %calculating the angle for the rotatioν
    %creating the rotation       
    %calculating the velocity and the position of ball in demanded when
    %they dont match
    %getting the p_cb in the correct orientation   
    errorTransformation = SE3.angvec(thetainrad,vectorofRotation);
    %
    unitqwanted = UnitQuaternion(rotationwanted);
    unitqnow  = UnitQuaternion(rotationnow);
    unitqerror = unitqnow*inv(unitqwanted);
    errorofRotation = unitqerror.v';         
    %finding the wanted velocity
    kp  = 8;
    currenttrans = currentpos.T;   
    if stage == 0
        errorofPosition = currenttrans(1:3,4) - ballxyz-  [0;0;0.4]; 
        v = vballxyz - kp*errorofPosition;
    else
        %starting going closer to the ball
        temptime = stepsinstage1*4*10^-3;
        %temptime is the time that going down in z
        pznow = dot(kappas,[1,temptime,temptime^2,temptime^3,temptime^4,temptime^5]);
        vznow = dot(kappas,[0,1,2*temptime,3*temptime^2,4*temptime^3,5*temptime^4]);
        errorofPosition = currenttrans(1:3,4) - ballxyz;
        errorofPosition(3) = currenttrans(3,4) - pznow;
        v = vballxyz +[0;0;vznow] -kp*errorofPosition;
    end
    ko = 2; 
    w = -ko*errorofRotation; 
    %vbnow = [vendeffectornow(1:3);zeros(3,1)];
    velocity = [v;w];
end