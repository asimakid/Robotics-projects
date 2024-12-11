function [angularVelocity,wref,error,thetainrad,wend] = calculateAngularVelocity(currentpos,p_cb,v_cb,w_0c,initialpositionin0)   
    %the unitary vector here is expressed in the frame of camera
    %expressing the  p_cb in {0} frame
    unitary_p_cb0 = p_cb/norm(p_cb);
    %the vector is vertical to both unitary_p_cb and z of camera frame 
    zC = [0;0;1];
    %expressing it int the {0} frame
    %zC0 = currentpos.R*zC;
    zC0 = zC;
    %vector expressed in terms of  C frame     
    %getting the unitary vector of rotation
    vectorofRotation = cross(zC0,unitary_p_cb0);
    vectorofRotation = vectorofRotation/norm(vectorofRotation);
    thetainrad= -acos(dot(zC0,unitary_p_cb0)); 
    %calculating the angle for the rotatioν
    %creating the rotation       
    %calculating the velocity and the position of ball in demanded when
    %they dont match
    %getting the p_cb in the correct orientation   
    errorTransformation = SE3.angvec(thetainrad,vectorofRotation);   
    jacobianfromcameratodemanded = tr2jac(errorTransformation.T);
    jacobianfromdemandedtocamera = tr2jac(inv(errorTransformation.T));  
    p_cbdemanded = (errorTransformation.T)*[p_cb;1];
    p_cbdemanded = p_cbdemanded(1:3); 
    % w till now in camera 
    jacobian0tocamera = tr2jac(currentpos.T);
    wtillnowincamera = jacobian0tocamera(4:6,4:6)*w_0c;
    wtillnowindemanded = jacobianfromcameratodemanded(4:6,4:6)*wtillnowincamera;
    vfromrotationalindemanded = cross(wtillnowindemanded,p_cbdemanded);
    v_cbdemanded = jacobianfromcameratodemanded(1:3,1:3)*(v_cb) + vfromrotationalindemanded;   
    p_cbdemandedskew = [0 -p_cbdemanded(3) p_cbdemanded(2);...
    p_cbdemanded(3) 0 -p_cbdemanded(1);...
    -p_cbdemanded(2) p_cbdemanded(1) 0];
    pseudoinvskewdemanded = pinv(p_cbdemandedskew);
    windemanded = - pseudoinvskewdemanded * v_cbdemanded;
    wincamera = jacobianfromdemandedtocamera(4:6,4:6)*windemanded;
    wref = wincamera;
   % normofwref = norm(v_cbdemanded)/norm(p_cbdemanded) ;
    %wref = normofwref*vectorofRotation;    
    wref = wref;
    wend = norm(cross(wref,p_cbdemanded))- norm(v_cbdemanded);
    %wref =  jacobianfromdemandedtocamera*[zeros(3,1);wrefindemanded];
    %wref = wref(4:6);
    %wref = wrefindemanded;
    % this is the wd
    %error rotation
    error = thetainrad*vectorofRotation;
    %definition of kp 
    ko =5;
    w = wref -ko*error ;    
    % keeping the initial position control 
    initialpositionincamera = inv(currentpos.T)*[initialpositionin0;1];
    initialpostionincamera = initialpositionincamera(1:3);
    kp = 1;
    errorincamera = -initialpostionincamera;
    v  = -kp*errorincamera;
    %w = zeros(3,1);
    %angular velocity expressed in frame of the camera
    %angularVelocityfromError = tr2delta(cameraTransformation,demandedpos);
    angularVelocity =  [v;w] ;
end