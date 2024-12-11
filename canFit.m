function result = canFit(currentTrans,camerapos,ballpos,vcamera,vball,wantedOrientation,timetomove)
    % a safety diastance from the wall of the ball
    safedistance = 0.08; %2 cm safety diastance from the wall
    %given from the dimension and the position of the
    %box expressed in 0 frame
    xmin = -0.25;
    xmax =0.25;
    ymin = -0.75;
    ymax = -0.25;
    % all position and veloctities are expected in {0}
    limitforposerror = 0.01;
    limitforvelerror = 0.01;
    errorofRotation = currentTrans(1:3,1:3)-wantedOrientation;
    errorofRotation = errorofRotation > 0.01;
    if sum(any(errorofRotation),'all')
       result = 0;          
       return       
    end
    %at first check the difference between camera and ball pso
    if (norm(camerapos(1:2)-ballpos(1:2)) > limitforposerror) 
        result = 0;
        norm(camerapos(1:2)-ballpos(1:2))
        return       
    end
    if (norm(vcamera(1:2)-vball(1:2)) > limitforvelerror)
        result = 0;
        norm(vcamera(1:2)-vball(1:2))
        return       
    end
    % if passed those checks means the manipulator has centered
    % good enough for the ball
    %the position at the end of the motion must be
    finishballpos = ballpos + vball*timetomove;
    if (~(finishballpos(1)> xmin &&finishballpos(1) < xmax &&...
       finishballpos(2)> ymin &&finishballpos(2) < ymax))
        result = 0;
        return
    end
    %if this check is passed means the ball will not have 
    %hit any wall during the time expected
    %now the four edges of the grab need to be checked
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %describing the position of the four points in camera frame
    %the last  1 to apply geometric transform
    %{
  
    currentTrans(3,4) = 0.125;
    pas  = currentTrans*pa;
    pbs = currentTrans*pb;
    pcs = currentTrans*pc;
    pds = currentTrans*pd;
    pas  = pas(1:3);
    pbs  = pbs(1:3);
    pcs  = pcs(1:3);
    pds  = pds(1:3);
    
    %}
     pa = [0.045;0;0;1];
    pb = [-0.045;0;0;1];
    pc = [0.035;0;0.1;1];
    pd = [-0.035;0;0.1;1];
    finishpos = finishballpos  + [0;0;0.1];
    transofrmationwanted = [wantedOrientation,finishpos;0 0 0 1];
    paf  = transofrmationwanted*pa;
    pbf = transofrmationwanted*pb;
    pcf = transofrmationwanted*pc;
    pdf = transofrmationwanted*pd;
    paf  = paf(1:3);
    pbf  = pbf(1:3);
    pcf  = pcf(1:3);
    pdf  = pdf(1:3);
    %% first check if at the end can fit
    %% if can fit then check the whole trajectory
    %% done this way in order not to spend time checking the whole trajectory
    %% if at the end cant fit
    if (~isinbounds(paf,safedistance))
        result = 0;
        return
     elseif (~isinbounds(pbf,safedistance))
        result = 0;
        return
     elseif ( ~isinbounds(pcf,safedistance))
        result = 0;
        return
    elseif( ~isinbounds(pdf,safedistance))
        result = 0;
        return 
    end
     if checkiftrajectorypossible(timetomove,camerapos,vball)
         result = 1;
         return
     else
         result =0;
         return
     end
    

end