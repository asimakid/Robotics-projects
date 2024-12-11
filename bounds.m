function [maxxr,maxyr,minxr,minyr] = bounds(maxx,maxy,minx,miny,ballxyz)
    if(ballxyz(1)> maxx)
       maxxr = ballxyz(1);
    else
        maxxr = maxx;
    end
     if(ballxyz(2)> maxy)
       maxyr = ballxyz(2);
    else
        maxyr = maxy;
     end
     if(ballxyz(1)< minx)
       minxr = ballxyz(1);
    else
        minxr = minx;
    end
     if(ballxyz(2)< miny)
       minyr = ballxyz(2);
    else
        minyr = miny;
    end    
end