function result = isinbounds(p,safedistance)
    xmin = -0.25;
    xmax =0.25;
    ymin = -0.75;
    ymax = -0.25;
   if(p(1)>xmin+safedistance &&p(1) < xmax-safedistance&&...
   p(2)>ymin+safedistance&& p(2) < ymax-safedistance)
    result = 1;
   else
       result = 0;
   end

end