function result = checkiftrajectorypossible(timetomove,camerapos,vballxyz)
kappas = trajectoryplanning(camerapos(3),0.125,timetomove)
safedistance = 0.02;
% 0.2 is safe heigth 0.1 wall and 0.1 fingers
%maybe and some safety distance
%need to flip the kappas
kappas = flip(kappas);
%want to solve the equation pz = 0.2 find the time and then x and y 
%if x and y accepted then x and y accepted
kappas(6) = kappas(6) - 0.2;
allroots = roots(kappas);
% keep only the real roots
allroots = allroots(real(allroots)==allroots)
criticaltime = allroots(allroots<timetomove)
criticaltime = allroots(allroots>0)
criticaltime = real(criticaltime)
criticalcamerapos(1:2)  = camerapos(1:2) +vballxyz(1:2)*criticaltime;
criticalcamerapos(3) = 0.2;
result = isinbounds(criticalcamerapos,safedistance);
end