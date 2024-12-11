clear all
close all
clc
q0 = [-90 -50 0 60 180 80 20];
q0inRad = q0*pi/180;
qrdotmax = [110 110 128 128 204 184 184];
qrdotmaxinRad = qrdotmax*pi/180;
lwr = lwr_create();
% initial position of the  end
%test = trplot(eye(4),'frame','0','color',[1 0 0],'length',0.25);
% Draw {B},{M},{F} frame in their initial positions
%radius of ball
radius = 0.025;
%calculating ball position in {0} xyz 
%creating the timespan of simulation 
%timestep = 4 ms
timeStep = 4*10^-3;
times = 0:timeStep:15;
%the simulation loop
ball = Ball();
pcbposx = [] ;
pcbposy = []; 
p0posx = [];
p0posy = [];
p0posz = [];
jointvelocitiez = [];
ws = [];
wxs = [];
wys = [];
wzs =[];
wends = [];
errors = [];
thetas = [];
vcbsx = [];
vcbsy = [];
vcbsz = [];
vballs=[];
v0cs =[];
ranks = [];
jointaccelerationz =[];
jointpositionz =[];
unitquaternionz = [];
v_0c = zeros(3,1);
w_0c = zeros(3,1);
jointvelocitiesprevious = zeros(7,1);
initialpos = lwr.fkine(q0inRad); 
initialTransformation = initialpos.T;
initialpositionin0 = initialTransformation(1:3,4);
plotthewalls()
%anim = Animate("part1.mp4");
for i = 1:length(times)
    ball.simOneStep();
    currentpos = lwr.fkine(q0inRad); 
    currentposTransformation = currentpos.T;
    p_0c = currentposTransformation(1:3,4);
    Q_0c = UnitQuaternion(currentpos.R);
    Q_0c = [Q_0c.s Q_0c.v];
    unitquaternionz = [unitquaternionz;Q_0c];
    v0cs = [v0cs;norm(v_0c)];
    [p_cb,v_cb] = ball.getState(p_0c,Q_0c,v_0c,w_0c);
    vcbsx =[vcbsx;v_cb(1)]; 
    vcbsy =[vcbsy;v_cb(2)]; 
    vcbsz =[vcbsz;v_cb(3)]; 
    p0posx = [p0posx;p_0c(1)];
    p0posy = [p0posy;p_0c(2)];
    p0posz = [p0posz;p_0c(3)];
    pcbposx = [pcbposx;p_cb(1)];
    pcbposy = [pcbposy;p_cb(2)];
    jacobfromcamerato0 = tr2jac(inv(currentposTransformation));
    vballxyz = jacobfromcamerato0*[v_cb;zeros(3,1)];
    vballxyz = vballxyz(1:3);
    vballs = [vballs;norm(v_cb)];
    [vb,wref,error,theta,wend] = calculateAngularVelocity(currentpos,p_cb,v_cb,w_0c,initialpositionin0);    
    thetas =[thetas;theta];
    ballxyz = currentposTransformation*[p_cb ; 1];
    ballxyz = ballxyz(1:3);
    jacobian = lwr.jacobe(q0inRad);
    pseudoinv = pinv(jacobian,10^-20);
    jointvelocitiesbeforefilter = pseudoinv*vb;
    jointvelocitiesfiltered = discretelowpass(jointvelocitiesbeforefilter,jointvelocitiesprevious,timeStep);
    %jointvelocitiesfiltered = jointvelocitiesbeforefilter';
    jointaccelerations = (jointvelocitiesfiltered-jointvelocitiesprevious)/timeStep;
    jointaccelerationz = [jointaccelerationz;jointaccelerations];
    jointvelocitiez = [jointvelocitiez;jointvelocitiesfiltered];
    %jacobian*jointvelocitiesfiltered'
    wxs = [wxs;wref(1)];
    wys = [wys;wref(2)];
    wzs = [wzs;wref(3)]; 
    errors =[errors,norm(error)];
    %velocity of end effector    
    q0inRad = q0inRad + timeStep*jointvelocitiesfiltered;    
    jointpositionz =[jointpositionz;q0inRad];
    jointvelocitiesprevious = jointvelocitiesfiltered;
    jacobian0 = lwr.jacob0(q0inRad);
    vendeffector = jacobian0*jointvelocitiesfiltered';
    v_0c= vendeffector(1:3);
    w_0c = vendeffector(4:6);
        %change the mod argument to change the ratio of updated
        if mod(i,50) == 0
       % calculating ball position in {0} xyz   
        lwr.plot(q0inRad,'workspace',[-1 1 -1.5 0.5 0 1],'floorlevel',-0.025,'jaxes')
        hold on;
        ballnow =  plot_sphere(ballxyz,radius);   
        %anim.add();
        pause(10^-10);
        delete(ballnow);
        end
end
%anim.close();
%%%%%%%%%%%%%%% 
%change the  value of this to 0 if you donta wanna see the plots
showplots =1;
%%%%%%%%%%%%%%%%%%%%%%
if showplots
    jointaccelerationz = jointaccelerationz(1:length(times),:);
    jointvelocitiez = jointvelocitiez(1:length(times),:);
    figure
    plot(times,pcbposx)
    title("Pcb x axis in camera")
    ylabel("Position in m")
    xlabel("Time in seconds")
    figure
    plot(times,pcbposy)
    title("Pcb y axis in camera")
    ylabel("Position in m")
    xlabel("Time in seconds")
    figure
    plot(times,vcbsx(:))
    title("Vcb x axis")
    ylabel("Velocity in m/s")
    xlabel("Time in seconds")
    figure
    plot(times,vcbsy(:))
    title("Vcb y axis")
    ylabel("Velocity in m/s")
    xlabel("Time in seconds")
    figure
    plot(times,vcbsz(:))
    title("Vcb z axis")
    ylabel("Velocity in m/s")
    xlabel("Time in seconds")
    figure
    plot(times,wxs(:))
    title("Angular velocity of end effector x axis")
    ylabel("Angular velocity in r/s")
    xlabel("Time in seconds")
    figure
    plot(times,wys(:))
    title("Angular velocity of end effector y axis")
    ylabel("Angular velocity in r/s")
    xlabel("Time in seconds")
    figure
    plot(times,wzs(:))
    title("Angular velocity of end effector z axis")
    ylabel("Angular velocity in r/s")
    xlabel("Time in seconds")
    figure
    plot(times,errors(:))
    title("Norm of error of orientation")
    ylabel("Norm of vector")
    xlabel("Time in seconds")
    figure
    plot(times,thetas(:))
    title("Theta of error ")
    ylabel("Angle in rad")
    xlabel("Time in seconds")
    figure
    plot(times,p0posx(:))
    title("Position of end effector x axis {0}")
    ylabel("Position in m")
    xlabel("Time in seconds")
    figure
    plot(times,p0posy(:))
    title("Position of end effector y axis {0}")
    ylabel("Position in m")
    xlabel("Time in seconds")
    figure
    plot(times,p0posz(:))
    title("Position of end effector z axis {0}")
    ylabel("Position in m")
    xlabel("Time in seconds")
    figure
    for i=1:7
        plot(times,jointvelocitiez(:,i));
        hold on
    end
    legend('Vel 1','Vel 2','Vel 3','Vel 4','Vel 5','Vel 6','Vel 7');
    title("Joint velocities")
    xlabel("Time in seconds")
    ylabel("Angular velocities in rad/s")
    figure
    for i=1:7
        plot(times,jointaccelerationz(:,i));
        hold on
    end
    legend('accel 1','accel 2','accel 3','accel 4','accel 5','accel 6','accel 7');
    title("Joint accelerations")
    ylabel("Acceleration in rad/s^2")
    xlabel("Time in seconds")
    figure
    for i=1:7
        plot(times,jointpositionz(:,i));
        hold on
    end
    legend('Pos 1','Pos 2','Pos 3','Pos 4','Pos 5','Pos 6','Pos 7');
    title("Position of joints")
    ylabel("Angle in rad/s")
    xlabel("Time in seconds")
    figure
    plot(times,unitquaternionz(:,1));  
    title("ηe of quaternion")
    xlabel("Time in seconds")
    figure
    plot(times,unitquaternionz(:,2)); 
    title(" 1st component if εe of quaternion")
    xlabel("Time in seconds")
    figure
    plot(times,unitquaternionz(:,3)); 
    title(" 2nd component if εe of quaternion")
    xlabel("Time in seconds")
    figure
    plot(times,unitquaternionz(:,4)); 
    title(" 3rd component if εe of quaternion")
    xlabel("Time in seconds")
end

