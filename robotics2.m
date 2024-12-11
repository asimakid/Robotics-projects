clear all
close all
clc
pcbposx = [];
pcbposy = []; 
p0posx = [];
p0posy = [];
p0posz = [];
jointvelocitiez = [];
ws = [];
wxs = [];
wys = [];
wzs =[];
vcbsx = [];
vcbsy = [];
vcbsz = [];
vballs=[];
v0cs =[];
pcbposz =[];
ballsxy = [];
jointaccelerationz =[];
jointpositionz =[];
unitquaternionz = [];
q0 = [-90 -50 0 60 180 80 20];
q0inRad = q0*pi/180;
qrdotmax = [110 110 128 128 204 184 184];
qrdotmaxinRad = qrdotmax*pi/180;
lwr = lwr_create('tool');
% initial position of the  end
%test = trplot(eye(4),'frame','0','color',[1 0 0],'length',0.25);
% Draw {B},{M},{F} frame in their initial positions
%radius of ball
radius = 0.025;
%calculating ball position in {0} xyz 
%creating the timespan of simulation 
%timestep = 4 ms
timeStep = 4*10^-3;
times = 0:timeStep:50;
%the simulation loop
ball = Ball();
v_0c = zeros(3,1);
w_0c = zeros(3,1);
jointvelocitiesprevious = zeros(7,1);
initialpos = lwr.fkine(q0inRad); 
initialTransformation = initialpos.T;
initialpositionin0 = initialTransformation(1:3,4);
stage = 0;
vendeffector = zeros(6,1);
%wanted orientation for attenting the ball
wantedOrientation =[0 -1 0;-1 0 0;0 0 -1];
%time for the trajectory to be planned if can fit
timetomove = 2;
kappas = 0;
stepsinstage1 = -1;
plotthewalls()
anim = Animate("test.mp4");
for i = 1:length(times)
    ball.simOneStep();
    currentpos = lwr.fkine(q0inRad); 
    currentposTransformation = currentpos.T;
    p_0c = currentposTransformation(1:3,4);
    if (stepsinstage1 == timetomove*250)
        break
    end
    Q_0c = UnitQuaternion(currentpos.R);
    Q_0c = [Q_0c.s Q_0c.v];
    [p_cb,v_cb] = ball.getState(p_0c,Q_0c,v_0c,w_0c);    
    %ball position in {0}frame 
    ballxyz = currentposTransformation*[p_cb ; 1];
    ballxyz = ballxyz(1:3);
    %ball velocity in {0} frame
    % that the sum of the v_0c + v_cb expressed in {0}
    jacobfromcamerato0 = tr2jac(inv(currentposTransformation));
    jacobfrom0tocamera = tr2jac((currentposTransformation));
    %since the v_cb is the related velocity
    %% w in camera
    winc = jacobfrom0tocamera(4:6,4:6)*w_0c;
    vfromrotationalincamera = cross(winc,p_cb);
    vfromrotationalin0 = jacobfromcamerato0(1:3,1:3)*vfromrotationalincamera;
    vballxyz = jacobfromcamerato0*[v_cb;zeros(3,1)] ;
    vballxyz = vballxyz(1:3)+ v_0c(1:3) +vfromrotationalin0;
    %%%%%%%
    %check if can fit
    if stage == 0 
        if(canFit(currentposTransformation,p_0c,ballxyz,v_0c,vballxyz,wantedOrientation,timetomove))
            kappas = trajectoryplanning(p_0c(3),0.125,timetomove);
            stage=1;
            stepsinstage1 = 0;
        end
    end
    %vb is expressed in 0 frame
    vb = clik2(currentpos,vendeffector,ballxyz,vballxyz,wantedOrientation,stage,stepsinstage1,kappas);
    jacobian = lwr.jacob0(q0inRad);
    pseudoinv = pinv(jacobian,10^-20);
    jointvelocitiesbeforefilter = pseudoinv*vb;
    jointvelocitiesfiltered = discretelowpass(jointvelocitiesbeforefilter,jointvelocitiesprevious,timeStep);
    q0inRad = q0inRad + timeStep*jointvelocitiesfiltered;
    jacobian0 = lwr.jacob0(q0inRad);
    vendeffector = jacobian0*jointvelocitiesfiltered';
    v_0c= vendeffector(1:3);
    w_0c = vendeffector(4:6); 
    if stage ==1 
        stepsinstage1 = stepsinstage1 + 1;
    end  
    jointaccelerations = (jointvelocitiesfiltered-jointvelocitiesprevious)/timeStep;
    jointaccelerationz = [jointaccelerationz;jointaccelerations];
    jointvelocitiez = [jointvelocitiez;jointvelocitiesfiltered];
    jointvelocitiesprevious = jointvelocitiesfiltered;
    wxs = [wxs;w_0c(1)];
    wys = [wys;w_0c(2)];
    wzs = [wzs;w_0c(3)];  
    jointpositionz =[jointpositionz;q0inRad];
    unitquaternionz = [unitquaternionz;Q_0c];
    vcbsx =[vcbsx;v_cb(1)]; 
    vcbsy =[vcbsy;v_cb(2)]; 
    vcbsz =[vcbsz;v_cb(3)]; 
    p0posx = [p0posx;p_0c(1)];
    p0posy = [p0posy;p_0c(2)];
    p0posz = [p0posz;p_0c(3)];
    pcbposx = [pcbposx;p_cb(1)];
    pcbposy = [pcbposy;p_cb(2)];
    pcbposz = [pcbposz;p_cb(3)];
    ballsxy = [ballsxy;ballxyz(1:2)'];
    %%{
    if mod(i,10) == 0
       % calculating ball position in {0} xyz   
        %box.plot
        
        lwr.plot(q0inRad,'workspace',[-1 1 -1.5 0.5 0 1],'floorlevel',-0.025,'jaxes')
        hold on;
        ballnow =  plot_sphere(ballxyz,radius);   
        anim.add();
        pause(10^-10);     
        delete(ballnow)          
        end
    %%}
end
anim.close();
showplots = 1;
if showplots
     jointaccelerationz = jointaccelerationz(1:i-1,:);
    times = times(1:i-1);
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
    title("Angular velocity of end effector x axis in 0")
    ylabel("Angular velocity in r/s")
    xlabel("Time in seconds")
    figure
    plot(times,wys(:))
    title("Angular velocity of end effector y axis in 0")
    ylabel("Angular velocity in r/s")
    xlabel("Time in seconds")
    figure
    plot(times,wzs(:))
    title("Angular velocity of end effector z axis in 0")
    ylabel("Angular velocity in r/s")
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
    figure
    plot(times,ballsxy(:,1))
    title("Position of ball x axis {0}")
    ylabel("Position in m")
    xlabel("Time in seconds")
    figure
    plot(times,ballsxy(:,2))
    title("Position of ball y axis {0}")
    ylabel("Position in m")
    xlabel("Time in seconds")
end