%%%% The Castaway Problem
clear all
clc
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID > -1)
    disp('Connected')
    %Handlers
       
        [returnCode,quadBase]=vrep.simxGetObjectHandle(clientID,'Quadricopter_base', vrep.simx_opmode_blocking );
    	[returnCode,quadBase_0]=vrep.simxGetObjectHandle(clientID,'Quadricopter_base#0', vrep.simx_opmode_blocking );  	
        [returnCode,quad]=vrep.simxGetObjectHandle(clientID,'Quadricopter', vrep.simx_opmode_blocking );
    	[returnCode,quadTarget]=vrep.simxGetObjectHandle(clientID,'Quadricopter_target', vrep.simx_opmode_blocking );
    	[returnCode,quadTarget_0]=vrep.simxGetObjectHandle(clientID,'Quadricopter_target#0', vrep.simx_opmode_blocking );   
        disp('Conected')
        pause(3)

     [returnCode,posQd]=vrep.simxGetObjectPosition(clientID,quadBase,-1,vrep.simx_opmode_blocking)
     [returnCode,posQd_0]=vrep.simxGetObjectPosition(clientID,quadBase_0,-1,vrep.simx_opmode_blocking)
     disp(posQd)
     [returnCode]=vrep.simxSetObjectPosition(clientID,quadTarget,-1,[0,10, 2],vrep.simx_opmode_blocking); 


     %%
dt=0.005;
Vs=4;
VLs=4; % radiands
Vc=1;
Sp=[posQd(1)/10;posQd(2)/10] ;
Cp=[posQd_0(1)/10;posQd_0(2)/10]; 
CDir=1;
while abs(CDir) > 0.01
        St=Sp/norm(Sp);
        CDir=(1-norm(Sp));
        Sp=(norm(Sp)+sign(CDir)*Vc*dt/2)*St;
        [returnCode]=vrep.simxSetObjectPosition(clientID,quadTarget,-1,[Sp(1)*10,Sp(2)*10, 2],vrep.simx_opmode_blocking); 
        CDir
        pause(0.1)
end
State=0;
normCast(5000)=0;
for i =1:5000
    Sang=atan2(Sp(2),Sp(1));
    Cang=atan2(Cp(2),Cp(1));

    Sr=Cang-Sang;
    if Sr>pi
        Sr=Sr-2*pi;
    end
    if Sr< -pi
        Sr=Sr+2*pi;
    end

    if abs(Sr/4) > (1-norm(Cp))
        State=3;
    else
        State=1;
    end

    if (State==1)      
        St=Cp/norm(Cp);
        Sp=[cos(Sang+sign(Sr)*Vs*dt);sin(Sang+sign(Sr)*Vs*dt)];
        CDir=(0.24-norm(Cp));
        Cp=(norm(Cp)+sign(CDir)*Vc*dt)*St;  
        if abs(CDir)<0.01
            State=2;
        end
    end
    
    if State==2
        St=Cp/norm(Cp);
        Sp=[cos(Sang+sign(Sr)*Vs*dt);sin(Sang+sign(Sr)*Vs*dt)];
        Cp=[cos(Cang+sign(Sr)*Vc*dt/norm(Cp));sin(Cang+sign(Sr)*Vc*dt/norm(Cp))]*norm(Cp);
    end
     
    if State==3
        St=Cp/norm(Cp);
        Sp=[cos(Sang+sign(Sr)*Vs*dt);sin(Sang+sign(Sr)*Vs*dt)];
        Cp=(norm(Cp)+Vc*dt)*St;
    end
    
    normCast(i)=norm(Cp);
    
    Ix=cos(0:0.01:2*pi);Iy=sin(0:0.01:2*pi);
    figure(1)
    plot(St(1),St(2),'pg',Cp(1),Cp(2),'ob',Sp(1),Sp(2),'r*',Ix,Iy,'g')
    axis([-1 1 -1 1])
    grid on

    [returnCode]=vrep.simxSetObjectPosition(clientID,quadTarget,-1,[Sp(1)*10,Sp(2)*10, 2],vrep.simx_opmode_blocking); 
    [returnCode]=vrep.simxSetObjectPosition(clientID,quadTarget_0,-1,[Cp(1)*10,Cp(2)*10, 3],vrep.simx_opmode_blocking); 

    pause(0.00001) 
end    
pause(5)    
    vrep.simxFinish(-1);
end