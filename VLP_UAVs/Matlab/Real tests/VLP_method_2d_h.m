%To do:
%Compare combinations of paremeters for tests
%Add titlting condition to remove VLP calculations with a lot of
%inclination
%Add CF for ax and ay
%Adjust start time during lift off where the barometric pressure drops

%Parameters to play around with
%<m,k> = <6,1> or <12,0.45>
%Kc= [1.4,1] , f=1,2,3,...
%Start time, end time (to discard pressure drop during lift off)
%Filter values
%Try 3RSS vs 4RSS calculation

%Reset workspace
clc;
clear all;
close all;
rng('default');

%Fixed coordinates of the Tx stations [x,y,z]
Tx1=[0.25, 1, 0];
Tx2=[1, 1.75, 0];
Tx3=[1.75, 1, 0];
Tx4=[1, 0.25, 0];

%Construct vectors for x,y,z coordinates
X=[Tx1(1) Tx2(1) Tx3(1) Tx4(1)];
Y=[Tx1(2) Tx2(2) Tx3(2) Tx4(2)];
Z=[Tx1(3) Tx2(3) Tx3(3) Tx4(3)];

%Starting position of the drone
rx_center=[1,1,0.2];
Rx = rx_center;

%Light channel parameters
m=6; %Lambertian order
k=1; %Calibration factor to account for lateral error deviation
Aeff=5.2; %[mm^2]
Pt=2.1; %[Watts]
%A=[0.86; 1.06; 1.06; 1.29]; %Calibration of individual LEDs intensity
A=[1; 1; 1; 1]; %Calibration of individual LEDs intensity
W = [0.8904, 0.1723, 0.2330, 0.2253, 0.3489, 0.4214]; %Values for simple NN height estimation (not used)
    %h=W(1)*(W(2)*Pr1 + W(3)*Pr2 + W(4)*Pr3 + W(5)*Pr4))+W(6)

%Load data logged from crazyflie tests        
    %~Errors with A=[1;1;1;1],kc=[1.4/f,1/f], %f=1,and 4RSS
                      %<m,k> = <6,1,> | <12,0.45>  
%load("data/logcurve_t1.mat")  % 20cm | 23 cm
%load("data/logcurve_t2.mat")  % 34cm | 28cm
%load("data/logcurve_t3.mat")  % 22cm | 22cm
%load("data/logcurve_t4.mat")  % 26cm | 25cm
load("data/logcurve_t5.mat")  % 22cm | 22cm
%load("data/logcurve_t6.mat")  % 23cm | 23cm
%load("data/logcurve_t7.mat")  % 25cm | 29cm
%load("data/logcurve_t8.mat")  % 25cm | 21cm

%Save logged data to local variables
    %x,y,z,roll,pitch,yaw
Rx_time_state = xyzrpy.time;
Rx_x = xyzrpy.x;
Rx_y = xyzrpy.y;
Rx_z = xyzrpy.z;
Rx_roll = xyzrpy.roll;
Rx_pitch = xyzrpy.pitch;
Rx_yaw = xyzrpy.yaw;

    %Power received
Rx_time_pr = pr1234.time;
Rx_Pr1 = pr1234.Pr1;
Rx_Pr2 = pr1234.Pr2;
Rx_Pr3 = pr1234.Pr3;
Rx_Pr4 = pr1234.Pr4;

    %Accelerometer
Rx_time_acc = axayaz.time;
Rx_ax = axayaz.ax;
Rx_ay = axayaz.ay;
Rx_az = axayaz.az;

    %Barometer
Rx_time_bar = asl.time;
Rx_bar = asl.asl - asl.asl(1); %Set starting position at 0m

%Apply filtering to raw signals
    %LP filter of accelerometer
Rx_ax = lowpass(Rx_ax, 0.01, 50);
Rx_ay = lowpass(Rx_ay, 0.01, 50);
Rx_az = lowpass(Rx_az, 0.01, 50);

    %Moving average filter of barometer
B=1/5*ones(5,1); %Window length = 5 samples
Rx_bar = filter(B,1,Rx_bar);

    %LP ffilter of power received
Rx_Pr1=lowpass(Rx_Pr1, 0.1, 10); 
Rx_Pr2=lowpass(Rx_Pr2, 0.1, 10); 
Rx_Pr3=lowpass(Rx_Pr3, 0.1, 10); 
Rx_Pr4=lowpass(Rx_Pr4, 0.1, 10); 

%Determine acceleration bias based on first samples when the rotors are off
acc_bias=[mean(Rx_ax(1:40)); mean(Rx_ay(1:40)); mean(Rx_az(1:40))];

%Time parameters of the simulation 
start_time = round(Rx_time_state(1)/10)*10;
end_time = round(Rx_time_bar(end)/10)*10;
t_step = 10; %[ms]

%Initialize local variables and structures to perform and evaluate
%simulation

    %Maximum Likelihood Estimation (MLE)variables
MLE_est_2D = rx_center(1:2);
MLE_est = rx_center;
MLE_err = 0;
MLE_all_err = [];

    %Counters for the position of the logged data 
state_ctr = 2;
pr_ctr = 2;
acc_ctr = 2;
bar_ctr = 2;

    %Containers to store ground truth and estimation points to compute
    %statistics at the end
Rx_all = [];
est_all = [];

    %Containers to store angles at the time of VLP method computation
all_roll = [];
all_pitch = [];

    %Containers to store all individual h estimates to plot at the end
all_h = []; %Real height
all_h_pr = []; %Estimate from simple NN approach, h=k*(a1*Pr1 + a2*Pr2 + a3*Pr3 + a4*Pr4))+o, not used
all_h_bar = []; %Barometer estimate
all_h_acc = []; %Accelerometer estimate
all_h_fus = []; %Fused estimate with complementary filter

    %Complementary filter variables
az=0;
vz=0;
pz=0.2;
h_pr=0.2;
h_bar=0.2;
h_fus = 0.2;
f=1; %Larger f means I trust the accelerometer more than the barometer
kc=[1.4/f,1/f];
%sigma_w = std(Rx_az(1:40));
%sigma_v = std(Rx_bar(1:40));
%kc = (1/f)*[sqrt((2*sigma_w)/sigma_v); sigma_w/sigma_v];
u=0; %Semaphore used to enable complementary filter calculation
v=0; %Semaphore used to enable VLP calculation

%Perform simulation
for t=start_time:t_step:end_time %Time in seconds
    t %Display time
    
    %Advance counters until start time is reached
    if (Rx_time_state(state_ctr) <= round(Rx_time_state(1)/10)*10)
        state_ctr = state_ctr+1;
        pr_ctr = pr_ctr+1;
        acc_ctr = acc_ctr+1;
        bar_ctr = bar_ctr+1;
    end
    
    %Stop simulation if reached the end of any logged measurement
    if state_ctr >= length(Rx_time_state) || pr_ctr >= length(Rx_time_pr) || acc_ctr >= length(Rx_time_acc) || bar_ctr >= length(Rx_time_bar) %|| t> 70000
        break
    end
    
    %Update ground truth coordinates of the receiver
    if (t == round(Rx_time_state(state_ctr)/10)*10)
        Rx=[rx_center(1)+Rx_x(state_ctr),rx_center(2)+Rx_y(state_ctr),rx_center(3)+Rx_z(state_ctr)];
        if abs(Rx_roll(state_ctr)) < 1 || abs(Rx_pitch(state_ctr)) < 1
        %if (abs(Rx_roll(state_ctr)) + abs(Rx_pitch(state_ctr))) < 2
            v=1;
        else
            v=0;
        end
        state_ctr = state_ctr + 1;      
    end
    
    %Update accelerometer measurement
    if (t == round(Rx_time_acc(acc_ctr)/10)*10)
        dt=(Rx_time_acc(acc_ctr)-Rx_time_acc(acc_ctr-1))/1000;
        az = (Rx_az(acc_ctr)-acc_bias(3))*9.81;
        acc_ctr = acc_ctr + 1;
        u=1; %Enable semaphore
    end
    
    %Update barometer measurement
    if (t == round(Rx_time_bar(bar_ctr)/10)*10)
        dt=(Rx_time_bar(bar_ctr)-Rx_time_bar(bar_ctr-1))/1000;
        h_bar = Rx_bar(bar_ctr)- Rx_bar(1);        
        bar_ctr = bar_ctr + 1;
        u=1; %Enable semaphore
    end
    
    %Update complementary filter
    if u==1
        %Difference equations from https://ieeexplore-ieee-org.ezproxy2.utwente.nl/stamp/stamp.jsp?tp=&arnumber=4101411
        %xn = xn-1 + vn-1*T + k*T(xp - x1)+ (T/2)*k*T*(xp-x1) + T/2*(T*az)
        pz = pz + vz*dt + kc(1)*dt*(h_bar - pz) + (dt/2)*kc(2)*(h_bar - pz) + (dt/2)*(dt*az);
        %pz = pz + vz*dt + kc(1)*dt*(h_pr - pz) + (dt/2)*kc(2)*(h_pr - pz) + (dt/2)*(dt*az);
        %vn = vn-1 + k*T(xp-x1) + (T*az)
        vz = vz + kc(2)*dt*(h_bar - pz) + (dt*az);  
        
        %Impose bounds for the estimates
        if pz > 2
            h_fus = 2;
        elseif pz < 0
            h_fus = 0;
        else
            h_fus = pz;
        end
        
        u=0; %Reset semaphore
    end
 
    %Perform VLP method
    if (t == round(Rx_time_pr(pr_ctr)/10)*10)
            
        %Obtain power received at time t       
        Pr=[A(1)*Rx_Pr1(pr_ctr), A(2)*Rx_Pr2(pr_ctr), A(3)*Rx_Pr3(pr_ctr), A(4)*Rx_Pr4(pr_ctr)];
    
        %Estimate height from Pr using simple NN approach (not used)
        h_pr = W(1)*(W(2)*Rx_Pr1(pr_ctr) + W(3)*Rx_Pr2(pr_ctr) + W(4)*Rx_Pr3(pr_ctr) + W(5)*Rx_Pr4(pr_ctr)) + W(6);
          
        %Estimate distance between Txs and Rxs assuming that they are parallel
        %to each other with the fused height estimation
        D_est = distance_est_parallel(m, k, Aeff, h_fus, Pt, Pr); 
    
        %Calculate x and y coordinates using the MLE method
        if v==1 %Check if parallel Rx/Tx can be assumed
            [MLE_est_2D] = MLE_method_2D(D_est,X,Y);
        end
        v=0;
    
        %Calculate x and y coordinates using using the 3 strongest RSS signals
        %[D3, X3, Y3, Z3] = get_max_rss(D_est,X,Y,Z); 
        %[MLE_est_2D, ~] = MLE_method_2D(D3,X3,Y3);
    
        %Combine h and 2D estimates
        MLE_est = [MLE_est_2D; h_fus]; 
        
        %Increase Pr counter
        pr_ctr = pr_ctr + 1;
            
        %Calculate error using the Euclidean norm
        MLE_err= norm(MLE_est - Rx');
        MLE_all_err=[MLE_all_err MLE_err];

        %Save trajectory every ~0.5s
        Rx_all = [Rx_all Rx'];
        est_all = [est_all MLE_est];
            
        %Store height estimates at the time of the VLP calculation
        all_h = [all_h Rx(3)];   
        all_h_pr = [all_h_pr h_pr];
        all_h_bar = [all_h_bar h_bar];
        all_h_acc = [all_h_acc pz];
        all_h_fus = [all_h_fus h_fus];
        
        %Store angles at the time of the VLP calculation
        all_roll = [all_roll Rx_roll(state_ctr)];
        all_pitch = [all_pitch Rx_pitch(state_ctr)];
  
    end 
        
end

%Plots
%Ground truth vs estimation - 3D
figure(1)
    %Ground truth
plot3(Rx_all(1,:), Rx_all(2,:), Rx_all(3,:), 'og')
hold on
    %Estimation
plot3(est_all(1,:), est_all(2,:), est_all(3,:), '*r')
    %Fixed transmitters
plot3(Tx1(1),Tx1(2),Tx1(3), 'b*');
plot3(Tx2(1),Tx2(2),Tx2(3), 'g*');
plot3(Tx3(1),Tx3(2),Tx3(3), 'r*');
plot3(Tx4(1),Tx4(2),Tx4(3), 'k*');
% for i=1:length(Rx_all)
% plot3([est_all(1,i) Rx_all(1,i)], [est_all(2,i) Rx_all(2,i)], [est_all(3,i) Rx_all(3,i)], 'b');
% end
    %Trajectory
plot3([est_all(1,:) Rx_all(1,:)], [est_all(2,:) Rx_all(2,:)], [est_all(3,:) Rx_all(3,:)], 'r');
hold off
xlabel('x')
ylabel('y')
zlabel('z')
legend('Ground truth', 'Estimate', 'Tx1', 'Tx2', 'Tx3', 'Tx4','Trajectory');
title('3D Drone trajectory - Ground truth vs estimation ');

%Ground truth vs estimation - Height
figure(2)
hold on
plot(all_h) %Ground truth
plot(all_h_bar) %Barometer
plot(all_h_acc) %Accelerometer
plot(all_h_fus) %Fused
plot(all_h_pr) %H est from simple NN approach using Pr
xlabel('time step')
ylabel('height (m)')
legend('Ground truth', 'Barometer', 'Accelerometer', 'Fused', 'NN Pr')
title('Height trajectory - Ground truth vs estimations');

%Plot to visualize the influence of inclination in error
figure(3)
plot(abs(all_pitch)+ abs(all_roll))
hold on
plot(MLE_all_err*10)
xlabel('time step')
ylabel('Error (m)')
legend('abs(pitch)+abs(roll)', '10*Error')
title('Error w.r.t. inclination');


%Compute statistics
disp("VLP 2D+h estimate: ");
disp("Mean error: " + mean(MLE_all_err));
disp("Median error: " + median(MLE_all_err));
disp("Max error: " + max(MLE_all_err));
disp("Std dev: " + std(MLE_all_err));

%Routine to extract the furthest estimated distance out
function [newD, newX, newY, newZ] = get_max_rss(D,X,Y,Z)

    [~, index] = max(D);
    newX=[];
    newY=[];
    newZ=[];
    newD=[];
    
    n=length(X);
    for i=1:n
        if (i~=index)
            newX=[newX X(i)];
            newY=[newY Y(i)];
            newZ=[newZ Z(i)];
            newD=[newD D(i)];
        end
    end
end

%Estimate distance assuming parallel Tx/Rx
function D_est = distance_est_parallel(m, k, Aeff, h_est, Pt, Pr)
    D_est = zeros(1,4);
    
    for i=1:4
        D_est(i) = (k*(m+1)*Aeff*Pt*(h_est^(m+1))/(2*pi*Pr(i)))^(1/(m+3));         
    end 
end

%Calculate x and y position using Maximum Likelihood Estimation method
function [MLE_est] = MLE_method_2D(D,X,Y)
    n=length(X); 
    for i=1:n-1
        A(i,1) = 2*(X(i)-X(n));
        A(i,2) = 2*(Y(i)-Y(n));
        b(i,1) = X(i)^2 + Y(i)^2 - D(i)^2 - ((X(n))^2 + Y(n)^2 - D(n)^2);
        W(i,i) = 1/(D(i)^2);
    end
    MLE_est=inv(A'*A)*A'*b;
end