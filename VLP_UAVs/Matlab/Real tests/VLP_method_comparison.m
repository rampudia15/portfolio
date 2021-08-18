%This script plots the trajectory of a drone from a selected flight test.
%The 3D route and height profile is shown for Firefly and two SoA methods
%as described in the thesis document. 
%
% Ricardo Ampudia <r.ampudia15@gmail.com>
% Last modification: August 2021

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
m=14;
Pt=2.7*0.5; %Pnom*Duty (we drive the LEDs at 2.7W instead of 4.7W)
Aeff=5.2; %[mm^2]
k=1; %Not used

%Load data logged from crazyflie tests (uncomment desired test)   
%load("data/logcurve_t1.mat") 
%load("data/logcurve_t2.mat")
%load("data/logcurve_t3.mat")
%load("data/logcurve_t4.mat")
load("data/logcurve_t5.mat") %For Firefly and Indirect 
%load("data/logcurve_t6.mat")
%load("data/logcurve_t7.mat") 
%load("data/logcurve_t8.mat") %PSO

%Save logged data to local variables
    %x,y,z,roll,pitch,yaw
Rx_time_state = xyzrpy.time;
Rx_x = xyzrpy.x;
Rx_y = xyzrpy.y;
Rx_z = xyzrpy.z;
    %Rotate 90° so yaw is centered at 0°
Rx_roll = xyzrpy.pitch;
Rx_pitch = -xyzrpy.roll;
Rx_yaw = xyzrpy.yaw-90; 

    %Power received
Rx_time_pr = pr1234.time;
Rx_Pr1 = pr1234.Pr1*pi/4;
Rx_Pr2 = pr1234.Pr2*pi/4;
Rx_Pr3 = pr1234.Pr3*pi/4;
Rx_Pr4 = pr1234.Pr4*pi/4;

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
Rx_ax = lowpass(Rx_ax, 0.1, 50);
Rx_ay = lowpass(Rx_ay, 0.1, 50);
Rx_az = lowpass(Rx_az, 0.1, 50);

    %Moving average filter for barometer
B=1/5*ones(5,1); %Window length = 5 samples
Rx_bar = filter(B,1,Rx_bar);

    %LP filter for power received
Rx_Pr1=lowpass(Rx_Pr1, 0.1, 10); 
Rx_Pr2=lowpass(Rx_Pr2, 0.1, 10); 
Rx_Pr3=lowpass(Rx_Pr3, 0.1, 10); 
Rx_Pr4=lowpass(Rx_Pr4, 0.1, 10); 

%Determine acceleration bias and set as constant
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

    %PSO 3D method
options = optimoptions('particleswarm','SwarmSize',40,'MaxIterations',20*3); %maxiter = 20*n_variables
lb=[0;0;0]; %lower bound
ub=[2;2;2]; %upper bound
PSO_est_3D = rx_center(1:3);
PSO_err = 0;
PSO_all_err = [];

    %Iterative height method with LLS
LLS_est_3D = rx_center(1:3)';
LLS_err = 0;
LLS_all_err = [];
best_LLS=[0;0;0.2]; %start position
best_h=0;
min_h=0;

    %Counters for the position of the logged data 
state_ctr = 2;
pr_ctr = 2;
acc_ctr = 2;
bar_ctr = 2;

    %Containers to store ground truth and estimation points to compute
    %statistics at the end
Rx_all = [];
est_all_MLE = [];
est_all_LLS = [];
est_all_PSO = [];

    %Containers to store angles at the time of VLP method computation
all_roll = [];
all_pitch = [];
all_ax = [];
all_ay = [];
all_x = [];
all_y = [];

    %Containers to store all individual h estimates to plot at the end
all_h = []; %Real height
all_h_bar = []; %Barometer estimate
all_h_acc = []; %Accelerometer estimate
all_h_fus = []; %Fused estimate with complementary filter
all_h_pr = []; %VLP estimate

    %Complementary filter variables
az=0;
vz=0;
pz=0.2;

ax=0;
vx=0;
px=1;

ay=0;
vy=0;
py=1;

h_pr=0.2; %start position
h_bar=0.2;
h_fus = 0.2; 
pz0=0.2;
h_bar0 = 0.2;
old_h_bar0 = 0.2;
epsilon=0.003; %For long-term drift correction

%Complementary filter gain
f=2; %Larger f means I trust the accelerometer more than the barometer
kc=[1.4/f,1/f];
vel_drift = 0;
pos_drift = 0;

u=0; %Semaphore used to enable complementary filter calculation

for t=start_time:t_step:end_time %Time in seconds
    %t %Display time
    
    %Advance counters until start time is reached
    if (Rx_time_state(state_ctr) <= round(Rx_time_state(10)/10)*10)
        state_ctr = state_ctr+1;
        pr_ctr = pr_ctr+1;
        acc_ctr = acc_ctr+1;
        bar_ctr = bar_ctr+1;   
    else
        %Stop simulation if the end of any logged measurement is reached
        if state_ctr >= length(Rx_time_state) || pr_ctr >= length(Rx_time_pr) || acc_ctr >= length(Rx_time_acc) || bar_ctr >= length(Rx_time_bar)
            break
        end

        %Update ground truth coordinates of the receiver
        if (t == round(Rx_time_state(state_ctr)/10)*10)
            Rx=[rx_center(1)+Rx_x(state_ctr),rx_center(2)+Rx_y(state_ctr),rx_center(3)+Rx_z(state_ctr)]; %Apply offset to reference of the system
         
            state_ctr = state_ctr + 1;      
        end

        %Update accelerometer measurement
        if (t == round(Rx_time_acc(acc_ctr)/10)*10)
            dt=(Rx_time_acc(acc_ctr)-Rx_time_acc(acc_ctr-1))/1000;         
            %Acceleration calculation with rotation matrix and moving
            %average filter of L=5 (ugly implementation)
            acc = rotation([Rx_roll(state_ctr);Rx_pitch(state_ctr);Rx_yaw(state_ctr)])*[0;0;
                (((Rx_az(acc_ctr)+Rx_az(acc_ctr-1)+Rx_az(acc_ctr-2) +Rx_az(acc_ctr-3) +Rx_az(acc_ctr-4))/5)-acc_bias(3))*9.81];
            ax = acc(1);
            ay = acc(2);
            az = acc(3);
            acc_ctr = acc_ctr + 1;
            u=1; %Enable semaphore
        end

        %Update barometer measurement
        if (t == round(Rx_time_bar(bar_ctr)/10)*10)
            dt=(Rx_time_bar(bar_ctr)-Rx_time_bar(bar_ctr-1))/1000;
            if (Rx_bar(bar_ctr) > 0)
                    h_bar0 = Rx_bar(bar_ctr);
                    %Long term drift correction
                        %Correct only if VLP can be trusted (tilt < 3°)
                    if (Rx_roll(state_ctr) < 3 && Rx_pitch(state_ctr) < 3)
                        h_bar = (h_bar + (h_bar0-old_h_bar0))*(1-epsilon) + epsilon*best_LLS(3);
                    else
                        h_bar = (h_bar + (h_bar0-old_h_bar0));
                    end
                    old_h_bar0 = h_bar0;
            end
            bar_ctr = bar_ctr + 1;
        end

        %Update complementary filter
        if u==1
            %Z estimate
            pz = pz + vz*dt + kc(1)*dt*(h_bar - pz) + (dt/2)*kc(2)*(h_bar - pz) + (dt/2)*(az*dt);

            %vn = vn-1 + k*T(xp-x1) + (T*az)
            vz = vz + kc(2)*dt*(h_bar - pz) + (az*dt); 
            
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
            dt=(Rx_time_pr(pr_ctr)-Rx_time_pr(pr_ctr-1))/1000;
            
            %Obtain power received at time t       
            Pr=[Rx_Pr1(pr_ctr), Rx_Pr2(pr_ctr), Rx_Pr3(pr_ctr), Rx_Pr4(pr_ctr)];

            %Estimate distance between Txs and Rxs assuming that they are parallel
            D_est = distance_est_parallel(m, k, Aeff, h_fus, Pt, Pr); 
            
            %Horizontal distance for 2D
            for i=1:4
                r(i) = sqrt(D_est(i)^2 - h_fus^2);
            end
        
            %Calculate position with different methods
            
            %a) 2D + h MLE method
            [MLE_est_2D] = MLE_method_2D(r,X,Y);
            
                %Recalculate the distance considering tilt information
            xr=MLE_est_2D(1);
            yr=MLE_est_2D(2);
            
            for i=1:4
                inc_angle(i) = acosd((xr-X(i))*cosd(Rx_roll(state_ctr))*sind(Rx_pitch(state_ctr)) + ...
                (yr-Y(i))*sind(Rx_roll(state_ctr))*sind(Rx_pitch(state_ctr)) + (h_fus - Z(i))*cosd(Rx_pitch(state_ctr))/D_est(i));
            end
            
            for i=1:4
                D_est2(i)= sqrt((k*Pt*(m+1)*Aeff/(Pr(i)*2*pi))*(((h_fus/D_est(i)))^m)*cosd(inc_angle(i))); %Lambertian model
            end   
            
                %Horizontal distance for 2D
            for i=1:4
                r(i) = sqrt(D_est2(i)^2 - h_fus^2);
            end   
            if(sum(isnan(D_est2))>0) %Check computation is real
            else
               [MLE_est_2D] = MLE_method_2D(r,X,Y);
            end
                %Combine output of the complementary filter
            MLE_est_2D_h = [MLE_est_2D(1); MLE_est_2D(2); h_fus];
            
            %b) 3D PSO method
            pso_func = @(PSO_est_3D)obj_func_pso(PSO_est_3D,Pr,X,Y,Z, Pt, Aeff, m, k);
            [PSO_est_3D,fval,exitflag,output] = particleswarm(pso_func, 3, lb, ub);
            PSO_est_3D = PSO_est_3D';
            
            %c) Iterative 2D+H method
            best_res = 100; %Initialize to a big residual
            for h=0:0.01:2
                %Obtain D(h)
                D_est = distance_est_parallel(m, k, Aeff, h, Pt, Pr);        
                %Obtain 2D position
                LLS_est_2D = LLS_method_2D(D_est,X,Y);
                %Take positive solution and height estimation
                LLS_est_3D = abs([LLS_est_2D(1); LLS_est_2D(2); h]);
                %Calculate the LSE
                res = obj_func(LLS_est_3D,D_est,X,Y,Z);
                
                %Take the result with the min LSE
                if res < best_res
                    best_LLS = LLS_est_3D;
                    best_res = res;
                end
            end
 
            %Increase Pr counter
            pr_ctr = pr_ctr + 1;

            %Calculate error using the Euclidean norm
            if Rx(3) > min_h
                MLE_err= norm(MLE_est_2D_h - Rx');
                MLE_all_err=[MLE_all_err MLE_err];

                LLS_err= norm(best_LLS - Rx');
                LLS_all_err=[LLS_all_err LLS_err];

                PSO_err= norm(PSO_est_3D - Rx');
                PSO_all_err=[PSO_all_err PSO_err];
            end

            %Save points in the trajectory
            Rx_all = [Rx_all Rx'];
            est_all_MLE = [est_all_MLE MLE_est_2D_h];
            est_all_PSO = [est_all_PSO PSO_est_3D];
            est_all_LLS = [est_all_LLS best_LLS];

            %Save height estimates
            all_h = [all_h Rx(3)];   
            all_h_bar = [all_h_bar h_bar];
            all_h_acc = [all_h_acc pos_drift];
            all_h_fus = [all_h_fus h_fus];
            all_h_pr = [all_h_pr best_LLS(3)];

            %Save angle and state estimates 
            all_roll = [all_roll Rx_roll(state_ctr)];
            all_pitch = [all_pitch Rx_pitch(state_ctr)];
            yaw = Rx_yaw(state_ctr);
            all_x = [all_x Rx_x(state_ctr)];
            all_y = [all_y Rx_y(state_ctr)];
            all_ax = [all_ax ax];
            all_ay = [all_ay ay];
        end 
    end
end

%Plots
green= [0.4660, 0.6740, 0.1880];
blue= [0, 0.4470, 0.7410];
red=[0.8500, 0.3250, 0.0980];
grey=[0.25, 0.25, 0.25];
yellow =[0.9290, 0.6940, 0.1250];
purple=[0.4940, 0.1840, 0.5560];

%Ground truth vs estimation - 3D plot - Indirect H
hold off
figure(1)
    %Ground truth
plot3(Rx_all(1,:), Rx_all(2,:), Rx_all(3,:), 'sg', 'LineWidth', 1.2); %, 'Color', purple)%,'Color', green) %og
hold on
    %Estimation
plot3(est_all_MLE(1,:), est_all_MLE(2,:), est_all_MLE(3,:), 'o', 'Color', red, 'LineWidth', 1.2) %ro
plot3(est_all_LLS(1,1:3:end), est_all_LLS(2,1:3:end), est_all_LLS(3,1:3:end), '*', 'Color', blue, 'LineWidth', 1) %bo
    %Fixed transmitters
plot3(Tx1(1),Tx1(2),Tx1(3), 'ko', 'MarkerFaceColor','k');
plot3(Tx2(1),Tx2(2),Tx2(3), 'ko', 'MarkerFaceColor','k');
plot3(Tx3(1),Tx3(2),Tx3(3), 'ko', 'MarkerFaceColor','k');
plot3(Tx4(1),Tx4(2),Tx4(3), 'ko','MarkerFaceColor','k');
    %Trajectory
plot3([est_all_MLE(1,:) Rx_all(1,:)], [est_all_MLE(2,:) Rx_all(2,:)], [est_all_MLE(3,:) Rx_all(3,:)], 'r');
plot3([est_all_LLS(1,:) Rx_all(1,:)], [est_all_LLS(2,:) Rx_all(2,:)], [est_all_LLS(3,:) Rx_all(3,:)], 'Color' , green );
hold off
xlabel('x','FontSize', 16);
ylabel('y','FontSize', 16);
zlabel('z','FontSize', 16);
[h,icons] =legend('Ground truth', 'Firefly', 'Indirect-H [12]', 'TXs', 'FontSize', 16);
icons = findobj(icons,'Type','line');
    %Find lines that use a marker
icons = findobj(icons,'Marker','none','-xor');
    %Resize the marker in the legend
set(icons(1:2),'LineWidth',2);
set(icons(4),'LineWidth',2);
%title('3D Drone trajectory - Ground truth vs estimations', 'FontSize', 20);
set(gca,'FontSize',16)
grid
height=500;
width=700;
set(gcf,'position',[0,0,width,height])
view(407,24)

%Ground truth vs estimation - 3D plot - 3D PSO
figure(2)
    %Ground truth
plot3(Rx_all(1,:), Rx_all(2,:), Rx_all(3,:), 'sg', 'LineWidth', 1.2) %og
hold on
    %Estimation
plot3(est_all_MLE(1,:), est_all_MLE(2,:), est_all_MLE(3,:), 'o', 'Color', red, 'LineWidth', 1.2) %ro
plot3(est_all_PSO(1,:), est_all_PSO(2,:), est_all_PSO(3,:), 'x', 'Color', grey, 'LineWidth', 1) %ok
    %Fixed transmitters
plot3(Tx1(1),Tx1(2),Tx1(3), 'ko', 'MarkerFaceColor','k'); %bo
plot3(Tx2(1),Tx2(2),Tx2(3), 'ko', 'MarkerFaceColor','k'); %go
plot3(Tx3(1),Tx3(2),Tx3(3), 'ko', 'MarkerFaceColor','k'); %ro
plot3(Tx4(1),Tx4(2),Tx4(3), 'ko','MarkerFaceColor','k'); %ko
    %Trajectory
plot3([est_all_MLE(1,:) Rx_all(1,:)], [est_all_MLE(2,:) Rx_all(2,:)], [est_all_MLE(3,:) Rx_all(3,:)], 'Color', red);
hold off
xlabel('x','FontSize', 16);
ylabel('y','FontSize', 16);
zlabel('z','FontSize', 16);
[h,icons] =legend('Ground truth', 'Firefly', '3D PSO [28]', 'TXs','FontSize', 16); 
icons = findobj(icons,'Type','line');
    %Find lines that use a marker
icons = findobj(icons,'Marker','none','-xor');
    %Resize the marker in the legend
set(icons,'LineWidth',2);
%title('3D Drone trajectory - Ground truth vs estimations', 'FontSize', 20);
set(gca,'FontSize',16)
grid
height=500;
width=700;
set(gcf,'position',[0,0,width,height])
view(407,24)

%Ground truth vs estimation - h plot - Indirect-H
figure(3)
L=length(all_h);
hold off
subplot(16,1,[1 10])
plot(1:5:L, all_h(1:5:end), '-sg', 'LineWidth', 1.3, 'MarkerSize', 6)%, 'Color', green) 1.2
hold on
plot(1:5:L, all_h_fus(1:5:end), '-o', 'LineWidth', 1.2, 'MarkerSize', 6, 'Color', red) %'LineWidth', 2,
plot(1:5:L, all_h_pr(1:5:end), '-*', 'LineWidth', 0.2, 'MarkerSize', 7, 'Color', blue)
xlabel('','FontSize', 16);
ylabel('Height (m)','FontSize', 16);
legend('Ground truth', 'Firefly', 'Indirect-H [12]','FontSize', 16);
set(gca,'FontSize',16)
xticklabels('');
grid

subplot(16,1,[11 15])
plot(1:L, abs(all_pitch), '-', 'LineWidth', 2, 'Color', yellow)
hold on
plot(1:L, abs(all_roll), ':', 'LineWidth', 2, 'Color', purple)
xlabel('Time step','FontSize', 16);
ylabel('| Angle [°] |','FontSize', 16);
legend('Pitch', 'Roll','FontSize', 16);
%title('Height estimation using sensor fusion', 'FontSize', 20);
set(gca,'FontSize',16)
ylim([0 7])
grid
ytickformat('%,.1f')
hold off

%Compute statistics
disp("VLP 2D+h estimate: ");
disp("Mean error: " + mean(MLE_all_err));
disp("Median error: " + median(MLE_all_err));
disp("Max error: " + max(MLE_all_err));
disp("Std dev: " + std(MLE_all_err));

disp("PSO 3D estimate: ");
disp("Mean error: " + mean(PSO_all_err));
disp("Median error: " + median(PSO_all_err));
disp("Max error: " + max(PSO_all_err));
disp("Std dev: " + std(PSO_all_err));

disp("LLS 3D estimate via iterative h estimation: ");
disp("Mean error: " + mean(LLS_all_err));
disp("Median error: " + median(LLS_all_err));
disp("Max error: " + max(LLS_all_err));
disp("Std dev: " + std(LLS_all_err));


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

%Calculate x and y position using Linear Least Square method
function [LLS_est] = LLS_method_2D(D,X,Y)
    n=length(X); 
    for i=1:n-1
        A(i,1) = -2*(X(i)-X(n));
        A(i,2) = -2*(Y(i)-Y(n));
        b(i,1) = (D(i)^2 - X(i)^2 - Y(i)^2 - D(n)^2 + X(n)^2 + Y(n)^2);
    end
    LLS_est=inv(A'*A)*A'*b;
end

%Rotation matix according to the ENU convention
function R = rotation(angles)
    r = deg2rad(angles(1)); %roll in radians
    p = deg2rad(angles(2)); %pitch in radians
    y = deg2rad(angles(3)); %yaw in radians
    
    roll=inv([1 0 0    ; 0 cos(r) -sin(r); 0 sin(r) cos(r)]);
    pitch=([cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)]);
    yaw=inv([cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1]);
    
   R=yaw*pitch*roll;
end

%Objetive function for Indirect-H
function sum=obj_func(est,D,X,Y,Z)
    sum=0;  
    for i=1:length(X)
        sum = sum + (D(i) - sqrt( (X(i)-est(1))^2 + (Y(i)-est(2))^2 + (Z(i)-est(3))^2))^2;
    end 
    sum=sum/length(X);
    
end

%Objective function for 3D PSO
function sum=obj_func_pso(est,Pr,X,Y,Z, Pt, Apd, m, k)
    sum=0;  
    C=k*Apd*(m+1)/(2*pi);
    for i=1:length(X)
        d= sqrt( (X(i)-est(1))^2 + (Y(i)-est(2))^2 + (Z(i)-est(3))^2);
        HLOS = C*(est(3)^(m+1))/(d^(m+3));
        sum = sum + (HLOS - Pr(i)/Pt)^2;
    end  
end