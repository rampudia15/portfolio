%Script that compares 3D RSS based methods and computes statistics from 
%different tests

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
m=12; %Lambertian order
k=0.45; %Calibration factor to account for lateral error deviation
Aeff=5.2; %[mm^2]
Pt=2; %[Watts]
%A=[0.86; 1.06; 1.06; 1.29]; %Calibration of individual LEDs intensity
A=[1; 1; 1; 1]; %Calibration of individual LEDs intensity

all_mean = [];
all_median= [];
all_max = [];
all_min = [];
all_std =[];

for test=1:8
    
    %Load data logged from crazyflie tests  
    switch test
        case 1
            load("data/logcurve_t1.mat")  
        case 2
            load("data/logcurve_t2.mat")  
        case 3
            load("data/logcurve_t3.mat")  
        case 4
            load("data/logcurve_t4.mat")  
        case 5
            load("data/logcurve_t5.mat")  
        case 6
            load("data/logcurve_t6.mat")  
        case 7
            load("data/logcurve_t7.mat")  
        case 8
            load("data/logcurve_t8.mat")  
    end
          

    %Save logged data to local variables
        %x,y,z,roll,pitch,yaw
    Rx_time_state = xyzrpy.time;
    Rx_x = xyzrpy.x;
    Rx_y = xyzrpy.y;
    Rx_z = xyzrpy.z;
    Rx_roll = xyzrpy.roll;
    Rx_pitch = xyzrpy.pitch;
    Rx_yaw = xyzrpy.yaw-90; %Rotate 90° so x and y of body frame matches reference frame

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
    %acc_bias=[mean(Rx_ax(1:40)); mean(Rx_ay(1:40)); mean(Rx_az(1:40))];
    acc_bias=[mean(Rx_ax); mean(Rx_ay); mean(Rx_az)];

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
    lb=[0;0;0];
    ub=[2;2;2];
    PSO_est_3D = rx_center(1:3);
    PSO_err = 0;
    PSO_all_err = [];

        %Iterative height method with GN
    GN_est_3D = rx_center(1:3)';
    GN_err = 0;
    GN_all_err = [];
    best_GN=[0;0;0.2];

        %Counters for the position of the logged data 
    state_ctr = 2;
    pr_ctr = 2;
    acc_ctr = 2;
    bar_ctr = 2;

        %Containers to store ground truth and estimation points to compute
        %statistics at the end
    Rx_all = [];
    est_all_MLE = [];
    est_all_GN = [];
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

    h_pr=0.2;
    h_bar=0.2;
    h_fus = 0.2; 
    pz0=0.2;
    
    %Complementary filter gains, kc for z and kc2 for x,y
    f=1; %Larger f means I trust the accelerometer more than the barometer
    kc=[1.4/f,1/f];
    f=0.05;
    kc2=[1.4/f,1/f];
    
%     f=0.05; %Larger f means I trust the accelerometer more than the barometer
%     sigma_w = std(Rx_az(1:40));
%     sigma_v = std(Rx_bar(1:40));
%     kc = (1/f)*[sqrt((2*sigma_w)/sigma_v); sigma_w/sigma_v];
    
    u=0; %Semaphore used to enable complementary filter calculation

    %Perform simulation
    for t=start_time:t_step:end_time %Time in seconds
    t %Display time
    
    %Advance counters until start time is reached
    if (Rx_time_state(state_ctr) <= round(Rx_time_state(10)/10)*10)
        state_ctr = state_ctr+1;
        pr_ctr = pr_ctr+1;
        acc_ctr = acc_ctr+1;
        bar_ctr = bar_ctr+1;   
    else
        %Stop simulation if reached the end of any logged measurement
        if state_ctr >= length(Rx_time_state) || pr_ctr >= length(Rx_time_pr) || acc_ctr >= length(Rx_time_acc) || bar_ctr >= length(Rx_time_bar)
            break
        end

        %Update ground truth coordinates of the receiver
        if (t == round(Rx_time_state(state_ctr)/10)*10)
            Rx=[rx_center(1)+Rx_x(state_ctr),rx_center(2)+Rx_y(state_ctr),rx_center(3)+Rx_z(state_ctr)];
         
            state_ctr = state_ctr + 1;      
        end

        %Update accelerometer measurement
        if (t == round(Rx_time_acc(acc_ctr)/10)*10)
            dt=(Rx_time_acc(acc_ctr)-Rx_time_acc(acc_ctr-1))/1000;         
            %Acceleration calculation with rotation matrix and moving
            %average filter of L=5 (forgive the ugly implementation)
            acc = rotation([Rx_roll(state_ctr);Rx_pitch(state_ctr);Rx_yaw(state_ctr)])*[((Rx_ax(acc_ctr)+Rx_ax(acc_ctr-1)+Rx_ax(acc_ctr-2) +Rx_ax(acc_ctr-3) +Rx_ax(acc_ctr-4))/5-acc_bias(1))*9.81; 
                ( (Rx_ay(acc_ctr)+Rx_ay(acc_ctr-1)+Rx_ay(acc_ctr-2) +Rx_ay(acc_ctr-3) +Rx_ay(acc_ctr-4))/5 -acc_bias(2))*9.81; 
                ((Rx_az(acc_ctr)+Rx_az(acc_ctr-1)+Rx_az(acc_ctr-2) +Rx_az(acc_ctr-3) +Rx_az(acc_ctr-4))/5-acc_bias(3))*9.81];
            ax = acc(1); 
            ay = acc(2); 
            az = acc(3); 
            acc_ctr = acc_ctr + 1;
            u=1; %Enable semaphore
        end

        %Update barometer measurement
        if (t == round(Rx_time_bar(bar_ctr)/10)*10)
            dt=(Rx_time_bar(bar_ctr)-Rx_time_bar(bar_ctr-1))/1000;
            h_bar = Rx_bar(bar_ctr)- Rx_bar(1);
            bar_ctr = bar_ctr + 1;
        end

        %Update complementary filter
        if u==1
            %Z estimate
            pz0 = vz*dt + kc(1)*dt*(h_bar - pz0) + (dt/2)*kc(2)*(h_bar - pz0) + (dt/2)*(az*dt); %delta pos with fused bar and acc data
            alpha=0.1;
            pz = (1-alpha)*(pz + pz0) + best_GN(3)*(alpha); %Long term bar 'zero drift' correction using VLP method
            
            %vn = vn-1 + k*T(xp-x1) + (T*az)
            vz = vz + kc(2)*dt*(h_bar - pz) + (az*dt);  
            
            %X,Y estimates
            px = px + vx*dt + kc2(1)*dt*(MLE_est_2D(1) - px) + (dt/2)*kc2(2)*(MLE_est_2D(1) - px) + (dt/2)*(ax*dt);
            vx = vx + kc2(2)*dt*(MLE_est_2D(1) - px) + (ax*dt); 
            
            py = py + vy*dt + kc2(1)*dt*(MLE_est_2D(2) - py) + (dt/2)*kc2(2)*(MLE_est_2D(2) - py) + (dt/2)*(ay*dt);
            vy = vy + kc2(2)*dt*(MLE_est_2D(2) - py) + (ay*dt); 
            
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
            Pr=[A(1)*Rx_Pr1(pr_ctr), A(2)*Rx_Pr2(pr_ctr), A(3)*Rx_Pr3(pr_ctr), A(4)*Rx_Pr4(pr_ctr)];

            %Estimate distance between Txs and Rxs assuming that they are parallel
            D_est = distance_est_parallel(m, k, Aeff, h_fus, Pt, Pr); 

            %Calculate position with different methods
            
            %a) 2D + h MLE method
            [MLE_est_2D] = MLE_method_2D(D_est,X,Y);
            
                %Recalculate the distance considering tilt information
            xr=MLE_est_2D(1);
            yr=MLE_est_2D(2);
            
            for i=1:4
                inc_angle(i) = acosd((xr-X(i))*cosd(Rx_pitch(state_ctr))*sind(-Rx_roll(state_ctr)) + ...
                (yr-Y(i))*sind(Rx_pitch(state_ctr))*sind(-Rx_roll(state_ctr)) + (h_fus - Z(i))*cosd(-Rx_roll(state_ctr))/D_est(i));
            end
            
            for i=1:4
                D_est2(i)= sqrt( (k*Pt*(m+1)*Aeff/(Pr(i)*2*pi))*(((h_fus/D_est(i)))^m)*cosd(inc_angle(i))); %Lambertian model
            end 
            
            if(sum(isnan(D_est2))>0) %Check computation is real
            else
               [MLE_est_2D] = MLE_method_2D(D_est2,X,Y);
            end
                %Combine output of the complementary filter    
            MLE_est_2D_h = [px; py; h_fus]; 
            
            %b) 3D PSO method
            pso_func = @(PSO_est_3D)obj_func_pso(PSO_est_3D,Pr,X,Y,Z, Pt, Aeff, m, k)
            [PSO_est_3D,fval,exitflag,output] = particleswarm(pso_func, 3, lb, ub);
            PSO_est_3D = PSO_est_3D';
            
            %c) Iterative 2D+H method
            best_res = 100;
            for h=0:0.05:2
                %Obtain D(h)
                D_est = distance_est_parallel(m, k, Aeff, h, Pt, Pr); 
                    
                %Obtain 2D position
                GN_est_2D = LLS_method_2D(D_est,X,Y);
                %Take positive solution and height estimation
                GN_est_3D = abs([GN_est_2D(1); GN_est_2D(2); h]);
                %Calculate the LSE
                res = obj_func(GN_est_3D,D_est,X,Y,Z);
                
                %Take the result with the min LSE
                if res < best_res
                    best_GN = GN_est_3D;
                    best_res = res;
                end
            end
 
            %Increase Pr counter
            pr_ctr = pr_ctr + 1;

            %Calculate error using the Euclidean norm
            MLE_err= norm(MLE_est_2D_h - Rx');
            MLE_all_err=[MLE_all_err MLE_err];
            
            GN_err= norm(best_GN - Rx');
            GN_all_err=[GN_all_err GN_err];
            
            PSO_err= norm(PSO_est_3D - Rx');
            PSO_all_err=[PSO_all_err PSO_err];

            %Save points in the trajectory
            Rx_all = [Rx_all Rx'];
            est_all_MLE = [est_all_MLE MLE_est_2D_h];
            est_all_PSO = [est_all_PSO PSO_est_3D];
            est_all_GN = [est_all_GN best_GN];

            %Save height estimates
            all_h = [all_h Rx(3)];   
            all_h_bar = [all_h_bar h_bar];
            all_h_acc = [all_h_acc pz];
            all_h_fus = [all_h_fus h_fus];

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

    
    %Gather statistics across different tests
    all_mean(1, test) = mean(MLE_all_err);
    all_median(1, test) = median(MLE_all_err);
    all_max(1, test) = max(MLE_all_err);
    all_min(1, test) = min(MLE_all_err);
    all_std(1, test) = std(MLE_all_err);
    
    all_mean(2, test) = mean(PSO_all_err);
    all_median(2, test) = median(PSO_all_err);
    all_max(2, test) = max(PSO_all_err);
    all_min(2, test) = min(PSO_all_err);
    all_std(2, test) = std(PSO_all_err);
    
    all_mean(3, test) = mean(GN_all_err);
    all_median(3, test) = median(GN_all_err);
    all_max(3, test) = max(GN_all_err);
    all_min(3, test) = min(GN_all_err);
    all_std(3, test) = std(GN_all_err);
    
    MLE_all_err = [];
    PSO_all_err = [];
    GN_all_err = [];
    
end

%Plots
figure(1)
plot(all_mean(1,:), '-s')
hold on
plot(all_mean(2,:), '-s')
plot(all_mean(3,:), '-s')
xlabel('Test number')
ylabel('Mean error')
legend('Direct h + 2D MLE', '3D PSO', 'Indirect h + 2D LLS')
title('Mean error of different methods');

figure(2)
plot(all_median(1,:), '-s')
hold on
plot(all_median(2,:), '-s')
plot(all_median(3,:), '-s')
xlabel('Test number')
ylabel('Median error')
legend('Direct h + 2D MLE', '3D PSO', 'Indirect h + 2D LLS')
title('Median error of different methods');

figure(3)
plot(all_max(1,:), '-s')
hold on
plot(all_max(2,:), '-s')
plot(all_max(3,:), '-s')
xlabel('Test number')
ylabel('Maximum error')
legend('Direct h + 2D MLE', '3D PSO', 'Indirect h + 2D LLS')
title('Maximum error of different methods');

figure(4)
plot(all_std(1,:), '-s')
hold on
plot(all_std(2,:), '-s')
plot(all_std(3,:), '-s')
xlabel('Test number')
ylabel('Standard deviation of the error')
legend('Direct h + 2D MLE', '3D PSO', 'Indirect h + 2D LLS')
title('Standard deviation of error of different methods');

figure(5)
x1 = 1:8;
y1 = all_mean(1,:);
yneg1=all_mean(1,:)-all_min(1,:);
ypos1=all_mean(1,:)-all_max(1,:);
errorbar(x1-0.2,y1,yneg1,ypos1,'s', 'LineWidth', 3, 'MarkerSize',12)
hold on
y2 = all_mean(2,:);
yneg2=all_mean(2,:)-all_min(2,:);
ypos2=all_mean(2,:)-all_max(2,:);
errorbar(x1,y2,yneg2,ypos2,'s', 'LineWidth', 3, 'MarkerSize',12)

y3 = all_mean(3,:);
yneg3=all_mean(3,:)-all_min(3,:);
ypos3=all_mean(3,:)-all_max(3,:);
errorbar(x1+0.2,y3,yneg3,ypos3,'s', 'LineWidth', 3, 'MarkerSize',12)
xlabel('Test number', 'FontSize', 24);
ylabel('Error (m)', 'FontSize', 24);
legend('2D+H (direct height estimation)', '3D PSO', '3D trilateration (indirect height estimation)', 'FontSize', 24);
%title('Mean-error plots', 'FontSize', 24);
set(gca,'FontSize',24)
grid on
grid minor

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

%Calculate x and y position using Maximum Likelihood Estimation method
function [LLS_est] = LLS_method_2D(D,X,Y)
    n=length(X); 
    for i=1:n-1
        A(i,1) = (X(i)-X(n));
        A(i,2) = (Y(i)-Y(n));
        b(i,1) = 0.5*(D(i)^2 - X(i)^2 - Y(i)^2 - D(n)^2 + X(n)^2 + Y(n)^2);
    end
    LLS_est=inv(A'*A)*A'*b;
end

function R = rotation(angles)
    r = deg2rad(angles(1)); %roll in radians
    p = deg2rad(angles(2)); %pitch in radians
    y = deg2rad(angles(3)); %yaw in radians
    
    roll=inv([1 0 0    ; 0 cos(r) -sin(r); 0 sin(r) cos(r)]);
    pitch=([cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)]);
    yaw=inv([cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1]);
    
   R=yaw*pitch*roll;
end

function sum=obj_func(est,D,X,Y,Z)
    sum=0;  
    for i=1:length(X)
        sum = sum + (D(i) - sqrt( (X(i)-est(1))^2 + (Y(i)-est(2))^2 + (Z(i)-est(3))^2))^2;
    end 
    sum=sum/length(X);
    
end

function sum=obj_func_pso(est,Pr,X,Y,Z, Pt, Apd, m, k)
    sum=0;  
    C=k*Apd*(m+1)/(2*pi);
    for i=1:length(X)
        sum = sum + ( C*(est(3)^(m+1))/sqrt( (X(i)-est(1))^2 + (Y(i)-est(2))^2 + (Z(i)-est(3))^2) - Pr(i)/Pt)^2;
    end 
    
end

function [MLE_est, iter, tol] = GN_method_3D(MLE_est,D,X,Y,Z,max_iter,tol)
    res=obj_func_3D(MLE_est,D,X,Y,Z);
    iter=0;
    alpha=1;
    while iter<max_iter && tol<res
        J=jacobian_3D(MLE_est,X,Y,Z);
        R=residuals_3D(MLE_est,D,X,Y,Z)';
        MLE_est=MLE_est-((J'*J)\(J'))*R*alpha;
        res=obj_func_3D(MLE_est,D,X,Y,Z);
        iter=iter+1;
    end
end

function sum=obj_func_3D(est,D,X,Y,Z)
    sum=0;  
    for i=1:length(X)
        sum = sum + (D(i) - sqrt( (X(i)-est(1))^2 + (Y(i)-est(2))^2 + (Z(i)-est(3))^2))^2;
    end 
    sum=1/length(X);
end

function R=residuals_3D(est,D,X,Y,Z)
    R=[];  
    for i=1:length(X)
        R(i) = D(i) - sqrt((X(i)-est(1))^2 + (Y(i)-est(2))^2 + (Z(i)-est(3))^2);
    end 
end

function J=jacobian_3D(est,X,Y,Z)
    J=[];
    for i=1:length(X)
        J(i,1)= 0.5*(2*X(i) - 2*est(1))*((X(i)-est(1))^2 + (Y(i)-est(2))^2 + (Z(i)-est(3))^2)^(-0.5);
        J(i,2)= 0.5*(2*Y(i) - 2*est(2))*((X(i)-est(1))^2 + (Y(i)-est(2))^2 + (Z(i)-est(3))^2)^(-0.5);
        J(i,3)= 0.5*(2*Z(i) - 2*est(3))*((X(i)-est(1))^2 + (Y(i)-est(2))^2 + (Z(i)-est(3))^2)^(-0.5);
    end
end