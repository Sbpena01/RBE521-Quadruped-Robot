clear all;
close all;
clc;

% Order of legs in P vectors:
% HR FR HL FL
% ------ !!! All commands at 100 Hz !!! ------- %

%% Walking Gait
% HR FR HL FL
clc;

t_end_hw = 20;
T_hw = 4; %s
iter_hw = 1;
t_hw = linspace(0, T_hw*(iter_hw + 0.75), (T_hw*(iter_hw + 0.75) - 0) * 100);


stride_length_hw = 10; %cm
L_hw = stride_length_hw/2;
H = 5; %cm

duty_cycle = 0.75;
beta_hw = 1 - duty_cycle;
w = 1; %unit unk

for i=1:size(t_hw,2)
    ts_arg_hw(1,i) = mod(t_hw(i),T_hw);
    ts_arg_hw(2,i) = mod(t_hw(i) + 3*T_hw/4, T_hw);
    ts_arg_hw(3,i) = mod(t_hw(i) + 2*T_hw/4, T_hw);
    ts_arg_hw(4,i) = mod(t_hw(i) + T_hw/4, T_hw);
end

vHW = 2*L_hw/(beta_hw * T_hw);
equalizer = 3;
a_x_hw = [-L_hw, (-8/3)*(L_hw/T_hw), (640/3)*(L_hw/T_hw^2), (-5120/3)*(L_hw/T_hw^3), (20480/3)*(L_hw/T_hw^4), (-32768/3)*(L_hw/T_hw^5)]';
a_z_hw = [0, 0, 256*H/T_hw^2, -2048*H/T_hw^3, 4096*H/T_hw^4, 0]';
t_s = zeros(4,size(ts_arg_hw,2),6);
for i=1:4
    for j=1:size(ts_arg_hw,2)
        t_s(i,j,:) = [1, ts_arg_hw(i,j), ts_arg_hw(i,j)^2, ts_arg_hw(i,j)^3, ts_arg_hw(i,j)^4, ts_arg_hw(i,j)^5]';
        ts_vec = squeeze(t_s(i,j,:));
        if ts_arg_hw(i,j) < beta_hw*T_hw
            Px_hw(i,j) = a_x_hw' * ts_vec;
            Pz_hw(i,j) = a_z_hw' * ts_vec;
        else
            Px_hw(i,j) = -vHW * (ts_arg_hw(i,j) - beta_hw*T_hw)/equalizer + L_hw;
            Pz_hw(i,j) = 0;
        end
        Py_hw(i,j) = -w * sin(2*pi*(ts_arg_hw(i,j) - T_hw)/T_hw);
    end
end
figure
tiledlayout("flow")
nexttile
plot(Px_hw(1,:),Pz_hw(1,:));
title('Hybrid Walk Gait Cycle')
nexttile
% plot(t_hw,Pz_hw(1,:),t_hw+T_hw/4,Pz_hw(2,:),t_hw+2*T_hw/4,Pz_hw(3,:),t_hw+3*T_hw/4,Pz_hw(4,:))
plot(t_hw,Pz_hw(1,:),t_hw,Pz_hw(2,:),t_hw,Pz_hw(3,:),t_hw,Pz_hw(4,:))
title('Z Position in Hybrid Walk')
legend('Hind Right','Front Right','Hind Left','Front Left')
% yline(H);


nexttile
plot(t_hw,Px_hw(1,:),t_hw,Px_hw(2,:),t_hw,Px_hw(3,:),t_hw,Px_hw(4,:))
ylim([-6, 12])
title('X Position in Hybrid Walk')
legend('Hind Right','Front Right','Hind Left','Front Left')
% yline(L_hw);
% yline(1.5*L_hw);
% yline(0);
% yline(-L_hw);
% yline(-1.5*L_hw);


%% Trotting Gait
beta_t = 0.5;
T_t = 1;
vT = 2 * vHW;
adjustment = 0.8;
vT_h = vT * adjustment;
L_tf = vT * beta_t * T_t/2;
L_th = adjustment * L_tf;
a_x_tf = [-L_tf, -4*(L_tf/T_t), 80*(L_tf/T_t^2), -320*(L_tf/T_t^3), 640*(L_tf/T_t^4), -512*(L_tf/T_t^5)]';
a_x_th = [-L_th, -4*(L_th/T_t), 80*(L_th/T_t^2), -320*(L_th/T_t^3), 640*(L_th/T_t^4), -512*(L_th/T_t^5)]';
a_z_t = [0, 0, 64*(H/T_t^2), -256*(H/T_t^3), 256*(H/T_t^4), 0]';
iter = 8;
t_t = linspace(t_hw(end) + 1, t_hw(end) + T_t*iter, ((t_hw(end) + T_t*iter) - (t_hw(end) + 1)) * 100);

for i=1:size(t_t,2)
    ts_arg_t(1,i) = mod(t_t(i),T_t);
    ts_arg_t(2,i) = mod(t_t(i), T_t);
    ts_arg_t(3,i) = mod(t_t(i), T_t);
    ts_arg_t(4,i) = mod(t_t(i), T_t);
end

% arb_param = 0.33;

for i = 1:4
    if i==2 || i==4
        for j=1:size(ts_arg_t,2)
            t_s(i,j,:) = [1, ts_arg_t(i,j), ts_arg_t(i,j)^2, ts_arg_t(i,j)^3, ts_arg_t(i,j)^4, ts_arg_t(i,j)^5]';
            ts_vec = squeeze(t_s(i,j,:));
            if ts_arg_t(i,j) < beta_t*T_t
                Px_t(i,j) = a_x_tf' * ts_vec;
                Pz_t(i,j) = a_z_t' * ts_vec;
            else
                Px_t(i,j) = -vT * (ts_arg_t(i,j) - beta_t*T_t) + L_tf;
                Pz_t(i,j) = beta_t * H * 0.5 * (1 - cos(2*pi * (ts_arg_t(i,j) - beta_t*T_t)/(beta_t*T_t)));
            end
            Py_t(i,j) = 0;
        end
    else
        for j=1:size(ts_arg_t,2)
            t_s(i,j,:) = [1, ts_arg_t(i,j), ts_arg_t(i,j)^2, ts_arg_t(i,j)^3, ts_arg_t(i,j)^4, ts_arg_t(i,j)^5]';
            ts_vec = squeeze(t_s(i,j,:));
            if ts_arg_t(i,j) < beta_t*T_t
                Px_t(i,j) = a_x_th' * ts_vec;
                Pz_t(i,j) = a_z_t' * ts_vec;
            else
                Px_t(i,j) = -vT_h * (ts_arg_t(i,j) - beta_t*T_t) + L_th;
                Pz_t(i,j) = beta_t * H * 0.5 * (1 - cos(2*pi * (ts_arg_t(i,j) - beta_t*T_t)/(beta_t*T_t)));
            end
            Py_t(i,j) = 0;
        end
end
end

trotGait = figure();
trotGait.Name = "Trot Gait";
tiledlayout("flow")

nexttile
plot(Px_t(1,:),Pz_t(1,:))
title('Hind Gait Cycle in XZ Plane (Stationary)')
hold on;
yline(H);
yline(0);
xline(-L_th);
xline(L_th);
axis padded

nexttile
plot(Px_t(2,:),Pz_t(2,:));
title('Front Gait Cycle in XZ Plane (Stationary)')
hold on;
yline(H);
yline(0);
xline(-L_tf);
xline(L_tf);
axis padded

nexttile
plot3(t_t,Px_t(1,:),Pz_t(1,:),t_t,Px_t(3,:),Pz_t(3,:));
title('Hind Gait Position in XZ Direction')
xlim([t_t(1),t_t(1)+T_t*2])
xlabel('Time, t (s)')
ylabel('X Position, x (cm)')
zlabel('Z Position, z (cm)')
grid on;

nexttile
plot3(t_t+T_t/2,Px_t(2,:),Pz_t(2,:),t_t+T_t/2,Px_t(4,:),Pz_t(4,:));
title('Front Gait Position in XZ Direction')
xlim([t_t(1),t_t(1)+T_t*2])
xlabel('Time, t (s)')
ylabel('X Position, x (cm)')
zlabel('Z Position, z (cm)')
grid on;

Px_t = [Px_t(1,51:end); Px_t(2,1:size(Px_t,2)-50); Px_t(3,1:size(Px_t,2)-50); Px_t(4,51:end)];
Pz_t = [Pz_t(1,51:end); Pz_t(2,1:size(Pz_t,2)-50); Pz_t(3,1:size(Pz_t,2)-50); Pz_t(4,51:end)];
t_t = t_t(1:size(t_t,2)-50);


% %% Gait Transition
% % HR FR HL FL
% 
% t_GT = linspace(0, 1.5, 150);
% t_GT_cut = linspace(0,1.0,100);
% 
% FL_pos = linspace(Px_hw(4,end),L_tf, 100);
% 
% acc = -(vT - vHW)/1.0;
% vel = acc * t_GT - vHW;
% FR_pos = Px_hw(2,end) + acc/2 * t_GT(1:100).^2 - vHW*t_GT(1:100);
% HL_pos = Px_hw(3,end) + acc/2 * t_GT(1:100).^2 - vHW*t_GT(1:100);
% 
% HR_acc = -(vT - vHW)/0.5;
% HR_vel = HR_acc * t_GT(1:50) - vHW;
% % HR_pos = -0.5*L_hw + HR_acc/2 * t_GT(1:50).^2 + vHW*t_GT(1:50);
% HR_pos = linspace(Px_hw(1,end), 0.5 * L_hw, 50);
% 
% HR_pos = cat(2,HR_pos,linspace(0.5 * L_hw, L_th, 50));
% % HR_pos = cat(2,HR_pos, ones(1,50)*HR_pos(end));
% 
% FR_pos = cat(2, FR_pos, linspace(FR_pos(end), L_tf, 50));
% HL_pos = cat(2, HL_pos, linspace(HL_pos(end), L_th, 50));
% t_GT = t_GT + t_hw(end);
% t_GT_cut = t_GT_cut + t_hw(end);
% 
% % Px_gt = [HR_pos; FR_pos; HL_pos; FL_pos];
% 
% Pz_gt = [linspace(Pz_hw(1,end),Pz_t(1,1),150);
%     linspace(Pz_hw(2,end),Pz_t(2,50),150);
%     linspace(Pz_hw(3,end),Pz_t(3,50),150);
%     linspace(Pz_hw(4,end),Pz_t(4,1),150);];
% 
% % for i=1:size(t_GT,2)
% %     ts_arg_gt(1,i) = mod(t_GT(i), T_t);
% %     ts_arg_gt(2,i) = mod(t_GT(i) + T_t/2, T_t);
% %     ts_arg_gt(3,i) = mod(t_GT(i) + T_t/2, T_t);
% %     ts_arg_gt(4,i) = mod(t_GT(i), T_t);
% % end
% % 
% % for i=1:4
% %     for j=1:size(ts_arg_gt,2)
% %         t_s(i,j,:) = [1, ts_arg_gt(i,j), ts_arg_gt(i,j)^2, ts_arg_gt(i,j)^3, ts_arg_gt(i,j)^4, ts_arg_gt(i,j)^5]';
% %         ts_vec = squeeze(t_s(i,j,:));
% %         if ts_arg_gt(i,j) > beta_t*T_t
% %             Pz_gt(i,j) = (a_z_hw' * ts_vec) +  a_z_t' * ts_vec;
% %         else
% %             Pz_gt(i,j) = a_z_t' * ts_vec * beta_t * H * 0.5 * (1 - cos(2*pi * (ts_arg_gt(i,j) - beta_t*T_t)/(beta_t*T_t)));
% %         end
% %         Py_t(i,j) = -w*(1-(t_GT(i)/t_GT(end))) * sin(2*pi*(ts_arg_gt(i,j) - T_t)/T_t);
% %     end
% % end
% 
% figure
% tiledlayout("flow")
% title("Gait Transition")
% 
% nexttile
% plot(t_GT(1:100),HR_pos,t_GT,FR_pos,t_GT,HL_pos,t_GT(1:100),FL_pos);
% legend('Hind Right','Front Right','Hind Left','Front Left')
% title('X Position in Gait Transition')
% 
% nexttile
% plot(HR_pos,Pz_gt(1,1:100),FR_pos,Pz_gt(2,:),HL_pos,Pz_gt(3,:),FL_pos,Pz_gt(4,1:100));
% 
% 
% % t 0.0-0.5
% % FL pos -1.5* L_hw to L_tf
% % FR vHW to vT from 0.5*L_hw
% % HL vHW to vT from 1.5*L_hw
% % HR vHW to vT from -0.5*L_hw
% % 
% % t 0.5-1.0
% % FL pos -1.5* L_hw to L_tf
% % FR vHW to vT from 0.5*L_hw
% % HL vHW to vT from 1.5*L_hw
% % HR pos 0.5*L_hw to L_th
% % 
% % t 1.0-1.5
% % FL move and support
% % FR pos to L_tf
% % HL pos to L_th
% % HR move and support

%% Gait Transition 2

vHWx = 10/3;
vTx = 20;

t_1 = linspace(0, 1, 100);
t_05 = linspace(0, 0.5, 50);

FL_bx = [Px_hw(4,end) L_tf -vHWx -vTx 0 0]'; % 1s
FR_bx = [Px_hw(2,end) -L_tf -vHWx 0 0 0]'; % 1s
HL_bx = [Px_hw(3,end) -L_th 0 0 0 0]'; % 1s
HR_bx = [Px_hw(1,end) 0.5*L_hw -vHWx vTx 0 0]'; % 0.5s %perhaps (-)vTx here and start 293

FL_ax = quintic_solve(FL_bx, 1);
FR_ax = quintic_solve(FR_bx, 1);
HL_ax = quintic_solve(HL_bx, 1);
HR_ax = quintic_solve(HR_bx, 0.5);


for i=1:size(t_1,2)
    t_num = t_1(i);
    t_vec = [1 t_num t_num^2 t_num^3 t_num^4 t_num^5]';
    Px_gt_FL(i) = FL_ax * t_vec;
    Px_gt_FR(i) = FR_ax * t_vec;
    Px_gt_HL(i) = HL_ax * t_vec;
end
for i=1:size(t_05,2)
    t_num = t_05(i);
    t_vec = [1 t_num t_num^2 t_num^3 t_num^4 t_num^5]';
    Px_gt_HR(i) = HR_ax * t_vec;
end
HR_bx = [0.5*L_hw L_th vTx -vTx 0 0]';
HR_ax = quintic_solve(HR_bx,0.5);
for i=1:size(t_05,2)
    t_num = t_05(i);
    t_vec = [1 t_num t_num^2 t_num^3 t_num^4 t_num^5]';
    Px_gt_HR = cat(2,Px_gt_HR, (HR_ax * t_vec));
end
T_gt = 4*1;
FL_az = [0, 0, 256*H/T_gt^2, -2048*H/T_gt^3, 4096*H/T_gt^4, 0];
for i=1:size(t_1,2)
    t_num = t_1(i);
    t_vec = [1 t_num t_num^2 t_num^3 t_num^4 t_num^5]';
    Pz_gt_FL(i) = FL_az * t_vec;
    Pz_gt_FR(i) = 0;
    Pz_gt_HL(i) = 0;
end
Pz_gt_HR = zeros(1,size(t_05,2));
T_gt = 4*0.5;
HR_az = [0, 0, 256*H/T_gt^2, -2048*H/T_gt^3, 4096*H/T_gt^4, 0];
for i=1:size(t_05,2)
    t_num = t_1(i);
    t_vec = [1 t_num t_num^2 t_num^3 t_num^4 t_num^5]';
    Pz_gt_HR = cat(2,Pz_gt_HR, (HR_az * t_vec));
end


figure()
title("Gait Transition 2")
plot(t_1,Px_gt_HR,t_1,Px_gt_FR,t_1,Px_gt_HL,t_1,Px_gt_FL)
legend('Hind Right','Front Right','Hind Left','Front Left')

Px_gt(1,:) = Px_gt_HR;
Px_gt(2,:) = Px_gt_FR;
Px_gt(3,:) = Px_gt_HL;
Px_gt(4,:) = Px_gt_FL;

Pz_gt(1,:) = Pz_gt_HR;
Pz_gt(2,:) = Pz_gt_FR;
Pz_gt(3,:) = Pz_gt_HL;
Pz_gt(4,:) = Pz_gt_FL;


%% Total Motion Plots
% HR FR HL FL

Px = Px_hw;

Px1 = cat(2,Px(1,:),Px_gt(1,:));
Px2 = cat(2,Px(2,:),Px_gt(2,:));
Px3 = cat(2,Px(3,:),Px_gt(3,:));
Px4 = cat(2,Px(4,:),Px_gt(4,:));

Px1 = cat(2,Px1,Px_t(1,:));
Px2 = cat(2,Px2,Px_t(2,:));
Px3 = cat(2,Px3,Px_t(3,:));
Px4 = cat(2,Px4,Px_t(4,:));

Pz = Pz_hw;

% Pz_gt = zeros(4,100);

Pz1 = cat(2,Pz(1,:),Pz_gt(1,:));
Pz2 = cat(2,Pz(2,:),Pz_gt(2,:));
Pz3 = cat(2,Pz(3,:),Pz_gt(3,:));
Pz4 = cat(2,Pz(4,:),Pz_gt(4,:));

Pz1 = cat(2,Pz1,Pz_t(1,:));
Pz2 = cat(2,Pz2,Pz_t(2,:));
Pz3 = cat(2,Pz3,Pz_t(3,:));
Pz4 = cat(2,Pz4,Pz_t(4,:));

t_gt = t_1 + t_hw(end);
t = cat(2,t_hw,t_gt);
t = cat(2,t,t_t);
% t = cat(2,t_hw,t_GT_cut);
% t = cat(2,t,t_t);

totalPlotX = figure();
totalPlotX.Name = 'Total Motion Plots in X';
tiledlayout("vertical")
nexttile
plot(t,Px1,t,Px2,t,Px3,t,Px4);
hold on;
% plot(t_t,Px_t(1,:),t_t,Px_t(2,:))
title("X Axis Motion");
xlabel("Time, t (s)")
ylabel("X Position, x (cm)")
legend('Hind Right','Front Right','Hind Left','Front Left')
nexttile
plot(t,Px1,t,Px2,t,Px3,t,Px4)
hold on;
% plot(t_t(1:250),Px_t(1,51:end),t_t(1:250),Px_t(2,1:250),t_t(1:250),Px_t(3,1:250),t_t(1:250),Px_t(4,51:end));

title("X Axis Motion Cut");
xlabel("Time, t (s)")
ylabel("X Position, x (cm)")
% legend('Hind Right','Front Right','Hind Left','Front Left')
xline(t_hw(end));
xline(t_hw(end)+1);
xline(t_hw(end)+1.5);
xlim([t_hw(end)-1,t_hw(end)+3])

totalPlotZ = figure();
totalPlotZ.Name = 'Total Motion Plots in Z';
tiledlayout("vertical")
nexttile
plot(t,Pz1,t,Pz2,t,Pz3,t,Pz4);
hold on;
% plot(t_t,Px_t(1,:),t_t,Px_t(2,:))
title("Z Axis Motion");
xlabel("Time, t (s)")
ylabel("X Position, x (cm)")
legend('Hind Right','Front Right','Hind Left','Front Left')
nexttile
plot(t,Pz1,t,Pz2,t,Pz3,t,Pz4)
hold on;
% plot(t_t(1:250),Px_t(1,51:end),t_t(1:250),Px_t(2,1:250),t_t(1:250),Px_t(3,1:250),t_t(1:250),Px_t(4,51:end));

title("Z Axis Motion Cut");
xlabel("Time, t (s)")
ylabel("X Position, x (cm)")
% legend('Hind Right','Front Right','Hind Left','Front Left')
xline(t_hw(end));
xline(t_hw(end)+1);
xline(t_hw(end)+1.5);
xlim([t_hw(end)-1,t_hw(end)+3])

totalPlotTime = figure();
totalPlotTime.Name = 'Total Motion Plots in Time';
tiledlayout("flow")
nexttile
plot3(Px1,t,Pz1);
title("Hind Right Motion")
xlabel("X Position, x (cm)")
ylabel("Time, t (s)")
zlabel("Z Position, z (cm)")
ylim([t_hw(end)-1,t_hw(end)+3])
grid on;
nexttile
plot3(Px2,t,Pz2);
title("Front Right Motion")
xlabel("X Position, x (cm)")
ylabel("Time, t (s)")
zlabel("Z Position, z (cm)")
ylim([t_hw(end)-1,t_hw(end)+3])

grid on;
nexttile
plot3(Px3,t,Pz3);
title("Hind Left Motion")
xlabel("X Position, x (cm)")
ylabel("Time, t (s)")
zlabel("Z Position, z (cm)")
ylim([t_hw(end)-1,t_hw(end)+3])

grid on;
nexttile
plot3(Px4,t,Pz4);
title("Front Left Motion")
xlabel("X Position, x (cm)")
ylabel("Time, t (s)")
zlabel("Z Position, z (cm)")
ylim([t_hw(end)-1,t_hw(end)+3])

grid on;

%% functions
function vec = quintic_solve(b, T)
	vec(1) = b(1);
	vec(2) = b(3);
	vec(3) = 0.5 * b(5);
	vec(4) =(1/(2*T^3)) * (20 * (b(2) - b(1)) - (8 * b(4)+ 12*b(3) )*T - (3 * b(6) - b(5) )*T^2 );
	vec(5) =(1/(2*T^4)) * (30 * (b(1) - b(2)) + (14 * b(4)+ 16*b(3) )*T + (3 * b(6) - 2*b(5) )*T^2 );
	vec(6) =(1/(2*T^5)) * (12 * (b(2) - b(1)) - 6*(b(4)+ b(3) )*T - (b(6) - b(5) )*T^2 );
end