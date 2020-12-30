% Script for plotting ground truth/estimated state histories and inertial
% parameters
clear
close all

%% Ground Truth
% 
% chaser_truth_pose = readmatrix('chaser_gt_pose_stat_good_DEC10.csv');
% chaser_truth_twist = readmatrix('chaser_gt_twist_stat_good_DEC10.csv');
% chaser_est_pose = readmatrix('chaser_est_pose_stat_good_DEC10.csv');
% chaser_est_twist = readmatrix('chaser_est_twist_stat_good_DEC10.csv');
% target_est_pose = readmatrix('target_est_pose_stat_good_DEC10.csv');
% target_est_twist = readmatrix('target_est_twist_stat_good_DEC10.csv');

chaser_truth_pose = readmatrix('chaser_gt_pose_moving_good_DEC10.csv');
chaser_truth_twist = readmatrix('chaser_gt_twist_moving_good_DEC10.csv');
chaser_est_pose = readmatrix('chaser_est_pose_moving_good_DEC10.csv');
chaser_est_twist = readmatrix('chaser_est_twist_moving_good_DEC10.csv');
target_est_pose = readmatrix('target_est_pose_moving_good_DEC10.csv');
target_est_twist = readmatrix('target_est_twist_moving_good_DEC10.csv');

% delta_pose = readmatrix('chaser_deltapose_stat_good_DEC10.csv');
delta_pose = readmatrix('chaser_deltapose_moving_good_DEC10.csv');
%delta_pose = readmatrix('delta_pose_DEC3_insp.csv');
delta_pose_t = delta_pose(:,1)./1e9;
delta_pose_t = delta_pose_t - delta_pose_t(1);
delta_pos = delta_pose(:,5:7);
delta_att = delta_pose(:,8:11);
figure(30);
plot(delta_pose_t(2:end), delta_pos(2:end,1), 'r-', ...
     delta_pose_t(2:end), delta_pos(2:end,2), 'g-',...
     delta_pose_t(2:end), delta_pos(2:end,3), 'b-');
title('PCD odometry translation');
xlabel('t (s)');
ylabel('tranlsation (m)');
legend('x', 'y', 'z');
grid on;

figure(31);
plot(delta_pose_t(2:end), delta_att(2:end,1), 'r-', ...
     delta_pose_t(2:end), delta_att(2:end,2), 'g-',...
     delta_pose_t(2:end), delta_att(2:end,3), 'b-',...
     delta_pose_t(2:end), delta_att(2:end,4), 'k-');
title('PCD odometry rotation');
xlabel('t (s)');
ylabel('rotation (quat)');
legend('x', 'y', 'z', 'w');
grid on;

aa = zeros(length(delta_att(:,1)), 4);
for i = 2:length(delta_att(:,1))
    aa(i,:) = q2aa(delta_att(i,1:4));
end

figure(41);
plot(delta_pose_t(2:end), aa(2:end,4), 'r-');
title('PCD odometry rotation');
xlabel('t (s)');
ylabel('rotation (angle-axis)');
legend('alpha');
grid on;


set(0,'defaultAxesFontSize',15)
 set(0, 'DefaultLineLineWidth', 1);
  set(0, 'DefaultAxesLineWidth', 1);

chaser_est_pose_t = chaser_est_pose(:,1)./1e9;
chaser_est_pose_t = chaser_est_pose_t - chaser_est_pose_t(1);
chaser_est_pos = chaser_est_pose(:,5:7);
chaser_est_att = chaser_est_pose(:,8:11);

chaser_est_twist_t = chaser_est_twist(:,1)./1e9;
chaser_est_twist_t = chaser_est_twist_t - chaser_est_twist_t(1);
chaser_est_vel = chaser_est_twist(:,5:7);
chaser_est_w = chaser_est_twist(:,8:10);
%chaser_est_vel = chaser_est_twist(1:2:(end-1),5:7);
%chaser_est_w = chaser_est_twist(1:2:(end-1),8:10);
for i = 1:length(chaser_est_w(:,1))
    if (isnan(chaser_est_w(i,1)))
        chaser_est_w(i,:) = [0.0, 0.0, 0.0];
    end
end

target_est_pose_t = target_est_pose(:,1)./1e9;
target_est_pose_t = target_est_pose_t - target_est_pose_t(1);
target_est_pos = target_est_pose(:,5:7);
target_est_att = target_est_pose(:,8:11);

target_est_twist_t = target_est_twist(:,1)./1e9;
target_est_twist_t = target_est_twist_t - target_est_twist_t(1);
target_est_vel = target_est_twist(:,5:7);
target_est_w = target_est_twist(:,8:10);

%chaser_imu = readmatrix('chaser_imu_stat_good_DEC10.csv');
% chaser_imu = readmatrix('chaser_teleop_imu.csv');
chaser_imu = readmatrix('chaser_imu_moving_good_DEC10.csv');
chaser_imu_t = chaser_imu(:,1)./1e9;
chaser_imu_t = chaser_imu_t - chaser_imu_t(1);
chaser_imu_omega = chaser_imu(:,18:20);
chaser_imu_accel = chaser_imu(:,30:32);

figure(20);
plot(chaser_imu_t, chaser_imu_accel(:,1), 'r-', ...
     chaser_imu_t, chaser_imu_accel(:,2), 'g-',...
     chaser_imu_t, chaser_imu_accel(:,3), 'b-');
%plot(chaser_imu_t, chaser_imu_accel(:,2), 'g-');

xlabel('Time (s)');
ylabel('Accel (m/s^2)');
title('Chaser IMU');
grid on;

figure(21);
plot(chaser_imu_t, chaser_imu_omega(:,1), 'r-', ...
     chaser_imu_t, chaser_imu_omega(:,2), 'g-',...
     chaser_imu_t, chaser_imu_omega(:,3), 'b-');
xlabel('Time (s)');
ylabel('Omega (rad/s)');
title('Chaser IMU');
grid on;

chaser_gt_pose_t = chaser_truth_pose(:,1)./1e9;
chaser_gt_pose_t = chaser_gt_pose_t - chaser_gt_pose_t(1);
chaser_gt_pos = chaser_truth_pose(:,5:7);
chaser_gt_pos = chaser_gt_pos - [10.9, -6.65, 4.9];
chaser_gt_att = chaser_truth_pose(:,8:11);

chaser_gt_twist_t = chaser_truth_twist(:,1)./1e9;
chaser_gt_twist_t = chaser_gt_twist_t - chaser_gt_twist_t(1);
chaser_gt_vel = chaser_truth_twist(:,5:7);
chaser_gt_w = chaser_truth_twist(:,8:10);
% body frame required
chaser_gt_w_body = zeros(size(chaser_gt_w));
for i = 1:length(chaser_gt_twist_t)
    R = q2dcm(chaser_gt_att(i,:)');
    %chaser_gt_w_body(i,:) = (R * chaser_gt_w(i,:)')';
    chaser_gt_w_body(i,:) = chaser_gt_w(i,:);
end

% target_truth_pose = readmatrix('target_gt_pose_stat_good_DEC10.csv');
% target_truth_twist = readmatrix('target_gt_twist_stat_good_DEC10.csv');

target_truth_pose = readmatrix('target_gt_pose_moving_good_DEC10.csv');
target_truth_twist = readmatrix('target_gt_twist_moving_good_DEC10.csv');

target_gt_pose_t = target_truth_pose(:,1)./1e9;
target_gt_pose_t = target_gt_pose_t - target_gt_pose_t(1);
target_gt_pos = target_truth_pose(:,5:7);
target_gt_pos = target_gt_pos - [10.9, -6.65, 4.9];
target_gt_att = target_truth_pose(:,8:11);

target_gt_twist_t = target_truth_twist(:,1)./1e9;
target_gt_twist_t = target_gt_twist_t - target_gt_twist_t(1);
target_gt_vel = target_truth_twist(:,5:7);
target_gt_w = target_truth_twist(:,8:10);
% body frame required
target_gt_w_body = zeros(size(target_gt_w));
for i = 1:length(target_gt_pose_t)
    R = q2dcm(target_gt_att(i,:)');
    target_gt_w_body(i,:) = (R * target_gt_w(i,:)')';
end

% Chaser position (ISS)
figure(1);
subplot(2,2,1);
plot(chaser_est_pose_t, chaser_est_pos(:,1), 'r-', ...
     chaser_est_pose_t, chaser_est_pos(:,2), 'g-',...
     chaser_est_pose_t, chaser_est_pos(:,3), 'b-');
 hold on;
plot(chaser_gt_pose_t, chaser_gt_pos(:,1), 'r--', ...
     chaser_gt_pose_t, chaser_gt_pos(:,2), 'g--',...
     chaser_gt_pose_t, chaser_gt_pos(:,3), 'b--');
grid on;
%title('Chaser position (World frame)');
%xlabel('t (s)');
ylabel('position (m)');
%legend('x', 'y', 'z', 'x_{gt}', 'y_{gt}', 'z_{gt}');

% Chaser velocity (ISS)
subplot(2,2,2);
plot(chaser_est_twist_t, chaser_est_vel(:,1), 'r-', ...
     chaser_est_twist_t, chaser_est_vel(:,2), 'g-',...
     chaser_est_twist_t, chaser_est_vel(:,3), 'b-');
 hold on;
plot(chaser_gt_twist_t, chaser_gt_vel(:,1), 'r--', ...
     chaser_gt_twist_t, chaser_gt_vel(:,2), 'g--',...
     chaser_gt_twist_t, chaser_gt_vel(:,3), 'b--');
grid on;
%title('Chaser velocity (World frame)');
%xlabel('t (s)');
ylabel('velocity (m/s)');
%ylim([-0.25, 0.25]);
%legend('x', 'y', 'z', 'x_{gt}', 'y_{gt}', 'z_{gt}');

% Chaser attitude (ISS)
subplot(2,2,3);
plot(chaser_est_pose_t, chaser_est_att(:,1), 'r-', ...
     chaser_est_pose_t, chaser_est_att(:,2), 'g-',...
     chaser_est_pose_t, chaser_est_att(:,3), 'b-',...
     chaser_est_pose_t, chaser_est_att(:,4), 'k-');
 hold on;
plot(chaser_gt_pose_t, chaser_gt_att(:,1), 'r--', ...
     chaser_gt_pose_t, chaser_gt_att(:,2), 'g--',...
     chaser_gt_pose_t, chaser_gt_att(:,3), 'b--', ...
     chaser_gt_pose_t, chaser_gt_att(:,4), 'k--');
grid on;
%title('Chaser attitude (w/r to World frame)');
xlabel('t (s)');
ylabel('attitude (quat)');
legend('x', 'y', 'z', 'w', 'x_{gt}', 'y_{gt}', 'z_{gt}', 'w_{gt}');

% Chaser angular velocity (ISS)
subplot(2,2,4);
plot(chaser_est_twist_t, -chaser_est_w(:,1), 'r-', ...
     chaser_est_twist_t, -chaser_est_w(:,2), 'g-',...
     chaser_est_twist_t, -chaser_est_w(:,3), 'b-');
 hold on;
plot(chaser_gt_twist_t, chaser_gt_w_body(:,1), 'r--', ...
     chaser_gt_twist_t, chaser_gt_w_body(:,2), 'g--',...
     chaser_gt_twist_t, chaser_gt_w_body(:,3), 'b--');
grid on;
%title('Chaser angular velocity (body frame)');
xlabel('t (s)');
ylabel('omega (rad/s)');
ylim([-0.04, 0.04]);
%legend('x', 'y', 'z', 'x_{gt}', 'y_{gt}', 'z_{gt}');

set(gcf, 'OuterPosition', [160, 3, 980, 980]);

% Target position (ISS)
figure(5);
subplot(2,2,1);
plot(target_est_pose_t, target_est_pos(:,1), 'r-', ...
     target_est_pose_t, target_est_pos(:,2), 'g-',...
     target_est_pose_t, target_est_pos(:,3), 'b-');
 hold on;
plot(target_gt_pose_t, target_gt_pos(:,1), 'r--', ...
     target_gt_pose_t, target_gt_pos(:,2), 'g--',...
     target_gt_pose_t, target_gt_pos(:,3), 'b--');
grid on;
%title('Target position (World frame)');
%xlabel('t (s)');
ylabel('position (m)');
%legend('x', 'y', 'z', 'x_{gt}', 'y_{gt}', 'z_{gt}');


% Target velocity (ISS)
%figure(6);
subplot(2,2,2);
plot(target_est_twist_t, target_est_vel(:,1), 'r-', ...
     target_est_twist_t, target_est_vel(:,2), 'g-',...
     target_est_twist_t, target_est_vel(:,3), 'b-');
 hold on;
plot(target_gt_twist_t, target_gt_vel(:,1), 'r--', ...
     target_gt_twist_t, target_gt_vel(:,2), 'g--',...
     target_gt_twist_t, target_gt_vel(:,3), 'b--');
grid on;
%title('Target velocity (World frame)');
%xlabel('t (s)');
ylabel('velocity (m/s)');
ylim([-0.025, 0.025]);
%legend('x', 'y', 'z', 'x_{gt}', 'y_{gt}', 'z_{gt}');

% est_w = target_est_w(51,:)'./norm(target_est_w(51,:));
% gt_w = target_gt_w_body(3133,:)'./norm(target_gt_w_body(3133,:));
% GG = [dot(est_w, gt_w) -norm(cross(est_w, gt_w)) 0;
%       norm(cross(est_w, gt_w)) dot(est_w, gt_w) 0;
%       0 0 1];
% FFi = [est_w (gt_w-dot(est_w,gt_w)*est_w)/norm(gt_w-dot(est_w,gt_w)*est_w) cross(gt_w,est_w)];
% UU = FFi*GG*inv(FFi);

target_gt_twist_t_interp = target_gt_twist_t(1:63:end);
target_gt_w_body_interp = target_gt_w_body(1:63:end,:);
target_gt_pose_t_interp = target_gt_pose_t(1:63:end);
target_gt_att_interp = target_gt_att(1:63:end,:);

% quat difference 
q_twist_diff = zeros(length(target_gt_twist_t_interp),4);
q_att_diff = zeros(length(target_gt_pose_t_interp),4);
quat_twist_diff_obj_array = quaternion();
quat_att_diff_obj_array = quaternion();
for i = 1:length(target_gt_twist_t_interp)
    vec = cross(target_gt_w_body_interp(i,:), target_est_w(i,:));
    quat = [vec(1), vec(2), vec(3), sqrt((norm(target_gt_w_body_interp(i,:))^2) * (norm(target_est_w(i,:))^2)) + dot(target_gt_w_body_interp(i,:), target_est_w(i,:))];
    quat = quat./norm(quat);
    quat_twist_diff_obj_array = [quat_twist_diff_obj_array, quaternion(quat(1), quat(2), quat(3), quat(4))];
    q_twist_diff(i,:) = quat;
    q_att_diff(i,:) = qmult(target_gt_att_interp(i,:), [-target_est_att(i,1), -target_est_att(i,2), -target_est_att(i,3), target_est_att(i,4)]);
    %quat_twist_diff_obj_array = [quat_twist_diff_obj_array, quaternion(q_twist_diff(i,1), q_twist_diff(i,2), q_twist_diff(i,3), q_twist_diff(i,4))];
    quat_att_diff_obj_array = [quat_att_diff_obj_array, quaternion(q_att_diff(i,1), q_att_diff(i,2), q_att_diff(i,3), q_att_diff(i,4))];
end

for i = 2:length(q_att_diff(:,1))
    if (norm(q_att_diff(i,:) - q_att_diff(i-1,:)) > norm(-q_att_diff(i,:) - q_att_diff(i-1,:)))
        q_att_diff(i,:) = -q_att_diff(i,:);
        quat_att_diff_obj_array(i) = quaternion(q_att_diff(i,1), q_att_diff(i,2), q_att_diff(i,3), q_att_diff(i,4));
    end
end

mean_q_twist = meanrot(quat_twist_diff_obj_array);
[q_offset_x, q_offset_y, q_offset_z, q_offset_w] = parts(mean_q_twist);
q_offset = [q_offset_x, q_offset_y, q_offset_z, q_offset_w];
R_offset = q2dcm(q_offset);


% R_offset = 
quat_att_diff_obj_array = quaternion();
for i = 1:length(target_est_att(:,1))
    R_est = q2dcm(target_est_att(i,:));
    R_gt = q2dcm(target_gt_att_interp(i,:));
    est_vec = R_est * [1 0 0]';
    gt_vec = R_gt * [1 0 0]';
    GG = [dot(est_vec, gt_vec) -norm(cross(est_vec, gt_vec)) 0;
          norm(cross(est_vec, gt_vec)) dot(est_vec, gt_vec) 0;
          0 0 1];
    FFi = [est_vec (gt_vec-dot(est_vec,gt_vec)*est_vec)/norm(gt_vec-dot(est_vec,gt_vec)*est_vec) cross(gt_vec,est_vec)];
    UU = FFi*GG*inv(FFi);
    norm(UU)
    norm(gt_vec - UU*est_vec)
    R_att_offset = UU;
    q_att_offset = dcm2q(R_att_offset);
    quat_att_diff_obj_array = [quat_att_diff_obj_array, quaternion(q_att_offset(1), q_att_offset(2), q_att_offset(3), q_att_offset(4))];
end
mean_q_att = meanrot(quat_att_diff_obj_array);
[q_offset_x, q_offset_y, q_offset_z, q_offset_w] = parts(mean_q_att);
q_att_offset = [q_offset_x, q_offset_y, q_offset_z, q_offset_w];
R_att_offset = q2dcm(q_att_offset);



% R_est = q2dcm(target_est_att(112,:));
% R_gt = q2dcm(target_gt_att(6939,:));
% est_vec = R_est * [1 0 0]';
% gt_vec = R_gt * [1 0 0]';
% GG = [dot(est_vec, gt_vec) -norm(cross(est_vec, gt_vec)) 0;
%       norm(cross(est_vec, gt_vec)) dot(est_vec, gt_vec) 0;
%       0 0 1];
% FFi = [est_vec (gt_vec-dot(est_vec,gt_vec)*est_vec)/norm(gt_vec-dot(est_vec,gt_vec)*est_vec) cross(gt_vec,est_vec)];
% UU = FFi*GG*inv(FFi);
% norm(UU)
% norm(gt_vec - UU*est_vec)
% R_att_offset = UU;
% q_att_offset = dcm2q(R_att_offset);

target_est_w_unrotated = target_est_w;
target_est_att_unrotated = target_est_att;

for i = 1:length(target_est_att(:,1))
    R = q2dcm(target_est_att(i,:));
    Rnew = R'*R_att_offset;
    qnew = dcm2q(Rnew);
    %qnew = qmult(target_est_att(i,:), q_att_offset);
    omega_new = (R_offset * target_est_w(i,:)')';
    target_est_att(i,:) = qnew;
    target_est_w(i,:) = omega_new;
end

for i = 2:length(target_est_att(:,1))
    if (norm(target_est_att(i,:) - target_est_att(i-1,:)) > norm(-target_est_att(i,:) - target_est_att(i-1,:)))
        target_est_att(i,:) = -target_est_att(i,:);
    end
end
subplot(2,2,3);
plot(target_est_pose_t, target_est_att(:,1), 'r-', ...
     target_est_pose_t, target_est_att(:,2), 'g-',...
     target_est_pose_t, target_est_att(:,3), 'b-',...
     target_est_pose_t, target_est_att(:,4), 'k-');
 hold on;
plot(target_gt_pose_t, target_gt_att(:,1), 'r--', ...
     target_gt_pose_t, target_gt_att(:,2), 'g--',...
     target_gt_pose_t, target_gt_att(:,3), 'b--', ...
     target_gt_pose_t, target_gt_att(:,4), 'k--');
grid on;
%title('Target attitude (w/r to World frame)');
xlabel('t (s)');
ylabel('attitude (quat)');
legend('x', 'y', 'z', 'w', 'x_{gt}', 'y_{gt}', 'z_{gt}', 'w_{gt}');

% Target angular velocity (body)
subplot(2,2,4);
plot(target_est_twist_t, target_est_w(:,1), 'r-', ...
     target_est_twist_t, target_est_w(:,2), 'g-',...
     target_est_twist_t, target_est_w(:,3), 'b-');
 hold on;
plot(target_gt_twist_t, target_gt_w_body(:,1), 'r--', ...
     target_gt_twist_t, target_gt_w_body(:,2), 'g--',...
     target_gt_twist_t, target_gt_w_body(:,3), 'b--');
grid on;
%title('Target angular velocity (target body frame)');
xlabel('t (s)');
ylabel('omega (rad/s)');
%legend('x', 'y', 'z', 'x_{gt}', 'y_{gt}', 'z_{gt}');
set(gcf, 'OuterPosition', [160, 3, 980, 980]);


figure(50);
subplot(1,2,1);
plot(target_est_pose_t, target_est_att(:,1), 'r-', ...
     target_est_pose_t, target_est_att(:,2), 'g-',...
     target_est_pose_t, target_est_att(:,3), 'b-',...
     target_est_pose_t, target_est_att(:,4), 'k-');
 hold on;
plot(target_gt_pose_t, target_gt_att(:,1), 'r--', ...
     target_gt_pose_t, target_gt_att(:,2), 'g--',...
     target_gt_pose_t, target_gt_att(:,3), 'b--', ...
     target_gt_pose_t, target_gt_att(:,4), 'k--');
grid on;
%title('Target attitude (w/r to World frame)');
xlabel('t (s)');
ylabel('attitude (quat)');
legend('x', 'y', 'z', 'w', 'x_{gt}', 'y_{gt}', 'z_{gt}', 'w_{gt}');

% Target angular velocity (body)
subplot(1,2,2);
plot(target_est_twist_t, target_est_w(:,1), 'r-', ...
     target_est_twist_t, target_est_w(:,2), 'g-',...
     target_est_twist_t, target_est_w(:,3), 'b-');
 hold on;
plot(target_gt_twist_t, target_gt_w_body(:,1), 'r--', ...
     target_gt_twist_t, target_gt_w_body(:,2), 'g--',...
     target_gt_twist_t, target_gt_w_body(:,3), 'b--');
grid on;
%title('Target angular velocity (target body frame)');
xlabel('t (s)');
ylabel('omega (rad/s)');
%legend('x', 'y', 'z', 'x_{gt}', 'y_{gt}', 'z_{gt}');
set(gcf, 'OuterPosition', [160, 3, 980, 490]);


% Plot for paper

figure(60);
subplot(3,2,1);
plot(chaser_est_pose_t, chaser_est_pos(:,1), 'r-', ...
     chaser_est_pose_t, chaser_est_pos(:,2), 'g-',...
     chaser_est_pose_t, chaser_est_pos(:,3), 'b-');
 hold on;
plot(chaser_gt_pose_t, chaser_gt_pos(:,1), 'r--', ...
     chaser_gt_pose_t, chaser_gt_pos(:,2), 'g--',...
     chaser_gt_pose_t, chaser_gt_pos(:,3), 'b--');
grid on;
ylim([-2.25 0.75]);
title('Chaser position');
ylabel('(m)');

subplot(3,2,2);
plot(chaser_est_twist_t, chaser_est_vel(:,1), 'r-', ...
     chaser_est_twist_t, chaser_est_vel(:,2), 'g-',...
     chaser_est_twist_t, chaser_est_vel(:,3), 'b-');
 hold on;
plot(chaser_gt_twist_t, chaser_gt_vel(:,1), 'r--', ...
     chaser_gt_twist_t, chaser_gt_vel(:,2), 'g--',...
     chaser_gt_twist_t, chaser_gt_vel(:,3), 'b--');
grid on;
title('Chaser velocity');
ylabel('(m/s)');
ylim([-0.15, 0.15]);

% Chaser attitude (ISS)
subplot(3,2,3);
plot(chaser_est_pose_t, chaser_est_att(:,1), 'r-', ...
     chaser_est_pose_t, chaser_est_att(:,2), 'g-',...
     chaser_est_pose_t, chaser_est_att(:,3), 'b-',...
     chaser_est_pose_t, chaser_est_att(:,4), 'k-');
 hold on;
plot(chaser_gt_pose_t, chaser_gt_att(:,1), 'r--', ...
     chaser_gt_pose_t, chaser_gt_att(:,2), 'g--',...
     chaser_gt_pose_t, chaser_gt_att(:,3), 'b--', ...
     chaser_gt_pose_t, chaser_gt_att(:,4), 'k--');
grid on;
title('Chaser attitude');
ylabel('(quat)');
% legend('x', 'y', 'z', 'w', 'x_{gt}', 'y_{gt}', 'z_{gt}', 'w_{gt}');

% Chaser angular velocity (ISS)
subplot(3,2,4);
plot(chaser_est_twist_t, -rad2deg(chaser_est_w(:,1)), 'r-', ...
     chaser_est_twist_t, -rad2deg(chaser_est_w(:,2)), 'g-',...
     chaser_est_twist_t, -rad2deg(chaser_est_w(:,3)), 'b-');
 hold on;
plot(chaser_gt_twist_t, rad2deg(chaser_gt_w_body(:,1)), 'r--', ...
     chaser_gt_twist_t, rad2deg(chaser_gt_w_body(:,2)), 'g--',...
     chaser_gt_twist_t, rad2deg(chaser_gt_w_body(:,3)), 'b--');
grid on;
title('Chaser angular velocity');
%xlabel('t (s)');
ylabel('(deg/s)');
ylim([-2.5, 2.5]);
%legend('x', 'y', 'z', 'x_{gt}', 'y_{gt}', 'z_{gt}');

subplot(3,2,5);
plot(target_est_pose_t, target_est_att(:,1), 'r-', ...
     target_est_pose_t, target_est_att(:,2), 'g-',...
     target_est_pose_t, target_est_att(:,3), 'b-',...
     target_est_pose_t, target_est_att(:,4), 'k-');
 hold on;
plot(target_gt_pose_t, target_gt_att(:,1), 'r--', ...
     target_gt_pose_t, target_gt_att(:,2), 'g--',...
     target_gt_pose_t, target_gt_att(:,3), 'b--', ...
     target_gt_pose_t, target_gt_att(:,4), 'k--');
grid on;
title('Target attitude');
xlabel('t (s)');
ylabel('attitude (quat)');

% Target angular velocity (body)
subplot(3,2,6);
plot(target_est_twist_t, rad2deg(target_est_w(:,1)), 'r-', ...
     target_est_twist_t, rad2deg(target_est_w(:,2)), 'g-',...
     target_est_twist_t, rad2deg(target_est_w(:,3)), 'b-');
 hold on;
plot(target_gt_twist_t(1:8485), rad2deg(target_gt_w_body(1:8485,1)), 'r--', ...
     target_gt_twist_t(1:8485), rad2deg(target_gt_w_body(1:8485,2)), 'g--',...
     target_gt_twist_t(1:8485), rad2deg(target_gt_w_body(1:8485,3)), 'b--');
grid on;
title('Target angular velocity');
xlabel('t (s)');
ylabel('omega (deg/s)');
ylim([-7.5, 7.5]);
%legend('x', 'y', 'z', 'x_{gt}', 'y_{gt}', 'z_{gt}');
set(gcf, 'OuterPosition', [160, 3, 1000, 1500]);









%% Relative ground truth state
% t_gt_rel = target_gt_pose_t;
% c_gt_rel = chaser_gt_pose_t;
% gt_rel_pos = zeros(length(t_gt_rel), 3);
% gt_rel_vel = zeros(length(t_gt_rel), 3);
% gt_rel_att = zeros(length(t_gt_rel), 4);
% gt_rel_w = zeros(length(t_gt_rel), 3);
% 
% 
% for i = 1:length(t_gt_rel)
%     gt_rel_pos(i,:) = chaser_gt_pos(i,:) - target_gt_pos(i,:);
%     gt_rel_vel(i,:) = chaser_gt_vel(i,:) - target_gt_vel(i,:);
%     gt_rel_w(i,:) = chaser_gt_w_body(i,:) - target_gt_w_body(i,:);
%     gt_rel_att(i,:) = qmult([-target_gt_att(i,1:3), target_gt_att(i,4)], chaser_gt_att(i,:));
% end
% 
% % Relative position (Target)
% figure(9);
% plot(t_gt_rel, gt_rel_pos(:,1), 'r-', ...
%      t_gt_rel, gt_rel_pos(:,2), 'g-',...
%      t_gt_rel, gt_rel_pos(:,3), 'b-');
% grid on;
% title('Relative position (Target frame)');
% xlabel('t (s)');
% ylabel('position (m)');
% legend('x', 'y', 'z');
% 
% % Relative velocity (Target)
% figure(10);
% plot(t_gt_rel, gt_rel_vel(:,1), 'r-', ...
%      t_gt_rel, gt_rel_vel(:,2), 'g-',...
%      t_gt_rel, gt_rel_vel(:,3), 'b-');
% grid on;
% title('Relative velocity (Target frame)');
% xlabel('t (s)');
% ylabel('velocity (m/s)');
% legend('x', 'y', 'z');
% 
% % Relative attitude (Target)
% figure(11);
% plot(t_gt_rel, gt_rel_att(:,1), 'r-', ...
%      t_gt_rel, gt_rel_att(:,2), 'g-',...
%      t_gt_rel, gt_rel_att(:,3), 'b-', ...
%      t_gt_rel, gt_rel_att(:,4), 'k-');
% grid on;
% title('Relative attitude (Target frame)');
% xlabel('t (s)');
% ylabel('attitude (quat)');
% legend('x', 'y', 'z', 'w');
% 
% % Relative angular velocity (Target)
% figure(12);
% plot(t_gt_rel, gt_rel_w(:,1), 'r-', ...
%      t_gt_rel, gt_rel_w(:,2), 'g-',...
%      t_gt_rel, gt_rel_w(:,3), 'b-');
% grid on;
% title('Relative angular velocity (Target frame)');
% xlabel('t (s)');
% ylabel('omega (rad/s)');
% legend('x', 'y', 'z');
% 

%% TODO: Get SLAM estimates
% 
% %% Inertial parameter estimation
J_truth = [0.153, 0.0, 0.0; ...
             0.0, 0.143, 0.0; ...
             0.0, 0.0, 0.162];
%x_true = [target_gt_att(1:end,:), target_gt_w_body(1:8545,:)]';
x_true = [target_est_att_unrotated, target_est_w_unrotated]';

%tspan = target_gt_pose_t;
tspan = target_est_pose_t;
% % check for weird time array bugs (some time-steps are the same as the
% % previous, which screws up ode45)
% i = 2;
% bad_idx = [];
% while i <= length(tspan)
%     if (tspan(i) - tspan(i-1) <= 0.000001 && i < 8545)
%         bad_idx = [bad_idx, i];
%     end
%     i = i + 1;
% end
% tspan(bad_idx) = [];
% x_true(:,bad_idx) = [];

% q0 = x_true(1:4,1)'; % for now, set initial quat condition to the ground truth value
% w0 = x_true(5:7,1)';


q01 = x_true(1:4,1);
q02 = x_true(1:4,2);
q03 = x_true(1:4,3);
q04 = x_true(1:4,4);
q05 = x_true(1:4,5);
w01 = x_true(5:7,1);
w02 = x_true(5:7,2);
w03 = x_true(5:7,3);
w04 = x_true(5:7,4);
w05 = x_true(5:7,5);
x01 = [q01;w01];
x02 = [q02;w02];
x03 = [q03;w03];
x04 = [q04;w04];
x05 = [q05;w05];

w01 = [-0.0 0.06 0.06]';

% Ok guess of w01 = [0 -0.06 0.06] rad/s
% p1_guess = [0.14, 0.14, 0.14, 0.005, 0.005, 0.005 w01'];
% [0.09 0.09 0.09 -0.07 -0.07 -0.07 -0.08 -0.08 -0.08], ...
 %   [0.2 0.2 0.2 0.07 0.07 0.07 0.08 0.08 0.08]);

p1 = optimvar('p', 9, "LowerBound", -0.2, "UpperBound", 0.2);

    p1_guess = [0.14, 0.14, 0.14, 0.005, 0.005, 0.005 w01'];
    % p1_guess = [0.14, 0.14, 0.14, 0.005, 0.04, 0.05 w01'];
    %[0.09 0.09 0.09 -0.07 -0.07 -0.07 -0.08 -0.08 -0.08], ...
    % [0.2 0.2 0.2 0.07 0.07 0.07 0.08 0.08 0.08])
    ODE_sol = ode45(@(t,x) dynamics_step(t,x,p1_guess), tspan, x01);
    result1 = deval(ODE_sol, tspan);

    %myfnc = fcn2optimexpr(@myfunc, p1, tspan, x01);
    %obj = sum(sum((myfnc - x_true(1:3,:)).^2));
    objective = @(x) objfunc(x, tspan, x_true, x01);

    [best_p, resnorm] = lsqnonlin(objective, p1_guess, [0.09 0.09 0.09 -0.07 -0.07 -0.07 -0.08 -0.08 -0.08], ...
    [0.2 0.2 0.2 0.07 0.07 0.07 0.08 0.08 0.08]);

    J = [best_p(1), best_p(4), best_p(5); ...
         best_p(4), best_p(2), best_p(6); ...
         best_p(5), best_p(6), best_p(3)]

    w0_est = [best_p(7);best_p(8);best_p(9)]

    [U, S, V] = svd(J);
    J_prinp = S
    resnorm

    w0_est_rot = V' * w0_est

    J_test = J;
    % try to propagate:
    ODE_Sol = ode45(@(t,x) dynamics_step(t, x, J_test), tspan, [q05; w0_est]);
    ODE_result1 = deval(ODE_Sol, tspan);

    for i = 2:length(ODE_result1(:,1))
        if (norm(ODE_result1(i,:) - ODE_result1(i-1,:)) > norm(-ODE_result1(i,:) - ODE_result1(i-1,:)))
            ODE_result1(i,:) = -ODE_result1(i,:);
        end
    end

    for i = 2:length(target_est_att_unrotated(:,1))
        if (norm(target_est_att_unrotated(i,:) - target_est_att_unrotated(i-1,:)) > norm(-target_est_att_unrotated(i,:) - target_est_att_unrotated(i-1,:)))
            target_est_att_unrotated(i,:) = -target_est_att_unrotated(i,:);
        end
    end

    figure(70);
    plot(tspan, ODE_result1(1,:), 'r-', ...
         tspan, ODE_result1(2,:), 'g-',...
         tspan, ODE_result1(3,:), 'b-',...
         tspan, ODE_result1(4,:), 'k-');
     grid on;
     hold on;
     plot(tspan, target_est_att_unrotated(:,1), 'r--', ...
         tspan, target_est_att_unrotated(:,2), 'g--', ...
         tspan, target_est_att_unrotated(:,3), 'b--', ...
         tspan, target_est_att_unrotated(:,4), 'k--');
     title('Propagated using estimated inertia');

 








% 
% 
% 
% 
% 
% 
% 
% q01 = x_true(1:4,1);
% w01 = x_true(5:7,1);
% 
% q02 = x_true(1:4,2);
% w02 = x_true(5:7,2);
% 
% q03 = x_true(1:4,3);
% w03 = x_true(5:7,3);
% 
% q04 = x_true(1:4,4);
% w04 = x_true(5:7,4);
% 
% q05 = x_true(1:4,5);
% w05 = x_true(5:7,5);
% 
% x01 = [q01; w01];
% x02 = [q02; w02];
% x03 = [q03; w03];
% x04 = [q04; w04];
% x05 = [q05; w05];
% 
% p1 = optimvar('p', 9, "LowerBound", -0.2, "UpperBound", 0.2);
% p1_guess = [0.16, 0.14, 0.12, 0, 0, 0 w01'];
% ODE_sol = ode45(@(t,x) dynamics_step(t,x,p1_guess), tspan, x01);
% result1 = deval(ODE_sol, tspan);
% 
% myfnc = fcn2optimexpr(@myfunc, p1, tspan, x01);
% obj = sum(sum((myfnc - x_true(1:3,:)).^2));
% 
% prob = optimproblem("Objective", obj);
% %prob.Objective = fcn2optimexpr;
% 
% % inertia physical constraints
% I33 = p1(1) + p1(2) >= p1(3);
% I11 = p1(2) + p1(3) >= p1(1);
% I22 = p1(1) + p1(3) >= p1(2);
% pos1 = p1(1) >= 0;
% pos2 = p1(2) >= 0;
% pos3 = p1(3) >= 0;
% prob.Constraints.I33 = I33;
% prob.Constraints.I11 = I11;
% prob.Constraints.I22 = I22;
% prob.Constraints.pos1 = pos1;
% prob.Constraints.pos2 = pos2;
% prob.Constraints.pos3 = pos3;
% 
% 
% % initial guess (can mess around and tune these, for now very close to the true ones)
% r0.p = p1_guess;
% 
% [p1sol, obj_result1] = solve(prob, r0);
% 
% J1_est = [p1sol.p(1), p1sol.p(4), p1sol.p(5);
%          p1sol.p(4), p1sol.p(2), p1sol.p(6);
%          p1sol.p(5), p1sol.p(6), p1sol.p(3)]
%      
% w01_est = [p1sol.p(7);p1sol.p(8);p1sol.p(9)]
% 
% [U, S, V] = svd(J1_est);
% J1_prinp = S
% 
% J1_test = p1sol.p;
% % try to propagate:
% ODE_Sol = ode45(@(t,x) dynamics_step(t, x, J1_test), tspan, [q01; w01_est]);
% ODE_result1 = deval(ODE_Sol, tspan);
% 
% figure(70);
% plot(tspan, ODE_result1(1,:), 'r-', ...
%      tspan, ODE_result1(2,:), 'g-',...
%      tspan, ODE_result1(3,:), 'b-',...
%      tspan, ODE_result1(4,:), 'k-');
%  grid on;
%  hold on;
%  plot(tspan, target_est_att_unrotated(:,1), 'r--', ...
%      tspan, target_est_att_unrotated(:,2), 'g--', ...
%      tspan, target_est_att_unrotated(:,3), 'b--', ...
%      tspan, target_est_att_unrotated(:,4), 'k--');
%  title('Propagated using estimated inertia');
% 
%  
%  %%%%%%%%%%%%%%%%%
%  
%  p2 = optimvar('p', 9, "LowerBound", -0.2, "UpperBound", 0.2);
% p2_guess = [0.16, 0.14, 0.12, 0, 0, 0 w02'];
% ODE_sol2 = ode45(@(t,x) dynamics_step(t,x,p2_guess), tspan(2:end), x02);
% result2 = deval(ODE_sol, tspan(2:end));
% 
% myfnc = fcn2optimexpr(@myfunc, p2, tspan(2:end), x02);
% obj = sum(sum((myfnc - x_true(1:3,2:end)).^2));
% 
% prob = optimproblem("Objective", obj);
% %prob.Objective = fcn2optimexpr;
% 
% % inertia physical constraints
% I33 = p2(1) + p2(2) >= p2(3);
% I11 = p2(2) + p2(3) >= p2(1);
% I22 = p2(1) + p2(3) >= p2(2);
% pos1 = p2(1) >= 0;
% pos2 = p2(2) >= 0;
% pos3 = p2(3) >= 0;
% prob.Constraints.I33 = I33;
% prob.Constraints.I11 = I11;
% prob.Constraints.I22 = I22;
% prob.Constraints.pos1 = pos1;
% prob.Constraints.pos2 = pos2;
% prob.Constraints.pos3 = pos3;
% 
% 
% % initial guess (can mess around and tune these, for now very close to the true ones)
% r0.p = p2_guess;
% 
% [p2sol, obj_result2] = solve(prob, r0);
% 
% J2_est = [p2sol.p(1), p2sol.p(4), p2sol.p(5);
%          p2sol.p(4), p2sol.p(2), p2sol.p(6);
%          p2sol.p(5), p2sol.p(6), p2sol.p(3)]
%      
% w02_est = [p2sol.p(7);p2sol.p(8);p2sol.p(9)]
% 
% [U, S, V] = svd(J2_est);
% J2_prinp = S
% 
% J2_test = p2sol.p;
% % try to propagate:
% ODE_Sol = ode45(@(t,x) dynamics_step(t, x, J2_test), tspan(2:end), [q02; w02_est]);
% ODE_result2 = deval(ODE_Sol, tspan(2:end));
% 
% figure(71);
% plot(tspan(2:end), ODE_result2(1,:), 'r-', ...
%      tspan(2:end), ODE_result2(2,:), 'g-',...
%      tspan(2:end), ODE_result2(3,:), 'b-',...
%      tspan(2:end), ODE_result2(4,:), 'k-');
%  grid on;
%  hold on;
%  plot(tspan, target_est_att_unrotated(:,1), 'r--', ...
%      tspan, target_est_att_unrotated(:,2), 'g--', ...
%      tspan, target_est_att_unrotated(:,3), 'b--', ...
%      tspan, target_est_att_unrotated(:,4), 'k--');
%  title('Propagated using estimated inertia');
%  
%  
%  %%%%%%%%%%%%%%%%%%%
%  
%  p3 = optimvar('p', 9, "LowerBound", -0.2, "UpperBound", 0.2);
% p3_guess = [0.16, 0.14, 0.12, 0, 0, 0 w03'];
% ODE_sol3 = ode45(@(t,x) dynamics_step(t,x,p3_guess), tspan(3:end), x03);
% result3 = deval(ODE_sol, tspan(3:end));
% 
% myfnc = fcn2optimexpr(@myfunc, p3, tspan(3:end), x03);
% obj = sum(sum((myfnc - x_true(1:3,3:end)).^2));
% 
% prob = optimproblem("Objective", obj);
% %prob.Objective = fcn2optimexpr;
% 
% % inertia physical constraints
% I33 = p3(1) + p3(2) >= p3(3);
% I11 = p3(2) + p3(3) >= p3(1);
% I22 = p3(1) + p3(3) >= p3(2);
% pos1 = p3(1) >= 0;
% pos2 = p3(2) >= 0;
% pos3 = p3(3) >= 0;
% prob.Constraints.I33 = I33;
% prob.Constraints.I11 = I11;
% prob.Constraints.I22 = I22;
% prob.Constraints.pos1 = pos1;
% prob.Constraints.pos2 = pos2;
% prob.Constraints.pos3 = pos3;
% 
% 
% % initial guess (can mess around and tune these, for now very close to the true ones)
% r0.p = p3_guess;
% 
% [p3sol, obj_result3] = solve(prob, r0);
% 
% J3_est = [p3sol.p(1), p3sol.p(4), p3sol.p(5);
%          p3sol.p(4), p3sol.p(2), p3sol.p(6);
%          p3sol.p(5), p3sol.p(6), p3sol.p(3)]
%      
% w03_est = [p3sol.p(7);p3sol.p(8);p3sol.p(9)]
% 
% [U, S, V] = svd(J3_est);
% J3_prinp = S
% 
% J3_test = p3sol.p;
% % try to propagate:
% ODE_Sol = ode45(@(t,x) dynamics_step(t, x, J3_test), tspan(3:end), [q03; w03_est]);
% ODE_result3 = deval(ODE_Sol, tspan(3:end));
% 
% figure(72);
% plot(tspan(3:end), ODE_result3(1,:), 'r-', ...
%      tspan(3:end), ODE_result3(2,:), 'g-',...
%      tspan(3:end), ODE_result3(3,:), 'b-',...
%      tspan(3:end), ODE_result3(4,:), 'k-');
%  grid on;
%  hold on;
%  plot(tspan, target_est_att_unrotated(:,1), 'r--', ...
%      tspan, target_est_att_unrotated(:,2), 'g--', ...
%      tspan, target_est_att_unrotated(:,3), 'b--', ...
%      tspan, target_est_att_unrotated(:,4), 'k--');
%  title('Propagated using estimated inertia');
%  
%  
%   %%%%%%%%%%%%%%%%%%%
%  
%  p4 = optimvar('p', 9, "LowerBound", -0.2, "UpperBound", 0.2);
% p4_guess = [0.16, 0.14, 0.12, 0, 0, 0 w04'];
% ODE_sol4 = ode45(@(t,x) dynamics_step(t,x,p4_guess), tspan(4:end), x04);
% result4 = deval(ODE_sol, tspan(4:end));
% 
% myfnc = fcn2optimexpr(@myfunc, p4, tspan(4:end), x04);
% obj = sum(sum((myfnc - x_true(1:3,4:end)).^2));
% 
% prob = optimproblem("Objective", obj);
% %prob.Objective = fcn2optimexpr;
% 
% % inertia physical constraints
% I33 = p4(1) + p4(2) >= p4(3);
% I11 = p4(2) + p4(3) >= p4(1);
% I22 = p4(1) + p4(3) >= p4(2);
% pos1 = p4(1) >= 0;
% pos2 = p4(2) >= 0;
% pos3 = p4(3) >= 0;
% prob.Constraints.I33 = I33;
% prob.Constraints.I11 = I11;
% prob.Constraints.I22 = I22;
% prob.Constraints.pos1 = pos1;
% prob.Constraints.pos2 = pos2;
% prob.Constraints.pos3 = pos3;
% 
% 
% % initial guess (can mess around and tune these, for now very close to the true ones)
% r0.p = p4_guess;
% 
% [p4sol, obj_result4] = solve(prob, r0);
% 
% J4_est = [p4sol.p(1), p4sol.p(4), p4sol.p(5);
%          p4sol.p(4), p4sol.p(2), p4sol.p(6);
%          p4sol.p(5), p4sol.p(6), p4sol.p(3)]
%      
% w04_est = [p4sol.p(7);p4sol.p(8);p4sol.p(9)]
% 
% [U, S, V] = svd(J4_est);
% J4_prinp = S
% 
% J4_test = p4sol.p;
% % try to propagate:
% ODE_Sol = ode45(@(t,x) dynamics_step(t, x, J4_test), tspan(4:end), [q04; w04_est]);
% ODE_result4 = deval(ODE_Sol, tspan(4:end));
% 
% figure(73);
% plot(tspan(4:end), ODE_result4(1,:), 'r-', ...
%      tspan(4:end), ODE_result4(2,:), 'g-',...
%      tspan(4:end), ODE_result4(3,:), 'b-',...
%      tspan(4:end), ODE_result4(4,:), 'k-');
%  grid on;
%  hold on;
%  plot(tspan, target_est_att_unrotated(:,1), 'r--', ...
%      tspan, target_est_att_unrotated(:,2), 'g--', ...
%      tspan, target_est_att_unrotated(:,3), 'b--', ...
%      tspan, target_est_att_unrotated(:,4), 'k--');
%  title('Propagated using estimated inertia');
%  
%  
%  
%  
%  
%  
%   %%%%%%%%%%%%%%%%%%%
%  
%  p5 = optimvar('p', 9, "LowerBound", -0.2, "UpperBound", 0.2);
% p5_guess = [0.16, 0.14, 0.12, 0, 0, 0 w05'];
% ODE_sol5 = ode45(@(t,x) dynamics_step(t,x,p5_guess), tspan(5:end), x05);
% result5 = deval(ODE_sol, tspan(5:end));
% 
% myfnc = fcn2optimexpr(@myfunc, p5, tspan(5:end), x05);
% obj = sum(sum((myfnc - x_true(1:3,5:end)).^2));
% 
% prob = optimproblem("Objective", obj);
% %prob.Objective = fcn2optimexpr;
% 
% % inertia physical constraints
% I33 = p5(1) + p5(2) >= p5(3);
% I11 = p5(2) + p5(3) >= p5(1);
% I22 = p5(1) + p5(3) >= p5(2);
% pos1 = p5(1) >= 0;
% pos2 = p5(2) >= 0;
% pos3 = p5(3) >= 0;
% prob.Constraints.I33 = I33;
% prob.Constraints.I11 = I11;
% prob.Constraints.I22 = I22;
% prob.Constraints.pos1 = pos1;
% prob.Constraints.pos2 = pos2;
% prob.Constraints.pos3 = pos3;
% 
% 
% % initial guess (can mess around and tune these, for now very close to the true ones)
% r0.p = p5_guess;
% 
% [p5sol, obj_result5] = solve(prob, r0);
% 
% J5_est = [p5sol.p(1), p5sol.p(4), p5sol.p(5);
%          p5sol.p(4), p5sol.p(2), p5sol.p(6);
%          p5sol.p(5), p5sol.p(6), p5sol.p(3)]
%      
% w05_est = [p5sol.p(7);p5sol.p(8);p5sol.p(9)]
% 
% [U, S, V] = svd(J5_est);
% J5_prinp = S
% 
% J5_test = p5sol.p;
% % try to propagate:
% ODE_Sol = ode45(@(t,x) dynamics_step(t, x, J5_test), tspan(5:end), [q05; w05_est]);
% ODE_result5 = deval(ODE_Sol, tspan(5:end));
% 
% figure(73);
% plot(tspan(5:end), ODE_result5(1,:), 'r-', ...
%      tspan(5:end), ODE_result5(2,:), 'g-',...
%      tspan(5:end), ODE_result5(3,:), 'b-',...
%      tspan(5:end), ODE_result5(4,:), 'k-');
%  grid on;
%  hold on;
%  plot(tspan, target_est_att_unrotated(:,1), 'r--', ...
%      tspan, target_est_att_unrotated(:,2), 'g--', ...
%      tspan, target_est_att_unrotated(:,3), 'b--', ...
%      tspan, target_est_att_unrotated(:,4), 'k--');
%  title('Propagated using estimated inertia');
% 



% with current guess and ground truth data, seems to work OK.
% note that inertia parameters are determined up to scale (which is
% perfectly OK since they're being used to propagate a zero-torque case)



