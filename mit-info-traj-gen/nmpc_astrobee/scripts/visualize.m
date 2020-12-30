figure()

subplot(2,3,1)
plot(time_store, state_sim(:,1), 'r')
title('x');

subplot(2,3,2)
plot(time_store, state_sim(:,2), 'r')
title('y');

subplot(2,3,3)
plot(time_store,state_sim(:,3), 'r')
title('z');
X = state_sim(:,1);
Y = state_sim(:,2);
Z = state_sim(:,3);

subplot(2,3,4)
plot(time_store, state_sim(:,4), 'r')
title('Vx');

subplot(2,3,5)
plot(time_store,state_sim(:,5) , 'r')
title('Vy');

subplot(2,3,6)
plot(time_store,state_sim(:,6) , 'r')
title('Vz');

velx = state_sim(:,4);
vely = state_sim(:,5);
velz = state_sim(:,6);
vel = [velx vely velz];

figure()
subplot(2,2,1)
plot(time_store, state_sim(:,7), 'r')
title('q1');

subplot(2,2,2)
plot(time_store, state_sim(:,8), 'r')
title('q2');

subplot(2,2,3)
plot(time_store, state_sim(:,9), 'r')
title('q3');

subplot(2,2,4)
plot(time_store, state_sim(:,10), 'r')
title('q4');

quat1 = state_sim(:,7);
quat2= state_sim(:,8);
quat3 = state_sim(:,9);
quat4 =  state_sim(:,10);

quat = [quat1 quat2 quat3 quat4];





angvelx = state_sim(:,11);
angvely = state_sim(:,12);
angvelz = state_sim(:,13);
omega  = [angvelx angvely angvelz];




q_des
q_obtained =  [state_sim(end,7); state_sim(end,8); state_sim(end,9);state_sim(end,10) ]
rad2deg(acos(q_obtained(1)))*2;
pos_obtained = [state_sim(end,1); state_sim(end,2); state_sim(end,3) ] 

pos = [X, Y, Z];
q =  [state_sim(:,7), state_sim(:,8), state_sim(:,9),state_sim(:,10) ];





figure()
subplot(2,2,1)
plot(time_store, state_sim(:,11), 'r')
title('Omega_x');

subplot(2,2,2)
plot(time_store, state_sim(:,12), 'r')
title('Omega_y');

subplot(2,2,3)
plot(time_store, state_sim(:,13), 'r')
title('Omega_z');
 U = [control_ip];
 
figure()
time_store = time_store(1:end-1);
 subplot(2,3,1)
 plot(time_store, control_ip(:,1), 'r')
 title('u1');
 
 subplot(2,3,2)
 plot(time_store, control_ip(:,2), 'r')
 title('u2');
 
 subplot(2,3,3)
 plot(time_store, control_ip(:,3), 'r')
 title('u3');
 
 subplot(2,3,4)
 plot(time_store, control_ip(:,4), 'r')
 title('u4');
 
 subplot(2,3,5)
 plot(time_store, control_ip(:,5), 'r')
 title('u5');
 
 subplot(2,3,6)
 plot(time_store, control_ip(:,6), 'r')
title('u6');
 
 
% M = [time_store, pos, vel, quat, omega];
% csvwrite('../../../../../../../V-REP_PRO_EDU_V3_4_0_Linux/programming/cpp/VmatlabDat.csv',M );
% 
% 
% M = [time_store'; pos(:,1)';pos(:,2)'; pos(:,3)'; vel(:,1)'; vel(:,2)'; vel(:,3)'; quat(:,1)'; quat(:,2)';quat(:,3)';quat(:,4)';omega(:,1)';omega(:,2)';omega(:,3)'];
% fileID = fopen('../../../../../../../V-REP_PRO_EDU_V3_4_0_Linux/programming/cpp/VmatlabDat.txt','w');
% fprintf(fileID,'%6.8f %6.8f %6.8f %6.8f %6.8f %6.8f %6.8f %6.8f %6.8f  %6.8f %6.8f %6.8f %6.8f %6.8f \n',M);
% fclose(fileID);





