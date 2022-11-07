% function [] = post_processing(X, data, vehicle_param, inputs)
% %
% % This function is for post_processing the information form the
% % time-integration. 
% 
% 
% figure
% plot(inputs.time, X(1,:)), axis tight, xlabel('time [s]')
% hold on
% plot(inputs.time, X(4,:))
% legend('Position [m]', 'Velocity [m/s]')
% 
% figure
% semilogy(inputs.time(2:end), data'), axis tight, xlabel('time [s]')
% legend('F_{wf}', 'F_{wr}', 'F_{nf}', 'F_{nr}')
% 

%%
% Uncomment this for a visualization of the longitudinal car
% 
% for k = 1:10:length(inputs.time)
%     
%    
%     p(:,1) = [X(1,k) - vehicle_param.b; vehicle_param.r_w];
%     p(:,2) = [X(1,k) - vehicle_param.b; vehicle_param.r_w+0.5];
%     p(:,3) = [X(1,k) - vehicle_param.b+0.4; vehicle_param.r_w+0.5];
%     p(:,4) = [X(1,k) - vehicle_param.b+0.4; vehicle_param.r_w+0.5+0.3];
%     p(:,5) = [X(1,k) + vehicle_param.a-1; vehicle_param.r_w+0.5+0.3];
%     p(:,6) = [X(1,k) + vehicle_param.a-1; vehicle_param.r_w+0.5];
%     p(:,7) = [X(1,k) + vehicle_param.a;  vehicle_param.r_w+0.5];
%     p(:,8) = [X(1,k) + vehicle_param.a; vehicle_param.r_w];
%     p(:,9) = p(:,1);
%     
%     wr(:,1) = [X(1,k) - vehicle_param.b; vehicle_param.r_w];
%     wr(:,2) = [X(1,k) - vehicle_param.b-sin(X(3,k))*vehicle_param.r_w; vehicle_param.r_w-cos(X(3,k))*vehicle_param.r_w];
%     
%     
%     wf(:,1) = [X(1,k) + vehicle_param.a; vehicle_param.r_w];
%     wf(:,2) = [X(1,k) + vehicle_param.a-sin(X(2,k))*vehicle_param.r_w; vehicle_param.r_w-cos(X(2,k))*vehicle_param.r_w];
%     
%     figure(123)
%     plot(p(1,:)', p(2,:)', 'b', wr(1,:), wr(2,:), 'r', wf(1,:), wf(2,:), 'g', 'linewidth', 2), axis([-2 8 -2 8])
%     
% %     pause
% end
function [] = post_processing(X, data, vehicle_param, inputs)
%
% This function is for post_processing the information form the
% time-integration. 
t = inputs.time;
w = smooth_input(0.5,0.75, inputs);
figure
plot(t, X(1,:)-w);
hold on
%plot(t, X(2,:));

legend('X_1 with C', 'X_2 with C', 'X_1 no C', 'X_2 no C')