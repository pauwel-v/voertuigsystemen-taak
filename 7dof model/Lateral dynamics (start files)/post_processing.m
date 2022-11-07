function [] = post_processing(X, data, vehicle_param, inputs)
%
% This function is for post_processing the information form the
% time-integration. 
close all;

% set(0,'DefaultAxesXGrid','on')
% set(0,'DefaultAxesYGrid','on')
% set(0,'defaulttextinterpreter','latex')
% set(0,'defaultlegendinterpreter','latex')
data = [zeros(size(data,1),1) data];


load('dataopdracht1nieuw2')
tijd = (1:603)*0.01;
xcoord0 = -dataopdracht1nieuw2(1:603,4)/10^3;
xcoord1 = xcoord0-xcoord0(3);
ycoord0 = dataopdracht1nieuw2(1:603,2)/10^3;
ycoord1 = ycoord0-ycoord0(3);
hoek = -(dataopdracht1nieuw2(1:603,5)+180)*pi/180;

figure
subplot(311)
plot(inputs.time, X(1,:)), axis tight, ylabel('x [m]')
hold on
plot(tijd, xcoord1)
legend('Matlab','Simcenter', 'Location', 'SouthEast')
subplot(312)
plot(inputs.time, X(2,:)), axis tight, ylabel('y [m]')
hold on
plot(tijd, ycoord1)
legend('Matlab','Simcenter', 'Location', 'SouthEast')
subplot(313)
plot(inputs.time, X(3,:)), axis tight, ylabel('\phi [rad]')
hold on
plot(tijd, hoek)
legend('Matlab','Simcenter')
sgtitle('Comparison Matlab and Simcenter 3D simulations')

% figure
% plot(inputs.time, 





%%

myVideo = VideoWriter('animation');
myVideo.FrameRate = 30;
open(myVideo)

if inputs.animation == 1
    for k = 1:10:length(inputs.time)


        p(:,1) = [X(1,k) - vehicle_param.b*cos(X(3,k)); X(2,k) - vehicle_param.b*sin(X(3,k))];
        p(:,2) = [X(1,k) + vehicle_param.a*cos(X(3,k)); X(2,k) + vehicle_param.a*sin(X(3,k))];

        figure(123)
        plot(p(1,:)', p(2,:)', 'b-o', 'linewidth', 2), axis([-10 80 -10 60])
        grid on
        xlabel('x [m]')
        ylabel('y [m]')

        pause(0.001)
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
    end
    close(myVideo)
end

% % Animation (Uncomment this section to animate your model)
% Ackerman_R = vehicle_param.L/inputs.delta(1,1); % delta = L/R => R=L/Delta
% 
% for k = 1:10:length(inputs.time)
%     
%     Cm(:,1) = [X(1,k) ; X(2,k)];
%     
%     i = 1; % Driven path
%     for j=1:10:k
%         Path(:,i)= [X(1,j) ; X(2,j)];
%         i=i+1;
%     end
%     
%     p(:,1) = [X(1,k) - cos(X(3,k)) * vehicle_param.b + vehicle_param.tr/2 * sin(X(3,k));  X(2,k) - sin(X(3,k)) * vehicle_param.b - vehicle_param.tr/2 * cos(X(3,k))]; 
%     p(:,2) = [X(1,k) - cos(X(3,k)) * vehicle_param.b - vehicle_param.tr/2 * sin(X(3,k));  X(2,k) - sin(X(3,k)) * vehicle_param.b + vehicle_param.tr/2 * cos(X(3,k))];
%     p(:,3) = [X(1,k) + cos(X(3,k)) * vehicle_param.a - vehicle_param.tf/2 * sin(X(3,k));  X(2,k) + sin(X(3,k)) * vehicle_param.a + vehicle_param.tf/2 * cos(X(3,k))];
%     p(:,4) = [X(1,k) + cos(X(3,k)) * vehicle_param.a + vehicle_param.tf/2 * sin(X(3,k));  X(2,k) + sin(X(3,k)) * vehicle_param.a - vehicle_param.tf/2 * cos(X(3,k))];
%     p(:,5) = [X(1,k) - cos(X(3,k)) * vehicle_param.b + vehicle_param.tr/2 * sin(X(3,k));  X(2,k) - sin(X(3,k)) * vehicle_param.b - vehicle_param.tr/2 * cos(X(3,k))];
%     
%     Wrl (:,1) = [X(1,k) - cos(X(3,k)) * vehicle_param.b - vehicle_param.tr/2 * sin(X(3,k));  X(2,k) - sin(X(3,k)) * vehicle_param.b + vehicle_param.tr/2 * cos(X(3,k))];
%     Wrl (:,2) = [X(1,k) - cos(X(3,k)) * vehicle_param.b - vehicle_param.tr/2 * sin(X(3,k)) + vehicle_param.r_w * cos(X(3,k));  X(2,k) - sin(X(3,k)) * vehicle_param.b + vehicle_param.tr/2 * cos(X(3,k))+ vehicle_param.r_w * sin(X(3,k))];
%     Wrl (:,3) = [X(1,k) - cos(X(3,k)) * vehicle_param.b - vehicle_param.tr/2 * sin(X(3,k)) - vehicle_param.r_w * cos(X(3,k));  X(2,k) - sin(X(3,k)) * vehicle_param.b + vehicle_param.tr/2 * cos(X(3,k))- vehicle_param.r_w * sin(X(3,k))];
%     
%     Wrr (:,1) = [X(1,k) - cos(X(3,k)) * vehicle_param.b + vehicle_param.tr/2 * sin(X(3,k));  X(2,k) - sin(X(3,k)) * vehicle_param.b - vehicle_param.tr/2 * cos(X(3,k))]; 
%     Wrr (:,2) = [X(1,k) - cos(X(3,k)) * vehicle_param.b + vehicle_param.tr/2 * sin(X(3,k)) + vehicle_param.r_w * cos(X(3,k));  X(2,k) - sin(X(3,k)) * vehicle_param.b - vehicle_param.tr/2 * cos(X(3,k))+ vehicle_param.r_w * sin(X(3,k))];
%     Wrr (:,3) = [X(1,k) - cos(X(3,k)) * vehicle_param.b + vehicle_param.tr/2 * sin(X(3,k)) - vehicle_param.r_w * cos(X(3,k));  X(2,k) - sin(X(3,k)) * vehicle_param.b - vehicle_param.tr/2 * cos(X(3,k))- vehicle_param.r_w * sin(X(3,k))];
%     
%     Wfl (:,1) = [X(1,k) + cos(X(3,k)) * vehicle_param.a - vehicle_param.tf/2 * sin(X(3,k));  X(2,k) + sin(X(3,k)) * vehicle_param.a + vehicle_param.tf/2 * cos(X(3,k))];
%     Wfl (:,2) = [X(1,k) + cos(X(3,k)) * vehicle_param.a - vehicle_param.tf/2 * sin(X(3,k)) + vehicle_param.r_w * cos(X(3,k)+inputs.delta(1,k));  X(2,k) + sin(X(3,k)) * vehicle_param.a + vehicle_param.tf/2 * cos(X(3,k))+ vehicle_param.r_w * sin(X(3,k)+inputs.delta(1,k))];
%     Wfl (:,3) = [X(1,k) + cos(X(3,k)) * vehicle_param.a - vehicle_param.tf/2 * sin(X(3,k)) - vehicle_param.r_w * cos(X(3,k)+inputs.delta(1,k));  X(2,k) + sin(X(3,k)) * vehicle_param.a + vehicle_param.tf/2 * cos(X(3,k))- vehicle_param.r_w * sin(X(3,k)+inputs.delta(1,k))];
%     
%     Wfr (:,1) = [X(1,k) + cos(X(3,k)) * vehicle_param.a + vehicle_param.tf/2 * sin(X(3,k));  X(2,k) + sin(X(3,k)) * vehicle_param.a - vehicle_param.tf/2 * cos(X(3,k))];
%     Wfr (:,2) = [X(1,k) + cos(X(3,k)) * vehicle_param.a + vehicle_param.tf/2 * sin(X(3,k)) + vehicle_param.r_w * cos(X(3,k)+inputs.delta(1,k));  X(2,k) + sin(X(3,k)) * vehicle_param.a - vehicle_param.tf/2 * cos(X(3,k))+ vehicle_param.r_w * sin(X(3,k)+inputs.delta(1,k))];
%     Wfr (:,3) = [X(1,k) + cos(X(3,k)) * vehicle_param.a + vehicle_param.tf/2 * sin(X(3,k)) - vehicle_param.r_w * cos(X(3,k)+inputs.delta(1,k));  X(2,k) + sin(X(3,k)) * vehicle_param.a - vehicle_param.tf/2 * cos(X(3,k))- vehicle_param.r_w * sin(X(3,k)+inputs.delta(1,k))];
%    
%     figure(555)
%     plot(p(1,:)', p(2,:)', 'b', Wrl(1,:)', Wrl(2,:)', 'g', Wrr(1,:)', Wrr(2,:)', 'g', Wfr(1,:)', Wfr(2,:)','r', Wfl(1,:)', Wfl(2,:)','r', Cm(1,:)', Cm(2,:)', 'go', 'linewidth', 2), axis([-20 20 -5 35]) % AXIS([XMIN XMAX YMIN YMAX])
%     
%     hold on
%     
%     plot(Path(1,:)', Path(2,:)','--','linewidth', 1)
%     
%     th = 0:pi/50:2*pi;
%     xunit = Ackerman_R * cos(th) + 0;
%     yunit = Ackerman_R * sin(th) + Ackerman_R;
%     plot(xunit, yunit,'r--','linewidth', 1 );
%     
%     hold off
%     
% end

%%

% energy = zeros(size(inputs.time));
% for k = 1:length(inputs.time)
%     
%     energy(k) = 1/2*((X(vehicle_param.n_dofs+1,k)^2+X(vehicle_param.n_dofs+2,k)^2)*vehicle_param.M + ...
%         X(vehicle_param.n_dofs+3,k)^2*vehicle_param.Izz + X(vehicle_param.n_dofs+4,k)^2*vehicle_param.I_w+...
%         X(vehicle_param.n_dofs+5,k)^2*vehicle_param.I_w);
%     
% end

% figure
% plot(inputs.time, energy, 'linewidth', 1.5), axis tight, xlabel('t [s]'), ylabel('Energy [J]')
% figure
% plot(X(1,:), X(2,:))
% xlabel('x [m]')
% ylabel('y [m]')
% title('Vehicle trajectory Matlab')

% figure
% plot(inputs.time,data([1,4,7,10],:));
% legend('$F_{fl,x}$','$F_{fr,x}$','$F_{rl,x}$','$F_{rr,x}$')
% xlabel('t [s]')
% ylabel('[N]')
% title('Longitudinal tyre forces')

% figure
% plot(inputs.time,data([2,5,8,11],:));
% legend('$F_{fl,y}$','$F_{fr,y}$','$F_{rl,y}$','$F_{rr,y}$')
% xlabel('t [s]')
% ylabel('[N]')
% title('Lateral tyre forces')

% figure
% plot(inputs.time,data([3,6,9,12],:));
% legend('$F_{fl,z}$','$F_{fr,z}$','$F_{rl,z}$','$F_{rr,z}$')
% xlabel('t [s]')
% ylabel('[N]')
% title('Normal tyre forces')

% figure
% plot(inputs.time,X([4,5],:));
% hold on
% plot(inputs.time,X([6,7],:));
% legend('$\theta_{fr}$', '$\theta_{fl}$','$\theta_{rr}$', '$\theta_{rl}$')
% xlabel('t [s]')
% ylabel('[rad]')
% title('Tyre angles')


% figure
% plot(inputs.time, data(17,:));
% title('$F_{aero}$')
% xlabel('t [s]')
% ylabel('[N]')


% figure
% plot(inputs.time, data([13,14,15,16],:));
% legend('$s_{fl}$', '$s_{fr}$', '$s_{rl}$', '$s_{rr}$')
% xlabel('t [s]')
% ylabel('[-]')
% title('Longitudinal slip values')

% figure
% subplot(411) 
% plot(inputs.time, data(18,:));
% ylabel('$\beta_{fl}$')
% subplot(412)
% plot(inputs.time, data(19,:));
% ylabel('$\beta_{fr}$')
% subplot(413)
% plot(inputs.time, data(20,:));
% ylabel('$\beta_{rl}$')
% subplot(414)
% plot(inputs.time, data(21,:));
% ylabel('$\beta_{rr}$')
% xlabel('t [s]')
% sgtitle('Slip angles')

% torque vectoring
figure
plot(inputs.time, data(47:48,:));
xlabel('t [s]')
ylabel('[Nm]')
axis([0 6 -60 150])
legend('Torque left', 'Torque right');
title('Torque vectoring')

figure
yyaxis left
plot(inputs.time, data(24,:));
hold on
plot(inputs.time, X(vehicle_param.n_dofs + 3,:));
% plot(inputs.time, inputs.delta);
yyaxis right
plot(inputs.time, data(25,:));
ylabel('V_x [m/s]')
xlabel('t [s]')
legend('Ackerman yaw rate [rad/s]', 'Effective yaw rate [rad/s]')%, 'steering angle delta [rad]')
sgtitle('Comparison Ackerman yaw rate and effective yaw rate')

figure
subplot(311)
plot(inputs.time, X(1,:))
hold on
plot(inputs.time, X(23,:))
ylabel('x [m]')
legend('Without torque vectoring', 'With torque vectoring', 'Location', 'SouthEast')
subplot(312)
plot(inputs.time, X(2,:))
hold on
plot(inputs.time, X(24,:))
ylabel('y [m]')
legend('Without torque vectoring', 'With torque vectoring', 'Location', 'SouthEast')
subplot(313)
plot(inputs.time, X(3,:))
hold on 
plot(inputs.time, X(25,:))
ylabel('\phi [rad]')
xlabel('t [s]')
sgtitle('Comparison without and with torque vectoring')
legend('Without torque vectoring', 'With torque vectoring')

figure
plot(inputs.time, data(22:23,:));
hold on
plot(inputs.time, data(47:48,:));
xlabel('t [s]')
ylabel('T [Nm]')

legend('Torque left (no T-vec)', 'Torque right(no T-vec)', 'Torque left (T-vec)', 'Torque right(T-vec)');
title('Torque vectoring')

figure
plot(inputs.time, data(24,:));
hold on
plot(inputs.time, X(vehicle_param.n_dofs + 3,:));
hold on
plot(inputs.time, X(22+vehicle_param.n_dofs+3,:));
% plot(inputs.time, inputs.delta);
xlabel('t [s]')
ylabel('d\phi [rad/s]')
legend('Ackerman yaw rate', 'Effective yaw rate without T-vec', 'Effective yaw rate with T-vec')
title('Comparison yaw rates to Ackerman yaw rate without and with torque vectoring')

figure
subplot(311)
plot(inputs.time, X(23,:))
hold on
plot(inputs.time, X(45,:))
ylabel('x [m]')
legend('Static slip', 'Transient slip')
subplot(312)
plot(inputs.time, X(24,:))
hold on
plot(inputs.time, X(46,:))
ylabel('y [m]')
legend('Static slip', 'Transient slip')
subplot(313)
plot(inputs.time, X(25,:))
hold on 
plot(inputs.time, X(47,:))
ylabel('\phi [rad]')
xlabel('t [s]')
legend('Static slip', 'Transient slip')
sgtitle('Comparison Matlab simulations with static and transient slip')

figure
plot(inputs.time, data([13,14,15,16],:));
hold on 
plot(inputs.time, data([63, 64,65,66],:), '--');
legend('s_{fl}', 's_{fr}', '$s_{rl}$', '$s_{rr}$','transient $s_{fl}$', 'transient $s_{fr}$', 'transient $s_{rl}$', 'transient $s_{rr}$')
sgtitle('Comparison non-transient and transient longitudinal slip')

dataopdracht3Tvec = load('dataopdracht3Tvec');
dataopdracht3Tvec = dataopdracht3Tvec.dataopdracht3Tvec1;

tijd = (1:603)*0.01;
xcoord0 = -dataopdracht1nieuw2(1:603,4)/10^3;
xcoord1 = xcoord0-xcoord0(3);
ycoord0 = dataopdracht1nieuw2(1:603,2)/10^3;
ycoord1 = ycoord0-ycoord0(3);
hoek = -(dataopdracht1nieuw2(1:603,5)+180)*pi/180;

xcoord2 = -dataopdracht3Tvec(1:length(tijd),3)/10^3;
xcoord2 = xcoord2 - xcoord2(1);
ycoord2 = dataopdracht3Tvec(1:length(tijd),4)/10^3;
ycoord2 = ycoord2 - ycoord2(1);

hoek2 = -(dataopdracht3Tvec(1:length(tijd),2)+180)*pi/180;

figure
subplot(311)
plot(tijd, xcoord1)
hold on
plot(tijd, xcoord2)
legend('Without torque vectoring', 'With torque vectoring', 'Location', 'SouthEast')
subplot(312)
plot(tijd, ycoord1)
hold on
plot(tijd, ycoord2)
legend('Without torque vectoring', 'With torque vectoring', 'Location', 'SouthEast')
subplot(313)
plot(tijd, hoek)
hold on
plot(tijd, hoek2)
sgtitle('Simcenter without and with torque vectoring')
legend('Without torque vectoring', 'With torque vectoring')



% legend('F_{fl,x}','F_{fl,y}','F_{fl,z}','F_{fr,x}','F_{fr,y}','F_{fr,z}','F_{rl,x}','F_{rl,y}','F_{rl,z}','F_{rr,x}','F_{rr,y}','F_{rr,z}')
% figure
% subplot(3,1,1)
% plot(inputs.time,data(1,:)),ylabel('F_{fl,long}')
% 
% subplot(3,1,2)
% plot(inputs.time,data(2,:)),ylabel('F_{fl,lat}')
% 
% subplot(3,1,3)
% plot(inputs.time,data(3,:)),ylabel('F_{fl,N}')
