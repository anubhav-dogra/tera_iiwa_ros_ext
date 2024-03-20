clc
clear
prefix = 'manipulability_robot_test';
mid = num2str(8);
ext = '.bag';
name = strcat(prefix,mid,ext);

bag = rosbag(name);
bSel = select(bag,'Topic','/cartesian_wrench_tool');
msgStructs = readMessages(bSel,'DataFormat','struct');
tx = cellfun(@(m) double(m.Wrench.Torque.X),msgStructs);
ty = cellfun(@(m) double(m.Wrench.Torque.Y),msgStructs);
tz = cellfun(@(m) double(m.Wrench.Torque.Z),msgStructs);
fx = cellfun(@(m) double(m.Wrench.Force.X),msgStructs);
fy = cellfun(@(m) double(m.Wrench.Force.Y),msgStructs);
fz = cellfun(@(m) double(m.Wrench.Force.Z),msgStructs);
n = size(fx,1);
time_init = double(msgStructs{1}.Header.Stamp.Sec);
time_final = double(msgStructs{n}.Header.Stamp.Sec);
n_sec = time_final-time_init;
time=linspace(0,n_sec,n);

bSel1 = select(bag,'Topic','/tool_link_ee_pose');
msgStructs1 = readMessages(bSel1,'DataFormat','struct');
ox = cellfun(@(m) double(m.Transform.Rotation.X),msgStructs1);
oy = cellfun(@(m) double(m.Transform.Rotation.Y),msgStructs1);
oz = cellfun(@(m) double(m.Transform.Rotation.Z),msgStructs1);
x = cellfun(@(m) double(m.Transform.Translation.X),msgStructs1);
y = cellfun(@(m) double(m.Transform.Translation.Y),msgStructs1);
z = cellfun(@(m) double(m.Transform.Translation.Z),msgStructs1);
n1 = size(x,1);
time_init1 = double(msgStructs1{1}.Header.Stamp.Sec);
time_final1 = double(msgStructs1{n1}.Header.Stamp.Sec);
n_sec1 = time_final1-time_init1;
time1=linspace(0,n_sec1,n1);

%%
subplot(2,1,1)
plot(time, fz,'-r', 'LineWidth',2)
grid on
box on
xlabel('Time (sec)')
ylabel('F_z (N)')
xlim([0 50])
set(gca,'LineWidth',0.75,'FontSize',16,'XMinorTick','on','YMinorTick','on','TickLength',[.01 0.1], 'XMinorGrid','on','YMinorGrid','on');

subplot(2,1,2)
plot(time1, 1000*z,'-k', 'LineWidth',2)
grid on
box on
xlabel('Time (sec)')
ylabel('EE tip {z} (mm)')
xlim([0 50])
set(gca,'LineWidth',0.75,'FontSize',16,'XMinorTick','on','YMinorTick','on','TickLength',[.01 0.1], 'XMinorGrid','on','YMinorGrid','on');
% figure('Color','w','units','normalized','OuterPosition',[.1 .2 .5 .5])
% % 
% figure(2),plot(time, xyz_{i}(:,1), 'LineWidth',2);
% hold on
% figure(2),plot(time, xyz_{i}(:,2), 'LineWidth',2);
% figure(2),plot(time, xyz_{i}(:,3), 'LineWidth',2);
%%
% figure(2),plot(time1, ox1, 'LineWidth',2);
% hold on
% figure(2),plot(time1, oy1, 'LineWidth',2);
% figure(2),plot(time1, oz1, 'LineWidth',2);
% figure(2),plot(time1, ow1, 'LineWidth',2);

% colormap1=jet(11);
% for i=1:11
% figure(1),plot(x,y+i,'color',colormap1(i,:),'LineWidth',1),hold on
% end
% set(gca,'LineWidth',0.75,'FontSize',16,'XMinorTick','on','YMinorTick','on','TickLength',[.01 0.1], 'XMinorGrid','on','YMinorGrid','on');
% xlabel('Time (sec)')
% ylabel('Force (N)')
% grid on
% tightfig;
%text(4,-.4,'Newtons','FontSize',16,'Color','b')

% for i = 1:3
%     for j = 1:4
%     plot(time(1:5:numpoints),hampel(delta_eul_{j}(1:5:numpoints,i)),'LineWidth',1.5)
%     hold on
%     end
%     set(gca,'LineWidth',1,'FontSize',18,'XMinorTick','on','YMinorTick','on','TickLength',[.01 0.1], 'XMinorGrid','on','YMinorGrid','on');
%     legend('Trial 1','Trial 2','Trial 3','Trial 4')
%     figure
% end
%%
% boxplot(box_x,'BoxStyle','outline','Colors','y')
% boxchart(box_x,'BoxFaceColor',"#EDB120","BoxEdgeColor",'k')
