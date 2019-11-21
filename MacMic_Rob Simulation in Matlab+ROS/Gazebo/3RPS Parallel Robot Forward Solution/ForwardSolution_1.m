% 时间：2019年11月21日
% 功能：3-RPS并联平台位姿正解求解（已知驱动杆长，求动平台位姿参数)
% 算法：给定初始位姿，利用梯度搜索法循环


%% Initialization
% initial_data =load('F:\科研\Project\室内粉刷研究\课题答辩\毕业论文\写作素材\第5章\微机器人调姿测试实验\POSE_MOTOR_ENCODER.mat',data1)
% data = (initial_data)';
L_len(1,:) = 125 + 100*data2(1,:);
L_len(2,:) = 125 + 100*data2(3,:);
L_len(3,:) = 125 + 100*data2(2,:);
data_begin =2150;
data_end = 4150;

%% Calculation Loop
i = 1;
for j = data_begin:data_end
    
    L_actual = L_len(:,j); % 待求的未知位姿对应的驱动杆长，已知量
    
    % solution #01
    L_origin = [sum(L_len(:,j))/3; sum(L_len(:,j))/3; sum(L_len(:,j))/3]; % 数值迭代计算的起始位姿（两平台平行）下对应的驱动杆长，已知量
%     L_theory = L_origin;
    P_origin = [ sum(L_len(:,j))/3; 0; 0 ]; % 基准位姿，自定（P = [ 动平台中心点 Z坐标值， α， β] )
    Jac_origin = [ 0.3333    0.3333    0.3333; 
                   0.0000    0.0096   -0.0096; 
                  -0.0111    0.0056    0.0056 ];
              
    % Numerical iterative method based on rod length approximation
    [ P_actual, Jac_actual ] = LinkLengthIteration( L_actual, L_origin, P_origin, Jac_origin );
      
%     P_actual(:,j) = P_origin;  
      P_actual(:,i) = P_actual;
      i = i+1;
end

%% Figure Plot

% plot3( P_actual(2,:), P_actual(3,:), P_actual(1,:),'.');
% plot(P_actual(3,:), '.');
x = [data_begin:data_end];
y1 = data2(end,data_begin:data_end);
y2 = P_actual(3,:);
plot(x, y1,'-.',x,y2, '-' );
legend('实测','理论', 'Location','northeast');
% set(gca, 'YLim', [-2.0 1], 'YTick', [-2.0:0.2:1]);
% save Posture_actual.mat, P_actual
% xlswrite('C:\Users\Administrator\Documents\MATLAB\PKM Forward Kinematics\3-RPS\Posture_actual.xlsx', P_actual, '动平台实际位姿')

title('基于正解的3-RPS并联机构工作空间') 
hold on  % 保持当前图像不被刷新，准备接受后续绘制的图像，多图共存
grid on  % 画图时添加网格线
% axis([-20 20 -20 20])