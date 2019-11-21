% ʱ�䣺2019��11��21��
% ���ܣ�3-RPS����ƽ̨λ��������⣨��֪�����˳�����ƽ̨λ�˲���)
% �㷨��������ʼλ�ˣ������ݶ�������ѭ��


%% Initialization
% initial_data =load('F:\����\Project\���ڷ�ˢ�о�\������\��ҵ����\д���ز�\��5��\΢�����˵��˲���ʵ��\POSE_MOTOR_ENCODER.mat',data1)
% data = (initial_data)';
L_len(1,:) = 125 + 100*data2(1,:);
L_len(2,:) = 125 + 100*data2(3,:);
L_len(3,:) = 125 + 100*data2(2,:);
data_begin =2150;
data_end = 4150;

%% Calculation Loop
i = 1;
for j = data_begin:data_end
    
    L_actual = L_len(:,j); % �����δ֪λ�˶�Ӧ�������˳�����֪��
    
    % solution #01
    L_origin = [sum(L_len(:,j))/3; sum(L_len(:,j))/3; sum(L_len(:,j))/3]; % ��ֵ�����������ʼλ�ˣ���ƽ̨ƽ�У��¶�Ӧ�������˳�����֪��
%     L_theory = L_origin;
    P_origin = [ sum(L_len(:,j))/3; 0; 0 ]; % ��׼λ�ˣ��Զ���P = [ ��ƽ̨���ĵ� Z����ֵ�� ���� ��] )
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
legend('ʵ��','����', 'Location','northeast');
% set(gca, 'YLim', [-2.0 1], 'YTick', [-2.0:0.2:1]);
% save Posture_actual.mat, P_actual
% xlswrite('C:\Users\Administrator\Documents\MATLAB\PKM Forward Kinematics\3-RPS\Posture_actual.xlsx', P_actual, '��ƽ̨ʵ��λ��')

title('���������3-RPS�������������ռ�') 
hold on  % ���ֵ�ǰͼ�񲻱�ˢ�£�׼�����ܺ������Ƶ�ͼ�񣬶�ͼ����
grid on  % ��ͼʱ���������
% axis([-20 20 -20 20])