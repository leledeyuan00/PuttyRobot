function [ P_actual, Jac_actual ] = LinkLengthIteration( L_actual, L_origin, P_origin, Jac_origin )
%% ReadMe
% Function : 3RPS Parallel Robot forward solution ���� Numerical iterative method based on rod length approximation
% Time:  2019.11.21��Nov 11, 2019)
% Author: LiKang
% Parameter Description: 
%   L_actual   ���� The rod length of the parallel mechanism at the current moment
%   L_origin   ���� The bar length at the beginning of numerical iteration
%   Jac_origin ���� The jacbian matrix at the beginning of numerical iteration
%   P_theory   ���� The pose of the center point of the moving platform calculated in one iteration cycle( while loop )

%% Initialization
  a = 60; % ��ƽ̨�ϵ�ת�����ֲ��뾶
  b = 60; % ��ƽ̨�ϵ���¸��ֲ��뾶
  Rev_base = [ a, -a/2, -a/2 ; 
               0, (3^0.5)*a/2, -(3^0.5)*a/2 ; 
               0, 0, 0 ]; % ת���µ��ڶ�����ϵ�±�ʾ
  Sph_top = [ b, -b/2, -b/2 ; 
              0, (3^0.5)*b/2, -(3^0.5)*b/2 ; 
              0, 0, 0 ]; % ��µ��ڶ�����ϵ�±�ʾ
  Sph_base = []; % ��µ��ڶ�����ϵ�±�ʾ
  Ctop_base = []; %��ƽ̨���ĵ��ڶ�����ϵ�µı�ʾ
  P_actual = []; % ��������Ŀ��������ƽ̨��Զ�ƽ̨δ֪��λ�ˣ���[160.7; 3 ;0]
  P_theory = []; % �������������Ŀ�����ıƽ�ֵ
  L_vec = []; % ����֧�������ڶ�����ϵ�µı�ʾ
  r = []; % ��ƽ̨���ĵ㵽��µ��ʸ���ڶ�����ϵ�±�ʾ
  n = []; % ����������λ�����ڶ�����ϵ�±�ʾ
  Trans_t2b = [];  % ��XYZŷ������������ƽ̨�ϵĶ�����ϵ��Ե����ϵĶ�����ϵ����ת�任����
  Jac_inv = []; % ��ʸ�����취��õĹ����ռ���ؽڿռ�ӳ�������ſ˱Ⱦ��󣬴�СΪ3*6
  Jac_e2r = []; % ��ƽ̨�ڹ̶�����ϵ�µ���λ�ٶ�����ŷ������������ά�ٶ�֮���ӳ��任����СΪ 6*6
  Jac_six2three = []; % ����λ�˲���Լ�����̣��ö�ƽ̨��ά�����ٶȱ�ʾ����ά�ٶȣ���СΪ6*3

%% Calculation Loop
  L_theory = L_origin;

  while ( ( abs( L_theory(1,1) - L_actual(1,1) ) >= 0.01 ) || ...
          ( abs( L_theory(2,1) - L_actual(2,1) ) >= 0.01 ) || ...
          ( abs( L_theory(3,1) - L_actual(3,1) ) >= 0.01 ) )
        %1
       P_theory = P_origin + Jac_origin * ( L_actual - L_origin );
       P_origin = P_theory;
       %2
       alpha = P_origin(2,1)*pi/180; % ��ƽ̨�ռ�ת����ŷ����
       beta = P_origin(3,1)*pi/180;
       gamma = -atan(sin(alpha)*sin(beta)/(cos(alpha)+cos(beta)));
       Ctop_base = [ b*(cos(beta)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma)-cos(alpha)*cos(gamma))/2; b*cos(beta)*sin(gamma); P_origin(1,1) ];
       Trans_t2b = [ cos(beta)*cos(gamma),                                  -cos(beta)*sin(gamma),                                  sin(beta); 
                     sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma), -sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), -sin(alpha)*cos(beta);
                     -cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma), cos(alpha)*cos(beta) ];
       Sph_base = Trans_t2b * Sph_top + [ Ctop_base, Ctop_base, Ctop_base ] ;
       L_vec = Sph_base - Rev_base; % ����������
       L_theory = [ sqrt( L_vec(1,1)^2 + L_vec(2,1)^2 + L_vec(3,1)^2 ); sqrt( L_vec(1,2)^2 + L_vec(2,2)^2 + L_vec(3,2)^2 ); sqrt( L_vec(1,3)^2 + L_vec(2,3)^2 + L_vec(3,3)^2 ) ];
       L_origin = L_theory;
       %3
       r = Trans_t2b * Sph_top;
       n(:,1) = ( Sph_base(:,1) - Rev_base(:,1) ) / norm( Sph_base(:,1) - Rev_base(:,1) );
       n(:,2) = ( Sph_base(:,2) - Rev_base(:,2) ) / norm( Sph_base(:,2) - Rev_base(:,2) );
       n(:,3) = ( Sph_base(:,3) - Rev_base(:,3) ) / norm( Sph_base(:,3) - Rev_base(:,3) );
       Jac_inv = ( [ n(:,1), n(:,2), n(:,3); cross(r(:,1),n(:,1)), cross(r(:,2),n(:,2)), cross(r(:,3),n(:,3)) ] )';
       Jac_e2r = [1, 0, 0, 0, 0, 0;
                  0, 1, 0, 0, 0, 0;
                  0, 0, 1, 0, 0, 0;
                  0, 0, 0, 1, 0, sin(beta);
                  0, 0, 0, 0, cos(alpha), -sin(alpha)*cos(beta);
                  0, 0, 0, 0, sin(alpha), cos(alpha)*cos(beta) ];
       Jac_six2three = [ 0, 0.5*b*( sin(alpha)*cos(beta)*cos(gamma)*(cos(beta) + cos(alpha)) + sin(beta)*cos(beta)*sin(gamma)*(1 + (cos(alpha))^2) ) / (1+cos(alpha)*cos(beta)), ...
                            0.5*b*( 2*sin(alpha)*cos(beta)*sin(gamma) - sin(beta)*cos(gamma)*(1 + cos(alpha)*cos(beta) + (sin(alpha))^2) - sin(alpha)*cos(alpha)*sin(gamma)*((sin(beta))^2)) / (1+cos(alpha)*cos(beta));
                         0, -b*sin(beta)*cos(beta)*cos(gamma) / (1+cos(alpha)*cos(beta)), -b*(1 + sin(beta)*sin(gamma)*(1 + cos(alpha)*cos(beta)) + sin(alpha)*cos(beta)*cos(gamma)) / (1+cos(alpha)*cos(beta));
                         1, 0, 0;
                         0, 1, 0;
                         0, 0, 1;
                         0, -sin(beta) / (1+cos(alpha)*cos(beta)), -sin(alpha) /(1+cos(alpha)*cos(beta)) ];
       Jac_origin = inv( Jac_inv * Jac_e2r * Jac_six2three );  
  end 
       Jac_actual = Jac_origin;
       P_actual = P_theory;
       
end

