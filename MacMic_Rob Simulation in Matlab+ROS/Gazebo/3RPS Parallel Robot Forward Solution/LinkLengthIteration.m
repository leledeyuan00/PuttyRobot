function [ P_actual, Jac_actual ] = LinkLengthIteration( L_actual, L_origin, P_origin, Jac_origin )
%% ReadMe
% Function : 3RPS Parallel Robot forward solution —— Numerical iterative method based on rod length approximation
% Time:  2019.11.21（Nov 11, 2019)
% Author: LiKang
% Parameter Description: 
%   L_actual   —— The rod length of the parallel mechanism at the current moment
%   L_origin   —— The bar length at the beginning of numerical iteration
%   Jac_origin —— The jacbian matrix at the beginning of numerical iteration
%   P_theory   —— The pose of the center point of the moving platform calculated in one iteration cycle( while loop )

%% Initialization
  a = 60; % 定平台上的转动副分布半径
  b = 60; % 动平台上的球铰副分布半径
  Rev_base = [ a, -a/2, -a/2 ; 
               0, (3^0.5)*a/2, -(3^0.5)*a/2 ; 
               0, 0, 0 ]; % 转动铰点在定坐标系下表示
  Sph_top = [ b, -b/2, -b/2 ; 
              0, (3^0.5)*b/2, -(3^0.5)*b/2 ; 
              0, 0, 0 ]; % 球铰点在动坐标系下表示
  Sph_base = []; % 球铰点在定坐标系下表示
  Ctop_base = []; %动平台中心点在定坐标系下的表示
  P_actual = []; % 程序求解的目标量，动平台相对定平台未知的位姿，如[160.7; 3 ;0]
  P_theory = []; % 理论求解量，是目标量的逼近值
  L_vec = []; % 驱动支链向量在定坐标系下的表示
  r = []; % 动平台中心点到球铰点的矢量在定坐标系下表示
  n = []; % 驱动杆轴向单位向量在定坐标系下表示
  Trans_t2b = [];  % 用XYZ欧拉角描述的上平台上的动坐标系相对底座上的定坐标系的旋转变换矩阵
  Jac_inv = []; % 由矢量构造法求得的工作空间向关节空间映射的逆解雅克比矩阵，大小为3*6
  Jac_e2r = []; % 动平台在固定坐标系下的六位速度与用欧拉角描述的六维速度之间的映射变换，大小为 6*6
  Jac_six2three = []; % 利用位姿参数约束方程，用动平台三维独立速度表示其六维速度，大小为6*3

%% Calculation Loop
  L_theory = L_origin;

  while ( ( abs( L_theory(1,1) - L_actual(1,1) ) >= 0.01 ) || ...
          ( abs( L_theory(2,1) - L_actual(2,1) ) >= 0.01 ) || ...
          ( abs( L_theory(3,1) - L_actual(3,1) ) >= 0.01 ) )
        %1
       P_theory = P_origin + Jac_origin * ( L_actual - L_origin );
       P_origin = P_theory;
       %2
       alpha = P_origin(2,1)*pi/180; % 动平台空间转动的欧拉角
       beta = P_origin(3,1)*pi/180;
       gamma = -atan(sin(alpha)*sin(beta)/(cos(alpha)+cos(beta)));
       Ctop_base = [ b*(cos(beta)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma)-cos(alpha)*cos(gamma))/2; b*cos(beta)*sin(gamma); P_origin(1,1) ];
       Trans_t2b = [ cos(beta)*cos(gamma),                                  -cos(beta)*sin(gamma),                                  sin(beta); 
                     sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma), -sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), -sin(alpha)*cos(beta);
                     -cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma), cos(alpha)*cos(beta) ];
       Sph_base = Trans_t2b * Sph_top + [ Ctop_base, Ctop_base, Ctop_base ] ;
       L_vec = Sph_base - Rev_base; % 驱动杆向量
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

