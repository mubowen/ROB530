clc
clear 
close all
%--------------------------------------------
%Task2
fig=1;
%A
%10000 sample
n=10000;
%Mean of range and bearing 
mu=[10.0, 0];
%Create noise
noise=0;
%noise=0.12;
%Standard Deviation of range and bearing
sigma=[0.5 ,0.25-noise];
%Covariance matrix of r and theta
Cov_d=[sigma(1)^2 0;0 sigma(2)^2];
%Sample data from univariate Gaussian
sensor_f=mu+sigma.*randn(n,2);
r=sensor_f(1:end,1);
theta=sensor_f(1:end,2);
%Cartesian coordinate transfer
Car_d=@(r,theta) deal(r.*cos(theta),r.*sin(theta));
[x,y]=Car_d(r,theta);

%B
%Jacobian of f(x) evaluated at mean
syms J r1 theta1
J(r1,theta1)=jacobian([r1*cos(theta1),r1*sin(theta1)],[r1,theta1])
J_x0=double(J(mu(1),mu(2)));

%Linearlization by using first order Taylor
[x0,y0]=Car_d(mu(1),mu(2)); %f(x0)
linear_f=[x0,y0]'+J_x0*(sensor_f'-mu');
linear_x=linear_f(1,1:end);
linear_y=linear_f(2,1:end);
%plot(linear_x,linear_y,'o')
Cov_xy_lin=J_x0*Cov_d*J_x0';
Cov_rtheta_meas=cov(r,theta);
%Actual Cov
Cov_xy_meas=cov(x,y);

%C
% Linearlized mean and cov
mu_L=[mean(linear_x), mean(linear_y)];
%Actual mean
mu_r=[mean(r),mean(theta)];
mu_x=[mean(x),mean(y)];
%Plot in r and theta

figure(fig)
plot(r,theta,'o')
%axis([8 12 -1.75 1.75])
title('Measure in sensor frame with prtheta=0')
axis('equal')
xlabel('range(m)')
ylabel('bearing(rad)')
hold on
draw_ellipse(mu',Cov_rtheta_meas,1,'r')
draw_ellipse(mu',Cov_rtheta_meas,4,'r')
draw_ellipse(mu',Cov_rtheta_meas,9,'r')
hold off 

%Plot in Cartesian Coordinate

fig=fig+1;
figure(fig)
plot(x,y,'o')
title('Linearlized Measure in Cartesian Coordinate frame with prtheta=0')
axis('equal')
xlabel('x(m)')
ylabel('y(m)')
hold on 
draw_ellipse(mu_L',Cov_xy_lin,1,'r')
draw_ellipse(mu_L',Cov_xy_lin,4,'r')
draw_ellipse(mu_L',Cov_xy_lin,9,'r')
draw_ellipse(mu_x',Cov_xy_meas,1,'b')
draw_ellipse(mu_x',Cov_xy_meas,4,'b')
draw_ellipse(mu_x',Cov_xy_meas,9,'b')
hold off 
fig=fig+1;
%D
%Calculate Mahalanobus distance for r and theta
x_m=[r';theta'];
%x_m=[linear_x;linear_y];
u_m=[mean(r); mean(theta)];
%u_m=[mean(linear_x); mean(linear_y)];
Maha_dis=diag(sqrt((x_m-u_m)'*inv(Cov_rtheta_meas)*(x_m-u_m)));
count_1sigma=sum(Maha_dis<1);
count_2sigma=sum(Maha_dis<2);
count_3sigma=sum(Maha_dis<3);

%Calculate Mahalanobus distance for unlinear x and y
x_m_unl=[x';y'];
u_m_unl=[mean(x); mean(y)];
Maha_dis=diag(sqrt((x_m_unl-u_m_unl)'*inv(Cov_xy_meas)*(x_m_unl-u_m_unl)));
count_1sigma_unl=sum(Maha_dis<1);
count_2sigma_unl=sum(Maha_dis<2);
count_3sigma_unl=sum(Maha_dis<3);

%Calculate Mahalanobus distance for linear x and y
x_m_l=[linear_x;linear_y];
u_m_l=[mean(linear_x); mean(linear_y)];
Maha_dis=diag(sqrt((x_m_l-u_m_l)'*inv(Cov_xy_lin)*(x_m_l-u_m_l)));
count_1sigma_l=sum(Maha_dis<1);
count_2sigma_l=sum(Maha_dis<2);
count_3sigma_l=sum(Maha_dis<3);
%By adjust the noise that reduce the corvariance by 0.12
%it matched the theoretically predicted values

%F
prt=[0.1,0.5,0.9];

for i=1:length(prt)
    %reA
    %correlation coefficients Covariance matrix of r and theta
    Cov_nd=[sigma(1)^2, prt(i)*sigma(1)*sigma(2);
         prt(i)*sigma(1)*sigma(2),sigma(1)^2];
    %Sample data
    [L,p]=chol(Cov_nd);
    %New Measurement  
    sensor_L=mu'+L*randn(2,n);
    r_L=sensor_L(1,1:end)';
    theta_L=sensor_L(2,1:end)';
    %Cartesian coordinate transfer
    %Car_d=@(r,theta) deal(r.*cos(theta),r.*sin(theta));
    [x_L,y_L]=Car_d(r_L,theta_L);
    
    %reB
    
    %Linearlization by using first order Taylor
    %[x0,y0]=Car_d(mu(1),mu(2)); %f(x0)
    linear_L=[x0,y0]'+J_x0*(sensor_L-mu');
    linear_xL=linear_L(1,1:end);
    linear_yL=linear_L(2,1:end);
    Cov_xy_lin_L=J_x0*Cov_nd*J_x0';
    Cov_rtheta_meas_L=cov(r_L,theta_L);
    %Actual Cov
    Cov_xy_meas_L=cov(x_L,y_L);
    mu_x_L=[mean(x_L),mean(y_L)];
    mu_r_L=[mean(r_L),mean(theta_L)];
    mu_L_L=[mean(linear_xL),mean(linear_yL)];
    
    figure(fig)
    plot(r_L,theta_L,'o')
    %axis([6 15 -1.75 1.75])
    title(['Measure in sensor frame with p_rtheta=',num2str(prt(i))])
    axis('equal')   
    xlabel('range(m)')
    ylabel('bearing(rad)')
    hold on
    draw_ellipse(mu_r_L',Cov_rtheta_meas_L,1,'r')
    draw_ellipse(mu_r_L',Cov_rtheta_meas_L,4,'r')
    draw_ellipse(mu_r_L',Cov_rtheta_meas_L,9,'r')
    hold off 
    fig=fig+1;
    
    figure(fig)
    plot(x_L,y_L,'o')
    %axis([-6 15 -20 15])
    title(['Linearlized Measure in Cartesian Coordinate frame with p_rtheta=',num2str(prt(i))])
    axis('equal')
    xlabel('x(m)')
    ylabel('y(m)')
    hold on 
    draw_ellipse(mu_L_L',Cov_xy_lin_L,1,'r')
    draw_ellipse(mu_L_L',Cov_xy_lin_L,4,'r')
    draw_ellipse(mu_L_L',Cov_xy_lin_L,9,'r')
    draw_ellipse(mu_x_L',Cov_xy_meas_L,1,'b')
    draw_ellipse(mu_x_L',Cov_xy_meas_L,4,'b')
    draw_ellipse(mu_x_L',Cov_xy_meas_L,9,'b')
    hold off 
    fig=fig+1;
end
