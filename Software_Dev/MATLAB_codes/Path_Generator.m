clear all
clc 

x = [1, 5]
y = [1, 0]

t_step = 3;
t = 0:0.02:t_step;


ax = poliCurve3G(x(1),x(2), t_step);
ay = poliCurve3G(y(1),y(2), t_step);

% ax = poliCurve5G(x(1),x(2),0,0,0,0, t_step);
% ay = poliCurve5G(y(1),y(2),0,0,0,0, t_step);


px = zeros(size(t));
px_p = zeros(size(t));
px_pp = zeros(size(t));
py = zeros(size(t));
py_p = zeros(size(t));
py_pp = zeros(size(t));

for i = 1:size(t,2)
    px(i)= ax(1)+ ax(2)*t(i) + ax(3)*t(i)*t(i) + ax(4)*t(i)*t(i)*t(i);
    px_p(i) = ax(2) + 2*ax(3)*t(i)+ 3*ax(4)*t(i)*t(i);
    px_pp(i) = 2*ax(3)+ 6*ax(4)*t(i);
        
    py(i)= ay(1)+ ay(2)*t(i) + ay(3)*t(i)*t(i) + ay(4)*t(i)*t(i)*t(i);
    py_p(i) = ay(2) + 2*ay(3)*t(i)+ 3*ay(4)*t(i)*t(i);
    py_pp(i) = 2*ay(3)+ 6*ay(4)*t(i);

%     px(i) = ax(1)+ ax(2)*t(i) + ax(3)*t(i)*t(i) + ax(4)*t(i)*t(i)*t(i) + ax(5)*t(i)*t(i)*t(i)*t(i) + ax(6)*t(i)*t(i)*t(i)*t(i)*t(i);
%     px_p(i)= ax(2) + 2*ax(3)*t(i)+ 3*ax(4)*t(i)*t(i) + 4*ax(5)*t(i)*t(i)*t(i) + 5*ax(6)*t(i)*t(i)*t(i)*t(i); 
%     px_pp(i)= 2*ax(3)+ 6*ax(4)*t(i) + 12*ax(5)*t(i)*t(i) + 20*ax(6)*t(i)*t(i)*t(i);
% 
%     py(i) = ay(1)+ ay(2)*t(i) + ay(3)*t(i)*t(i) + ay(4)*t(i)*t(i)*t(i) + ay(5)*t(i)*t(i)*t(i)*t(i) + ay(6)*t(i)*t(i)*t(i)*t(i)*t(i);
%     py_p(i)= ay(2) + 2*ay(3)*t(i)+ 3*ay(4)*t(i)*t(i) + 4*ay(5)*t(i)*t(i)*t(i) + 5*ay(6)*t(i)*t(i)*t(i)*t(i); 
%     py_pp(i)= 2*ay(3)+ 6*ay(4)*t(i) + 12*ay(5)*t(i)*t(i) + 20*ay(6)*t(i)*t(i)*t(i);

end

% p_p = d_da_Num(p,t);

subplot(1,3,1)
hold on
plot(t,px)
plot(t,px_p)
plot(t,px_pp)
hold off

subplot(1,3,2)
hold on
plot(t,py)
plot(t,py_p)
plot(t,py_pp)
hold off

subplot(1,3,3)
plot(px,py,"magenta")

