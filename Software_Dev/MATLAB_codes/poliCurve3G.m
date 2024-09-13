function [a] = poliCurve3G(p_start, p_end, t_f)
% a0 =  p_start;
% a1 = 0;
% a2 = (3/(t_f*t_f))*(p_end - p_start);
% a3 = -(2/(t_f*t_f*3))*(p_end - p_start);
% a = {a0,a1,a2,a3};
a(1) =  p_start;
a(2) = 0;
a(3) = (3/(t_f*t_f))*(p_end - p_start);
a(4) = -(2/(t_f*t_f*3))*(p_end - p_start);
