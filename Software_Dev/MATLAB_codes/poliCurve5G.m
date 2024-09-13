function [a] = poliCurve5G(p_i,p_f, v_i, v_f, a_i, a_f, t_f)
a(1) = p_i;
a(2) = v_i;
a(3) = a_f/2;
a(4) = (20*p_f - 20*p_i -(8*v_f+12*v_i)*t_f -(3*a_i - a_f)*t_f*t_f)/(2*t_f*t_f*t_f);
a(5) = (30*p_i - 30*p_f +(14*v_f+16*v_i)*t_f +(3*a_i - 2*a_f)*t_f*t_f)/ (2*t_f*t_f*t_f*t_f);
a(6) = (12*p_f - 12*p_i -(6*v_f+6*v_i)*t_f +(a_i - a_f)*t_f*t_f)/(2*t_f*t_f*t_f*t_f*t_f);
