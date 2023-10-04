clear all
clc

w = [-40,-30,-20,-10,10,20,30,40];

s = [0, 0, 0 , 0, 1, 0, 0, 0];

s_complemento = zeros(1,size(s,2));

for i=1:size(s,2)
    s_complemento(i) = 1- s(i);
end

sw = s.*w;
sum_sw = sum(sw);
sum_s = sum(s);

sw_comp = s_complemento.*w;
sum_sw_comp = sum(sw_comp);
sum_s_comp = sum(s_complemento);

centro = sum_sw/sum_s

centro_comp = sum_sw_comp/sum_s



