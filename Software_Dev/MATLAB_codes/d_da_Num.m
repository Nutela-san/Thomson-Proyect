function [result] = d_da_Num(curve,curve_da)

if size(curve) ~= size(curve_da)
    result = 0;
else
    result = zeros(size(curve_da));
    for i=1:size(curve_da,2)-1
        result(i) = (curve(i+1)- curve(i))/ (curve_da(i+1)- curve_da(i));
    end
    result(size(curve_da,2)) = result(size(curve_da,2)-1); 
end