%%
%function to get deriviate
function dTf = get_coefficient(f0, a0, L)

dTf=zeros(3, 3);
for i = 1:3
    dTf(1, i) = -f0(i)*sin(a0(i));
    dTf(2, i) = f0(i)*cos(a0(i));
    dTf(3, i) = f0(i)*(L(i, 2)*sin(a0(i))+L(i, 1)*cos(a0(i)));
end

end