function y = isexist(A,b)
%该函数用来判断线性方程组Ax = b 的解的存在性
%若方程组无解，则返回0，若有唯一解则返回1，若有无穷多解则返回inf
[m,n] = size(A);
[mb,nb] = size(A);
if m~=mb
    error('输入有误');
    return;
end
r = rank(A);
s = rank([A,b]);
if r == s && r == n
    y = 1;
elseif r == s && r<n
    y = inf;
else
    y = 0;
end