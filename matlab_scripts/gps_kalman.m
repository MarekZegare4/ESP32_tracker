rng(0,'twister');

a = 5;
b = 0;
y = a.*randn(1000,1) + b;
y = []
 for i = 1:100
     d = (a.*randn(10, 1) + i)';
     y = [y, d];
end

x_m = y(1);
p_m = 180;
q = 0.001;
r = 0.1452; % m variance
x_p = x_m;
p_p = p_m + q;

z = 0;
k = 0;
r = 1;
y_kalman = [];
for i = 1:length(y)
    % z = heading(i);
    % k = p_p /(p_p + r);
    % x_p = x_p + k * (z - x_p);
    % y_kalman = [y_kalman, x_p];
    % p_p = (1 - k) * p_p;
    % 
    % p_p = p_p + q^2;

    x_p = x_p + q * y(i);
    p_p = p_p + q * r;
    k = p_p / (p_p + r);
    x_p = x_p + k * (y(i) - x_p);
    y_kalman = [y_kalman, x_p];
    p_p = (1 - k) * p_p;
end

figure(1)
plot(y);
hold on;
plot(y_kalman);
hold off;