% https://teslabs.com/articles/magnetometer-calibration/

D = [x.^2; y.^2; z.^2; 2*y.*z; 2*x.*z; 2*x.*y; 2*x; 2*y; 2*z; ones(1, length(x))];

C = [-1 1 1 0 0 0;
     1 -1 1 0 0 0;
     1 1 -1 0 0 0;
     0 0 0 -4 0 0;
     0 0 0 0 -4 0;
     0 0 0 0 0 -4;];

S = D*D';
S_11 = S(1:6, 1:6);
S_12 = S(1:6, 7:end);
S_21 = S(7:end, 1:6);
S_22 = S(7:end, 7:end);

E = inv(C)*(S_11 - (S_12 * (inv(S_22) * S_12')));

[E_v, E_w] = eig(E, "vector");


[argvalue, argmax] = max(E_w);

v_1 = E_v(:, argmax);

if v_1(1) < 0
     v_1 = -v_1;
end

v_2 = (-inv(S_22) * S_21) * v_1;

M = [v_1(1) v_1(6) v_1(5);
    v_1(6) v_1(2) v_1(4);
    v_1(5) v_1(4) v_1(3)];

n = [v_2(1);
     v_2(2);
     v_2(3)];

d = v_2(4);

M_1 = inv(M);
b = -M_1 * n;

r = mean([abs(max(x) - min(x))/2 abs(max(y)-min(y))/2 abs(max(z) - min(z))/2]);
%r = 9.81;

A_1 = real(r / (sqrt(n' * (M_1 * n) - d)) * sqrtm(M));

XYZ = [x;
       y;
       z];

% Przekształcone punkty
XYZ_n = A_1 * (XYZ - b);

% Wykres



[xs, ys, zs] = sphere;
xs = xs * r;
ys = ys * r;
zs = zs * r;

x_offset = (max(XYZ(1,:)) + min(XYZ(1,:)))/2;
y_offset = (max(XYZ(2,:)) + min(XYZ(2,:)))/2;
z_offset = (max(XYZ(3,:)) + min(XYZ(3,:)))/2;

figure(1)
scatter3(XYZ(1,:), XYZ(2,:), XYZ(3,:));
hold on
scatter3(XYZ(1,:) - x_offset, XYZ(2,:)- y_offset, XYZ(3,:)- z_offset);
scatter3(XYZ_n(1,:), XYZ_n(2,:), XYZ_n(3,:), "filled");
surf(xs, ys, zs, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
axis equal
hold off
legend("Raw data", "Raw data with offset", "Corrected data");
