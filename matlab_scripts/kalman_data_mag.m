serialportlist("available");

serial = serialport("/dev/cu.usbserial-V001",115200);
setDTR(serial, false)
setRTS(serial, false)

y = [];
y_k = []
x = [];

figure(1)
p = scatter(y, x);
hold on
pp = scatter(y_k, x);
hold off

p.XDataSource = 'x';
p.YDataSource = 'y';

pp.XDataSource = 'x';
pp.YDataSource = 'y_k';

linkdata on

p_m = 180;
q = 0.004;
r = 0.1452; % m variance
x_p = 0;
p_p = p_m + q;

k = 0;
for i = 1:10000
    data = str2num(readline(serial))
    y = [y, data(1)];
    % =============
    p_p = p_p + q * r;
    k = p_p / (p_p + r);
    x_p = x_p + k * (y(i) - x_p);
    y_k = [y_k, x_p];
    p_p = (1 - k) * p_p;
    
    % =============
    x = [x, i];
    refreshdata;
    drawnow
    %pause
end


clear serial

