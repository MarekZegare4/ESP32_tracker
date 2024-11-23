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

var_prediction = 1;
var_measurement = 5; % m variance
var_speed = 1;
v = 0;
heading = 0;
for i = 1:10000
    data = str2num(readline(serial));
    y = [y, data(1)];
    % =============
    td = 0.2;
    heading_meas = y(end);
    heading = heading + td * v
    v_new = v + var_speed * td * td;
    L = v_new + var_measurement * td * td;
    kalman_gain = v_new / L;
    heading = heading + kalman_gain * (heading_meas - heading);
    v = (1 - kalman_gain) * v_new;

    y_k = [y_k, heading];
    % =============
    x = [x, i];
    refreshdata;
    drawnow
    %pause
end


clear serial

