serialportlist("available");

serial = serialport("/dev/cu.usbserial-V001",115200);
setDTR(serial, false)
setRTS(serial, false)

pitch = [];
roll = [];
yaw = [];
x = [];

figure(1)
p = scatter(pitch, x);
hold on
pp = scatter(roll, x);
ppp = scatter(yaw, x)
legend("Pitch", "Roll", "Yaw");
hold off


p.XDataSource = 'x';
p.YDataSource = 'pitch';

pp.XDataSource = 'x';
pp.YDataSource = 'roll';

ppp.XDataSource = 'x';
ppp.YDataSource = 'yaw';

linkdata on

for i = 1:1000
    data = str2num(readline(serial))
    pitch = [pitch, data(1)];
    roll = [roll, data(2)];
    yaw = [yaw, data(3)];
    
    x = [x, i];
    refreshdata;
    drawnow
    %pause
end


clear serial

