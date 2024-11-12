% serialportlist("available");

serial = serialport("/dev/cu.usbserial-V001",115200);
setDTR(serial, false)
setRTS(serial, false)

x = [];
y = [];
z = [];

figure(1)
p = scatter3(x, y, z, "filled");
% xlim([-8000 8000]);
% ylim([-8000 8000]);
% zlim([-8000 8000]);

p.XDataSource = 'x';
p.YDataSource = 'y';
p.ZDataSource = 'z';

linkdata on

for i = 1:500
    data = str2num(readline(serial))
    x = [x, data(1)];
    y = [y, data(2)];
    z = [z, data(3)];
    refreshdata
    drawnow
end

save("x.mat", "x");
save("y.mat", "y");
save("z.mat", "z");

clear serial