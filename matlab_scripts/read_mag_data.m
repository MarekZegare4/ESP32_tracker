function read_mag_data(src, ~)

% Read the ASCII data from the serialport object.
data = readline(src)
%strings_data = strsplit(data, ",")

% % If over maxDataPoints points have been collected from the Arduino, switch off the
% % callbacks and plot the data, starting from the second point. 
% if src.UserData.Count > maxDataPoints
%     configureCallback(src, "off");
%     plot(src.UserData.Data(2:end));
% end
end