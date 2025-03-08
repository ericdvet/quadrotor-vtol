data = load('data.mat');

% header = ["time", "prop1Thrust", "prop2Thrust", "prop3Thrust", "prop4Thrust", "px", "py", "pz", "vx", "vy", "vz", "roll", "pitch", "yaw"];
csvwrite('data.csv', [data.data.Time data.data.Data])