function Force = ATISensor()

obj = py.importlib.import_module('NetFT');

sensor = obj.Sensor("192.168.3.1");

init_data = sensor.getMeasurement();
matlabArray = double(init_data);
numRows = 6;
numCols = 1;
matlabMatrix = reshape(matlabArray, numRows, numCols)/10e5;


RealmatlabMatrix = [matlabMatrix(1); matlabMatrix(2); matlabMatrix(3)];

Force = RealmatlabMatrix;

end









