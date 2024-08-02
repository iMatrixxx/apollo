import sys
sys.path.append("/apollo/")
sys.path.append("/apollo/bazel-bin/")
sys.path.append("/apollo/cyber/")
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.sensor_msgs import pointcloud_pb2 

if len(sys.argv) < 3:
    print("Usage: %s <filename> <fbags>" % sys.argv[0])
    sys.exit(0)

filename = sys.argv[1]
fbags = sys.argv[2:]

with open(filename, 'w') as f:
    for fbag in fbags:
        reader = RecordReader(fbag)
        for msg in reader.read_messages():
            if msg.topic == "/apollo/sensor/lslidarCH64/PointCloud2":
                pc = pointcloud_pb2.PointCloud()
                pc.ParseFromString(msg.message)
                for point in pc.point:
                    x = point.x
                    y = point.y
                    intensity = pc.point[0].intensity
                    f.write(str(x) + " " + str(y) + " "+str(0)+" " +str(intensity)+"\n")
f.close()
print("File written to: %s" % filename)