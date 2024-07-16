import sys
sys.path.append("/apollo/")
sys.path.append("/apollo/bazel-bin/")
sys.path.append("/apollo/modules/")
import time
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.external_command_msgs import lane_follow_command_pb2
from modules.common_msgs.external_command_msgs import command_status_pb2

def main():
    """
    主函数
    """
    # 初始化Cyber
    cyber.init()
    node = cyber.Node("mock_routing_requester")
    sequence_num = 1

    routing_request = lane_follow_command_pb2.LaneFollowCommand()

    routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
    routing_request.header.module_name = '111'
    routing_request.header.sequence_num = sequence_num

    routing_request.is_start_pose_set = True

    sequence_num = sequence_num + 1

    #start 587680.24, 4141269.34, 1.40
    #end  587683.12, 4141468.09, 2.91
    
    # 从命令行选择起点坐标
    # start_heading = float(input("start_heading : "))
    # start_x = float(input("start_x : "))
    # start_y = float(input("start_y : "))
    start_heading = float(1.40)
    start_x = float(587680.24)
    start_y = float(4141269.34)

    # 从命令行选择终点坐标
    # end_heading = float(input("end_heading: "))
    # end_x = float(input("end_x : "))
    # end_y = float(input("end_y: "))
    end_heading = float(2.91)
    end_x = float(587683.12)
    end_y = float(4141468.09)

    # 添加路径起点
    waypoint = routing_request.way_point.add()
    waypoint.heading = start_heading
    waypoint.x = start_x
    waypoint.y = start_y

    # 设置终点坐标
    waypoint1 = routing_request.end_pose
    waypoint1.heading = end_heading
    waypoint1.x = end_x
    waypoint1.y = end_y

    # 创建Cyber writer
    writer = node.create_client('/apollo/external_command/lane_follow', lane_follow_command_pb2.LaneFollowCommand, command_status_pb2.CommandStatus)
    time.sleep(2.0)
    print("routing", routing_request)
    start = True
    while start:
        usr_input = input("input:")
        if usr_input == "lane":
            writer.send_request(routing_request)
        elif usr_input == "stop":
            start = False

if __name__ == '__main__':
    main()