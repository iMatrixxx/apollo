import sys
sys.path.append("/apollo/")
sys.path.append("/apollo/bazel-bin/")
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2
import matplotlib.pyplot as plt

class ControlInfo(object):
    def __init__(self):
        self.sequece_number = [] #序列号
        self.matched_point = {"xs":[], "ys":[]} #匹配点
        self.ref_point = {"xs":[], "ys":[]} #参考点
        self.preview_ref_point = {"xs":[], "ys":[]} #预瞄参考点坐标

        self.target_angle = [] #目标角度
        self.steer_angle_feedforward = []
        self.steer_angle_feedback = []
        self.curvature = [] #曲率

        self.lateral_error = [] #横向误差
        self.heading_error = [] #航向误差
        self.heading = [] #航向角
        self.ref_heading = [] #参考航向角


    def __callback_control(self, entity): 
        self.sequece_number.append(entity.header.sequence_num)
        x = entity.debug.simple_mpc_debug.current_matched_point.path_point.x
        y = entity.debug.simple_mpc_debug.current_matched_point.path_point.y
        self.matched_point["xs"].append(x)
        self.matched_point["ys"].append(y)

        x = entity.debug.simple_mpc_debug.current_reference_point.path_point.x
        y = entity.debug.simple_mpc_debug.current_reference_point.path_point.y
        self.ref_point["xs"].append(x)
        self.ref_point["ys"].append(y)

        x = entity.debug.simple_mpc_debug.preview_reference_point.path_point.x
        y = entity.debug.simple_mpc_debug.preview_reference_point.path_point.y
        self.preview_ref_point["xs"].append(x)
        self.preview_ref_point["ys"].append(y)

        self.target_angle.append(entity.steering_target)
        self.steer_angle_feedforward.append(entity.debug.simple_mpc_debug.steer_angle_feedforward)
        self.steer_angle_feedback.append(entity.debug.simple_mpc_debug.steer_angle_feedback)
        self.curvature.append(entity.debug.simple_mpc_debug.curvature)

        self.lateral_error.append(entity.debug.simple_mpc_debug.lateral_error)
        self.heading_error.append(entity.debug.simple_mpc_debug.heading_error)
        self.heading.append(entity.debug.simple_mpc_debug.heading)
        self.ref_heading.append(entity.debug.simple_mpc_debug.ref_heading)


    def read_record_file(self,file_path):
        reader = RecordReader(file_path)
        print("Begin reading the file: ", file_path)
        for msg in reader.read_messages():
            if msg.topic == "/apollo/control":
                control_cmd = control_cmd_pb2.ControlCommand()
                control_cmd.ParseFromString(msg.message)
                self.__callback_control(control_cmd)
        print("Done reading the file: ", file_path)

    def plot_point(self):
        plt.figure(figsize=(15, 8))
        #y轴坐标按照实际值显示
        plt.scatter(self.sequece_number,self.matched_point["ys"], color="r",  alpha=0.5, s = 5)
        plt.scatter(self.sequece_number,self.ref_point["ys"], color="b",  alpha=0.5, s = 5)
        plt.scatter(self.sequece_number,self.preview_ref_point["ys"], color="g",  alpha=0.5, s = 5)
        # 获取当前的坐标轴
        ax = plt.gca()
        # 隐藏右侧和上侧的边框
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        # 设置y轴刻度为实际值
        plt.ticklabel_format(useOffset=False, style='plain', axis='y')
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("matched_point VS ref_point VS preview_ref_point")
        plt.legend(["matched_point", "ref_point", "preview_ref_point"], loc='upper right')
        plt.savefig("my_data/picture/control_point.png", dpi=300)
        plt.close()

    def plot_steering_target(self):
        fig, axs = plt.subplots(2, 2, sharex='all', figsize=(10, 2 * 3))
        axs[0, 0].plot(self.sequece_number, self.target_angle, "r-", lw=1.0, alpha=0.8)
        axs[0, 0].set_xlabel("Sequence(n)")
        axs[0, 0].set_ylabel("Target Angle Percent(%)")
        axs[0, 0].set_title("Steer Target Angle Percent")
        axs[0, 0].spines['right'].set_visible(False)
        axs[0, 0].spines['top'].set_visible(False)

        axs[0, 1].plot(self.sequece_number, self.steer_angle_feedforward, "r-", lw=1.0, alpha=0.8)
        axs[0, 1].set_xlabel("Sequence(n)")
        axs[0, 1].set_ylabel("SteerAngleFeedforward")
        axs[0, 1].set_title("Steer Angle Feedforward")
        axs[0, 1].spines['right'].set_visible(False)
        axs[0, 1].spines['top'].set_visible(False)

        axs[1, 0].plot(self.sequece_number, self.steer_angle_feedback, "r-", lw=1.0, alpha=0.8)
        axs[1, 0].set_xlabel("Sequence(n)")
        axs[1, 0].set_ylabel("SteerAngleFeedback")
        axs[1, 0].set_title("Steer Angle Feedback")
        axs[1, 0].spines['right'].set_visible(False)
        axs[1, 0].spines['top'].set_visible(False)  

        axs[1, 1].plot(self.sequece_number, self.curvature, "r-", lw=1.0, alpha=0.8)
        axs[1, 1].set_xlabel("Sequence(n)")
        axs[1, 1].set_ylabel("Curvature")
        axs[1, 1].set_title("Curvature")
        axs[1, 1].spines['right'].set_visible(False)
        axs[1, 1].spines['top'].set_visible(False)           
        
        plt.tight_layout()
        plt.savefig("my_data/picture/steering_angle.png", dpi=300) 
        plt.close()     

    def plot_error(self):
        fig, axs = plt.subplots(2, 2, sharex='all', figsize=(10, 2 * 3))
        axs[0, 0].plot(self.sequece_number, self.lateral_error, "r-", lw=1.0, alpha=0.8, label="Lateral Error")
        axs[0, 0].set_xlabel("Sequence(n)")
        axs[0, 0].set_ylabel("Lateral Error")
        axs[0, 0].set_title("Lateral Error")
        axs[0, 0].spines['right'].set_visible(False)
        axs[0, 0].spines['top'].set_visible(False)

        axs[0, 1].plot(self.sequece_number, self.heading_error, "r-", lw=1.0, alpha=0.8, label="Heading Error")
        axs[0, 1].set_xlabel("Sequence(n)")
        axs[0, 1].set_ylabel("Heading Error")
        axs[0, 1].set_title("Heading Error")
        axs[0, 1].spines['right'].set_visible(False)
        axs[0, 1].spines['top'].set_visible(False)

        axs[1, 0].plot(self.sequece_number, self.heading, "r-", lw=1.0, alpha=0.8, label="Heading")
        axs[1, 0].plot(self.sequece_number, self.ref_heading, "b-", lw=1.0, alpha=0.8, label="Ref Heading")
        axs[1, 0].set_xlabel("Sequence(n)")
        axs[1, 0].set_ylabel("Heading")
        axs[1, 0].set_title("Heading VS Ref Heading")
        axs[1, 0].spines['right'].set_visible(False)
        axs[1, 0].spines['top'].set_visible(False)
        axs[1, 0].legend(loc='best')          
        
        plt.tight_layout()
        plt.savefig("my_data/picture/error.png", dpi=300) 
        plt.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: %s <filename> <fbags>" % sys.argv[0])
        sys.exit(0)
    file_path = sys.argv[1]
    control_info = ControlInfo()
    control_info.read_record_file(file_path)
    control_info.plot_point()
    control_info.plot_steering_target()
    control_info.plot_error()
    print("Done!")



