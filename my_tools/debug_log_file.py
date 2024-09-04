import sys
sys.path.append("/apollo/")
sys.path.append("/apollo/bazel-bin/")
sys.path.append("/apollo/cyber/")


def DebugLogfile(log_file_path, output_file_path, keyword):
    # 打开日志文件并逐行读取，指定编码为 'utf-8'
    with open(log_file_path, 'r') as log_file:
        lines = log_file.readlines()
    with open(output_file_path, 'w') as output_file:
        for line in lines:
            line = line.replace("\n", " ")
            data = line.split(" ")
            for key in keyword:
                if key in data:
                    output_file.write(line[60:])
                    output_file.write("\n")
        

def CallFunction(modulename, filename):
    if modulename == "lidar":
        keyword = ["debug_pub_scan", "count_num"]
        output_file_path = "my_data/log_debug_file_txt/debug_lidar.txt"
        
    elif modulename == "slam":
        keyword = ["scan_rangs_size", "gsp_laser_beam_count_"]
        output_file_path = "my_data/log_debug_file_txt/debug_slam_gmapping.txt"

    DebugLogfile(filename, output_file_path, keyword)
        
    

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: %s <filename> <fbags>" % sys.argv[0])
        sys.exit(0)

    filename = sys.argv[1]
    modulename = sys.argv[2]
    
    CallFunction(modulename, filename)