import sys
sys.path.append("/apollo/")
sys.path.append("/apollo/bazel-bin/")
sys.path.append("/apollo/cyber/")


def filter_log_by_keyword(log_file_path, output_file_path, keyword):
    # 打开日志文件并逐行读取，指定编码为 'utf-8'
    with open(log_file_path, 'r') as log_file:
        lines = log_file.readlines()
    with open(output_file_path, 'w') as output_file:
        for line in lines:
            line = line.replace("\n", " ")
            data = line.split(" ")
            # 检查关键词是否在行中
            #if "debug" in data or "debug_pub_scan" in data:
            if "debug_pub_scan" in data or "count_num" in data:
                output_file.write(line[60:])
                output_file.write("\n")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: %s <filename> <fbags>" % sys.argv[0])
        sys.exit(0)

    filename = sys.argv[1]
    outfile = sys.argv[2]
    keyword = "debug"
    # 运行函数
    filter_log_by_keyword(filename, outfile, keyword)