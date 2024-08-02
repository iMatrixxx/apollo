import numpy as np  
import matplotlib.pyplot as plt  
from mpl_toolkits.mplot3d import Axes3D  
  
def read_point_cloud(filename):  
    """  
    从文件中读取点云数据。  
    假设每行格式为 "x y intensity"  
    """  
    points = []  
    intensities = []  
    with open(filename, 'r') as file:  
        for line in file:  
            x, y, z, intensity = map(float, line.split(" "))  
            points.append([x, y])  
            intensities.append(intensity)  
    points = np.array(points)  
    intensities = np.array(intensities)  
    return points, intensities  
  
def plot_2d_point_cloud(points, intensities, cmap='viridis'):  
    """  
    在2D平面上绘制点云，并使用强度值作为颜色映射。  
    """  
    plt.figure(figsize=(10, 8))  
      
    # 标准化强度值，以便它们可以映射到颜色图上  
    normalized_intensities = (intensities - intensities.min()) / (intensities.max() - intensities.min())  
      
    # 绘制点云  
    sc = plt.scatter(points[:, 0], points[:, 1], c=normalized_intensities, cmap=cmap, s=2)  
      
    # 添加颜色条  
    plt.colorbar(sc, label='Intensity')  
      
    # 设置坐标轴标签  
    plt.xlabel('X')  
    plt.ylabel('Y')  
      
    plt.title('2D LiDAR Point Cloud')  
    plt.grid(True)  
      
      
    # plt.show()
    plt.savefig("my_data/point_cloud.png", dpi=300)
    plt.close() 
  
# 读取点云数据  
points, intensities = read_point_cloud('my_data/pointcloud.txt')  
  
# 绘制点云  
plot_2d_point_cloud(points, intensities)