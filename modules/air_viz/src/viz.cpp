
#include "cyber/class_loader/class_loader.h"
#include "cyber/common/file.h"
#include "cyber/time/time.h"
#include "cyber/time/duration.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include <iostream>
#include <thread>
#include <GL/glut.h>
#include "modules/air_viz/include/viz.h"

using namespace apollo::AirViz;

void VizComponent::MouseButton(int button, int state, int x, int y) {
    if (is_space_pressed_.load()) {
        // 如果按下了 space 键，屏蔽鼠标操作
        return;
    }
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            is_dragging_ = true;
            last_mouse_x_ = x;
            last_mouse_y_ = y;
        } else {
            is_dragging_ = false;
        }
    }

    if (button == GLUT_MIDDLE_BUTTON) {
        if (state == GLUT_DOWN) {
            is_panning_ = true;
            last_mouse_x_ = x;
            last_mouse_y_ = y;
        } else {
            is_panning_ = false;
        }
    }

    if (button == 3 || button == 4) { // 滚轮
        if (button == 3) { // 滚轮向上滚动
            zoom_ *= 1.1f;
        } else if (button == 4) { // 滚轮向下滚动
            zoom_ /= 1.1f;
        }
        glutPostRedisplay();  // 请求重绘
    }
}

void VizComponent::MouseMotion(int x, int y) {
    if (is_space_pressed_.load()) {
        // 如果按下了 space 键，屏蔽鼠标操作
        return;
    }

    if (is_dragging_) {
        rotation_x_ += (y - last_mouse_y_);
        rotation_y_ += (x - last_mouse_x_);
    }

    if (is_panning_) {
        // 反转平移方向
        pan_x_ -= (x - last_mouse_x_) * 0.005f; // 鼠标向右平移场景向左移动
        pan_y_ += (y - last_mouse_y_) * 0.005f; // 鼠标向上平移场景向上移动
    }

    last_mouse_x_ = x;
    last_mouse_y_ = y;
    glutPostRedisplay(); // 请求重绘
}

void VizComponent::SetupGlobalFrame() {
    glLoadIdentity();  // 确保加载了单位矩阵

    if (is_space_pressed_.load()) {
        // 计算地图的中心点
        float map_center_x = width_ * resolution_ / 2.0f;
        float map_center_y = height_ * resolution_ / 2.0f;

        // 使用地图的最大边作为高度基础，并乘以一个系数以确保合适的视角
        float map_height = std::max(width_, height_) * resolution_; // 增加系数以俯瞰整个地图

        // 切换到从 Z 轴正方向俯瞰全局坐标系原点的视角
        gluLookAt(0,0, map_height,  // 摄像机位置（Z 轴高于地图中心）
                  0,0, 0.0f,       // 观察点（地图中心）
                  0.0f, 1.0f, 0.0f);                      // 上方向（Y 轴方向）
    } else {
        // 恢复到原始视角
        gluLookAt(pan_x_, pan_y_, 5.0 / zoom_,  // 使用缩放和平移来调整视野
                  pan_x_, pan_y_, 0.0f,         // 观察点
                  0.0f, 1.0f, 0.0f);            // 上方向
        glRotatef(rotation_x_, 0.5f, 0.0f, 0.0f);
        glRotatef(rotation_y_, 0.0f, 0.5f, 0.0f);
    }

    // 将 Eigen 矩阵转换为 OpenGL 矩阵（列主序）
    GLfloat gl_transform[16];
    Eigen::Map<Eigen::Matrix4f>(gl_transform, 4, 4) = global_frame_;

    // 将全局变换应用到 OpenGL 的模型视图矩阵
    glMultMatrixf(gl_transform);
    DrawCoordinateAxes();  // 调用函数绘制坐标轴

}

void VizComponent::DrawCoordinateAxes() {
    const float axis_length = 16.0f;  // 轴的长度
    const float axis_radius = 0.03f;  // 轴的半径
    const float arrow_length = 0.2f;  // 箭头的长度
    const float arrow_radius = 0.05f;  // 箭头的半径

    GLUquadric* quadric = gluNewQuadric();  // 创建一个新的 GLUquadric 对象，用于绘制圆柱和圆锥

    // 绘制X轴 (红色)
    glColor3f(1.0f, 0.0f, 0.0f);  // 红色
    glPushMatrix();
    glRotatef(90, 0.0f, 1.0f, 0.0f);  // 将圆柱体沿 X 轴对齐
    gluCylinder(quadric, axis_radius, axis_radius, axis_length, 32, 32);  // 绘制圆柱体
    glTranslatef(0.0f, 0.0f, axis_length);  // 移动到圆柱体的末端
    gluCylinder(quadric, arrow_radius, 0.0f, arrow_length, 32, 32);  // 绘制箭头
    glPopMatrix();

    // 绘制Y轴 (绿色)
    glColor3f(0.0f, 1.0f, 0.0f);  // 绿色
    glPushMatrix();
    glRotatef(-90, 1.0f, 0.0f, 0.0f);  // 将圆柱体沿 Y 轴对齐
    gluCylinder(quadric, axis_radius, axis_radius, axis_length, 32, 32);  // 绘制圆柱体
    glTranslatef(0.0f, 0.0f, axis_length);  // 移动到圆柱体的末端
    gluCylinder(quadric, arrow_radius, 0.0f, arrow_length, 32, 32);  // 绘制箭头
    glPopMatrix();

    // 绘制Z轴 (蓝色)
    glColor3f(0.0f, 0.0f, 1.0f);  // 蓝色
    glPushMatrix();
    gluCylinder(quadric, axis_radius, axis_radius, axis_length, 32, 32);  // 绘制圆柱体
    glTranslatef(0.0f, 0.0f, axis_length);  // 移动到圆柱体的末端
    gluCylinder(quadric, arrow_radius, 0.0f, arrow_length, 32, 32);  // 绘制箭头
    glPopMatrix();

    gluDeleteQuadric(quadric);  // 删除 GLUquadric 对象
}




void VizComponent::DrawElements() {
    // 使用 OpenGL 绘制网格
    glClearColor(0.156f, 0.156f, 0.156f, 1.0f);  // 深灰黑色背景
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPushMatrix();
    glLoadIdentity();
    
    // 设置全局坐标系
    SetupGlobalFrame();

    // 绘制地图
    DrawMap();

    // 绘制车辆
    DrawVehicle();
    // DrawWalls();

    glPopMatrix();
    glutSwapBuffers();  // 切换缓冲区以显示绘制结果
}




void VizComponent::DrawVehicle(){

}




void VizComponent::CreateMapTexture() {
    if (texture_id_ != 0) {
        glDeleteTextures(1, &texture_id_);  // 删除旧的纹理
    }

    glGenTextures(1, &texture_id_);
    glBindTexture(GL_TEXTURE_2D, texture_id_);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    std::vector<uint8_t> map_texture_data(width_ * height_ * 3);

    int grid_size_in_pixels = static_cast<int>(1.0f / resolution_);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int index = (y * width_ + x) * 3;

            if (x % grid_size_in_pixels == 0 || y % grid_size_in_pixels == 0) {
                map_texture_data[index] = 77;    // 深灰色的 RGB 值为 (77, 77, 77)
                map_texture_data[index + 1] = 77;
                map_texture_data[index + 2] = 77;
            } else {
                int8_t cell_value = map_mat_[y][x];
                float occupancy = cell_value / 100.0f;

                if (cell_value == -1) {
                    map_texture_data[index] = 255;  // 未知区域 - 白色
                    map_texture_data[index + 1] = 255;
                    map_texture_data[index + 2] = 255;
                } else if (occupancy > occupied_thresh_) {
                    map_texture_data[index] = 0;  // 障碍物 - 黑色
                    map_texture_data[index + 1] = 0;
                    map_texture_data[index + 2] = 0;
                } else if (occupancy < free_thresh_) {
                    map_texture_data[index] = 255;  // 空白区域 - 白色
                    map_texture_data[index + 1] = 255;
                    map_texture_data[index + 2] = 255;
                } else {
                    map_texture_data[index] = 128;  // 未决策区域 - 灰色
                    map_texture_data[index + 1] = 128;
                    map_texture_data[index + 2] = 128;
                }
            }
        }
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, &map_texture_data[0]);
    glBindTexture(GL_TEXTURE_2D, 0);  // 解除纹理绑定
}



void VizComponent::DrawMap() {
    if (!is_map_requested_) return;

    if (!is_map_cached_) {
        std::lock_guard<std::mutex> lock(map_access_mutex_);
        height_ = map_->meta_data().height();
        width_ = map_->meta_data().width();
        resolution_ = map_->meta_data().resolution();

        map_mat_.resize(height_, std::vector<int8_t>(width_, -1));  // 默认为 -1 表示未知区域
        const auto& data = map_->data();
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                int index = y * width_ + x;
                if (index < data.size()) {
                    int32_t value = data[index];
                    map_mat_[height_ - y - 1][x] = static_cast<int8_t>(value);
                } else {
                    AERROR << "Index out of bounds: " << index;
                }
            }
        }
        AINFO << "Map cast into std::vector<std::vector<int8_t>> map_mat_ and being memloaded";
        is_map_cached_ = true;

        // 创建纹理
        CreateMapTexture();
    }

    // 平移地图，使其中心对齐到原点
    float map_center_x = width_ * resolution_ / 2.0f;
    float map_center_y = height_ * resolution_ / 2.0f;

    glPushMatrix();
    glTranslatef(-map_center_x, -map_center_y, 0.0f);  // 平移地图，使其中心位于原点

    // 绘制地图
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glColor3f(1.0f, 1.0f, 1.0f);  // 确保绘制颜色为白色，不受其他设置影响

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex2f(0.0f, 0.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex2f(width_ * resolution_, 0.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex2f(width_ * resolution_, height_ * resolution_);
    glTexCoord2f(0.0f, 1.0f); glVertex2f(0.0f, height_ * resolution_);
    glEnd();
    glDisable(GL_TEXTURE_2D);

    // 绘制墙壁
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.3f, 0.3f, 0.3f, 0.5f);  // 深灰色，50%透明度
    float wall_height = 1.0f;  // 墙壁的高度

    glBegin(GL_QUADS);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int8_t cell_value = map_mat_[y][x];
            if (cell_value != -1 && cell_value > occupied_thresh_) {
                float x0 = x * resolution_;
                float y0 = y * resolution_;
                float x1 = x0 + resolution_;
                float y1 = y0 + resolution_;

                // 在Z轴方向延伸墙壁
                glVertex3f(x0, y0, 0.0f);  // 底面左下角
                glVertex3f(x1, y0, 0.0f);  // 底面右下角
                glVertex3f(x1, y0, wall_height);  // 顶面右下角
                glVertex3f(x0, y0, wall_height);  // 顶面左下角

                glVertex3f(x1, y0, 0.0f);  // 底面右下角
                glVertex3f(x1, y1, 0.0f);  // 底面右上角
                glVertex3f(x1, y1, wall_height);  // 顶面右上角
                glVertex3f(x1, y0, wall_height);  // 顶面右下角

                glVertex3f(x1, y1, 0.0f);  // 底面右上角
                glVertex3f(x0, y1, 0.0f);  // 底面左上角
                glVertex3f(x0, y1, wall_height);  // 顶面左上角
                glVertex3f(x1, y1, wall_height);  // 顶面右上角

                glVertex3f(x0, y1, 0.0f);  // 底面左上角
                glVertex3f(x0, y0, 0.0f);  // 底面左下角
                glVertex3f(x0, y0, wall_height);  // 顶面左下角
                glVertex3f(x0, y1, wall_height);  // 顶面左上角
            }
        }
    }
    glEnd();
    glDisable(GL_BLEND);

    glPopMatrix();
}


void VizComponent::InitializeOpenGL() {
    static bool glut_initialized = false;

    if (!glut_initialized) {
        int argc = 1;
        char* argv[1] = {(char*)"VizComponent"};
        glutInit(&argc, argv);
        glut_initialized = true;
    }

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1200, 800);
    glutCreateWindow("Apollo OpenGL Window");
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1200.0 / 800.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 设置鼠标回调函数
    glutMouseFunc([](int button, int state, int x, int y) {
        if (g_instance_) {
            g_instance_->MouseButton(button, state, x, y);
        }
    });

    glutMotionFunc([](int x, int y) {
        if (g_instance_) {
            g_instance_->MouseMotion(x, y);
        }
    });


    glutKeyboardFunc([](unsigned char key, int x, int y) {
        if (g_instance_) {
            g_instance_->KeyboardDown(key, x, y);  // 键盘按下事件
        }
    });

    glutKeyboardUpFunc([](unsigned char key, int x, int y) {
        if (g_instance_) {
            g_instance_->KeyboardUp(key, x, y);  // 键盘松开事件
        }
    });



    glutDisplayFunc(DrawElementCallBack);
    glutIdleFunc([]() {
        glutPostRedisplay();  // 在空闲时重绘
    });
}


void VizComponent::RenderLoop() {
    glutMainLoop();  // 启动GLUT事件循环
}



bool VizComponent::RequestMapData() {
    AINFO<<"Trying to Request Map Data";
    try {
        auto request = std::make_shared<LOAD_MAP_REQUEST>();
        request->set_map_url(map_file_path_);
        auto response = map_service_client_->SendRequest(request);
        if (response && response->load_status() == LOAD_MAP_RESPONSE::RESULT_SUCCESS) {
            OnMapDataReceived(response);
            AINFO << "Map data received and processed successfully.";
            return true; 
        } else {
            AERROR << "Failed to receive valid map data.";
            return false; 
        }
    } catch (const std::exception& e) {
        AERROR << "Exception caught during map data request: " << e.what();
        return false; 
    } catch (...) {
        AERROR << "Unknown exception caught during map data request.";
        return false; 
    }
}



void VizComponent::OnMapDataReceived(const std::shared_ptr<LOAD_MAP_RESPONSE>& response) {
        std::lock_guard<std::mutex> lock(map_access_mutex_);
        map_->CopyFrom(response->map());
        AINFO << "Map data successfully received and stored.";
}


void VizComponent::RoutineTask(){
    static auto last_map_request_time = cyber::Time::Now();
    while(true){
        auto current_time = cyber::Time::Now();
        if (!is_map_requested_ && (current_time - last_map_request_time) >= map_request_period_) {

            if (RequestMapData()) {
                AINFO << "Map data requested successfully.";
                is_map_requested_ = true;
            } else {
                AERROR << "Map data request failed.";
            }
            last_map_request_time = current_time;  // 更新上次请求时间
        }
        //....  
        cyber::Duration(0.5).Sleep();  // 控制任务循环的检查频率
    }
}


bool VizComponent::Init() {
    node_ = apollo::cyber::CreateNode("viz_node");

    // 调用封装的OpenGL初始化函数
    InitializeOpenGL();
    // 开启渲染线程
    render_thread_ = std::make_shared<std::thread>(std::bind(&VizComponent::RenderLoop, this));
    AINFO << "Rendering thread initialized";
    global_frame_ = Eigen::Matrix4f::Identity();

    map_ = std::make_shared<apollo::akman::OccupancyGrid>();
    map_service_client_ = node_->CreateClient<LOAD_MAP_REQUEST, LOAD_MAP_RESPONSE>(load_map_service_name_);

    try {
        routine_task_thread_ = std::make_shared<std::thread>(&VizComponent::RoutineTask, this);
        AINFO << "Routine task thread initialized.";
    } catch (const std::exception& e) {
        AERROR << "Failed to create routine task thread: " << e.what();
        return false;
    }

    return true;
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
VizComponent* VizComponent::g_instance_ = nullptr;
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




VizComponent::VizComponent() {
    g_instance_ = this;  // 在构造函数中设置全局实例指针
    old_env_ = std::getenv("LIBGL_ALWAYS_SOFTWARE");
    if (old_env_ == nullptr) {
        setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
    }
    // 禁用 MIT-SHM 环境变量
    setenv("QT_X11_NO_MITSHM", "1", 1);
    pan_x_ = 0.0f;
    pan_y_ = 0.0f;
}








VizComponent::~VizComponent() {
    if (render_thread_ && render_thread_->joinable()) {
        render_thread_->join();  // 等待渲染线程结束
    }

    if (routine_task_thread_ && routine_task_thread_->joinable()) {
        routine_task_thread_->join();  // 确保线程安全退出
    }

    if (old_env_ == nullptr) {
        unsetenv("LIBGL_ALWAYS_SOFTWARE");
    } else {
        setenv("LIBGL_ALWAYS_SOFTWARE", old_env_, 1);  // 恢复原有环境变量
    }
}






void VizComponent::DrawElementCallBack() {
    if (g_instance_) {
        g_instance_->DrawElements();  // 调用实际的绘制函数
    }
}








void VizComponent::KeyboardDown(unsigned char key, int x, int y) {
    if (key == 32) {  // 32 是 space 键的 ASCII 码
        is_space_pressed_.store(true);

        // 记录当前视角状态
        original_eye_x_ = pan_x_;
        original_eye_y_ = pan_y_;
        original_eye_z_ = 5.0 / zoom_;
        original_center_x_ = pan_x_;
        original_center_y_ = pan_y_;
        original_center_z_ = 0.0;
        original_up_x_ = 0.0;
        original_up_y_ = 1.0;
        original_up_z_ = 0.0;

        // 切换到鸟瞰视角
        gluLookAt(0.0, 0.0, 10.0,   // 摄像机位置
                  0.0, 0.0, 0.0,    // 观察点位置
                  0.0, 1.0, 0.0);   // 上方向
        glutPostRedisplay();
    }
}



void VizComponent::KeyboardUp(unsigned char key, int x, int y) {
    if (key == 32) {  // 32 是 space 键的 ASCII 码
        is_space_pressed_.store(false);

        // 恢复原始视角
        gluLookAt(original_eye_x_, original_eye_y_, original_eye_z_,
                  original_center_x_, original_center_y_, original_center_z_,
                  original_up_x_, original_up_y_, original_up_z_);
        glutPostRedisplay();
    }
}
















CYBER_REGISTER_COMPONENT(VizComponent);
