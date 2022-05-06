#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;//平移相机位置到原点,这里应该是默认不用旋转相机角度了

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)//模型变换矩阵
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotation;
    double fangle = rotation_angle / 180 * MY_PI;//角度转弧度，便于计算

    rotation << cos(fangle), -sin(fangle), 0, 0,
                sin(fangle), cos(fangle), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;//模型旋转矩阵（绕z轴）

    model = rotation * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)//投影变换矩阵
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f proj, ortho;

    proj << zNear, 0, 0, 0,
            0, zNear, 0, 0,
            0, 0, zNear + zFar, -zNear * zFar,
            0, 0, 1, 0;//透视投影矩阵

    double w, h, z;
    h = zNear * tan(eye_fov / 2) * 2;
    w = h * aspect_ratio;
    z = zFar - zNear;

    ortho << 2 / w, 0, 0, 0,
             0, 2 / h, 0, 0,
             0, 0, 2 / z, -(zFar+zNear)/2,
             0, 0, 0, 0;//正交投影矩阵，因为在观测投影时x0y平面视角默认是中心，所以这里的正交投影就不用平移x和y了
    projection = ortho * proj * projection;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {//任意轴旋转矩阵（罗德里格斯旋转公式，默认轴过原点）
    double fangle = angle / 180 * MY_PI;
    Eigen::Matrix4f I, N, Rod;
    Eigen::Vector4f axi;
    Eigen::RowVector4f taxi;

    axi << axis.x(), axis.y(), axis.z(), 0;
    taxi << axis.x(), axis.y(), axis.z(), 0;

    I << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    N << 0, -axis.z(), axis.y(), 0,
         axis.z(), 0, -axis.x(), 0,
         -axis.y(), axis.x(), 0, 0,
         0, 0, 0, 1;
    
    Rod = cos(fangle) * I + (1 - cos(fangle)) * axi * taxi + sin(fangle) * N;
    Rod(3, 3) = 1;
    return Rod;
}

int main(int argc, const char** argv)
{
    float angle = 0;//定义角度
    bool command_line = false;//定义命令行开关标志，默认为关
    std::string filename = "output.png";//定义文件名，默认为output.png"

    Eigen::Vector3f raxis(0, 0, 1);
    double rangle = 0, ra;

    if (argc >= 3) {//接收到的参数大于三个，即检测到通过命令行传入参数时
        command_line = true;//设命令行开关标志为开
        angle = std::stof(argv[2]); //从命令行获取角度参数
        if (argc == 4) {//接收到的参数为四个，那么说明命令行输入了文件名参数
            filename = std::string(argv[3]);//从命令行获取文件名
        }
    }

    rst::rasterizer r(700, 700);//设定700*700像素的光栅器视口
    Eigen::Vector3f eye_pos = { 0, 0, 5 };//设定相机位置
    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };//设定三顶点位置
    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };//设定三顶点序号,用于画图时确定需要处理几个顶点，这里表示的是三个顶点

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);//保存多个图形的顶点和序号，本次作业只涉及一个图形，可以不管

    int key = 0;//键盘输入
    int frame_count = 0;//帧序号

    if (command_line) {//如果命令行开关标志为开（这一段if代码是为了应用命令行传入的参数，比如初始角度和文件名）

        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//初始化帧缓存和深度缓存（本次作业本次作业只涉及一个图形，可以不管，所以不涉及深度，可以不管）

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));//向光栅器传入MVP矩阵
        r.set_rodrigues(get_rotation(raxis, rangle));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);//开始光栅化画图
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imwrite(filename, image);

        return 0;
    }

    bool rflag = false;

    std::cout << "Please enter the axis and angle:" << std::endl;
    std::cin >> raxis.x() >> raxis.y() >> raxis.z() >> ra;//定义罗德里格斯旋转轴和角

    while (key != 27) {//只要没有检测到按下ESC就循环(ESC的ASCII码是27)

        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        if (rflag) //如果按下r了，就开始绕给定任意轴旋转
            r.set_rodrigues(get_rotation(raxis, rangle));
        else
            r.set_rodrigues(get_rotation({ 0,0,1 }, 0));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);//等待键盘输入

        std::cout << "frame count: " << frame_count++ << '\n';//显示当前是第几帧画面

        if (key == 'a') {//按下a，逆时针旋转10°
            angle += 10;
        }
        else if (key == 'd') {//按下d，顺时针旋转10°
            angle -= 10;
        }
        else if (key == 'r') {//按下r，绕给定任意轴旋转
            rflag = true;
            rangle += ra;
        }
    }

    return 0;

}