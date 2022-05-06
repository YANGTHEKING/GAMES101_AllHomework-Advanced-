#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)//直线扫描画线，本次作业没有用到，这段代码的原理是用的中点画线算法
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();//获得传入的线段起始点坐标

    Eigen::Vector3f line_color = {255, 255, 255};//设置默认线段颜色

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)//如果线段斜率的绝对值小于等于1就执行下列代码，这样区分是因为像素点坐标是整数，而如果斜率的绝对值小于1，在对线段进行采样时，每次x坐标加1，y坐标要么是加1，要么是减1，要么不变，简化了计算，而当斜率的绝对值大于1时，斜率的倒数的绝对值就小于1，就可以每次对y坐标加1,也是一样的效果
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }//这一段if else是为了保证x,y是起始点的坐标（从左到右，最左为起始点）
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);//转换齐次坐标
        set_pixel(point,line_color);//设置起始点颜色
        for(i=0;x<xe;i++)//从起始点开始遍历处理线段上每个点
        {
            x=x+1;
            if(px<0)//若px小于0,说明中点在线段之上，y坐标还不用加1
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))//根据斜率判断x坐标加1时y坐标是加还是减
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);//将得到的新点画上颜色
        }
    }
    else//如果线段斜率的绝对值大于1，基本处理与上面相似，只是是以y坐标为基准处理，不再赘述
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)//转换齐次坐标
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w); 
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
    if (type != rst::Primitive::Triangle)
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }
    auto& buf = pos_buf[pos_buffer.pos_id]; //根据传入的id参数获取图形顶点
    auto& ind = ind_buf[ind_buffer.ind_id];//根据传入的id参数获取图形顶点序号

    float f1 = (100 - 0.1) / 2.0;
    float f2 = (100 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model * rodrigues;//计算mvp矩阵
    for (auto& i : ind)//顺序处理所有图形顶点
    {
        Triangle t;

        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };//计算获得三个顶点经过mvp转换后的坐标

        for (auto& vec : v) {//齐次坐标除以第四维转常规坐标
            vec /= vec.w();
        }

        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }//视口变换

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        t.setColor(0, 255.0,  0.0,  0.0);
        t.setColor(1, 0.0  ,255.0,  0.0);
        t.setColor(2, 0.0  ,  0.0,255.0);

        rasterize_wireframe(t);
    }
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)//画出三角形的框架（三条边）
{
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::set_rodrigues(const Eigen::Matrix4f& r)
{
    rodrigues = r;
}

void rst::rasterizer::clear(rst::Buffers buff)//初始化，设置帧缓冲内所有像素颜色为（0，0，0），深度缓冲的所有像素深度为无限大
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)//根据宽高比设置帧缓冲大小和深度缓冲大小（大小就是像素个数）
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)//根据坐标求像素在缓冲区的序号
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)//将屏幕像素点 (x, y) 设 为 (r, g, b) 的颜色，并写入相应的帧缓冲区位置。
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;//如果像素点坐标超出屏幕范围，不处理
    auto ind = (height-1-point.y())*width + point.x();//这一步是根据坐标求像素在帧缓冲区的序号
    frame_buf[ind] = color;//将屏幕像素点 (x, y) 设 为 (r, g, b) 的颜色，并写入相应的帧缓冲区位置。
}

