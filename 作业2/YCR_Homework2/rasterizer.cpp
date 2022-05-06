// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


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

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static float insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    std::vector<Vector3f> ver, ver2;
    ver.push_back({_v[1].x()-_v[0].x(),_v[1].y()-_v[0].y(),0}); ver2.push_back({x-_v[0].x(),y-_v[0].y(),0});    
    ver.push_back({_v[2].x()-_v[1].x(),_v[2].y()-_v[1].y(),0}); ver2.push_back({x-_v[1].x(),y-_v[1].y(),0});
    ver.push_back({_v[0].x()-_v[2].x(),_v[0].y()-_v[2].y(),0}); ver2.push_back({x-_v[2].x(),y-_v[2].y(),0});

    for(int i=0;i<3;i++){
        if(ver[i].cross(ver2[i]).z() < 0)
            return false;
    }
    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    std::vector<float> a{0.25,0.25,0.75,0.75,0.25};
    Vector3f color; //color at each vertex;
    float alpha, beta, gamma, lmin=INT_MAX, rmax=INT_MIN, tmax=INT_MIN, bmin=INT_MAX, mindep = INT_MAX, count = 0, eid;
    // TODO : Find out the bounding box of current triangle.
    for(auto &k:v){
        lmin = int(std::min(lmin,k.x()));
        rmax = std::max(rmax,k.x());rmax = rmax == int(rmax) ? int(rmax)-1 : rmax;
        tmax = std::max(tmax,k.y());tmax = tmax == int(tmax) ? int(tmax)-1 : tmax;
        bmin = int(std::min(bmin,k.y()));
    }
    // iterate through the pixel and find if the current pixel is inside the triangle
    for(float i = lmin; i <= rmax; i++){
        for(float j = bmin; j <= tmax; j++){
            count = 0; mindep = INT_MAX, eid = get_index(i,j)*4;
            for(int k = 0; k < 4; k++){
                if(insideTriangle(i+a[k], j+a[k+1], t.v)){
                    // If so, use the following code to get the interpolated z value.
                    std::tie(alpha, beta, gamma) = computeBarycentric2D(i+a[k], j+a[k+1], t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if (-z_interpolated < depth_sample[eid + k]){
                        depth_sample[eid + k] = -z_interpolated;
                        frame_sample[eid + k] = t.getColor() / 4;
                    }
                    mindep = std::min(depth_sample[eid + k], mindep);
                }
            }
            // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
            color = frame_sample[eid] + frame_sample[eid + 1] + frame_sample[eid + 2] + frame_sample[eid + 3];
            set_pixel({i,j,1}, color);
            depth_buf[get_index(i,j)] = std::min(depth_buf[get_index(i,j)], mindep);   
            /*if(insideTriangle(i, j, t.v)){//如果在三角形内
                // If so, use the following code to get the interpolated z value.
                std::tie(alpha, beta, gamma) = computeBarycentric2D(i+0.5, j+0.5, t.v);//对当前像素坐标z值插值

                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if(-z_interpolated < depth_buf[get_index(i,j)]){//如果当前z值比像素z值小（这里是把z值换成正数比较的）
                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    set_pixel({i,j,1},t.getColor());
                    depth_buf[get_index(i,j)] = -z_interpolated;//设置像素颜色，修改像素当前深度   
                }
            }*/
        }
    }
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

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_sample.resize(w * h * 4);
    depth_sample.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

// clang-format on