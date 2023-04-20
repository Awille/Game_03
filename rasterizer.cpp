//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <Eigen/Core>
#include "Eigen/Dense"

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

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
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
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
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
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
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
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
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
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

/**
 * 向量叉乘（也称为向量叉积或向量外积）是一种在三维空间中定义的操作，用于计算两个三维向量之间的结果向量。
 * 向量叉乘的结果向量垂直于原始两个向量所在的平面，并且其大小与两个向量之间的夹角和两个向量的长度有关。
 * 在二维空间中，没有定义向量叉乘的概念，因为在二维空间中，两个向量总是在同一平面内，无法产生垂直于两个向量的结果向量。
 * 在二维空间中，可以使用向量的叉乘的一种特殊情况，称为 "伪叉乘" 或 "叉乘的标量形式"，其结果是一个标量而不是向量。
 * 伪叉乘的结果等于两个向量的长度乘积与其夹角的正弦值的乘积，这可以表示为以下公式：
 * 对于二维向量 u = (u1, u2) 和 v = (v1, v2)：u x v = u1 * v2 - u2 * v1
 * 注意，这里的结果是一个标量（即一个实数），而不是一个向量。这与三维空间中向量叉乘的结果不同，后者是一个向量。
 * @param pos 三角形顶点
 * @param x  点的x坐标
 * @param y  点的y坐标
 * @return
 */
static bool insideTriangle(const Eigen::Vector4f* pos, float x, float y) {
    Eigen::Vector2f ab = {pos[1][0] - pos[0][0], pos[1][1] - pos[0][1]};
    Eigen::Vector2f bc = {pos[2][0] - pos[1][0], pos[2][1] - pos[1][1]};
    Eigen::Vector2f ca = {pos[0][0] - pos[2][0], pos[0][1] - pos[2][1]};

    Eigen::Vector2f ap = {x - pos[0][0], y - pos[0][1]};
    Eigen::Vector2f bp = {x - pos[1][0], y - pos[1][1]};
    Eigen::Vector2f cp = {x - pos[2][0], y - pos[2][1]};

    float z1 = ab[0] * ap[1] - ab[1] * ap[0];
    float z2 = bc[0] * bp[1] - bc[1] * bp[0];
    float z3 = ca[0] * cp[1] - ca[1] * cp[0];

    if ((z1 >= 0 && z2 >= 0 && z3 >= 0) || (z1 < 0 && z2 < 0 && z3 < 0)) {
        return true;
    }
    return false;
// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{


    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    //第一步创建三角形的bounding box
    auto v = t.toVector4();
    float min_x = std::min(v[0][0], std::min(v[1][0], v[2][0]));
    float max_x = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    float min_y = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    float max_y = std::max(v[0][1], std::max(v[1][1], v[2][1]));

    for (int i = floor(min_x); i < ceil(max_x); i ++) {
        for (int j = floor(min_y); j < ceil(max_y); j ++) {
            float x = i + 0.5f; float y = j + 0.5f;
            if (insideTriangle(t.v, x, y)) {
                //重心坐标求出三个顶点分量占比
                auto [alpha, beta, gamma] = computeBarycentric2D(x , y, t.v);
                //插值求出深度
                //但是该插值是有问题，因为是坐标转换后进行的插值，需要进行透视矫正插值
                //float zp = alpha * v[0].z() + beta * v[1].z() + gamma * v[2].z();
                float z = 1.0f / (alpha / v[0].w() + beta * v[1].w() + gamma * v[2].w());
                float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w()
                        + gamma * v[2].z() / v[2].w();
                //需要堆投影后的坐标做你变化
                zp *= z;

                if (zp < depth_buf[get_index(i, j)]) {
                    // TODO: Interpolate the attributes:
                    // auto interpolated_color
                    // auto interpolated_normal
                    // auto interpolated_texcoords
                    // auto interpolated_shadingcoords

                    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                    // Use: payload.view_pos = interpolated_shadingcoords;
                    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
                    // Use: auto pixel_color = fragment_shader(payload);

                    //颜色插值
                    auto interpolated_color = interpolate(alpha, beta, gamma,
                                                          t.color[0], t.color[1], t.color[2], 1);

                    //法向量插值
                    auto interpolated_normal =
                            interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], 1).normalized();

                    //纹理坐标插值
                    auto interpolated_texcoords = interpolate(alpha, beta, gamma,
                                                              t.tex_coords[0], t.tex_coords[1], t.tex_coords[2], 1);


                    /**
                     * shaing Coords
                     * 首先，你进行了光栅化，然后这个时候你其实是只知道像素坐标x，y的，经过深度插值，你得到了screen space中对应的某点的坐标（x，y，z）。
                     * 这个坐标其实也不是真实世界（或者说camera space）里面的点！因为你想projection矩阵本质上是把视锥压缩成了长方体，
                     * 你算出来的这个坐标是经历了压缩之后的。那么怎么才能知道camera space中像素对应过去的那个点呢？当然是插值！
                     * 也就是利用alpha，beta，gamma结合rasterize_triangle传进来的参数viewspace_pos插值出来一个空间中的点。
                     * 好了，我们得到了这个所谓的interpolated_shadingcoords，
                     * 这个点有什么意义呢？这个点其实就是camera space中你真正在着色的那个点！这个点是你插值出来的，所以起了这么个名字。
                     * 这个点知道了有什么用？你想想Bling Phong反射模型是不是需要一个叫r的参数，也就是着色点到光源的距离？
                     * 我们需要拿光源位置和这个着色点来计算出r，同时得到一个light ray的单位向量（也就是那个所谓的向量l）！
                     * 最后一个点：我们的重心坐标明明是在2D空间里做的啊？
                     * 为什么可以拿来插值3D空间的坐标呢？
                     * 答案是不行的！这个alpha，beta，gamma本质上是需要经过矫正才能用的！但是！其实误差不大，我们就直接拿过来用了。详见
                     * https://stackoverflow.com/questions/24441631/how-exactly-does-opengl-do-perspectively-correct-linear-interpolation#:~:text=Perspective%20correct%20interpolation,-So%20let%E2%80%99s%20say&text=First%20we%20calculate%20its%20barycentric,on%20the%202D%20
                     */

                    auto interpolated_shadingcoords = interpolate(alpha, beta, gamma,
                                                                  view_pos[0], view_pos[1], view_pos[2], 1);
                    fragment_shader_payload payload(interpolated_color, interpolated_normal, interpolated_texcoords, texture ? &*texture : nullptr);

                    payload.view_pos = interpolated_shadingcoords;

                    auto pixel_color = fragment_shader(payload);

                    depth_buf[get_index(i, j)] = zp;
                    set_pixel(Eigen::Vector2i(x, y), pixel_color);
                }
            }
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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}

