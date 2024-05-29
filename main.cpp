#include <vector>
#include <iostream>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

// Set rendering parameters
Model *model = NULL;
const int width = 800;
const int height = 800;

Vec3f light_dir = Vec3f(1, 1, 1);
Vec3f eye(1, 1, 3);
Vec3f center(0, 0, 0);
Vec3f up(0, 1, 0);

////////////////////////////////////////////////////////////////////////
// Shaders
struct GouraudShader : public IShader
{
    // varying = interpolated data between vertices
    Vec3f varying_intensity;

    // Gets object geometry, transforms to screen coordinates, and writes intensity
    virtual Vec4f vertex(int iface, int nthvert)
    {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));                               // read the vertex from .obj file
        gl_Vertex = Viewport * Projection * ModelView * gl_Vertex;                             // transform it to screen coordinates
        varying_intensity[nthvert] = std::max(0.f, model->normal(iface, nthvert) * light_dir); // get diffuse lighting intensity
        return gl_Vertex;
    }

    // Reads intensity, barymetric coords and interpolates values, writes color
    virtual bool fragment(Vec3f bar, TGAColor &color)
    {
        float intensity = varying_intensity * bar; // interpolate intensity for the current pixel
        color = TGAColor(255, 255, 255) * intensity;
        return false; // Do not discard this pixel
    }
};

struct TextureShader : public IShader
{
    Vec3f varying_intensity;
    mat<2, 3, float> varying_uv; // (u,v) * (v1,v2,v3)

    virtual Vec4f vertex(int iface, int nthvert)
    {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));
        varying_intensity[nthvert] = std::max(0.f, model->normal(iface, nthvert) * light_dir);
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        return Viewport * Projection * ModelView * gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color)
    {
        float intensity = varying_intensity * bar;
        Vec2f uv = varying_uv * bar;
        color = model->diffuse(uv) * intensity;
        return false;
    }
};

////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // Initialize model
    if (2 == argc)
    {
        model = new Model(argv[1]);
    }
    else
    {
        model = new Model("obj/african_head/african_head.obj");
    }

    // Initialize our matrices and shader in our GL
    lookat(eye, center, up);
    viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    projection(-1.f / (eye - center).norm());
    light_dir.normalize();
    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);
    TextureShader shader;

    // Rasterize each triangle
    for (int i = 0; i < model->nfaces(); i++)
    {
        Vec4f screen_coords[3];
        for (int j = 0; j < 3; j++)
        {
            // Call vertex shader to prepare data
            screen_coords[j] = shader.vertex(i, j);
        }
        triangle(screen_coords, shader, image, zbuffer);
    }

    image.flip_vertically(); // to place the origin in the bottom left corner of the image
    zbuffer.flip_vertically();
    image.write_tga_file("output.tga");
    zbuffer.write_tga_file("zbuffer.tga");

    delete model;
    return 0;
}