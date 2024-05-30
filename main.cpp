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
        float intensity = varying_intensity * bar; // normal vecs interpolated
        Vec2f uv = varying_uv * bar;
        color = model->diffuse(uv) * intensity;
        return false;
    }
};

struct TextureShaderNorm : public IShader
{
    mat<2, 3, float> varying_uv;
    mat<4, 4, float> uniform_M;   //  Projection*ModelView
    mat<4, 4, float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    virtual Vec4f vertex(int iface, int nthvert)
    {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        return Viewport * Projection * ModelView * gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color)
    {
        Vec2f uv = varying_uv * bar;
        // Now we get intensity from the texture's provided normal mapping
        // (M^-1)^T * normal * M * light = 0
        // We can dot the normal with the light direction
        Vec3f n = proj<3>(uniform_MIT * embed<4>(model->normal(uv))).normalize();
        Vec3f l = proj<3>(uniform_M * embed<4>(light_dir)).normalize();
        float intensity = std::max(0.f, n * l);
        color = model->diffuse(uv) * intensity;
        return false;
    }
};

struct PhongShader : public IShader
{
    mat<2, 3, float> varying_uv;
    mat<4, 4, float> uniform_M;
    mat<4, 4, float> uniform_MIT;

    virtual Vec4f vertex(int iface, int nthvert)
    {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));
        return Viewport * Projection * ModelView * gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color)
    {
        Vec2f uv = varying_uv * bar;
        Vec3f n = proj<3>(uniform_MIT * embed<4>(model->normal(uv))).normalize(); // normal mapping in global coords
        Vec3f l = proj<3>(uniform_M * embed<4>(light_dir)).normalize();
        Vec3f r = (n * (n * l * 2.f) - l).normalize();              // reflected light is 2 * normal - light source
        float spec = pow(std::max(r.z, 0.0f), model->specular(uv)); // texture glossiness
        float diff = std::max(0.f, n * l);                          // diffuse light is intensity
        TGAColor c = model->diffuse(uv);
        color = c;

        float diffusion_coeff = 0.6;
        float specular_coeff = 0.2;
        float ambient_coeff = 3;
        for (int i = 0; i < 3; i++)
            color[i] = std::min<float>(ambient_coeff + c[i] * (diffusion_coeff * diff + specular_coeff * spec), 255);
        return false;
    }
};

struct DarbouxShader : public IShader
{
    mat<2, 3, float> varying_uv;  // triangle texture uv coordinates
    mat<4, 3, float> varying_tri; // triangle coordinates (clip coordinates) after projection
    mat<3, 3, float> varying_nrm; // computed normal mapping per vertex, interpolated from texture map
    mat<3, 3, float> ndc_tri;     // triangle in normalized device coordinates (divided by homogenous)
    mat<4, 4, float> uniform_M;   //  Projection*ModelView
    mat<4, 4, float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    virtual Vec4f vertex(int iface, int nthvert)
    {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        varying_nrm.set_col(nthvert, proj<3>(uniform_MIT * embed<4>(model->normal(iface, nthvert), 0.f)));
        Vec4f gl_Vertex = uniform_M * embed<4>(model->vert(iface, nthvert));
        varying_tri.set_col(nthvert, gl_Vertex);
        ndc_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));
        return Viewport * gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color)
    {
        Vec3f bn = (varying_nrm * bar).normalize();
        Vec2f uv = varying_uv * bar;

        // Matrix A is the triangle in NDC
        mat<3, 3, float> A;
        A[0] = ndc_tri.col(1) - ndc_tri.col(0);
        A[1] = ndc_tri.col(2) - ndc_tri.col(0);
        A[2] = bn;

        mat<3, 3, float> AI = A.invert();

        // Get the Darboux tangent plane (basis) to the normal
        Vec3f i = AI * Vec3f(varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0);
        Vec3f j = AI * Vec3f(varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0);

        mat<3, 3, float> B;
        B.set_col(0, i.normalize());
        B.set_col(1, j.normalize());
        B.set_col(2, bn);

        // Change normal vector to tangent space basis for global coords
        // This gives a sense of depth with interpolation based on triangle positioning
        Vec3f n = (B * model->normal(uv)).normalize();

        float diff = std::max(0.f, n * light_dir);
        color = model->diffuse(uv) * diff;

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
    DarbouxShader shader;
    shader.uniform_M = Projection * ModelView;
    shader.uniform_MIT = (Projection * ModelView).invert_transpose();

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