#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const int width = 800;
const int height = 800;
const int depth = 255;

Model *model = NULL;
int *zbuffer = NULL;
Vec3f light_dir = Vec3f(1, -1, 1).normalize();
Vec3f eye(1, 1, 3);
Vec3f center(0, 0, 0);

////////////////////////////////////////////////////////////////////////
// Matrix Operations for projections
////////////////////////////////////////////////////////////////////////
Vec3f m2v(Matrix m)
{
    return Vec3f(m[0][0] / m[3][0], m[1][0] / m[3][0], m[2][0] / m[3][0]);
}

Matrix v2m(Vec3f v)
{
    Matrix m(4, 1);
    m[0][0] = v.x;
    m[1][0] = v.y;
    m[2][0] = v.z;
    m[3][0] = 1.f;
    return m;
}

Matrix viewport(int x, int y, int w, int h)
{
    Matrix m = Matrix::identity(4);
    m[0][3] = x + w / 2.f;
    m[1][3] = y + h / 2.f;
    m[2][3] = depth / 2.f;

    m[0][0] = w / 2.f;
    m[1][1] = h / 2.f;
    m[2][2] = depth / 2.f;
    return m;
}

Matrix lookat(Vec3f eye, Vec3f center, Vec3f up)
{
    Vec3f z = (eye - center).normalize();
    Vec3f x = (up ^ z).normalize();
    Vec3f y = (z ^ x).normalize();
    Matrix res = Matrix::identity(4);
    for (int i = 0; i < 3; i++)
    {
        res[0][i] = x[i];
        res[1][i] = y[i];
        res[2][i] = z[i];
        res[i][3] = -center[i];
    }
    return res;
}

////////////////////////////////////////////////////////////////////////
// Triangle Geometry
////////////////////////////////////////////////////////////////////////
Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P)
{
    Vec3f s[2]; // sides
    for (int i = 2; i--;)
    {
        s[i][0] = C[i] - A[i];
        s[i][1] = B[i] - A[i];
        s[i][2] = A[i] - P[i];
    }
    Vec3f u = s[0] ^ s[1];
    if (std::abs(u[2]) > 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
    return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void triangle(Vec3i *pts, Vec2i *uv, float *intensity, TGAImage &image, int *zbuffer)
{
    Vec3i t0 = pts[0], t1 = pts[1], t2 = pts[2];
    Vec2i uv0 = uv[0], uv1 = uv[1], uv2 = uv[2];
    float intensity0 = intensity[0], intensity1 = intensity[1], intensity2 = intensity[2];

    if (t0.y == t1.y && t0.y == t2.y)
        return; // I don't care about degenerate triangles

    // Bounding box
    int minX = std::min(t0.x, std::min(t1.x, t2.x));
    int maxX = std::max(t0.x, std::max(t1.x, t2.x));
    int minY = std::min(t0.y, std::min(t1.y, t2.y));
    int maxY = std::max(t0.y, std::max(t1.y, t2.y));

    Vec3i P;
    for (P.y = minY; P.y <= maxY; P.y++)
    {
        for (P.x = minX; P.x <= maxX; P.x++)
        {
            Vec3f bc_screen = barycentric(t0, t1, t2, P);
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0)
                continue;

            P.z = 0;
            P.z += t0.z * bc_screen.x;
            P.z += t1.z * bc_screen.y;
            P.z += t2.z * bc_screen.z;

            int idx = P.x + P.y * image.get_width();
            if (zbuffer[idx] < P.z)
            {
                zbuffer[idx] = P.z;
                // Interpolating UV color and intensity for this pixel
                Vec2i uvP = uv0 * bc_screen.x + uv1 * bc_screen.y + uv2 * bc_screen.z;
                float intensity = intensity0 * bc_screen.x + intensity1 * bc_screen.y + intensity2 * bc_screen.z;
                TGAColor color = model->diffuse(uvP);
                image.set(P.x, P.y, TGAColor(color.r, color.g, color.b) * intensity);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // Initialize model and buffers
    if (2 == argc)
    {
        model = new Model(argv[1]);
    }
    else
    {
        model = new Model("obj/african_head/african_head.obj");
    }

    zbuffer = new int[width * height];
    for (int i = width * height; i--;)
    {
        zbuffer[i] = std::numeric_limits<int>::min();
    }

    // Draw the model
    {
        // Camera (ModelView) to perspective (Projection) to screen (ViewPort)
        Matrix ModelView = lookat(eye, center, Vec3f(0, 1, 0));
        Matrix Projection = Matrix::identity(4);
        Matrix ViewPort = viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
        Projection[3][2] = -1.f / (eye - center).norm();
        Matrix z = (ViewPort * Projection * ModelView);

        // Drawing triangles
        TGAImage image(width, height, TGAImage::RGB);
        for (int i = 0; i < model->nfaces(); i++)
        {
            std::vector<int> face = model->face(i);
            Vec3i screen_coords[3];
            Vec3f world_coords[3];
            float intensity[3];

            for (int j = 0; j < 3; j++)
            {
                Vec3f v = model->vert(face[j]);
                screen_coords[j] = m2v(z * v2m(v));
                world_coords[j] = v;
                intensity[j] = model->norm(i, j) * light_dir;
            }

            Vec2i uv[3];
            for (int k = 0; k < 3; k++)
            {
                uv[k] = model->uv(i, k);
            }
            triangle(screen_coords, uv, intensity, image, zbuffer);
        }

        image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
        image.write_tga_file("output.tga");
    }

    // Dump the z-buffer for debugging
    {
        TGAImage zbimage(width, height, TGAImage::GRAYSCALE);
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                zbimage.set(i, j, TGAColor(zbuffer[i + j * width], 1));
            }
        }
        zbimage.flip_vertically(); // i want to have the origin at the left bottom corner of the image
        zbimage.write_tga_file("zbuffer.tga");
    }
    delete model;
    delete[] zbuffer;
    return 0;
}