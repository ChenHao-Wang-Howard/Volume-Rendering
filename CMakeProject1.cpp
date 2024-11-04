// CMakeProject1.cpp: 定義應用程式的進入點。
//


#define _USE_MATH_DEFINES
#include <cmath>
#include "CMakeProject1.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <algorithm>
#include <vector>
#include <random>
using namespace std;


struct vec3
{
    float x{ 0 }, y{ 0 }, z{ 0 };
    // 向量歸一化
    vec3& nor()
    {
        float len = x * x + y * y + z * z;
        if (len != 0) len = sqrtf(len);
        x /= len, y /= len, z /= len;
        return *this;
    }
    float length() const
    {
        return sqrtf(x * x + y * y + z * z);
    }
    float operator * (const vec3& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }
    vec3 operator - (const vec3& v) const
    {
        return vec3{ x - v.x, y - v.y, z - v.z };
    }
    vec3 operator + (const vec3& v) const
    {
        return vec3{ x + v.x, y + v.y, z + v.z };
    }
    vec3& operator += (const vec3& v)
    {
        x += v.x, y += v.y, z += v.z;
        return *this;
    }
    vec3& operator *= (const float& r)
    {
        x *= r, y *= r, z *= r;
        return *this;
    }
    friend vec3 operator * (const float& r, const vec3& v)
    {
        return vec3{ v.x * r, v.y * r, v.z * r };
    }
    friend std::ostream& operator << (std::ostream& os, const vec3& v)
    {
        os << v.x << " " << v.y << " " << v.z;
        return os;
    }
    vec3 operator * (const float& r) const
    {
        return vec3{ x * r, y * r, z * r };
    }
};
// 設定背景色
constexpr vec3 background_color{ 0.572f, 0.772f, 0.921f };
constexpr float floatMax = std::numeric_limits<float>::max();
struct IsectData
{
    float t0{ floatMax }, t1{ floatMax }; // 相交點的參數值
    vec3 pHit; // 相交點
    vec3 nHit; // 相交點的法線
    bool inside{ false }; // 光線是否在物體內部
};
struct Object
{
public:
    vec3 color;
    int type{ 0 };
    virtual bool intersect(const vec3&, const vec3&, IsectData&) const = 0;
    virtual ~Object() {}
    Object() {}
};
bool solveQuadratic(float a, float b, float c, float& r0, float &r1) {
    float d = b * b - 4 * a * c;
    if (d < 0) {
        return false;
    }
    else if (d == 0) {
        r0 = r1 = -b / (2 * a);
    }
    else {
        r0 = (-b + sqrtf(d)) / (2 * a);
        r1 = (-b - sqrtf(d)) / (2 * a);
    }
    if (r0 > r1) swap(r0, r1);
    return true;

}
struct  Sphere : Object {
public:
    Sphere() {
        color = vec3{ 1, 0, 0 };
        type = 1;
    }
    bool intersect(const vec3& rayOrig, const vec3& rayDir, IsectData& isect) const override {
        vec3 rayOrigc = rayOrig - center; // 計算光線起點到球心的向量
        float a = rayDir * rayDir; // 計算二次方程的係數
        float b = 2 * (rayDir * rayOrigc);
        float c = rayOrigc * rayOrigc - radius * radius;

        if (!solveQuadratic(a, b, c, isect.t0, isect.t1)) return false; // 解二次方程，若無解則返回 false

        if (isect.t0 < 0) {
            if (isect.t1 < 0) return false; // 光線完全在球體後方
            else {
                isect.inside = true; // 光線從內部射出
                isect.t0 = 0;
            }
        }

        return true; // 相交成功
    }

    float radius{ 1 };
    vec3 center{ 0, 0, -4 }; // 球體的中心位置
};
// 光線積分函數，用於計算光線穿過體積物體時的顏色
vec3 integrate(const vec3& ray_orig, const vec3& ray_dir, const std::vector<std::unique_ptr<Object>>& objects, string algo ) {
    const Object* hit_object = nullptr;
    IsectData isect;

    // 檢測光線與場景中物體的相交
    for (const auto& object : objects) {
        IsectData isect_object;
        if (object->intersect(ray_orig, ray_dir, isect_object)) {
            hit_object = object.get();
            isect = isect_object;
        }
    }

    if (!hit_object) return background_color; // 若無相交物體，返回背景色

    // 初始化參數
    float step_size = 0.2;
    float absorption = 0.1;
    float scattering = 0.1;
    float density = 1;
    int ns = std::ceil((isect.t1 - isect.t0) / step_size);
    step_size = (isect.t1 - isect.t0) / ns;

    vec3 light_dir{ 0, 1, 0 }; // 光源方向
    vec3 light_color{ 1.3, 0.3, 0.9 }; // 光源顏色
    IsectData isect_vol;
    float transparency = 1; // 透明度
    vec3 result{ 0 }; // 累積顏色
    if (algo == "BACKWARD_RAYMARCHING") {
        // 後向步進（從 t1 向 t0 步進）
        for (int n = 0; n < ns; ++n) {
            float t = isect.t1 - step_size * (n + 0.5);
            vec3 sample_pos = ray_orig + t * ray_dir; // 計算樣本位置

            float sample_transparency = exp(-step_size * (scattering + absorption));
            transparency *= sample_transparency; // 更新透明度

            // 檢查光線是否穿過體積物體，並計算光的衰減
            if (hit_object->intersect(sample_pos, light_dir, isect_vol) && isect_vol.inside) {
                float light_attenuation = exp(-density * isect_vol.t1 * (scattering + absorption));
                result += light_color * light_attenuation * scattering * density * step_size;
            }
            else {
                std::cerr << "oops\n"; // 錯誤處理
            }

            result *= sample_transparency; // 累積顏色
        }
    }
    else if (algo == "FORWARD_RAYMARCHING") {
        // 前向步進（從 t0 向 t1 步進）
        for (int n = 0; n < ns; ++n) {
            float t = isect.t0 + step_size * (n + 0.5);
            vec3 sample_pos = ray_orig + t * ray_dir; // 計算樣本位置

            float sample_attenuation = exp(-step_size * (scattering + absorption));
            transparency *= sample_attenuation; // 更新透明度

            // 計算入射光線的衰減
            if (hit_object->intersect(sample_pos, light_dir, isect_vol) && isect_vol.inside) {
                float light_attenuation = exp(-density * isect_vol.t1 * (scattering + absorption));
                result += transparency * light_color * light_attenuation * scattering * density * step_size;
            }
            else {
                std::cerr << "oops\n"; // 錯誤處理
            }
        }
    }


    // 混合背景色和累積顏色
    return background_color * transparency + result;
}

int main()
{
    int algo;
    string str;
	cout << "1. backward or 2. forward" << endl;
    cin >> algo;
    if (algo == 1) {
        str = "BACKWARD_RAYMARCHING";
    }
    else if(algo == 2){
        str = "FORWARD_RAYMARCHING";
    }
    else {
        return 0;
    }
    unsigned int width = 640, height = 480;
    auto buffer = std::make_unique<unsigned char[]>(width * height * 3);

    // 計算影像寬高比和焦距
    auto frameAspectRatio = width / float(height);
    float fov = 45;
    float focal = tan(M_PI / 180 * fov * 0.5);

    // 建立場景中的物體
    std::vector<std::unique_ptr<Object>> geo;
    std::unique_ptr<Sphere> sph = std::make_unique<Sphere>();
    sph->radius = 5;
    sph->center = { 0, 0, -20 }; // 設定球體的位置
    geo.push_back(std::move(sph));

    vec3 rayOrig, rayDir; // 光線起點和方向
    unsigned int offset = 0;

    // 遍歷每個像素，計算光線的顏色
    for (unsigned int j = 0; j < height; ++j) {
        for (unsigned int i = 0; i < width; ++i) {
            rayDir.x = (2.f * (i + 0.5f) / width - 1) * focal;
            rayDir.y = (1 - 2.f * (j + 0.5f) / height) * focal * 1 / frameAspectRatio;
            rayDir.z = -1.f;
            rayDir.nor(); // 歸一化光線方向

            vec3 c = integrate(rayOrig, rayDir, geo, str); // 計算光線顏色

            // 將顏色值轉換為 0-255，並存入緩衝區
            buffer[offset++] = std::clamp(c.x, 0.f, 1.f) * 255;
            buffer[offset++] = std::clamp(c.y, 0.f, 1.f) * 255;
            buffer[offset++] = std::clamp(c.z, 0.f, 1.f) * 255;
        }
    }

    // 將緩衝區寫入 PPM 影像檔
    std::ofstream ofs;
    ofs.open("./image.ppm", std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    ofs.write(reinterpret_cast<const char*>(buffer.get()), width * height * 3);
    ofs.close();

    return 0;
	
}
