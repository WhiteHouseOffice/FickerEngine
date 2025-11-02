#pragma once
#include <cmath>
#include <cstring>

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s)     const { return {x*s, y*s, z*s}; }

    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator-=(const Vec3& o){ x-=o.x; y-=o.y; z-=o.z; return *this; }   // ← needed
};

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return { a.y*b.z - a.z*b.y,
             a.z*b.x - a.x*b.z,
             a.x*b.y - a.y*b.x };
}
inline float dot(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 normalize(const Vec3& v){
    float L = std::sqrt(dot(v,v));
    return (L>0.f) ? Vec3{v.x/L, v.y/L, v.z/L} : Vec3{};
}

struct Mat4 {
    // Column-major 4x4
    float m[16];
    static Mat4 identity(){
        Mat4 r{}; std::memset(r.m, 0, sizeof(r.m));
        r.m[0]=r.m[5]=r.m[10]=r.m[15]=1.f; return r;
    }
};

inline Mat4 mul(const Mat4& A, const Mat4& B){
    Mat4 R{};
    for(int c=0;c<4;++c){
        for(int r=0;r<4;++r){
            R.m[c*4+r] =
                A.m[0*4+r]*B.m[c*4+0] +
                A.m[1*4+r]*B.m[c*4+1] +
                A.m[2*4+r]*B.m[c*4+2] +
                A.m[3*4+r]*B.m[c*4+3];
        }
    }
    return R;
}
inline Mat4 matMul(const Mat4& A, const Mat4& B) { return mul(A,B); }

inline Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up){
    Vec3 f = normalize(Vec3{center.x-eye.x, center.y-eye.y, center.z-eye.z});
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);

    Mat4 M = Mat4::identity();
    M.m[0]=s.x; M.m[4]=s.y; M.m[8] =s.z;
    M.m[1]=u.x; M.m[5]=u.y; M.m[9] =u.z;
    M.m[2]=-f.x;M.m[6]=-f.y;M.m[10]=-f.z;

    M.m[12] = -dot(s, eye);
    M.m[13] = -dot(u, eye);
    M.m[14] =  dot(f, eye);
    return M;
}

inline Mat4 perspective(float fovyRad, float aspect, float znear, float zfar){
    float f = 1.0f / std::tan(fovyRad * 0.5f);
    Mat4 M{}; std::memset(M.m, 0, sizeof(M.m));
    M.m[0] = f / aspect;
    M.m[5] = f;
    M.m[10]= (zfar + znear) / (znear - zfar);
    M.m[11]= -1.0f;
    M.m[14]= (2.0f * zfar * znear) / (znear - zfar);
    return M;
}
