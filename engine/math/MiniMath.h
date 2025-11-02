#pragma once
#include <cmath>
#include <cstring>
struct Mat4 { float m[16]; };
inline void matIdentity(Mat4& o){ std::memset(o.m,0,sizeof(o.m)); o.m[0]=o.m[5]=o.m[10]=o.m[15]=1.0f; }
