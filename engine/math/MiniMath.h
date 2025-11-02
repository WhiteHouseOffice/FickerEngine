#pragma once
#include <cmath>
#include <cstring>

struct Mat4 { float m[16]; };

inline void matIdentity(Mat4& o){
  std::memset(o.m,0,sizeof(o.m));
  o.m[0]=o.m[5]=o.m[10]=o.m[15]=1.0f;
}

inline Mat4 matMul(const Mat4& A, const Mat4& B){
  Mat4 R;
  for(int r=0;r<4;++r) for(int c=0;c<4;++c){
    R.m[c*4+r] = A.m[0*4+r]*B.m[c*4+0] + A.m[1*4+r]*B.m[c*4+1] +
                 A.m[2*4+r]*B.m[c*4+2] + A.m[3*4+r]*B.m[c*4+3];
  }
  return R;
}

inline Mat4 perspective(float fovyRad, float aspect, float zNear, float zFar){
  float f = 1.0f / std::tan(fovyRad*0.5f);
  Mat4 o; std::memset(o.m,0,sizeof(o.m));
  o.m[0] = f/aspect;
  o.m[5] = f;
  o.m[10]= (zFar+zNear)/(zNear - zFar);
  o.m[11]= -1.0f;
  o.m[14]= (2.0f*zFar*zNear)/(zNear - zFar);
  return o;
}

inline Mat4 lookAt(const float eye[3], const float center[3], const float upv[3]){
  float f[3] = { center[0]-eye[0], center[1]-eye[1], center[2]-eye[2] };
  float fl = std::sqrt(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]); f[0]/=fl; f[1]/=fl; f[2]/=fl;
  float up[3] = { upv[0], upv[1], upv[2] };
  float ul = std::sqrt(up[0]*up[0]+up[1]*up[1]+up[2]*up[2]); up[0]/=ul; up[1]/=ul; up[2]/=ul;
  float s[3] = { f[1]*up[2]-f[2]*up[1], f[2]*up[0]-f[0]*up[2], f[0]*up[1]-f[1]*up[0] };
  float sl = std::sqrt(s[0]*s[0]+s[1]*s[1]+s[2]*s[2]); s[0]/=sl; s[1]/=sl; s[2]/=sl;
  float u[3] = { s[1]*f[2]-s[2]*f[1], s[2]*f[0]-s[0]*f[2], s[0]*f[1]-s[1]*f[0] };

  Mat4 M; matIdentity(M);
  M.m[0]=s[0]; M.m[4]=s[1]; M.m[8] =s[2];
  M.m[1]=u[0]; M.m[5]=u[1]; M.m[9] =u[2];
  M.m[2]=-f[0];M.m[6]=-f[1];M.m[10]=-f[2];

  Mat4 T; matIdentity(T);
  T.m[12]=-eye[0]; T.m[13]=-eye[1]; T.m[14]=-eye[2];

  return matMul(M,T);
}
