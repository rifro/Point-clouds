#pragma once
#include &lt;cmath&gt;
#include &lt;algorithm&gt;

struct Vec3f { float x,y,z; };
struct Quat  { float w,x,y,z; };

**host** **device** inline Vec3f normalize(const Vec3f& v){
float n = std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z)+1e-20f;
return {v.x/n, v.y/n, v.z/n};
}
**host** **device** inline float dot(const Vec3f& a, const Vec3f& b){
return a.x*b.x + a.y*b.y + a.z*b.z;
}
**host** **device** inline Vec3f cross(const Vec3f& a, const Vec3f& b){
return { a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x };
}

// Welford: gemiddelde & M2 (variance som) voor streaming cosines
struct Welford {
int   n   = 0;
float mean= 0.f;
float M2  = 0.f;

```
__host__ __device__ inline void add(float x){
    n++;
    float delta = x - mean;
    mean += delta / float(n);
    float delta2 = x - mean;
    M2 += delta * delta2;
}
__host__ __device__ inline float variance() const {
    return (n &gt; 1) ? (M2 / float(n-1)) : 0.f;
}
__host__ __device__ inline float sigma() const {
    float v = variance();
    return v &gt; 0.f ? std::sqrt(v) : 0.f;
}
```

};

// Exponential moving average op een normalerichting
struct EmaNormaal {
Vec3f n   = {0,0,1};
bool  init= false;
float alpha= 0.2f;

```
__host__ __device__ inline void setAlpha(float a){ alpha = a; }

__host__ __device__ inline void add(const Vec3f&amp; m){
    if(!init){ n = normalize(m); init = true; return; }
    // lineaire mix gevolgd door normaliseren
    Vec3f mix{ (1-alpha)*n.x + alpha*m.x,
               (1-alpha)*n.y + alpha*m.y,
               (1-alpha)*n.z + alpha*m.z };
    n = normalize(mix);
}
```

};

// Quaternion helpers
inline Quat quat_from_axis_angle(const Vec3f& axis, float rad){
Vec3f a = normalize(axis);
float s = std::sin(rad*0.5f);
return { std::cos(rad*0.5f), a.x*s, a.y*s, a.z*s };
}
inline Vec3f quat_rotate(const Quat& q, const Vec3f& v){
// v' = q * (0,v) * q^{-1}
Vec3f u{q.x,q.y,q.z};
float s = q.w;
Vec3f uv = cross(u,v);
Vec3f uuv= cross(u,uv);
Vec3f out{ v.x + 2.0f*( s*uv.x + uuv.x ),
v.y + 2.0f*( s*uv.y + uuv.y ),
v.z + 2.0f*( s*uv.z + uuv.z ) };
return out;
}
inline Quat quat_mul(const Quat& a, const Quat& b){
return {
a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
};
}

// Decompose quaternion naar Z-rotatie (yaw) vervolgens Y-rotatie (pitch).
// Doel: twee as-rotaties kunnen printen in graden (CloudCompare UI check).
inline void quat_to_ZY_angles(const Quat& q, float& yawDeg, float& pitchDeg){
// Converteer naar rotatiematrix en haal ZY-Euler (yaw-pitch, geen roll).
// r = Rz(yaw)*Ry(pitch)
float ww=q.w*q.w, xx=q.x*q.x, yy=q.y*q.y, zz=q.z*q.z;
float xy=q.x*q.y, xz=q.x*q.z, yz=q.y*q.z, wx=q.w*q.x, wy=q.w*q.y, wz=q.w*q.z;

```
float r00 = ww+xx-yy-zz;
float r01 = 2*(xy - wz);
float r02 = 2*(xz + wy);

float r10 = 2*(xy + wz);
float r11 = ww-xx+yy-zz;
float r12 = 2*(yz - wx);

float r20 = 2*(xz - wy);
float r21 = 2*(yz + wx);
float r22 = ww-xx-yy+zz;

// ZY-extract
float pitch = std::asin( std::clamp(-r20, -1.0f, 1.0f) ); // Ry
float yaw   = std::atan2(r10, r00);                       // Rz

yawDeg   = yaw   * 180.0f / float(M_PI);
pitchDeg = pitch * 180.0f / float(M_PI);
```

}

// Clip kleine correcties (|graad| < clip) naar 0
inline void clip_small_degrees(float clipDeg, float& deg){
if(std::fabs(deg) < clipDeg) deg = 0.f;
}