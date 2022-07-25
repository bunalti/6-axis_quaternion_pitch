#include "sensor_processing_lib.h"


Quaternion quaternion_from_accelerometer(float ax, float ay, float az)
{
    /*vector_ijk gravity = vector_3d_initialize(0.0f, 0.0f, -1.0f);
    vector_ijk accelerometer = vector_3d_initialize(ax, ay, az);
    Quaternion orientation = quaternion_between_vectors(gravity,accelerometer);
    return orientation;*/
    float norm_u_norm_v = 1.0;
    float cos_theta = -1.0*az;
    //float half_cos = sqrt(0.5*(1.0 + cos_theta));
    float half_cos = 0.7071*sqrt(1.0 + cos_theta);
    Quaternion orientation;
    orientation.a = half_cos;
    //float temp = 1/(2.0*half_cos);
    float temp = 0.5/half_cos;
    orientation.b = -ay*temp;
    orientation.c = ax*temp;
    orientation.d = 0.0;
    return orientation;
}

// Symbolize degrees of rotation in quaternion universe (take integral of gyro data)
Quaternion quaternion_from_gyro(float wx, float wy, float wz, float time)
{
    // wx,wy,wz in radians per second: time in seconds
    float alpha = 0.5*time;
    float a,b,c,d;
    b = alpha*(-wx);
    c = alpha*(-wy);
    d = alpha*(-wz);
    a = 1 - 0.5*(b*b+c*c+d*d);
    Quaternion result = quaternion_initialize(a,b,c,d);
    return result;
}

float fusion_coeffecient(vector_ijk virtual_gravity, vector_ijk sensor_gravity)
{
    float dot = vector_3d_dot_product(sensor_gravity,virtual_gravity);
    /**
     * dot is cosine of the angle between gyro gravity vector
     * and accel gravity vector returns
     * cos() <= ~16.5deg  then increase gyro gravity vector weight
     * 
     * This eliminates accel jiggle using gyro
     * 
     * Adjust these to change the effect of gyro sensor
     * 
     **/
    if (dot<=0.96)
        return 40.0;

    return 10.0;
}

vector_ijk sensor_gravity_normalized(int16_t ax, int16_t ay, int16_t az)
{
    vector_ijk result;
    result.a = ax;
    result.b = ay;
    result.c = az;
    result = vector_3d_normalize(result);
    return result;
}

vector_ijk fuse_vector(vector_ijk virtual_gravity, vector_ijk sensor_gravity)
{
    float fusion = fusion_coeffecient(virtual_gravity, sensor_gravity);
    virtual_gravity = vector_3d_scale(virtual_gravity,fusion);
    vector_ijk result = vector_3d_sum(virtual_gravity,sensor_gravity);
    result = vector_3d_normalize(result);
    return result;
}

vector_ijk update_gravity_vector(vector_ijk gravity_vector,float wx,float wy,float wz,float delta)
{
    Quaternion q_gyro = quaternion_from_gyro(wx,wy,wz,delta);
    // Rotate ,initially (0,0,-1), if rotational speed is present
    gravity_vector = quaternion_rotate_vector(gravity_vector,q_gyro);
    return gravity_vector;
}

vector_ijk update_fused_vector(vector_ijk fused_vector, int16_t ax, int16_t ay, int16_t az,float wx,float wy,float wz,float delta)
{
    // virtual_gravity is rotated fused_vector using data from gyro
    vector_ijk virtual_gravity = update_gravity_vector(fused_vector,wx,wy,wz,delta);
    // sensor_gravity is normalized accel data
    vector_ijk sensor_gravity = sensor_gravity_normalized(ax,ay,az);
    fused_vector = fuse_vector(virtual_gravity,sensor_gravity);
    return fused_vector;
}
