<<<<<<< HEAD

#define ORIENTATION_H

#include <Arduino.h>

namespace MyQuaternion {
    class Quaternion {
    public:
        // Constructor for 4 parameters
        Quaternion(float w, float x, float y, float z) : q{w, x, y, z} {}
        
        // Constructor for 3 parameters (roll, pitch, yaw in radians)
        Quaternion(float roll, float pitch, float yaw) {
            // Convert Euler angles to quaternion
            float cy = cos(yaw * 0.5);
            float sy = sin(yaw * 0.5);
            float cp = cos(pitch * 0.5);
            float sp = sin(pitch * 0.5);
            float cr = cos(roll * 0.5);
            float sr = sin(roll * 0.5);

            q[0] = cr * cp * cy + sr * sp * sy; // w
            q[1] = sr * cp * cy - cr * sp * sy; // x
            q[2] = cr * sp * cy + sr * cp * sy; // y
            q[3] = cr * cp * sy - sr * sp * cy; // z
        }
        
        float* getQ() { return q; }
        
    private:
        float q[4];
    };
}

// Helper functions for compatibility (declarations only)
int appendValue2Str(char *buffer, int bufSize, int curIndex, float value, int decimalPlaces);
int appendValues2Str(char *buffer, int bufSize, int curIndex, const float *values, int numValues, int decimalPlaces);
void write_big_endian(uint8_t *dst, uint8_t *src, size_t len);

=======
#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <Arduino.h>

namespace MyQuaternion {
    class Quaternion {
    public:
        // Constructor for 4 parameters
        Quaternion(float w, float x, float y, float z) : q{w, x, y, z} {}
        
        // Constructor for 3 parameters (roll, pitch, yaw in radians)
        Quaternion(float roll, float pitch, float yaw) {
            // Convert Euler angles to quaternion
            float cy = cos(yaw * 0.5);
            float sy = sin(yaw * 0.5);
            float cp = cos(pitch * 0.5);
            float sp = sin(pitch * 0.5);
            float cr = cos(roll * 0.5);
            float sr = sin(roll * 0.5);

            q[0] = cr * cp * cy + sr * sp * sy; // w
            q[1] = sr * cp * cy - cr * sp * sy; // x
            q[2] = cr * sp * cy + sr * cp * sy; // y
            q[3] = cr * cp * sy - sr * sp * cy; // z
        }
        
        float* getQ() { return q; }
        
    private:
        float q[4];
    };
}

// Helper functions for compatibility (declarations only)
int appendValue2Str(char *buffer, int bufSize, int curIndex, float value, int decimalPlaces);
int appendValues2Str(char *buffer, int bufSize, int curIndex, const float *values, int numValues, int decimalPlaces);
void write_big_endian(uint8_t *dst, uint8_t *src, size_t len);

>>>>>>> a53aa67a6dee201668c571b3e5861cfe6dcbfc84
#endif