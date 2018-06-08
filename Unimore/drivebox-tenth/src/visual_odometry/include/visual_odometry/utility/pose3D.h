#ifndef POSE3D_H
#define POSE3D_H

#include <math.h>

class Pose3D {
private:
    void setFromValues(const double x0, const double y0, const double z0,
                       const double yaw, const double pitch, const double roll);
    double wrapToPi(const double a);

protected:

public:
    double   coords[3];
    double   yaw, pitch, roll;
    double   rotation_matrix[3][3];
    bool    ypr_uptodate;

    Pose3D();
    Pose3D(const double x, const double y, const double z,
           const double yaw,const double pitch, const double roll);
    ~Pose3D();

    void rebuildRotationMatrix();
    void updateYawPitchRoll();
    void composeFrom(const Pose3D& a, const Pose3D& b);

    // Traslaction
    void setTranslation(const double x, const double y, const double z);

    // Rotation matrix
    void setRotationMatrix(double *rotation_matrix);

    // Yaw Pitch and Roll
    void setYawPitchRoll(const double yaw, const double pitch, const double roll);

    // Operator
    Pose3D operator+(const Pose3D& b);
};

#endif // POSE3D_H
