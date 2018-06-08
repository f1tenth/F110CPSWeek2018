#include "utility/pose3D.h"

Pose3D::Pose3D() {
    this->x = this->y = this->z = 0;

    // Set rotation matrix as identity matrix
    int     i,j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            if(i == j)
                this->rotation_matrix[i][j] = (double) 1.0f;
            else
                this->rotation_matrix[i][j] = (double) 0.0f;
        }
    }
}

Pose3D::Pose3D(const double x, const double y, const double z,
               const double yaw,const double pitch, const double roll) {
    setFromValues(x, y, z, yaw, pitch, roll);
}

Pose3D::~Pose3D() {

}

void Pose3D::rebuildRotationMatrix() {
    const double cy = cos(this->yaw);
    const double sy = sin(this->yaw);
    const double cp = cos(this->pitch);
    const double sp = sin(this->pitch);
    const double cr = cos(this->roll);
    const double sr = sin(this->roll);

    this->rotation_matrix[0][0] = cy * cp;
    this->rotation_matrix[0][1] = cy * sp * sr - sy * cr;
    this->rotation_matrix[0][2] = cy * sp * cr + sy * sr;
    this->rotation_matrix[1][0] = sy * cp;
    this->rotation_matrix[1][1] = sy * sp * sr + cy * cr;
    this->rotation_matrix[1][2] = sy * sp * cr - cy * sr;
    this->rotation_matrix[2][0] = -sp;
    this->rotation_matrix[2][1] = cp * sr;
    this->rotation_matrix[2][2] = cp * cr;
}

void Pose3D::updateYawPitchRoll() {
    if (!m_ypr_uptodate) {
        m_ypr_uptodate = true;
        //getYawPitchRoll(m_yaw, m_pitch, m_roll); // TODO è un macello
    }
}

void Pose3D::setFromValues(const double x0, const double y0, const double z0,
                           const double yaw, const double pitch, const double roll) {
    this->coords[0] = x0;
    this->coords[1] = y0;
    this->coords[2] = z0;
    this->yaw = wrapToPi(yaw);
    this->pitch = wrapToPi(pitch);
    this->roll = wrapToPi(roll);

    ypr_uptodate = true;

    rebuildRotationMatrix();
}

double wrapToPi(const double a) {
    double b = a + static_cast<double>(M_PI);
    bool was_neg = b < 0;
    b = fmod(b, static_cast<double>(2.0 * M_PI));
    if (was_neg)
        b += static_cast<double>(2.0 * M_PI);
    return b + static_cast<double>(M_PI);
}

void Pose3D::composeFrom(const Pose3D& a, const Pose3D& b) {
    // The translation part HM(0:3,3)
    if (this == &b) {
        // we need to make a temporary copy of the vector:
        const double b_coords[3];
        b_coords[0] = b.coords[0];
        b_coords[1] = b.coords[1];
        b_coords[2] = b.coords[2];
        for (int r = 0; r < 3; r++)
            coords[r] = a.coords[r] + a.rotation_matrix[r][0] * b_coords[0] +
                                      a.rotation_matrix[r][1] * b_coords[1] +
                                      a.rotation_matrix[r][2] * b_coords[2];
    } else {
        for (int r = 0; r < 3; r++)
            coords[r] = a.coords[r] + a.rotation_matrix[r][0] * b.coords[0] +
                                      a.rotation_matrix[r][1] * b.coords[1] +
                                      a.rotation_matrix[r][2] * b.coords[2];
    }

    // Important: Make this multiplication AFTER the translational part, to cope
    // with the case when A==this
    int     i, j, k;
    for(i = 0; i < 3; ++i)
        for(j = 0; j < 3; ++j)
            for(k = 0; k < 3; ++k)
                this->rotation_matrix[i][j] += a.rotation_matrix[i][k] * b.rotation_matrix[k][j];


    ypr_uptodate = false;
}

// Traslaction
void Pose3D::setTranslation(const double x, const double y, const double z) {
    this->coords[0] = x;
    this->coords[1] = y;
    this->coords[2] = z;
}

// Rotation matrix
void Pose3D::setRotationMatrix(double *rotation_matrix) {
    this->rotation_matrix = rotation_matrix;
    ypr_uptodate = false;
}

// Yaw Pitch and Roll
void Pose3D::setYawPitchRoll(const double yaw, const double pitch, const double roll) {

}

// Operator
Pose3D Pose3D::operator+(const Pose3D& b) {
    Pose3D ret;
    ret.composeFrom(*this, b);
    return ret;
}
