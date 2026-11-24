#pragma once
/* Standard DH. Angles in radians, geometry in m. Transform is row-major 4x4. */
void c3_fk(const double q[6],double pose[16]);
double c3_sigma(const double q[6]);
int c3_ik(const double pose[16],const double seed[6],double out[6]);
