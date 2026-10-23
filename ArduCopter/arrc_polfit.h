#pragma once

/*
  ARRC gimbal alignment fit

  A linearly polarized probe rotated by phi about its boresight receives the linear power (mW, not dBm)
      P(phi) = Pco*cos^2(phi - phi0) + Px*sin^2(phi - phi0)
             = D + A*sin(2*phi) + B*cos(2*phi)
  with D = (Pco + Px)/2, A = (Pco - Px)/2*sin(2*phi0) and B = (Pco - Px)/2*cos(2*phi0), where Px is the cross
  polarized power. The model is exact for any phi0 and cross polarization level. D, A and B are found by ordinary
  least squares and the co-polarized peak is at phi0 = atan2(A, B)/2.

  Has no ArduPilot dependencies so it can be tested on its own.
*/

#include <math.h>
#include <stdint.h>

/*
  phi_deg:   probe rotation of each sample in degrees
  power:     linear power of each sample, any unit
  n:         number of samples, at least 4 distinct angles
  phi0_deg:  rotation of the co-polarized peak in degrees, -90 to 90
  r:         correlation coefficient between the samples and the fitted curve, 0 to 1
  sigma_deg: standard error of phi0_deg: the residual variance (n - 3 degrees of freedom) propagated through the
             least squares covariance of A and B. Unlike r it says how far phi0 can be off
  returns false if the fit is singular or the power does not vary
*/
inline bool ARRC_fit_polarization(const float *phi_deg, const float *power, uint8_t n, float &phi0_deg, float &r,
                                  float &sigma_deg)
{
    if (n < 4) {
        return false;
    }

    // normalize the power so the sums do not depend on its unit
    double p_max = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (!(power[i] >= 0) || !isfinite(power[i])) {
            return false;
        }
        if (power[i] > p_max) {
            p_max = power[i];
        }
    }
    if (!(p_max > 0)) {
        return false;
    }

    // normal equations X'X b = X'P of P = D + A*sin(2*phi) + B*cos(2*phi)
    double m[3][3] {};
    double v[3] {};
    for (uint8_t i = 0; i < n; i++) {
        const float two_phi = 2.0f * phi_deg[i] * float(M_PI / 180.0);
        const double x[3] {1.0, sinf(two_phi), cosf(two_phi)};
        const double p = power[i] / p_max;
        for (uint8_t j = 0; j < 3; j++) {
            for (uint8_t k = 0; k < 3; k++) {
                m[j][k] += x[j] * x[k];
            }
            v[j] += x[j] * p;
        }
    }

    // invert X'X by Gauss-Jordan elimination with partial pivoting
    double inv[3][3] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (uint8_t col = 0; col < 3; col++) {
        uint8_t pivot = col;
        for (uint8_t row = col + 1; row < 3; row++) {
            if (fabs(m[row][col]) > fabs(m[pivot][col])) {
                pivot = row;
            }
        }
        if (fabs(m[pivot][col]) < 1e-9 * n) {
            return false;
        }
        if (pivot != col) {
            for (uint8_t k = 0; k < 3; k++) {
                double tmp = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = tmp;
                tmp = inv[col][k];
                inv[col][k] = inv[pivot][k];
                inv[pivot][k] = tmp;
            }
        }
        const double d = m[col][col];
        for (uint8_t k = 0; k < 3; k++) {
            m[col][k] /= d;
            inv[col][k] /= d;
        }
        for (uint8_t row = 0; row < 3; row++) {
            if (row == col) {
                continue;
            }
            const double f = m[row][col];
            for (uint8_t k = 0; k < 3; k++) {
                m[row][k] -= f * m[col][k];
                inv[row][k] -= f * inv[col][k];
            }
        }
    }
    double coef[3];
    for (uint8_t j = 0; j < 3; j++) {
        coef[j] = inv[j][0] * v[0] + inv[j][1] * v[1] + inv[j][2] * v[2];
    }
    const double D = coef[0];
    const double A = coef[1];
    const double B = coef[2];

    // goodness of fit
    double mean = 0;
    for (uint8_t i = 0; i < n; i++) {
        mean += power[i] / p_max;
    }
    mean /= n;
    double ss_tot = 0;
    double ss_res = 0;
    for (uint8_t i = 0; i < n; i++) {
        const float two_phi = 2.0f * phi_deg[i] * float(M_PI / 180.0);
        const double p = power[i] / p_max;
        const double fit = D + A * sinf(two_phi) + B * cosf(two_phi);
        ss_tot += (p - mean) * (p - mean);
        ss_res += (p - fit) * (p - fit);
    }
    const double ab2 = A * A + B * B;
    if (!(ss_tot > 1e-12) || !(ab2 > 1e-18)) {
        return false;
    }
    const double r2 = 1.0 - ss_res / ss_tot;
    r = (r2 > 0) ? sqrtf(float(r2)) : 0.0f;

    // standard error of phi0 = atan2(A, B) / 2
    const double s2 = ss_res / (n - 3);
    const double d_a = 0.5 * B / ab2;
    const double d_b = -0.5 * A / ab2;
    const double var = s2 * (d_a * d_a * inv[1][1] + 2.0 * d_a * d_b * inv[1][2] + d_b * d_b * inv[2][2]);
    sigma_deg = float(sqrt(var > 0 ? var : 0) * (180.0 / M_PI));

    phi0_deg = 0.5f * atan2f(float(A), float(B)) * float(180.0 / M_PI);
    return isfinite(phi0_deg) && isfinite(r) && isfinite(sigma_deg);
}
