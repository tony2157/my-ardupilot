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
  phi_deg:  probe rotation of each sample in degrees
  power:    linear power of each sample, any unit
  n:        number of samples, at least 3 distinct angles
  phi0_deg: rotation of the co-polarized peak in degrees, -90 to 90
  r:        correlation coefficient between the samples and the fitted curve, 0 to 1
  returns false if the fit is singular or the power does not vary
*/
inline bool ARRC_fit_polarization(const float *phi_deg, const float *power, uint8_t n, float &phi0_deg, float &r)
{
    if (n < 3) {
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

    // normal equations [X'X | X'P] of P = D + A*sin(2*phi) + B*cos(2*phi)
    double m[3][4] {};
    for (uint8_t i = 0; i < n; i++) {
        const float two_phi = 2.0f * phi_deg[i] * float(M_PI / 180.0);
        const double v[3] {1.0, sinf(two_phi), cosf(two_phi)};
        const double p = power[i] / p_max;
        for (uint8_t j = 0; j < 3; j++) {
            for (uint8_t k = 0; k < 3; k++) {
                m[j][k] += v[j] * v[k];
            }
            m[j][3] += v[j] * p;
        }
    }

    // Gauss-Jordan elimination with partial pivoting
    for (uint8_t col = 0; col < 3; col++) {
        uint8_t pivot = col;
        for (uint8_t row = col + 1; row < 3; row++) {
            if (fabsf(float(m[row][col])) > fabsf(float(m[pivot][col]))) {
                pivot = row;
            }
        }
        if (fabsf(float(m[pivot][col])) < 1e-9f * n) {
            return false;
        }
        if (pivot != col) {
            for (uint8_t k = 0; k < 4; k++) {
                const double tmp = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = tmp;
            }
        }
        for (uint8_t row = 0; row < 3; row++) {
            if (row == col) {
                continue;
            }
            const double f = m[row][col] / m[col][col];
            for (uint8_t k = col; k < 4; k++) {
                m[row][k] -= f * m[col][k];
            }
        }
    }
    const double D = m[0][3] / m[0][0];
    const double A = m[1][3] / m[1][1];
    const double B = m[2][3] / m[2][2];

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
    if (!(ss_tot > 1e-12)) {
        return false;
    }
    const double r2 = 1.0 - ss_res / ss_tot;
    r = (r2 > 0) ? sqrtf(float(r2)) : 0.0f;

    phi0_deg = 0.5f * atan2f(float(A), float(B)) * float(180.0 / M_PI);
    return isfinite(phi0_deg) && isfinite(r);
}
