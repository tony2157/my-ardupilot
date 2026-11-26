#pragma once

/*
  ARRC gimbal alignment fit

  A linearly polarized probe rotated by phi about its boresight receives the linear power (mW, not dBm)
      P(phi) = Pco*cos^2(phi - phi0) + Px*sin^2(phi - phi0)
             = D + A*sin(2*phi) + B*cos(2*phi)
  with D = (Pco + Px)/2, A = (Pco - Px)/2*sin(2*phi0) and B = (Pco - Px)/2*cos(2*phi0), where Px is the cross
  polarized power. The model is exact for any phi0 and cross polarization level. D, A and B are found by ordinary
  least squares and the co-polarized peak is at phi0 = atan2(A, B)/2.

  The same three terms describe a beam that carries H and V at once with the transmit differential phase
  beta = arg(T_h* T_v), seen by a linear probe at psi from the H axis toward V:
      P(psi) = E_h^2*cos^2(psi) + E_v^2*sin^2(psi) + E_h*E_v*cos(beta)*sin(2*psi)
  D = (E_h^2 + E_v^2)/2, B = (E_h^2 - E_v^2)/2 and A = E_h*E_v*cos(beta), so cos(beta) = A/sqrt(D^2 - B^2). The
  power never shows the sign of sin(beta): only abs(beta) follows (ARRC_tx_diff_phase).

  Three terms is the exact law of an ideal probe. A response of the installed probe that changes with its roll, like
  1 + a*cos(2*psi), multiplies that curve and adds sin(4*psi) and cos(4*psi) terms. ARRC_fit_polarization_terms_4psi
  fits them too, so that they do not bias D, A and B; beta still comes from D, A and B.

  Has no ArduPilot dependencies so it can be tested on its own.
*/

#include <math.h>
#include <stdint.h>

// terms of P = D + A*sin(2*phi) + B*cos(2*phi) fitted to the power normalized by its largest sample (p_max)
struct ARRC_PolFitTerms {
    double D, A, B;         // terms in units of p_max
    double inv[3][3];       // (X'X)^-1 of the regressors 1, sin(2*phi), cos(2*phi)
    double s2;              // residual variance (n - 3 degrees of freedom): the covariance of D, A, B is s2*inv
    double ss_tot;          // total and residual sums of squares
    double ss_res;
    double p_max;           // power unit of the terms
    float r;                // correlation coefficient between the samples and the fitted curve, 0 to 1
};

/*
  phi_deg: probe rotation of each sample in degrees
  power:   linear power of each sample, any unit
  n:       number of samples, at least 4 distinct angles
  returns false if a power is negative or not finite, all are zero, or the fit is singular
*/
inline bool ARRC_fit_polarization_terms(const float *phi_deg, const float *power, uint8_t n, ARRC_PolFitTerms &t)
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
    t.D = coef[0];
    t.A = coef[1];
    t.B = coef[2];
    for (uint8_t j = 0; j < 3; j++) {
        for (uint8_t k = 0; k < 3; k++) {
            t.inv[j][k] = inv[j][k];
        }
    }
    t.p_max = p_max;

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
        const double fit = t.D + t.A * sinf(two_phi) + t.B * cosf(two_phi);
        ss_tot += (p - mean) * (p - mean);
        ss_res += (p - fit) * (p - fit);
    }
    t.ss_tot = ss_tot;
    t.ss_res = ss_res;
    t.s2 = ss_res / (n - 3);
    const double r2 = (ss_tot > 1e-12) ? 1.0 - ss_res / ss_tot : 0.0;
    t.r = (r2 > 0) ? sqrtf(float(r2)) : 0.0f;
    return true;
}

/*
  The three terms plus C*sin(4*phi) + E*cos(4*phi), by least squares on the relative error: each sample is weighted by
  1/power, which is the fit of the power in dB to first order and gives the null of the sweep the same say as the
  peak. One linear solve of five equations.

  phi_deg, power, n: as ARRC_fit_polarization_terms; at least 7 distinct angles spread over 90 deg or more, or the
           terms are poorly separated (the caller checks the sweep)
  t:       D, A and B with the 3x3 part of the covariance that belongs to them and the variance of the relative
           residuals (n - 5 degrees of freedom), ready for ARRC_tx_diff_phase; r is that of the weighted residuals
  c4, e4:  the sin(4*phi) and cos(4*phi) terms in units of t.p_max
  returns false if a power is not positive and finite, or the fit is singular
*/
inline bool ARRC_fit_polarization_terms_4psi(const float *phi_deg, const float *power, uint8_t n, ARRC_PolFitTerms &t,
                                             double &c4, double &e4)
{
    const uint8_t NT = 5;
    if (n < NT + 2) {
        return false;
    }
    double p_max = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (!(power[i] > 0) || !isfinite(power[i])) {
            return false;
        }
        if (power[i] > p_max) {
            p_max = power[i];
        }
    }

    // weighted normal equations: rows and power divided by the power, floored 40 dB under the largest sample
    double m[NT][NT] {};
    double v[NT] {};
    for (uint8_t i = 0; i < n; i++) {
        const double two_phi = 2.0 * phi_deg[i] * (M_PI / 180.0);
        const double x[NT] {1.0, sin(two_phi), cos(two_phi), sin(2 * two_phi), cos(2 * two_phi)};
        const double p = power[i] / p_max;
        const double w = 1.0 / ((p > 1e-4) ? p : 1e-4);
        for (uint8_t j = 0; j < NT; j++) {
            for (uint8_t k = 0; k < NT; k++) {
                m[j][k] += w * w * x[j] * x[k];
            }
            v[j] += w * w * x[j] * p;
        }
    }

    // invert by Gauss-Jordan elimination with partial pivoting
    double inv[NT][NT] {};
    for (uint8_t j = 0; j < NT; j++) {
        inv[j][j] = 1;
    }
    for (uint8_t col = 0; col < NT; col++) {
        uint8_t pivot = col;
        for (uint8_t row = col + 1; row < NT; row++) {
            if (fabs(m[row][col]) > fabs(m[pivot][col])) {
                pivot = row;
            }
        }
        if (fabs(m[pivot][col]) < 1e-9 * n) {
            return false;
        }
        if (pivot != col) {
            for (uint8_t k = 0; k < NT; k++) {
                double tmp = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = tmp;
                tmp = inv[col][k];
                inv[col][k] = inv[pivot][k];
                inv[pivot][k] = tmp;
            }
        }
        const double d = m[col][col];
        for (uint8_t k = 0; k < NT; k++) {
            m[col][k] /= d;
            inv[col][k] /= d;
        }
        for (uint8_t row = 0; row < NT; row++) {
            if (row == col) {
                continue;
            }
            const double f = m[row][col];
            for (uint8_t k = 0; k < NT; k++) {
                m[row][k] -= f * m[col][k];
                inv[row][k] -= f * inv[col][k];
            }
        }
    }
    double coef[NT];
    for (uint8_t j = 0; j < NT; j++) {
        coef[j] = 0;
        for (uint8_t k = 0; k < NT; k++) {
            coef[j] += inv[j][k] * v[k];
        }
    }
    t.D = coef[0];
    t.A = coef[1];
    t.B = coef[2];
    c4 = coef[3];
    e4 = coef[4];
    for (uint8_t j = 0; j < 3; j++) {
        for (uint8_t k = 0; k < 3; k++) {
            t.inv[j][k] = inv[j][k];
        }
    }
    t.p_max = p_max;

    // goodness of fit, of the relative residuals
    double mean = 0;
    double sum_w2 = 0;
    for (uint8_t i = 0; i < n; i++) {
        const double p = power[i] / p_max;
        const double w = 1.0 / ((p > 1e-4) ? p : 1e-4);
        mean += w * w * p;
        sum_w2 += w * w;
    }
    mean /= sum_w2;
    double ss_tot = 0;
    double ss_res = 0;
    for (uint8_t i = 0; i < n; i++) {
        const double two_phi = 2.0 * phi_deg[i] * (M_PI / 180.0);
        const double p = power[i] / p_max;
        const double w = 1.0 / ((p > 1e-4) ? p : 1e-4);
        const double fit = coef[0] + coef[1] * sin(two_phi) + coef[2] * cos(two_phi) + coef[3] * sin(2 * two_phi) +
                           coef[4] * cos(2 * two_phi);
        ss_tot += w * w * (p - mean) * (p - mean);
        ss_res += w * w * (p - fit) * (p - fit);
    }
    t.ss_tot = ss_tot;
    t.ss_res = ss_res;
    t.s2 = ss_res / (n - NT);
    const double r2 = (ss_tot > 1e-12) ? 1.0 - ss_res / ss_tot : 0.0;
    t.r = (r2 > 0) ? sqrtf(float(r2)) : 0.0f;
    return isfinite(t.D) && isfinite(t.A) && isfinite(t.B) && isfinite(c4) && isfinite(e4);
}

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
    ARRC_PolFitTerms t;
    if (!ARRC_fit_polarization_terms(phi_deg, power, n, t)) {
        return false;
    }
    const double A = t.A;
    const double B = t.B;
    const double ab2 = A * A + B * B;
    if (!(t.ss_tot > 1e-12) || !(ab2 > 1e-18)) {
        return false;
    }
    r = t.r;

    // standard error of phi0 = atan2(A, B) / 2
    const double d_a = 0.5 * B / ab2;
    const double d_b = -0.5 * A / ab2;
    const double var = t.s2 * (d_a * d_a * t.inv[1][1] + 2.0 * d_a * d_b * t.inv[1][2] + d_b * d_b * t.inv[2][2]);
    sigma_deg = float(sqrt(var > 0 ? var : 0) * (180.0 / M_PI));

    phi0_deg = 0.5f * atan2f(float(A), float(B)) * float(180.0 / M_PI);
    return isfinite(phi0_deg) && isfinite(r) && isfinite(sigma_deg);
}

// transmit differential phase of a beam carrying H and V at once, from the terms of a probe sweep
struct ARRC_TxDiffPhase {
    float beta_deg;         // abs(beta), 0 to 180
    float sigma_deg;        // standard error of beta_deg (NaN when clamped)
    float zdr_db;           // 10*log10(E_h^2/E_v^2)
    float swing_db;         // peak to null ratio of the fitted curve (NaN when clamped)
    float peak_deg;         // probe angle of the fitted peak, -90 to 90
    bool clamped;           // abs(cos(beta)) came out above 1: beta is 0 or 180
};

/*
  t:      terms of the sweep (ARRC_fit_polarization_terms)
  h_sign: +1 if psi = 0 is the probe along +H and psi turns it toward +V, -1 if psi = 0 is along -H (then
          cos(beta) = -A/sqrt(D^2 - B^2))
  returns false if the terms give no positive E_h^2 and E_v^2
*/
inline bool ARRC_tx_diff_phase(const ARRC_PolFitTerms &t, int8_t h_sign, ARRC_TxDiffPhase &out)
{
    const double D = t.D;
    const double A = t.A;
    const double B = t.B;
    if (!(D - B > 0) || !(D + B > 0)) {
        return false;
    }
    const double q = 1.0 / sqrt(D * D - B * B);
    double c = (h_sign < 0 ? -A : A) * q;
    out.zdr_db = float(10.0 * log10((D + B) / (D - B)));
    out.peak_deg = float(0.5 * atan2(A, B) * (180.0 / M_PI));
    out.clamped = !(fabs(c) < 1.0);
    if (out.clamped) {
        c = (c < 0) ? -1.0 : 1.0;
        out.sigma_deg = NAN;
        out.swing_db = NAN;
    } else {
        // gradient of c with respect to D, A and B through the covariance s2*inv
        const double s = (h_sign < 0) ? -1.0 : 1.0;
        const double q3 = q * q * q;
        const double g[3] {-s * A * D * q3, s * q, s * A * B * q3};
        double var_c = 0;
        for (uint8_t j = 0; j < 3; j++) {
            for (uint8_t k = 0; k < 3; k++) {
                var_c += g[j] * t.inv[j][k] * g[k];
            }
        }
        var_c *= t.s2;
        out.sigma_deg = float(sqrt(var_c > 0 ? var_c : 0) / sqrt(1.0 - c * c) * (180.0 / M_PI));
        const double m = sqrt(A * A + B * B);
        out.swing_db = float(10.0 * log10((D + m) / (D - m)));
    }
    out.beta_deg = float(acos(c) * (180.0 / M_PI));
    return isfinite(out.beta_deg) && isfinite(out.zdr_db) && isfinite(out.peak_deg);
}
