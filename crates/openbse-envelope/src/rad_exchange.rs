//! Interior long-wave radiation exchange between a zone's surfaces.
//!
//! EnergyPlus-style grey-interchange ("ScriptF") model, built once at model
//! construction, matching `HeatBalanceIntRadExchange`:
//!
//! 1. **Approximate view factors** (`CalcApproximateViewFactors`): surface i
//!    sees surface j in proportion to j's area; surfaces facing the same way
//!    (azimuth and tilt both within 10°) don't see each other.
//! 2. **Fix** (`FixViewFactors`): alternately enforce reciprocity
//!    `A_i·F_ij = A_j·F_ji` and completeness `Σ_j F_ij = 1` until both hold.
//! 3. **ScriptF** (`CalcScriptF`): fold inter-reflections into total grey
//!    interchange factors via the Gebhart matrix solve,
//!    `B = (I − F·diag(ρ))⁻¹ · F · diag(ε)`, `𝔉_ij = ε_i·B_ij`.
//!
//! The net long-wave flux leaving surface i is then
//! `q_i = σ · Σ_j 𝔉_ij · (T_i⁴ − T_j⁴)` [W/m² of i], and `A_i·𝔉_ij = A_j·𝔉_ji`
//! makes the zone's interior exchange sum to exactly zero.
//!
//! Only areas, orientations and emissivities are needed, so every zone gets
//! this exchange — box-shaped or not, with or without vertex data.

/// Stefan-Boltzmann constant [W/m²·K⁴] (EnergyPlus value).
pub const SIGMA: f64 = 5.6704e-8;

/// Two surfaces whose azimuth and tilt both differ by less than this [°]
/// face the same way and don't see each other (EnergyPlus SameAngleLimit).
const SAME_ANGLE_LIMIT: f64 = 10.0;

/// Grey interchange factors for one zone's surfaces.
#[derive(Debug, Clone)]
pub struct ZoneRadExchange {
    /// Number of surfaces (order follows the zone's surface list).
    pub n: usize,
    /// 𝔉_ij, row-major n×n. `q_i = σ·Σ_j 𝔉_ij·(T_i⁴ − T_j⁴)` [W/m² of i].
    pub script_f: Vec<f64>,
    /// Row sums `w_i = Σ_j 𝔉_ij` (≤ ε_i).
    pub w: Vec<f64>,
}

impl ZoneRadExchange {
    /// Effective radiant temperature [°C] surface `pos` exchanges with, from
    /// the other surfaces' `(T+273.15)⁴` values: `T_mrt⁴ = Σ_j 𝔉_ij·T_j⁴ / w_i`.
    /// `None` when this surface exchanges with nothing (w ≈ 0).
    pub fn radiant_temp(&self, pos: usize, t4: &[f64]) -> Option<f64> {
        let w = self.w[pos];
        if w <= 1.0e-12 {
            return None;
        }
        let row = &self.script_f[pos * self.n..(pos + 1) * self.n];
        let num: f64 = row.iter().zip(t4).map(|(f, t)| f * t).sum();
        Some((num / w).powf(0.25) - 273.15)
    }
}

/// Linearized exchange coefficient `h` [W/m²K] such that `h·(T_mrt − T_s)`
/// equals the exact net flux `σ·w·(T_mrt⁴ − T_s⁴)` at these temperatures
/// (secant linearization; both temperatures in °C).
pub fn lw_coeff(w: f64, t_surf: f64, t_mrt: f64) -> f64 {
    if w <= 0.0 {
        return 0.0;
    }
    let t1 = (t_surf + 273.15).max(200.0);
    let t2 = (t_mrt + 273.15).max(200.0);
    let dt = (t1 - t2).abs();
    if dt > 0.1 {
        SIGMA * w * (t1.powi(4) - t2.powi(4)).abs() / dt
    } else {
        4.0 * SIGMA * w * ((t1 + t2) / 2.0).powi(3)
    }
}

/// Build the interchange factors for one zone. `None` when fewer than two
/// surfaces have positive area (nothing to exchange with).
pub fn build(areas: &[f64], azimuths: &[f64], tilts: &[f64], emissivities: &[f64]) -> Option<ZoneRadExchange> {
    let n = areas.len();
    if n < 2 || areas.iter().filter(|&&a| a > 1.0e-6).count() < 2 {
        return None;
    }

    let mut f = approximate_view_factors(areas, azimuths, tilts);
    fix_view_factors(&mut f, areas, n);
    let script_f = calc_script_f(&f, areas, emissivities, n)?;
    let w: Vec<f64> = (0..n)
        .map(|i| script_f[i * n..(i + 1) * n].iter().sum::<f64>())
        .collect();
    Some(ZoneRadExchange { n, script_f, w })
}

/// EnergyPlus `CalcApproximateViewFactors`: `F_ij ∝ A_j` over the surfaces
/// that i can see (every j that doesn't face the same way as i), row-normalized.
fn approximate_view_factors(areas: &[f64], azimuths: &[f64], tilts: &[f64]) -> Vec<f64> {
    let n = areas.len();
    let mut f = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            if i == j || same_orientation(azimuths[i], tilts[i], azimuths[j], tilts[j]) {
                continue;
            }
            f[i * n + j] = areas[j];
        }
        let row_sum: f64 = f[i * n..(i + 1) * n].iter().sum();
        if row_sum > 1.0e-12 {
            for j in 0..n {
                f[i * n + j] /= row_sum;
            }
        }
    }
    f
}

fn same_orientation(az_i: f64, tilt_i: f64, az_j: f64, tilt_j: f64) -> bool {
    if (tilt_i - tilt_j).abs() >= SAME_ANGLE_LIMIT {
        return false;
    }
    // A face-up or face-down surface has no meaningful azimuth: compare on
    // tilt alone near horizontal, like E+ does for floors and ceilings.
    if tilt_i.min(tilt_j) < SAME_ANGLE_LIMIT || tilt_i.max(tilt_j) > 180.0 - SAME_ANGLE_LIMIT {
        return true;
    }
    let mut d_az = (az_i - az_j).abs() % 360.0;
    if d_az > 180.0 {
        d_az = 360.0 - d_az;
    }
    d_az < SAME_ANGLE_LIMIT
}

/// EnergyPlus `FixViewFactors`: alternate enforcing reciprocity
/// (`A_i·F_ij = A_j·F_ji`, by averaging the two sides of each pair) and
/// completeness (each row sums to 1) until both hold.
fn fix_view_factors(f: &mut [f64], areas: &[f64], n: usize) {
    for _ in 0..100 {
        // Reciprocity: replace A_i·F_ij and A_j·F_ji by their mean.
        for i in 0..n {
            for j in (i + 1)..n {
                if areas[i] <= 1.0e-12 || areas[j] <= 1.0e-12 {
                    continue;
                }
                let af = (areas[i] * f[i * n + j] + areas[j] * f[j * n + i]) / 2.0;
                f[i * n + j] = af / areas[i];
                f[j * n + i] = af / areas[j];
            }
        }
        // Completeness: row-normalize.
        let mut worst = 0.0_f64;
        for i in 0..n {
            let row_sum: f64 = f[i * n..(i + 1) * n].iter().sum();
            if row_sum > 1.0e-12 {
                worst = worst.max((row_sum - 1.0).abs());
                for j in 0..n {
                    f[i * n + j] /= row_sum;
                }
            }
        }
        // Row sums are now exact; converged once reciprocity survived the
        // normalization (the normalization barely moved anything).
        if worst < 1.0e-6 {
            break;
        }
    }
}

/// EnergyPlus `CalcScriptF` via the Gebhart formulation: solve
/// `(I − F·diag(ρ))·B = F·diag(ε)` for B, then `𝔉_ij = ε_i·B_ij`,
/// symmetrized so `A_i·𝔉_ij = A_j·𝔉_ji` holds exactly.
fn calc_script_f(f: &[f64], areas: &[f64], emissivities: &[f64], n: usize) -> Option<Vec<f64>> {
    // m = I − F·diag(ρ);  rhs = F·diag(ε)
    let mut m = vec![0.0; n * n];
    let mut b = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            let eps_j = emissivities[j].clamp(0.01, 1.0);
            m[i * n + j] = (if i == j { 1.0 } else { 0.0 }) - f[i * n + j] * (1.0 - eps_j);
            b[i * n + j] = f[i * n + j] * eps_j;
        }
    }
    solve_matrix(&mut m, &mut b, n)?;
    let mut g = vec![0.0; n * n];
    for i in 0..n {
        let eps_i = emissivities[i].clamp(0.01, 1.0);
        for j in 0..n {
            g[i * n + j] = eps_i * b[i * n + j];
        }
    }
    // Symmetrize A_i·𝔉_ij (removes the last of any numerical asymmetry so the
    // zone exchange sums to exactly zero), and clamp tiny negatives.
    for i in 0..n {
        for j in (i + 1)..n {
            if areas[i] <= 1.0e-12 || areas[j] <= 1.0e-12 {
                continue;
            }
            let ag = ((areas[i] * g[i * n + j] + areas[j] * g[j * n + i]) / 2.0).max(0.0);
            g[i * n + j] = ag / areas[i];
            g[j * n + i] = ag / areas[j];
        }
        g[i * n + i] = g[i * n + i].max(0.0);
    }
    Some(g)
}

/// Solve `M·X = B` in place (B becomes X) by Gaussian elimination with
/// partial pivoting. Returns `None` on a singular matrix.
fn solve_matrix(m: &mut [f64], b: &mut [f64], n: usize) -> Option<()> {
    for col in 0..n {
        // Pivot
        let mut pivot = col;
        for row in (col + 1)..n {
            if m[row * n + col].abs() > m[pivot * n + col].abs() {
                pivot = row;
            }
        }
        if m[pivot * n + col].abs() < 1.0e-12 {
            return None;
        }
        if pivot != col {
            for k in 0..n {
                m.swap(col * n + k, pivot * n + k);
                b.swap(col * n + k, pivot * n + k);
            }
        }
        let diag = m[col * n + col];
        for k in 0..n {
            m[col * n + k] /= diag;
            b[col * n + k] /= diag;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = m[row * n + col];
            if factor == 0.0 {
                continue;
            }
            for k in 0..n {
                m[row * n + k] -= factor * m[col * n + k];
                b[row * n + k] -= factor * b[col * n + k];
            }
        }
    }
    Some(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two equal parallel plates: 𝔉₁₂ = 1/(1/ε₁ + 1/ε₂ − 1), the classic
    /// grey parallel-plate interchange factor.
    #[test]
    fn test_script_f_parallel_plates() {
        let areas = [10.0, 10.0];
        let f = [0.0, 1.0, 1.0, 0.0];
        let eps = [0.9, 0.84];
        let g = calc_script_f(&f, &areas, &eps, 2).unwrap();
        let expected = 1.0 / (1.0 / 0.9 + 1.0 / 0.84 - 1.0);
        assert!((g[1] - expected).abs() < 1.0e-10, "got {}, want {}", g[1], expected);
        assert!((g[2] - expected).abs() < 1.0e-10);
    }

    /// Black surfaces (ε=1): ScriptF reduces to the view factors themselves.
    #[test]
    fn test_script_f_black_surfaces() {
        let areas = [4.0, 2.0, 2.0];
        let f = [0.0, 0.5, 0.5, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0];
        let eps = [1.0, 1.0, 1.0];
        let g = calc_script_f(&f, &areas, &eps, 3).unwrap();
        for (gi, fi) in g.iter().zip(f.iter()) {
            assert!((gi - fi).abs() < 1.0e-10);
        }
    }

    /// A 4×3×2.7 m box zone: rows sum to 1, reciprocity holds, and the
    /// interchange factors satisfy A_i·𝔉_ij = A_j·𝔉_ji with Σ_j 𝔉_ij ≤ ε_i.
    #[test]
    fn test_box_zone_fix_and_reciprocity() {
        let (l, w, h) = (4.0, 3.0, 2.7);
        let areas = [l * w, l * w, l * h, l * h, w * h, w * h];
        let azimuths = [0.0, 0.0, 180.0, 0.0, 90.0, 270.0];
        let tilts = [180.0, 0.0, 90.0, 90.0, 90.0, 90.0];
        let eps = [0.9; 6];
        let zx = build(&areas, &azimuths, &tilts, &eps).unwrap();
        let n = zx.n;
        for i in 0..n {
            // Σ_j 𝔉_ij ≤ ε_i for exact view factors; the fixed approximate
            // factors can overshoot by a numerical hair.
            assert!(zx.w[i] <= eps[i] + 1.0e-5, "w[{i}] = {} > ε", zx.w[i]);
            assert!(zx.w[i] > 0.5, "w[{i}] = {} unexpectedly small", zx.w[i]);
            for j in 0..n {
                let lhs = areas[i] * zx.script_f[i * n + j];
                let rhs = areas[j] * zx.script_f[j * n + i];
                assert!((lhs - rhs).abs() < 1.0e-9, "reciprocity broken at ({i},{j})");
            }
        }
    }

    /// An odd-shaped (U-ish) zone with unequal surfaces and windows:
    /// whatever the temperatures, the zone's net interior long-wave
    /// exchange sums to zero — the bug the box model had.
    #[test]
    fn test_energy_conservation_odd_zone() {
        let areas = [48.0, 48.0, 10.8, 5.4, 8.1, 2.7, 13.5, 4.0];
        let azimuths = [0.0, 0.0, 180.0, 90.0, 0.0, 90.0, 247.0, 180.0];
        let tilts = [180.0, 0.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0];
        let eps = [0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.84];
        let zx = build(&areas, &azimuths, &tilts, &eps).unwrap();
        let temps = [19.0, 23.5, 15.2, 21.0, 18.3, 25.0, 12.0, 6.5];
        let t4: Vec<f64> = temps.iter().map(|t| (t + 273.15_f64).powi(4)).collect();
        let mut net = 0.0;
        for i in 0..zx.n {
            let mut q = 0.0;
            for j in 0..zx.n {
                q += SIGMA * zx.script_f[i * zx.n + j] * (t4[i] - t4[j]);
            }
            net += areas[i] * q;
        }
        let scale: f64 = areas
            .iter()
            .zip(&t4)
            .map(|(a, t)| a * SIGMA * t)
            .sum();
        assert!(net.abs() < scale * 1.0e-10, "net exchange {net} W");
    }

    /// Coplanar surfaces (a wall and its window, two floor patches) get no
    /// direct view of each other, but still exchange via reflections.
    #[test]
    fn test_same_orientation_zeroed_in_f() {
        let areas = [20.0, 4.0, 24.0, 24.0];
        let azimuths = [180.0, 180.0, 0.0, 0.0];
        let tilts = [90.0, 90.0, 180.0, 0.0];
        let f = approximate_view_factors(&areas, &azimuths, &tilts);
        assert_eq!(f[1], 0.0, "wall shouldn't see its coplanar window");
        assert_eq!(f[4], 0.0, "window shouldn't see its parent wall");
        assert!(f[2] > 0.0 && f[3] > 0.0);
    }

    /// The secant coefficient reproduces the exact quartic flux.
    #[test]
    fn test_lw_coeff_matches_quartic() {
        let (w, ts, tm) = (0.85, 18.0, 22.5);
        let h = lw_coeff(w, ts, tm);
        let exact = SIGMA * w * ((tm + 273.15_f64).powi(4) - (ts + 273.15_f64).powi(4));
        assert!((h * (tm - ts) - exact).abs() < 1.0e-9);
    }

    #[test]
    fn test_single_surface_zone_builds_none() {
        assert!(build(&[10.0], &[0.0], &[90.0], &[0.9]).is_none());
    }
}
