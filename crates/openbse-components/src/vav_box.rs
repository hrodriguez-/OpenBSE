//! VAV terminal box with optional reheat coil.
//!
//! Models a variable air volume terminal unit that modulates primary airflow
//! between minimum and maximum limits, with optional reheat (hot water or
//! electric) when the zone requires heating.
//!
//! Control sequence (dual maximum — ASHRAE Guideline 36 / E+ ReverseWithLimits):
//!   1. Cooling mode: damper opens from minimum toward maximum flow.
//!      Primary air from the central AHU is cold (typically 12-14°C).
//!      More cold air = more cooling.
//!   2. Deadband: damper at minimum position, no reheat.
//!   3. Heating mode: damper opens from minimum toward max_reheat_fraction,
//!      reheat coil activates. Higher heating demand opens damper wider
//!      for better heat distribution.
//!
//! The VAV box does NOT know zone loads directly — it receives a zone load
//! signal via control_signal (positive = heating demand [W], negative =
//! cooling demand [W]) and modulates accordingly.
//!
//! Reference: EnergyPlus Engineering Reference, "AirTerminal:SingleDuct:VAV:Reheat"

use openbse_core::ports::*;
use openbse_psychrometrics::{self as psych, FluidState};
use serde::{Deserialize, Serialize};

fn default_submeter() -> String {
    "General".to_string()
}

/// Reheat coil type for VAV boxes.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum ReheatType {
    /// No reheat coil (cooling-only VAV box)
    None,
    /// Electric resistance reheat
    Electric,
    /// Hot water reheat (connected to HW plant loop)
    HotWater,
}

/// VAV terminal box with optional reheat.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VAVBox {
    pub name: String,
    #[serde(default = "default_submeter")]
    pub submeter: String,
    /// Zone this VAV box serves
    pub zone_name: String,
    /// Maximum primary air flow rate [kg/s]
    pub max_air_flow: f64,
    /// Minimum air flow fraction [0-1] of max (typically 0.3-0.4)
    pub min_flow_fraction: f64,
    /// Reheat coil type
    pub reheat_type: ReheatType,
    /// Reheat coil capacity [W] (used for Electric and HotWater)
    pub reheat_capacity: f64,
    /// Maximum reheat outlet air temperature [°C] (typically 35-50)
    pub max_reheat_temp: f64,
    /// Maximum flow fraction during reheat [0-1] (E+ "ReverseWithLimits").
    /// In heating mode, the damper opens from min_flow_fraction toward this
    /// value to deliver more reheat energy. Typically 0.5 (ASHRAE G36
    /// dual-maximum). Default: 0.5.
    pub max_reheat_fraction: f64,

    // ─── Hot water reheat parameters ─────────────────────────────────────
    /// Design hot water flow rate [m³/s] (for HotWater reheat)
    pub design_hw_flow_rate: f64,
    /// Design hot water inlet temperature [°C] (typically 82°C / 180°F)
    pub design_hw_inlet_temp: f64,
    /// Design hot water outlet temperature [°C] (typically 71°C / 160°F)
    pub design_hw_outlet_temp: f64,

    // ─── Control signal ──────────────────────────────────────────────────
    /// Zone control signal [-1.0 to +1.0]. Positive = heating demand fraction,
    /// negative = cooling demand fraction. Set by the controls framework before
    /// simulate_air is called. The fraction scales the capacity:
    ///   heating: q_reheat = control_signal × reheat_capacity
    ///   cooling: damper opens proportional to |control_signal|
    #[serde(skip)]
    pub control_signal: f64,
    /// Temperature of the served zone [°C], set by the driver each timestep
    /// (`set_zone_temp`).  `None` until the first call; the box then falls back
    /// to treating the control signal as the coil duty fraction.
    #[serde(skip)]
    pub zone_temp: Option<f64>,
    /// Net zone load to setpoint [W] (+heating / −cooling), set by the driver
    /// each timestep (`set_zone_load`).  When both this and `zone_temp` are
    /// known the box picks its own mode from them instead of `control_signal`.
    #[serde(skip)]
    pub zone_load: Option<f64>,

    // ─── Runtime state ───────────────────────────────────────────────────
    /// Current damper position [0-1]
    #[serde(skip)]
    pub damper_position: f64,
    /// Current primary air mass flow rate [kg/s]
    #[serde(skip)]
    pub primary_air_flow: f64,
    /// Reheat coil heating rate [W]
    #[serde(skip)]
    pub reheat_rate: f64,
    /// Reheat coil power consumption [W] (electric reheat only)
    #[serde(skip)]
    pub reheat_power: f64,

    #[serde(skip)]
    water_inlet: Option<WaterPort>,
    #[serde(skip)]
    water_outlet_state: Option<WaterPort>,
}

impl VAVBox {
    /// Create a new VAV box.
    ///
    /// # Arguments
    /// * `name` - Component name
    /// * `zone_name` - Name of the zone this box serves
    /// * `max_air_flow` - Maximum primary air flow rate [kg/s]
    /// * `min_flow_fraction` - Minimum flow as fraction of max (0.3-0.4 typical)
    /// * `reheat_type` - Type of reheat coil
    /// * `reheat_capacity` - Reheat coil capacity [W]
    pub fn new(
        name: &str,
        zone_name: &str,
        max_air_flow: f64,
        min_flow_fraction: f64,
        reheat_type: ReheatType,
        reheat_capacity: f64,
    ) -> Self {
        Self {
            name: name.to_string(),
            submeter: "General".to_string(),
            zone_name: zone_name.to_string(),
            max_air_flow,
            min_flow_fraction: min_flow_fraction.clamp(0.0, 1.0),
            reheat_type,
            reheat_capacity,
            max_reheat_temp: 40.0, // E+ default Maximum Reheat Air Temperature
            max_reheat_fraction: 0.5,
            design_hw_flow_rate: 0.0,
            design_hw_inlet_temp: 82.0,
            design_hw_outlet_temp: 71.0,
            control_signal: 0.0,
            zone_temp: None,
            zone_load: None,
            damper_position: 0.0,
            primary_air_flow: 0.0,
            reheat_rate: 0.0,
            reheat_power: 0.0,
            water_inlet: None,
            water_outlet_state: None,
        }
    }

    /// Set hot water reheat design parameters.
    pub fn with_hw_reheat_params(
        mut self,
        water_flow_rate: f64,
        water_inlet_temp: f64,
        water_outlet_temp: f64,
    ) -> Self {
        self.design_hw_flow_rate = water_flow_rate;
        self.design_hw_inlet_temp = water_inlet_temp;
        self.design_hw_outlet_temp = water_outlet_temp;
        self
    }

    /// Set maximum reheat air outlet temperature [°C].
    pub fn with_max_reheat_temp(mut self, temp: f64) -> Self {
        self.max_reheat_temp = temp;
        self
    }

    /// Reheat coil duty [W] needed at `flow` so the zone receives the net
    /// heating demand carried by the control signal.
    ///
    /// The primary air arrives at the AHU cold-deck temperature (12-15 °C).
    /// Before the box delivers any net heat it has to lift that air to zone
    /// temperature, so the coil duty is the net demand plus
    /// `m × cp × (T_zone − T_inlet)`.  Without that term a box fed 13 °C air
    /// at minimum flow cools the zone it is meant to heat and the zone never
    /// recovers (the engine's own `vav_reheat` example logged thousands of
    /// unmet heating hours for exactly this reason).
    fn coil_duty_w(&self, t_inlet: f64, cp_air: f64, flow: f64) -> f64 {
        // Net load: the driver's signed zone load when known (it may be a
        // small cooling load that minimum flow over-serves), else the signal.
        let q_net = match (self.zone_temp, self.zone_load) {
            (Some(_), Some(q)) => q,
            _ => self.control_signal.clamp(0.0, 1.0) * self.reheat_capacity,
        };
        let lift = match self.zone_temp {
            Some(t_zone) => (flow * cp_air * (t_zone - t_inlet)).max(0.0),
            None => 0.0,
        };
        (q_net + lift).max(0.0)
    }

    /// Fraction of coil capacity the box will need at the heating maximum
    /// flow — drives the dual-maximum damper ramp.
    fn heating_duty_fraction(&self, t_inlet: f64, cp_air: f64, heat_max_flow: f64) -> f64 {
        if self.reheat_capacity <= 0.0 {
            return self.control_signal.clamp(0.0, 1.0);
        }
        // Only a positive net heating demand opens the damper; tempering a
        // small cooling load happens at minimum flow.  The ramp reaches the
        // heating maximum when the demand uses up what the coil has left
        // after lifting the heating-max flow to zone temperature.
        let q_net = match (self.zone_temp, self.zone_load) {
            (Some(_), Some(q)) => q,
            _ => self.control_signal.clamp(0.0, 1.0) * self.reheat_capacity,
        };
        if q_net <= 0.0 {
            return 0.0;
        }
        let lift_at_max = match self.zone_temp {
            Some(t_zone) => (heat_max_flow * cp_air * (t_zone - t_inlet)).max(0.0),
            None => 0.0,
        };
        let available = (self.reheat_capacity - lift_at_max).max(0.1 * self.reheat_capacity);
        (q_net / available).clamp(0.0, 1.0)
    }
}

impl AirComponent for VAVBox {
    fn name(&self) -> &str {
        &self.name
    }

    fn component_kind(&self) -> ComponentKind {
        ComponentKind::HeatingCoil
    }

    fn simulate_air(&mut self, inlet: &AirPort, _ctx: &SimulationContext) -> AirPort {
        self.reheat_rate = 0.0;
        self.reheat_power = 0.0;
        self.water_outlet_state = None;

        if inlet.mass_flow <= 0.0 || self.max_air_flow <= 0.0 {
            self.damper_position = 0.0;
            self.primary_air_flow = 0.0;
            return *inlet;
        }

        let cp_air = psych::cp_air_fn_w(inlet.state.w);
        let min_flow = self.max_air_flow * self.min_flow_fraction;

        // ─── Load-based control (zone temperature and net load known) ─────
        //
        // E+ "Reverse" reheat action: at minimum flow the cold primary air
        // removes m_min·cp·(T_zone − T_in) from the zone.  If the zone needs
        // less cooling than that (a small cooling load, deadband, or heating),
        // the damper stays at minimum and the coil tempers the air so the zone
        // receives exactly its net load.  Only when the zone needs more
        // cooling than minimum flow provides does the damper open.
        if let (Some(t_zone), Some(q_net)) = (self.zone_temp, self.zone_load) {
            let min_flow_cooling = min_flow * cp_air * (t_zone - inlet.state.t_db);
            let duty_at_min = q_net + min_flow_cooling;
            if duty_at_min > 0.0 && !matches!(self.reheat_type, ReheatType::None) {
                // Heating / tempering: express the net load as a capacity
                // fraction so the existing heating path (dual-maximum ramp,
                // coil duty incl. the cold-air lift) handles it.
                self.control_signal = if self.reheat_capacity > 0.0 {
                    (q_net / self.reheat_capacity)
                        .clamp(0.0, 1.0)
                        .max(f64::MIN_POSITIVE)
                } else {
                    f64::MIN_POSITIVE
                };
            } else if duty_at_min > 0.0 {
                // No coil: nothing to temper with, sit at minimum flow.
                self.control_signal = 0.0;
            } else {
                // Cooling: flow that meets the load at the current supply temp.
                let dt = (t_zone - inlet.state.t_db).max(0.5);
                let m_needed = (-q_net) / (cp_air * dt);
                let frac = ((m_needed - min_flow) / (self.max_air_flow - min_flow).max(1e-6))
                    .clamp(0.0, 1.0);
                self.control_signal = -frac;
            }
        }

        // ─── Determine damper position based on zone load signal ─────────
        // control_signal > 0 → zone needs heating → damper at minimum, reheat on
        // control_signal < 0 → zone needs cooling → damper modulates open
        // control_signal = 0 → deadband → damper at minimum

        let (air_flow, needs_reheat) = if self.control_signal < 0.0 {
            // COOLING MODE: modulate damper from min toward max
            // control_signal is -1.0 (full cooling) to 0.0 (no cooling)
            // |control_signal| directly scales the damper position.
            let cooling_frac = (-self.control_signal).clamp(0.0, 1.0);

            // Linear interpolation between min and max flow
            let flow = min_flow + cooling_frac * (self.max_air_flow - min_flow);
            (flow, false)
        } else if self.control_signal > 0.0 {
            // HEATING MODE: dual-maximum (ReverseWithLimits)
            // Damper opens from min_flow toward max_reheat_fraction for more
            // reheat capacity. Higher coil duty → wider damper.
            let heat_max_flow = self.max_air_flow * self.max_reheat_fraction;
            let heat_frac = self.heating_duty_fraction(inlet.state.t_db, cp_air, heat_max_flow);
            let flow = min_flow + heat_frac * (heat_max_flow - min_flow);
            (flow, true)
        } else {
            // DEADBAND: minimum flow, no reheat
            (min_flow, false)
        };

        self.primary_air_flow = air_flow;
        self.damper_position = if self.max_air_flow > 0.0 {
            air_flow / self.max_air_flow
        } else {
            0.0
        };

        // Scale inlet mass flow to the VAV box's modulated flow
        // (inlet.mass_flow represents what the AHU provides; VAV box takes
        // only what it needs by damper modulation)
        let actual_flow = air_flow.min(inlet.mass_flow);
        let mut outlet_t = inlet.state.t_db;

        // ─── Reheat coil ─────────────────────────────────────────────────
        if needs_reheat && !matches!(self.reheat_type, ReheatType::None) {
            // control_signal is the zone's net heating demand as a fraction of
            // reheat_capacity; the coil also has to lift the primary air to
            // zone temperature (see coil_duty_w).
            let q_heating_demand = self.coil_duty_w(inlet.state.t_db, cp_air, actual_flow);

            match self.reheat_type {
                ReheatType::Electric => {
                    let q_actual = q_heating_demand.min(self.reheat_capacity).max(0.0);
                    let dt = q_actual / (actual_flow * cp_air).max(0.001);
                    outlet_t = (inlet.state.t_db + dt).min(self.max_reheat_temp);
                    let q_delivered = actual_flow * cp_air * (outlet_t - inlet.state.t_db);
                    self.reheat_rate = q_delivered.max(0.0);
                    self.reheat_power = self.reheat_rate; // COP = 1.0
                }
                ReheatType::HotWater => {
                    // Water-side available capacity
                    let water_cap = if let Some(ref wi) = self.water_inlet {
                        if wi.state.mass_flow > 0.0 {
                            wi.state.mass_flow
                                * wi.state.cp
                                * (wi.state.temp - self.design_hw_outlet_temp).max(0.0)
                        } else {
                            0.0
                        }
                    } else {
                        // No water loop connected — use nominal capacity
                        self.reheat_capacity
                    };

                    let q_actual = q_heating_demand
                        .min(self.reheat_capacity)
                        .min(water_cap)
                        .max(0.0);
                    let dt = q_actual / (actual_flow * cp_air).max(0.001);
                    outlet_t = (inlet.state.t_db + dt).min(self.max_reheat_temp);
                    let q_delivered = actual_flow * cp_air * (outlet_t - inlet.state.t_db);
                    self.reheat_rate = q_delivered.max(0.0);
                    self.reheat_power = 0.0; // HW coils don't consume electricity

                    // Water outlet temperature
                    if let Some(ref wi) = self.water_inlet {
                        if wi.state.mass_flow > 0.0 {
                            let water_out_t = wi.state.temp
                                - self.reheat_rate / (wi.state.mass_flow * wi.state.cp);
                            self.water_outlet_state = Some(WaterPort::new(FluidState::water(
                                water_out_t,
                                wi.state.mass_flow,
                            )));
                        }
                    }
                }
                ReheatType::None => {} // already handled above
            }
        }

        AirPort::new(
            psych::MoistAirState::new(outlet_t, inlet.state.w, inlet.state.p_b),
            actual_flow,
        )
    }

    fn has_water_side(&self) -> bool {
        matches!(self.reheat_type, ReheatType::HotWater)
    }

    fn set_water_inlet(&mut self, inlet: &WaterPort) {
        self.water_inlet = Some(*inlet);
    }

    fn water_outlet(&self) -> Option<WaterPort> {
        self.water_outlet_state
    }

    fn design_air_flow_rate(&self) -> Option<f64> {
        if openbse_core::types::is_autosize(self.max_air_flow) {
            None
        } else {
            Some(self.max_air_flow)
        }
    }

    fn set_design_air_flow_rate(&mut self, flow: f64) {
        self.max_air_flow = flow;
    }

    fn set_setpoint(&mut self, signal: f64) {
        // For VAV boxes, "setpoint" is overloaded as the zone load signal
        self.control_signal = signal;
    }

    fn set_zone_temp(&mut self, temp: f64) {
        self.zone_temp = Some(temp);
    }

    fn set_zone_load(&mut self, q_net_w: f64) {
        self.zone_load = Some(q_net_w);
    }

    /// Autosize from the zone's sizing results.
    ///
    /// - `max_air_flow`: the zone design airflow.
    /// - `max_reheat_fraction`: raised so the heating maximum flow is at least
    ///   the zone's heating design airflow (E+ autosizes "Maximum Flow Fraction
    ///   During Reheat" the same way).  A perimeter zone whose heating airflow
    ///   exceeds half its cooling airflow would otherwise never reach setpoint.
    /// - `reheat_capacity`: enough to take the heating-max flow from the AHU
    ///   cold deck to `max_reheat_temp` (the E+ reheat coil sizing basis), and
    ///   at least the zone peak heating load plus the cold-air lift.
    fn set_terminal_sizing(&mut self, sizing: &TerminalSizing) {
        if openbse_core::types::is_autosize(self.max_air_flow) {
            self.max_air_flow = sizing.design_flow.max(0.01);
        }
        if self.max_air_flow > 0.0 && sizing.heating_flow > 0.0 {
            let needed = (sizing.heating_flow / self.max_air_flow).clamp(0.0, 1.0);
            self.max_reheat_fraction = self.max_reheat_fraction.max(needed);
        }
        if openbse_core::types::is_autosize(self.reheat_capacity) {
            if matches!(self.reheat_type, ReheatType::None) {
                self.reheat_capacity = 0.0;
                return;
            }
            let cp = 1005.0;
            let m_heat = self.max_air_flow * self.max_reheat_fraction;
            let to_max_temp =
                m_heat * cp * (self.max_reheat_temp - sizing.primary_supply_temp).max(1.0);
            let for_peak = sizing.peak_heating
                + m_heat * cp * (sizing.heating_setpoint - sizing.primary_supply_temp).max(0.0);
            self.reheat_capacity = to_max_temp.max(for_peak);
        }
    }

    fn setpoint(&self) -> Option<f64> {
        Some(self.control_signal)
    }

    fn nominal_capacity(&self) -> Option<f64> {
        Some(self.reheat_capacity)
    }

    fn set_nominal_capacity(&mut self, cap: f64) {
        self.reheat_capacity = cap;
    }

    fn power_consumption(&self) -> f64 {
        self.reheat_power
    }

    fn thermal_output(&self) -> f64 {
        self.reheat_rate
    }

    fn detailed_outputs(&self) -> std::collections::HashMap<String, f64> {
        let mut m = std::collections::HashMap::new();
        m.insert("sensible_load".to_string(), self.reheat_rate);
        m
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use openbse_core::types::{DayType, TimeStep};
    use openbse_psychrometrics::MoistAirState;

    fn make_ctx() -> SimulationContext {
        SimulationContext {
            timestep: TimeStep {
                month: 7,
                day: 15,
                hour: 14,
                sub_hour: 1,
                timesteps_per_hour: 1,
                sim_time_s: 0.0,
                dt: 3600.0,
            },
            outdoor_air: MoistAirState::from_tdb_rh(35.0, 0.4, 101325.0),
            day_type: DayType::WeatherDay,
            is_sizing: false,
            sizing_internal_gains: SizingInternalGains::Full,
        }
    }

    fn make_supply_air(temp: f64, flow: f64) -> AirPort {
        AirPort::new(MoistAirState::from_tdb_rh(temp, 0.5, 101325.0), flow)
    }

    #[test]
    fn test_vav_cooling_mode_increases_flow() {
        let mut vav = VAVBox::new("Zone1 VAV", "Zone1", 1.0, 0.3, ReheatType::Electric, 5000.0);

        // Supply air at 13°C, max flow 1.0 kg/s
        let inlet = make_supply_air(13.0, 2.0); // AHU provides plenty
        let ctx = make_ctx();

        // Light cooling demand (20% signal)
        vav.control_signal = -0.2;
        let out1 = vav.simulate_air(&inlet, &ctx);
        let flow1 = out1.mass_flow;

        // Heavy cooling demand (100% signal)
        vav.control_signal = -1.0;
        let out2 = vav.simulate_air(&inlet, &ctx);
        let flow2 = out2.mass_flow;

        // Heavier cooling → more air flow
        assert!(flow2 > flow1);
        // Both above minimum
        assert!(flow1 >= 0.3 * 1.0 - 0.001);
        // Heavy demand should be at or near max
        assert!(flow2 >= 0.8 * 1.0);
    }

    #[test]
    fn test_vav_deadband_minimum_flow() {
        let mut vav = VAVBox::new("Zone1 VAV", "Zone1", 1.0, 0.3, ReheatType::None, 0.0);

        let inlet = make_supply_air(13.0, 2.0);
        let ctx = make_ctx();

        vav.control_signal = 0.0; // deadband
        let outlet = vav.simulate_air(&inlet, &ctx);

        // Should be at minimum flow
        assert_relative_eq!(outlet.mass_flow, 0.3, max_relative = 0.01);
        assert_eq!(vav.reheat_rate, 0.0);
    }

    #[test]
    fn test_vav_heating_mode_with_electric_reheat() {
        let mut vav = VAVBox::new("Zone1 VAV", "Zone1", 1.0, 0.3, ReheatType::Electric, 5000.0);

        let inlet = make_supply_air(13.0, 2.0);
        let ctx = make_ctx();

        // Heating demand (60% of capacity)
        vav.control_signal = 0.6;
        let outlet = vav.simulate_air(&inlet, &ctx);

        // Dual-maximum control (ASHRAE G36 / E+ ReverseWithLimits):
        // Damper opens from min (0.3) toward max_reheat_fraction (0.5).
        // flow = 0.3 + 0.6 * (0.5 - 0.3) = 0.42
        assert_relative_eq!(outlet.mass_flow, 0.42, max_relative = 0.01);
        assert!(outlet.state.t_db > 13.0); // Reheated
        assert!(vav.reheat_rate > 0.0);
        assert!(vav.reheat_power > 0.0); // Electric reheat consumes power
    }

    #[test]
    fn test_vav_heating_mode_with_hw_reheat() {
        let mut vav = VAVBox::new("Zone1 VAV", "Zone1", 1.0, 0.3, ReheatType::HotWater, 5000.0)
            .with_hw_reheat_params(0.001, 82.0, 71.0);

        // Connect hot water
        let hw = WaterPort::new(FluidState::water(82.0, 0.1));
        vav.set_water_inlet(&hw);

        let inlet = make_supply_air(13.0, 2.0);
        let ctx = make_ctx();

        vav.control_signal = 0.6; // 60% of reheat capacity
        let outlet = vav.simulate_air(&inlet, &ctx);

        assert!(outlet.state.t_db > 13.0);
        assert!(vav.reheat_rate > 0.0);
        assert_eq!(vav.reheat_power, 0.0); // HW reheat = no electricity

        // Water outlet should be cooler than inlet
        let wo = vav.water_outlet().unwrap();
        assert!(wo.state.temp < 82.0);
    }

    #[test]
    fn test_vav_reheat_covers_cold_primary_air_lift() {
        // Zone at 21 °C fed 13 °C primary air. Net demand 1 kW. Without the
        // lift term the coil would deliver 1 kW but the zone would see
        // 1 kW − m·cp·8 K, i.e. net cooling.
        let mut vav = VAVBox::new(
            "Zone1 VAV",
            "Zone1",
            1.0,
            0.3,
            ReheatType::Electric,
            20000.0,
        );
        vav.set_zone_temp(21.0);
        vav.control_signal = 1000.0 / 20000.0;
        let inlet = make_supply_air(13.0, 2.0);
        let outlet = vav.simulate_air(&inlet, &make_ctx());
        let cp = psych::cp_air_fn_w(inlet.state.w);
        let net_to_zone = outlet.mass_flow * cp * (outlet.state.t_db - 21.0);
        assert_relative_eq!(net_to_zone, 1000.0, max_relative = 0.02);
        assert!(vav.reheat_rate > 1000.0);
    }

    #[test]
    fn test_vav_tempers_minimum_flow_when_cooling_load_is_small() {
        // Zone at 21 °C with a 500 W cooling load. Minimum flow of 13 °C air
        // removes 0.3·cp·8 ≈ 2.4 kW, so the coil must temper the air (E+
        // Reverse action) and the zone must receive exactly −500 W.
        let mut vav = VAVBox::new(
            "Zone1 VAV",
            "Zone1",
            1.0,
            0.3,
            ReheatType::HotWater,
            20000.0,
        );
        vav.set_zone_temp(21.0);
        vav.set_zone_load(-500.0);
        let inlet = make_supply_air(13.0, 2.0);
        let outlet = vav.simulate_air(&inlet, &make_ctx());
        let cp = psych::cp_air_fn_w(inlet.state.w);
        assert_relative_eq!(outlet.mass_flow, 0.3, max_relative = 0.01);
        let net_to_zone = outlet.mass_flow * cp * (outlet.state.t_db - 21.0);
        assert_relative_eq!(net_to_zone, -500.0, max_relative = 0.02);
        assert!(vav.reheat_rate > 0.0);
    }

    #[test]
    fn test_vav_opens_damper_when_cooling_load_exceeds_minimum_flow() {
        let mut vav = VAVBox::new(
            "Zone1 VAV",
            "Zone1",
            1.0,
            0.3,
            ReheatType::HotWater,
            20000.0,
        );
        vav.set_zone_temp(24.0);
        vav.set_zone_load(-6000.0); // needs ~0.54 kg/s of 13 °C air
        let inlet = make_supply_air(13.0, 2.0);
        let outlet = vav.simulate_air(&inlet, &make_ctx());
        let cp = psych::cp_air_fn_w(inlet.state.w);
        assert_eq!(vav.reheat_rate, 0.0);
        let net_to_zone = outlet.mass_flow * cp * (outlet.state.t_db - 24.0);
        assert_relative_eq!(net_to_zone, -6000.0, max_relative = 0.02);
    }

    #[test]
    fn test_vav_terminal_sizing_raises_heating_max_flow_and_capacity() {
        let mut vav = VAVBox::new(
            "Zone1 VAV",
            "Zone1",
            openbse_core::types::AUTOSIZE,
            0.3,
            ReheatType::HotWater,
            openbse_core::types::AUTOSIZE,
        );
        vav.set_terminal_sizing(&TerminalSizing {
            design_flow: 1.0,
            heating_flow: 0.8, // heating governs more than the 50% dual max
            peak_heating: 8000.0,
            primary_supply_temp: 13.0,
            heating_setpoint: 21.0,
        });
        assert_relative_eq!(vav.max_air_flow, 1.0);
        assert_relative_eq!(vav.max_reheat_fraction, 0.8);
        // 0.8 kg/s x 1005 x (40 - 13) = 21.7 kW >= 8 kW + 0.8*1005*8 = 14.4 kW
        assert!(vav.reheat_capacity > 21_000.0 && vav.reheat_capacity < 22_000.0);
    }

    #[test]
    fn test_vav_max_reheat_temp_limit() {
        let mut vav = VAVBox::new(
            "Zone1 VAV",
            "Zone1",
            0.1,
            0.3,
            ReheatType::Electric,
            50000.0,
        )
        .with_max_reheat_temp(35.0);

        let inlet = make_supply_air(13.0, 2.0);
        let ctx = make_ctx();

        // Full heating demand (100% of capacity) with small airflow
        vav.control_signal = 1.0;
        let outlet = vav.simulate_air(&inlet, &ctx);

        // Should not exceed max reheat temp
        assert!(outlet.state.t_db <= 35.1);
    }

    #[test]
    fn test_vav_no_reheat_type_none() {
        let mut vav = VAVBox::new("Zone1 VAV", "Zone1", 1.0, 0.3, ReheatType::None, 0.0);

        let inlet = make_supply_air(13.0, 2.0);
        let ctx = make_ctx();

        vav.control_signal = 0.5; // heating demand (fractional)
        let outlet = vav.simulate_air(&inlet, &ctx);

        // No reheat — air passes through at supply temp
        assert_relative_eq!(outlet.state.t_db, 13.0, max_relative = 0.001);
        assert_eq!(vav.reheat_rate, 0.0);
    }

    #[test]
    fn test_vav_zero_inlet_flow() {
        let mut vav = VAVBox::new("Zone1 VAV", "Zone1", 1.0, 0.3, ReheatType::Electric, 5000.0);

        let inlet = make_supply_air(13.0, 0.0);
        let ctx = make_ctx();

        vav.control_signal = -0.5;
        let outlet = vav.simulate_air(&inlet, &ctx);

        assert_eq!(outlet.mass_flow, 0.0);
        assert_eq!(vav.primary_air_flow, 0.0);
    }
}
