use crate::sim::{converged, Lerp};

use super::{f, ComponentState, ComponentValue, HasConverged, NetState};

// ---------------------- LINEAR COMPONENTS ----------------------
// [capacitors, resistors, inductors, sources]

#[derive(Debug, Clone, Copy)]
pub enum LinearComponentValue {
    Capacitive(f),
    Resistive(f),
    Inductive(f),
    Source(f),
    Switch { closed: bool },
}
#[derive(Debug)]
pub struct LinearComponentState {
    connected_nets_i: [usize; 2],
    pub value: LinearComponentValue,
    /// `= [Q, Q', Q''] = [Q, I, d/dt I]`, where `Q` is charge and `I` is current from terminal 0 to 1.
    pub q: [f; 3],
    pub offset_emf: f,
}

impl ComponentValue for LinearComponentValue {
    type State = LinearComponentState;
    fn n_terminals(&self) -> usize {
        2
    }
    fn create(&self, connected_nets_i: &[usize]) -> Self::State {
        LinearComponentState::new(*self, connected_nets_i)
    }
}

impl LinearComponentState {
    fn new(value: LinearComponentValue, connected_nets_i: &[usize]) -> Self {
        let mut this = Self {
            connected_nets_i: [0, 0],
            value,
            q: [0.0; 3],
            offset_emf: 0.0,
        };
        this.set_nets(connected_nets_i);
        this
    }
}

impl ComponentState for LinearComponentState {
    fn set_nets(&mut self, connected_nets_i: &[usize]) {
        assert_eq!(
            connected_nets_i.len(),
            2,
            "can only create a linear component with exactly two connected nets."
        );
        for i in 0..2 {
            self.connected_nets_i[i] = connected_nets_i[i];
        }
    }

    fn impart_voltage_to_nets(&self, nets: &mut [NetState], step: f) {
        let v_prev =
            nets[self.connected_nets_i[1]].voltage - nets[self.connected_nets_i[0]].voltage;
        let v_target = self.offset_emf
            + match self.value {
                LinearComponentValue::Capacitive(c) => -self.q[0] / c,
                LinearComponentValue::Resistive(r) => -self.q[1] * r,
                LinearComponentValue::Inductive(l) => -self.q[2] * l,
                LinearComponentValue::Source(v) => v,
                LinearComponentValue::Switch { closed: true } => 0.0,
                LinearComponentValue::Switch { closed: false } => return,
            };
        let v_diff = (v_target - v_prev) * 0.5 * step;

        let net0 = &mut nets[self.connected_nets_i[0]];
        net0.voltage_accumulator += net0.voltage - v_diff;
        net0.voltage_accumulator_sources += 1;
        let net1 = &mut nets[self.connected_nets_i[1]];
        net1.voltage_accumulator += net1.voltage + v_diff;
        net1.voltage_accumulator_sources += 1;
    }
    fn impart_currents_to_nets(&self, nets: &mut [NetState]) {
        if let LinearComponentValue::Switch { closed: false } = self.value {
            return;
        }
        for i in 0..2 {
            let net0 = &mut nets[self.connected_nets_i[0]];
            net0.current[i] -= self.q[i + 1];
            net0.current_sources += 1;
            let net1 = &mut nets[self.connected_nets_i[1]];
            net1.current[i] += self.q[i + 1];
            net1.current_sources += 1;
        }
    }

    fn purturb_from_nets(&mut self, nets: &mut [NetState]) -> HasConverged {
        let v_target =
            nets[self.connected_nets_i[1]].voltage - nets[self.connected_nets_i[0]].voltage;
        let i_target = [0, 1].map(|i| {
            // self_current + avg( excess_current_flowing_in, -excess_current_flowing_out )
            // attempt to force the self current to accept excess inflowing and deliver exess outflowing current.
            self.q[i + 1]
                + 0.5
                    * (nets[self.connected_nets_i[0]].current[i]
                        - nets[self.connected_nets_i[1]].current[i])
        });

        // set `q` to attempt to satisfy the constraints of the different types of components.
        const FACTOR_R: f = 1.0;
        const FACTOR_L: f = 0.0;
        let mut q_next = self.q;
        match self.value {
            LinearComponentValue::Capacitive(_) | LinearComponentValue::Source(_) => {
                // V = q[0] / C   // V = <const>
                q_next[1] = i_target[0];
                q_next[2] = i_target[1];
                // q_next[2] = 0.0;
            }
            LinearComponentValue::Resistive(r) => {
                // V = q[1] R  ->  q[1] = V / R
                q_next[1] = FACTOR_R.lerp(-v_target / r, i_target[0]);
                q_next[2] = i_target[1];
                // q_next[2] = 0.0;
                // dbg!(v_target, i_target, self.q[1], q_next[1]);
            }
            LinearComponentValue::Inductive(l) => {
                // V = q[2] L  ->  q[2] = V / L
                q_next[2] = FACTOR_L.lerp(-v_target / l, i_target[1]);
                // dbg!(v_target, i_target, self.q[2], q_next[2]);
            }
            LinearComponentValue::Switch { closed } => {
                if closed {
                    q_next[1] = i_target[0];
                    q_next[2] = i_target[1];
                } else {
                    q_next[1] = 0.0;
                    q_next[2] = 0.0;
                }
            }
        }

        let converged = converged(self.q[1], q_next[1]) && converged(self.q[2], q_next[2]);
        self.q = q_next;
        converged
    }

    fn tick(&mut self, dt: f) {
        self.q[1] += self.q[2] * dt;
        self.q[0] += self.q[1] * dt;
    }
}

// ---------------------- MOSFETS ----------------------

#[derive(Debug, Clone, Copy)]
pub enum MOSFETDopingType {
    PChannel,
    NChannel,
}
#[derive(Debug, Clone, Copy)]
pub struct MOSFETComponentValue {
    pub ty: MOSFETDopingType,
    pub beta: f,
    pub threshold_voltage: f,
    pub body_diode_saturation_current: f,
    pub body_diode_ideality_facotor: f,
}

#[derive(Debug)]
pub struct MOSFETComponentState {
    /// `[source, gate, drain]`
    connected_nets_i: [usize; 3],
    pub value: MOSFETComponentValue,
    pub i: [f; 2],
    pub v_gs_positive: f,
    pub temperature: f,
}

impl ComponentValue for MOSFETComponentValue {
    type State = MOSFETComponentState;
    fn n_terminals(&self) -> usize {
        3
    }
    fn create(&self, connected_nets_i: &[usize]) -> Self::State {
        MOSFETComponentState::new(*self, connected_nets_i)
    }
}

impl MOSFETComponentState {
    fn new(value: MOSFETComponentValue, connected_nets_i: &[usize]) -> Self {
        let mut this = Self {
            connected_nets_i: [0; 3],
            value,
            i: [0.0; 2],
            v_gs_positive: 0.0,
            temperature: 295.0,
        };
        this.set_nets(connected_nets_i);
        this
    }
}

const ELEMENTARY_CHARGE_OVER_BOLTZMANN_CONSTANT: f = 1.1604518121550082e+4;

impl ComponentState for MOSFETComponentState {
    fn set_nets(&mut self, connected_nets_i: &[usize]) {
        assert_eq!(
            connected_nets_i.len(),
            3,
            "can only create a MOSFET with exactly three connected nets."
        );
        for i in 0..3 {
            self.connected_nets_i[i] = connected_nets_i[i];
        }
    }

    fn impart_voltage_to_nets(&self, nets: &mut [NetState], step: f) {
        let MOSFETComponentValue {
            beta,
            threshold_voltage: v_th,
            ty: doping_type,
            body_diode_ideality_facotor,
            body_diode_saturation_current,
        } = self.value;
        let i_ds = self.i[0];
        let i_ds = match doping_type {
            MOSFETDopingType::PChannel => i_ds,
            MOSFETDopingType::NChannel => -i_ds,
        };
        let v_gs = self.v_gs_positive;

        // dbg!("V", v_gs, i_ds, v_gs - v_th);
        // const SATURATION_RESISTANCE: f = 1e9; // one gigaohm lol
        let v_ds = if i_ds < 0.0 {
            // dbg!("V: // diode //");
            // body diode forward flow //
            -((-i_ds) / body_diode_saturation_current + 1.0).ln()
                * (body_diode_ideality_facotor * self.temperature
                    / ELEMENTARY_CHARGE_OVER_BOLTZMANN_CONSTANT)
        } else {
            let v_ctrl = v_gs - v_th;

            if v_ctrl > 0.0 {
                // dbg!(v_ctrl * v_ctrl, 2.0 * i_ds / beta);
                if v_ctrl * v_ctrl * 0.99999 > 2.0 * i_ds / beta {
                    // dbg!("V: //TRIODE//");
                    // linear/triode region //
                    v_ctrl - (v_ctrl * v_ctrl - 2.0 * i_ds / beta).sqrt()
                } else {
                    // dbg!("V: //SATURATION//");
                    // saturation region //
                    return; // no influence on voltage
                            // // have near infinite resistance for all current above saturation point
                            // v_ctrl + SATURATION_RESISTANCE * (i_ds - beta * 0.5 * v_ctrl * v_ctrl)
                }
            } else {
                // dbg!("V: //CLOSED//");
                // closed region //
                // no influence on voltage
                return;
            }
        };
        let v_ds = match doping_type {
            MOSFETDopingType::PChannel => -v_ds,
            MOSFETDopingType::NChannel => v_ds,
        };

        let v_ds_prev =
            nets[self.connected_nets_i[2]].voltage - nets[self.connected_nets_i[0]].voltage;

        let v_diff = (v_ds - v_ds_prev) * 0.5 * step;
        // dbg!(i_ds, v_ds, v_ds_prev);

        let net_source = &mut nets[self.connected_nets_i[0]];
        net_source.voltage_accumulator += net_source.voltage - v_diff;
        net_source.voltage_accumulator_sources += 1;
        let net_drain = &mut nets[self.connected_nets_i[2]];
        net_drain.voltage_accumulator += net_drain.voltage + v_diff;
        net_drain.voltage_accumulator_sources += 1;
    }

    fn impart_currents_to_nets(&self, nets: &mut [NetState]) {
        for i in 0..2 {
            let net_source = &mut nets[self.connected_nets_i[0]];
            net_source.current[i] -= self.i[i];
            net_source.current_sources += 1;
            let net_drain = &mut nets[self.connected_nets_i[2]];
            net_drain.current[i] += self.i[i];
            net_drain.current_sources += 1;
        }
    }

    fn purturb_from_nets(&mut self, nets: &mut [NetState]) -> HasConverged {
        let MOSFETComponentValue {
            beta,
            threshold_voltage: v_th,
            ty: doping_type,
            body_diode_ideality_facotor,
            body_diode_saturation_current,
        } = self.value;

        let v_gs = nets[self.connected_nets_i[1]].voltage - nets[self.connected_nets_i[0]].voltage;
        let v_ds = nets[self.connected_nets_i[2]].voltage - nets[self.connected_nets_i[0]].voltage;
        let (v_gs, v_ds) = match doping_type {
            MOSFETDopingType::PChannel => (-v_gs, -v_ds),
            MOSFETDopingType::NChannel => (v_gs, v_ds),
        };

        // dbg!("P", v_gs, v_ds);

        let i_ds = if v_ds > 0.0 {
            let v_ctrl = v_gs - v_th;
            beta * if v_ctrl > 0.0 {
                if v_ds < v_ctrl {
                    // dbg!("P: // linear/triode region //");
                    // linear/triode region //
                    v_ctrl * v_ds - v_ds * v_ds * 0.5
                } else {
                    // dbg!("P: // saturation region //");
                    // saturation region //
                    v_ctrl * v_ctrl * 0.5
                }
            } else {
                // dbg!("P: // closed region //");
                // closed region //
                let i_next = [0.0; 2];
                let converged = converged(self.i[0], i_next[0]) && converged(self.i[1], i_next[1]);
                self.i = i_next;
                return converged;
            }
        } else {
            // dbg!("P: // diode //");
            // body diode //
            -body_diode_saturation_current
                * ((-v_ds * ELEMENTARY_CHARGE_OVER_BOLTZMANN_CONSTANT
                    / (body_diode_ideality_facotor * self.temperature))
                    .min(64.0)
                    .exp()
                    - 1.0)
        };
        // dbg!(i_ds);
        let i_ds = match doping_type {
            MOSFETDopingType::PChannel => i_ds,
            MOSFETDopingType::NChannel => -i_ds,
        };
        let i_target = [0, 1].map(|i| {
            // self_current + avg( excess_current_flowing_in, -excess_current_flowing_out )
            // attempt to force the self current to accept excess inflowing and deliver exess outflowing current.
            self.i[i]
                + 0.5
                    * (nets[self.connected_nets_i[0]].current[i]
                        - nets[self.connected_nets_i[2]].current[i])
        });

        let i_next = [0.5.lerp(i_ds, i_target[0]), i_target[1]];
        let converged = converged(self.i[0], i_next[0])
            && converged(self.i[1], i_next[1])
            && converged(self.v_gs_positive, v_gs);
        self.i = i_next;
        self.v_gs_positive = v_gs;
        converged
    }

    fn tick(&mut self, dt: f) {
        self.i[1] += self.i[2] * dt;
    }
}
