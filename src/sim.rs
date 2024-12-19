// V = I R
// V = I' L
// V = int(t){I} / C

// struct LinearSecondOrderSystemSolver {

// }

use std::ops::{Add, Mul, Sub};

trait Lerp:
    Add<Self, Output = Self>
    + Sub<Self, Output = Self>
    + Mul<Self, Output = Self>
    + Clone
    + Copy
    + From<i16>
{
    fn lerp(self, a: Self, b: Self) -> Self {
        a * (Self::from(1) - self) + b * self
    }
}
impl Lerp for f32 {}
impl Lerp for f64 {}

#[derive(Debug, Clone, Copy)]
enum ComponentValue {
    Capacitive(f32), // constrains Q
    Resistive(f32),  // constrains Q'
    Inductive(f32),  // constrains Q''
}
impl ComponentValue {
    fn voltage(self, q: [f32; 3]) -> f32 {
        match self {
            // disallows instantaneous change in
            Self::Capacitive(c) => q[0] / c,
            Self::Resistive(r) => q[1] * r,
            Self::Inductive(l) => q[2] * l,
        }
    }
    fn purturb(self, v_target: f32, i_target: [f32; 2], step: f32, factor: f32, q: &mut [f32; 3]) {
        match self {
            Self::Capacitive(_) => {
                // V = q[0] / C
                q[1] = step.lerp(q[1], i_target[0]);
                q[2] = step.lerp(q[2], i_target[1]);
            }
            Self::Resistive(r) => {
                // V = q[1] R  ->  q[1] = V / R
                q[1] = step.lerp(q[1], factor.lerp(v_target / r, i_target[0]));
                q[2] = step.lerp(q[2], i_target[1]);
            }
            Self::Inductive(l) => {
                // V = q[2] L  ->  q[2] = V / L
                q[2] = step.lerp(q[2], factor.lerp(v_target / l, i_target[1]));
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct ComponentState {
    value: ComponentValue,
    connected_nets: [usize; 2],
    q: [f32; 3],
    v: f32,
}
impl ComponentState {
    fn tick(&mut self, dt: f32) {
        self.q[0] += self.q[1] * dt * 0.5;
        self.q[1] += self.q[2] * dt;
        self.q[0] += self.q[1] * dt * 0.5;
        // dbg!(self.q);
    }
    fn update_v(&mut self) {
        self.v = self.value.voltage(self.q);
    }
}

#[derive(Debug, Clone, Copy)]
struct NetComponentEntry {
    component_i: usize,
    negate: bool,
}

#[derive(Debug, Clone)]
pub struct NetState {
    components: Vec<NetComponentEntry>,
    i_through: [f32; 2],
    voltage_accepted: f32,
    voltage_accum: f32,
}

pub fn make_rc_test() {
    let mut state = CircuitState {
        components: vec![
            ComponentState {
                connected_nets: [0, 1],
                value: ComponentValue::Capacitive(1.0),
                q: [1.0, 0.0, 0.0],
                v: 1.0,
            },
            ComponentState {
                connected_nets: [0, 1],
                value: ComponentValue::Resistive(1.0),
                q: [0.0, 0.0, 0.0],
                v: 1.0,
            },
        ],
        nets: vec![
            NetState {
                components: vec![
                    NetComponentEntry {
                        component_i: 0,
                        negate: false,
                    },
                    NetComponentEntry {
                        component_i: 1,
                        negate: false,
                    },
                ],
                i_through: [0.0, 0.0],
                voltage_accepted: 0.0,
                voltage_accum: 0.0,
            },
            NetState {
                components: vec![
                    NetComponentEntry {
                        component_i: 0,
                        negate: true,
                    },
                    NetComponentEntry {
                        component_i: 1,
                        negate: true,
                    },
                ],
                i_through: [0.0, 0.0],
                voltage_accepted: 0.0,
                voltage_accum: 0.0,
            },
        ],
    };

    dbg!(state.solve_state_initial());

    let n = 10001;
    let dt = 0.0001;
    // let mut j = Vec::with_capacity(n);
    for i in 0..n {
        if i % 100 == 0 {
            dbg!(state.components[0].v);
        }
        // j.push(state.components[0].v);
        if !state.tick(dt) {
            dbg!("convergence failed!");
            break;
        }
    }
    // dbg!(j);
}

#[derive(Debug, Clone)]
pub struct CircuitState {
    components: Vec<ComponentState>,
    nets: Vec<NetState>,
}
impl CircuitState {
    pub fn tick(&mut self, dt: f32) -> bool {
        for component in self.components.iter_mut() {
            component.tick(dt);
        }
        self.solve_state()
    }
    fn perturb_voltages(&mut self, convergence_thresh: f32) -> bool {
        for net in &mut self.nets {
            net.voltage_accum = 0.0;
        }
        for ComponentState {
            connected_nets, v, ..
        } in &self.components
        {
            let accepted = connected_nets.map(|i| self.nets[i].voltage_accepted);
            let delta = (v - (accepted[1] - accepted[0])) * 0.5;
            let target = [accepted[0] - delta, accepted[1] + delta];
            for i in 0..connected_nets.len() {
                self.nets[i].voltage_accum += target[i];
            }
        }
        let range = self
            .nets
            .iter()
            .map(|net| net.voltage_accum / net.components.len() as f32)
            .map(|v| (v, v))
            .reduce(|a, b| (a.0.min(b.0), a.1.max(b.1)))
            .unwrap_or((0.0, 0.0));
        let range = (range.1 - range.0) + 1e-8;

        let mut changed = false;
        for net in &mut self.nets {
            let new_accepted = net.voltage_accum / net.components.len() as f32;
            if (new_accepted - net.voltage_accepted).abs() / range > convergence_thresh {
                changed = true;
            }
            net.voltage_accepted = new_accepted;
        }

        // dbg!(self
        //     .nets
        //     .iter()
        //     .map(|v| v.voltage_accepted)
        //     .collect::<Vec<_>>());

        changed
    }
    fn compute_i_through(&mut self) {
        for net in &mut self.nets {
            net.i_through = [0.0, 0.0];
            for component_ent in &net.components {
                let component = &self.components[component_ent.component_i];
                if component_ent.negate {
                    net.i_through[0] -= component.q[1];
                    net.i_through[1] -= component.q[2];
                } else {
                    net.i_through[0] += component.q[1];
                    net.i_through[1] += component.q[2];
                }
            }
        }
    }
    fn perturb_charge_states(&mut self, step: f32, convergence_thresh: f32) -> bool {
        self.compute_i_through();
        let mut changed = false;
        for component_state in &mut self.components {
            let q_prev = component_state.q;
            let mut q = q_prev;
            let v_target = component_state
                .connected_nets
                .map(|net_i| self.nets[net_i].voltage_accepted);
            let i_target = component_state
                .connected_nets
                .map(|net_i| self.nets[net_i].i_through);
            let i_target = [
                // (i_target[0][0] - i_target[1][0]) * 0.5,
                // (i_target[0][1] - i_target[1][1]) * 0.5,
                q[1] + (i_target[1][0] - i_target[0][0]) * 0.5,
                q[2] + (i_target[1][1] - i_target[0][1]) * 0.5,
            ];

            component_state
                .value
                .purturb(v_target[1] - v_target[0], i_target, step, 0.5, &mut q);
            // component_state
            //     .value
            //     .purturb(v_target[1] - v_target[0], i_target, step, 1.0, &mut q);

            // dbg!((q, q_prev));

            component_state.q = q;

            changed |= q
                .into_iter()
                .zip(q_prev.into_iter())
                .any(|(a, b)| (a - b).abs() / (a * b + 1e-16).abs().sqrt() > convergence_thresh);
        }
        self.update_component_voltages();

        // dbg!(self.nets.iter().map(|v| v.i_through).collect::<Vec<_>>());

        changed
    }
    fn update_component_voltages(&mut self) {
        for component in &mut self.components {
            component.update_v();
        }
    }

    fn solve_state_initial(&mut self) -> bool {
        const SOLVE_STEP_SIZE: f32 = 0.5;
        const SOLVE_CONVERGENCE_THRESH: f32 = 1e-10; // converge to [1 part in 1e10] variance
        let mut solved = false;

        self.update_component_voltages();
        for _ in 0..100 {
            let changed = self.perturb_voltages(SOLVE_CONVERGENCE_THRESH);
            if !changed {
                solved = true;
                break;
            }
        }
        if !solved {
            return false;
        }
        solved = false;
        for _ in 0..1000 {
            let changed = self.perturb_voltages(SOLVE_CONVERGENCE_THRESH)
                || self.perturb_charge_states(SOLVE_STEP_SIZE, SOLVE_CONVERGENCE_THRESH);
            if !changed {
                solved = true;
                break;
            }
        }
        if !solved {
            return false;
        }

        true
    }
    fn solve_state(&mut self) -> bool {
        const SOLVE_STEP_SIZE: f32 = 1.0;
        const SOLVE_CONVERGENCE_THRESH: f32 = 1e-10; // converge to [1 part in 1e10] variance

        self.update_component_voltages();
        for i in 0..100 {
            let mut changed = false;
            if self.perturb_voltages(SOLVE_CONVERGENCE_THRESH) {
                changed = true;
            }
            if self.perturb_charge_states(SOLVE_STEP_SIZE, SOLVE_CONVERGENCE_THRESH) {
                changed = true;
            }

            if !changed {
                // dbg!(i);
                return true;
            }
        }
        false
    }
}