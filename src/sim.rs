use std::{
    fmt::Debug,
    ops::{Add, Mul, Sub},
};

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

type f = f64;

#[derive(Debug, Clone, Copy)]
pub enum ComponentValueEnum {
    Linear(LinearComponentValue),
}
impl ComponentValueEnum {
    fn create(self, connected_nets_i: &[usize]) -> ComponentStateEnum {
        match self {
            Self::Linear(v) => ComponentStateEnum::Linear(v.create(connected_nets_i)),
        }
    }
}
#[derive(Debug)]
pub enum ComponentStateEnum {
    Linear(LinearComponentState),
}
impl AsRef<dyn ComponentState> for ComponentStateEnum {
    fn as_ref<'a>(&'a self) -> &'a (dyn ComponentState + 'static) {
        match self {
            Self::Linear(v) => v,
        }
    }
}
impl AsMut<dyn ComponentState> for ComponentStateEnum {
    fn as_mut<'a>(&'a mut self) -> &'a mut (dyn ComponentState + 'static) {
        match self {
            Self::Linear(v) => v,
        }
    }
}

pub trait ComponentValue: Debug + Clone + Copy {
    type State: ComponentState;
    fn n_terminals(&self) -> usize;
    fn create(&self, connected_nets_i: &[usize]) -> Self::State;
}
pub trait ComponentState: Debug {
    fn set_nets(&mut self, connected_nets_i: &[usize]);

    fn impart_voltage_to_nets(&self, nets: &mut [NetState], step: f);
    fn impart_currents_to_nets(&self, nets: &mut [NetState]);

    fn purturb_from_nets(&mut self, nets: &mut [NetState]) -> HasConverged;
    fn tick(&mut self, dt: f);
}

#[derive(Debug, Clone, Copy)]
enum LinearComponentValue {
    Capacitive(f),
    Resistive(f),
    Inductive(f),
    Source(f),
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

#[derive(Debug)]
struct LinearComponentState {
    connected_nets_i: [usize; 2],
    value: LinearComponentValue,
    /// `= [Q, Q', Q''] = [Q, I, d/dt I]`, where `Q` is charge and `I` is current from terminal 0 to 1.
    q: [f; 3],
    offset_emf: f,
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

type HasConverged = bool;
fn converged(prev: f, next: f) -> HasConverged {
    const EPSILON: f = 1e-12;
    (prev - next).abs() <= EPSILON
}
fn converged_to_zero(v: f) -> HasConverged {
    const EPSILON: f = 0.0;
    v.abs() <= EPSILON
}

#[derive(Debug, Clone)]
pub struct NetState {
    components: Vec<(usize, usize)>,
    /// `= [I, d/dt I]`, where `I` is excess current being created or destroyed at the junction (should be zero).
    current: [f; 2],
    current_sources: u16,
    voltage: f,
    voltage_accumulator: f,
    voltage_accumulator_sources: u16,
}
impl NetState {
    fn new_empty() -> Self {
        Self {
            components: Vec::new(),
            current: [0.0; 2],
            current_sources: 0,
            voltage: 0.0,
            voltage_accumulator: 0.0,
            voltage_accumulator_sources: 0,
        }
    }
    fn apply_accumulated_voltage(&mut self) -> HasConverged {
        if self.voltage_accumulator_sources == 0 {
            return true;
        }
        let voltage_next = self.voltage_accumulator / self.voltage_accumulator_sources as f;
        let converged = converged(self.voltage, voltage_next);

        self.voltage = voltage_next;
        self.voltage_accumulator = 0.0;
        self.voltage_accumulator_sources = 0;

        converged
    }
    fn normalize_current(&mut self) {
        if self.current_sources == 0 {
            return;
        }
        self.current[0] /= self.current_sources as f;
        self.current[1] /= self.current_sources as f;
        self.current_sources = 0;
    }
    fn current_converged(&self) -> HasConverged {
        let converged = self.current.map(|v| converged_to_zero(v));
        true || converged[0] && converged[1]
    }
}

pub fn make_rc_test() {
    let mut circuit = CircuitState::new_empty();

    let nets_i = [
        circuit.create_net(),
        circuit.create_net(),
        circuit.create_net(),
    ];

    let c = circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Capacitive(0.1)),
        &[nets_i[0], nets_i[1]],
    );
    circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Resistive(2.0)),
        &[nets_i[1], nets_i[2]],
    );
    circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Inductive(0.1)),
        &[nets_i[2], nets_i[0]],
    );

    let ComponentStateEnum::Linear(c) = &mut circuit.components[c];
    // else {
    //     unreachable!()
    // };
    c.q[0] = -1.0; // push 1C of charge in the capacitor

    dbg!(circuit.solve_state());

    let n = 1_000_001;
    let dt = 0.000_01;
    // let mut j = Vec::with_capacity(n);
    let mut prev = Vec::new();
    for i in 0..n {
        let v = circuit.nets[0].voltage - circuit.nets[1].voltage;
        prev.push(v);
        if i % 1000 == 0 {
            dbg!(prev
                .drain(..)
                .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)));
        }
        // dbg!(v);
        // j.push(v);
        if !circuit.tick(dt) {
            dbg!(circuit);
            dbg!("convergence failed!");
            break;
        }
    }
    // dbg!(j);
}

#[derive(Debug)]
pub struct CircuitState {
    components: Vec<ComponentStateEnum>,
    nets: Vec<NetState>,
}
impl CircuitState {
    pub fn new_empty() -> Self {
        Self {
            components: Vec::new(),
            nets: Vec::new(),
        }
    }

    pub fn create_net(&mut self) -> usize {
        self.nets.push(NetState::new_empty());
        self.nets.len() - 1
    }
    pub fn create_component(
        &mut self,
        value: ComponentValueEnum,
        connected_nets_i: &[usize],
    ) -> usize {
        for net_i in connected_nets_i.iter() {
            assert!(*net_i < self.nets.len(), "net id invalid");
        }
        let component = value.create(connected_nets_i);
        let component_i = self.components.len();
        self.components.push(component);
        for (terminal_i, net_i) in connected_nets_i.iter().enumerate() {
            self.nets[*net_i].components.push((component_i, terminal_i));
        }
        component_i
    }

    pub fn tick(&mut self, dt: f) -> HasConverged {
        for component in self.components.iter_mut() {
            component.as_mut().tick(dt);
        }
        self.solve_state()
    }

    pub fn solve_state(&mut self) -> HasConverged {
        for i in 0..1000 {
            let mut converged = true;
            for _ in 0..10 {
                if !self.correct_voltages(((i * 1349) as f).sin() * 0.5 + 0.5) {
                    converged = false;
                } else {
                    break;
                }
            }
            if !self.correct_charge_states() {
                converged = false;
            }

            if converged {
                // dbg!(i);
                return true;
            }
        }
        false
    }

    fn correct_voltages(&mut self, step: f) -> HasConverged {
        for component in &self.components {
            component
                .as_ref()
                .impart_voltage_to_nets(&mut self.nets, step);
        }

        let mut converged = true;
        for net in &mut self.nets {
            if !net.apply_accumulated_voltage() {
                converged = false;
            }
        }

        // let v = self.nets.iter().map(|v| v.voltage).collect::<Vec<_>>();
        // // dbg!(format!("[{},{}]", v[0], v[1]));
        // dbg!(v);

        converged
    }
    fn correct_charge_states(&mut self) -> HasConverged {
        for net in &mut self.nets {
            net.current = [0.0; 2];
        }
        for component in &self.components {
            component.as_ref().impart_currents_to_nets(&mut self.nets);
        }
        for net in &mut self.nets {
            net.normalize_current();
        }

        let mut converged = true;
        for component in &mut self.components {
            if !component.as_mut().purturb_from_nets(&mut self.nets) {
                converged = false;
            }
        }

        // let i = self.nets.iter().map(|v| v.current).collect::<Vec<_>>();
        // dbg!(i);

        converged && self.nets.iter().all(|net| net.current_converged())
    }
}
