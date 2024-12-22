use std::{
    fmt::Debug,
    ops::{Add, Mul, Sub},
};

use components::{
    LinearComponentState, LinearComponentValue, MOSFETComponentState, MOSFETComponentValue,
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

pub mod components;

pub type f = f64;

#[derive(Debug, Clone, Copy)]
pub enum ComponentValueEnum {
    Linear(LinearComponentValue),
    MOSFET(MOSFETComponentValue),
}
impl ComponentValueEnum {
    fn create(self, connected_nets_i: &[usize]) -> ComponentStateEnum {
        match self {
            Self::Linear(v) => ComponentStateEnum::Linear(v.create(connected_nets_i)),
            Self::MOSFET(v) => ComponentStateEnum::MOSFET(v.create(connected_nets_i)),
        }
    }
}
#[derive(Debug)]
pub enum ComponentStateEnum {
    Linear(LinearComponentState),
    MOSFET(MOSFETComponentState),
}
impl AsRef<dyn ComponentState> for ComponentStateEnum {
    fn as_ref<'a>(&'a self) -> &'a (dyn ComponentState + 'static) {
        match self {
            Self::Linear(v) => v,
            Self::MOSFET(v) => v,
        }
    }
}
impl AsMut<dyn ComponentState> for ComponentStateEnum {
    fn as_mut<'a>(&'a mut self) -> &'a mut (dyn ComponentState + 'static) {
        match self {
            Self::Linear(v) => v,
            Self::MOSFET(v) => v,
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
        circuit.create_net(),
        circuit.create_net(),
    ];

    let c = circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Capacitive(0.1)),
        &[nets_i[0], nets_i[1]],
    );
    circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Inductive(0.1)),
        &[nets_i[1], nets_i[2]],
    );
    circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Inductive(0.1)),
        &[nets_i[2], nets_i[0]],
    );

    let c1 = circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Capacitive(0.1)),
        &[nets_i[3], nets_i[4]],
    );
    circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Inductive(0.2)),
        &[nets_i[4], nets_i[3]],
    );

    let ComponentStateEnum::Linear(c) = &mut circuit.components[c] else {
        unreachable!()
    };
    c.q[0] = -1.0; // push 1C of charge in the capacitor
    let ComponentStateEnum::Linear(c1) = &mut circuit.components[c1] else {
        unreachable!()
    };
    c1.q[0] = -1.0; // push 1C of charge in the capacitor

    dbg!(circuit.solve_state());

    let n = 1_000_001;
    let dt = 0.000_01;
    // let mut j = Vec::with_capacity(n);
    let mut prev = Vec::new();
    let mut prev1 = Vec::new();
    for i in 0..n {
        let v = circuit.nets[0].voltage - circuit.nets[1].voltage;
        prev.push(v);
        let v1 = circuit.nets[3].voltage - circuit.nets[4].voltage;
        prev1.push(v1);
        if i % 1000 == 0 {
            let v = prev
                .drain(..)
                .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let v1 = prev1
                .drain(..)
                .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            dbg!(v, v1);
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

pub fn make_mosfet_test() {
    let mut circuit = CircuitState::new_empty();

    let nets_i = [
        circuit.create_net(),
        circuit.create_net(),
        circuit.create_net(),
    ];

    circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Source(5.0)),
        &[nets_i[0], nets_i[1]],
    );
    circuit.create_component(
        ComponentValueEnum::Linear(LinearComponentValue::Source(5.0)),
        &[nets_i[2], nets_i[1]],
    );
    dbg!(circuit.solve_state());

    let mosfet = circuit.create_component(
        ComponentValueEnum::MOSFET(MOSFETComponentValue {
            beta: 0.02,
            ty: components::MOSFETDopingType::PChannel,
            body_diode_ideality_facotor: 1.0,
            body_diode_saturation_current: 0.1,
            threshold_voltage: 1.0,
        }),
        &[nets_i[0], nets_i[2], nets_i[1]],
    );

    dbg!(circuit.solve_state());

    let ComponentStateEnum::MOSFET(mosfet) = &mut circuit.components[mosfet] else {
        unreachable!();
    };

    dbg!(mosfet.i);
    dbg!(circuit.nets[1].voltage - circuit.nets[0].voltage);

    // let n = 1_000_001;
    // let dt = 0.000_01;
    // let mut prev = Vec::new();
    // for i in 0..n {
    // let v = circuit.nets[1].voltage - circuit.nets[0].voltage;
    //     prev.push(v);
    //     if i % 1000 == 0 {
    //         let v = prev
    //             .drain(..)
    //             .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    // dbg!(v);
    //     }
    //     if !circuit.tick(dt) {
    //         dbg!(circuit);
    //         dbg!("convergence failed!");
    //         break;
    //     }
    // }
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
        for i in 0..10000 {
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
                dbg!(i);
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
