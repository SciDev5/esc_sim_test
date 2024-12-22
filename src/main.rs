use linalg::Mat;
use sim::{make_mosfet_test, make_rc_test};

mod linalg;
mod sim;

fn main() {
    // make_rc_test();
    make_mosfet_test();
    // let a = Mat::new([[0.0, 1.0], [1.0, 0.0]]);
    // let b = Mat::new([[2.0], [3.0]]);
    // dbg!(a * b);
    // println!("Hello, world!");
}
