use buff_rust::comms::hid_interface::*;
use std::time::Instant;
fn main() {
    /*
        Start an hid layer
    */
    let mut interface = HidInterface::sim("penguin");

    interface.robot_fw.load_run(0, u16::MAX);
    interface.robot_fw.display();

    loop {
        let t = Instant::now();
        while t.elapsed().as_millis() < 50 {}
    }
}
