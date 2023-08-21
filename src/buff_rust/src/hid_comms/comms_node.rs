use buff_rust::hid_comms::hid_interface::*;
use std::thread::Builder;

fn main() {
    /*
        Start an hid layer
    */
    let (interface, mut reader, mut writer) = HidInterface::new("Penguin");

    interface.layer.print();

    let reader_handle = Builder::new()
        .name("HID Reader".to_string())
        .spawn(move || {
            reader.pipeline();
        })
        .unwrap();

    let writer_handle = Builder::new()
        .name("HID Writer".to_string())
        .spawn(move || {
            writer.pipeline();
        })
        .unwrap();

    // let interface_sim = Builder::new()
    //     .name("HID Control".to_string())
    //     .spawn(move || {
    //         sim_interface(interface);
    //     })
    //     .unwrap();

    reader_handle.join().expect("HID Reader failed");
    // interface_sim.join().expect("HID Control failed");
    writer_handle.join().expect("HID Writer failed");
}
