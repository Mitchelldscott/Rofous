use buff_rust::hid_comms::hid_interface::*;
use std::thread::Builder;

fn main() {
    /*
        Start an hid layer
    */
    let (mut interface, mut reader, mut writer) = HidInterface::new("penguin");

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

    let interface_handle = Builder::new()
        .name("HID Control".to_string())
        .spawn(move || {
            interface.pipeline();
        })
        .unwrap();

    reader_handle.join().expect("HID Reader failed");
    interface_handle.join().expect("HID Control failed");
    writer_handle.join().expect("HID Writer failed");
}
