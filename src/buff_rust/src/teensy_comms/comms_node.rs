use buff_rust::teensy_comms::{hid_layer::*, hid_reader::*, hid_ros::*, hid_writer::*};
use std::thread::Builder;

fn main() {
    /*
        Start an hid layer
    */
    let (layer, writer_rx) = HidLayer::new("penguin");
    let mut hidreader = HidReader::new(layer.clone());
    let mut hidwriter = HidWriter::new(layer.clone(), writer_rx);
    let mut hidros = HidROS::new(layer.clone());

    let hidreader_handle = Builder::new()
        .name("HID Reader".to_string())
        .spawn(move || {
            hidreader.pipeline();
        })
        .unwrap();

    let hidwriter_handle = Builder::new()
        .name("HID Writer".to_string())
        .spawn(move || {
            hidwriter.pipeline();
        })
        .unwrap();

    let ros_handle = Builder::new()
        .name("HID ROS".to_string())
        .spawn(move || {
            hidros.pipeline();
        })
        .unwrap();

    let control_handle = Builder::new()
        .name("HID Control".to_string())
        .spawn(move || {
            layer.control_pipeline();
        })
        .unwrap();

    hidreader_handle.join().expect("HID Reader failed");
    hidwriter_handle.join().expect("HID Writer failed");
    ros_handle.join().expect("HID ROS failed");
    control_handle.join().expect("HID Control failed");
}
