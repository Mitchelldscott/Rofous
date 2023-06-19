extern crate hidapi;

use crate::teensy_comms::{data_structures::*, hid_reader::*, hid_ros::*, hid_writer::*};
use crate::utilities::loaders::*;
use hidapi::HidApi;
use std::{
    sync::{
        mpsc,
        mpsc::{Receiver, Sender},
        Arc, RwLock,
    },
    thread::spawn,
};

/// An HID Comms layer for teensys
/// Usage:
/// '''
///     HidLayer::spin();
/// '''
pub struct HidLayer;

impl HidLayer {
    /// careful hard to shutdown...

    pub fn rospipeline() {
        let byu = BuffYamlUtil::from_self();

        let vid = byu.load_u16("teensy_vid");
        let pid = byu.load_u16("teensy_pid");

        let shutdown = Arc::new(RwLock::new(false));
        let shdn1 = shutdown.clone();
        let shdn2 = shutdown.clone();

        let mut hidapi = HidApi::new().expect("Failed to create API instance");
        let mut hidreader = HidReader::new(&mut hidapi, vid, pid);
        let mut hidwriter = HidWriter::new(&mut hidapi, vid, pid);
        let mut hidros = HidROS::new();

        // create the rust channels
        let (reply_tx, reply_rx): (Sender<RobotStatus>, Receiver<RobotStatus>) = mpsc::channel();
        let (send_tx, send_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = mpsc::channel();

        let hidwriter_handle = spawn(move || {
            hidwriter.pipeline(shutdown, send_rx);
        });

        let hidros_handle = spawn(move || {
            hidros.pipeline(shdn2, send_tx, reply_rx);
        });

        let hidreader_handle = spawn(move || {
            hidreader.pipeline(shdn1, reply_tx);
        });

        hidreader_handle.join().expect("HID Reader failed");
        hidros_handle.join().expect("HID ROS failed");
        hidwriter_handle.join().expect("HID Writer failed");
    }

    pub fn pipeline() {
        let byu = BuffYamlUtil::from_self();

        let vid = byu.load_u16("teensy_vid");
        let pid = byu.load_u16("teensy_pid");

        let shutdown = Arc::new(RwLock::new(false));
        let shdn1 = shutdown.clone();
        let shdn2 = shutdown.clone();

        let mut hidapi = HidApi::new().expect("Failed to create API instance");
        let mut hidreader = HidReader::new(&mut hidapi, vid, pid);
        let mut hidwriter = HidWriter::new(&mut hidapi, vid, pid);

        // create the rust channels
        let (reply_tx, reply_rx): (Sender<RobotStatus>, Receiver<RobotStatus>) = mpsc::channel();
        let (send_tx, send_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = mpsc::channel();

        let hidwriter_handle = spawn(move || {
            hidwriter.pipeline(shutdown, send_rx);
        });

        let hidreader_handle = spawn(move || {
            hidreader.pipeline(shdn1, reply_tx);
        });

        hidreader_handle.join().expect("HID Reader failed");
        hidwriter_handle.join().expect("HID Writer failed");
    }
}
