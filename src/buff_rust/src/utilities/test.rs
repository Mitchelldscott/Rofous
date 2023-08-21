#![allow(unused_imports)]
use crate::utilities::{data_structures::*, loaders::*};
use rand::Rng;
use std::env;

#[cfg(test)]
pub mod byu_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    pub fn load_yaml() {
        /*
            Use the penguin yaml file to test some loading functions
        */

        let byu = BuffYamlUtil::new("penguin");

        assert_eq!(byu.load_string("robot_type"), "demo");

        byu.load_tasks();
    }
}

#[cfg(test)]
pub mod buffer_tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    ///
    /// Test the byte buffer
    ///
    #[test]
    pub fn basic_byte_buffer() {
        let mut rng = rand::thread_rng();
        let n1: u8 = rng.gen();
        let i: usize = rng.gen_range(0..63);

        let mut buffer = ByteBuffer::new(64);
        // buffer.print_data();
        buffer.put(i, n1);
        assert_eq!(
            buffer.get(i),
            n1,
            "[{}] Failed get check {} != {}",
            i,
            n1,
            buffer.get(i)
        );
    }

    #[test]
    pub fn intermediate_byte_buffer() {
        let mut rng = rand::thread_rng();
        let n1: Vec<u8> = vec![rng.gen(); 10];
        let i: usize = rng.gen_range(0..53);

        let mut buffer = ByteBuffer::new(64);
        // buffer.print_data();
        buffer.puts(i, n1.clone());

        assert_eq!(
            buffer.get(i),
            n1[0],
            "[{}] Failed get check {} != {}",
            i,
            n1[0],
            buffer.get(i)
        );
        assert_eq!(
            buffer.get(i + 1),
            n1[1],
            "[{}] Failed get check {} != {}",
            i + 1,
            n1[1],
            buffer.get(i + 1)
        );
        assert_eq!(
            buffer.get(i + 2),
            n1[2],
            "[{}] Failed get check {} != {}",
            i + 2,
            n1[2],
            buffer.get(i + 2)
        );
        assert_eq!(
            buffer.get(i + 3),
            n1[3],
            "[{}] Failed get check {} != {}",
            i + 3,
            n1[3],
            buffer.get(i + 3)
        );
    }

    #[test]
    pub fn get_float_byte_buffer() {
        let n1: Vec<u8> = vec![0x40, 0x49, 0xf, 0xdb];

        let mut buffer = ByteBuffer::new(64);
        buffer.puts(2, n1.clone());
        buffer.print();

        assert_eq!(
            buffer.get_float(2),
            3.1415927410125732,
            "Failed Float check {} != {}",
            3.1415927410125732,
            buffer.get_float(2)
        );
    }
}
