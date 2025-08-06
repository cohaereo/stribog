use std::borrow::Cow;

/// MSB first bit reader
pub struct BitReader<'a> {
    data: Cow<'a, [u8]>,
    bit_index: usize,
}

impl<'a> BitReader<'a> {
    pub fn new(data: Cow<'a, [u8]>) -> Self {
        BitReader { data, bit_index: 0 }
    }

    pub fn data(&self) -> &[u8] {
        &self.data
    }

    pub fn read_bit(&mut self) -> bool {
        // if self.bit_index >= self.data.len() * 8 {
        //     return None;
        // }

        let byte_index = self.bit_index / 8;
        let bit_offset = self.bit_index % 8;

        let byte = self.data[byte_index];
        let bit = (byte >> (7 - bit_offset)) & 1;

        self.bit_index += 1;
        bit == 1
    }

    pub fn read_bits(&mut self, count: usize) -> u32 {
        // if self.bit_index + count > self.data.len() * 8 {
        //     return None;
        // }

        let mut result = 0;
        for _ in 0..count {
            result = (result << 1) | self.read_bit() as u32;
        }

        result
    }
}
