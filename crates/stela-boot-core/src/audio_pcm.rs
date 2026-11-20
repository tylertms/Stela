//! Conversion from the virtual USB speaker's signed, MSB-aligned stereo PCM.

#[derive(Clone, Copy)]
pub enum Pcm {
    I16,
    I24,
}

impl Pcm {
    pub const fn sample_bytes(self) -> usize {
        match self {
            Self::I16 => 2,
            Self::I24 => 3,
        }
    }

    /// Pack the available frames and zero the tail. When a full batch is off
    /// by one frame, the caller can instead stretch it across the output.
    pub fn render(self, input: &[[i32; 2]], output: &mut [u8], stretch: bool) {
        let bytes = self.sample_bytes();
        let frames = output.len() / (2 * bytes);
        let stretch = stretch && input.len() >= 2 && frames >= 2;
        for (index, out) in output.chunks_exact_mut(2 * bytes).enumerate() {
            let samples = if stretch {
                let position = index * (input.len() - 1);
                let whole = position / (frames - 1);
                let fraction = (position % (frames - 1)) as i64;
                let first = input[whole];
                let second = input[(whole + 1).min(input.len() - 1)];
                core::array::from_fn(|channel| {
                    let a = i64::from(first[channel]);
                    let b = i64::from(second[channel]);
                    (a + (b - a) * fraction / (frames - 1) as i64) as i32
                })
            } else {
                input.get(index).copied().unwrap_or([0; 2])
            };
            for (value, slot) in samples.into_iter().zip(out.chunks_exact_mut(bytes)) {
                slot.copy_from_slice(&value.to_le_bytes()[4 - bytes..]);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Pcm;

    #[test]
    fn packs_signed_stereo_at_both_widths_and_zeros_the_tail() {
        let input = [[0x1234_5600, -0x1234_5600]];
        let mut narrow = [0xff; 8];
        let mut wide = [0xff; 12];
        Pcm::I16.render(&input, &mut narrow, false);
        Pcm::I24.render(&input, &mut wide, false);
        assert_eq!(narrow, [0x34, 0x12, 0xcb, 0xed, 0, 0, 0, 0]);
        assert_eq!(wide, [0x56, 0x34, 0x12, 0xaa, 0xcb, 0xed, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn clock_correction_keeps_the_start_and_end_of_a_stereo_batch() {
        let input = [[0, 0], [4 << 16, -(4 << 16)]];
        let mut bytes = [0; 12];
        Pcm::I16.render(&input, &mut bytes, true);
        assert_eq!(bytes, [0, 0, 0, 0, 2, 0, 0xfe, 0xff, 4, 0, 0xfc, 0xff]);
    }
}
