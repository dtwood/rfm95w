/// Delay for approximately a microsecond. Very roughly calibrated by eye to
/// within about 20% precision.
/// @param delay Number of microseconds to delay.
pub fn delay_us(delay: u32) {
    for _ in 0..delay {
        for _ in 0..9 {
            unsafe {
                asm!("nop");
            }
        }
    }
}
