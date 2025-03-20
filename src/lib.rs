#![no_std]

use fugit::HertzU32;
use rp2040_hal::{
    gpio::{FunctionNull, Pin, PinId, PullDown, ValidFunction},
    pio::{
        InstallError, PIOExt, Rx, StateMachine, StateMachineIndex, Stopped, Tx, UninitStateMachine,
        ValidStateMachine, PIO,
    },
};

// Based on a 48kHz sample rate
// TODO(bschwind) - Support other sample rates. It's basically
// sample_rate * 2 (stereo) * 2 (2 clock cycles per bit output) * 32 (32 bits per sample)
const PIO_CLOCK_HZ: u32 = 6_144_000;

#[derive(Debug)]
pub enum I2SError {
    PioInstallationError(InstallError),
}

pub enum PioClockDivider {
    Exact { integer: u16, fraction: u8 },
    FromSystemClock(HertzU32),
}

impl PioClockDivider {
    fn pio_divider(&self) -> (u16, u8) {
        match self {
            Self::Exact { integer, fraction } => (*integer, *fraction),
            Self::FromSystemClock(system_clock_hz) => {
                let hertz = system_clock_hz.to_Hz();
                let (fraction, integer) = libm::modf(hertz as f64 / PIO_CLOCK_HZ as f64);

                (integer as u16, (fraction * 256.0) as u8)
            },
        }
    }
}

pub trait SampleReader {
    fn read(&mut self) -> Option<u32>;
}

impl<SM: ValidStateMachine> SampleReader for Rx<SM> {
    fn read(&mut self) -> Option<u32> {
        self.read()
    }
}

pub struct I2SOutput<P: rp2040_hal::pio::PIOExt, SM: rp2040_hal::pio::StateMachineIndex> {
    state_machine: StateMachine<(P, SM), Stopped>,
    fifo_rx: Rx<(P, SM)>,
    fifo_tx: Tx<(P, SM)>,
}

impl<P: PIOExt, SM: StateMachineIndex> I2SOutput<P, SM> {
    /// Create an I2S output with a data line, a bit clock, and a left/right word clock.
    /// The left/right word clock pin MUST consecutively follow the bit clock pin. So if
    /// the bit clock pin is 7, the word clock pin MUST be 8.
    pub fn new<DataPin, BitClockPin, LeftRightClockPin>(
        pio: &mut PIO<P>,
        clock_divider: PioClockDivider,
        state_machine: UninitStateMachine<(P, SM)>,
        data_out_pin: Pin<DataPin, FunctionNull, PullDown>,
        bit_clock_pin: Pin<BitClockPin, FunctionNull, PullDown>,
        left_right_clock_pin: Pin<LeftRightClockPin, FunctionNull, PullDown>,
    ) -> Result<Self, I2SError>
    where
        DataPin: PinId + ValidFunction<P::PinFunction>,
        BitClockPin: PinId + ValidFunction<P::PinFunction>,
        LeftRightClockPin: PinId + ValidFunction<P::PinFunction>,
    {
        let data_out_pin: Pin<_, P::PinFunction, _> = data_out_pin.into_function();
        let bit_clock_pin: Pin<_, P::PinFunction, _> = bit_clock_pin.into_function();
        let left_right_clock_pin: Pin<_, P::PinFunction, _> = left_right_clock_pin.into_function();

        let data_pin_id = data_out_pin.id().num;
        let bit_clock_pin_id = bit_clock_pin.id().num;
        let left_right_clock_pin_id = left_right_clock_pin.id().num;

        assert_eq!(
            left_right_clock_pin_id - bit_clock_pin_id,
            1,
            "The word clock pin must consecutively follow the bit clock pin"
        );

        #[rustfmt::skip]
        let dac_pio_program = pio_proc::pio_asm!(
            ".side_set 2",
            "    set x, 30          side 0b01", // side 0bWB - W = Word Clock, B = Bit Clock
            "left_data:",
            "    out pins, 1        side 0b00",
            "    jmp x-- left_data  side 0b01",
            "    out pins 1         side 0b10",
            "    set x, 30          side 0b11",
            "right_data:",
            "    out pins 1         side 0b10",
            "    jmp x-- right_data side 0b11",
            "    out pins 1         side 0b00",
        );

        let installed =
            pio.install(&dac_pio_program.program).map_err(I2SError::PioInstallationError)?;

        let (divider_int, divider_fraction) = clock_divider.pio_divider();

        let (mut dac_sm, fifo_rx, fifo_tx) =
            rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
                .out_pins(data_pin_id, 1)
                .side_set_pin_base(bit_clock_pin_id)
                .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
                .clock_divisor_fixed_point(divider_int, divider_fraction)
                .buffers(rp2040_hal::pio::Buffers::OnlyTx)
                .autopull(true)
                .pull_threshold(32)
                .build(state_machine);

        dac_sm.set_pindirs([
            (data_pin_id, rp2040_hal::pio::PinDir::Output),
            (bit_clock_pin_id, rp2040_hal::pio::PinDir::Output),
            (left_right_clock_pin_id, rp2040_hal::pio::PinDir::Output),
        ]);

        Ok(Self { state_machine: dac_sm, fifo_rx, fifo_tx })
    }

    #[allow(clippy::type_complexity)]
    pub fn split(self) -> (StateMachine<(P, SM), Stopped>, Rx<(P, SM)>, Tx<(P, SM)>) {
        (self.state_machine, self.fifo_rx, self.fifo_tx)
    }
}

pub struct I2SInput<P: rp2040_hal::pio::PIOExt, SM: rp2040_hal::pio::StateMachineIndex> {
    state_machine: StateMachine<(P, SM), Stopped>,
    fifo_rx: Rx<(P, SM)>,
    fifo_tx: Tx<(P, SM)>,
}

impl<P: PIOExt, SM: StateMachineIndex> I2SInput<P, SM> {
    /// Create an I2S input with a data line, a bit clock, and a left/right word clock.
    /// The left/right word clock pin MUST consecutively follow the bit clock pin. So if
    /// the bit clock pin is 7, the word clock pin MUST be 8.
    pub fn new<DataPin, BitClockPin, LeftRightClockPin>(
        pio: &mut PIO<P>,
        clock_divider: PioClockDivider,
        state_machine: UninitStateMachine<(P, SM)>,
        data_in_pin: Pin<DataPin, FunctionNull, PullDown>,
        bit_clock_pin: Pin<BitClockPin, FunctionNull, PullDown>,
        left_right_clock_pin: Pin<LeftRightClockPin, FunctionNull, PullDown>,
    ) -> Result<Self, I2SError>
    where
        DataPin: PinId + ValidFunction<P::PinFunction>,
        BitClockPin: PinId + ValidFunction<P::PinFunction>,
        LeftRightClockPin: PinId + ValidFunction<P::PinFunction>,
    {
        let data_in_pin: Pin<_, P::PinFunction, _> = data_in_pin.into_function();
        let bit_clock_pin: Pin<_, P::PinFunction, _> = bit_clock_pin.into_function();
        let left_right_clock_pin: Pin<_, P::PinFunction, _> = left_right_clock_pin.into_function();

        let data_pin_id = data_in_pin.id().num;
        let bit_clock_pin_id = bit_clock_pin.id().num;
        let left_right_clock_pin_id = left_right_clock_pin.id().num;

        assert_eq!(
            left_right_clock_pin_id - bit_clock_pin_id,
            1,
            "The word clock pin must consecutively follow the bit clock pin"
        );

        #[rustfmt::skip]
        let mic_pio_program = pio_proc::pio_asm!(
            ".side_set 2",
            "    nop                side 0b10", // side 0bWB - W = Word Clock, B = Bit Clock
            "    nop                side 0b11",
            "    nop                side 0b00",
            "    nop                side 0b01",
            ".wrap_target",
            "    set x, 29          side 0b00",
            "left_data:",
            "    in pins, 1         side 0b01",
            "    jmp x-- left_data  side 0b00",
            "    in pins, 1         side 0b01", // We have read 30 bits after the loop, read 31st here
            "    nop                side 0b10",
            "    in pins, 1         side 0b11", // Read the 32nd bit here
            "    set x, 29          side 0b10",
            "right_data:",
            "    in pins, 1         side 0b11",
            "    jmp x-- right_data side 0b10",
            "    in pins, 1         side 0b11", // We have read 30 bits after the loop, read 31st here
            "    nop                side 0b00",
            "    in pins, 1         side 0b01", // Read the 32nd bit here
            ".wrap",
        );

        let installed = pio.install(&mic_pio_program.program).unwrap();

        let (divider_int, divider_fraction) = clock_divider.pio_divider();

        let (mut mic_sm, fifo_rx, fifo_tx) =
            rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
                .in_pin_base(data_pin_id)
                .side_set_pin_base(bit_clock_pin_id)
                .in_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
                .clock_divisor_fixed_point(divider_int, divider_fraction)
                .buffers(rp2040_hal::pio::Buffers::OnlyRx)
                .autopush(true)
                .push_threshold(32)
                .build(state_machine);

        mic_sm.set_pindirs([
            (data_pin_id, rp2040_hal::pio::PinDir::Input),
            (bit_clock_pin_id, rp2040_hal::pio::PinDir::Output),
            (left_right_clock_pin_id, rp2040_hal::pio::PinDir::Output),
        ]);

        Ok(Self { state_machine: mic_sm, fifo_rx, fifo_tx })
    }

    /// Create an I2S input which only reads from a data line. This assumes
    /// the I2S device is clocked by another state machine returned from
    /// `I2SInput::new`. The state machines must have their clock dividers
    /// sychronized, and start at the exact same time. The PIO program code
    /// must also stay in sync in terms of when reads occur, and the number
    /// of instructions.
    pub fn new_data_only<DataPin>(
        pio: &mut PIO<P>,
        clock_divider: PioClockDivider,
        state_machine: UninitStateMachine<(P, SM)>,
        data_in_pin: Pin<DataPin, FunctionNull, PullDown>,
    ) -> Result<Self, I2SError>
    where
        DataPin: PinId + ValidFunction<P::PinFunction>,
    {
        let data_in_pin: Pin<_, P::PinFunction, _> = data_in_pin.into_function();
        let data_pin_id = data_in_pin.id().num;

        #[rustfmt::skip]
        let mic_pio_program = pio_proc::pio_asm!(
            "    set x, 30",
            "left_data:",
            "    in pins, 1",
            "    jmp x-- left_data",
            "    in pins, 1",
            "    set x, 30",
            "right_data:",
            "    in pins, 1",
            "    jmp x-- right_data",
            "    in pins, 1",
        );

        let installed = pio.install(&mic_pio_program.program).unwrap();

        let (divider_int, divider_fraction) = clock_divider.pio_divider();

        let (mut mic_sm, fifo_rx, fifo_tx) =
            rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
                .in_pin_base(data_pin_id)
                .in_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
                .clock_divisor_fixed_point(divider_int, divider_fraction)
                .buffers(rp2040_hal::pio::Buffers::OnlyRx)
                .autopush(true)
                .push_threshold(32)
                .build(state_machine);

        mic_sm.set_pindirs([(data_pin_id, rp2040_hal::pio::PinDir::Input)]);

        Ok(Self { state_machine: mic_sm, fifo_rx, fifo_tx })
    }

    #[allow(clippy::type_complexity)]
    pub fn split(self) -> (StateMachine<(P, SM), Stopped>, Rx<(P, SM)>, Tx<(P, SM)>) {
        (self.state_machine, self.fifo_rx, self.fifo_tx)
    }
}
