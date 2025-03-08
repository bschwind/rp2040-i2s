# rp2040-i2s

Read and write to I2S devices like MEMS microphones or DACs, on the RP2040 microcontroller.

## Usage

### I2S Output

```rust
let mut pac = pac::Peripherals::take().unwrap();
let _core = pac::CorePeripherals::take().unwrap();

let mut watchdog = Watchdog::new(pac.WATCHDOG);

// Start Clocks
// Found with: https://github.com/bschwind/rp2040-clock-calculator
let desired_system_clock = 61440000.Hz();

// Exercise for the reader
let clocks = set_system_clock_exact(
    desired_system_clock,
    pac.XOSC,
    pac.CLOCKS,
    pac.PLL_SYS,
    pac.PLL_USB,
    &mut pac.RESETS,
    &mut watchdog,
)
.unwrap();

let sio = rp2040_hal::Sio::new(pac.SIO);

let pins =
    rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

// PIO Globals
let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

// I2S output
let dac_output = I2SOutput::new(&mut pio0, sm0, pins.gpio6, pins.gpio7, pins.gpio8).unwrap();

let (dac_sm, _dac_fifo_rx, dac_fifo_tx) = dac_output.split();

// Write your audio samples to `dac_fifo_tx` in your favorite way (direct read or DMA transfer).
```

### I2S Input

```rust
let mut pac = pac::Peripherals::take().unwrap();
let _core = pac::CorePeripherals::take().unwrap();

let mut watchdog = Watchdog::new(pac.WATCHDOG);

// Start Clocks
// Found with: https://github.com/bschwind/rp2040-clock-calculator
let desired_system_clock = 61440000.Hz();

// Exercise for the reader
let clocks = set_system_clock_exact(
    desired_system_clock,
    pac.XOSC,
    pac.CLOCKS,
    pac.PLL_SYS,
    pac.PLL_USB,
    &mut pac.RESETS,
    &mut watchdog,
)
.unwrap();

let sio = rp2040_hal::Sio::new(pac.SIO);

let pins =
    rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

// PIO Globals
let (mut pio0, _, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);

// I2S input
let mic_input = I2SInput::new(&mut pio0, sm1, pins.gpio10, pins.gpio11, pins.gpio12).unwrap();

let (mic_sm, mic_fifo_rx, _mic_fifo_tx) = mic_input.split();

// Read mic audio samples from `mic_fifo_rx` in your favorite way (direct read or DMA transfer).
```

### I2S Multi-Input

One PIO state machine can generate bit clock and word clock signals, while other PIO state
machines can synchronize with the clock generator PIO and just read IS2 data at the proper
timing. This reduces the number of pins required when interfacing with multiple microphones.

```rust
let mut pac = pac::Peripherals::take().unwrap();
let _core = pac::CorePeripherals::take().unwrap();

let mut watchdog = Watchdog::new(pac.WATCHDOG);

// Start Clocks
// Found with: https://github.com/bschwind/rp2040-clock-calculator
let desired_system_clock = 61440000.Hz();

// Exercise for the reader
let clocks = set_system_clock_exact(
    desired_system_clock,
    pac.XOSC,
    pac.CLOCKS,
    pac.PLL_SYS,
    pac.PLL_USB,
    &mut pac.RESETS,
    &mut watchdog,
)
.unwrap();

let sio = rp2040_hal::Sio::new(pac.SIO);

let pins =
    rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

// PIO Globals
let (mut pio0, _, sm1, sm2, _) = pac.PIO0.split(&mut pac.RESETS);

// I2S input
let mic_1_input = I2SInput::new(&mut pio0, sm1, pins.gpio10, pins.gpio11, pins.gpio12).unwrap();
let mic_2_input = I2SInput::new_data_only(&mut pio0, sm2, pins.gpio15).unwrap();

let (mic_1_sm, mic_1_fifo_rx, _mic_1_fifo_tx) = mic_1_input.split();
let (mic_2_sm, mic_2_fifo_rx, _mic_2_fifo_tx) = mic_2_input.split();

// Synchronize both PIO state machines so they start at the exact same time and
// execute their instructions in lockstep.
mic_sm.with(mic_2_sm).sync().start();

// Receive mic data from `mic_1_fifo_rx` and `mic_2_fifo_rx`.
```

## Code Format

The formatting options currently use nightly-only options.

```
$ cargo +nightly fmt
```

## Code Linting

```
$ cargo clippy --all-targets -- -D warnings
```
