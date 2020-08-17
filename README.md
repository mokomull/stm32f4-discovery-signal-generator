# Signal generator using a STM32F407G-DISC1 board

## Quick start

The STM32F407G-DISC1 board is powered through its mini-USB port; this also provides the ST-Link/V2-1
interface to flash the firmware onto the microcontroller.

```
cargo build --release

gdb-multiarch target/thumbv7em-none-eabihf/release/stm32f4-discovery-signal-generator
    (gdb) target remote :3333
    Remote debugging using :3333
    0x00000000 in ?? ()
    (gdb) load
    Loading section .vector_table, size 0x1a8 lma 0x8000000
    Loading section .text, size 0x55d0 lma 0x80001a8
    Loading section .rodata, size 0x1794 lma 0x8005780
    Start address 0x80056de, load size 28428
    Transfer rate: 21 KB/sec, 7107 bytes/write.
    (gdb) c
```

Connect an oscilloscope between PA4 and GND; the default signal is a 2.7Vpp 1000Hz sine wave.  This
is **NOT** an isolated power supply, and averages +1.5V relative to ground.

## Controlling with a PC

This provides a CDC-ACM serial port on the micro-USB connector of the STM32F407G-DISC1 board, which
takes simple commands:

  * `f12345\n` sets the frequency to 12345Hz
  * `v12345\n` sets the voltage to 12345 mV peak-to-peak

The DAC on the STM32F4 seems to be reasonably reliable up to 100 kHz; beyond that, the resulting
amplitude decreases quickly and the wave becomes less sinusoidal and more triangular.

The STM32F407G-DISC1 board cannot provide more than 3.0Vpp on the analog output due to the 3V power
supply for the main microcontroller.

If you are using a terminal application (e.g. `picocom`, `screen`) to set the frequency, try using
Ctrl-J for the `\n` character rather than the enter-key.

## Controlling from an Oscilloscope

This is only currently tested with the automated Bode plot utility on an SDS 1104X-E.

Use a [custom branch of the sds1004x_bode
project](https://github.com/mokomull/sds1004x_bode/tree/stm32f4) to provide a VXI-11 server that the
Siglent scope will control.

It can be run by executing `sudo python bode.py mokomull /dev/ttyACM2`, where `/dev/ttyACM2` is the
device node for the micro-USB serial port (vid 0x1337, pid 0xd00d), **NOT** the ACM device for the
mini-USB port (which is handled by the embedded ST-Link instead).

In the Bode Plot II application, set Source -> Interface -> LAN, and set Source -> Interface -> Set
IP -> (choose the IP address of the computer running `bode.py` using the multi-function knob).

Recommended settings:

  * Set Sweep
    * Mode: Decade
    * Start: 1kHz (minimum value with this signal generator is 250Hz)
    * Stop: 100kHz (with low amplitudes, higher frequencies are possible, though flaky)
  * Set Stimulus
    * Amplitude: choose a value less than approx. 2.9Vpp to avoid distortion
    * Unit: only Vpp has been tested
    * Offset and Load are currently not honored, and the resulting waveform will be a 1.5V-offset
      sine wave, with high-Z output coupling, regardless of choices made here
  * Set Channel
    * DUT Input: the probe attached to the PA4 output
    * DUT Output: the probe attached to the output of the device-under-test.
