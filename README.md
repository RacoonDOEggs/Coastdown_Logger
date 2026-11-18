# DC Motor Coast-Down Test

This tool measures the coast-down of a brushed DC motor. You use the results to estimate the rotor inertia **J** and the friction ratios **b/J** (viscous) and **τc/J** (dry/Coulomb).

The test spins the motor up to a steady speed, cuts the drive (PWM = 0, coasting with no braking) and records the speed until the motor stops. An Arduino runs the test and measures the speed; a Python script on the PC controls the runs and saves the data to a CSV file.

## Project structure

```
.
├── platformio.ini           # PlatformIO configuration (two environments, see below)
├── coastdown_logger.py      # PC-side script: runs tests, writes the CSV
├── README.md
├── include/
│   └── pinout.h             # MOTOR_PWM_PIN, ENCONDER_PIN, ...
└── src/
    ├── main.cpp             # Manual control firmware
    └── coastdown_test.cpp   # Coast-down test firmware
```

## Requirements

**Hardware**
- An Arduino Nano (ATmega328P with the new bootloader, PlatformIO board `nanoatmega328new`). `ENCONDER_PIN` must be **D2 or D3**, the only external-interrupt pins on the Nano. `MOTOR_PWM_PIN` must be a PWM pin (D3, D5, D6, D9, D10 or D11).
- A motor driver that leaves the motor **open circuit (coasting)** when PWM = 0. If the driver brakes by shorting the motor terminals, J will be underestimated.
- An encoder with 50 pulses per revolution. If yours is different, change `ENCODER_PPR` in `coastdown_test.cpp`.

**Software**
- PlatformIO (the VS Code extension or the CLI)
- Python 3.8 or newer, with these packages:

```bash
pip install pyserial matplotlib
```

matplotlib is optional and is only used by `--plot`.

## PlatformIO setup

```ini
; PlatformIO Project Configuration File
; https://docs.platformio.org/page/projectconf.html

; Settings shared by both environments
[env]
platform = atmelavr
board = nanoatmega328new
framework = arduino
monitor_speed = 115200
```

> If uploading fails with `stk500_recv(): programmer is not responding`, your Nano probably has the old bootloader (common on clones). Change the board to `nanoatmega328` in the `[env]` section.

## Running a test

### 1. Upload the coast-down firmware

```bash
pio run -e coastdown -t upload
```

### 2. Find the serial port

```bash
pio device list
```

Typical names for a Nano are `COM3` (or another `COMx`) on Windows, `/dev/ttyUSB0` on Linux, and `/dev/cu.usbserial-*` or `/dev/cu.wchusbserial*` on macOS. Many Nano clones use a CH340 USB chip, which needs a driver on Windows and older macOS versions if the port does not appear.

> Close the PlatformIO serial monitor (and any other program using the port) before you run the logger. Only one program can open the port at a time.

### 3. Run the logger from the project root

```bash
python coastdown_logger.py --port COM3 --pwm 120 180 255 --runs 3 --spinup 5000 --plot
```

This example runs 3 tests at each of the PWM levels 120, 180 and 255, for 9 runs in total, and plots the coast-down curves at the end.

After each run, the script prints a short summary:

```
Run 1: PWM 120, spin-up 5000 ms ...
  end: STOPPED, 812 samples, speed at cut 1843 RPM, coast time 2.95 s
```

Press **Ctrl+C** at any time to abort. The script then tells the Arduino to turn the motor off.

### Command-line options

| Option     | Default                         | Description                                                                 |
|------------|---------------------------------|-----------------------------------------------------------------------------|
| `--port`   | *(required)*                    | Serial port of the Arduino                                                  |
| `--baud`   | `115200`                        | Must match `SERIAL_BAUD` in the firmware                                    |
| `--pwm`    | `200`                           | One or more PWM levels (1–255) to spin up to                                |
| `--runs`   | `3`                             | Number of repeats at each PWM level                                         |
| `--spinup` | `5000`                          | Spin-up time in ms. It must be long enough for the motor to reach a steady speed |
| `--rest`   | `2.0`                           | Pause between runs, in seconds                                              |
| `--out`    | `coastdown_YYYYMMDD_HHMMSS.csv` | Path of the output CSV file                                                 |
| `--plot`   | off                             | Show the coast-down curves when the runs are finished                       |

### 4. Return to manual control

```bash
pio run -e manual -t upload
```

## Output CSV

All runs are written to a single file with one row per sample, taken at 100 Hz. The file is saved after every run, so if a later run fails, the earlier runs are kept.

| Column        | Description                                                                                 |
|---------------|---------------------------------------------------------------------------------------------|
| `run`         | Run index (1, 2, …)                                                                         |
| `pwm`         | PWM used during spin-up                                                                     |
| `phase`       | `0` = spin-up (motor driven), `1` = coasting                                                |
| `t_s`         | Sample time in seconds, relative to the moment the drive was cut (negative values are before the cut) |
| `edge_t_s`    | Time of the most recent encoder pulse, in seconds, relative to the cut                      |
| `count`       | Raw cumulative encoder pulse count                                                          |
| `new_edges`   | Number of pulses since the previous sample                                                  |
| `rpm`         | Speed estimate                                                                              |
| `omega_rad_s` | The same speed in rad/s                                                                     |

### Using the data for fitting

- Keep only the rows where `phase == 1` and `new_edges > 0`. On rows where `new_edges == 0`, no pulse arrived during that sample, so `rpm` is only an upper limit, not a measurement.
- With the motor open circuit, no current flows, so the motor follows

  J·dω/dt = −b·ω − τc

  and the speed decays as

  ω(t) = (ω₀ + τc/b)·e^(−b·t/J) − τc/b

  Fit this curve to `omega_rad_s` against `t_s` to get b/J and τc/J. If b and τc are known from steady-state tests, J follows directly.
- Fit the curve itself rather than differentiating the speed data, because the derivative of encoder data is very noisy.
- Runs from different PWM levels should give the same b/J and τc/J. If they don't, the friction model needs refining.

## How the firmware measures speed

The encoder interrupt counts pulses and records the time of each one with `micros()`. At every sample, the speed is calculated as

```
rpm = new_pulses × 60 / (ENCODER_PPR × time between the first and last of those pulses)
```

This method (often called M/T) stays accurate both at high speed and near standstill, where a fixed counting window would give only a few pulses. When no new pulse arrives during a sample, the reported speed is limited to the largest value consistent with the time since the last pulse. After `STOP_TIMEOUT_US` (1 s) with no pulse, the motor is considered stopped and the run ends.

### Firmware configuration (`src/coastdown_test.cpp`)

| Constant            | Default    | Meaning                                                    |
|---------------------|------------|------------------------------------------------------------|
| `ENCODER_PPR`       | `50`       | Encoder pulses per revolution                              |
| `SERIAL_BAUD`       | `115200`   | Serial speed (keep in sync with `--baud`)                  |
| `SAMPLE_PERIOD_US`  | `10000`    | Logging period (10 ms = 100 Hz)                            |
| `STOP_TIMEOUT_US`   | `1000000`  | Time without a pulse before the motor is considered stopped |
| `MAX_COAST_US`      | `60000000` | Maximum length of the coast phase                          |
| `ENCODER_GLITCH_US` | `50`       | Pulses closer together than this are rejected as noise     |

## Serial protocol

You can also drive the firmware by hand from a serial terminal (115200 baud, newline line ending).

| Direction       | Message                                     | Meaning                                         |
|-----------------|---------------------------------------------|-------------------------------------------------|
| PC → Arduino    | `RUN <pwm> <spinup_ms>`                     | Start one test (PWM 1–255, spin-up 100–60000 ms) |
| PC → Arduino    | `X`                                         | Abort / turn the motor off                      |
| Arduino → PC    | `#READY`                                    | Board has booted                                |
| Arduino → PC    | `#START,<pwm>,<spinup_ms>`                  | Test accepted                                   |
| Arduino → PC    | `D,<t_us>,<phase>,<count>,<edge_us>,<rpm>`  | One sample (times in µs since the test started) |
| Arduino → PC    | `#CUT,<t_us>`                               | The drive was cut at this time                  |
| Arduino → PC    | `#END,<reason>`                             | `STOPPED`, `TIMEOUT` or `ABORTED`               |
| Arduino → PC    | `#ERR,<message>`                            | Invalid command                                 |

## Troubleshooting

**"Could not open port" or "Access denied"**
Another program is using the port. Close the PlatformIO serial monitor or the Arduino IDE.

**"Warning: no #READY from the Arduino"**
The Nano normally resets when the port is opened and prints `#READY` within about 2 seconds. If it doesn't, the board may still be running the manual firmware (upload with `-e coastdown`), or the reset did not happen. The script continues anyway; if the runs then fail, press the reset button on the Nano and start the script again.

**Upload fails with `stk500_recv(): programmer is not responding`**
Your Nano has the old bootloader. Set `board = nanoatmega328` in the `[env]` section of `platformio.ini`.

**"Speed varied X% in the last second of spin-up"**
The motor had not reached a steady speed when the drive was cut. Increase `--spinup`.

**The run ends immediately with a coast time of 0 s**
No encoder pulses are being counted. Check the wiring, check that `ENCONDER_PIN` is D2 or D3, and check that the encoder output needs `INPUT` and not `INPUT_PULLUP` (open-collector encoders need a pull-up).

**The speed looks noisy or too high**
Electrical noise from the motor brushes may be creating false pulses. Shield or shorten the encoder cable, add a small RC filter, or raise `ENCODER_GLITCH_US`. Keep it below the pulse period at your top speed: 60 / (RPM_max × PPR) seconds.

**The motor stops much faster than expected**
Check that the driver really coasts at PWM = 0 rather than braking.

**Linker error: multiple definition of `setup` / `loop`**
You are building both firmware files together. Build with `-e coastdown` or `-e manual` as described in [PlatformIO setup](#platformio-setup).
