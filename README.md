# DC Motor Coast-Down Test

This tool measures the coast-down of a brushed DC motor. You use the results to estimate the rotor inertia **J** and the friction ratios **b/J** (viscous) and **τc/J** (dry/Coulomb).

Each test waits for the motor to be stationary, spins it up until its speed is steady, cuts the drive (PWM = 0, coasting with no braking) and records the speed until the motor stops. An Arduino runs the test and measures the speed, a Python script on the PC controls the runs and saves the data to a CSV file, and a second script fits the coast-down curves to get b/J and τc/J (and J itself when b or τc is known).

## Project structure

```
.
├── platformio.ini           # PlatformIO configuration
├── coastdown_logger.py      # PC-side script: runs tests, writes the CSV
├── coastdown_fit.py         # PC-side script: fits the coast-down curves in the CSV
├── README.md
├── include/
│   └── pinout.h             # MOTOR_PWM_PIN, ENCONDER_PIN, ...
└── src/
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
pip install pyserial numpy scipy matplotlib
```

`coastdown_logger.py` needs only pyserial (plus matplotlib for `--plot`). `coastdown_fit.py` needs numpy and scipy (plus matplotlib for `--plot` and `--save-plot`).

## PlatformIO setup

```ini
; PlatformIO Project Configuration File
; https://docs.platformio.org/page/projectconf.html

[env:nanoatmega328new]
platform = atmelavr
board = nanoatmega328new
framework = arduino
monitor_speed = 115200
```

> If uploading fails with `stk500_recv(): programmer is not responding`, your Nano probably has the old bootloader (common on clones). Change the board to `nanoatmega328` in `platformio.ini`.

## Running a test

### 1. Upload the coast-down firmware

```bash
pio run -t upload
```

### 2. Find the serial port

```bash
pio device list
```

Typical names for a Nano are `COM3` (or another `COMx`) on Windows, `/dev/ttyUSB0` on Linux, and `/dev/cu.usbserial-*` or `/dev/cu.wchusbserial*` on macOS. Many Nano clones use a CH340 USB chip, which needs a driver on Windows and older macOS versions if the port does not appear.

> Close the PlatformIO serial monitor (and any other program using the port) before you run the logger. Only one program can open the port at a time.

### 3. Run the logger from the project root

```bash
python coastdown_logger.py --port COM3 --pwm 120 180 255 --runs 3 --plot
```

This example runs 3 tests at each of the PWM levels 120, 180 and 255, for 9 runs in total, and plots the coast-down curves at the end.

Each run goes through three stages, all handled by the firmware:

1. **Wait for standstill.** If the motor is still turning (for example from the previous run), the firmware waits until no encoder pulse has been seen for 1 s. The script prints `waiting for the motor to stop ...` while this happens.
2. **Spin-up.** The motor is driven at the requested PWM. The firmware averages the speed over 1-second blocks and cuts the drive as soon as two consecutive blocks differ by less than 0.3 %. If that never happens, the drive is cut after `--spinup` ms.
3. **Coast-down.** PWM is set to 0 and the speed is logged until the motor stops.

After each run, the script prints a short summary:

```
Run 1: PWM 120, max spin-up 30000 ms ...
  spin-up: 9.0 s (STEADY), speed at cut 2821 RPM
  coast:   6.08 s, end: STOPPED, 1514 samples
```

`STEADY` means the speed had settled when the drive was cut. `MAX_TIME` means the `--spinup` limit was reached first, and the script prints a warning.

Press **Ctrl+C** at any time to abort. The script then tells the Arduino to turn the motor off.

### Command-line options

| Option     | Default                         | Description                                                                 |
|------------|---------------------------------|-----------------------------------------------------------------------------|
| `--port`   | *(required)*                    | Serial port of the Arduino                                                  |
| `--baud`   | `115200`                        | Must match `SERIAL_BAUD` in the firmware                                    |
| `--pwm`    | `200`                           | One or more PWM levels (1–255) to spin up to                                |
| `--runs`   | `3`                             | Number of repeats at each PWM level                                         |
| `--spinup` | `30000`                         | Maximum spin-up time in ms (100–60000). The drive is normally cut earlier, as soon as the speed is steady |
| `--rest`   | `0.5`                           | Extra pause between runs, in seconds. The firmware already waits for the motor to stop before each run |
| `--out`    | `coastdown_YYYYMMDD_HHMMSS.csv` | Path of the output CSV file                                                 |
| `--plot`   | off                             | Show the coast-down curves when the runs are finished                       |

### 4. Fit the results

```bash
python coastdown_fit.py coastdown_YYYYMMDD_HHMMSS.csv --plot
```

See [Fitting the results](#fitting-the-results) for details.

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

## Fitting the results

`coastdown_fit.py` fits the coast-down curves in a CSV written by the logger.

### The model

With the motor open circuit, no current flows, so the rotor obeys

```
J·dω/dt = −b·ω − τc        (while ω > 0)
```

and the speed decays as

```
ω(t) = (ω₀ + a/β)·e^(−β·t) − a/β,     β = b/J,  a = τc/J
```

The coast-down alone can only give the ratios **b/J** (in 1/s) and **τc/J** (in rad/s²). To get **J**, you need b or τc from a separate test, such as a steady-state sweep of voltage against speed:

```
J = b / (b/J)     or     J = τc / (τc/J)
```

### Running the fit

From the project root:

```bash
python coastdown_fit.py coastdown_20261117_220222.csv --plot
```

If you know b and/or τc, pass them to get J:

```bash
python coastdown_fit.py coastdown_20261117_220222.csv --b 1.2e-4 --tau-c 3.5e-3 --save-plot fit.png
```

The script does two fits:

1. **Per run** (ω₀, b/J and τc/J all free). The spread between runs shows how repeatable the test is.
2. **Joint fit** over all runs, with b/J and τc/J shared and a separate ω₀ for each run. This is the final estimate.

Example output:

```
Per-run fits (±1σ)
 run  pwm   pts   ω0 [RPM]          b/J [1/s]        τc/J [rad/s²]  RMSE [RPM]
   1  120   603       2820    0.3000 ± 0.0003     15.001 ± 0.032           4.0
   2  120   605       2830    0.3000 ± 0.0003     15.002 ± 0.031           4.0
   ...

Joint fit over all runs (±1σ)
  b/J  = 0.29997 ± 0.00013 1/s   (mechanical coast time constant J/b = 3.33 s)
  τc/J = 15.003 ± 0.015 rad/s²
  RMSE = 4.0 RPM
  Viscous and Coulomb friction are equal at 478 RPM (viscous dominates above, Coulomb below).

  J = b / (b/J)     = 0.001 kg·m²   (from --b 0.0003)
  J = τc / (τc/J)   = 0.0009998 kg·m²   (from --tau-c 0.015)
  The two estimates of J differ by 0.0 %.
```

When both `--b` and `--tau-c` are given, the difference between the two J estimates is a useful check of the whole parameter set: a large difference means at least one of b, τc or the friction model is off.

### Fit options

| Option        | Default                | Description                                                              |
|---------------|------------------------|--------------------------------------------------------------------------|
| `csv`         | *(required)*           | CSV file written by `coastdown_logger.py`                                |
| `--runs`      | all                    | Only fit these run numbers, e.g. `--runs 1 2 3`                          |
| `--min-rpm`   | `0`                    | Ignore points below this speed (to exclude stiction effects near standstill) |
| `--b`         | —                      | Viscous friction b in N·m·s/rad, to compute J                            |
| `--tau-c`     | —                      | Coulomb friction τc in N·m, to compute J                                 |
| `--out`       | `<input>_fit.csv`      | Path of the results CSV                                                  |
| `--plot`      | off                    | Show the data, fitted curves and residuals                               |
| `--save-plot` | —                      | Save the plot to an image file (e.g. `fit.png`)                          |

### Results CSV

One row per run (`fit = run`) and one row for the joint fit (`fit = joint`), with columns `run`, `pwm`, `points`, `omega0_rad_s`, `b_over_J`, `b_over_J_err`, `tauc_over_J`, `tauc_over_J_err`, `rmse_rad_s`, `J_from_b` and `J_from_tauc`. The `_err` columns are 1σ uncertainties, and the J columns are filled only on the joint row when `--b` or `--tau-c` is given.

### Which data points are used

- Only rows with `phase == 1` (coasting) and `new_edges > 0` are used. On rows where `new_edges == 0`, no pulse arrived during that sample, so `rpm` is only an upper limit, not a measurement.
- Each speed is an average over the interval between two encoder pulses, so the fit timestamps it at the middle of that interval (from `edge_t_s`), not at the sample time.
- Intervals that started before the drive was cut are dropped, because they mix driven and coasting motion.
- Runs with fewer than 5 usable points are skipped.

### Checking the fit

- **Residuals:** in the plot, they should be scattered randomly around zero. A clear S-shape or other pattern means the viscous-plus-Coulomb model is missing something, such as friction that grows faster than linearly with speed (a fan or air drag on the load).
- **Consistency:** runs at different PWM levels should give the same b/J and τc/J. If they don't, that also points to the friction model.
- **Low-speed points:** stiction can distort the last few points before the motor stops. If they pull the fit off, exclude them with `--min-rpm` (for example `--min-rpm 100`).

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
| `MAX_WAIT_STOP_MS`  | `120000`   | Maximum time to wait for the motor to stop before a run    |
| `STEADY_BLOCK_SAMPLES` | `100`   | Samples per averaging block for steady-state detection (100 = 1 s) |
| `STEADY_TOL_PCT`    | `0.3`      | Two consecutive blocks closer than this (%) count as steady |
| `ENCODER_GLITCH_US` | `50`       | Pulses closer together than this are rejected as noise     |

## Serial protocol

You can also drive the firmware by hand from a serial terminal (115200 baud, newline line ending).

| Direction       | Message                                     | Meaning                                         |
|-----------------|---------------------------------------------|-------------------------------------------------|
| PC → Arduino    | `RUN <pwm> <max_spinup_ms>`                 | Start one test (PWM 1–255, maximum spin-up 100–60000 ms) |
| PC → Arduino    | `X`                                         | Abort / turn the motor off                      |
| Arduino → PC    | `#READY`                                    | Board has booted                                |
| Arduino → PC    | `#START,<pwm>,<max_spinup_ms>`              | Test accepted                                   |
| Arduino → PC    | `#WAIT`                                     | Motor still turning; waiting for it to stop (sent every second) |
| Arduino → PC    | `D,<t_us>,<phase>,<count>,<edge_us>,<rpm>`  | One sample (times in µs since the motor was switched on) |
| Arduino → PC    | `#CUT,<t_us>,<reason>`                      | The drive was cut at this time; reason is `STEADY` or `MAX_TIME` |
| Arduino → PC    | `#END,<reason>`                             | `STOPPED`, `TIMEOUT` or `ABORTED`               |
| Arduino → PC    | `#ERR,<message>`                            | Invalid command, or the motor did not stop before a run |

## Troubleshooting

**"Could not open port" or "Access denied"**
Another program is using the port. Close the PlatformIO serial monitor or the Arduino IDE.

**"Warning: no #READY from the Arduino"**
The Nano normally resets when the port is opened and prints `#READY` within about 2 seconds. If it doesn't, the board may be running different firmware (upload again with `pio run -t upload`), or the reset did not happen. The script continues anyway; if the runs then fail, press the reset button on the Nano and start the script again.

**Upload fails with `stk500_recv(): programmer is not responding`**
Your Nano has the old bootloader. Set `board = nanoatmega328` in `platformio.ini`.

**"Speed was not steady when the drive was cut"**
The spin-up reached the `--spinup` limit before the speed settled. Increase `--spinup` (up to 60000 ms). If the speed never settles, for example because the load fluctuates, increase `STEADY_TOL_PCT` in the firmware.

**"Arduino error: motor did not stop"**
The motor kept turning for `MAX_WAIT_STOP_MS` (2 minutes) before a run. Check that PWM = 0 really removes the drive, and that the encoder isn't picking up noise or vibration while the motor is stationary.

**The run ends immediately with a coast time of 0 s**
No encoder pulses are being counted. Check the wiring, check that `ENCONDER_PIN` is D2 or D3, and check that the encoder output needs `INPUT` and not `INPUT_PULLUP` (open-collector encoders need a pull-up).

**The speed looks noisy or too high**
Electrical noise from the motor brushes may be creating false pulses. Shield or shorten the encoder cable, add a small RC filter, or raise `ENCODER_GLITCH_US`. Keep it below the pulse period at your top speed: 60 / (RPM_max × PPR) seconds.

**The motor stops much faster than expected**
Check that the driver really coasts at PWM = 0 rather than braking.

**`coastdown_fit.py`: "No usable coast-down data in the file"**
No run has at least 5 coasting points with real measurements. This happens with files recorded before the firmware fix in commit `e0b3db9`, where each run ended right after the cut. Record new data with the current firmware.

**`coastdown_fit.py`: b/J comes out as 0, or the fit is poor**
A b/J of 0 means the fit found no viscous component, so the motor slows almost linearly (Coulomb friction dominates). Check the residual plot; if the low-speed points are the problem, try `--min-rpm`. Very short coast-downs (only a few dozen points) also give poorly determined values, so spin up to a higher PWM if you can.

**Linker error: multiple definition of `setup` / `loop`**
More than one firmware file is in `src/`. PlatformIO compiles every file there, so keep only `coastdown_test.cpp`, or give each firmware its own environment with `build_src_filter`.
