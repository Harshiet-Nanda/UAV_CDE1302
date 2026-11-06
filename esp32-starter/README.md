# esp-everything-qgc-bridge

ESP32-S3 companion-computer firmware for CDE1302.
It consists of ESP32 firmware in `main/` and laptop-side Python scripts in `laptop/`.

Note that the example drone ID in this README is 22 (it's indicated as [drone ID] in some places). You need to change it to your drone's ID when running the scripts.

## 1. Setup ESP32 (Optional)

You can skip this if you only want to run the laptop scripts. The ESP32 firmware has already been built and flashed onto the drone's ESP32-S3.

In this step, you need an internet connection to build and close the QGroundControl app if it is running. The ESP32-S3 must be connected to the laptop via USB.

Everything this firmware needs is committed in-tree (MAVLink `c_library_v2`,
esp-apriltag, the VL53L5CX driver and `managed_components/`). The only external
dependency is ESP-IDF itself:

```bash
git clone https://github.com/espressif/esp-idf.git ../esp-idf
cd ../esp-idf && git checkout "$(sed -n 's/^IDF_CHECKOUT_REF=//p' ../esp-everything-qgc-bridge/tools/idf-pin.env)"
git submodule update --init --recursive
./install.sh esp32s3          # ~2 GB of toolchain into ~/.espressif
cd -
```

See `tools/idf-pin.env` for the pinned version and why it matters.

### Build and flash a drone

For example, let drone ID = 22 and the ESP32-S3 is on `/dev/tty.usbmodem2101` (default MacBook Pro USB-C port):

`flash_drone.sh` pins `CONFIG_DRONE_ID` and `CONFIG_HOST_IPV4_ADDR`
(host IP = `192.168.1.<100 + drone id>`) into `sdkconfig`, builds, verifies the
generated header really is that drone, flashes, then restores `sdkconfig`:

```bash
./flash_drone.sh                           # prompts for drone ID and port
./flash_drone.sh 22                        # prompts for the port
./flash_drone.sh 22 /dev/tty.usbmodem2101  # non-interactive
```

It finds ESP-IDF automatically (`$IDF_PATH`, `./esp-idf`, `../esp-idf`,
`~/esp/esp-idf`, `~/esp-idf`), sourcing `export.sh` only if needed.

### Deterministic drone addresses

`flash_drone.sh` asks for the drone ID when no ID argument is supplied and
builds the firmware with this mapping:

```text
drone IPv4 address = 192.168.1.(200 + drone ID)
```

For example, drone 9 is `192.168.1.209` and drone 22 is
`192.168.1.222`. This is separate from `CONFIG_HOST_IPV4_ADDR`: the flash
script retains the existing QGC/laptop mapping `192.168.1.(100 + drone ID)`.
After flashing, it prints the exact `camera_stream.py --esp-ip ...` command.
The WiFi router must use gateway `192.168.1.1` with netmask
`255.255.255.0`.

## 2. Setup (Laptop)

You can choose one of the following two options to install the laptop-side Python scripts and dependencies:

### Pip install (Python 3.10+)

Laptop side (Python 3.10+), with pip:

```bash
python3 -m pip install -r laptop/requirements.txt
```

or in a conda environment (below), which brings its own Python, so it also
works on a machine whose Python is too old or shared with other projects.

### Conda environment

`laptop/environment.yml` creates an environment named `cde1302-laptop` with
Python 3.12 and the packages from `laptop/requirements.txt`. Every package
installs prebuilt binaries on 64-bit Windows, Linux with glibc 2.27+ (Ubuntu 18.04+),
and macOS 13+ (14+ on an Intel Mac); older systems fall back to compiling
OpenCV or SciPy from source.

1. Install conda once per machine. An existing Anaconda or Miniconda works;
   on a new machine, install Miniforge (on Windows, see [Windows](#windows-havent-tested-yet)):

   ```bash
   curl -LO "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
   bash "Miniforge3-$(uname)-$(uname -m).sh"   # answer yes when it offers to initialize conda
   ```

   Then open a new terminal. If `conda activate` later says to run
   `conda init`, run `conda init zsh` (or `bash`) and open a new terminal.

2. Create the environment once from the repo root:

   ```bash
   conda env create -f laptop/environment.yml
   ```

3. Activate it in every new terminal before running the scripts:

   ```bash
   conda activate cde1302-laptop
   python laptop/tag_stream.py --esp-ip 192.168.1.222   # for example
   conda deactivate                          # when done
   ```

After `laptop/requirements.txt` changes, update the environment with
`conda env update -f laptop/environment.yml`. The laptop tests run with
`cd laptop && python -m pytest tests`.

### C compiler

`tag_stream.py` compiles the drone's AprilTag detector on its first run, so
it needs a C compiler: `xcode-select --install` on macOS,
`sudo apt install build-essential` on Ubuntu. For Windows, see step 3 of
[Windows](#windows-havent-tested-yet).

### Windows (Haven't tested yet)

Follow the conda steps above, with these changes:

1. **Folders:** put the repo and conda in paths with only English letters
   and no spaces, e.g. `C:\cde1302` and `C:\miniforge3`. The compiler in
   step 3 can't handle other paths.
2. **Installing conda:** run `Miniforge3-Windows-x86_64.exe` from
   <https://github.com/conda-forge/miniforge/releases/latest> and type every
   command in "Miniforge Prompt" from the Start menu ("Anaconda Prompt" with
   Anaconda). Don't use WSL: its default networking never passes the drones'
   packets on.
3. **C compiler:** after creating the environment, install MinGW-w64 gcc into
   it (about 110 MB; Visual Studio's compiler won't work) and check the build:

   ```bat
   conda activate cde1302-laptop
   conda install -c conda-forge gcc
   python laptop/apriltag_host.py
   ```

   It should print `tag16h5 ncodes = 22 (expected 22)`.
4. **`python`, not `python3`:** conda's Python on Windows has no `python3`
   command, so type `python` wherever this README says `python3`.
5. **Firewall:** the drones send telemetry and video to the laptop unprompted,
   so allow Python through Windows Defender Firewall when it asks, and set
   the drone Wi-Fi to a Private network. It asks once per environment; if you
   clicked Cancel, remove the block rule under "Allow an app through
   firewall".

### Laptop IP address

The drones send MAVLink, telemetry, and ToF debug data to
`192.168.1.(100 + drone ID)` (set when flashing), so the QGroundControl or
mission laptop must hold that address on the drone Wi-Fi. Camera video follows
the source IP of the latest `camera_stream.py` or `tag_stream.py` keepalive and
can therefore be viewed from another laptop on the same Wi-Fi. The small
AprilTag debug stream is also copied to that viewer when
`tag_stream_legacy.py` uses `--detail`.

## 3. ESP32 camera preview

View a drone's camera over WiFi (drone IP = `192.168.1.(200 + drone ID)`; here the example drone ID is 22, so the drone IP becomes 192.168.1.222):

```bash
python3 laptop/camera_stream.py --esp-ip 192.168.1.222 [--fps 10] [--quality 60]
```

`s` saves a frame.

With AprilTags drawn on the frame:
```bash
python3 laptop/tag_stream.py --esp-ip 192.168.1.222
```

For the camera pose expressed in each marker frame, use:
```bash
python3 laptop/tag_stream_unprocessed.py --esp-ip 192.168.1.222
```
It boxes tags and shows/prints `tvec` (camera centre in marker coordinates,
metres) and `rvec` (Rodrigues axis-angle, radians; camera-to-marker rotation).
The detector pose is inverted using R_inv = R.T and t_inv = -R.T @ t.
Add `--no-display` for terminal-only output. The detector still uses its
existing intrinsics, 0.12 m tag size and Hamming/margin gate; no smoothing or
body/local transform is applied. Keys: q/Esc quit, s saves the annotated view.

For tuning, use `tag_stream_legacy.py --detail` (or `d` while it runs): it adds
rejected tags, a HUD and the drone's own detections.

For both commands, `q`/Esc quits. The drone streams only while the viewer
runs (~9 fps, ~130 ms from capture to the laptop on the OV3660). The overlay
and the console show the drone-side frame age and dropped frames.

Only one camera destination is active at a time: if viewers on two laptops
send keepalives, the most recent keepalive wins. The Wi-Fi access point must
allow client-to-client traffic, and the viewer firewall must allow Python to
receive UDP 5008 and 5009. MAVLink remains on `CONFIG_HOST_IPV4_ADDR` while a
different laptop is viewing the camera.

## 4. Simple arming and takeoff (Lab 5)

`--drone-id` (required) is the ID flashed into the ESP32. The camera/AprilTag
pipeline (section 3) never affects flights: tags are only detected and reported.

Check the communication without sending flight commands.
```bash
python3 laptop/simple_arm_takeoff_land.py --monitor-only --drone-id [DRONE_ID]
```

Takeoff -> hover at 0.5 m altitude -> land. The altitude is `CRUISE_ALT_M` in the ESP32 firmware (`main/main.c`; keep `WIFI_CRUISE_ALT_M` in `main/wifi_task.c` equal).
```bash
python3 laptop/simple_arm_takeoff_land.py --drone-id [DRONE_ID]
```
Type `ARM-[drone ID]` when prompted.

Full procedure: [laptop/TEST_GUIDE.md](laptop/TEST_GUIDE.md).

## 5. Send a waypoint mission (Lab 5)

```bash
python3 laptop/simple_waypoint_mission.py \
  --waypoints-file waypoints/waypoints_example.txt --confirm \
  --drone-id [DRONE_ID]
```

`--confirm` asks for `MISSION-[drone ID]`; without it, the drone takes off as
soon as telemetry arrives. Waypoints are `x,y` metres (x = north, y = east) from
PX4's local origin, plus the drone's start offset in `laptop/setup.yaml` if listed.

Full procedure: [waypoints/WAYPOINT_FLIGHT_TEST_GUIDE.md](waypoints/WAYPOINT_FLIGHT_TEST_GUIDE.md).

## 6. Fly from a CSV trajectory

This is a 2x1 ellipse minimum-snap trajectory. Put the drone at the tip of the major axis, facing outward. The takeoff placement can be seen in `trajectory/ellipse_start_placement.png`.

```bash
python3 laptop/send_trajectory.py --takeoff --takeoff-wait 12 \
  --trajectory trajectory/ellipse_min_snap_traj.csv --confirm \
  --drone-id [DRONE_ID]
```

`--confirm` asks for `TRAJ-[drone ID]` before anything is sent.

See `trajectory/README.md` for the included circle and compact minimum-snap ellipse, how to regenerate their CSV files, and the space each path needs.

## Onboard-only detection dashboard (`harshiet`, 5 November 2026)

On `harshiet`, the trajectory and waypoint logged launchers and standalone
`plot_trajectory.py --listen --drone-id 7` now consume only the ESP32 AprilTag
results on UDP 5008. They never start SnapshotSaver, request camera frames, or
instantiate the laptop detector. The previous camera panel shows onboard IDs,
camera-relative tvec values, processing time and fresh/stale status instead.
Existing tag map placement/fusion and CSV summaries remain laptop geometry on
ESP32 measurements, not laptop image detection. Waypoint runs now log these
onboard detections too. The small status sidecar `<flight>_detections.json`
includes empty results so old tags clear from the panel. `--no-snaps` remains a
compatibility option; video and image snapshots are always off in these flows.

No firmware change/reflash is required. Close separately launched camera/tag
viewers or older mission processes: those can still request video and run their
own laptop detector. Without keepalives the existing firmware stops video after
2.5 seconds. Small detection packets, telemetry, MAVLink and mission commands
still cross Wi-Fi. At most eight detections fit an existing 251-byte payload,
sent at 10 Hz (up to 2.51 KB/s of detection payload, excluding network overhead).
No guarantee of Wi-Fi or flight performance is implied. Onboard pose has no
transmitted rotation; existing floor-height, yaw-only transform and timing
assumptions remain. Image snapshots/preview are no longer collected by the GUI.


## Gradual turnaround before the reverse snake

On local `harshiet`, based on `kaiwei` (4 November 2026), `send_trajectory_logged.py` now uses a
position-holding firmware turn before `--return-trajectory`. Build and flash
the updated ESP32 firmware before using this feature; older firmware does not
understand `CMD_TURN` and the script aborts rather than using GOTO hops.
The normal mission arguments and CSVs are unchanged. `--no-turn` explicitly
skips the turn and keeps the original camera direction.

The firmware holds the captured local NED X/Y/Z, waits for low translational
speed for one second, then changes yaw along a quintic profile with zero
initial/final commanded speed and acceleration. Initial limits in
`main/turn_profile.h` are **20 deg/s** and **10 deg/s²**; the 180-degree profile
lasts about **16.9 seconds**, plus braking and settling. These are commanded
profile limits, not guarantees on measured vehicle motion. `--turn left|right`
(or return-CSV metadata) chooses the rotation direction.

The target is the first pass's starting heading plus 180 degrees, independent
of intermediate overshoot. At the end of the profile the firmware continues
holding that fixed yaw so PX4 corrects residual error. Completion requires
heading error <=5 degrees, measured yaw speed <=5 deg/s, translational speed
<=0.10 m/s, horizontal/vertical position errors <=0.15 m, for one second.
The laptop additionally requires one second of fresh aligned `TURN_DONE`
telemetry. Trajectory start rechecks alignment and retains the intended target
yaw, rather than capturing residual heading error. The existing slow glide
to the return CSV's start is preserved.

A turn exceeding 45 seconds, 0.5 m horizontal drift, 0.3 m vertical drift,
or losing fresh PX4 telemetry/Offboard/armed state is marked failed. The laptop
requires a TURN acknowledgement within three seconds and fresh telemetry
within 0.5 seconds; it will not launch the reverse pass on failure. It sends
HOLD on turn abort, then the mission runner's existing exception handler sends
LAND when it considers the flight active. The existing RC/mode takeover
behaviour in the mission firmware remains applicable.

For a supervised turn-only check after flashing (this command arms/takes off):

```sh
python laptop/send_trajectory_logged.py --drone-id 7 --turn-test --turn left --confirm --takeoff-wait 15
```

Software checks cover profile speed/acceleration, angle wraparound, overshoot
correction, completion gating, duplicate requests, interruption and failures.
No hardware turn or flight has been performed by the agent; validate these
initial limits on the actual vehicle before using a full coverage mission.


## Verify launch X/Y after takeoff and before normal landing

Local `harshiet`, based on `kaiwei`, 4 November 2026: the firmware saves the disarmed local NED X/Y
immediately before requesting Offboard/arming. This reference is captured anew
for every START; it is not assumed to be (0,0), and is not recaptured after
arming drift or after the flight. Takeoff holds that X/Y. After reaching the
existing cruise-altitude target, the firmware checks measured X/Y, slowly
corrects toward the saved reference and requires position/speed to settle before
accepting laptop navigation. A takeoff timeout now blocks mission readiness
instead of allowing an unsuccessful takeoff to enter laptop control.

For normal completion, `send_trajectory_logged.py` and `waypoint_logged.py`
send **CMD_HOME_LAND (0x0B)**. This immediately gates further movement requests,
cancels navigation, checks actual X/Y against the same pre-arm reference and
corrects horizontally at the current altitude. Only verified alignment permits
the normal PX4 LAND request. The existing reverse snake ends at its start in
its CSV; this step corrects residual flight error. It is a bounded drift
correction, not a general return route from elsewhere in the arena.

Initial settings in `main/xy_recovery.h`:

- Horizontal error <= **0.20 m after takeoff**, **0.10 m before home landing** (Euclidean X/Y distance).
- Total speed <= **0.10 m/s** and altitude within **0.15 m**, for **one second**
  using fresh position samples.
- Quintic correction profile: commanded speed <= **0.10 m/s**, acceleration
  <= **0.05 m/s²**. Hold heading and altitude during the correction.
- Maximum home error **1.0 m** and timeout **30 s** for each correction. Excess
  error, stale data, loss of Wi-Fi/flight mode or excess altitude error blocks
  normal completion. Firmware reports failure and holds for LAND/pilot action.

**CMD_LAND (0x02) remains immediate, in-place landing.** It overrides recovery
and is still used by abort/exception handlers; an abort may therefore land away
from the launch position. HOLD stops an active recovery. RC mode takeover is
respected. Older lab clients that send CMD_LAND directly retain immediate
landing behaviour; use the updated logged clients for verified normal landing.

The logged clients now watch fresh TAKEOFF_ALIGN -> FLIGHT_READY status rather
than relying on a blind sleep (`--takeoff-wait` is a minimum delay). Normal
landing requires HOME_ALIGN -> HOME_READY acknowledgement. A correction failure
or timeout aborts the logged mission; its existing fallback sends immediate
LAND. The CSV logger records these state names. The firmware serial log prints
the saved launch coordinates, measured coordinates, horizontal error and
successful settling. States 9..13 are appended without changing telemetry size.

Build/flash the updated ESP32 firmware and run the updated laptop code together.
Software tests simulate pre-arm drift, takeoff recovery, normal landing,
non-convergence, stale position samples, lost links and immediate LAND override.
No hardware flight has been performed. This checks PX4's estimated local
position, not an independent physical reference: estimator drift and drift during
the subsequent PX4 descent can still change the physical touchdown location.
The inherited absolute takeoff Z target (-0.5 m local NED) is unchanged.


## Alignment reception gaps — 5 November 2026

On harshiet, the laptop no longer aborts takeoff/home-landing alignment merely
because telemetry is more than 0.5 seconds old. It waits for newly received
alignment states, retaining the 55-second overall limit, 3-second home-command
acknowledgement limit, explicit firmware failure/interruption checks and minimum
takeoff wait. An old READY packet cannot advance the mission after a receive gap.
This is a laptop-only change: no ESP32 reflash. Turn-specific freshness checks,
waypoint travel timeout and firmware PX4 freshness requirements are unchanged.
A lost link can now delay the alignment abort until the remaining limit expires;
this does not repair the underlying telemetry gap.


## Takeoff diagnostics — 6 November 2026

On branch `harshiet2`, ESP32 serial logs distinguish the 10-second altitude gate
from subsequent XY recovery. `TAKEOFF` reports gate pass, timeout or interruption;
`ALIGN` reports entry checks, Wi-Fi/telemetry/distance/altitude failures, mode
release and recovery timeout. Rate-limited snapshots include armed/mode, Wi-Fi,
position validity, position/attitude/heartbeat ages, measured and target NED Z,
launch XY error and speed. Settling logs expose individual XY/Z/speed/profile
conditions. The laptop failure message points to these logs. Flight thresholds
and failure behavior are unchanged; takeoff radius remains 0.20 m and home radius
0.10 m. No assumption that ALIGN_FAILED necessarily means horizontal drift.
