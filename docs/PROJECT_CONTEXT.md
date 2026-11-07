# CDE1302 drone mini-project context

Retrieved and reviewed 21 October 2026 (Singapore time).

## Sources and local copies

Course: https://canvas.nus.edu.sg/courses/96675

- CDE1302 Mini Project.pdf: Requirements, 7 pages.
- CDE1302-Mini Project-Briefing.pdf: Canvas file 10398107, 18 slides; Canvas updated 19 October. Read slide text and inspected the arena illustration visually.
- Lab 1 setup; Lab 2, 3, 4 and 5 prelabs; Lab 4 and 5 reports: local PDF copies and extracted text in this folder.
- Risk assessment template: CDE1302 Risk Assessment Template.xls (downloaded, not completed).
- MajorQuest Drone Asm-1.zip: 56 CAD files including SolidWorks and STEP files; archive inspected, no mechanical design validation performed.
- Supporting control, PID and trajectory-generation lectures copied from existing course material cache.

## Repository

Course Lab 5 explicitly points to https://github.com/linfengfrank/esp-everything-qgc-bridge .

Recent upstream changes: September 27 trajectory timing adjustment; October 3 printable marker generator; October 18 simplified tag viewer and camera/tag relative pose support. Current tag viewer displays forward/right/down metres from drone centre. Camera mount assumptions are 45 degrees downward pitch and 0.02 m forward offset; tag side length 0.12 m and fixed camera intrinsics. These assumptions require verification against actual hardware and tags before trusting localization accuracy.

## System understanding

PX4 flight controller handles estimation and motor control. ESP32-S3 companion firmware communicates with it over UART MAVLink, bridges MAVLink to QGroundControl via Wi-Fi, streams camera images and supports navigation/setpoints. Laptop Python sends UDP commands and receives telemetry/video. Firmware main/ contains mission, MAVLink, navigation, odometry, ToF, VFH and tag detection tasks. Laptop scripts provide takeoff/land, waypoint missions, CSV trajectories and camera/tag viewers.

Local coordinates follow NED: x North, y East, z Down. Drone body axes are forward/right/down. Camera axes are image-right/image-down/out-of-lens. A relative body-frame tag vector is not yet an absolute local-frame target position: use synchronized vehicle attitude and position plus the camera extrinsics, and retain timestamps. Avoid confusing setup.yaml's map/start offsets with PX4 local coordinates.

Drone address is 192.168.1.(200 + ID); mission/QGC laptop address is 192.168.1.(100 + ID). README examples use ID 22, not evidence of our assigned ID. Existing cde1302-laptop conda environment is present. No dependency installation, flashing, flight commands or hardware tests performed.

Repository also contains older fleet/SAFMC exploration and relay routines. Their names do not establish that they satisfy this mini-project: run_full_mission.py describes a seven-minute multi-drone explore/relay mission. Current README says tags are detected/reported without affecting flight. Treat these routines as code to assess, not a ready project solution.

## Required outcome

Existing lab groups implement autonomous laptop-commanded takeoff, search unknown AprilTag positions within designated boundaries, identify tags and estimate their positions in UAV local coordinates, then return to the start and land. Manual teleoperation, manually selected goals and manually specified target positions are disallowed during the assessed autonomous mission unless the examiner authorizes them.

Record/display takeoff-command time, each detected ID and local position (plus relative pose and timestamp), and landing-command time. Accuracy is horizontal Euclidean position error. Mission timing ends after results are reported and the drone has landed in the start area. Final heading is not assessed.

Demo session: 30 minutes including setup/execution plus 5 minutes individual questions. Demonstration 70%, report 30%. Within demonstration: takeoff 5%, return/land 5%, detection 15%, localization 15%, completion time 15%, implementation quality 15%, individual questions 20%, extras 10%.

Report: at most 10 pages including cover/appendices, excluding source code; single-column, single-spaced, Times New Roman >=12 pt; GitHub source link; submission within 48 hours of demo. Must include PX4 .ulg trajectory reconstruction combined with tag IDs/locations and start/landing points, AI-assisted analysis and verification, and AI-use declaration. Python is the primary requested development language.

## Engineering gaps to evaluate next

- Choose/configure a coverage strategy and mission state machine for actual arena limits.
- Synchronize camera detections with PX4 pose; transform camera -> body -> local coordinates; consolidate repeated detections and reject outliers.
- Integrate event/detection recording, return-to-start and complete mission termination.
- Validate actual calibration, marker size, sensor quality, communication behavior and flight readiness.
- Build .ulg + detection reconstruction and report evidence.

## Starter behavior verified by source inspection — 22 October 2026

The inherited architecture notes overstate active obstacle navigation: `TOF_ENABLED=0`, `collision_avoid()` immediately returns false, and GOTO sends direct PX4 position targets without VFH steering or peer-obstacle injection. Blocked-bin feedback remains clear, which limits the older exploration director. MAVLink setpoints and navigation run at 20 Hz; telemetry runs at 10 Hz. The 0.10 m/s navigation constant does not constrain direct-position GOTO.

The firmware waits for START, enters Offboard, arms, targets local NED z=-0.5, then accepts laptop goals/trajectories until LAND. A takeoff timeout can still enter the flying phase while armed. LAND does not itself return to launch. Access-point loss cancels active navigation and attempts ordinary disarm after three seconds; it does not detect a dead laptop process, and cancellation is a no-op without active navigation.

Telemetry provides map XY and heading, but no altitude, roll/pitch, armed/mode status or pose timestamp. One protocol test expects absent PX4 status fields; this mismatch was found by inspection, without running tests. Onboard telemetry latches one claimed tag until reboot; live debug/video detections are separate. No software execution, build, installation or hardware validation was performed during this review.

## Legacy-firmware GUI — 2 November 2026

The local `gui2` branch was created directly from `kaiwei` (`605980a`). It carries
the styled `gui` dashboard and shared camera preview, with all extended field/value
tables removed. Map/reference paths/localized tags/heading and navigation/stuck
events remain; height is explicitly unavailable. Only laptop GUI/camera integration
and documentation change: firmware, protocol.py and flight_log.py match kaiwei.
Existing local sdkconfig edits, esp-idf/ and build-kaiwei/ are preserved.


### gui2 turn-in-place update — 7 November 2026

The logged snake turnaround uses a dedicated CMD_TURN instead of GOTO hops.
ESP32 holds captured NED x/y/z while ramping yaw at up to 15°/s and 10°/s².
Completion requires measured yaw within 5° for 1 s; timeout is 30 s. Existing
armed/Offboard/fresh-position checks and the laptop's 3 s stale threshold apply.
Snake CSVs, return glide and telemetry packet size remain unchanged. Requires
matching updated ESP32/laptop code; no PX4 firmware changes. No commit, flash or
flight. Software checks passed for the turn/trajectory paths and firmware build;
hardware position hold and rotation remain unverified. See esp32-starter/README.md.
