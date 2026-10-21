# CDE1302 drone mini-project context

Retrieved and reviewed 21 October 2026 (Singapore time).

## Sources and local copies

Course: https://canvas.nus.edu.sg/courses/96675

- CDE1302 Mini Project.pdf: Canvas file 10343339, requirements, 7 pages.
- CDE1302-Mini Project-Briefing.pdf: Canvas file 10398107, 18 slides; Canvas updated 19 October. Read slide text and inspected the arena illustration visually.
- Lab 1 setup; Lab 2, 3, 4 and 5 prelabs; Lab 4 and 5 reports: local PDF copies and extracted text in this folder.
- Risk assessment template: CDE1302 Risk Assessment Template.xls (downloaded, not completed).
- MajorQuest Drone Asm-1.zip: 56 CAD files including SolidWorks and STEP files; archive inspected, no mechanical design validation performed.
- Supporting control, PID and trajectory-generation lectures copied from existing course material cache.
- Canvas announcements and all four modules reviewed. Raw API metadata is retained locally only. Announcement query explicitly covered July 1–October 22 with pagination.

## Repository

Course Lab 5 explicitly points to https://github.com/linfengfrank/esp-everything-qgc-bridge .

Existing clone: original local clone, clean working tree, HEAD b8fe166 (25 September).
Fresh workspace clone: ../esp32-starter, HEAD c4caf91 (18 October), clean working tree. Original clone preserved.

Recent upstream changes: September 27 trajectory timing adjustment; October 3 printable marker generator; October 18 simplified tag viewer and camera/tag relative pose support. Current tag viewer displays forward/right/down metres from drone centre. Camera mount assumptions are 45 degrees downward pitch and 0.02 m forward offset; tag side length 0.12 m and fixed camera intrinsics. These assumptions require verification against actual hardware and tags before trusting localization accuracy.

Lab 4 also references linfengfrank/CDE1302_UAV for the simulation work; it is distinct from the ESP32 companion repository and was not cloned for this request.

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

## Recent course updates

- 20 October: current drone-frame CAD uploaded (announcement 506772).
- 15 October: preparation slot registration and Telegram invitation (announcement 504712). Recommended one preparation slot per team per week; extra sessions may lack GA support. Detailed arena layout promised for the following week.
- 13 October: mini-project requirements released; briefing on 15 October (announcement 503758).
- 25 September: Lab 5 introduces autonomous waypoint/trajectory flight and tag detection, updated ESP32 firmware; no Lab 6.

The preparation spreadsheet and Telegram links are available in Canvas announcement 504712. Their external contents were not fetched. No final arena dimensions, confirmed tag count, team/drone ID, booked preparation slot or team demonstration date were established. Slide 5 explicitly labels its drawing illustrative. The separate individual UAV trajectory assignment is due 6 November 2026 at 23:59 Singapore time; do not mistake this for the mini-project report deadline.

## Engineering gaps to evaluate next

- Choose/configure a coverage strategy and mission state machine for actual arena limits.
- Synchronize camera detections with PX4 pose; transform camera -> body -> local coordinates; consolidate repeated detections and reject outliers.
- Integrate event/detection recording, return-to-start and complete mission termination.
- Validate actual calibration, marker size, sensor quality, communication behavior and flight readiness.
- Build .ulg + detection reconstruction and report evidence.

This is a source/context review, not a claim that the firmware, scripts or mission have been tested on hardware.
