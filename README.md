# CDE1302 — Introduction to UAVs

Course reference materials and a starting point for the autonomous UAV exploration mini-project, AY 2026/27 Semester 1.

## Contents

- **materials/** — lecture slides, lab guides and blank report templates, mini-project requirements and briefing, risk assessment template, and drone-frame CAD archive.
- **esp32-starter/** — snapshot of the course ESP32-S3 companion firmware and laptop Python tools.
- **docs/PROJECT_CONTEXT.md** — project requirements, architecture, recent updates and open questions.

The mini-project is an autonomous takeoff, AprilTag search/localization, return and landing mission. It also requires timestamped mission records and analysis combining PX4 flight logs with tag detections.

## Starter code provenance

Source: https://github.com/linfengfrank/esp-everything-qgc-bridge

Snapshot: `c4caf91` (18 October 2026). Imported as ordinary files so future course work can be committed directly here. Upstream authorship and license notices are retained. See [the starter README](esp32-starter/README.md) for setup and the supplied lab examples. Example drone IDs and arena configuration are not our confirmed team settings.

This repository has not been flight-tested. The included fleet/relay examples require assessment before adapting them to the course mission.

## Course sources

Canvas: https://canvas.nus.edu.sg/courses/96675

Materials collected on 21 October 2026. PDFs and CAD files remain the property of their respective authors; no new license is granted over course materials. Keep this repository private unless redistribution is authorized.

Raw Canvas API responses, signed download links, credentials and personal submission/grade data are excluded.

## Pull-request automation

CDE1302-specific PR-Agent configuration is installed for Gemini descriptions, reviews, suggestions and automatic changelog/version entries. See [setup, behavior and validation](docs/PR_AUTOMATION.md). The workflow is enabled on main; final AI/changelog validation is pending recovery from Gemini 503 high-demand errors.

## Legacy-firmware GUI

The local `gui2` branch starts from `kaiwei` and retains the styled flight map,
AprilTag camera and event log, with unsupported telemetry tables removed and
height marked unavailable. That dashboard change retained the legacy telemetry layout.
See the [dashboard notes](esp32-starter/README.md#legacy-firmware-dashboard-gui2-branch).


## Gradual snake turnaround

Local `harshiet` (based on `kaiwei`) now implements a position-holding gradual yaw turn, correction
and settled-heading gate before the reverse snake. It requires updated ESP32
firmware. See [turnaround setup, limits and validation](esp32-starter/README.md#gradual-turnaround-before-the-reverse-snake).
Built and flashed for drone 7 on 5 November 2026; boot confirmed. Wi-Fi connection and supervised flight validation remain pending.


## Launch-position checks

The local firmware also saves pre-arm X/Y, corrects drift after takeoff, and
verifies/corrects that same X/Y before normal landing with the updated logged
snake/waypoint clients. Initial tolerance is 10 cm; immediate abort landing stays
available. See [launch-position recovery](esp32-starter/README.md#verify-launch-xy-after-takeoff-and-before-normal-landing)
for limits and failure handling. Firmware flashed on 5 November 2026;
supervised flight validation remains pending.
