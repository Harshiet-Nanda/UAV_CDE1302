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

CDE1302-specific PR-Agent configuration is installed for Gemini descriptions, reviews, suggestions and automatic changelog/version entries. See [setup, behavior and validation](docs/PR_AUTOMATION.md). The workflow is enabled on main; live API validation is in progress.
