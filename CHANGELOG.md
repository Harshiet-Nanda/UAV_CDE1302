## [0.2.0] - 2026-10-22 [*](https://github.com/Harshiet-Nanda/CDE1302/pull/7)
Added an unprocessed camera AprilTag stream script to inspect raw pose vectors and documented verified starter firmware behavior and telemetry limits.

### Added / Changed
- feat(apriltag): add `tag_stream_unprocessed.py` to stream camera-frame AprilTag translation and Rodrigues rotation vectors without body transforms or smoothing.

### Documentation
- docs(project): document verified starter firmware characteristics, navigation constants, and telemetry limits in `PROJECT_CONTEXT.md`.

## [0.1.0] - 2026-10-22
Established the CDE1302 course repository with reference materials, an attributed ESP32 starter snapshot, project context and PR automation configuration.

### Added / Changed
- feat(esp32): Imported the course ESP32-S3 firmware and laptop tools from upstream commit `c4caf91`; no course mission or flight validation is claimed.

### Documentation
- docs(materials): Collected lecture/lab guides, mini-project requirements/briefing, the individual assignment brief, risk template and drone-frame CAD archive.
- docs(project): Added repository overview, project context and maintained agent handoff guidance.

### Testing
- test(ci): Added offline validation coverage for automation event authorization and configuration. GitHub/API integration and hardware validation remain pending.
