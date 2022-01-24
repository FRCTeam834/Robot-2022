# Team 834 Robot Code (2022)

Repository for 2022 Robot Code written in Java using WPILib.

## Hardware

This robot uses REV NEO Motors, Vex 775 Pro Motors, and Kauai Lab's NavX.

## Vendor Dependency Links

| Vendor Name              | Link                                                                                                           |
|--------------------------|----------------------------------------------------------------------------------------------------------------|
| REV Robotics (REVLib)    | <https://software-metadata.revrobotics.com/REVLib.json>                                                          |
| CTRE (Phoenix)           | <http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json>                          |
| PhotonVision (PhotonLib) | <https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json> |
| NavX (AHRS)              | <https://www.kauailabs.com/dist/frc/2022/navx_frc.json>                                                          |

## CAN ID Table

| CAN ID | Motor |
|--------|-------|
| 1      | Front Left Steer Motor Spark Max    |
| 2      | Front Right Steer Motor Spark Max   |
| 3      | Back Left Steer Motor Spark Max     |
| 4      | Back Right Steer Motor Spark Max    |
| 5      | Front Left Drive Motor Spark Max    |
| 6      | Front Right Drive Motor Spark Max   |
| 7      | Back Left Drive Motor Spark Max     |
| 8      | Back Right Drive Motor Spark Max    |
| 9      | Front Left CANCoder                 |
| 10     | Front Right CANCoder                |
| 11     | Back Left CANCoder                  |
| 12     | Back Right CANCoder                 |
| 13     | Power Distribution Hub (PDH)        |

## Terminology note

Code uses the word "angle" to describe the angle of the pivot of the module in degrees. It also uses "velocity" to describe the speed of the wheel in m/s (meters per second). Speed, RPM, or radians should not be used.
