# Robot Specification

## Gamepad Layouts
The tables below is a general outline of what the gamepad controls do for each mode.

# Standard

|  Button/Stick |             Action                   |
|---------------|:------------------------------------:|
| Left Axis     | Controls forward and backward motion |
| Right Axis    | Controls in-place rotation           |
| Left Bumper   | Controls note intake                 |
| Right Bumper  | Controls note shooting               |
| Left Trigger  | TBT |
| Right Trigger | TBT |
| A             | N/A |
| B             | N/A |
| X             | N/A |
| Y             | N/A |
| Start         | N/A |
| Back          | N/A |
| D-pad         | N/A |

## Motor Controllers
The values in this table are probably wrong. They need verified -MRF
| Label | CAN ID   | Controller | For?              |
|-------|:--------:|:----------:|-------------------|
| M1    |  0       | SparkMax   | Left drive no. 1  |
| M3/M4 |  1       | SparkMax   | Right drive no. 2 |
| M3    |  2       | SparkMax   | Shooter intake    |
| M4    |  3       | SparkMax   | Shooter           |
| M5    |  4       | SparkMax   | Rotate arm        |
| M6    |  5       | SparkMax   | Pickup Roller     |
| M7    |  - | - | - |
| M8    |  - | - | - |
