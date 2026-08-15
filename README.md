# Sweet Potato Robot Intelligent Medical Challenge
# Full Autonomous Medical Vehicle Solution Documentation – Team Mengde
The 21st National University Intelligent Vehicle Competition | Undergraduate Group
---
## Documentation Summary
| Item | Content |
|---|---|
| Participating Institution | Nantong Institute of Technology |
| Team Name | Mengde Team |
| Competition Division | East China Division |
| Operation Mode | Fully autonomous onboard operation based on RDK X5 |
| Core Technical Lines | Heading Integration for Lap Counting, Hierarchical Task Scheduling, Visual Obstacle Avoidance, Dual-channel Image-to-Text Publishing |

## Part 1 Team Information
Team Leader: Zhu Shiyu, Contact Number: 16621340390
Team Members: Zhu Shiyu, Geng Jiayi, Li Yilin, Liu Qi
Instructors: Wang Qi, Wang Junlong
The team participates in the undergraduate group of the Sweet Potato Robot Intelligent Medical Challenge.

After the vehicle departs, all perception, computation, path planning and vehicle control logic run locally on the onboard RDK X5. No task commands or motion instructions are sent to the vehicle by team members via wireless network, remote controllers, keyboards or other external devices.

## Part 2 Hardware Platform
### 2.1 Core Computing Unit
The system takes the Sweet Potato Robot RDK X5 development board as its core computing hardware.
The RDK X5 is equipped with an octa-core Arm Cortex-A55 processor, a 10 TOPS INT8 BPU, and abundant interfaces including camera, USB, HDMI, Ethernet, CAN and storage.
The official Ubuntu and ROS 2 environments manage all onboard nodes. The BPU undertakes real-time inference for yellow center line detection and competition target recognition, while the CPU handles heading accumulation, phase scheduling, QR code parsing, scene text generation and safety control.

### 2.2 Overall Vehicle Hardware Composition
| Subsystem | Hardware Components | Functions |
|---|---|---|
| Perception System | RDK-compatible forward-facing camera | Continuously detect yellow lanes, QR codes, obstacles, human-shaped signboards, channel entrances and P parking zones |
| Computing System | RDK X5, TF card, heat dissipation modules | Model inference, task scheduling, data communication and program storage |
| Execution System | ROS 2-compatible vehicle chassis | Execute linear velocity, angular velocity, obstacle avoidance and parking commands |
| Human-Machine Interaction System | HDMI display, onboard speaker | Separately display QR code data and scene text, and broadcast voice prompts |
| Power Supply System | Lithium battery pack, voltage regulator module, protected wiring harness | Stable power supply for computing, perception, interaction and driving modules |

### 2.3 Physical Connection Logic
| Forward Camera | → | RDK Image Stream | → | Center line perception / scene safety detection / QR code decoding |
|---|---|---|---|---|
| Chassis Odometry | → | Heading Variation Integration | → | Hierarchical Task Scheduler |
| Task Scheduler | → | Motion Commands / Image Capture Requests | → | Chassis Controller / Scene Interpreter |
| Scene Interpreter | → | On-screen Text & Voice Text | → | Onboard Display Terminal / Speaker |

The camera connects to the RDK camera interface, and the display connects via HDMI. The chassis communicates with the main controller through ROS 2-compatible drivers. Power cables and signal wires are bundled and fixed separately. The RDK board is equipped with active heat dissipation to avoid calculation interference caused by motor electromagnetic noise and vehicle vibration.

## Part 3 Autonomous System Design
### 3.1 Hierarchical Task Chain
The main controller divides the entire competition process into eight sequential execution stages. Each stage only responds to its corresponding valid trigger events:
```text
Wait for QR Code Marker
   ↓
Select Driving Direction ─→ Pass Entrance ─→ Circle Zone C ─→ Exit Loop Channel
                                      ↓
             Timeout Parking ← Return to Zone P ← Complete Image-to-Text Generation
                                      ↓
                                   Locked Parking State
Raw numeric data parsed from QR codes is displayed on the onboard terminal. Valid numbers are converted into clockwise or counter-clockwise driving directions. Direction signals control both branch lane selection and the sign of heading integration value, ensuring the actual circling direction matches task requirements.
After the vehicle enters the channel entrance, cumulative calculation of odometry heading variation starts. The system processes cross ±π angle conversion for adjacent heading values and filters abnormal angle jumps; only rotation values consistent with the QR code direction are accumulated.
The system switches to the exit stage only when three conditions are met: the integrated heading variation completes a full circle, the channel entrance is detected again, and human figure text description generation is finished. This lap-judgment method relies purely on vehicle posture changes, independent of fixed time thresholds.
3.2 Perception-Control Closed-Loop
The yellow center line detection model outputs lane offset values. The task scheduler generates steering correction commands based on horizontal image deviation.
The scene safety model identifies obstacles, channel gates, human-shaped signboards and parking markers. Once obstacle avoidance is triggered, safety control commands override normal line-following instructions.
All control commands carry a valid time window; the vehicle automatically outputs zero velocity if data transmission is interrupted.
text
Yellow Lane Offset ─→ Steering Adjustment ─→ Chassis Motion Output
       ↑                         ↓
       └──── Camera Frame Feedback ────────┘

Obstacle Detected ─→ Safety Control Takeover ─→ Obstacle Circumvention ─→ Restore Lane-Following Closed-Loop
3.3 Human Figure Recognition & Information Terminal
Upon the first detection of a human-shaped signboard, the main controller sends an image capture request once. The scene interpreter receives the captured human figure image and generates a concise text description. The generated text is simultaneously sent to the speech synthesis node and the dark-themed onboard display terminal.
The upper area of the screen permanently shows QR code numbers and circling directions, while the lower area presents scene description text. Independent prompt texts are used for processing and recognition failure states, which will not replace valid identification results.
3.4 Three Core Task Execution Flow
Task 1: Receive Driving Instructions
The vehicle departs Zone P autonomously toward the QR code posting point, scans and decodes the QR code, displays raw numeric data and clockwise/counter-clockwise direction on the screen, then executes lane branch actions corresponding to the parsed direction.
Task 2: Cruise in Zone C
The vehicle enters Zone C through the designated channel and drives only along the yellow circular track. The main controller accumulates heading angles according to the QR code direction, safely bypasses detected obstacles, generates image-to-text content with screen and voice feedback after detecting human signboards, and exits Zone C through the specified channel in Area B once full circling requirements are satisfied.
Task 3: Return and Park
After exiting Area B, the system switches to Zone P homing stage. The parking lock state is triggered when the vehicle enters the starting position range or receives the P-zone parking signal, continuously outputting zero velocity to ensure at least two-thirds of the vehicle body stays within Zone P and remains stationary.
Competition timing relies on a monotonic system clock. When total runtime reaches 180 seconds, the system jumps directly to the termination stage from any running state, disables yellow lane tracking and maintains full stop.
Part 4 Software Resource Sources
表格
Resource Category	Source & Application Purpose
RDK System & BSP	Official Sweet Potato Robot image and hardware documentation, for board-level driver and peripheral management
TROS & ROS 2 Function Packages	Official development materials of Sweet Potato Robot, for image transmission, message communication and node scheduling
BPU Deployment Tools	Official model conversion, quantization and inference toolkit, for visual model deployment on the edge board
Basic Chassis Drivers	RDK-compatible ROS 2 chassis interface, for odometry reading and velocity regulation
Competition Application Program	Self-developed modules including hierarchical task scheduling, heading lap counting, obstacle avoidance takeover, image capture, scene interpretation and display logic
Part 5 Reliability Optimization Design
A. Power-On Self-Check
The startup script sequentially loads chassis, camera, image conversion, visual inference, QR code decoding, main control, scene interpretation, display and voice nodes.
The system automatically checks image streams, model inference outputs, odometry data, display terminal connection and chassis communication. The vehicle remains stationary if critical input channels are not established.
B. Track Environment Adaptation
The yellow center line algorithm adopts continuous frame feedback. Camera exposure parameters and driving speed are calibrated to adapt to strong light, shadow and reflective track surfaces.
Multi-frame result filtering is applied to channel entrance and human target detection to prevent false single-frame detection from triggering incorrect stage jumps.
C. Safety Degradation Mechanism
Obstacle detection signals have highest priority to take over vehicle control; normal line-following resumes after obstacle avoidance completes.
Unreasonable heading angle jumps are filtered during integration. QR code signals are only accepted in the QR waiting stage; P-zone parking trigger only activates in the homing stage.
Zero velocity locking is activated under multiple abnormal conditions: motion data timeout, program exit, task completion and 180-second competition timeout.
D. Backup Solutions
A spare TF card stores the complete system and project mirror. Backup cameras, power cables and structural fixing parts are carried on the vehicle.
Pre-competition full-process tests cover clockwise/counter-clockwise circling, obstacle bypass, human figure image-to-text generation, designated channel entry & exit, Zone P parking and automatic stop at 180 seconds.