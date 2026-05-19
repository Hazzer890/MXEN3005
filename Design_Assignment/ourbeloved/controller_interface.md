# Controller Interface (`ourbeloved`)

`/joy` (`sensor_msgs/Joy`) layout for the PS4 controller as consumed by
`robot_controller_node` and `fire_controller_node`.

## axes (1.0 to -1.0)

| Index | Stick / Trigger |
|---|---|
| [0] | Left Stick X |
| [1] | Left Stick Y |
| [2] | L2 |
| [3] | Right Stick X |
| [4] | Right Stick Y |
| [5] | R2 — fire trigger (`< 0` => fire) |
| [6] | D-Pad X |
| [7] | D-Pad Y |

## buttons (0 unpressed, 1 pressed)

| Index | Button | Action |
|---|---|---|
| [0]  | X       | Attack Mode |
| [1]  | Circle  | - |
| [2]  | Triangle| - |
| [3]  | Square  | - |
| [4]  | L1      | - |
| [5]  | R1      | - |
| [6]  | -       | - |
| [7]  | -       | - |
| [8]  | Share   | Cartesian Mode |
| [9]  | Options | Joint Mode |
| [10] | Home    | Homing |

Deadzone: `0.1` on every mapped axis.
