#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Tuning parameters for the autonomous maze run, all in one place.
 *
 * Units are always part of the name: _MM millimeters, _MS milliseconds, _DEG whole degrees,
 * _MMPS millimeters per second. Everything is integer, there is no floating point in the control path.
 *
 * The values marked MEASURED are taken from the real car. The values marked START are the starting
 * points from the task description and still have to be confirmed by a test. */

///////////////////// MAZE GEOMETRY ///////////////////////////////////////////

/* Corridor width. START: the task says about 60 cm. */
#define CONFIG_CORRIDOR_WIDTH_MM (600)

/* Distance from the ultrasonic sensor to the turning axis of the car, along the car axis.
 * Positive when the sensor sits in front of the axis, which is the normal mounting.
 * It shifts every "stop at the front wall" distance, so a wrong value makes the car stop too early
 * or too late in every single turn.
 * TODO MEASURE on the real car with a ruler: from the front face of the HC-SR04 to the middle
 * between the front and the rear wheels. */
#define CONFIG_SENSOR_TO_AXIS_MM (60)

///////////////////// DIRECTION CLASSIFICATION ////////////////////////////////

/* A direction counts as open when the measured distance is above its threshold. Below it, and also
 * when there is no valid echo at all, the direction is not free. See decide.h for why no valid echo
 * is never treated as a free path.
 *
 * The three thresholds are separate on purpose: the sensor looks straight ahead when it measures the
 * front, but sideways at 90 degrees for left and right, where the beam hits the wall at a worse angle
 * and the readings are less reliable. They may well need different values.
 *
 * START: the task gives 50 cm for the right side. The other two start from the same value. */
#define CONFIG_RIGHT_OPEN_MM (500)
#define CONFIG_FRONT_OPEN_MM (500)
#define CONFIG_LEFT_OPEN_MM  (500)

///////////////////// WALL FOLLOWING //////////////////////////////////////////

/* Distance to the right wall the car tries to hold while stepping: the middle of the corridor.
 * START: half of the corridor width. */
#define CONFIG_RIGHT_TARGET_MM (300)

/* Below this the right wall is dangerously close: stop and straighten out instead of stepping on.
 * START. */
#define CONFIG_RIGHT_TOO_CLOSE_MM (150)

/* Largest heading correction the wall following may ask for, so that one bad reading cannot
 * throw the car across the corridor. START. */
#define CONFIG_HEADING_CORRECTION_MAX_DEG (10)

/* Wall following while stepping. The car holds the grid heading (a multiple of 90 degrees from the
 * start of the run) plus a small offset that steers it back to CONFIG_RIGHT_TARGET_MM from the right
 * wall. Without this nothing ever corrects a sideways drift: every turn on a slippery floor ends a
 * few degrees off, and those degrees used to be kept forever.
 *
 * One degree of offset per this many millimeters of distance error, and never more than the limit.
 * The limit is kept small on purpose: the side sensor only gets a clean echo near a right angle to
 * the wall, and the beam is about 14 degrees wide, so a few degrees of tilt are harmless. START. */
/* 1 = steer by the right wall while stepping, 0 = hold the grid heading only. Switch it off first
 * if the car drives into walls, to tell a wrong sign here from any other problem. */
#define CONFIG_WALL_FOLLOW_ENABLE (1)

#define CONFIG_WALL_MM_PER_DEG         (10)
#define CONFIG_WALL_CORRECTION_MAX_DEG (5)

/* The side sensor is only believed while the car is within this many degrees of the grid heading.
 * Turned further, the beam hits the wall at a slant: the echo either never comes back or comes back
 * from further along the wall, and reads as "much further away" or even as an opening. Readings
 * taken then neither steer the car nor end the step as "wall lost". Must be larger than
 * CONFIG_WALL_CORRECTION_MAX_DEG, or wall following would switch itself off. START. */
#define CONFIG_WALL_TRUST_MAX_DEG (8)

/* Before every scan, and before an opening is confirmed, the car turns in place back to the grid
 * heading when it is further off than this. A scan taken at a slant measures nonsense, and driving
 * the error out while moving forward is what made the car skid. */
#define CONFIG_ALIGN_TOLERANCE_DEG (5)

///////////////////// STEPPING ////////////////////////////////////////////////

/* Longest single step forward. Short steps mean more scans and a slower run, long steps mean the car
 * reacts later to an opening. START. */
#define CONFIG_STEP_MAX_MM (200)

/* Free space that must be left in front of the car at the end of a step. START. */
#define CONFIG_FRONT_MARGIN_MM (120)

/* How far the car keeps sliding forward after the motors are cut, on this slippery floor. Every
 * "stop in front of the wall" distance is reached this much earlier, so the car comes to rest where
 * it was meant to and not that much closer to the wall. Also taken off the length of a step that
 * ends in front of a wall.
 * TODO MEASURE: let the car approach a wall, mark where the "drive: end target reached" log line
 * comes, and measure how far it still slid. Too close to the wall means raise it. START. */
#define CONFIG_STOP_SLIDE_MM (50)

/* While approaching a front wall the car slows down over this last stretch before the stop point,
 * from CONFIG_PWM_STEP to CONFIG_PWM_MIN. A slow car barely slides when it stops, which makes the
 * stop both earlier and much more repeatable. START. */
#define CONFIG_APPROACH_SLOWDOWN_MM (150)

/* Short step after a left turn or a half turn, before the right side is checked again. START. */
#define CONFIG_STEP_AFTER_TURN_MM (80)

/* After a RIGHT turn the car stands in the middle of the old corridor, facing the opening. Its right
 * side then looks back down the old corridor, which is open, so a scan taken there would send it
 * right again, back where it came from. It first has to drive into the new corridor until the side
 * beam is past the corner, which is half a corridor less the sensor offset, plus this margin for the
 * width of the beam and the wall. START. */
#define CONFIG_ENTER_MARGIN_MM (80)

/* How far the car has already driven past the real edge of the wall by the time it notices that the
 * wall ended. Subtracted from the distance driven up to the middle of an opening.
 *
 * Two parts. The median filter of three still holds two close readings from before the edge, so it
 * takes two fresh far readings to flip, which is 2 * 50 ms; at 200 mm/s that is 20 mm. On top of that
 * the beam is wide, so the edge of the wall is seen late. START, and the measured value will be
 * larger if the real speed is higher. */
#define CONFIG_OPENING_LAG_MM (25)

/* How far to creep forward before turning right into an opening that was found by a standstill scan
 * instead of by the wall going away on the move. In that case nothing located the edge of the
 * opening, so there is no distance to compute: this is the one number that says where the turn is
 * taken, and it is bracketed by the car itself.
 *
 * Measured on the car: 180 mm (what the old constant produced) turned too late and the car hit the
 * far wall of the new corridor; 0 mm turned too early and it hit the near wall. 90 is the middle of
 * that bracket. Raise it if the car clips the near wall, lower it if it clips the far one. START. */
#define CONFIG_TURN_NUDGE_MM (90)

/* When the right wall got dangerously close, the car drives a short way forward aimed this many
 * degrees away from the wall, then scans again. The angle is relative to the grid heading and only
 * applies to that one short move: the next step goes back to the grid heading, so the escape never
 * leaves the car driving diagonally down the corridor. START. */
#define CONFIG_ESCAPE_TURN_DEG (15)
#define CONFIG_ESCAPE_MOVE_MM  (100)

/* Starting guess for the forward speed, which is how a length in millimeters becomes a time in
 * milliseconds. Without encoders this is the only bridge between the two, so it is a calibration
 * constant and not a wish about how fast to go; the actual speed comes from CONFIG_PWM_STEP.
 *
 * The car measures its own speed while driving up to a front wall and corrects this value, so it
 * only has to be right enough for the first approach of a run.
 * TODO MEASURE anyway: drive a known distance at CONFIG_PWM_STEP and divide, then put the result
 * here, so that even the first approach is close. START: about 0.2 m/s. */
#define CONFIG_SPEED_FORWARD_MMPS (200)

/* Range the measured speed is kept inside. A single bad pair of readings must never be able to put a
 * nonsense value into the geometry of every later movement. START. */
#define CONFIG_SPEED_MIN_MMPS (60)
#define CONFIG_SPEED_MAX_MMPS (800)

/* A speed measurement is only trusted when the car covered at least this much ground in at least
 * this much time. Shorter than that and the result is mostly measurement noise. START. */
#define CONFIG_SPEED_SAMPLE_MIN_MM (60)
#define CONFIG_SPEED_SAMPLE_MIN_MS (250u)

/* How often the ultrasonic driver measures. This mirrors MEASUREMENT_PERIOD_US in HCSR04.c and has
 * to be kept in step with it by hand. Used to work out how far the car travelled while the median
 * filter was catching up. */
#define CONFIG_DISTANCE_PERIOD_MS (60)

///////////////////// MOTOR OUTPUT ////////////////////////////////////////////

/* Motor_Move() takes -4095..4095. Below the minimum the car only buzzes and does not move, so every
 * non-zero output is lifted to at least this value.
 * TODO MEASURE: raise the value until the car starts moving reliably from standstill. */
#define CONFIG_PWM_MIN (800)

/* Output used for a straight step. START. */
#define CONFIG_PWM_STEP (1600)

/* Every forward movement ramps its output up from CONFIG_PWM_MIN to CONFIG_PWM_STEP over this time.
 * Jumping straight to full output on a slippery floor spins the wheels, and spinning wheels both
 * waste the distance estimate and kick the heading. START. */
#define CONFIG_DRIVE_RAMP_MS (200u)

/* How hard the car steers back onto its heading while stepping, in PWM units per degree of error.
 * Too small and the car drifts, too large and it weaves from side to side.
 * At the correction limit of 10 degrees this gives 400, a quarter of the step speed. START. */
#define CONFIG_HEADING_GAIN_PWM_PER_DEG (40)

/* The steering difference between the two sides is never more than this percentage of the forward
 * output. Without the cap a few degrees of error at the slow start of a step gave 400 against 1200,
 * which on a slippery floor is a spin, not a correction. START. */
#define CONFIG_HEADING_CORRECTION_MAX_PERCENT (20)

/* Turning in place on a slippery floor.
 *
 * MEASURED on the real car: these come from the reworked Car_TurnBy() in car.c on the simple_pass
 * branch, where they were tuned on slippery tiles. The car keeps sliding after the motors stop, so a
 * turn that just spins at full speed until the heading matches always overshoots and then hunts back
 * and forth. Three things fix that: slow down near the target, cut the motors early by the angle the
 * car is expected to slide, and accept a few degrees of error instead of chasing one. */

/* A turn is finished when the heading is this close to the target. Five degrees, not one: on a
 * slippery floor the car physically cannot settle inside one degree, and a tighter value only makes
 * it swing past the target over and over until the timeout. */
#define CONFIG_TURN_TOLERANCE_DEG (5)

/* Output far from the target, and near it. Below the minimum the wheels stall instead of turning.
 * MEASURED for turning in place, so it is used as is and NOT lifted to CONFIG_PWM_MIN, which is a
 * separate guess for driving forward. */
#define CONFIG_TURN_SPEED_MAX (1000)
#define CONFIG_TURN_SPEED_MIN (700)

/* The output ramps down from MAX to MIN over this last stretch before the target. */
#define CONFIG_TURN_SLOWDOWN_DEG (30)

/* Scale for the commanded angle, as a percentage, to cancel a scale error in the gyro: turns too far
 * means lower it, turns too short means raise it. Check by commanding a full 360 and measuring what
 * the car actually did. */
#define CONFIG_TURN_ANGLE_PERCENT (100)

/* How long the car is expected to keep rotating after the motors are cut. This is only the starting
 * value: the car corrects it after every first-pass turn from how far it actually slid, so a floor
 * that is more slippery than expected is learned instead of configured. */
#define CONFIG_TURN_COAST_MS_INIT (150)
#define CONFIG_TURN_COAST_MS_MAX (600)

/* Standing still after the motors are cut, so the slide finishes before the heading is judged. The
 * gyro keeps being read during it, otherwise the rotation that happens here would be lost. */
#define CONFIG_TURN_SETTLE_MS (300u)

/* Extra slow passes allowed to correct what is left after the first pass. Normally none are needed. */
#define CONFIG_TURN_MAX_CORRECTIONS (2u)

/* A turn that takes longer than this in total, corrections included, is given up on, so a stuck
 * wheel or a dead gyro cannot hold up the whole run. Taken from Car_TurnBy(). */
#define CONFIG_TURN_TIMEOUT_MS (5000u)

/* Pause between the two halves of a half turn, so the car stands still and the gyro settles before
 * the second 90 degrees. START. */
#define CONFIG_TURN_PAUSE_MS (200u)

/* Which side of the car each Motor_Move() argument belongs to: 1 = left, 0 = right.
 * car.c has the comment "check on the real car which motors are left/right", so this mapping is a
 * guess taken from Car_TurnBy(). If the car turns the wrong way or spins instead of driving straight,
 * these four lines are the first thing to fix.
 * TODO VERIFY on the real car. */
#define CONFIG_MOTOR_M1_IS_LEFT (1)
#define CONFIG_MOTOR_M2_IS_LEFT (1)
#define CONFIG_MOTOR_M3_IS_LEFT (0)
#define CONFIG_MOTOR_M4_IS_LEFT (0)

///////////////////// HEAD AND SENSING ////////////////////////////////////////

/* Head angles used for scanning. Pan is positive to the left (see head.h), so the right side is
 * negative. The head limits are -90..90, so these are the extremes. */
#define CONFIG_HEAD_PAN_RIGHT_DEG (-90)
#define CONFIG_HEAD_PAN_FRONT_DEG (0)
#define CONFIG_HEAD_PAN_LEFT_DEG  (90)

/* Tilt used for all scanning, the same for every direction. Positive is up, the head limits are
 * -20 to 10 (see head.h).
 *
 * MEASURED on the car: the sensor sits 90 mm above the floor and the walls are 135 mm tall, so the
 * top of a wall is 45 mm ABOVE the beam. The beam can never fly over a wall, which is the only thing
 * a downward tilt would have been good for. Level it is.
 *
 * Tilting down would actively hurt here. A tilted beam ends up on the floor, which answers like a
 * wall at that distance, and that distance is the height divided by the tangent of the angle. The
 * beam is about 14 degrees wide, so what matters is its lower edge, not its centre:
 *
 *   tilt    lower edge    floor answers at
 *    0         -7 deg         733 mm     beyond CONFIG_RIGHT_OPEN_MM, harmless
 *   -5        -12 deg         423 mm     inside the "wall" range already
 *  -15        -22 deg         223 mm     every direction reads as a wall, the car spins forever
 *
 * Grazing echoes off hard tiles are weak, which is why the lower edge at 0 degrees causes no
 * trouble. That stops being true as the angle steepens. */
#define CONFIG_HEAD_TILT_DEG (0)

/* Time the servo needs to travel to a new angle. The servos do not report their position, so this is
 * pure waiting. Measuring during the movement gives the distance to whatever the head passed on the
 * way, which is why no measurement is trusted before this time is over.
 * TODO MEASURE: the longest travel is 180 degrees, from one extreme to the other.
 * START: the task says 100..200 ms. */
#define CONFIG_HEAD_SETTLE_MS (150)

/* Measurements that must be collected after the head has settled, before the filtered value is used.
 * The driver measures every 50 ms (HCSR04.c) and the median filter window is 3 (distance.h), so 3
 * measurements are needed to flush the readings taken at the old head angle: about 150 ms. */
#define CONFIG_SCAN_SAMPLES (3)

/* Hard limit for one direction scan, so a dead sensor cannot stop the whole run.
 * Head travel plus the samples is about 300 ms, so this is generous. */
#define CONFIG_SCAN_TIMEOUT_MS (1000u)

/* Lazy scanning: stop as soon as one direction is confirmed open.
 *
 * The scan goes right, then front, then left, which is exactly the priority order of the right hand
 * rule. Once a direction is open the action is already decided and the remaining directions cannot
 * change it, so measuring them is wasted time. The directions that were skipped stay unknown, and an
 * unknown direction counts as a wall, which is the lowest priority: it can never affect the outcome.
 *
 * One scan costs about 300 ms per direction, so this turns the common corridor case from 900 ms into
 * 600 ms, and an opening on the right into 300 ms.
 *
 * Set to 0 to always measure all three directions. Slower, but the log then shows the full picture
 * at every decision, which is useful while debugging. */
#define CONFIG_SCAN_LAZY (1)

/* How many times one direction is measured again when it gives no valid echo, before it is finally
 * reported as unknown. Each retry costs another set of samples, about 150 ms.
 * The task also suggests looking slightly off axis on a retry. That is an improvement for later:
 * first the log has to show how often this happens at all. START. */
#define CONFIG_SCAN_RETRIES (2u)

/* 1 = silence on the RIGHT, after every retry, counts as an opening instead of as unknown.
 *
 * The task says a missing echo is never an opening, and that holds for the front and the left: the
 * beam often meets those walls at a steep angle, where an echo scatters away and silence really says
 * nothing. The right side is different. Every scan is taken after alignToGrid(), so the right beam
 * is square to the right wall, and a flat wall 200 to 300 mm away at a right angle is the easiest
 * target ultrasound has. Silence from there means there is no wall within range.
 *
 * Measured on the car: an opening leading into a long corridor swallows the beam and returns nothing,
 * so with silence counted as a wall the car drove past every such opening, which is most of them.
 * Set to 0 to go back to the conservative reading. START. */
#define CONFIG_RIGHT_SILENCE_IS_OPEN (1)

///////////////////// SAFETY //////////////////////////////////////////////////

/* Longest time for one complete run before the fallback behaviour starts. START. */
#define CONFIG_RUN_TIMEOUT_MS (300000u)

/* Pause between power up and the start of the run, so there is time to put the car down and get the
 * USB cable out of the way. The car does not move during it. START. */
#define CONFIG_RUN_START_DELAY_MS (3000u)

/* A step that does not change the front distance at all means the car is stuck against something.
 * START. */
#define CONFIG_STUCK_TIMEOUT_MS (1500u)
#define CONFIG_STUCK_PROGRESS_MM (20)

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
