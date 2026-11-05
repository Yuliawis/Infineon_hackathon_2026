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

///////////////////// STEPPING ////////////////////////////////////////////////

/* Longest single step forward. Short steps mean more scans and a slower run, long steps mean the car
 * reacts later to an opening. START. */
#define CONFIG_STEP_MAX_MM (200)

/* Free space that must be left in front of the car at the end of a step. START. */
#define CONFIG_FRONT_MARGIN_MM (120)

/* Short step after a turn, before the right side is checked again. Without it the car can see the
 * same opening a second time and spin in place. START. */
#define CONFIG_STEP_AFTER_TURN_MM (80)

/* How far the car has already driven past the real edge of the wall by the time it notices that the
 * wall ended. Subtracted from the distance driven up to the middle of an opening.
 *
 * Two parts. The median filter of three still holds two close readings from before the edge, so it
 * takes two fresh far readings to flip, which is 2 * 50 ms; at 200 mm/s that is 20 mm. On top of that
 * the beam is wide, so the edge of the wall is seen late. START, and the measured value will be
 * larger if the real speed is higher. */
#define CONFIG_OPENING_LAG_MM (25)

/* Small turn away from the wall when the right wall got dangerously close, before scanning again.
 * Without it the car would stop at the same spot on every step and never get anywhere. START. */
#define CONFIG_ESCAPE_TURN_DEG (15)

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
#define CONFIG_DISTANCE_PERIOD_MS (50)

///////////////////// MOTOR OUTPUT ////////////////////////////////////////////

/* Motor_Move() takes -4095..4095. Below the minimum the car only buzzes and does not move, so every
 * non-zero output is lifted to at least this value.
 * TODO MEASURE: raise the value until the car starts moving reliably from standstill. */
#define CONFIG_PWM_MIN (800)

/* Output used for a straight step and for a turn in place. Car_TurnBy() in car.c uses 1200 for
 * turning, which is a working starting point. START. */
#define CONFIG_PWM_STEP (1600)
#define CONFIG_PWM_TURN (1200)

/* How hard the car steers back onto its heading while stepping, in PWM units per degree of error.
 * Too small and the car drifts, too large and it weaves from side to side.
 * At the correction limit of 10 degrees this gives 400, a quarter of the step speed. START. */
#define CONFIG_HEADING_GAIN_PWM_PER_DEG (40)

/* A turn in place is finished when the heading is this close to the target. Taken from Car_TurnBy()
 * in car.c, which uses 1 degree and works. */
#define CONFIG_TURN_TOLERANCE_DEG (1)

/* A turn that takes longer than this is given up on, so a stuck wheel or a dead gyro cannot hold the
 * whole run. Taken from Car_TurnBy(). */
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

/* Tilt used for all scanning: level, straight at the walls. */
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
