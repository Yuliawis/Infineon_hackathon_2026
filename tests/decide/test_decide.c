/* Host test for the decide module. Builds and runs on the PC, no board needed:
 *
 *     cd tests/decide && make
 *
 * The decide module is pure logic, so it can be checked exhaustively here instead of on the car. */

#include "decide.h"
#include <stdio.h>
#include <stdlib.h>

static int failures = 0;
static int checks = 0;

static const Decide_SideState all_states[] = {DECIDE_UNKNOWN, DECIDE_WALL, DECIDE_OPEN};
#define STATE_COUNT ((int) (sizeof(all_states) / sizeof(all_states[0])))

static const char* stateName(Decide_SideState state) {
    switch (state) {
        case DECIDE_UNKNOWN: return "unknown";
        case DECIDE_WALL: return "wall";
        case DECIDE_OPEN: return "open";
        default: return "invalid";
    }
}

static void checkAction(Decide_SideState left, Decide_SideState front, Decide_SideState right,
                        Decide_Action expected, const char* situation) {
    const Decide_Scan scan = {.left = left, .front = front, .right = right};
    const Decide_Action actual = Decide_NextAction(&scan);

    ++checks;
    if (actual != expected) {
        ++failures;
        printf("FAIL %-22s L=%-7s F=%-7s R=%-7s -> %s, expected %s\n", situation, stateName(left),
               stateName(front), stateName(right), Decide_ActionName(actual), Decide_ActionName(expected));
    }
}

static void expect(int condition, const char* message) {
    ++checks;
    if (!condition) {
        ++failures;
        printf("FAIL %s\n", message);
    }
}

/* The eight situations from the task description, written out literally so that a change in the rule
 * shows up as a failing row and not as a surprise on the track. */
static void testTruthTable(void) {
    checkAction(DECIDE_OPEN, DECIDE_OPEN, DECIDE_OPEN, DECIDE_ACTION_TURN_RIGHT, "crossing");
    checkAction(DECIDE_WALL, DECIDE_OPEN, DECIDE_OPEN, DECIDE_ACTION_TURN_RIGHT, "branch right");
    checkAction(DECIDE_OPEN, DECIDE_WALL, DECIDE_OPEN, DECIDE_ACTION_TURN_RIGHT, "T crossing");
    checkAction(DECIDE_WALL, DECIDE_WALL, DECIDE_OPEN, DECIDE_ACTION_TURN_RIGHT, "corner right");
    checkAction(DECIDE_OPEN, DECIDE_OPEN, DECIDE_WALL, DECIDE_ACTION_FORWARD, "branch left");
    checkAction(DECIDE_WALL, DECIDE_OPEN, DECIDE_WALL, DECIDE_ACTION_FORWARD, "corridor");
    checkAction(DECIDE_OPEN, DECIDE_WALL, DECIDE_WALL, DECIDE_ACTION_TURN_LEFT, "corner left");
    checkAction(DECIDE_WALL, DECIDE_WALL, DECIDE_WALL, DECIDE_ACTION_TURN_AROUND, "dead end");
}

/* An unknown direction must behave exactly like a wall: replacing every unknown with a wall may not
 * change the action. Checked on all 27 combinations, so it holds by construction and not by luck. */
static void testUnknownBehavesAsWall(void) {
    for (int l = 0; l < STATE_COUNT; ++l) {
        for (int f = 0; f < STATE_COUNT; ++f) {
            for (int r = 0; r < STATE_COUNT; ++r) {
                const Decide_Scan as_measured = {
                        .left = all_states[l], .front = all_states[f], .right = all_states[r]};
                const Decide_Scan as_walls = {
                        .left = (all_states[l] == DECIDE_UNKNOWN) ? DECIDE_WALL : all_states[l],
                        .front = (all_states[f] == DECIDE_UNKNOWN) ? DECIDE_WALL : all_states[f],
                        .right = (all_states[r] == DECIDE_UNKNOWN) ? DECIDE_WALL : all_states[r]};

                ++checks;
                if (Decide_NextAction(&as_measured) != Decide_NextAction(&as_walls)) {
                    ++failures;
                    printf("FAIL unknown is not a wall: L=%s F=%s R=%s\n", stateName(all_states[l]),
                           stateName(all_states[f]), stateName(all_states[r]));
                }
            }
        }
    }
}

/* The car may only be sent into a direction that is confirmed open. This is the safety property of
 * the whole rule, so it is checked on all 27 combinations as well. */
static void testNeverMovesIntoUnconfirmedSpace(void) {
    for (int l = 0; l < STATE_COUNT; ++l) {
        for (int f = 0; f < STATE_COUNT; ++f) {
            for (int r = 0; r < STATE_COUNT; ++r) {
                const Decide_Scan scan = {
                        .left = all_states[l], .front = all_states[f], .right = all_states[r]};
                const Decide_Action action = Decide_NextAction(&scan);
                int ok = 1;

                switch (action) {
                    case DECIDE_ACTION_FORWARD: ok = (scan.front == DECIDE_OPEN); break;
                    case DECIDE_ACTION_TURN_RIGHT: ok = (scan.right == DECIDE_OPEN); break;
                    case DECIDE_ACTION_TURN_LEFT: ok = (scan.left == DECIDE_OPEN); break;
                    case DECIDE_ACTION_TURN_AROUND:
                        /* Only allowed when nothing is confirmed open */
                        ok = (scan.left != DECIDE_OPEN) && (scan.front != DECIDE_OPEN) &&
                             (scan.right != DECIDE_OPEN);
                        break;
                    default: ok = 0; break;
                }

                ++checks;
                if (!ok) {
                    ++failures;
                    printf("FAIL unsafe action %s for L=%s F=%s R=%s\n", Decide_ActionName(action),
                           stateName(all_states[l]), stateName(all_states[f]), stateName(all_states[r]));
                }
            }
        }
    }
}

/* Right has priority over front, front over left, and left over turning around. */
static void testPriorityOrder(void) {
    const Decide_Scan right_and_front = {.left = DECIDE_WALL, .front = DECIDE_OPEN, .right = DECIDE_OPEN};
    const Decide_Scan front_and_left = {.left = DECIDE_OPEN, .front = DECIDE_OPEN, .right = DECIDE_WALL};

    expect(Decide_NextAction(&right_and_front) == DECIDE_ACTION_TURN_RIGHT, "right must win over front");
    expect(Decide_NextAction(&front_and_left) == DECIDE_ACTION_FORWARD, "front must win over left");
}

/* The heading changes feed the loop detection in the safety module, so the signs matter:
 * positive is left, matching Motion_GetYaw(). */
static void testTurnDegrees(void) {
    expect(Decide_ActionTurnDegrees(DECIDE_ACTION_FORWARD) == 0, "forward must not change the heading");
    expect(Decide_ActionTurnDegrees(DECIDE_ACTION_TURN_LEFT) == 90, "left must be +90");
    expect(Decide_ActionTurnDegrees(DECIDE_ACTION_TURN_RIGHT) == -90, "right must be -90");
    expect(Decide_ActionTurnDegrees(DECIDE_ACTION_TURN_AROUND) == 180, "turn around must be +180");

    /* Four rights add up to a full turn, which is what going around an island looks like */
    expect(4 * Decide_ActionTurnDegrees(DECIDE_ACTION_TURN_RIGHT) == -360, "four rights must be -360");
}

/* Names go into the UART log and the BLE telemetry, where a NULL would crash the print. */
static void testNamesAreSafe(void) {
    for (int action = 0; action < (int) DECIDE_ACTION_COUNT; ++action) {
        expect(Decide_ActionName((Decide_Action) action) != NULL, "every action needs a name");
    }
    expect(Decide_ActionName((Decide_Action) 99) != NULL, "an out of range action still needs a name");
    expect(Decide_ActionTurnDegrees((Decide_Action) 99) == 0, "an out of range action must turn by 0");
}

/* A missing scan is a caller bug. The answer must still be safe: never forward. */
static void testNullScan(void) {
    expect(Decide_NextAction(NULL) == DECIDE_ACTION_TURN_AROUND, "a NULL scan must not drive forward");
}

int main(void) {
    testTruthTable();
    testUnknownBehavesAsWall();
    testNeverMovesIntoUnconfirmedSpace();
    testPriorityOrder();
    testTurnDegrees();
    testNamesAreSafe();
    testNullScan();

    if (failures == 0) {
        printf("decide: all %d checks passed\n", checks);
        return EXIT_SUCCESS;
    }

    printf("decide: %d of %d checks FAILED\n", failures, checks);
    return EXIT_FAILURE;
}
