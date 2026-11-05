#include "decide.h"
#include <stddef.h>

/* Heading change of each action in whole degrees, positive to the left, indexed by Decide_Action.
 * Kept as a table instead of a switch so that adding an action without updating the table is caught
 * by the compile time check below. */
static const int16_t action_turn_degrees[DECIDE_ACTION_COUNT] = {
        [DECIDE_ACTION_FORWARD] = 0,
        [DECIDE_ACTION_TURN_RIGHT] = -90,
        [DECIDE_ACTION_TURN_LEFT] = 90,
        [DECIDE_ACTION_TURN_AROUND] = 180,
};

static const char* const action_names[DECIDE_ACTION_COUNT] = {
        [DECIDE_ACTION_FORWARD] = "forward",
        [DECIDE_ACTION_TURN_RIGHT] = "right",
        [DECIDE_ACTION_TURN_LEFT] = "left",
        [DECIDE_ACTION_TURN_AROUND] = "around",
};

/* Both tables must cover every action. A missing entry would silently read as zero. */
_Static_assert(sizeof(action_turn_degrees) / sizeof(action_turn_degrees[0]) == DECIDE_ACTION_COUNT,
               "action_turn_degrees must have one entry per action");
_Static_assert(sizeof(action_names) / sizeof(action_names[0]) == DECIDE_ACTION_COUNT,
               "action_names must have one entry per action");

Decide_Action Decide_NextAction(const Decide_Scan* scan) {
    /* A missing scan is a bug in the caller, not a situation on the track. Answer with the action
     * that cannot drive the car into anything: turn around on the spot. */
    if (scan == NULL) {
        return DECIDE_ACTION_TURN_AROUND;
    }

    /* The whole right hand rule. The order of the three checks is the priority, and comparing
     * against DECIDE_OPEN rather than against DECIDE_WALL is what keeps DECIDE_UNKNOWN from ever
     * counting as a free path. */
    if (scan->right == DECIDE_OPEN) {
        return DECIDE_ACTION_TURN_RIGHT;
    }
    if (scan->front == DECIDE_OPEN) {
        return DECIDE_ACTION_FORWARD;
    }
    if (scan->left == DECIDE_OPEN) {
        return DECIDE_ACTION_TURN_LEFT;
    }
    return DECIDE_ACTION_TURN_AROUND;
}

/* The enum has no negative values, so a compiler is free to make it unsigned. The cast to int keeps
 * the lower bound check meaningful for a value that was forced in from outside the enum. */
int16_t Decide_ActionTurnDegrees(Decide_Action action) {
    if (((int) action < 0) || ((int) action >= (int) DECIDE_ACTION_COUNT)) {
        return 0;
    }
    return action_turn_degrees[action];
}

const char* Decide_ActionName(Decide_Action action) {
    if (((int) action < 0) || ((int) action >= (int) DECIDE_ACTION_COUNT)) {
        return "invalid";
    }
    return action_names[action];
}
