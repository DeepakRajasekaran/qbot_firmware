#include "action.h"

Action::Action() {
    currentMode = ACTION_IDLE;
}

void Action::setMode(ActionMode mode) {
    currentMode = mode;
}

ActionMode Action::getMode() {
    return currentMode;
}

void Action::update() {
    switch (currentMode) {
        case ACTION_1:
            action_1();
            break;
        case ACTION_2:
            action_2();
            break;
        case ACTION_3:
            action_3();
            break;
        default:
            // idle → do nothing
            break;
    }
}

void Action::action_1() {
    // dummy → to be implemented later
}

void Action::action_2() {
    // dummy → to be implemented later
}

void Action::action_3() {
    // dummy → to be implemented later
}
