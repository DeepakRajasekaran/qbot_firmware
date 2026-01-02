#ifndef ACTION_H
#define ACTION_H

#include <Arduino.h>

enum ActionMode {
    ACTION_IDLE,
    ACTION_1,
    ACTION_2,
    ACTION_3
};

class Action {
public:
    Action();
    void update();

    void setMode(ActionMode mode);
    ActionMode getMode();

private:
    ActionMode currentMode;

    void action_1();
    void action_2();
    void action_3();
};

#endif
