//
// Created by hs on 22. 9. 22.
//

#ifndef RAISIM_XBOXCONTROLLER_H
#define RAISIM_XBOXCONTROLLER_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"

class XboxController {
public:
    int joy_fd;
    int num_of_axis;
    int num_of_buttons;
    char name_of_joystick[80];
    bool joyAvailable;
    std::vector<int> joy_button;
    std::vector<int> joy_axis;

    js_event js;

    XboxController() {
        joy_fd=-1;
        num_of_axis=0;
        num_of_buttons=0;
        name_of_joystick[80];

        joyAvailable = joySetup();
    }

    bool joySetup();
    void joyRead();
    void joyButton();

private:
};


#endif //RAISIM_XBOXCONTROLLER_H
