#include <Zumo32U4Motors.h>
#include <Zumo32U4Encoders.h>

#include "button.h"       //include your button class from last week
#include "event_timer.h"  //include your shiny, new event timer class

#include "segments.h"
#include "params.h"

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

DebouncedButton buttonA(14, HIGH);
EventTimer timer;

// Define the robot direction of movement
typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    DRIVING_FORWARD,
    DRIVING_BACKWARD,
    PERFORM_SPIN

} ROBOT_MOVEMENT;

bool robot_move(const ROBOT_MOVEMENT move_type)
{
    if (move_type == STOP) {
        Serial.print("[ROBOT] HALT! \n");
        
        motor_command.angular.x = 0.0;
        motor_command.linear.z = 0.0;
    }

    else if (move_type == DRIVING_FORWARD) {
        Serial.print("[ROBOT] ONWARD! \n");
        motor_command.angular.x = 0.5;
        motor_command.linear.z = 0.0;
    }

    else if (move_type == DRIVING_BACKWARD) {
        Serial.print("[ROBOT] Backing up! \n");
        motor_command.linear.x = -0.5;
        motor_command.angular.z = 0.0;
    }

    else if (move_type == PERFORM_SPIN) {
        Serial.print("[ROBOT] I'm spinning! \n");
        motor_command.linear.x = 0.0;
        motor_command.angular.z = 1.0;
    }

    else {
        Serial.print("[ROBOT_MOVE] Move type wrong! \n");
        return false;
    }
