/***************************************************************************

    file                 : driver.cpp
    created              : fre 23 feb 2018 11:08:47 CET
    copyright            : (C) 2002 Zijian

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "driver.h"

const float Driver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI;  /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;           /* [s] */
const float OFFSET_FACTOR = 1;
const float STEERANGLE_FACTOR = 1.2;

Driver::Driver(int index)
{
    INDEX = index;
}

/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle,
                       void **carParmHandle, tSituation *s)
{
    track = t;
    *carParmHandle = NULL;
}

/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s)
{
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    stuck = 0;
}

/* Drive during race. */
void Driver::drive(tCarElt* car, tSituation *s)
{
    update(car, s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck(car)) {
        car->ctrl.steer = -angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } else {
        float steerangle = STEERANGLE_FACTOR * angle - OFFSET_FACTOR * (car->_trkPos.toMiddle/car->_trkPos.seg->width - 0.25);

        car->ctrl.steer = steerangle / car->_steerLock;
        car->ctrl.gear = rand()%3+1; // random gear between 1 and 3
        float throttle_control = (float)(rand()%10)/10-0.2;
        if (throttle_control > 0) {
          car->ctrl.accelCmd = throttle_control;
          car->ctrl.brakeCmd = 0.0;
        }
        else {
          car->ctrl.accelCmd = 0.0;
          car->ctrl.brakeCmd = throttle_control;
        }
        //printf("control throttle is %f",throttle_control);

    }
}

/* Set pitstop commands. */
int Driver::pitCommand(tCarElt* car, tSituation *s)
{
    return ROB_PIT_IM; /* return immediately */
}

/* End of the current race */
void Driver::endRace(tCarElt *car, tSituation *s)
{
}

/* Update my private data every timestep */
void Driver::update(tCarElt* car, tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
}

/* Check if I'm stuck */
bool Driver::isStuck(tCarElt* car)
{
    if (fabs(angle) < MAX_UNSTUCK_ANGLE) {
        stuck = 0;
        return false;
    }
    if (stuck < MAX_UNSTUCK_COUNT) {
        stuck++;
        return false;
    } else {
        return true;
    }
}
