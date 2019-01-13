/*
 * Copyright (c) 2009 Mark Liversedge (liversedge@gmail.com)
 * Copyright (c) 2015 Erik Botö (erik.boto@gmail.com)
 * Copyright (c) 2019 ....
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "FortiusSerialController.h"
#include "FortiusSerialConnection.h"
#include "RealtimeData.h"

#include <QMessageBox>
#include <QSerialPort>

FortiusSerialController::FortiusSerialController(TrainSidebar *parent,  DeviceConfiguration *dc) : RealtimeController(parent, dc)
{
    m_fortius = new FortiusSerialConnection();
    m_fortius->setSerialPort(dc ? dc->portSpec : "");
}

bool FortiusSerialController::find()
{
    return false;
}

int
FortiusSerialController::start()
{
    m_fortius->start();
    return 0;
}


int
FortiusSerialController::restart()
{
    return 0;
}


int
FortiusSerialController::pause()
{
    return 0;
}


int
FortiusSerialController::stop()
{
    return 0;
}


bool
FortiusSerialController::discover(QString name)
{
   return m_fortius->discover(name);
}


bool FortiusSerialController::doesPush() { return false; }
bool FortiusSerialController::doesPull() { return true; }
bool FortiusSerialController::doesLoad() { return true; }

/*
 * gets called from the GUI to get updated telemetry.
 * so whilst we are at it we check button status too and
 * act accordingly.
 *
 */
void
FortiusSerialController::getRealtimeData(RealtimeData &rtData)
{
    if (m_fortius->isFinished())
    {
        QMessageBox msgBox;
        msgBox.setText(tr("Cannot Connect to Fortius (serial)"));
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.exec();
        parent->Stop(0);
        parent->Disconnect();
        return;
    }

    rtData.setWatts(m_fortius->power());
    rtData.setCadence(m_fortius->cadence());
    rtData.setSlope(m_fortius->gradient());
    rtData.setSpeed(m_fortius->speed());
}

void FortiusSerialController::pushRealtimeData(RealtimeData &) { } // update realtime data with current values

void FortiusSerialController::setMode(int mode)
{
    if (mode == RT_MODE_ERGO) mode = FTS_ERGOMODE;
    else if (mode == RT_MODE_SPIN) mode = FTS_SSMODE;
    else if (mode == RT_MODE_CALIBRATE) mode = FTS_CALIBRATE;
    else mode = FTS_IDLE;
    m_fortius->setMode(mode);
}

// Alters the relationship between brake setpoint at load.
void FortiusSerialController::setBrakeCalibrationFactor(double brakeCalibrationFactor)
{
    m_fortius->setBrakeCalibrationFactor(brakeCalibrationFactor);
}

// output power adjusted by this value so user can compare with hub or crank based readings
void FortiusSerialController::setPowerScaleFactor(double powerScaleFactor)
{
    if (powerScaleFactor < 0.8) powerScaleFactor = 0.8;
    if (powerScaleFactor > 1.2) powerScaleFactor = 1.2;

    m_fortius->setPowerScaleFactor(powerScaleFactor);
}

// User weight used by brake in slope mode
void FortiusSerialController::setWeight(double weight)
{
    // need to apply range as same byte used to signify erg mode
    if (weight < 50) weight = 50;
    if (weight > 120) weight = 120;

    m_fortius->setWeight(weight);
}

void FortiusSerialController::setLoad(double load)
{
    m_fortius->setLoad(load);
}

void FortiusSerialController::setGradient(double gradient)
{
    m_fortius->setGradient(gradient);
}
