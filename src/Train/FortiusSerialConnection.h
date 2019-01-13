/*
 * Copyright (c) 2015 Erik Botö (erik.boto@gmail.com)
 * Copyright (c) 2019 Michael Hipp
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

#ifndef _GC_FortiusSerialConnection_h
#define _GC_FortiusSerialConnection_h 1

#include <QtSerialPort/QSerialPort>
#include <QThread>
#include <QTimer>
#include <QMutex>

#define FT_IDLE        0x00
#define FT_ERGOMODE    0x01
#define FT_SSMODE      0x02
#define FT_CALIBRATE   0x04

#define DEFAULT_LOAD         100.00
#define DEFAULT_GRADIENT     2.00
#define DEFAULT_WEIGHT       77
#define DEFAULT_CALIBRATION  0.00
#define DEFAULT_SCALING      1.00

class FortiusSerialConnection : public QThread
{
    Q_OBJECT

public:
    FortiusSerialConnection();
    void setPollInterval(int interval);
    int pollInterval();
    void setSerialPort(const QString serialPortName);
    static void configurePort(QSerialPort * serialPort);
    static bool discover(QString portName);

    quint32 power();
    quint32 cadence();
    double gradient();
    double speed();

    void setLoad(double load);
    void setGradient(double gradient);
    void setBrakeCalibrationFactor(double calibrationFactor);
    void setPowerScaleFactor(double calibrationFactor);
    void setMode(unsigned int mode);
    void setWeight(double weight);                 //  weight of rider & bike in kg

public slots:
    void requestAll();

private:
    QString m_serialPortName;
    QSerialPort *m_serial;
    int m_pollInterval;
    QString m_id;
    void run();
    QTimer *m_timer;
    QByteArray readAnswer(int timeoutMs = -1);
    QMutex m_mutex;
    QMutex m_readMutex;
    double m_load;
    double m_gradient;
    double m_speed;
    double m_powerscale;
    int m_mode;
    uint8_t m_events; // stores last event from trainer to trigger cad-sensor signal

    quint32 m_power;
    quint32 m_cadence;
    quint32 m_calibrate;
    quint32 m_weight;

    static uint8_t hex2bin(uint8_t b);
    static uint8_t bin2hex(uint8_t b);
    static int parity16(uint16_t b);
    static uint16_t checksum(const QByteArray buffer);
    static QByteArray marshal(const QByteArray in);
    static QByteArray unmarshal(const QByteArray in);

//signals:
//    void cadence(quint32);
//    void power(quint32);
};

#endif // _GC_FortiusSerialConnection_h
