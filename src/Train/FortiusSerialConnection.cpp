/*
 * Copyright (c) 2015 by Erik Botö (erik.boto@gmail.com)
 * Copyright (c) 2019 Michael Hipp (totalreverse@mhipp.com)
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

#include "FortiusSerialConnection.h"

#include <QByteArray>
#include <QDebug>

FortiusSerialConnection::FortiusSerialConnection() :
    m_serial(0),
    m_pollInterval(1000),
    m_timer(0),
    m_load(FTS_DEFAULT_LOAD),
    m_gradient(FTS_DEFAULT_GRADIENT),
    m_speed(0),
    m_rawspeed(0),
    m_powerscale(FTS_DEFAULT_SCALING),
    m_mode(FTS_IDLE),
    m_events(0),
    m_power(0),
    m_cadence(0),
    m_weight(FTS_DEFAULT_WEIGHT)
{
}

void FortiusSerialConnection::setSerialPort(const QString serialPortName)
{
    if (! this->isRunning())
    {
        m_serialPortName = serialPortName;
    } else {
        qWarning() << "FortiusSerialConnection: Cannot set serialPortName while running";
    }
}

void FortiusSerialConnection::setPollInterval(int interval)
{
    if (interval != m_pollInterval)
    {
        m_pollInterval = interval;
        m_timer->setInterval(m_pollInterval);
    }
}

int FortiusSerialConnection::pollInterval()
{
    return m_pollInterval;
}

/**
 * Private function that reads a complete reply and prepares if for
 * processing by replacing \r with \0
 */
QByteArray FortiusSerialConnection::readAnswer(int timeoutMs)
{
    QByteArray data;

    do
    {
        if (m_serial->waitForReadyRead(timeoutMs))
        {
            data.append(m_serial->readAll());
        } else {
            data.append(FTS_END_OF_FRAME);
        }
    } while ((data.indexOf(FTS_END_OF_FRAME) == -1));

    return unmarshal(data);
}

/**
 * QThread::run()
 *
 * Open the serial port and set it up, then starts polling.
 *
 */
void FortiusSerialConnection::run()
{
    // Open and configure serial port
    m_serial = new QSerialPort();
    m_serial->setPortName(m_serialPortName);

    m_timer = new QTimer();

    if (!m_serial->open(QSerialPort::ReadWrite))
    {
        qDebug() << "Error opening serial";
        this->exit(-1);
    } else {
        configurePort(m_serial);

        // Discard any existing data
        QByteArray data = m_serial->readAll();

        // Set up polling
        connect(m_timer, SIGNAL(timeout()), this, SLOT(requestAll()),Qt::DirectConnection);
    }

    m_timer->setInterval(100);
    m_timer->start();

    exec();
}

void FortiusSerialConnection::requestAll()
{
    // If something else is blocking mutex, don't start another round of requests
    if (! m_mutex.tryLock())
        return;

    // Discard any existing data
    m_serial->readAll();

    const double calibration_speed  = 20;       // in km/h
    const double kph2rawspeed_magic = 289.75;   // convert km/h to "raw_speed"
    // const double scale_power     = 13;       // convert to watt
    const double scale_slope        = 5*130;
    const double offset_slope       = -0.4;     // -0.4 * scale_slope = -260
    const double scale_calibrate    = 130;
    const double power2load_magic   = 128866;   // ??depends on calibration value??? (128866 matchs for calibration 93*13=1209 - instead of standard 80*13=1040)

    uint16_t nextLoad = 0;
    uint16_t nextCalibrate = scale_calibrate * (m_calibrate + 8.0);  // m_calibrate range -8.0 ... 8.0
    uint8_t nextMode = 0;
    uint8_t nextWeight = 0x52;

    switch(m_mode) {
    case FTS_ERGOMODE:
        // to make it simpler to start: switch off resistance for speeds lower than 5 kph
        if(m_rawspeed > 5*kph2rawspeed_magic) {
            nextLoad = power2load_magic * m_load / m_rawspeed;
        }
        nextMode = 2;
        nextWeight = 0x0a;
        break;
    case FTS_SSMODE:
        nextLoad = (m_gradient + offset_slope) * scale_slope;
        nextMode = 2;
        nextWeight = m_weight;
        break;
    case FTS_CALIBRATE:
        /*
            Note: default calibration is 0x0410 = 1040 = 80*13 (or 8*130)

            The Tacx TTS4 software calibrates with 20 kph (selectedLoad = 0x16a3). If the 'gauge'
            of the calibration screen is perfect in the "green" middle of the calibration bar,
            then TTS4 sets the calibration value to 0x0492 (= 90*13)

            How to find the calibration value without starting the TTS4 software? It'S quite simple:

            The calibration value is exactly the negated value you can read as "resistance" when running
            the calibration at 20 kph. The resistance is byte #14 & #15 (little endian) in the
            unmarshalled 25-byte-T1941-data-frame (or Byte #38 and #39 in a 48 bytes data frame
            from the head unit).

            You start the calibration for 20 kph and for example the avg. "resistance" is about 0xfb70 then
                1. 0x10000-0xFB70 = 0x490
                2. Rounded to multiples of 13 is 0x492. (TTS4 rounds to multiples of 13)
            => your calibration value should be 0x490 (or 0x492)
        */

        nextLoad = calibration_speed * kph2rawspeed_magic; // 20 km/h
        nextMode = 3;
        nextWeight = 0;
        nextCalibrate = 0;
        break;
    case FTS_IDLE:
        default:
        break;
    }

    QByteArray send;

    // header for a "run" command
    send.append('\x01');
    send.append('\x08');
    send.append('\x01');
    send.append('\x00');
    // Load
    send.append((char)((nextLoad>>0) & 0xff));
    send.append((char)((nextLoad>>8) & 0xff));
    // Cad-Echo
    send.append((char)(m_events & 0x01));
    // unknown
    send.append('\x00');
    //
    send.append((char)nextMode);
    send.append((char)nextWeight);
    send.append((char)((nextCalibrate>>0) & 0xff));
    send.append((char)((nextCalibrate>>8) & 0xff));

    m_serial->write(marshal(send));
    if (!m_serial->waitForBytesWritten(500))
    {
        // failure to write to device, bail out
        this->exit(-1);
    }
    QByteArray recvData = readAnswer(500);

    double speed, power;
    //uint32_t totDistance; // total Distance
    uint8_t cadence;
    //uint16_t avgPower, accelerate;  // guessed
    int16_t resistance;    // always the power (*13)

    //uint16_t theLoad;  // "echos" the load we set
    //uint8_t theMode;  // "echos" the mode we set (T1932 has a non chaning "2" after one command)
    //uint16_t checksum;

    const unsigned char *recv = reinterpret_cast<const unsigned char*>(recvData.constData());

    if(recvData.length() >= 23
            && recv[0] == 0x03
            && recv[1] == 19
            && recv[2] == 2
            && recv[3] == 0) {
        m_rawspeed  = (double) (recv[8] | (recv[9]<<8));
        speed       = m_rawspeed / kph2rawspeed_magic;
        cadence     = recv[20];
        m_events    = recv[18];  // 0x01 pedal-sensor event, 0x04 brake-stops event
        //totDistance = recv[4]  | (recv[5] << 8)  | (recv[6] << 16)  | (recv[7] << 24);
        //accelerate  = recv[10] | (recv[11]<<8);  // unknwon
        //avgPower    = recv[12] | (recv[13]<<8);  // MAYBE avg. Power
        resistance       = (recv[14] | (recv[15]<<8));
        power   =  (double) resistance * m_rawspeed / power2load_magic;
        //theLoad     = recv[16] | (recv[17]<<8);
        //theMode     = recv[22];
        //checksum    = recv[23] | (recv[24]<<8);

        m_readMutex.lock();
        m_power = power;
        m_cadence = cadence;
        m_speed = speed;
        m_readMutex.unlock();
    }


    m_mutex.unlock();
}

void FortiusSerialConnection::setMode(unsigned int mode)
{
    m_mode = mode;
}


void FortiusSerialConnection::setLoad(double load)
{
    m_load = load;
}

void FortiusSerialConnection::setGradient(double gradient)
{
    m_gradient = gradient;
}

void FortiusSerialConnection::setBrakeCalibrationFactor(double calibrate)
{
    m_calibrate = calibrate;
}

void FortiusSerialConnection::setWeight(double weight)
{
    m_weight = weight;
}

void FortiusSerialConnection::setPowerScaleFactor(double scale)
{
    m_powerscale = scale;
}

/*
 * Configures a serialport for communicating with a Fortius Brake directly via a serial connection (without a headunit)
 */
void FortiusSerialConnection::configurePort(QSerialPort *serialPort)
{
    if (!serialPort)
    {
        qFatal("Trying to configure null port, start debugging.");
    }
    serialPort->setBaudRate(QSerialPort::Baud19200);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);
    serialPort->setParity(QSerialPort::NoParity);
}

/**
 * This functions takes a serial port and tries if it can find a Fortius Brake
 */
bool FortiusSerialConnection::discover(QString portName)
{
    bool found = false;
    QSerialPort sp;

    qInfo() << "Discover " << portName;

    sp.setPortName(portName);

    if (sp.open(QSerialPort::ReadWrite))
    {
        configurePort(&sp);

        sp.readAll();  // Discard any existing data
        QByteArray send;

        // "version" command 0x2,0x0,0x0,0x0
        send.append('\x02');
        send.append('\x00');
        send.append('\x00');
        send.append('\x00');

        sp.write(marshal(send));
        sp.waitForBytesWritten(1000);

        QByteArray recvFrame;
        do
        {
            bool readyToRead = sp.waitForReadyRead(1000);
            if (readyToRead)
            {
                recvFrame.append(sp.readAll());
            } else {
                recvFrame.append(FTS_END_OF_FRAME);
            }
        } while ((recvFrame.indexOf(FTS_END_OF_FRAME) == -1));

        QByteArray recvData = unmarshal(recvFrame);
        const unsigned char *recv = reinterpret_cast<const unsigned char*>(recvData.constData());

        if (recvData.length() >= 16
                && (recv[0] == 0x03)
                && (recv[1] == 0x0c)
                && (recv[2] == 0x00)
                && (recv[3] == 0x00) ) {
            uint32_t firmwareVersion = recv[4]  | (recv[5] << 8)  | (recv[6] << 16)  | (recv[7] << 24);
            uint32_t serialNr        = recv[8]  | (recv[9] << 8)  | (recv[10] << 16) | (recv[11] << 24);
            uint32_t dateOfFirmware  = recv[12] | (recv[13] << 8) | (recv[14] << 16) | (recv[15] << 24);
            qInfo() << "Found Fortius Serial"
                        << " FirmwareVersion=" << QString::number( firmwareVersion, 16 ).toUpper()
                        << " serialNr=" << QString::number( serialNr, 16 ).toUpper()
                        << " Month/Day(?)=" << QString::number( dateOfFirmware, 16 ).toUpper();
            found = true;
        }
    }

    sp.close();

    return found;
}

int8_t FortiusSerialConnection::hex2bin(uint8_t b)
{
    if(b >= 0x30 && b <= 0x39)    // '0'..'9'
        return b - 0x30;
    if(b >= 0x41 && b <= 0x46)    // 'A'..'F'
        return b + 10 - 0x41;
    if(b >= 0x61 && b <= 0x66)    // 'a'..'f'
        return b + 10 - 0x61;
    if(b == 0x00) {
        // Fallback to handle case with wrong initialized brake
        qDebug() << "hex2bin Fallback for " << b;
        return 0;
    }

    return -1;
}

uint8_t FortiusSerialConnection::bin2hex(uint8_t b)
{
    if( /* b >= 0 && */ b < 10)
        return b + 0x30;      // '0..9'
    if(b >= 10 && b < 16)
        return b - 10 + 0x41; // 'A..F'

    return 0x30;
}

int FortiusSerialConnection::parity16(uint16_t b)
{
    b ^= b >> 8;
    b ^= b >> 4;
    b &= 0xf;
    return (0x6996 >> b) & 1;
}

uint16_t FortiusSerialConnection::checksum(const QByteArray buffer)
{
    uint16_t shiftreg = 0x0000;
    uint16_t poly = 0xc001;
    for(int i=0;i<buffer.length();i++)
    {
        uint8_t a = buffer[i];
        uint16_t tmp = a ^ (shiftreg & 0xff);
        shiftreg >>= 8;

        if(parity16(tmp) == 1)
            shiftreg ^= poly;

        tmp ^= tmp << 1;
        shiftreg ^= tmp << 6;
    }

    return shiftreg;
}

QByteArray FortiusSerialConnection::marshal(const QByteArray in)
{
    QByteArray out;

    out.append(FTS_START_OF_FRAME); // Start Of Frame

    for(int i=0;i<in.length();i++) {
        uint8_t b = in[i];
        out.append(bin2hex((b>>4)&0xf));
        out.append(bin2hex((b>>0)&0xf));
    }

    uint16_t chk = checksum(out);
    out.append(bin2hex((chk>>4)&0xf));
    out.append(bin2hex((chk>>0)&0xf));
    chk >>= 8;
    out.append(bin2hex((chk>>4)&0xf));
    out.append(bin2hex((chk>>0)&0xf));

    out.append(FTS_END_OF_FRAME);  // End of Frame

    return out;
}

QByteArray FortiusSerialConnection::unmarshal(const QByteArray in)
{
    QByteArray out;
    int len = in.length();

    if(in.length() < 6) {
        qDebug() << "frame too short";
        return out;
    }

    if(!in.startsWith(FTS_START_OF_FRAME) || !in.endsWith(FTS_END_OF_FRAME)) {
        qDebug() << "no valid frame";
        return out;
    }

    for(int i=1;i<len-1;i++) {
        if(hex2bin(in[i]) < 0) {
            qDebug() << "illegal data in received data " << in[i];
            return out;
        }
    }

    uint16_t chk = checksum(in.left(len-5));

    uint16_t chkBuf  = hex2bin(in[len-5])<<4;
    chkBuf += hex2bin(in[len-4])<<0;
    chkBuf += hex2bin(in[len-3])<<12;
    chkBuf += hex2bin(in[len-2])<<8;

    if(chk != chkBuf) {
        qDebug() << "checksum error " << chk << " != " << chkBuf;
        return out;
    }

    for(int i=1;i<len-5;i+=2) {
        out.append( (hex2bin(in[i])<<4)+hex2bin(in[i+1]) );
    }

    return out;

}

quint32 FortiusSerialConnection::power()
{
    QMutexLocker mlock(&m_readMutex);
    return m_power * m_powerscale;
}

quint32 FortiusSerialConnection::cadence()
{
    QMutexLocker mlock(&m_readMutex);
    return m_cadence;
}

double FortiusSerialConnection::speed()
{
    QMutexLocker mlock(&m_readMutex);
    return m_speed;
}

double FortiusSerialConnection::gradient()
{
    return m_gradient;
}
