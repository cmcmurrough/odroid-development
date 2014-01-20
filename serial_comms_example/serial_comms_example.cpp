//
//    Copyright 2013 Christopher D. McMurrough
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

/***********************************************************************************************************************
FILENAME:   serial_comms.cpp
AUTHORS:    Christopher D. McMurrough

DESCRIPTION:
Provides an example of communcating with a USB-serial enabled microcontroller

REVISION HISTORY:
01.19.2014  CDM     original file creation
01.20.2014  CDM     published under GPL
***********************************************************************************************************************/

#include <iostream>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_PACKET_START_CHAR 0xAA
#define SERIAL_PACKET_LENGTH 0x06

/***********************************************************************************************************************
int openSerialPort(char* portName)
attempt to open a serial port with the given name, returning -1 on failure
***********************************************************************************************************************/
int openSerialPort(char* portName)
{
    // store the file descriptor for the serial port
    int fd;

    // attempt to open the port
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    // return -1 if we are unable to open the port
    if(fd == -1)
    {
        return -1;
    }
    else
    {
        // clear any existing file descriptor flags
        fcntl(fd, F_SETFL, 0);

        // create a structure to store the port settings
        struct termios port_settings;

        // set the baud rates
        cfsetispeed(&port_settings, B9600);
        cfsetospeed(&port_settings, B9600);

        // set parity, stop bits, and data bits
        port_settings.c_cflag &= ~PARENB;
        port_settings.c_cflag &= ~CSTOPB;
        port_settings.c_cflag &= ~CSIZE;
        port_settings.c_cflag |= CS8;

        // set the port to use all 8 bits
        port_settings.c_iflag &= ~ISTRIP;

        // apply the settings to the port
        tcsetattr(fd, TCSANOW, &port_settings);

        // set the non blocking functionality
        fcntl(fd, F_SETFL, O_NONBLOCK);

        // return the file descriptor
        return(fd);
    }
}

/***********************************************************************************************************************
void closeSerialPort(int serialPort)
close the given serial port
***********************************************************************************************************************/
void closeSerialPort(int serialPort)
{
    tcflush(serialPort, TCIOFLUSH);
    close(serialPort);
}

/***********************************************************************************************************************
void writeSerialBytes(int serialPort, unsigned char* data, int numBytes)
write the byte data to the serial port
***********************************************************************************************************************/
void writeSerialBytes(int serialPort, unsigned char* data, int numBytes)
{
    int result = write(serialPort, data, numBytes);
}

/***********************************************************************************************************************
void makeTeensyPacket(unsigned char* buffer, unsigned char val_1, unsigned char val_2, unsigned char val_3)
create a serial packet containing the data and framing bytes
***********************************************************************************************************************/
void makeTeensyPacket(unsigned char* buffer, unsigned char b1, unsigned char b2, unsigned char b3)
{
    // initialize the packet
    buffer[0] = SERIAL_PACKET_START_CHAR;
    buffer[1] = SERIAL_PACKET_LENGTH;
    buffer[2] = b1;
    buffer[3] = b2;
    buffer[4] = b3;
    buffer[5] = 0x00;

    // compute the checksum
    for(int i = 0; i < SERIAL_PACKET_LENGTH - 1; i++)
    {
        buffer[SERIAL_PACKET_LENGTH - 1] = buffer[SERIAL_PACKET_LENGTH - 1] ^ buffer[i];
    }
}

/***********************************************************************************************************************
void sendPacket(int serialPort, unsigned char b1, unsigned char b2, unsigned char b3)
create a packet from the given parameters and send to the serial port
***********************************************************************************************************************/
void sendPacket(int serialPort, unsigned char b1, unsigned char b2, unsigned char b3)
{
    // create the outgoing packet buffer
    unsigned char packet[SERIAL_PACKET_LENGTH];

    // create the packet
    makeTeensyPacket(packet, b1, b2, b3);

    // write the packet
    writeSerialBytes(serialPort, packet, SERIAL_PACKET_LENGTH);
}

/***********************************************************************************************************************
int pollSerialPort(int serialPort)
processed received data from the serial port, returning the number of bytes
***********************************************************************************************************************/
int pollSerialPort(int serialPort)
{
    const int bufferSize = 100;
    static unsigned char buff[bufferSize];
    int n;

    // attempt to read bytes from the port
    n = read(serialPort, buff, bufferSize);

    // print any received bytes to terminal
    if(n > 0)
    {
        printf("RECEIVED BYTES: ");
        for(int i = 0; i < n; i++)
        {
            char ch = buff[i];
            printf("%x ", ch & 0xff);
        }
        printf("/n");
    }

    // return the number of bytes processed
    return n;
}

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, expects the serial port name as a command line argument
***********************************************************************************************************************/
int main(int argc, char **argv)
{
    // store the port parameters
    char* portName;
    int serialPort;

    // parse the command line arguments
    if(argc != 2)
    {
        printf("USAGE: %s <port_name> \n", argv[0]);
        return 0;
    }
    portName = argv[1];

    // attempt to open the serial port
    serialPort = openSerialPort(portName);

    // check to see if we connected successfully
    if(serialPort == -1)
    {
        printf("unable to open serial port %s \n", portName);
        return(0);
    }

    // continuously check for received serial data
    int numBytes = 0;
    while(1)
    {
        // check for received serial data
        numBytes = numBytes + pollSerialPort(serialPort);

        // send a packet
        sendPacket(serialPort, 0x01, 0x02, 0x03);

        // stop listening for data once we receive 100 bytes
        if(numBytes >= 100)
        {
            break;
        }

        // sleep for 20ms (20*1000 microseconds)
        usleep(20*1000);
    }

    // close the serial port
    closeSerialPort(serialPort);
}

