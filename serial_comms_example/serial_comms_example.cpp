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

/*******************************************************************************************************************//**
 * @file serial_comms_example.cpp
 * @brief Provides an example of communcating with a USB-serial enabled microcontroller
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/

#include <iostream>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_PACKET_START_CHAR 0xAA
#define SERIAL_PACKET_LENGTH 0x06

/*******************************************************************************************************************//**
 * @brief attempt to open a serial port with the given name
 * @param[in] portName the name of the port to connect to (dev/tty/USB0, COM1, etc)
 * @return file descriptor of the serial port, or -1 on failure
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
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
        // create a structure to store the port settings
        struct termios port_settings;

        // get the current port settings
        tcgetattr(fd, &port_settings);

        // set the baud rates
        cfsetispeed(&port_settings, B9600);
        cfsetospeed(&port_settings, B9600);

        // set 8 bits, no parity, no stop bits
        port_settings.c_cflag &= ~PARENB;
        port_settings.c_cflag &= ~CSTOPB;
        port_settings.c_cflag &= ~CSIZE;
        port_settings.c_cflag |= CS8;

        // set raw input mode
        port_settings.c_lflag |= ~ICANON;

        // apply the settings to the port
        tcsetattr(fd, TCSANOW, &port_settings);

        // set the non blocking functionality
        fcntl(fd, F_SETFL, O_NONBLOCK);

        // return the file descriptor
        return(fd);
    }
}

/*******************************************************************************************************************//**
 * @brief close the given serial port
 * @param[in] serialPort file descriptor of the serial port
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
void closeSerialPort(int serialPort)
{
    tcflush(serialPort, TCIOFLUSH);
    close(serialPort);
}

/*******************************************************************************************************************//**
 * @brief write a byte buffer to the serial port
 * @param[in] serialPort file descriptor of the serial port
 * @param[in] data byte buffer to write to the port
 * @param[in] numBytes number of bytes to write
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
void writeSerialBytes(int serialPort, unsigned char* data, int numBytes)
{
    int result = write(serialPort, data, numBytes);
}

/*******************************************************************************************************************//**
 * @brief create a serial packet containing the data and framing bytes
 * @param[out] buffer output packet buffer after assembly
 * @param[in] b1 input byte 1
 * @param[in] b2 input byte 2
 * @param[in] b3 input byte 3
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
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

/*******************************************************************************************************************//**
 * @brief assemble a packet buffer of bytes and write them to the serial port
 * @param[in] serialPort file descriptor of the serial port
 * @param[in] b1 input byte 1
 * @param[in] b2 input byte 2
 * @param[in] b3 input byte 3
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
void sendPacket(int serialPort, unsigned char b1, unsigned char b2, unsigned char b3)
{
    // create the outgoing packet buffer
    unsigned char packet[SERIAL_PACKET_LENGTH];

    // create the packet
    makeTeensyPacket(packet, b1, b2, b3);

    printf("SENDING BYTES: ");
    for(int i = 0; i < SERIAL_PACKET_LENGTH; i++)
    {
        char ch = packet[i];
        printf("0x%X ", ch & 0xff);
    }
    printf("\n");

    // write the packet
    writeSerialBytes(serialPort, packet, SERIAL_PACKET_LENGTH);
}

/*******************************************************************************************************************//**
 * @brief processed received data from the serial port, returning the number of bytes
 * @param[in] serialPort file descriptor of the serial port
 * @return the number of bytes read from the port
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
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
            printf("0x%X ", ch & 0xff);
        }
        printf("\n");
    }

    // return the number of bytes processed
    return n;
}

/***********************************************************************************************************************
 * @brief program entry point
 * @param[in] argc number of command line arguments
 * @param[in] argv string array of command line arguments
 * @return return code (0 for normal termination)
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
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
    printf("%s opened successfully!\n", portName);

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

