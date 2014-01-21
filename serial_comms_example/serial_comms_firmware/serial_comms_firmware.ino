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
FILENAME:   serial_comms_firmware.cpp
AUTHORS:    Christopher D. McMurrough

DESCRIPTION:
An example Arduino sketch showing USB-serial communications with the Teensy 3.0 microcontroller

REVISION HISTORY:
01.19.2014  CDM     original file creation
01.20.2014  CDM     published under GPL
***********************************************************************************************************************/

// set up the LED
const int LED_PIN = 13;
const int LED_ON = HIGH;
const int LED_OFF = LOW;

// define serial parameters
const byte PACKET_START_CHAR = 0xAA;
const byte PACKET_LENGTH = 0x06;           // [0XAA,length,b1,b2,b3,checksum]
unsigned long BAUD_RATE = 9600;
byte Packet_Buffer[PACKET_LENGTH];

/***********************************************************************************************************************
void setup()
set up the teensy
***********************************************************************************************************************/
void setup()
{
    // set up the LED
    pinMode(LED_PIN, OUTPUT);

    // flash the LED state
    for(int i = 0; i < 25; i++)
    {
        digitalWrite(LED_PIN, LED_ON);
        delay(50);
        digitalWrite(LED_PIN, LED_OFF);
        delay(50);
    }

    Serial.begin(BAUD_RATE);
}

/***********************************************************************************************************************
void sendPacket(byte b1, byte b2, byte b3)
send a packet with 2 payload bytes
***********************************************************************************************************************/
void sendPacket(byte b1, byte b2, byte b3)
{
    byte packet[PACKET_LENGTH] = {PACKET_START_CHAR, PACKET_LENGTH, b1, b2, b3, 0x00};

    // compute the checksum
    for(int i = 0; i < PACKET_LENGTH - 1; i++)
    {
        packet[PACKET_LENGTH - 1] = packet[PACKET_LENGTH - 1] ^ packet[i];
    }

    // send the packet
    Serial.write(packet, PACKET_LENGTH);
    Serial.flush();
}


/***********************************************************************************************************************
boolean isValidPacket()
check to see if the buffer contains a valid packet
***********************************************************************************************************************/
boolean isValidPacket()
{
    byte checksum = 0x00;

    // check the start byte
    if(Packet_Buffer[0] != PACKET_START_CHAR)
    {
        return false;
    }

    // check the length byte
    if(Packet_Buffer[1] != PACKET_LENGTH)
    {
        return false;
    }

    // compute the checksum
    for(int i = 0; i < (PACKET_LENGTH - 1); i++)
    {
        checksum = checksum ^ Packet_Buffer[i];
    }

    // validate with checksum
    if(Packet_Buffer[PACKET_LENGTH - 1] == checksum)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/***********************************************************************************************************************
void doSomething(byte b1, byte b2, byte b3)
perform some action with the data we received from the host
***********************************************************************************************************************/
void doSomething(byte b1, byte b2, byte b3)
{                   
    // echo the packet to acknowledge successful receipt
    sendPacket(Packet_Buffer[2], Packet_Buffer[3], Packet_Buffer[4]);
}

/***********************************************************************************************************************
void loop()
main program loop
***********************************************************************************************************************/
void loop()
{
    byte b;
    int index = 0;
    boolean packetIndex = 0;
    boolean ledState = false;
    boolean validPacket = false;

    // continuously check for received packets and echo upon receipt
    while(true)
    {
        // check to see if a character is available
        if(Serial.available())
        { 
            // get the byte
            b = Serial.read();

            // handle the first byte of a packet
            if(packetIndex == 0)
            {
                if(b == PACKET_START_CHAR)
                {
                    Packet_Buffer[packetIndex] = b;
                    packetIndex++;
                }
            }
            else  // handle bytes other than the packet start byte
            {
                Packet_Buffer[packetIndex] = b;

                // check to see if we have enough bytes for a complete packet
                if(packetIndex >= (PACKET_LENGTH - 1))
                {
                    // check to see if we have a valid packet
                    if(isValidPacket())
                    {
                        // we have a valid packet, now do something with it
                        doSomething(Packet_Buffer[2], Packet_Buffer[3], Packet_Buffer[4]);
                    }

                    // reset the byte counter
                    packetIndex = 0;
                }
                else
                {
                    // increment the byte counter
                    packetIndex++;
                }
            }
        }
    }
}


