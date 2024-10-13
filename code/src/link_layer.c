// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// flag
#define FLAG 0x7E

// address field values
#define ADDR_TX_COMMAND 0x03  // Transmitter
#define ADDR_RX_COMMAND 0x01  // Receiver

// control field values (for specific frames)
#define CTRL_SET 0x03  // sent by the transmitter to initiate a connection
#define CTRL_UA  0x07  // confirmation to the reception of a valid supervision frame
#define CTRL_RR0 0xAA  // indication sent by the Receiver that it is ready to receive frame 0
#define CTRL_RR1 0xAB  // indication sent by the Receiver that it is ready to receive frame 1
#define CTRL_REJ0 0x54 // Receiver rejects frame 0 (detected an error)
#define CTRL_REJ1 0x55 // Receiver rejects frame 1 (detected an error)
#define CTRL_DISC 0x0B // termination of a connection

// BCC1 calculation
#define BCC1(addr, ctrl) ((addr) ^ (ctrl))  // Address XOR Control field

#define MAX_RETRIES 3
#define TIMEOUT_SECONDS 1

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    // TODO

    if (connectionParameters.role == LlTx)
    {
        unsigned char setFrame[5] = {FLAG, ADDR_TX_COMMAND, CTRL_SET, 0x00, FLAG};
        setFrame[3] = BCC1(setFrame[1], setFrame[2]); // calculate BCC1
        
        for (int retry = 0; retry < MAX_RETRIES; retry++) 
        {
            // send set frame
            if (writeBytesSerialPort(setFrame, sizeof(setFrame)) < 0) return -1;

            // wait for ua frame with timeout
            unsigned char uaFrame[5] = {0};
            int bytesRead = 0;
            for (int i = 0; i < 5; i++)
            {
                int res = readByteSerialPort(&uaFrame[i]);
                if (res <= 0) break;
                bytesRead++;
            }

            // verify ua frame
            if (bytesRead == 5 && uaFrame[2] == CTRL_UA && uaFrame[1] 
            == ADDR_RX_COMMAND && uaFrame[3] == BCC1(uaFrame[1], uaFrame[2])) 
                return 1;

        }
        return -1; //failed
    }


    else if (connectionParameters.role == LlRx)
    {
        // wait for set frame
        unsigned char setFrame[5] = {0};
        int bytesRead = 0;

        for (int i = 0; i < 5; i++)
        {
            int res = readByteSerialPort(&setFrame[i]);  // Read one byte at a time
            if (res <= 0) break;
            bytesRead++;
        }

        // verify set frame
        if (setFrame[2] != CTRL_SET || setFrame[1] != ADDR_TX_COMMAND || setFrame[3] != BCC1(setFrame[1], setFrame[2])) return -1; 

        // create the ua frame
        unsigned char uaFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_UA, 0x00, FLAG};
        uaFrame[3] = BCC1(uaFrame[1], uaFrame[2]); //calcculate BCC1

        // Send the UA frame
        if (writeBytesSerialPort(uaFrame, sizeof(uaFrame)) < 0) return -1;

    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
