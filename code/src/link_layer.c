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

// retries and timeout
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
        setFrame[3] = BCC1(setFrame[1], setFrame[2]);  // calculate BCC1

        enum State {SEND_SET, WAIT_UA, STOP};
        enum State state = SEND_SET;
        unsigned char uaFrame[5] = {0};
        int retry = 0;

        while (state != STOP && retry < MAX_RETRIES){
            switch(state){
                case SEND_SET:
                    if (writeBytesSerialPort(setFrame, sizeof(setFrame)) < 0) return -1;
                    state = WAIT_UA;
                    break;

                case WAIT_UA:
                    unsigned char byte;
                    int bytesRead = 0;
                    for (int i = 0; i < 5; i++)
                    {
                        int res = readByteSerialPort(&byte);
                        if (res <= 0) break;
                        bytesRead++;
                        uaFrame[i] = byte;
                    }

                    if (bytesRead == 5 && uaFrame[2] == CTRL_UA && uaFrame[1] == ADDR_RX_COMMAND 
                    && uaFrame[3] == BCC1(uaFrame[1], uaFrame[2])) 
                        return 1;
                    else
                    {
                        state = SEND_SET;
                        retry++;
                    }

                    break;
            }
        }
        return -1; //failed
    }


    else if (connectionParameters.role == LlRx)
    {
        enum State {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP};
        enum State state = START;
        unsigned char byte;
        unsigned char setFrame[5];
        int i = 0;

        while (state != STOP)
        {
            int res = readByteSerialPort(&byte); 
            if (res <= 0) return -1; 

            switch (state)
            {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == ADDR_TX_COMMAND) {
                        state = A_RCV;
                        setFrame[1] = byte;
                    }
                    else if (byte != FLAG) state = START;  
                    break;

                case A_RCV:
                    if (byte == CTRL_SET) {
                        state = C_RCV;
                        setFrame[2] = byte;
                    }
                    else if (byte == FLAG) state = FLAG_RCV; 
                    else state = START; 
                    break;

                case C_RCV:
                    if (byte == (setFrame[1] ^ setFrame[2])) state = BCC_OK; 
                    else if (byte == FLAG) state = FLAG_RCV; 
                    else state = START;  
                    break;

                case BCC_OK:
                    if (byte == FLAG) state = STOP;  
                    else state = START; 
                    break;
            }
        }

        // After successfully receiving the SET frame, send UA frame
        unsigned char uaFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_UA, 0x00, FLAG};
        uaFrame[3] = BCC1(uaFrame[1], uaFrame[2]);  // Calculate BCC1
        if (writeBytesSerialPort(uaFrame, sizeof(uaFrame)) < 0)
            return -1;
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
