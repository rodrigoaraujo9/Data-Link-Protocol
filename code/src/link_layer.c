// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/signal.h>

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
#define MAX_RETRIES 10
#define TIMEOUT_SECONDS 3
#define READ_RETRIES 5     // Retries for individual byte reads

// Error Codes
#define ERR_MAX_RETRIES_EXCEEDED -2
#define ERR_WRITE_FAILED         -3
#define ERR_READ_TIMEOUT         -4
#define ERR_INVALID_BCC          -5
#define ERR_FRAME_REJECTED       -6
#define ERR_WRITE_TIMEOUT         -7

enum StateSND {SEND_SET, WAIT_UA, SND_STOP};
enum StateRCV {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, RCV_STOP};

// Global variables to handle alarm logic
volatile int alarmTriggered = FALSE;
int alarmCount = 0;
int timeout = TIMEOUT_SECONDS;  // Set your desired timeout


void handle_alarm(int sig) {
    alarmTriggered = 1;  // Set the flag when the alarm triggers
    alarmCount++;        // Increment alarm count
    printf("[ALARM] Timeout occurred. Alarm count: %d seconds.\n", alarmCount);
    alarm(1);  // Set next alarm for 1 second
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    signal(SIGALRM, handle_alarm);  
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        printf("[ERROR] Failed to open serial port\n");
        return -1;
    }

    if (connectionParameters.role == LlTx) {
        unsigned char setFrame[5] = {FLAG, ADDR_TX_COMMAND, CTRL_SET, 0x00, FLAG};
        setFrame[3] = BCC1(setFrame[1], setFrame[2]);

        enum StateSND state = SEND_SET;
        unsigned char uaFrame[5] = {0};
        int retry = 0;

        while (state != SND_STOP && retry < MAX_RETRIES) {
            switch (state) {
                case SEND_SET: {
                    if (writeBytesSerialPort(setFrame, sizeof(setFrame)) < 0) {
                        printf("[ERROR] Failed to send SET frame\n");
                        return ERR_WRITE_FAILED;
                    } 

                    printf("[INFO] Sent SET frame, setting alarm for 1 second retry\n");
                    alarmTriggered = 0; 
                    alarm(1);           

                    state = WAIT_UA;
                    break;
                }

                case WAIT_UA: {
                    enum StateRCV stateR = START;
                    unsigned char byte;
                    int bytesRead = 0;

                    while (stateR != RCV_STOP && bytesRead < 5 && alarmTriggered == 0) {
                        int res = readByteSerialPort(&byte);
                        if (res <= 0) {
                            if (alarmTriggered) {
                                retry++;
                                if (retry >= MAX_RETRIES) {
                                    printf("[ERROR] Maximum retries exceeded while trying to establish connection\n");
                                    alarm(0); 
                                    return ERR_MAX_RETRIES_EXCEEDED;
                                }
                                printf("[WARN] Timeout while waiting for UA frame. Retrying... (%d/%d)\n", retry, MAX_RETRIES);
                                alarmTriggered = 0; 
                                state = SEND_SET;     
                                break;
                            }
                            continue;
                        }

                        uaFrame[bytesRead++] = byte;

                        switch (stateR) {
                            case START:
                                if (byte == FLAG) stateR = FLAG_RCV;
                                break;

                            case FLAG_RCV:
                                if (byte == ADDR_RX_COMMAND) stateR = A_RCV;
                                else if (byte != FLAG) stateR = START;
                                break;

                            case A_RCV:
                                if (byte == CTRL_UA) stateR = C_RCV;
                                else if (byte == FLAG) stateR = FLAG_RCV;
                                else stateR = START;
                                break;

                            case C_RCV:
                                if (byte == (uaFrame[1] ^ uaFrame[2])) stateR = BCC_OK;
                                else if (byte == FLAG) stateR = FLAG_RCV;
                                else stateR = START;
                                break;

                            case BCC_OK:
                                if (byte == FLAG) stateR = RCV_STOP;
                                else stateR = START;
                                break;
                        }
                    }

                    if (stateR == RCV_STOP) {
                        printf("[INFO] UA frame successfully received, connection established\n");
                        alarm(0);  
                        return 1; 
                    }
                    break;
                }
            }
        }

        printf("[ERROR] Maximum retries exceeded while trying to establish connection\n");
        alarm(0);
        return ERR_MAX_RETRIES_EXCEEDED;
    }
    else if (connectionParameters.role == LlRx)
    {
        signal(SIGALRM, handle_alarm);

        enum StateRCV state = START;
        unsigned char byte;
        unsigned char setFrame[5];
        int retry = 0;

        alarm(1); 
        alarmTriggered = 0; 

        while (state != RCV_STOP && retry < MAX_RETRIES) 
        {
            int res = readByteSerialPort(&byte); 
            if (res <= 0)
            {
                if (alarmTriggered) {
                    retry++;
                    if (retry >= MAX_RETRIES) {
                        printf("[ERROR] Maximum retries exceeded while waiting for SET frame\n");
                        return ERR_MAX_RETRIES_EXCEEDED;
                    }
                    printf("[ERROR] Timeout while waiting for SET frame, retrying... (%d/%d)\n", retry, MAX_RETRIES);
                    alarmTriggered = 0; 
                    state = START;       
                    alarm(1);            
                    continue;
                }
                continue;
            }

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
                    if (byte == FLAG) state = RCV_STOP; 
                    else state = START; 
                    break;
            }

            if (alarmTriggered && retry >= MAX_RETRIES) {
                printf("[ERROR] Maximum retries exceeded while waiting for SET frame\n");
                return ERR_MAX_RETRIES_EXCEEDED;
            }

            if (alarmTriggered) {
                retry++;
                printf("[ERROR] Timeout reached while waiting for SET frame, retrying... (%d/%d)\n", retry, MAX_RETRIES);
                alarmTriggered = 0;    
                alarm(1);              
            }
        }

        alarm(0);  

        unsigned char uaFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_UA, 0x00, FLAG};
        uaFrame[3] = BCC1(uaFrame[1], uaFrame[2]); 

        if (writeBytesSerialPort(uaFrame, sizeof(uaFrame)) < 0)
        {
            printf("[ERROR] Failed to send UA frame\n");
            return ERR_WRITE_FAILED;
        }

        printf("[INFO] UA frame sent successfully\n");
        printf("[INFO] SUCCESS!\n");
    }

    return 1;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    int overhead = 6;  // 2 flags + Address + Control + BCC1 + BCC2
    int frameSize = bufSize + overhead;

    unsigned char frame[frameSize];
    int index = 0;
    static int Ns = 0;
    unsigned char bcc2 = 0;

    frame[index++] = FLAG;
    frame[index++] = ADDR_TX_COMMAND; 
    if (Ns == 0) {
    frame[index++] = CTRL_RR0;
    } else {
    frame[index++] = CTRL_RR1;
    }
    frame[index++] = BCC1(frame[1], frame[2]);

    for (int i = 0; i < bufSize; i++) {
        if (buf[i] == FLAG) {
            frame[index++] = 0x7D;  
            frame[index++] = 0x5E;  
        } else if (buf[i] == 0x7D) {
            frame[index++] = 0x7D; 
            frame[index++] = 0x5D;  
        } else {
            frame[index++] = buf[i];
        }
        bcc2 ^= buf[i];
    }

    frame[index++] = bcc2;

    frame[index++] = FLAG;

    int result = writeBytesSerialPort(frame, index);

    if (result < 0) {
        return ERR_WRITE_FAILED; 
    }

    Ns = (Ns + 1) % 2;

    return bufSize;

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
