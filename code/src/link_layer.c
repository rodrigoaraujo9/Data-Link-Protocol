#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/signal.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/termios.h>
#define _POSIX_SOURCE 1
#define FLAG 0x7E
#define ADDR_TX_COMMAND 0x03
#define ADDR_RX_COMMAND 0x01
#define CTRL_SET 0x03
#define CTRL_UA 0x07
#define CTRL_RR0 0xAA
#define CTRL_RR1 0xAB
#define CTRL_REJ0 0x54
#define CTRL_REJ1 0x55
#define CTRL_DISC 0x0B
#define BCC1(addr, ctrl) ((addr) ^ (ctrl))
#define MAX_RETRIES 10
#define ERR_MAX_RETRIES_EXCEEDED -2
#define ERR_WRITE_FAILED -3
#define ERR_READ_TIMEOUT -4
#define ERR_INVALID_BCC -5
#define ERR_FRAME_REJECTED -6
#define ERR_WRITE_TIMEOUT -7

enum StateSND {SEND_SET, WAIT_UA, SND_STOP};
enum StateRCV {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, RCV_STOP};

extern int totalBytesSent;
extern int totalBytesReceived;
extern double transmissionStartTime;
extern double transmissionEndTime;

volatile int alarmTriggered = FALSE;
int alarmCount = 0;
int timeout;
LinkLayerRole role;
int baudRate;
int transmissions;
static int expectedSequence = 0;

int serialPortOpen = 1;



void handle_alarm(int sig) {
    alarmTriggered = 1;
    alarmCount++;
    alarm(1);
}

void resetPortState() {
    flushPort();
    alarmCount = 0;
    alarmTriggered = FALSE;
    printf("[INFO] Port state reset: alarm count and trigger reset.\n");
}

int llopen(LinkLayer connectionParams) {
    role = connectionParams.role;
    baudRate = connectionParams.baudRate;
    timeout = connectionParams.timeout;
    transmissions = connectionParams.nRetransmissions;
    alarmCount = 0;
    alarmTriggered = FALSE;
    totalBytesSent = 0;
    totalBytesReceived = 0;
    transmissionStartTime = 0;
    transmissionEndTime = 0;
    signal(SIGALRM, handle_alarm);

    if (openSerialPort(connectionParams.serialPort, connectionParams.baudRate) < 0) {
        printf("[ERROR] Failed to open serial port: %s\n", connectionParams.serialPort);
        return -1;
    }

    printf("[INFO] Serial port opened successfully: %s\n", connectionParams.serialPort);

    //resetPortState();

    if (connectionParams.role == LlTx) {
        unsigned char setFrame[5] = {FLAG, ADDR_TX_COMMAND, CTRL_SET, BCC1(ADDR_TX_COMMAND, CTRL_SET), FLAG};
        int retry = 0;

        while (retry < connectionParams.nRetransmissions) {
            printf("[INFO] Sending SET frame, attempt %d\n", retry + 1);
            if (writeBytesSerialPort(setFrame, sizeof(setFrame)) < 0) {
                printf("[ERROR] Failed to send SET frame. Reinitializing serial port.\n");
                closeSerialPort();
                if (openSerialPort(connectionParams.serialPort, connectionParams.baudRate) < 0) {
                    printf("[ERROR] Failed to reopen serial port: %s\n", connectionParams.serialPort);
                    return -1;
                }
                resetPortState();
                retry++;
                continue;
            }

            printf("[INFO] Sent SET frame, waiting for UA response...\n");
            alarmTriggered = 0;
            alarm(connectionParams.timeout);
            unsigned char byte;
            enum StateRCV state = START;

            while (!alarmTriggered && state != RCV_STOP) {
                int res = readByteSerialPort(&byte);
                if (alarmTriggered) {
                	int bytes_write = 0; 
               		 bytes_write = writeBytesSerialPort(setFrame, sizeof(setFrame));
               		 printf("%d\n",bytes_write);
                        printf("[WARN] Timeout waiting for UA. Retrying...\n");
                        retry++;
                        if (retry >= connectionParams.nRetransmissions) {
                            printf("[ERROR] Maximum retries reached. Connection failed.\n");
                            resetPortState();
                            return ERR_MAX_RETRIES_EXCEEDED;
                        }

                    }
                if (res > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == ADDR_RX_COMMAND) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == CTRL_UA) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == BCC1(ADDR_RX_COMMAND, CTRL_UA)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) state = RCV_STOP;
                        else state = START;
                        break;
                    default:
                        break;
                }

                }

               
            }

            if (state == RCV_STOP) {
                printf("[INFO] UA frame received, connection established successfully.\n");
                alarm(0);
                return 1;
            }
        }
    } else if (connectionParams.role == LlRx) {
        
        enum StateRCV state = START;
        unsigned char byte;

        while (state != RCV_STOP) {
            int res = readByteSerialPort(&byte);
       
               if(res >0){ 
               printf("%02x\n",byte);
               switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == ADDR_TX_COMMAND) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case A_RCV:
                    if (byte == CTRL_SET) state = C_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == BCC1(ADDR_TX_COMMAND, CTRL_SET)) state = BCC_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG) state = RCV_STOP;
                    else state = START;
                    break;
                default:
                    break;
            }
               }
           
        }

        unsigned char uaFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_UA, BCC1(ADDR_RX_COMMAND, CTRL_UA), FLAG};
        if (writeBytesSerialPort(uaFrame, sizeof(uaFrame)) < 0) {
            printf("[ERROR] Failed to send UA frame.\n");
            resetPortState();
            return ERR_WRITE_FAILED;
        }

        printf("[INFO] UA frame sent.\n");
        return 1;
    }
    printf("[ERROR] Connection could not be established after maximum retries.\n");
    return ERR_MAX_RETRIES_EXCEEDED;
}

int llwrite(const unsigned char *buf, int bufSize) {
    int overhead = 6;
    int maxFrameSize = bufSize * 2 + overhead;
    unsigned char *frame = (unsigned char *)malloc(maxFrameSize);

    if (!frame) {
        printf("[ERROR] Memory allocation failed\n");
        return -1;
    }

    int index = 0;
    static int Ns = 0;
    unsigned char bcc2 = 0;
    int retries = 0;
    int result;
    enum StateRCV ackState = START;

    frame[index++] = FLAG;
    frame[index++] = ADDR_TX_COMMAND;
    frame[index++] = (Ns == 0) ? 0x00 : 0x80;
    frame[index++] = BCC1(frame[1], frame[2]);

    for (int i = 0; i < bufSize; i++) {
        bcc2 ^= buf[i];
        if (buf[i] == FLAG) {
            frame[index++] = 0x7D;
            frame[index++] = 0x5E;
        } else if (buf[i] == 0x7D) {
            frame[index++] = 0x7D;
            frame[index++] = 0x5D;
        } else {
            frame[index++] = buf[i];
        }
    }

    if (bcc2 == FLAG) {
        frame[index++] = 0x7D;
        frame[index++] = 0x5E;
    } else if (bcc2 == 0x7D) {
        frame[index++] = 0x7D;
        frame[index++] = 0x5D;
    } else {
        frame[index++] = bcc2;
    }

    frame[index++] = FLAG;

    int backoff = 1;
    while (retries < transmissions) {
        printf("[INFO] Sending frame (Attempt %d of %d)\n", retries + 1, transmissions);

        result = writeBytesSerialPort(frame, index);
        if (result < 0 || result != index) {
            printf("[ERROR] Write operation failed (Attempt %d)\n", retries + 1);
            retries++;
            sleep(backoff);
            backoff *= 2;
            continue;
        }

        totalBytesSent += index;
        printf("[INFO] Frame sent, awaiting acknowledgment\n");
        alarm(timeout);
        ackState = START;
        alarmTriggered = 0;

        while (ackState != RCV_STOP && !alarmTriggered) {
            unsigned char ackByte;
            int readResult = readByteSerialPort(&ackByte);
            if (readResult <= 0) {
                if (alarmTriggered) {
                    retries++;
                    if (retries >= transmissions) {
                        printf("[ERROR] Retries exhausted. No acknowledgment received.\n");
                        free(frame);
                        return ERR_WRITE_TIMEOUT;
                    }
                    printf("[WARN] Timeout waiting for acknowledgment. Retrying...\n");
                    break;
                }
                continue;
            }

            switch (ackState) {
                case START:
                    if (ackByte == FLAG) ackState = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (ackByte == ADDR_RX_COMMAND) ackState = A_RCV;
                    else if (ackByte != FLAG) ackState = START;
                    break;
                case A_RCV:
                    if (ackByte == CTRL_RR0 || ackByte == CTRL_RR1) {
                        int Nr = (ackByte == CTRL_RR0) ? 0 : 1;
                        if (Nr == Ns) {
                            Ns = (Ns + 1) % 2;
                            free(frame);
                            return bufSize;
                        }
                    } else if (ackByte == CTRL_REJ0 || ackByte == CTRL_REJ1) {
                        printf("[WARN] REJ received, resending frame\n");
                        ackState = START;
                        break;
                    } else if (ackByte == FLAG) {
                        ackState = FLAG_RCV;
                    } else {
                        ackState = START;
                    }
                    break;
                default:
                    ackState = START;
                    break;
            }
        }
    }

    printf("[ERROR] Transmission failed after maximum retries\n");
    free(frame);
    return ERR_MAX_RETRIES_EXCEEDED;
}

void sendRR() {
    unsigned char rrFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_RR0, 0x00, FLAG};
    rrFrame[3] = BCC1(rrFrame[1], rrFrame[2]);
    writeBytesSerialPort(rrFrame, sizeof(rrFrame));
}

void sendREJ() {
    unsigned char rejFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_REJ0, 0x00, FLAG};
    rejFrame[3] = BCC1(rejFrame[1], rejFrame[2]);
    writeBytesSerialPort(rejFrame, sizeof(rejFrame));
}

int llread(unsigned char *packet) {
    enum StateRCV state = START;
    unsigned char byte;
    unsigned char frame[256];
    int frameIndex = 0;
    unsigned char bcc2 = 0;
    int bytesRead = 0;
    int retries = 0;
    int success = 0;


    while (state != RCV_STOP && retries < transmissions) {
        int res = readByteSerialPort(&byte);

        if (res <= 0) {
            if (++retries < transmissions) {
                printf("[WARN] Read timeout or failure (retry %d of %d)\n", retries, transmissions);
                sleep(timeout);
                continue;
            } else {
                printf("[ERROR] Maximum retries exceeded. Read operation failed.\n");
                return ERR_MAX_RETRIES_EXCEEDED;
            }
        }

        if (byte != FLAG && byte < 0x01 && byte > 0x7F) {
            printf("[WARN] Ignored invalid byte: %02x\n", byte);
            continue;
        }

        switch (state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    frameIndex = 0;
                    bytesRead = 0;
                    bcc2 = 0;
                }
                break;

            case FLAG_RCV:
                if (byte == ADDR_TX_COMMAND) {
                    frame[frameIndex++] = byte;
                    state = A_RCV;
                } else if (byte != FLAG) {
                    state = START;
                }
                break;

            case A_RCV:
                if (byte == 0x00 || byte == 0x80) {
                    frame[frameIndex++] = byte;
                    state = C_RCV;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;

            case C_RCV:
                if (byte == BCC1(frame[frameIndex - 2], frame[frameIndex - 1])) {
                    frame[frameIndex++] = byte;
                    state = BCC_OK;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;

            case BCC_OK:
                if (byte == FLAG) {
                    if (bcc2 == 0) {
                        state = RCV_STOP;
                        sendRR();
                        printf("[INFO] Frame successfully received with sequence %d\n", expectedSequence);
                        expectedSequence = (expectedSequence + 1) % 2;
                        totalBytesReceived += bytesRead;
                        success = 1;
                    } else {
                        printf("[ERROR] BCC2 mismatch, frame rejected\n");
                        sendREJ();
                        state = START;
                    }
                } else {
                    if (byte == 0x7D) {
                        int stuffedRes = readByteSerialPort(&byte);
                        if (stuffedRes > 0) {
                            if (byte == 0x5E) byte = 0x7E;
                            if (byte == 0x5D) byte = 0x7D;
                        }
                    }
                    packet[bytesRead++] = byte;
                    bcc2 ^= byte;
                }
                break;

            default:
                state = START; 
                break;
        }
    }

    if (!success) {
        printf("[ERROR] Frame read failed after %d retries\n", retries);
        return ERR_READ_TIMEOUT;
    }

    printf("[INFO] Received %d bytes in the frame\n", bytesRead);
    return bytesRead;
}


void sendDISC() {
    unsigned char discFrame[5];
    discFrame[0] = FLAG; 
    discFrame[1] = ADDR_RX_COMMAND; 
    discFrame[2] = CTRL_DISC; 
    discFrame[3] = BCC1(discFrame[1], discFrame[2]);
    discFrame[4] = FLAG; 
    writeBytesSerialPort(discFrame, sizeof(discFrame));
}

enum StateRCV processStateForDISC(enum StateRCV state, unsigned char byte) {
    switch (state) {
        case START:
            return (byte == FLAG) ? FLAG_RCV : START;
        case FLAG_RCV:
            return (byte == ADDR_RX_COMMAND) ? A_RCV : (byte == FLAG ? FLAG_RCV : START);
        case A_RCV:
            return (byte == CTRL_DISC) ? C_RCV : (byte == FLAG ? FLAG_RCV : START);
        case C_RCV:
            return (byte == BCC1(ADDR_RX_COMMAND, CTRL_DISC)) ? BCC_OK : (byte == FLAG ? FLAG_RCV : START);
        case BCC_OK:
            return (byte == FLAG) ? RCV_STOP : START;
        default:
            return START;
    }
}

enum StateRCV processStateForUA(enum StateRCV state, unsigned char byte) {
    switch (state) {
        case START:
            return (byte == FLAG) ? FLAG_RCV : START;
        case FLAG_RCV:
            return (byte == ADDR_TX_COMMAND) ? A_RCV : (byte == FLAG ? FLAG_RCV : START);
        case A_RCV:
            return (byte == CTRL_UA) ? C_RCV : (byte == FLAG ? FLAG_RCV : START);
        case C_RCV:
            return (byte == BCC1(ADDR_TX_COMMAND, CTRL_UA)) ? BCC_OK : (byte == FLAG ? FLAG_RCV : START);
        case BCC_OK:
            return (byte == FLAG) ? RCV_STOP : START;
        default:
            return START;
    }
}

int llclose(int showStatistics) {
    unsigned char byte;
    enum StateRCV state = START;
    int retries = 0;
    int clstat = 0;

    if (role == LlTx) {
        while (retries < transmissions) {
            sendDISC();
            printf("[INFO] DISC packet from transmitter sent.\n");
            printf("[INFO] Waiting for DISC from receiver...\n");

            alarmTriggered = 0;
            alarm(timeout);

            while (state != RCV_STOP && !alarmTriggered) {
                int res = readByteSerialPort(&byte);
                if (res <= 0) {
                    if (alarmTriggered) {
                        retries++;
                        printf("[ERROR] Timeout while waiting for DISC frame\n");
                        if (retries >= transmissions) {
                            printf("[ERROR] Maximum retries exceeded while waiting for DISC frame\n");
                            resetPortState();
                            goto cleanup;
                        }
                        printf("[WARN] Retrying sending DISC...\n");
                        alarm(timeout);
                        break;
                    }
                    continue;
                }

                state = processStateForDISC(state, byte);
            }

            if (state == RCV_STOP) break;
        }

        // Send UA frame to confirm closure
        unsigned char uaFrame[5] = {FLAG, ADDR_TX_COMMAND, CTRL_UA, BCC1(ADDR_TX_COMMAND, CTRL_UA), FLAG};
        if (writeBytesSerialPort(uaFrame, sizeof(uaFrame)) < 0) {
            printf("[ERROR] Failed to send UA frame\n");
            resetPortState();
            goto cleanup;
        }
        printf("[INFO] Sent UA frame to confirm connection closure\n");

    } else if (role == LlRx) {
        alarmTriggered = 0;
        alarm(timeout);

        while (state != RCV_STOP && !alarmTriggered) {
            int res = readByteSerialPort(&byte);
            if (res <= 0) {
                if (alarmTriggered) {
                    printf("[ERROR] Timeout while waiting for DISC frame\n");
                    resetPortState();
                    goto cleanup;
                }
                continue;
            }

            state = processStateForDISC(state, byte);
        }
        sendDISC();
        printf("[INFO] DISC packet from receiver sent.\n");

        state = START;
        alarmTriggered = 0;
        alarm(timeout);

        while (state != RCV_STOP && !alarmTriggered) {
            int res = readByteSerialPort(&byte);
            if (res <= 0) {
                if (alarmTriggered) {
                    printf("[ERROR] Timeout while waiting for UA frame\n");
                    resetPortState();
                    goto cleanup;
                }
                continue;
            }

            state = processStateForUA(state, byte);
        }

        if (state == RCV_STOP) {
            printf("[INFO] UA frame received, connection closure confirmed.\n");
        }
    }

cleanup:
    flushPort();

    clstat = closeSerialPort();
    if (clstat == 0) {
        printf("[INFO] Connection closed.\n");
    } else {
        printf("[ERROR] Failed to close the serial port properly.\n");
    }

    if (showStatistics) {
        printf("[INFO] Statistics:\n");
    }

    return clstat;
}
