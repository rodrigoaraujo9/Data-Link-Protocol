#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/signal.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
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

double HEADER_ERR_PROB = 0.1;
double DATA_ERR_PROB = 0.1;
int PROP_DELAY_MS = 0;
volatile int alarmTriggered = FALSE;
int alarmCount = 0;
int timeout;
LinkLayerRole role;
int baudRate;
int transmissions;

void handle_alarm(int sig) {
    alarmTriggered = 1;
    alarmCount++;
    printf("[ALARM] Timeout occurred. Alarm count: %d seconds.\n", alarmCount);
    alarm(1);
}

void introduceErrors(unsigned char *frame, int frameSize, double headerErrorProb, double dataErrorProb) {
    srand(time(NULL));
    if ((double)rand() / RAND_MAX < headerErrorProb) {
        frame[1] ^= 0xFF;
        printf("[INFO] Header error introduced\n");
    }
    for (int i = 4; i < frameSize - 2; i++) {
        if ((double)rand() / RAND_MAX < dataErrorProb) {
            frame[i] ^= 0xFF;
            printf("[INFO] Data error introduced at byte %d\n", i);
        }
    }
}

void applyPropagationDelay() {
    if (PROP_DELAY_MS > 0) {
        usleep(PROP_DELAY_MS * 1000);
        printf("[INFO] Applied propagation delay of %d ms\n", PROP_DELAY_MS);
    }
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

    if (connectionParams.role == LlTx) {
        unsigned char setFrame[5] = {FLAG, ADDR_TX_COMMAND, CTRL_SET, BCC1(ADDR_TX_COMMAND, CTRL_SET), FLAG};
        int retry = 0;

        while (retry < connectionParams.nRetransmissions) {
            if (writeBytesSerialPort(setFrame, sizeof(setFrame)) < 0) {
                printf("[ERROR] Failed to send SET frame.\n");
                return ERR_WRITE_FAILED;
            }
            printf("[INFO] Sent SET frame, waiting for UA...\n");
            alarmTriggered = 0;
            alarm(connectionParams.timeout);
            unsigned char byte;
            enum StateRCV state = START;
            int bytesRead = 0;

            while (!alarmTriggered && state != RCV_STOP) {
                int res = readByteSerialPort(&byte);
                if (res <= 0 && alarmTriggered) {
                    retry++;
                    if (retry >= connectionParams.nRetransmissions) {
                        printf("[ERROR] Maximum retries reached.\n");
                        alarm(0);
                        return ERR_MAX_RETRIES_EXCEEDED;
                    }
                    break;
                }

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

            if (state == RCV_STOP) {
                printf("[INFO] UA frame received, connection established.\n");
                alarm(0);
                return 1;
            }
        }
    } else if (connectionParams.role == LlRx) {
        alarmTriggered = 0;
        alarm(connectionParams.timeout);
        enum StateRCV state = START;
        unsigned char byte;

        while (state != RCV_STOP) {
            int res = readByteSerialPort(&byte);
            if (res <= 0) {
                if (alarmTriggered) {
                    printf("[ERROR] Timeout while waiting for SET frame.\n");
                    return ERR_READ_TIMEOUT;
                }
                continue;
            }

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

        unsigned char uaFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_UA, BCC1(ADDR_RX_COMMAND, CTRL_UA), FLAG};
        if (writeBytesSerialPort(uaFrame, sizeof(uaFrame)) < 0) {
            printf("[ERROR] Failed to send UA frame.\n");
            return ERR_WRITE_FAILED;
        }

        printf("[INFO] UA frame sent.\n");
    }

    return 1;
}

int llwrite(const unsigned char *buf, int bufSize) {
    int overhead = 6;
    int maxFrameSize = bufSize * 2 + overhead;
    unsigned char *frame = (unsigned char*)malloc(maxFrameSize);
    
    if (frame == NULL) {
        printf("[ERROR] Memory allocation failed\n");
        return -1;
    }

    int index = 0;
    static int Ns = 0;
    unsigned char bcc2 = 0;
    int retries = 0;
    int result = 0;
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
    
    while (retries < transmissions) {
        printf("[INFO] Attempting to send frame (Attempt %d of %d)\n", retries + 1, transmissions);
        
        result = writeBytesSerialPort(frame, index);
        if (result < 0 || result != index) {
            printf("[ERROR] Failed to write full frame to serial port. Attempt %d\n", retries + 1);
            retries++;
            continue;
        }

        totalBytesSent += index;
        printf("[DEBUG] Frame sent successfully, total bytes sent so far: %d\n", totalBytesSent);

        printf("[DEBUG] Frame sent successfully, awaiting acknowledgment...\n");
        alarm(1);
        alarmTriggered = 0;

        while (ackState != RCV_STOP && alarmTriggered == 0) {
            unsigned char ackByte;
            int readResult = readByteSerialPort(&ackByte);

            if (readResult <= 0) {
                if (alarmTriggered) {
                    retries++;
                    if (retries >= transmissions) {
                        printf("[ERROR] Maximum retries exceeded while waiting for RR\n");
                        free(frame);
                        return ERR_MAX_RETRIES_EXCEEDED;
                    }
                    printf("[WARN] Timeout waiting for RR, retrying frame transmission\n");
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
                            ackState = RCV_STOP;
                            printf("[INFO] Acknowledgment (RR) received, transmission confirmed\n");
                            free(frame);
                            Ns = (Ns + 1) % 2;
                            return bufSize;
                        }
                    } else if (ackByte == CTRL_REJ0 || ackByte == CTRL_REJ1) {
                        printf("[WARN] REJ received, resending frame without toggling sequence\n");
                        ackState = START;
                        break;
                    } else if (ackByte == FLAG) {
                        ackState = FLAG_RCV;
                    } else {
                        ackState = START;
                    }
                    break;
                default:
                    break;
            }
        }

        if (ackState == RCV_STOP) break;

        if (retries >= transmissions) {
            printf("[ERROR] Failed to get acknowledgment after maximum retries\n");
            free(frame);
            return ERR_MAX_RETRIES_EXCEEDED;
        }
    }

    printf("[ERROR] Transmission failed after retries\n");
    free(frame);
    return ERR_WRITE_TIMEOUT;
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
    static int expectedSequence = 0;

    printf("[DEBUG] Starting llread\n");

    while (state != RCV_STOP && retries < transmissions) {
        int res = readByteSerialPort(&byte);

        if (res <= 0) {
            printf("[ERROR] Read timeout (retry %d of %d)\n", retries + 1, transmissions);
            retries++;
            sleep(timeout);
            continue;
        }

        retries = 0;

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
                        applyPropagationDelay();
                        introduceErrors(frame, frameIndex, HEADER_ERR_PROB, DATA_ERR_PROB);
                        if (expectedSequence == ((frame[2] & 0x80) ? 1 : 0)) {
                            state = RCV_STOP;
                            sendRR();
                            printf("[INFO] Frame successfully received\n");
                            expectedSequence = (expectedSequence + 1) % 2;
                            totalBytesReceived += bytesRead;
                        } else {
                            sendREJ();
                            printf("[ERROR] Unexpected sequence number. Expected: %d, Received: %d\n", expectedSequence, frame[2] & 0x80 ? 1 : 0);
                            state = START;
                        }
                    } else {
                        sendREJ();
                        printf("[ERROR] BCC2 mismatch, frame rejected\n");
                    }
                } else {
                    if (byte == 0x7D) {
                        int res = readByteSerialPort(&byte);
                        if (res > 0) {
                            if (byte == 0x5E) byte = 0x7E;
                            if (byte == 0x5D) byte = 0x7D;
                        }
                    }
                    packet[bytesRead++] = byte;
                    bcc2 ^= byte;
                }
                break;
            default:
                break;
        }
    }

    if (retries >= transmissions) {
        printf("[ERROR] Maximum retries exceeded. Failed to receive the packet.\n");
        return ERR_MAX_RETRIES_EXCEEDED;
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

int llclose(int showStatistics) {
    if (role == LlTx) {
        int retries = 0;
        while (retries < transmissions) {
            sendDISC();
            printf("[INFO] DISC packet from transmitter sent.\n");
            printf("[INFO] Waiting for DISC from receiver...\n");

            unsigned char byte;
            enum StateRCV state = START;
            alarm(1);
            while (state != RCV_STOP) {
                int res = readByteSerialPort(&byte);
                if (res <= 0) {
                    sleep(1);
                    printf("[ERROR] Timeout while waiting for DISC frame\n");
                    retries++;
                    if (retries >= transmissions) {
                        printf("[ERROR] Maximum retries exceeded while waiting for DISC frame\n");
                        return ERR_MAX_RETRIES_EXCEEDED;
                    }
                    printf("[WARN] Retrying sending DISC...\n");
                    break;
                }

                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == ADDR_RX_COMMAND) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == CTRL_DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == BCC1(ADDR_RX_COMMAND, CTRL_DISC)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) state = RCV_STOP;
                        break;
                    default:
                        break;
                }
            }

            if (state == RCV_STOP) break;
        }

        unsigned char uaFrame[5] = {FLAG, ADDR_TX_COMMAND, CTRL_UA, 0x00, FLAG};
        uaFrame[3] = BCC1(uaFrame[1], uaFrame[2]);

        if (writeBytesSerialPort(uaFrame, sizeof(uaFrame)) < 0) {
            printf("[ERROR] Failed to send UA frame\n");
            return ERR_WRITE_FAILED;
        }
        printf("[INFO] Sent UA frame to confirm connection closure\n");
    } else if (role == LlRx) {
        unsigned char byte;
        enum StateRCV state = START;

        while (state != RCV_STOP) {
            int res = readByteSerialPort(&byte);
            if (res <= 0) {
                printf("[ERROR] Timeout while waiting for DISC frame\n");
                return ERR_READ_TIMEOUT;
            }

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == ADDR_RX_COMMAND) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case A_RCV:
                    if (byte == CTRL_DISC) state = C_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == BCC1(ADDR_RX_COMMAND, CTRL_DISC)) state = BCC_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG) {
                        state = RCV_STOP;
                        sendDISC();
                        printf("[INFO] DISC packet from receiver sent.\n");
                    } else state = START;
                    break;
                default:
                    break;
            }
        }
    }

    int clstat = closeSerialPort();
    if (clstat != -1) {
        printf("[INFO] connection closed.\n");
    }

    if (showStatistics) {
        printf("[INFO] Statistics:\n");
    }
    return clstat;
}
