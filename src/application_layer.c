// application_layer.c
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern int fd;

unsigned char *getControlPacket(const unsigned int c, const char *filename, long int length, unsigned int *size);
unsigned char *getData(FILE *file, long int fileSize);
unsigned char *getDataPacket(unsigned char list, unsigned char *data, int dataSize, int *packetSize);
unsigned char *getSizeAndName(unsigned char *packet, int size, unsigned long int *fileSize);

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);

    if (strcmp("tx", role) == 0)
        connectionParameters.role = LlTx;
    else if (strcmp("rx", role) == 0)
        connectionParameters.role = LlRx;
    else {
        perror("Invalid role\n");
        exit(-1);
    }

    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (llopen(connectionParameters) != 0) {
        perror("Connection Error\n");
        exit(-1);
    }

    switch (connectionParameters.role) {
        case LlTx: {
            FILE *file = fopen(filename, "rb");
            printf("filename: %s;\n", filename);
            if (file == NULL) {
                perror("File not found\n");
                exit(-1);
            }

            int prev = ftell(file);
            fseek(file, 0L, SEEK_END);
            long int fileSize = ftell(file) - prev;
            fseek(file, prev, SEEK_SET);

            unsigned int cpSize;
            unsigned char *controlPacketStart = getControlPacket(1, filename, fileSize, &cpSize);
            if (llwrite(controlPacketStart, cpSize) == -1)
                exit(-1);

            unsigned char list = 0;
            unsigned char *cont = getData(file, fileSize);
            long int remain = fileSize;

            while (remain > 0) {
                int dataSize = remain > (long int)MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : remain;
                unsigned char *data = (unsigned char *)malloc(dataSize);
                memcpy(data, cont, dataSize);

                int packetSize;
                unsigned char *packet = getDataPacket(list, data, dataSize, &packetSize);

                if (llwrite(packet, packetSize) == -1)
                    exit(-1);

                remain -= (long int)MAX_PAYLOAD_SIZE;
                cont += dataSize;
                list = (list + 1) % 255;

                free(packet);
                free(data);
            }

            unsigned char *controlPacketEnd = getControlPacket(3, filename, fileSize, &cpSize);
            if (llwrite(controlPacketEnd, cpSize) == -1)
                exit(-1);

            llclose(connectionParameters);
            break;
        }

        case LlRx: {
            unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
            int packetSize = -1;

            while ((packetSize = llread(packet)) < 0);

            unsigned long int fileSize = 0;
            //unsigned char *name = getSizeAndName(packet, packetSize, &fileSize);
            char name[] = "penguin-received.gif";
            FILE *newFile = fopen((char *)name, "wb");
            printf("FILENAME: %s\n.", name);

            unsigned long int receivedBytes = 0;
            unsigned char list = 0;

            while (receivedBytes < fileSize) {
                while ((packetSize = llread(packet)) < 0);

                if (packet[0] == 3)
                    break;
                else if (packet[0] == 2) {
                    unsigned int dataSize = ((unsigned int)packet[2] << 8) | packet[3];
                    fwrite(packet + 4, sizeof(unsigned char), dataSize, newFile);
                    receivedBytes += dataSize;
                    list = (list + 1) % 255;
                } else
                    fprintf(stderr, "Invalid packet type received: %d\n", packet[0]);
            }

            fclose(newFile);
            free(packet);
            break;
        }

        default:
            exit(-1);
            break;
    }

    printf("Done!\n");
}

unsigned char *getControlPacket(const unsigned int c, const char *filename, long int length, unsigned int *size)
{
    int L1 = 0;
    long int temp = length;
    while (temp > 0) {
        L1++;
        temp >>= 8;
    }

    const int L2 = strlen(filename);
    *size = 1 + 2 + L1 + 2 + L2;

    unsigned char *packet = (unsigned char *)malloc(*size);
    unsigned int pos = 0;

    packet[pos++] = c;
    packet[pos++] = 0;
    packet[pos++] = L1;

    for (unsigned int i = 0; i < L1; i++) {
        packet[2 + L1 - i] = length & 0xFF;
        length >>= 8;
    }

    pos += L1;
    packet[pos++] = 1;
    packet[pos++] = L2;
    memcpy(packet + pos, filename, L2);

    return packet;
}

unsigned char *getData(FILE *file, long int fileSize)
{
    unsigned char *cont = (unsigned char *)malloc(sizeof(unsigned char) * fileSize);
    fread(cont, sizeof(unsigned char), fileSize, file);
    return cont;
}

unsigned char *getDataPacket(unsigned char list, unsigned char *data, int dataSize, int *packetSize)
{
    *packetSize = 1 + 1 + 2 + dataSize;
    unsigned char *packet = (unsigned char *)malloc(*packetSize);

    packet[0] = 2;
    packet[1] = list;
    packet[2] = (dataSize >> 8) & 0xFF;
    packet[3] = dataSize & 0xFF;
    memcpy(packet + 4, data, dataSize);

    return packet;
}

unsigned char *getSizeAndName(unsigned char *packet, int size, unsigned long int *fileSize)
{
    *fileSize = 0;
    unsigned char fileSizeBytes = packet[2];

    for (unsigned int i = 0; i < fileSizeBytes; i++)
        *fileSize |= ((unsigned long int)packet[3 + i] << (8 * (fileSizeBytes - i - 1)));

    unsigned char fileNameBytes = packet[3 + fileSizeBytes + 1];
    unsigned char *name = (unsigned char *)malloc(fileNameBytes + 1);

    memcpy(name, packet + 3 + fileSizeBytes + 2, fileNameBytes);
    name[fileNameBytes] = '\0';

    return name;
}
