// mbed
#include "mbed.h"
// RPC
#include "mbed_rpc.h"
// WIFI_MQTT
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
// uLCD
#include "uLCD_4DGL.h"
// Accelerometer
#include "stm32l475e_iot01_accelero.h"

using namespace std::chrono;

uLCD_4DGL uLCD(D1, D0, D2); // serial tx, serial rx, reset pin;

DigitalOut led1(LED1);
DigitalOut led2(LED2);
InterruptIn btn(USER_BUTTON);

Thread G_UI;  // gesture UI
Thread T_A_D; // tilt angle detection

void getAcc(Arguments *in, Reply *out);
BufferedSerial pc(USBTX, USBRX);
RPCFunction rpcAcc(&getAcc, "getAcc");

int main()
{
    BSP_ACCELERO_Init();
    char buf[256], outbuf[256];
    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    while (true)
    {
        memset(buf, 0, 256); // clear buffer
        for (int i = 0; i < 255; i++)
        {
            char recv = fgetc(devin);
            if (recv == '\r' || recv == '\n')
            {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
    }
}

void getAcc(Arguments *in, Reply *out)
{
    int16_t pDataXYZ[3] = {0};
    char buffer[200];
    BSP_ACCELERO_AccGetXYZ(pDataXYZ);
    sprintf(buffer, "Accelerometer values: (%d, %d, %d)", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
    out->putData(buffer);
}