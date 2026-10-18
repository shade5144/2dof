#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>

#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

#include "raylib.h"


static const int NUM_SAMPLES = 96;

// Server socket
static int server;

static int servo_moved = 0;

// Temporary file to log servo reading data in
static FILE *servo_log;

#define FILL_TO_SEND(to_send, angle1, angle2) to_send[1]=angle1; to_send[2]=angle2;
static int16_t to_send[4] = {0};

// Indicates whether we are calibrating currently or not
static bool cali_on = false;
static bool cali_done = false;
static bool cali_switch = true; // Set to false to trigger one-time setting of cali_on
static int servo_cali1[8] = {0};
static int servo_cali2[8] = {0};
static int16_t cali_angle1 = 0;
static int16_t cali_angle2 = 0;
static int cali_index = 0;

static void send_angles(int16_t angle1, int16_t angle2)
{
    if(cali_on)
    {
        cali_angle1 = angle1;
        cali_angle2 = angle2;
    }

    FILL_TO_SEND(to_send, angle1, angle2);
    printf("Sending data\n");
    // printf("Sending data: %d %d\n", to_send[1], to_send[2]);

    servo_moved = 0;
    cali_done = false;
   
    if(write(server, to_send, sizeof(int16_t) * 3) == -1)
    {
        perror("calibrate_servos-write1");
        return;
    }

    printf("Moving servos to: %d %d....\n", angle1, angle2);

    while(!servo_moved)
        ;
    
    printf("Servos moved!....\n");

    if(!cali_switch)
    {
        cali_switch = true;
        cali_on = true;
    }

    usleep(1000);

    if(cali_on)
    {
        while(!cali_done)
            ;
    }

    printf("Calibration done!....\n");
}

#define DO_CALI(angle1, angle2) servo_cali1[cali_index] = 0; servo_cali2[cali_index] = 0; send_angles(angle1, angle2); cali_index++;

static void calibrate_servos()
{
    to_send[0] = 0x0101;

    /*
    for(int16_t i = -90; i <= 90; i++)
    {
        DO_CALI(i, i);
    }
    */
    DO_CALI(-90, -90);
    DO_CALI(-60, -60);
    DO_CALI(-30, -30);
    DO_CALI(0, 0);
    DO_CALI(30, 30);
    DO_CALI(60, 60);
    DO_CALI(90, 90);

    cali_index = 0;
}

// Interrupt handler for SIGINT
void int_handler(int signum)
{
    printf("Handling SIGINT...\n");
    fclose(servo_log);
   
    struct linger sl;
    sl.l_onoff = 1; 
    sl.l_linger = 0;
    setsockopt(server, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl));

    close(server);

    _exit(0);
}

static int convert_voltage_to_angle_servo1(int x)
{
    return ((x - 384) * (90 - (-90)) / (3094-384)) + (-90);
}

static int convert_voltage_to_angle_servo2(int x)
{
    return ((x - 292) * (90 - (-90)) / (2924-292)) + (-90);
}

void *stream_servo_position(void *arg)
{
    float avg1 = 0;
    float avg2 = 0;
    struct timespec last_time, current_time;
    int num = 1;
    uint16_t servo_readings[512];
    uint16_t samples[32];
    int samp_top = 0;

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    // Run calibration for a single iteration
    bool cali_run = false;
    bool cali_do = false;

    while(1)
    {
        cali_run = false;

        int read_bytes = read(server, servo_readings, sizeof(uint16_t) * NUM_SAMPLES);

        if(read_bytes == -1)
        {
            perror("stream_servo_position-read1");
            break;
        }
        else if(read_bytes != sizeof(uint16_t) * NUM_SAMPLES)
        {
            printf("Bytes received: %d, Bytes expected: %d\n", read_bytes, (int)sizeof(uint16_t) * NUM_SAMPLES);
            break;
        }

        samp_top = 0;

        /*
        for(int i = 0; i < 480 ; i++)
        {
            printf("%u\n", servo_readings[i]);
        }
        */

        int batch_size = NUM_SAMPLES / 24;

        for(int i = 0; i < 24; i++)
        {
            uint16_t avg = 0;

            for(int j = 0; j < batch_size; j++)
            {
                avg += servo_readings[i * batch_size + j];
            }

            if(avg == 0)
            {
                servo_moved = 1;

                if(cali_on)
                {
                    cali_run = true;
                }

                // fprintf(servo_log, "INTERRUPT RAISED!!!\n");`
                break;
            }
            
            samples[samp_top++] = avg / batch_size;
        }

        if(cali_run)
        {
            cali_do = true;
            usleep(1000); // Wait for servo position to be available via BT. Might not need this
            continue;
        }

        // So that the zero values caused by the interrupt don't glitch out the position to be displayed
        // if(servo_moved && !cali_done)
        //     break;

        unsigned long cur_num = NUM_SAMPLES * num;

        int a1 = 0;
        int a2 = 0;
        avg1 = 0.0f;
        avg2 = 0.0f;

        for(int i = 0; i < 12; i++)
        {
            int a1 = convert_voltage_to_angle_servo1(samples[i]);
            int a2 = convert_voltage_to_angle_servo2(samples[i + 12]);

            avg1 += (float)samples[i];
            avg2 += (float)samples[i + 12];

            if(cali_on && cali_do)
            {
                servo_cali1[cali_index] += a1 - cali_angle1;
                servo_cali2[cali_index] += a2 - cali_angle2;
            }

            fprintf(servo_log, "Servo1: (%u %d) Servo2: (%u %d)\n", samples[i], a1, samples[i + 12], a2);
        }

        avg1 /= 12;
        avg2 /= 12;
        fprintf(servo_log, "Cur avg->S1: (%f %d) S2: (%f %d)\n", avg1, convert_voltage_to_angle_servo1((int)avg1), avg2, convert_voltage_to_angle_servo2((int)avg2));

        if(cali_on && cali_do)
        {
            servo_cali1[cali_index] /= 12;
            servo_cali2[cali_index] /= 12;

            fprintf(servo_log, "Calibrating Servo1: %d Servo2: %d\n", convert_voltage_to_angle_servo1(servo_cali1[cali_index]), convert_voltage_to_angle_servo2(servo_cali2[cali_index]));
            cali_done = true;
        }

        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed = (current_time.tv_sec - last_time.tv_sec) + 
                         (current_time.tv_nsec - last_time.tv_nsec) / 1000000000.0;
    
        // fprintf(servo_log, "servo_read: Iteration %d took: %.6f seconds\n", num, elapsed);
        num++;

        cali_do = false;
        last_time = current_time;
        fflush(servo_log);
    }

    // This shouldn't return
    return NULL;
}

void *interactive_client(void *arg)
{
    printf("0 -> Exit Client; 1 -> Set servo angles; 2 -> Set end-effector position(TODO) 3 -> Calibrate servos\n");

    while(1)
    {
        int option = -1;

        printf("Enter option: ");
        scanf("%d", &option);

        switch(option)
        {
            case 0:
            {
               raise(SIGINT);

               // So that we don't go back to the prompt before the signal is raised
               while(1)
                   ;

               break;
            }
            case 1:
            {
                int temp;
                int16_t to_send_angles[4];
               
                while(1)
                {
                    printf("Enter position for Servo1: ");
                    scanf("%d", &temp);

                    if(temp > 90 || temp < -90)
                    {
                        printf("Out of range, enter value within -90 and +90");
                        continue;
                    }

                    break;
                }

                to_send_angles[1] = (int16_t)(temp);

                while(1)
                {
                    printf("Enter position for Servo2: ");
                    scanf("%d", &temp);

                    if(temp > 90 || temp < -90)
                    {
                        printf("Out of range, enter value within -90 and +90");
                        continue;
                    }

                    break;
                }

                to_send_angles[2] = (int16_t)(temp);

                // opcode. specified twice as this is 16-bit data
                to_send_angles[0] = 0x0101;

                printf("Sending data\n");
                // printf("Sending data: %d %d\n", to_send[1], to_send[2]);

                servo_moved = 0;
               
                if(write(server, to_send_angles, sizeof(int16_t) * 3) == -1)
                {
                    perror("interactive_client-write1");
                    break;
                }

                printf("Moving servos....\n");

                while(!servo_moved)
                    usleep(1000);
                
                printf("Servos moved!....\n");

                break;
            }
            case 2:
            {
                // TODO
                break;
            }
            case 3:
            {
                cali_switch = false;
                calibrate_servos();
                cali_on = false;
                break;
            }
            default:
            {
                printf("Invalid option, try again!\n");
            }
        }
    }

    return NULL;
}

void *render_arm(void *arg)
{
}


int main()
{
    // Handle SIGINT by closing the socket to ensure that stale connections are removed
    struct sigaction new_action, old_action;

    new_action.sa_handler = int_handler;
    sigemptyset(&new_action.sa_mask);
    new_action.sa_flags = 0;

    sigaction(SIGINT, NULL, &old_action);

    if(old_action.sa_handler != SIG_IGN)
    {
        sigaction(SIGINT, &new_action, NULL);
    }

    struct sockaddr_rc addr = {0};
    char dest[18] = "68:09:47:76:ef:62";

    server = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    if(server == -1)
    {
        perror("main-socket1");
        return 0;
    }

    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba(dest, &addr.rc_bdaddr);

    if(connect(server, (struct sockaddr *)&addr, sizeof(addr)))
    {
        perror("main-connect1");
        close(server);
        return 0;
    }

    servo_log = fopen("servo_reading_log", "w");

    if(!servo_log)
    {
        perror("main-fopen1");
        return 0;
    }

    // Stream servo position, render the arm and send commands to the ESP32
    pthread_t threads[3];

    if(pthread_create(&threads[0], NULL, stream_servo_position, NULL))
    {
        perror("main-pthread_create1");
        return 0;
    }

    if(pthread_create(&threads[1], NULL, interactive_client, NULL))
    {
        perror("main-pthread_create2");
        return 0;
    }

    for(int i = 0; i < 2; i++)
    {
        if(pthread_join(threads[i], NULL))
        {
            perror("main-pthread_join");
        }
    }
    // This should be started only after stream_servo_data has given us actual info
    // pthread_create(&threads[2], NULL, render_arm, NULL);

    return 0;
}
