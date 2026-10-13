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

static const int NUM_SAMPLES = 480;

// Server socket
static int server;

static int servo_moved = 0;

// Temporary file to log servo reading data in
static FILE *servo_log;

// Interrupt handler for SIGINT
void int_handler(int signum)
{
    printf("Handling SIGINT...\n");
    fclose(servo_log);
    close(server);

    _exit(0);
}

static int convert_voltage_to_angle(int x)
{
    return ((x - 142) * (90 - (-90)) / (3124-142)) + (-90);
}

void *stream_servo_position(void *arg)
{
    struct timespec last_time, current_time;
    int num = 0;
    uint16_t servo_readings[512];
    uint16_t samples[32];
    int samp_top = 0;

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    while(1)
    {
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

        for(int i = 0; i < 24; i++)
        {
            uint16_t avg = 0;

            for(int j = 0; j < 20; j++)
            {
                avg += servo_readings[i * 20 + j];
            }

            if(avg == 0)
            {
                servo_moved = 1;
                // fprintf(servo_log, "INTERRUPT RAISED!!!\n");
            }

            samples[samp_top++] = avg / 20;
        }

        for(int i = 0; i < 12; i++)
        {
            fprintf(servo_log, "Servo1: (%u , %d) Servo2: (%u , %d)\n", samples[i], convert_voltage_to_angle(samples[i]), samples[i + 12], convert_voltage_to_angle(samples[i + 12]));
        } 

        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed = (current_time.tv_sec - last_time.tv_sec) + 
                         (current_time.tv_nsec - last_time.tv_nsec) / 1000000000.0;
    
        // fprintf(servo_log, "servo_read: Iteration %d took: %.6f seconds\n", num, elapsed);
        num++;

        last_time = current_time;
        fflush(servo_log);
    }

    // This shouldn't return
    return NULL;
}

void *interactive_client(void *arg)
{
    printf("0 -> Exit Client; 1 -> Set servo angles; 2 -> Set end-effector position(TODO)\n");

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
                int16_t to_send[4];
                
                printf("Enter position for Servo1: ");
                scanf("%d", &temp);
                to_send[1] = (int16_t)(temp);

                printf("Enter position for Servo2: ");
                scanf("%d", &temp);
                to_send[2] = (int16_t)(temp);

                // opcode. specified twice as this is 16-bit data
                to_send[0] = 0x0101;

                printf("Sending data\n");
                // printf("Sending data: %d %d\n", to_send[1], to_send[2]);

                servo_moved = 0;
               
                if(write(server, to_send, sizeof(int16_t) * 3) == -1)
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
