#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <math.h>

#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

#include "raylib.h"

#define PI_RAD 3.14159265358979323846

static bool vtwin = false;
static bool log_on = false;

#define SERVO_LOG(code) if(log_on) { code; }

static const int NUM_SAMPLES = 96;

// Angle1 -> MG90s and Angle2 -> MG995r
static int16_t angle1 = 0, angle2 = 0;
static int16_t ik_angle1 = 0, ik_angle2 = 0;
static int link1, link2;

// Server socket
static int server;

static int servo_moved = 0;

// Temporary file to log servo reading data in
static FILE *servo_log;

// Flag to indicate raylib has initialized, to make sure the interactive client's prompt is properly visible
static bool raylib_started = false;

// For inverse kinematics
static bool posn_clicked = false;
static Vector2 user_end_effector_posn = {0, 0};

// Interrupt handler for SIGINT
void int_handler(int signum)
{
    printf("Handling SIGINT...\n");

    if(servo_log)
    {
        fclose(servo_log);
    }
   
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

        int batch_size = NUM_SAMPLES / 24;

        for(int i = 0; i < 24; i++)
        {
            uint16_t avg = 0;

            for(int j = 0; j < batch_size; j++)
            {
                int16_t reading = servo_readings[i * batch_size + j];

                // Interrupt
                if(!reading)
                {
                    servo_moved = 1;
                    continue;
                }

                avg += reading;
            }

            if(avg == 0)
            {
                servo_moved = 1;
                // fprintf(servo_log, "INTERRUPT RAISED!!!\n");
            }

            samples[samp_top++] = avg / batch_size;
        }

        int16_t avg1 = 0;
        int16_t avg2 = 0;

        for(int i = 0; i < 12; i++)
        {
            int16_t a1 =  convert_voltage_to_angle_servo1(samples[i]);
            int16_t a2 =  convert_voltage_to_angle_servo2(samples[i + 12]);

            avg1 += a1;
            avg2 += a2;

            SERVO_LOG(
                    fprintf(servo_log, "Servo1: (%u , %d) Servo2: (%u , %d)\n", samples[i], convert_voltage_to_angle_servo1(samples[i]),  
                                                                                samples[i + 12], convert_voltage_to_angle_servo2(samples[i + 12]))
            );
        } 

        avg1 /= 12;
        avg2 /= 12;

        SERVO_LOG(fprintf(servo_log, "Current average -> (%d %d)\n", angle1, angle2));

        angle1 = avg1;
        angle2 = avg2;

        num++;

        last_time = current_time;
        fflush(servo_log);
    }

    // This shouldn't return
    return NULL;
}

void *interactive_client(void *arg)
{
    while(!raylib_started)
        ;

    printf("\n\n===================================================================================================\n");
    printf("0 -> Exit Client; 1 -> Set servo angles; 2 -> Set end-effector position(TODO) 3 -> Print current angles\n");

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
                if(!posn_clicked)
                {
                    printf("Click a position in the graphical client and try again!\n");
                    break;
                }
                
               // Inverse kinematics calculations go here 

                printf("POSN: %f %f\n", user_end_effector_posn.x, user_end_effector_posn.y);

                float x = user_end_effector_posn.x;
                float y = user_end_effector_posn.y;
                float l1 = (float)link1;
                float l2 = (float)link2;

                float q2 = acosf((x*x + y*y - l1*l1 - l2*l2) / (2*l1*l2));
                float q1 = atan2f(y, x) - atan2f((l2 * sinf(q2)), l1 + l2 * cos(q2));

                q2 = q2 * (float)180 / PI_RAD;
                q1 = q1 * (float)180 / PI_RAD;

                printf("Angle1: %f Angle2: %f\n", q2, q1);

                ik_angle1 = q2;
                ik_angle2 = q1;

                break;
            }
            case 3:
            {
                printf("Angle1: %d Angle2: %d\n", angle1, angle2);
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
    const int screenWidth = 1000;
    const int screenHeight = 525;

    InitWindow(screenWidth, screenHeight, "2dof Client");

    SetTargetFPS(60);             

    Vector2 fixed_point_coord = {screenWidth / 3, screenHeight / 2};
    Vector2 elbow_joint_coord = {screenWidth / 3 + link1, screenHeight / 2};
    Vector2 end_effector_coord = {screenWidth / 3 + link1 + link2, screenHeight / 2};
    Vector2 velbow_joint_coord = {screenWidth / 3 + link1, screenHeight / 2};
    Vector2 vend_effector_coord = {screenWidth / 3 + link1 + link2, screenHeight / 2};

    // float test_angle1 = 00.0f;
    // float test_angle2 = 00.0f;

    Vector2 mouse_posn = {0, 0};

    raylib_started = true;
    while (!WindowShouldClose())  
    {
        /*
        if (IsKeyDown(KEY_RIGHT)) test_angle1 += 1.0f;
        if (IsKeyDown(KEY_LEFT)) test_angle1 -= 1.0f;
        if (IsKeyDown(KEY_UP)) test_angle2 -= 1.0f;
        if (IsKeyDown(KEY_DOWN)) test_angle2 += 1.0f; 
        */


        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) 
        {
            posn_clicked = true;
            mouse_posn = GetMousePosition();

            user_end_effector_posn.x = mouse_posn.x - fixed_point_coord.x;
            user_end_effector_posn.y = fixed_point_coord.y - mouse_posn.y; // This is flipped, as negative direction for us is DOWN and positive is UP, but since the coordinate system
                                                                           // for windows starts from top-left, it is the opposite there
        }

        if(posn_clicked)
        {
            DrawLine(fixed_point_coord.x,fixed_point_coord.y, mouse_posn.x, mouse_posn.y, RED);
            DrawCircle(mouse_posn.x, mouse_posn.y, 5, GREEN);
        }

        // Update
        elbow_joint_coord.x = fixed_point_coord.x + link1 * cosf((float)angle2 * PI_RAD / (float)180);
        elbow_joint_coord.y = fixed_point_coord.y - link1 * sinf((float)angle2 * PI_RAD / (float)180);

        float abs_angle = (float)angle2 + (float)angle1;

        end_effector_coord.x = elbow_joint_coord.x + link2 * cosf(abs_angle * PI_RAD / (float)180);
        end_effector_coord.y = elbow_joint_coord.y - link2 * sinf(abs_angle * PI_RAD / (float)180);

        if(vtwin)
        {
            velbow_joint_coord.x = fixed_point_coord.x + link1 * cosf((float)ik_angle2 * PI_RAD / (float)180);
            velbow_joint_coord.y = fixed_point_coord.y - link1 * sinf((float)ik_angle2 * PI_RAD / (float)180);

            float vabs_angle = (float)ik_angle2 + (float)ik_angle1;

            vend_effector_coord.x = velbow_joint_coord.x + link2 * cosf(vabs_angle * PI_RAD / (float)180);
            vend_effector_coord.y = velbow_joint_coord.y - link2 * sinf(vabs_angle * PI_RAD / (float)180);
        }

        // Draw
        BeginDrawing();

        ClearBackground(RAYWHITE);

        // Links
        DrawLineEx(fixed_point_coord, elbow_joint_coord, 10.0f, YELLOW);
        DrawLineEx(elbow_joint_coord, end_effector_coord, 10.0f, YELLOW);

        if(vtwin)
        {
            DrawLineEx(fixed_point_coord, velbow_joint_coord, 10.0f, GREEN);
            DrawLineEx(velbow_joint_coord, vend_effector_coord, 10.0f, GREEN);
        }

        // Joints
        DrawCircle(fixed_point_coord.x, fixed_point_coord.y , 10, RED);

        if(vtwin)
        {
            DrawCircle(velbow_joint_coord.x, velbow_joint_coord.y, 10, BLUE);
            DrawCircle(vend_effector_coord.x, vend_effector_coord.y, 10, BLUE);
        }

        DrawCircle(elbow_joint_coord.x, elbow_joint_coord.y, 10, RED);
        DrawCircle(end_effector_coord.x, end_effector_coord.y, 10, RED);


        EndDrawing();
    }

    CloseWindow(); 

    return NULL;
}


int main(int argc, char **argv)
{
    // Rudimentary CLI
    if(argc < 4)
    {
        printf("USAGE: ./clt logging(y/n) l1_length l2_length\n");
        _exit(0);
    }

    if(argv[1][0] == 'y')
    {
        log_on = true;
    }

    if(argv[2][0] == 'y')
    {
        vtwin = true;
    }

    link1 = atoi(argv[3]);
    link2 = atoi(argv[4]);

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

    if(log_on)
    {
        servo_log = fopen("servo_reading_log", "w");

        if(!servo_log)
        {
            perror("main-fopen1");
            return 0;
        }
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

    if(pthread_create(&threads[2], NULL, render_arm, NULL))
    {
        perror("main-pthread_create3");
        return 0;
    }

    for(int i = 0; i < 3; i++)
    {
        if(pthread_join(threads[i], NULL))
        {
            perror("main-pthread_join");
        }
    }
    // This should be started only after stream_servo_data has given us actual info

    return 0;
}
