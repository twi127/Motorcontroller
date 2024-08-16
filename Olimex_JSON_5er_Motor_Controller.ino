// ################################################################################################################
//
//                              Motor Controller 1 - 5 Motors: Feeder + Print 1 + Print 2
//
// ################################################################################################################
#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h> 
#include <ArduinoJson.h>
#include "FastAccelStepper.h"

// Stepper Configuration Struct
//struct StepperConfig
//{
//    int8_t        Command = 0;      // 0=force stop 1=stop 3=run
//    int8_t        Direction = 1; // 1=Forward, -1=Reverse
//    int           Speed = 0;
//    int           Accel = 500;
//    int           Steps = 0;
//    int           rpm = 0;
//    int           steps_per_rpm = 3200;
//};

TaskHandle_t update_stripe_handle; // create task handle
TaskHandle_t status_signal_light_handle; // create task handle
TaskHandle_t update_outputs_handle; // create task handle
TaskHandle_t update_display_handle; // create task handle
TaskHandle_t run_simulation_handle; // create task handle
TaskHandle_t external_Start_handle; // create task handle
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--- Debugging --------------------
//#define ENABLE_DEBUG    //diese Zeile auskommentieren

#ifdef ENABLE_DEBUG
#define debugBegin(...)  Serial.begin(__VA_ARGS__)
#define debugPrint(...)   Serial.print(__VA_ARGS__)
#define debugPrintln(...)   Serial.println(__VA_ARGS__)
#else
#define debugBegin(...)
#define debugPrint(...)
#define debugPrintln(...)
#endif

//--- Ethernet connection ----------
IPAddress local_ip(192, 168, 178, 31);
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2 = (uint32_t)0x00000000;
static bool eth_connected = false;
WebServer server(80); // Declare the WebServer object

//--- External SIM Start -----------
#define startPin 35
bool ext_Start_execute = false;

//--- Steppers ---------------------
int updateStepperSettings = 0;

// Motor 1 on ESP32
#define dirPinStepper1 4 //14
#define stepPinStepper1 5 //13
#define steps_per_rpmStepper1 3200 //800

// Motor 2 on ESP32
#define dirPinStepper2 2 //16
#define stepPinStepper2 3 //15
#define steps_per_rpmStepper2 3200 //800

// Motor 3 on ESP32
#define dirPinStepper3 14 //4
#define stepPinStepper3 13 //5
#define steps_per_rpmStepper3 3200 //800

// Motor 4 on ESP32
#define dirPinStepper4 16 //2
#define stepPinStepper4 15 //3
#define steps_per_rpmStepper4 3200 //800

// Motor 5 on ESP32
#define dirPinStepper5 33 //2
#define stepPinStepper5 32 //3
#define steps_per_rpmStepper5 3200 //800

// Init Lib
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper1 = NULL;
FastAccelStepper* stepper2 = NULL;
FastAccelStepper* stepper3 = NULL;
FastAccelStepper* stepper4 = NULL;
FastAccelStepper* stepper5 = NULL;

struct StepperConfig
{
    int8_t        Command;      // 0=force stop 1=stop 3=run
    int8_t        Direction; // 1=Forward, -1=Reverse
    int           Speed;
    int           Steps;
    int           rpm;
    int           Accel;
    int           steps_per_rpm;
};

StepperConfig stepper1Config;
StepperConfig stepper2Config;
StepperConfig stepper3Config;
StepperConfig stepper4Config;
StepperConfig stepper5Config;

//--- Json Definition --------------
const int capacity = JSON_OBJECT_SIZE(7) + 4 * JSON_OBJECT_SIZE(5) + 1 * JSON_OBJECT_SIZE(8) + 1 * JSON_OBJECT_SIZE(5);
StaticJsonDocument<capacity> jsonDocument;
char buffer[250];

//--- JSON Befehlsliste ------------
//Motor 1
int8_t        M1_Command = 0;      // 0=force stop 1=stop 3=run
int8_t        M1_Direction = 1; // 1=Forward, -1=Reverse
int           M1_Speed = 0;
int           M1_Accel = 500;
int           M1_Steps = 0;
int           motor1_rpm = 0;
//Motor 2
int8_t        M2_Command = 0;
int8_t        M2_Direction = 1;
int           M2_Speed = 0;
int           M2_Accel = 500;
int           M2_Steps = 0;
int           motor2_rpm = 0;
//Motor 3
int8_t        M3_Command = 0;
int8_t        M3_Direction = 1;
int           M3_Speed = 0;
int           M3_Accel = 500;
int           M3_Steps = 0;
int           motor3_rpm = 0;
//Motor 4
int8_t        M4_Command = 0;
int8_t        M4_Direction = 1;
int           M4_Speed = 0;
int           M4_Accel = 500;
int           M4_Steps = 0;
int           motor4_rpm = 0;
//Motor 5
int8_t        M5_Command = 0;
int8_t        M5_Direction = 1;
int           M5_Speed = 0;
int           M5_Accel = 500;
int           M5_Steps = 0;
int           motor5_rpm = 0;

//--- Simulation -------------------
// Simulation:    0=off 1=stop 2=run 3=e-stop 4=failure1    5=failure2    6=failure3    7=failure4     8=failure5    9=failure6   10=failure7
// Simulation: // 0=off 1=stop 2=run 3=e-stop 4=fail feeder 5=fail print1 6=fail print2 7=fail print3  8=fail print4 9=fail dryer 10=fail packer

int         select_simulation = 0;
int         runtime_simulation = 0; // 0=on_off >=1 Sec.
int         mode_simulation = 0; // 0=off 1=on 3=auto

int         sim_reg_accel = 500;
int         sim_reg_speed = 800;
int         sim_reg_M1_dir = -1;
int         sim_reg_M2_dir = 1;
int         sim_reg_M3_dir = -1;
int         sim_reg_M4_dir = 1;
int         sim_reg_M5_dir = 1;

//Failure Feeder
int             sim_f4_steps = 1000;
int             sim_f4_accel = 100000;
int             sim_f4_speed = 500;
int             sim_f4_time = 800;

//Failure Print 1
int             sim_f5_steps = 1000;
int             sim_f5_accel = 100000;
int             sim_f5_speed = 500;
int             sim_f5_time = 800;

//Failure Print 2
int             sim_f6_steps = 1000;
int             sim_f6_accel = 100000;
int             sim_f6_speed = 500;
int             sim_f6_time = 800;

//Failure Print 3
int             sim_f7_time = 800;

//Failure Print 4
int             sim_f8_time = 800;


//--- General variables ------------
bool    upd_simulation = false;
bool    upd_motors = false;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void WiFiEvent(WiFiEvent_t event)
{
    switch (event) {
    case ARDUINO_EVENT_ETH_START: //20
        debugPrintln("ETH Started");
        //Set eth hostname here
        ETH.setHostname("esp32-ethernet");
        delay(1000);
        break;
    case ARDUINO_EVENT_ETH_CONNECTED: //22
        debugPrintln("ETH Connected");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP: //24
        debugPrint("ETH MAC: ");
        debugPrint(ETH.macAddress());
        debugPrint(", IPv4: ");
        debugPrint(ETH.localIP());
        if (ETH.fullDuplex()) {
            debugPrint(", FULL_DUPLEX");
        }
        debugPrint(", ");
        debugPrint(ETH.linkSpeed());
        debugPrintln("Mbps");
        eth_connected = true;
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED: //23
        debugPrintln("ETH Disconnected");
        eth_connected = false;
        break;
    case ARDUINO_EVENT_ETH_STOP: //21
        debugPrintln("ETH Stopped");
        eth_connected = false;
        break;
    default:
        break;
    }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup_Routing() {

    server.on("/control", HTTP_POST, handlePost_control);
    server.on("/motors", HTTP_POST, handlePost_motors);
    server.on("/simulation", HTTP_POST, handlePost_simulation);

    server.begin();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void handlePost_control() {
    if (server.hasArg("plain") == false) {
    }
    String body = server.arg("plain");
    DeserializationError err = deserializeJson(jsonDocument, body);
    if (err) {
        //debugPrint(F("deserializeJson() failed with code "));
        debugPrintln(err.f_str());
    }
    else {

        // Controller - Update
        upd_motors = true;
        select_simulation = 0; //Stop Simulation -> Overide

        // Motor 1
        stepper1Config.Direction            = jsonDocument["Motor 1"]["Direction"];
        stepper1Config.Speed                = jsonDocument["Motor 1"]["Speed"];
        stepper1Config.Accel                = jsonDocument["Motor 1"]["Acceleration"];
        stepper1Config.Command              = jsonDocument["Motor 1"]["Command"];
        stepper1Config.Steps                = jsonDocument["Motor 1"]["Steps"];

        // Motor 2
        stepper2Config.Direction            = jsonDocument["Motor 2"]["Direction"];
        stepper2Config.Speed                = jsonDocument["Motor 2"]["Speed"];
        stepper2Config.Accel                = jsonDocument["Motor 2"]["Acceleration"];
        stepper2Config.Command              = jsonDocument["Motor 2"]["Command"];
        stepper2Config.Steps                = jsonDocument["Motor 2"]["Steps"];

        // Motor 3
        stepper3Config.Direction            = jsonDocument["Motor 3"]["Direction"];
        stepper3Config.Speed                = jsonDocument["Motor 3"]["Speed"];
        stepper3Config.Accel                = jsonDocument["Motor 3"]["Acceleration"];
        stepper3Config.Command              = jsonDocument["Motor 3"]["Command"];
        stepper3Config.Steps                = jsonDocument["Motor 3"]["Steps"];

        // Motor 4
        stepper4Config.Direction            = jsonDocument["Motor 4"]["Direction"];
        stepper4Config.Speed                = jsonDocument["Motor 4"]["Speed"];
        stepper4Config.Accel                = jsonDocument["Motor 4"]["Acceleration"];
        stepper4Config.Command              = jsonDocument["Motor 4"]["Command"];
        stepper4Config.Steps                = jsonDocument["Motor 4"]["Steps"];

        // Motor 5
        stepper5Config.Direction            = jsonDocument["Motor 5"]["Direction"];
        stepper5Config.Speed                = jsonDocument["Motor 5"]["Speed"];
        stepper5Config.Accel                = jsonDocument["Motor 5"]["Acceleration"];
        stepper5Config.Command              = jsonDocument["Motor 5"]["Command"];
        stepper5Config.Steps                = jsonDocument["Motor 5"]["Steps"];

        //Serial
        debugPrintln("------- POST: Control Update -------");
        debugPrintln("Motor 1:");
        debugPrint("Dir: ");
        debugPrint(M1_Direction);
        debugPrint(", Speed: ");
        debugPrint(M1_Speed);
        debugPrint(", Accel: ");
        debugPrint(M1_Accel);
        debugPrint(", Command: ");
        debugPrint(M1_Command);
        debugPrint(", Steps: ");
        debugPrintln(M1_Steps);

        debugPrintln("Motor 2:");
        debugPrint("Dir: ");
        debugPrint(M2_Direction);
        debugPrint(", Speed: ");
        debugPrint(M2_Speed);
        debugPrint(", Accel: ");
        debugPrint(M2_Accel);
        debugPrint(", Command: ");
        debugPrint(M2_Command);
        debugPrint(", Steps: ");
        debugPrintln(M2_Steps);

        debugPrintln("Motor 3:");
        debugPrint("Dir: ");
        debugPrint(M3_Direction);
        debugPrint(", Speed: ");
        debugPrint(M3_Speed);
        debugPrint(", Accel: ");
        debugPrint(M3_Accel);
        debugPrint(", Command: ");
        debugPrint(M3_Command);
        debugPrint(", Steps: ");
        debugPrintln(M3_Steps);

        debugPrintln("Motor 4:");
        debugPrint("Dir: ");
        debugPrint(M4_Direction);
        debugPrint(", Speed: ");
        debugPrint(M4_Speed);
        debugPrint(", Accel: ");
        debugPrint(M4_Accel);
        debugPrint(", Command: ");
        debugPrint(M4_Command);
        debugPrint(", Steps: ");
        debugPrintln(M4_Steps);

        debugPrintln("Motor 4:");
        debugPrint("Dir: ");
        debugPrint(M5_Direction);
        debugPrint(", Speed: ");
        debugPrint(M5_Speed);
        debugPrint(", Accel: ");
        debugPrint(M5_Accel);
        debugPrint(", Command: ");
        debugPrint(M5_Command);
        debugPrint(", Steps: ");
        debugPrintln(M5_Steps);

        debugPrintln("----------------------------");

        server.send(200, "application/json", "{}");
    }

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void handlePost_simulation() {

    int sim_run_ramp_time;

    bool ext_start = false;
    long curr_millis;;
    static long prev_millis;
    int max_waittime = 10000; // 10s

    if (server.hasArg("plain") == false) {
    }
    String body = server.arg("plain");
    DeserializationError err = deserializeJson(jsonDocument, body);
    if (err) {

        //debugPrint(F("deserializeJson() failed with code "));
        debugPrintln(err.f_str());
    }
    else {

        // --- Simulation  Allgemein -------------------------------------------

        // Simulation:    0=off 1=stop 2=run 3=e-stop 4=failure1    5=failure2    6=failure3    7=failure4     8=failure5    9=failure6   10=failure7
        // Simulation: // 0=off 1=stop 2=run 3=e-stop 4=fail feeder 5=fail print1 6=fail print2 7=fail print3  8=fail print4 9=fail dryer 10=fail packer
        // Mode: 0=off 1=on 3=auto
        //Run time - Failure Simulation total

        select_simulation               = jsonDocument["Simulation"]["Selection"];
        mode_simulation                 = jsonDocument["Simulation"]["Auto mode"];
        runtime_simulation              = jsonDocument["Simulation"]["Duration"];

        // --- RUN -------------------------------------------------------------
        sim_reg_speed                   = jsonDocument["Parameter RUN"]["Speed"];
        sim_reg_M1_dir                  = jsonDocument["Parameter RUN"]["Direction Motor 1"];
        sim_reg_M2_dir                  = jsonDocument["Parameter RUN"]["Direction Motor 2"];
        sim_reg_M3_dir                  = jsonDocument["Parameter RUN"]["Direction Motor 3"];
        sim_reg_M4_dir                  = jsonDocument["Parameter RUN"]["Direction Motor 4"];
        sim_reg_M5_dir                  = jsonDocument["Parameter RUN"]["Direction Motor 5"];

        //Ramp Up_Down
        sim_run_ramp_time               = jsonDocument["Parameter RUN"]["Ramp Up_Down Time"];
        sim_reg_accel = round((sim_reg_speed * steps_per_rpmStepper1) / (60 * sim_run_ramp_time));


        // --- Feeder ---
        sim_f4_steps                    = jsonDocument["Parameter Feeder"]["Steps"];
        sim_f4_accel                    = jsonDocument["Parameter Feeder"]["Acceleration"];
        sim_f4_speed                    = jsonDocument["Parameter Feeder"]["Speed"];
        sim_f4_time                     = jsonDocument["Parameter Feeder"]["Time"];

        // --- Print 1 ---
        sim_f5_steps                    = jsonDocument["Parameter Print 1"]["Steps"];
        sim_f5_accel                    = jsonDocument["Parameter Print 1"]["Acceleration"];
        sim_f5_speed                    = jsonDocument["Parameter Print 1"]["Speed"];
        sim_f5_time                     = jsonDocument["Parameter Print 1"]["Time"];

        // --- Print 2 ---
        sim_f6_steps                    = jsonDocument["Parameter Print 2"]["Steps"];
        sim_f6_accel                    = jsonDocument["Parameter Print 2"]["Acceleration"];
        sim_f6_speed                    = jsonDocument["Parameter Print 2"]["Speed"];
        sim_f6_time                     = jsonDocument["Parameter Print 2"]["Time"];

        // --- Print 3 ---
        sim_f7_time                     = jsonDocument["Parameter Print 3"]["Time"];

        // --- Print 4 -------------------------------------------------------
        sim_f8_time                     = jsonDocument["Parameter Print 4"]["Time"];

        //Serial  
        debugPrintln("------- POST: Simulation Update -------");
        debugPrint("Selected: ");
        debugPrint(select_simulation);
        debugPrint(" | Mode: ");
        debugPrint(mode_simulation);
        debugPrint(" | Dauer: ");
        debugPrintln(runtime_simulation);

        debugPrintln("--- Parameter RUN:");
        debugPrintln("Motoren:");
        debugPrint("Speed: ");
        debugPrint(sim_reg_speed);
        debugPrint(" | Accel: ");
        debugPrint(sim_reg_accel);
        debugPrint(" Direction M1: ");
        debugPrint(sim_reg_M1_dir);
        debugPrint(" | M2: ");
        debugPrint(sim_reg_M2_dir);
        debugPrint(" | M3: ");
        debugPrint(sim_reg_M3_dir);
        debugPrint(" | M4: ");
        debugPrint(sim_reg_M4_dir);

        debugPrintln("--- Parameter Feeder:");
        debugPrint("Steps: ");
        debugPrint(sim_f4_steps);
        debugPrint(" | Accel: ");
        debugPrint(sim_f4_accel);
        debugPrint(" | Speed: ");
        debugPrint(sim_f4_speed);
        debugPrint(" | Time: ");
        debugPrintln(sim_f4_time);

        debugPrintln("--- Parameter Print 1:");
        debugPrint("Steps: ");
        debugPrint(sim_f5_steps);
        debugPrint(" | Accel: ");
        debugPrint(sim_f5_accel);
        debugPrint(" | Speed: ");
        debugPrint(sim_f5_speed);
        debugPrint(" | Time: ");
        debugPrintln(sim_f5_time);

        debugPrintln("--- Parameter Print 2:");
        debugPrint("Steps: ");
        debugPrint(sim_f6_steps);
        debugPrint(" | Accel: ");
        debugPrint(sim_f6_accel);
        debugPrint(" | Speed: ");
        debugPrintln(sim_f6_speed);
        debugPrint(" | Time: ");
        debugPrintln(sim_f6_time);

        debugPrintln("---------------------------------");

        server.send(200, "application/json", "{}");

        curr_millis = millis();
        prev_millis = curr_millis;

        debugPrint("Warte auf Startsignal ");

        while (ext_start == false) {
            debugPrint(".");

            ext_start = digitalRead(startPin);
            
            curr_millis = millis();
            if ((curr_millis - prev_millis) > max_waittime) {
                ext_start = true;
                debugPrintln("");
                debugPrintln("ABBRUCH: Kein Startsignal erhalten -> Starte");
            }
        }
        debugPrintln("");
        debugPrintln("-> Starte");

        upd_simulation = true;

    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void handlePost_motors() {
    if (server.hasArg("plain") == false) {
    }
    String body = server.arg("plain");
    deserializeJson(jsonDocument, body);

    motor1_rpm = jsonDocument["Motor 1 Speed"].as<int>();
    motor2_rpm = jsonDocument["Motor 2 Speed"].as<int>();
    motor3_rpm = jsonDocument["Motor 3 Speed"].as<int>();
    motor4_rpm = jsonDocument["Motor 4 Speed"].as<int>();
    motor5_rpm = jsonDocument["Motor 5 Speed"].as<int>();

    upd_motors = true;
    select_simulation = 0; //Stop Simulation -> Overide

    //Motor 1
    if (motor1_rpm == 0) {
        M1_Speed = 0;
        M1_Command = 0;
    }
    else if (motor1_rpm > 0) {
        M1_Direction = 1;
        M1_Speed = motor1_rpm;
        M1_Command = 2;
    }
    else if (motor1_rpm < 0) {
        M1_Direction = -1;
        M1_Speed = abs(motor1_rpm);
        M1_Command = 2;
    }

    //Motor 2

    if (motor2_rpm == 0) {
        M2_Speed = 0;
        M2_Command = 0;
    }
    else if (motor1_rpm > 0) {
        M2_Direction = 1;
        M2_Speed = motor2_rpm;
        M2_Command = 2;
    }
    else if (motor2_rpm < 0) {
        M2_Direction = -1;
        M2_Speed = abs(motor2_rpm);
        M2_Command = 2;
    }

    //Motor 3
    if (motor3_rpm == 0) {
        M3_Speed = 0;
        M3_Command = 0;
    }
    else if (motor3_rpm > 0) {
        M3_Direction = 1;
        M3_Speed = motor3_rpm;
        M3_Command = 2;
    }
    else if (motor3_rpm < 0) {
        M3_Direction = -1;
        M3_Speed = abs(motor3_rpm);
        M3_Command = 2;
    }

    //Motor 4
    if (motor4_rpm == 0) {
        M4_Speed = 0;
        M4_Command = 0;
    }
    else if (motor4_rpm > 0) {
        M4_Direction = 1;
        M4_Speed = motor4_rpm;
        M4_Command = 2;
    }
    else if (motor4_rpm < 0) {
        M4_Direction = -1;
        M4_Speed = abs(motor4_rpm);
        M4_Command = 2;
    }

    //Motor 5
    if (motor5_rpm == 0) {
        M5_Speed = 0;
        M5_Command = 0;
    }
    else if (motor5_rpm > 0) {
        M5_Direction = 1;
        M5_Speed = motor5_rpm;
        M5_Command = 2;
    }
    else if (motor5_rpm < 0) {
        M5_Direction = -1;
        M5_Speed = abs(motor5_rpm);
        M5_Command = 2;
    }

    // Serial
    debugPrintln("------- POST: Motor Update -------");
    debugPrint("Motor 1: ");
    debugPrint(motor1_rpm);
    debugPrint(" | Motor 2: ");
    debugPrintln(motor2_rpm);
    debugPrint("Motor 3: ");
    debugPrint(motor3_rpm);
    debugPrint("Motor 4: ");
    debugPrint(motor4_rpm);
    debugPrint(" | Motor 5: ");
    debugPrintln(motor5_rpm);

    debugPrintln("----------------------------");

    server.send(200, "application/json", "{}");

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void update_motors(FastAccelStepper* stepper, struct StepperConfig& stepperConfig) {

    if (!stepper)
        return;

    // M_Command 0=force, Stop 1=reg, Stop 2=run cont, 3=run steps
    debugPrintln("--- Update Motors ----");

    //--- Motor 1------------------
    if (stepperConfig.Command == 0) {
        // Force STOP
        stepper->forceStop();
        debugPrintln("M1 STOP forced");
    }
    else if (stepperConfig.Command == 1) {
        // Reg. STOP
        stepper->setAcceleration(stepperConfig.Accel);
        stepper->stopMove();
        debugPrintln("M1 STOP reg");
    }
    else if (stepperConfig.Command == 2) {
        // Run cont.
        if (stepperConfig.Speed > 0 && stepperConfig.Direction == 1) {
            stepper->setSpeedInHz(round((stepperConfig.Speed * stepperConfig.steps_per_rpm) / 60));
            stepper->setAcceleration(stepperConfig.Accel);
            stepper->runForward();
            debugPrintln("M1 RUN fwd");
        }
        else if (stepperConfig.Speed > 0 && stepperConfig.Direction == -1) {
            stepper->setSpeedInHz(round((stepperConfig.Speed * stepperConfig.steps_per_rpm) / 60));
            stepper->setAcceleration(stepperConfig.Accel);
            stepper->runBackward();
            debugPrintln("M1 RUN rev");
        }
    }
    else if (stepperConfig.Command == 3) {
        // Run Steps
        stepper->setSpeedInHz(round((stepperConfig.Speed * stepperConfig.steps_per_rpm) / 60));
        stepper->setAcceleration(stepperConfig.Accel);
        stepper->move(stepperConfig.Steps);
    }


}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void run_simulation(void* parameter) {

    // Simulation:    0=off 1=stop 2=run 3=e-stop 4=failure1    5=failure2    6=failure3    7=failure4     8=failure5    9=failure6   10=failure7
    // Simulation: // 0=off 1=stop 2=run 3=e-stop 4=fail feeder 5=fail print1 6=fail print2 7=fail print3  8=fail print4 9=fail dryer 10=fail packer
    // M_Command:      0=stop forced, 1=stop reg 2=run cont, 3=run steps

    static int prev_sim = 0;
    bool sim_f4 = false;
    bool sim_f5 = false;
    bool sim_f6 = false;
    bool sim_f7 = false;
    bool sim_f8 = false;
    bool sim_f9 = false;
    bool sim_f10 = false;

    long curr_millis = millis();
    static long prev_millis = curr_millis;
    long sim_remain_time = (runtime_simulation * 1000);

    while (1) {
        if (upd_simulation == true) {

            debugPrintln("------- Simulation geandert -------");
            debugPrint("Simulation bisher: ");
            debugPrint(prev_sim);
            debugPrint(" | neu: ");
            debugPrintln(select_simulation);
            debugPrint("Auto Modus: ");
            debugPrintln(mode_simulation);
        }

        //+++ Regular STOP ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if (upd_simulation == true && select_simulation == 1 && prev_sim != 1) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation;

            //Motor 1
            M1_Command = 1;
            M1_Accel = sim_reg_accel;
            M1_Speed = 0;
            M1_Direction = sim_reg_M1_dir;

            //Motor 2
            M2_Command = 1;
            M2_Accel = sim_reg_accel;
            M2_Speed = 0;
            M2_Direction = sim_reg_M2_dir;

            //Motor 3
            M3_Command = 1;
            M3_Accel = sim_reg_accel;
            M3_Speed = 0;
            M3_Direction = sim_reg_M3_dir;

            //Motor 4
            M4_Command = 1;
            M4_Accel = sim_reg_accel;
            M4_Speed = 0;
            M4_Direction = sim_reg_M4_dir;

            //Motor 5
            M5_Command = 1;
            M5_Accel = sim_reg_accel;
            M5_Speed = 0;
            M5_Direction = sim_reg_M5_dir;

            upd_motors = true;

            //Serial
            debugPrintln("REG STOP ausgefuehrt");
        }

        //+++ Regular RUN +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && select_simulation == 2 && prev_sim != 2) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation; //Setze REG RUN

            //Motor 1
            M1_Command = 2;
            M1_Accel = sim_reg_accel;
            M1_Speed = sim_reg_speed;
            M1_Direction = sim_reg_M1_dir;

            //Motor 2
            M2_Command = 2;
            M2_Accel = sim_reg_accel;
            M2_Speed = sim_reg_speed;
            M2_Direction = sim_reg_M2_dir;

            //Motor 3
            M3_Command = 2;
            M3_Accel = sim_reg_accel;
            M3_Speed = sim_reg_speed;
            M3_Direction = sim_reg_M3_dir;

            //Motor 4
            M4_Command = 2;
            M4_Accel = sim_reg_accel;
            M4_Speed = sim_reg_speed;
            M4_Direction = sim_reg_M4_dir;

            //Motor 5
            M5_Command = 2;
            M5_Accel = sim_reg_accel;
            M5_Speed = sim_reg_speed;
            M5_Direction = sim_reg_M5_dir;

            upd_motors = true;

            //Serial
            debugPrintln("REG RUN gestartet");
        }
        //+++ E-STOP (only after REG RUN) +++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && select_simulation == 3 && prev_sim != 3) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation;

            //Motor 1
            M1_Command = 0;

            //Motor 2
            M2_Command = 0;

            //Motor 3
            M3_Command = 0;

            //Motor 4
            M4_Command = 0;

            //Motor 5
            M5_Command = 0;

            upd_motors = true;

            //Serial
            debugPrintln("E-STOP ausgefuehrt");
        }
        // +++ Feeder ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && prev_sim == 2 && select_simulation == 4 && prev_sim != 4) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation;
            sim_f4 = true;

            curr_millis = millis();
            prev_millis = curr_millis;

            //Force STOP M1
            M1_Command = 0;

            upd_motors = true; // Stoppe Motoren

            //Serial
            debugPrintln("Failure Feeder gestartet");

            delay(sim_f4_time);

            do {
                // --- Do STEPs Forward --------------
                M1_Command = 3;
                M1_Steps = sim_f4_steps;
                M1_Accel = sim_f4_accel;
                M1_Speed = sim_f4_speed;

                upd_motors = true;
                delay(sim_f4_time);

                // --- Do STEPS Reversed ---------------
                M1_Command = 3;
                M1_Steps = -sim_f4_steps;
                M1_Accel = sim_f4_accel;
                M1_Speed = sim_f4_speed;

                upd_motors = true;
                // -------------------------------------

                curr_millis = millis();
                sim_remain_time = (runtime_simulation * 1000) + prev_millis - curr_millis;

                //Duration ended -> switch to REG RUN
                if (sim_remain_time <= 0 && mode_simulation == 1) {

                    //Force STOP all Motors
                    M1_Command = 0;

                    upd_motors = true;
                    delay(1000);

                    upd_simulation = true;
                    select_simulation = 2;
                    sim_f4 = false;
                    sim_remain_time = 0;
                    debugPrintln("Failure Feeder beendet -> RUN");

                }
                //Stop Failure 1 -> Resume according to update
                else if (upd_simulation == true && select_simulation != 7) {
                    sim_f4 = false;
                    debugPrintln("Failure Feeder gestoppt");
                    debugPrintln("Neuer Befehl empfangen");
                }
                else {
                    delay(sim_f4_time);
                    debugPrintln("Failure Feeder laueft");
                }
            } while (sim_f4 == true);
        }

        // +++ Print 1 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && prev_sim == 2 && select_simulation == 5 && prev_sim != 5) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation;
            sim_f5 = true;

            curr_millis = millis();
            prev_millis = curr_millis;

            //Force STOP M2 + M3
            M2_Command = 0;
            M3_Command = 0;

            upd_motors = true; // Stoppe Motoren

            //Serial
            debugPrintln("Failure Print 1 gestartet");

            delay(sim_f5_time);

            do {
                // --- Do STEPs Forward --------------
                M2_Command = 3;
                M2_Steps = -sim_f5_steps;
                M2_Accel = sim_f5_accel;
                M2_Speed = sim_f5_speed;

                M3_Command = 3;
                M3_Steps = sim_f5_steps;
                M3_Accel = sim_f5_accel;
                M3_Speed = sim_f5_speed;

                upd_motors = true;
                delay(sim_f5_time);

                // --- Do STEPS Reversed ---------------
                M2_Command = 3;
                M2_Steps = sim_f5_steps;
                M2_Accel = sim_f5_accel;
                M2_Speed = sim_f5_speed;

                M3_Command = 3;
                M3_Steps = -sim_f5_steps;
                M3_Accel = sim_f5_accel;
                M3_Speed = sim_f5_speed;

                upd_motors = true;
                // -------------------------------------

                curr_millis = millis();
                sim_remain_time = (runtime_simulation * 1000) + prev_millis - curr_millis;

                //Duration ended -> switch to REG RUN
                if (sim_remain_time <= 0 && mode_simulation == 1) {

                    //Force STOP all Motors
                    M2_Command = 0;
                    M3_Command = 0;

                    upd_motors = true;
                    delay(1000);

                    upd_simulation = true;
                    select_simulation = 2;
                    sim_f5 = false;
                    sim_remain_time = 0;
                    debugPrintln("Failure Print 1 beendet -> RUN");

                }
                //Stop Failure 1 -> Resume according to update
                else if (upd_simulation == true && select_simulation != 7) {
                    sim_f5 = false;
                    debugPrintln("Failure Print 3´1 gestoppt");
                    debugPrintln("Neuer Befehl empfangen");
                }
                else {
                    delay(sim_f5_time);
                    debugPrintln("Failure Print 1 laueft");
                }
            } while (sim_f5 == true);
        }

        // +++ Print 2 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && prev_sim == 2 && select_simulation == 6 && prev_sim != 6) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation;
            sim_f6 = true;

            curr_millis = millis();
            prev_millis = curr_millis;

            //Force STOP M4 + M5
            M4_Command = 0;
            M5_Command = 0;

            upd_motors = true; // Stoppe Motoren

            //Serial
            debugPrintln("Print 2 gestartet");

            delay(sim_f6_time);

            do {
                // --- Do STEPs Forward --------------
                M4_Command = 3;
                M4_Steps = -sim_f6_steps;
                M4_Accel = sim_f6_accel;
                M4_Speed = sim_f6_speed;

                M5_Command = 3;
                M5_Steps = sim_f6_steps;
                M5_Accel = sim_f6_accel;
                M5_Speed = sim_f6_speed;

                upd_motors = true;
                delay(sim_f6_time);

                // --- Do STEPS Reversed ---------------
                M4_Command = 3;
                M4_Steps = sim_f6_steps;
                M4_Accel = sim_f6_accel;
                M4_Speed = sim_f6_speed;

                M5_Command = 3;
                M5_Steps = -sim_f6_steps;
                M5_Accel = sim_f6_accel;
                M5_Speed = sim_f6_speed;

                upd_motors = true;
                // -------------------------------------

                curr_millis = millis();
                sim_remain_time = (runtime_simulation * 1000) + prev_millis - curr_millis;

                //Duration ended -> switch to REG RUN
                if (sim_remain_time <= 0 && mode_simulation == 1) {

                    //Force STOP all Motors
                    M4_Command = 0;
                    M5_Command = 0;
                    upd_motors = true;
                    delay(1000);

                    upd_simulation = true;
                    select_simulation = 2;
                    sim_f6 = false;
                    sim_remain_time = 0;
                    debugPrintln("Failure Print 2 beendet -> RUN");

                }
                //Stop Failure 2 -> Resume according to update
                else if (upd_simulation == true && select_simulation != 8) {
                    sim_f6 = false;
                    debugPrintln("Failure Print 2 gestoppt");
                    debugPrintln("Neuer Befehl empfangen");
                }
                else {
                    delay(sim_f6_time);
                    debugPrintln("Failure Print 2 laueft");
                }

            } while (sim_f6 == true);
        }

        // +++ Print 3 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && prev_sim == 2 && select_simulation == 7 && prev_sim != 7) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation;
            sim_f6 = true;

            curr_millis = millis();
            prev_millis = curr_millis;

            //Serial
            debugPrintln("Failure Print 3 gestartet");

            delay(500);

            do {

                curr_millis = millis();
                sim_remain_time = (runtime_simulation * 1000) + prev_millis - curr_millis;

                //Duration ended -> switch to REG RUN
                if (sim_remain_time <= 0 && mode_simulation == 1) {

                    upd_simulation = true;
                    select_simulation = 2;
                    sim_f6 = false;
                    sim_remain_time = 0;
                    debugPrintln("Failure Print 3 beendet -> RUN");

                }
                //Stop Failure Print 3 -> Resume according to update
                else if (upd_simulation == true && select_simulation != 5) {
                    sim_f6 = false;
                    debugPrintln("Failure Print 3 gestoppt");
                    debugPrintln("Neuer Befehl empfangen");
                }
                else {
                    delay(500);
                    debugPrintln("Failure Print 3 laueft");
                }
            } while (sim_f6 == true);
        }

        // +++ Print 4 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && prev_sim == 2 && select_simulation == 8 && prev_sim != 8) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation;
            sim_f8 = true;

            curr_millis = millis();
            prev_millis = curr_millis;

            //Serial
            debugPrintln("Failure Print 4 gestartet");

            delay(500);

            do {

                curr_millis = millis();
                sim_remain_time = (runtime_simulation * 1000) + prev_millis - curr_millis;

                //Duration ended -> switch to REG RUN
                if (sim_remain_time <= 0 && mode_simulation == 1) {

                    upd_simulation = true;
                    select_simulation = 2;
                    sim_f8 = false;
                    sim_remain_time = 0;
                    debugPrintln("Failure Print 4 beendet -> RUN");

                }
                //Stop Failure Print 4 -> Resume according to update
                else if (upd_simulation == true && select_simulation != 5) {
                    sim_f8 = false;
                    debugPrintln("Failure Print 4 gestoppt");
                    debugPrintln("Neuer Befehl empfangen");
                }
                else {
                    delay(500);
                    debugPrintln("Failure Print 4 laueft");
                }
            } while (sim_f8 == true);
        }

        // +++ Dryer ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && prev_sim == 2 && select_simulation == 9 && prev_sim != 9) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation;
            sim_f9 = true;

            curr_millis = millis();
            prev_millis = curr_millis;

            //Serial
            debugPrintln("Failure Dryer gestartet");

            delay(500);

            do {

                curr_millis = millis();
                sim_remain_time = (runtime_simulation * 1000) + prev_millis - curr_millis;

                //Duration ended -> switch to REG RUN
                if (sim_remain_time <= 0 && mode_simulation == 1) {

                    upd_simulation = true;
                    select_simulation = 2;
                    sim_f9 = false;
                    sim_remain_time = 0;
                    debugPrintln("Failure Dryer beendet -> RUN");

                }
                //Stop Failure Dryer -> Resume according to update
                else if (upd_simulation == true && select_simulation != 5) {
                    sim_f9 = false;
                    debugPrintln("Failure Dryer gestoppt");
                    debugPrintln("Neuer Befehl empfangen");
                }
                else {
                    delay(500);
                    debugPrintln("Failure Dryer laueft");
                }
            } while (sim_f9 == true);
        }

        // +++ Packer ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && prev_sim == 2 && select_simulation == 10 && prev_sim != 10) {
            upd_simulation = false; //Reset until next post
            prev_sim = select_simulation;
            sim_f10 = true;

            curr_millis = millis();
            prev_millis = curr_millis;

            //Serial
            debugPrintln("Failure Packer gestartet");

            delay(500);

            do {

                curr_millis = millis();
                sim_remain_time = (runtime_simulation * 1000) + prev_millis - curr_millis;

                //Duration ended -> switch to REG RUN
                if (sim_remain_time <= 0 && mode_simulation == 1) {

                    upd_simulation = true;
                    select_simulation = 2;
                    sim_f10 = false;
                    sim_remain_time = 0;
                    debugPrintln("Failure Packer beendet -> RUN");

                }
                //Stop Failure Packer -> Resume according to update
                else if (upd_simulation == true && select_simulation != 5) {
                    sim_f10 = false;
                    debugPrintln("Failure Packer gestoppt");
                    debugPrintln("Neuer Befehl empfangen");
                }
                else {
                    delay(500);
                    debugPrintln("Failure Packer laueft");
                }
            } while (sim_f10 == true);
        }

        // +++ Off ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (upd_simulation == true && select_simulation == 0 && prev_sim != 0) {
            upd_simulation = false;
            prev_sim = select_simulation;

            debugPrintln("Simulation - OFF");

            //Force STOP all Motors
            M1_Command = 0;
            M2_Command = 0;
            M3_Command = 0;
            M4_Command = 0;
            M5_Command = 0;
            upd_motors = true;

        }

        // +++ Simulation Allgemein +++++++++++++++++++++++++++++++++++++++++++++++++++++
        else if (select_simulation == 2 && prev_sim == 2 && upd_simulation == true) {
            upd_simulation = false;
            debugPrintln("Simulation - Befehl ungueltig");
            debugPrintln("REG RUN laeuft bereits");
        }
        else if (select_simulation != 2 && upd_simulation == true) {
            upd_simulation = false;
            debugPrintln("Simulation - Befehl ungueltig");
            debugPrintln("REG RUN nicht gestartet");
        }

        delay(100);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup_Tasks() {

    xTaskCreate(
        run_simulation,         // Function name of the task
        "run_simulation",       // Name of the task (e.g. for debugging)
        1024 * 2,               // Stack size (bytes)
        NULL,                   // Parameter to pass
        3,                      // Task priority
        &run_simulation_handle  // Task handle
    );

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup() {
    debugBegin(115200);
    debugPrintln("Starte ESP-Controller");

    //--- External Start ---
    pinMode(startPin, INPUT);

    //--- Steppers ---
    engine.init();
    stepper1 = engine.stepperConnectToPin(stepPinStepper1);
    if (stepper1) {
        stepper1->setDirectionPin(dirPinStepper1);
        stepper1->setAutoEnable(true);
        stepper1->setAcceleration(500);
    }
    stepper2 = engine.stepperConnectToPin(stepPinStepper2);
    if (stepper2) {
        stepper2->setDirectionPin(dirPinStepper2);
        stepper2->setAutoEnable(true);
        stepper2->setAcceleration(500);
    }
    stepper3 = engine.stepperConnectToPin(stepPinStepper3);
    if (stepper3) {
        stepper3->setDirectionPin(dirPinStepper3);
        stepper3->setAutoEnable(true);
        stepper3->setAcceleration(500);
    }
    stepper4 = engine.stepperConnectToPin(stepPinStepper4);
    if (stepper4) {
        stepper4->setDirectionPin(dirPinStepper4);
        stepper4->setAutoEnable(true);
        stepper4->setAcceleration(500);
    }
    stepper5 = engine.stepperConnectToPin(stepPinStepper5);
    if (stepper5) {
        stepper5->setDirectionPin(dirPinStepper5);
        stepper5->setAutoEnable(true);
        stepper5->setAcceleration(500);
    }

    //--- Ethernet ---
    WiFi.onEvent(WiFiEvent);
    ETH.begin(); // Enable ETH
    ETH.config(local_ip, gateway, subnet, dns1, dns2); // Static IP, leave without this line to get IP via DHCP

    while (!((uint32_t)ETH.localIP())) {
        delay(1000);
    }; // Waiting for IP (leave this line group to get IP via DHCP)

    server.begin(); // Start server
    debugPrintln("Web server started");

    setup_Tasks();
    setup_Routing();

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop() {

    server.handleClient();

    if (upd_motors == true) {
        update_motors(stepper1, stepper1Config);
        update_motors(stepper2, stepper2Config);
        update_motors(stepper3, stepper3Config);
        update_motors(stepper4, stepper4Config);
        update_motors(stepper5, stepper5Config);
        upd_motors = false;
    }
    else {
        if (M1_Command == 2) {
            if (M1_Speed > 0 && M1_Direction == 1) {
                stepper1->runForward();
            }
            else if (M1_Speed > 0 && M1_Direction == -1) {
                stepper1->runBackward();
            }
        }

        if (M2_Command == 2) {
            if (M2_Speed > 0 && M2_Direction == 1) {
                stepper2->runForward();
            }
            else if (M2_Speed > 0 && M2_Direction == -1) {
                stepper2->runBackward();
            }
        }

        if (M3_Command == 2) {
            if (M3_Speed > 0 && M3_Direction == 1) {
                stepper3->runForward();
            }
            else if (M3_Speed > 0 && M3_Direction == -1) {
                stepper3->runBackward();
            }
        }

        if (M4_Command == 2) {
            if (M4_Speed > 0 && M4_Direction == 1) {
                stepper4->runForward();
            }
            else if (M4_Speed > 0 && M4_Direction == -1) {
                stepper4->runBackward();
            }
        }

        if (M5_Command == 2) {
            if (M5_Speed > 0 && M5_Direction == 1) {
                stepper5->runForward();
            }
            else if (M5_Speed > 0 && M5_Direction == -1) {
                stepper5->runBackward();
            }
        }
    }

    delay(2);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


