#include <WiFi.h>
#include <WiFiUdp.h>
#include "MAVLink.h"

// --- Configuration ---
#define WIFI_SSID "Go"          
#define WIFI_PASSWORD "Gourav#14"   
#define UDP_PORT 14550

#define FC_RX_PIN 16  
#define FC_TX_PIN 17  
#define FC_BAUDRATE 115200

#define HEARTBEAT_INTERVAL 500  
#define RC_OVERRIDE_RATE 50  // 50ms = 20Hz for RC override

// MAVLink settings
const uint8_t SYSTEM_ID = 255;        
const uint8_t COMPONENT_ID = 0;       
const uint8_t TARGET_SYSTEM = 1;      
const uint8_t TARGET_COMPONENT = 0;   

WiFiUDP udp;
mavlink_message_t msg;
uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];

// RC Channel values
struct RCChannels {
    uint16_t ch1 = 1500;  // Roll
    uint16_t ch2 = 1500;  // Pitch  
    uint16_t ch3 = 1000;  // Throttle
    uint16_t ch4 = 1500;  // Yaw
    uint16_t ch5 = 1500;  // Mode switch
    uint16_t ch6 = 1500;  // Aux 2
    uint16_t ch7 = 1500;  // Aux 3
    uint16_t ch8 = 1500;  // Aux 4
} rcChannels;

// Throttle smoothing variables
uint16_t targetThrottle = 1000;
uint16_t currentSmoothThrottle = 1000;
unsigned long lastThrottleUpdate = 0;
int THROTTLE_SMOOTH_RATE = 10; // Max change per update cycle
const int THROTTLE_UPDATE_INTERVAL = 20; // milliseconds

// Drone State Variables
bool droneArmed = false;
bool droneFlying = false; 
String currentMode = "UNKNOWN";
unsigned long lastHeartbeatTime = 0;
bool wifiConnected = false;
int currentRSSI = 0;
float batteryVoltage = 0.0;
float batteryPercent = 0.0;
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsAltitude = 0.0;
int gpsSatellites = 0;

// RC Override variables
bool rcOverrideActive = false;
unsigned long lastRCOverrideTime = 0;

// Failsafe variables
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 1000; // 1 second timeout for LAND
const unsigned long WIFI_TIMEOUT = 3000;    // 3 seconds for WiFi loss
bool failsafeActive = false;
unsigned long lastFailsafeCheck = 0;
const unsigned long FAILSAFE_CHECK_INTERVAL = 100; // Check every 100ms for faster response
unsigned long wifiLostTime = 0;
bool wifiWasConnected = false;

// Flight modes
enum FlightMode {
    STABILIZE = 0,
    ACRO = 1,
    ALT_HOLD = 2,
    AUTO = 3,
    GUIDED = 4,
    LOITER = 5,
    RTL = 6,
    CIRCLE = 7,
    LAND = 9,
    DRIFT = 11,
    SPORT = 13,
    POSHOLD = 16,
    GUIDED_NOGPS = 20
};

// Motor test type (since MAV_MOTOR_TEST_THROTTLE_PERCENT is not defined)
#define MOTOR_TEST_THROTTLE_PERCENT 1

// Function declarations
void send_mavlink_message();
void send_rc_override();
void request_data_streams();
void send_status_response();
void processMavlinkMessage(mavlink_message_t& mav_msg);
void processCommand(String command);
void set_flight_mode(uint32_t mode);
void updateSmoothThrottle();
void start_esc_calibration();
void run_motor_test(uint8_t motor_number, uint16_t throttle_percent, uint16_t duration_ms);
void checkFailsafe();
void executeFailsafe(const char* reason);
void sendFailsafeStatus(String status);

// Helper functions
void send_mavlink_message() {
    uint16_t len = mavlink_msg_to_send_buffer(mavBuffer, &msg);
    Serial2.write(mavBuffer, len);
}

// Check for failsafe conditions - CORRECT VERSION
void checkFailsafe() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastFailsafeCheck < FAILSAFE_CHECK_INTERVAL) {
        return;
    }
    lastFailsafeCheck = currentTime;
    
    // Check WiFi connection
    bool currentWifiConnected = (WiFi.status() == WL_CONNECTED);
    
    // If WiFi just disconnected, record the time
    if (wifiWasConnected && !currentWifiConnected) {
        wifiLostTime = currentTime;
        Serial.println("WiFi connection lost!");
    }
    
    wifiWasConnected = currentWifiConnected;
    wifiConnected = currentWifiConnected;
    
    // Check for command timeout (1 second - primary failsafe)
    if (currentTime - lastCommandTime > COMMAND_TIMEOUT && rcOverrideActive) {
        if (!failsafeActive) {
            executeFailsafe("COMMAND_TIMEOUT");
        }
        return;
    }
    
    // Check for WiFi loss timeout (3 seconds - secondary failsafe)
    if (!currentWifiConnected && wifiLostTime > 0 && 
        (currentTime - wifiLostTime > WIFI_TIMEOUT)) {
        if (!failsafeActive) {
            executeFailsafe("WIFI_LOST");
        }
        return;
    }
    
    // If we get here and failsafe was active, we can recover
    if (failsafeActive && currentWifiConnected && 
        (currentTime - lastCommandTime < COMMAND_TIMEOUT)) {
        failsafeActive = false;
        Serial.println("Failsafe recovered - signal restored");
        sendFailsafeStatus("RECOVERED");
    }
} // ← FUNCTION SHOULD END HERE

// Execute failsafe action - always LAND
void executeFailsafe(const char* reason) {
    if (failsafeActive) return; // Already in failsafe
    
    failsafeActive = true;
    Serial.printf("FAILSAFE ACTIVATED: %s - Landing immediately!\n", reason);
    
    // Stop RC override
    rcOverrideActive = false;
    
    // Send LAND command
    set_flight_mode(LAND);
    
    // Send one more override with safe values before releasing
    rcChannels.ch1 = 1500; // Center roll
    rcChannels.ch2 = 1500; // Center pitch
    rcChannels.ch3 = 1000; // Min throttle (let auto-land handle it)
    rcChannels.ch4 = 1500; // Center yaw
    send_rc_override();
    
    // Release RC control
    delay(50);
    mavlink_msg_rc_channels_override_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                          TARGET_SYSTEM, TARGET_COMPONENT,
                                          0, 0, 0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    send_mavlink_message();
    
    // Send failsafe status
    sendFailsafeStatus(String("FAILSAFE_LAND:") + reason);
}

// Send failsafe status
void sendFailsafeStatus(String status) {
    if (udp.remoteIP() != IPAddress(0,0,0,0)) {
        String failsafeMsg = "FAILSAFE:" + status;
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print(failsafeMsg);
        udp.endPacket();
    }
}

void updateSmoothThrottle() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastThrottleUpdate >= THROTTLE_UPDATE_INTERVAL) {
        int throttleDiff = targetThrottle - currentSmoothThrottle;
        
        if (abs(throttleDiff) > THROTTLE_SMOOTH_RATE) {
            // Apply rate limiting
            if (throttleDiff > 0) {
                currentSmoothThrottle += THROTTLE_SMOOTH_RATE;
            } else {
                currentSmoothThrottle -= THROTTLE_SMOOTH_RATE;
            }
        } else {
            // Small change, apply directly
            currentSmoothThrottle = targetThrottle;
        }
        
        lastThrottleUpdate = currentTime;
    }
}

void send_rc_override() {
    // Don't send RC override if in failsafe
    if (failsafeActive) return;
    
    // Update smooth throttle
    targetThrottle = rcChannels.ch3;
    updateSmoothThrottle();
    
    mavlink_msg_rc_channels_override_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                          TARGET_SYSTEM, TARGET_COMPONENT,
                                          rcChannels.ch1, rcChannels.ch2, 
                                          currentSmoothThrottle, // Use smoothed throttle
                                          rcChannels.ch4,
                                          rcChannels.ch5, rcChannels.ch6,
                                          rcChannels.ch7, rcChannels.ch8,
                                          0, 0, // ch9-10 (0 = release control)
                                          0, 0, // ch11-12
                                          0, 0, // ch13-14
                                          0, 0, // ch15-16
                                          0, 0); // ch17-18
    send_mavlink_message();
}

void request_data_streams() {
    // Request all data streams at 4Hz
    mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                        TARGET_SYSTEM, TARGET_COMPONENT,
                                        MAV_DATA_STREAM_ALL, 4, 1);
    send_mavlink_message();
    
    // Also request specific streams for better compatibility
    const uint8_t streams[] = {
        MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1,
        MAV_DATA_STREAM_EXTRA2
    };
    
    for (int i = 0; i < sizeof(streams); i++) {
        mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                            TARGET_SYSTEM, TARGET_COMPONENT,
                                            streams[i], 2, 1);
        send_mavlink_message();
    }
    
    Serial.println("Requested all data streams");
}

void set_flight_mode(uint32_t mode) {
    // Set flight mode using MAV_CMD_DO_SET_MODE
    mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                  TARGET_SYSTEM, TARGET_COMPONENT,
                                  MAV_CMD_DO_SET_MODE, 0,
                                  MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                  mode, 0, 0, 0, 0, 0);
    send_mavlink_message();
    Serial.printf("Setting flight mode to %d\n", mode);
}

void start_esc_calibration() {
    Serial.println("Starting ESC Calibration");
    
    // Set throttle to maximum
    rcChannels.ch3 = 2000;
    rcOverrideActive = true;
    send_rc_override();
    
    // Send arm command with ESC calibration flag
    // Fixed: Added all required parameters
    mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                  TARGET_SYSTEM, TARGET_COMPONENT,
                                  MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                  1.0f,    // param1: arm (1) or disarm (0)
                                  21196.0f, // param2: force arm for calibration
                                  0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // param3-7
    send_mavlink_message();
    
    delay(3000); // Wait for ESCs to register max throttle
    
    // Set throttle to minimum
    rcChannels.ch3 = 1000;
    send_rc_override();
    
    delay(3000); // Wait for ESCs to register min throttle
    
    Serial.println("ESC Calibration Complete");
    rcOverrideActive = false;
    
    // Disarm
    mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                  TARGET_SYSTEM, TARGET_COMPONENT,
                                  MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                  0.0f,    // disarm
                                  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    send_mavlink_message();
}

void run_motor_test(uint8_t motor_number, uint16_t throttle_percent, uint16_t duration_ms) {
    Serial.printf("Testing motor %d at %d%% for %dms\n", 
                  motor_number, throttle_percent, duration_ms);
    
    // Fixed: Using correct parameter types and added all required parameters
    mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                  TARGET_SYSTEM, TARGET_COMPONENT,
                                  MAV_CMD_DO_MOTOR_TEST, 0,
                                  (float)motor_number,     // param1: motor number (0-based)
                                  (float)MOTOR_TEST_THROTTLE_PERCENT, // param2: test type
                                  (float)throttle_percent, // param3: throttle percentage
                                  (float)(duration_ms/1000.0), // param4: duration in seconds
                                  0.0f, 0.0f, 0.0f);      // param5-7: unused
    send_mavlink_message();
}

void send_status_response() {
    if (udp.remoteIP() == IPAddress(0,0,0,0)) return;

    String statusMsg = "STATUS:";
    statusMsg += "ARMED:" + String(droneArmed ? "1" : "0") + ";";
    statusMsg += "FLYING:" + String(droneFlying ? "1" : "0") + ";";
    statusMsg += "MODE:" + currentMode + ";";
    statusMsg += "THROTTLE:" + String(currentSmoothThrottle) + ";";
    statusMsg += "RSSI:" + String(currentRSSI) + ";";
    statusMsg += "RC_ACTIVE:" + String(rcOverrideActive ? "1" : "0") + ";";
    statusMsg += "BATTERY:" + String(batteryPercent, 1) + ";";
    statusMsg += "VOLTAGE:" + String(batteryVoltage, 2) + ";";
    statusMsg += "GPS_SAT:" + String(gpsSatellites) + ";";
    statusMsg += "LAT:" + String(gpsLatitude, 6) + ";";
    statusMsg += "LON:" + String(gpsLongitude, 6) + ";";
    statusMsg += "ALT:" + String(gpsAltitude, 1) + ";";
    statusMsg += "FAILSAFE:" + String(failsafeActive ? "1" : "0");

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print(statusMsg);
    udp.endPacket();
}

// Process MAVLink messages
void processMavlinkMessage(mavlink_message_t& mav_msg) {
    switch (mav_msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&mav_msg, &heartbeat);
            bool was_armed = droneArmed;
            droneArmed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);

            if (was_armed != droneArmed) {
                Serial.printf("ARM STATE CHANGED: %s\n", droneArmed ? "ARMED" : "DISARMED");
            }

            uint8_t base_mode = heartbeat.base_mode;
            uint32_t custom_mode = heartbeat.custom_mode;

            if (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
                 switch(custom_mode) {
                    case STABILIZE: currentMode = "STABILIZE"; break;
                    case ACRO: currentMode = "ACRO"; break;
                    case ALT_HOLD: currentMode = "ALT_HOLD"; break;
                    case AUTO: currentMode = "AUTO"; break;
                    case GUIDED: currentMode = "GUIDED"; break;
                    case LOITER: currentMode = "LOITER"; break;
                    case RTL: currentMode = "RTL"; break;
                    case CIRCLE: currentMode = "CIRCLE"; break;
                    case LAND: currentMode = "LAND"; break;
                    case DRIFT: currentMode = "DRIFT"; break;
                    case SPORT: currentMode = "SPORT"; break;
                    case POSHOLD: currentMode = "POSHOLD"; break;
                    case GUIDED_NOGPS: currentMode = "GUIDED_NOGPS"; break;
                    default: currentMode = "CUSTOM:" + String(custom_mode); break;
                 }
            }

            if (!droneArmed || currentMode == "LAND") {
                droneFlying = false;
            }
            break;
        }
        
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&mav_msg, &sys_status);
            batteryVoltage = sys_status.voltage_battery / 1000.0;
            batteryPercent = sys_status.battery_remaining;
            break;
        }
        
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&mav_msg, &gps);
            gpsLatitude = gps.lat / 1e7;
            gpsLongitude = gps.lon / 1e7;
            gpsAltitude = gps.alt / 1000.0;
            gpsSatellites = gps.satellites_visible;
            break;
        }
        
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
            mavlink_rc_channels_raw_t rc_raw;
            mavlink_msg_rc_channels_raw_decode(&mav_msg, &rc_raw);
            Serial.printf("RC Raw: CH1=%d, CH2=%d, CH3=%d, CH4=%d\n", 
                         rc_raw.chan1_raw, rc_raw.chan2_raw, rc_raw.chan3_raw, rc_raw.chan4_raw);
            break;
        }
        
        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&mav_msg, &ack);
            Serial.printf("Command ACK: ID=%d, Result=%d\n", ack.command, ack.result);
             
            if (udp.remoteIP() != IPAddress(0,0,0,0)) {
                udp.beginPacket(udp.remoteIP(), udp.remotePort());
                udp.printf("ACK: ID=%d, Result=%d", ack.command, ack.result);
                udp.endPacket();
            }
            break;
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(FC_BAUDRATE, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

    Serial.println("\nESP32 Advanced RC Controller with LAND Failsafe Starting...");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi ");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { 
        delay(500); 
        Serial.print("."); 
        attempts++; 
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!");
        Serial.print("IP Address: "); 
        Serial.println(WiFi.localIP());
        currentRSSI = WiFi.RSSI();
        wifiConnected = true;
        wifiWasConnected = true;
        udp.begin(UDP_PORT);
        Serial.printf("UDP Listener started on port %d\n", UDP_PORT);
        
        // Initialize lastCommandTime to prevent immediate failsafe
        lastCommandTime = millis();
        
        // Request data streams from FC
        delay(1000);
        request_data_streams();
    } else {
        Serial.println("\nWiFi Connection FAILED!");
        wifiConnected = false;
        wifiWasConnected = false;
    }
}

void loop() {
    unsigned long currentTime = millis();

    // Check failsafe conditions first
    checkFailsafe();

    // Send heartbeat
    if (currentTime - lastHeartbeatTime > HEARTBEAT_INTERVAL) {
        mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, 
                                   MAV_TYPE_GCS, // Ground Control Station
                                   MAV_AUTOPILOT_INVALID, 
                                   MAV_MODE_FLAG_SAFETY_ARMED, 
                                   0, 0);
        send_mavlink_message();
        lastHeartbeatTime = currentTime;
    }

    // Send continuous RC override when active (and not in failsafe)
    if (rcOverrideActive && !failsafeActive && 
        (currentTime - lastRCOverrideTime > RC_OVERRIDE_RATE)) {
        send_rc_override();
        lastRCOverrideTime = currentTime;
    }

    // Process UDP commands
    if (wifiConnected) {
        int packetSize = udp.parsePacket();
        if (packetSize) {
            char incomingPacket[255];
            int len = udp.read(incomingPacket, 254);
            if (len > 0) {
                incomingPacket[len] = '\0';
                String command = String(incomingPacket);
                command.trim();
                lastCommandTime = currentTime; // Update command received time
                processCommand(command);
            }
        }
        
        // Update RSSI
        static unsigned long lastRSSICheck = 0;
        if (currentTime - lastRSSICheck > 2000) {
            currentRSSI = WiFi.RSSI();
            lastRSSICheck = currentTime;
        }
    }

    // Process MAVLink from FC
    while (Serial2.available()) {
        uint8_t c = Serial2.read();
        mavlink_message_t received_msg;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &received_msg, &status)) {
            processMavlinkMessage(received_msg);
        }
    }

    delay(5);
}

// Command processing
void processCommand(String command) {
    Serial.printf("Processing command: %s\n", command.c_str());
    
    // Don't process most commands if in failsafe (except STATUS)
    if (failsafeActive && command != "STATUS") {
        Serial.println("Ignoring command - failsafe active");
        return;
    }
    
    if (command == "ARM") {
        Serial.println("Sending ARM command");
        mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, TARGET_COMPONENT, 
                                      MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                      1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        send_mavlink_message();
        
    } else if (command == "DISARM") {
        Serial.println("Sending DISARM command");
        rcOverrideActive = false; // Stop RC override when disarming
        rcChannels.ch3 = 1000; // Set throttle to minimum
        currentSmoothThrottle = 1000;
        targetThrottle = 1000;
        mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM, TARGET_COMPONENT, 
                                      MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        send_mavlink_message();
        
    } else if (command == "START_OVERRIDE") {
        Serial.println("Starting continuous RC override");
        rcOverrideActive = true;
        
    } else if (command == "STOP_OVERRIDE") {
        Serial.println("Stopping RC override");
        rcOverrideActive = false;
        // Send one last command to release control
        mavlink_msg_rc_channels_override_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                              TARGET_SYSTEM, TARGET_COMPONENT,
                                              0, 0, 0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        send_mavlink_message();
        
    } else if (command.startsWith("RC_CHANNELS:")) {
        // Parse all 8 channels: ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8
        String channelData = command.substring(12);
        int pos = 0;
        int channelIndex = 1;
        
        while (pos < channelData.length() && channelIndex <= 8) {
            int commaPos = channelData.indexOf(',', pos);
            if (commaPos == -1) commaPos = channelData.length();
            
            int value = channelData.substring(pos, commaPos).toInt();
            
            switch (channelIndex) {
                case 1: rcChannels.ch1 = value; break;
                case 2: rcChannels.ch2 = value; break;
                case 3: rcChannels.ch3 = value; break;
                case 4: rcChannels.ch4 = value; break;
                case 5: rcChannels.ch5 = value; break;
                case 6: rcChannels.ch6 = value; break;
                case 7: rcChannels.ch7 = value; break;
                case 8: rcChannels.ch8 = value; break;
            }
            
            pos = commaPos + 1;
            channelIndex++;
        }
        
    } else if (command.startsWith("THROTTLE:")) {
        int throttle = command.substring(9).toInt();
        if (throttle >= 1000 && throttle <= 2000) {
            rcChannels.ch3 = throttle;
            Serial.printf("Throttle updated to: %d\n", throttle);
        }
        
    } else if (command.startsWith("THROTTLE_MODE:")) {
        String mode = command.substring(14);
        if (mode == "SMOOTH") {
            THROTTLE_SMOOTH_RATE = 5;  // Very smooth
            Serial.println("Throttle mode: SMOOTH");
        } else if (mode == "NORMAL") {
            THROTTLE_SMOOTH_RATE = 10; // Normal smoothing
            Serial.println("Throttle mode: NORMAL");
        } else if (mode == "SPORT") {
            THROTTLE_SMOOTH_RATE = 20; // Faster response
            Serial.println("Throttle mode: SPORT");
        }
        
    } else if (command == "START_ESC_CAL") {
        start_esc_calibration();
        
    } else if (command.startsWith("MOTOR_TEST:")) {
        // Format: MOTOR_TEST:motor_num,throttle_percent,duration_ms
        String params = command.substring(11);
        int firstComma = params.indexOf(',');
        int secondComma = params.indexOf(',', firstComma + 1);
        
        if (firstComma > 0 && secondComma > firstComma) {
            uint8_t motor = params.substring(0, firstComma).toInt();
            uint16_t throttle = params.substring(firstComma + 1, secondComma).toInt();
            uint16_t duration = params.substring(secondComma + 1).toInt();
            
            run_motor_test(motor, throttle, duration);
        }
        
    } else if (command == "MODE_STABILIZE") {
        set_flight_mode(STABILIZE);
    } else if (command == "MODE_ACRO") {
        set_flight_mode(ACRO);
    } else if (command == "MODE_ALT_HOLD") {
        set_flight_mode(ALT_HOLD);
    } else if (command == "MODE_LOITER") {
        set_flight_mode(LOITER);
    } else if (command == "MODE_RTL") {
        set_flight_mode(RTL);
    } else if (command == "MODE_LAND") {
        set_flight_mode(LAND);
    } else if (command == "MODE_POSHOLD") {
        set_flight_mode(POSHOLD);
    } else if (command == "MODE_GUIDED") {
        set_flight_mode(GUIDED);
    } else if (command == "MODE_AUTO") {
        set_flight_mode(AUTO);
        
    } else if (command == "TEST_FAILSAFE") {
        Serial.println("Testing failsafe - simulating signal loss");
        executeFailsafe("TEST");
        
    } else if (command == "DEBUG") {
        Serial.println("=== DEBUG INFO ===");
        Serial.printf("Armed: %s\n", droneArmed ? "YES" : "NO");
        Serial.printf("Flying: %s\n", droneFlying ? "YES" : "NO");
        Serial.printf("Mode: %s\n", currentMode.c_str());
        Serial.printf("Channels: %d,%d,%d,%d,%d,%d,%d,%d\n", 
                      rcChannels.ch1, rcChannels.ch2, rcChannels.ch3, rcChannels.ch4,
                      rcChannels.ch5, rcChannels.ch6, rcChannels.ch7, rcChannels.ch8);
        Serial.printf("Throttle (target/smooth): %d/%d\n", targetThrottle, currentSmoothThrottle);
        Serial.printf("RC Override Active: %s\n", rcOverrideActive ? "YES" : "NO");
        Serial.printf("WiFi RSSI: %d dBm\n", currentRSSI);
        Serial.printf("Battery: %.2fV, %.1f%%\n", batteryVoltage, batteryPercent);
        Serial.printf("GPS: %d sats, %.6f, %.6f, %.1fm\n", 
                      gpsSatellites, gpsLatitude, gpsLongitude, gpsAltitude);
        Serial.printf("Failsafe Active: %s\n", failsafeActive ? "YES" : "NO");
        Serial.printf("Last Command: %lu ms ago\n", millis() - lastCommandTime);
        
    } else if (command == "STATUS") {
        send_status_response();
    }
}