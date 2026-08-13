#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define DEBOUNCE_MS 50
#define PAIRING_TIMEOUT_MS 25000U
#define PAIRING_CONFIRM_MS 300U 
#define PAIRING_LED_BLINK_MS 120U
#define WIFI_STUCK_MS 180000U
#define SWITCH1_PAIR_TOGGLES 15
#define SWITCH2_RESET_TOGGLES 20

const char* mqtt_server = "i26a1c71.ala.asia-southeast1.emqxsl.com";
const int mqtt_port = 8883;
const char* mqtt_user = "smartnest_client";
const char* mqtt_pass = "D2m9ga8JynJDEM6";

char NODE_ID[32];        
char command_topic[100]; 
char status_topic[100];  

WiFiClientSecure espClient;
PubSubClient client(espClient);
Preferences preferences;

const int relay1 = 15; 
const int relay2 = 5; 
const int relay3 = 4;  
const int relay4 = 22;

const int Speed1 = 21; 
const int Speed2 = 19; 
const int Speed4 = 18;

const int switch1 = 32; 
const int switch2 = 35; 
const int switch3 = 34; 
const int switch4 = 39; 
const int fan_switch = 33;

const int s1 = 27; 
const int s2 = 14; 
const int s3 = 12; 
const int s4 = 13;

const int gpio_reset = 0; 
const int wifiLed = 2;
const int RF_PIN = 23;

portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

bool switch_state_ch1 = false; 
bool switch_state_ch2 = false;
bool switch_state_ch3 = false; 
bool switch_state_ch4 = false;

int curr_speed = 0; 
int fan_speed_memory = 1; 
bool fan_power = false;

bool speed1_flag = 1;
bool speed2_flag = 1;
bool speed3_flag = 1;
bool speed4_flag = 1;
bool speed0_flag = 1;

uint32_t rf_code_l1 = 0;
uint32_t rf_code_l2 = 0;
uint32_t rf_code_l3 = 0;
uint32_t rf_code_l4 = 0;
uint32_t rf_code_up = 0;
uint32_t rf_code_dw = 0;
uint32_t rf_code_fan_toggle = 0;
uint32_t rf_code_master = 0;

volatile uint32_t rf_received_value = 0; 
volatile bool rf_available = false;

int pairing_target = 0; 
uint64_t pairing_timeout = 0; 
uint64_t pairing_confirm_until_ms = 0;

uint64_t last_switch1_toggle_time = 0;
uint64_t last_switch2_toggle_time = 0;

int switch1_toggle_count = 0;
int switch2_toggle_count = 0;

volatile int pending_fan_speed = -1;

static TaskHandle_t bulk_on_handle = NULL; 
static TaskHandle_t bulk_off_handle = NULL;

bool nvs_dirty = false;
uint64_t nvs_dirty_time = 0;

#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb"
#define WIFI_CHAR_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"
#define MAC_CHAR_UUID "0000ffe2-0000-1000-8000-00805f9b34fb"
#define DEVICE_ID_CHAR_UUID "0000ffe3-0000-1000-8000-00805f9b34fb"

bool inSetupMode = false; 
bool shouldReboot = false; 
String saved_ssid = ""; 
String saved_password = "";

uint8_t load_state_from_nvs(const char *key, uint8_t def) {
    preferences.begin("storage", true);
    uint8_t val = preferences.getUInt(key, def);
    preferences.end();
    return val;
}

void save_state_to_nvs(const char *key, uint8_t value) {
    preferences.begin("storage", false);
    preferences.putUInt(key, value);
    preferences.end();
}

uint32_t load_code(const char *key, uint32_t def) {
    preferences.begin("codes", true);
    uint32_t val = preferences.getUInt(key, def);
    preferences.end();
    return val;
}

void save_code(const char *key, uint32_t val) {
    preferences.begin("codes", false);
    preferences.putUInt(key, val);
    preferences.end();
}

void schedule_nvs_save() {
    portENTER_CRITICAL(&state_mux);
    nvs_dirty = true;
    portEXIT_CRITICAL(&state_mux);
    
    nvs_dirty_time = millis();
}

void sendChannelState(int channel, bool status, int val = -1) {
    StaticJsonDocument<200> doc;
    doc["channel"] = channel;
    
    if (status) {
        doc["status"] = "ON";
    } else {
        doc["status"] = "OFF";
    }
    
    if (val != -1 && channel == 5) {
        doc["speed"] = val;
    }

    char buffer[256];
    serializeJson(doc, buffer);
    
    if (client.connected()) {
        client.publish(status_topic, buffer, true);
    }
}

void pref_save_fan() {
    int safe_speed; 
    bool safe_power;
    
    portENTER_CRITICAL(&state_mux);
    safe_speed = curr_speed; 
    safe_power = fan_power;
    portEXIT_CRITICAL(&state_mux);
    
    if (safe_speed > 0) {
        portENTER_CRITICAL(&state_mux); 
        fan_speed_memory = safe_speed; 
        portEXIT_CRITICAL(&state_mux);
    }
    
    schedule_nvs_save();
    sendChannelState(5, safe_power, safe_speed);
}

void set_fan_relays(int t_s1, int t_s2, int t_s4) {
    digitalWrite(Speed1, LOW); 
    digitalWrite(Speed2, LOW); 
    digitalWrite(Speed4, LOW);
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    if (t_s1 == 1) {
        digitalWrite(Speed1, HIGH);
    }
    if (t_s2 == 1) {
        digitalWrite(Speed2, HIGH);
    }
    if (t_s4 == 1) {
        digitalWrite(Speed4, HIGH);
    }
}

void speed_0() { 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 0; 
    fan_power = false; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(0, 0, 0); 
    pref_save_fan(); 
}

void speed_1() { 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 1; 
    fan_power = true; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(1, 0, 0); 
    pref_save_fan(); 
}

void speed_2() { 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 2; 
    fan_power = true; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(0, 1, 0); 
    pref_save_fan(); 
}

void speed_3() { 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 3; 
    fan_power = true; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(1, 1, 0); 
    pref_save_fan(); 
}

void speed_4() { 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 4; 
    fan_power = true; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(0, 0, 1); 
    pref_save_fan(); 
}

void restore_fan_speed() {
    int mem_speed;
    
    portENTER_CRITICAL(&state_mux);
    if (fan_speed_memory < 1 || fan_speed_memory > 4) {
        fan_speed_memory = 1;
    }
    mem_speed = fan_speed_memory;
    portEXIT_CRITICAL(&state_mux);
    
    if (mem_speed == 1) {
        speed_1();
    } else if (mem_speed == 2) {
        speed_2();
    } else if (mem_speed == 3) {
        speed_3();
    } else if (mem_speed == 4) {
        speed_4();
    }
}

static void bulk_on_task(void *pv) {
    Serial.println("⚙️ [SYSTEM] Master Bulk ON Triggered");
    
    portENTER_CRITICAL(&state_mux); 
    switch_state_ch1 = 1; 
    portEXIT_CRITICAL(&state_mux);
    
    digitalWrite(relay1, HIGH); 
    sendChannelState(1, true); 
    schedule_nvs_save(); 
    vTaskDelay(pdMS_TO_TICKS(300)); 
    
    portENTER_CRITICAL(&state_mux); 
    switch_state_ch2 = 1; 
    portEXIT_CRITICAL(&state_mux);
    
    digitalWrite(relay2, HIGH); 
    sendChannelState(2, true); 
    schedule_nvs_save(); 
    vTaskDelay(pdMS_TO_TICKS(300));
    
    portENTER_CRITICAL(&state_mux); 
    switch_state_ch3 = 1; 
    portEXIT_CRITICAL(&state_mux);
    
    digitalWrite(relay3, HIGH); 
    sendChannelState(3, true); 
    schedule_nvs_save(); 
    vTaskDelay(pdMS_TO_TICKS(300));
    
    portENTER_CRITICAL(&state_mux); 
    switch_state_ch4 = 1; 
    portEXIT_CRITICAL(&state_mux);
    
    digitalWrite(relay4, HIGH); 
    sendChannelState(4, true); 
    schedule_nvs_save(); 
    vTaskDelay(pdMS_TO_TICKS(300));
    
    bool f_pow; 
    
    portENTER_CRITICAL(&state_mux); 
    f_pow = fan_power; 
    portEXIT_CRITICAL(&state_mux);
    
    if (!f_pow) {
        restore_fan_speed();
    }
    
    sendChannelState(6, true);
    
    portENTER_CRITICAL(&state_mux); 
    bulk_on_handle = NULL; 
    portEXIT_CRITICAL(&state_mux);
    
    vTaskDelete(NULL);
}

static void bulk_off_task(void *pv) {
    Serial.println("⚙️ [SYSTEM] Master Bulk OFF Triggered");
    
    portENTER_CRITICAL(&state_mux); 
    switch_state_ch1 = 0; 
    portEXIT_CRITICAL(&state_mux);
    
    digitalWrite(relay1, LOW); 
    sendChannelState(1, false); 
    schedule_nvs_save(); 
    vTaskDelay(pdMS_TO_TICKS(300));
    
    portENTER_CRITICAL(&state_mux); 
    switch_state_ch2 = 0; 
    portEXIT_CRITICAL(&state_mux);
    
    digitalWrite(relay2, LOW); 
    sendChannelState(2, false); 
    schedule_nvs_save(); 
    vTaskDelay(pdMS_TO_TICKS(300));
    
    portENTER_CRITICAL(&state_mux); 
    switch_state_ch3 = 0; 
    portEXIT_CRITICAL(&state_mux);
    
    digitalWrite(relay3, LOW); 
    sendChannelState(3, false); 
    schedule_nvs_save(); 
    vTaskDelay(pdMS_TO_TICKS(300));
    
    portENTER_CRITICAL(&state_mux); 
    switch_state_ch4 = 0; 
    portEXIT_CRITICAL(&state_mux);
    
    digitalWrite(relay4, LOW); 
    sendChannelState(4, false); 
    schedule_nvs_save(); 
    vTaskDelay(pdMS_TO_TICKS(300));
    
    speed_0(); 
    sendChannelState(6, false);
    
    portENTER_CRITICAL(&state_mux); 
    bulk_off_handle = NULL; 
    portEXIT_CRITICAL(&state_mux);
    
    vTaskDelete(NULL);
}

void All_On() { 
    portENTER_CRITICAL(&state_mux); 
    
    if (bulk_on_handle != NULL || bulk_off_handle != NULL) { 
        portEXIT_CRITICAL(&state_mux); 
        return; 
    } 
    
    portEXIT_CRITICAL(&state_mux); 
    
    if (xTaskCreate(bulk_on_task, "bulk_on", 8192, NULL, 3, &bulk_on_handle) != pdPASS) { 
        portENTER_CRITICAL(&state_mux); 
        bulk_on_handle = NULL; 
        portEXIT_CRITICAL(&state_mux); 
    } 
}

void All_Off() { 
    portENTER_CRITICAL(&state_mux); 
    
    if (bulk_off_handle != NULL || bulk_on_handle != NULL) { 
        portEXIT_CRITICAL(&state_mux); 
        return; 
    } 
    
    portEXIT_CRITICAL(&state_mux); 
    
    if (xTaskCreate(bulk_off_task, "bulk_off", 8192, NULL, 3, &bulk_off_handle) != pdPASS) { 
        portENTER_CRITICAL(&state_mux); 
        bulk_off_handle = NULL; 
        portEXIT_CRITICAL(&state_mux); 
    } 
}

void IRAM_ATTR rf_isr() {
    if (rf_available == true) {
        return;
    }
    
    static uint32_t lt = 0; 
    static uint16_t tm[64]; 
    static uint8_t ct = 0;
    
    uint32_t now = micros(); 
    uint32_t df = now - lt; 
    lt = now;
    
    if (df < 150) { 
        ct = 0; 
        return; 
    }
    
    if (df > 4000 && df < 20000) {
        if (ct >= 48) {
            uint32_t c = 0;
            
            for (int i = 0; i < 48; i += 2) {
                c = (c << 1) | (tm[i] > tm[i + 1]);
            }
            
            if (c > 0 && c != 0xFFFFFFFF) { 
                rf_received_value = c; 
                rf_available = true; 
            }
        }
        ct = 0;
    } else if (ct < 60) {
        tm[ct++] = df;
    }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<256> doc;
    
    if (deserializeJson(doc, payload, length)) {
        return;
    }

    if (doc.containsKey("action") && doc["action"] == "factory_reset") {
        Serial.println("🚨 [APP/CLOUD] Factory Reset Command Received!");
        preferences.begin("wifi", false); 
        preferences.clear(); 
        preferences.end(); 
        delay(1000); 
        ESP.restart();
    }
    
    if (!doc.containsKey("channel") || !doc.containsKey("status")) {
        return;
    }

    int channel = doc["channel"]; 
    bool state = false;
    
    if (strcmp(doc["status"], "ON") == 0) {
        state = true;
    }
    
    if (state) {
        Serial.printf("📱 [APP/CLOUD] Command: Channel %d -> ON\n", channel);
    } else {
        Serial.printf("📱 [APP/CLOUD] Command: Channel %d -> OFF\n", channel);
    }

    if (channel == 1) { 
        portENTER_CRITICAL(&state_mux); 
        switch_state_ch1 = state; 
        portEXIT_CRITICAL(&state_mux);
        
        if (state) {
            digitalWrite(relay1, HIGH);
        } else {
            digitalWrite(relay1, LOW);
        }
        
        sendChannelState(1, state); 
        schedule_nvs_save(); 
    }
    else if (channel == 2) { 
        portENTER_CRITICAL(&state_mux); 
        switch_state_ch2 = state; 
        portEXIT_CRITICAL(&state_mux);
        
        if (state) {
            digitalWrite(relay2, HIGH);
        } else {
            digitalWrite(relay2, LOW);
        }
        
        sendChannelState(2, state); 
        schedule_nvs_save(); 
    }
    else if (channel == 3) { 
        portENTER_CRITICAL(&state_mux); 
        switch_state_ch3 = state; 
        portEXIT_CRITICAL(&state_mux);
        
        if (state) {
            digitalWrite(relay3, HIGH);
        } else {
            digitalWrite(relay3, LOW);
        }
        
        sendChannelState(3, state); 
        schedule_nvs_save(); 
    }
    else if (channel == 4) { 
        portENTER_CRITICAL(&state_mux); 
        switch_state_ch4 = state; 
        portEXIT_CRITICAL(&state_mux);
        
        if (state) {
            digitalWrite(relay4, HIGH);
        } else {
            digitalWrite(relay4, LOW);
        }
        
        sendChannelState(4, state); 
        schedule_nvs_save(); 
    }
    else if (channel == 5) { 
        if (doc.containsKey("speed")) {
            int spd = doc["speed"];
            Serial.printf("📱 [APP/CLOUD] Command: Fan Speed -> %d\n", spd);
            
            portENTER_CRITICAL(&state_mux); 
            pending_fan_speed = spd; 
            portEXIT_CRITICAL(&state_mux);
        } else {
            portENTER_CRITICAL(&state_mux); 
            if (state) {
                pending_fan_speed = fan_speed_memory;
            } else {
                pending_fan_speed = 0;
            }
            portEXIT_CRITICAL(&state_mux);
        }
    }
    else if (channel == 6) {
        if (state) {
            All_On();
        } else {
            All_Off();
        }
    }
}

void system_task(void *arg) {
    int lsw1 = digitalRead(switch1); 
    int lsw2 = digitalRead(switch2);
    int lsw3 = digitalRead(switch3); 
    int lsw4 = digitalRead(switch4);
    int lfan = digitalRead(fan_switch);

    speed1_flag = 1; 
    speed2_flag = 1; 
    speed3_flag = 1; 
    speed4_flag = 1; 
    speed0_flag = 1;
    
    if (digitalRead(s1) == LOW) {
        speed1_flag = 0;
    } else if (digitalRead(s2) == LOW && digitalRead(s3) == HIGH) {
        speed2_flag = 0;
    } else if (digitalRead(s2) == LOW && digitalRead(s3) == LOW) {
        speed3_flag = 0;
    } else if (digitalRead(s4) == LOW) {
        speed4_flag = 0;
    } else {
        speed0_flag = 0;
    }

    static uint64_t reset_press_start = 0; 
    static bool reset_triggered = false; 
    static uint32_t last_valid_code = 0; 
    static uint64_t last_valid_rf_time = 0;
    
    static uint64_t last_deb_sw1 = 0;
    static uint64_t last_deb_sw2 = 0;
    static uint64_t last_deb_sw3 = 0;
    static uint64_t last_deb_sw4 = 0;
    static uint64_t last_deb_fan = 0;

    Serial.println("⚙️ [SYSTEM] Dual-Core Hardware Task Started on Core 1!");

    while (1) {
        uint64_t now_ms = millis();

        if (pairing_target > 0) {
            if (now_ms - pairing_timeout > PAIRING_TIMEOUT_MS) {
                Serial.println("⏳ [SYSTEM] RF Pairing Timeout! Exiting mode.");
                pairing_target = 0; 
            }
        }

        int local_pending_fan = -1;
        
        portENTER_CRITICAL(&state_mux); 
        if (pending_fan_speed != -1) { 
            local_pending_fan = pending_fan_speed; 
            pending_fan_speed = -1; 
        } 
        portEXIT_CRITICAL(&state_mux);
        
        if (local_pending_fan != -1) {
            if (local_pending_fan == 1) {
                speed_1();
            } else if (local_pending_fan == 2) {
                speed_2();
            } else if (local_pending_fan == 3) {
                speed_3();
            } else if (local_pending_fan == 4) {
                speed_4();
            } else {
                speed_0();
            }
        }

        if (rf_available) {
            uint32_t code = rf_received_value;

            if (pairing_target > 0) {
                if (code == last_valid_code && (now_ms - last_valid_rf_time < 2000)) { 
                    rf_available = false; 
                }
                else {
                    last_valid_code = code; 
                    last_valid_rf_time = now_ms;
                    Serial.printf("📻 [RF PAIRING] Saving Code %lu to Slot %d\n", (unsigned long)code, pairing_target);
                    
                    switch (pairing_target) {
                        case 1: 
                            rf_code_l1 = code; 
                            save_code("rf1", code); 
                            break;
                        case 2: 
                            rf_code_l2 = code; 
                            save_code("rf2", code); 
                            break;
                        case 3: 
                            rf_code_l3 = code; 
                            save_code("rf3", code); 
                            break;
                        case 4: 
                            rf_code_l4 = code; 
                            save_code("rf4", code); 
                            break;
                        case 5: 
                            rf_code_up = code; 
                            save_code("rfu", code); 
                            break;
                        case 6: 
                            rf_code_dw = code; 
                            save_code("rfd", code); 
                            break;
                        case 7: 
                            rf_code_fan_toggle = code; 
                            save_code("rft", code); 
                            break;
                        case 8: 
                            rf_code_master = code; 
                            save_code("rfm", code); 
                            break;
                    }
                    
                    pairing_target++; 
                    pairing_timeout = now_ms; 
                    pairing_confirm_until_ms = now_ms + PAIRING_CONFIRM_MS;
                    
                    if (pairing_target > 8) { 
                        pairing_target = 0; 
                        Serial.println("✅ [RF PAIRING] Complete!"); 
                    }
                    
                    rf_available = false;
                }
            } 
            else {
                bool is_known_code = false;
                
                if (code == rf_code_l1 || code == rf_code_l2 || code == rf_code_l3 || code == rf_code_l4 || code == rf_code_up || code == rf_code_dw || code == rf_code_fan_toggle || code == rf_code_master) {
                    is_known_code = true;
                }

                if (is_known_code) {
                    if (code == last_valid_code && (now_ms - last_valid_rf_time < 1000)) { 
                        last_valid_rf_time = now_ms; 
                    } 
                    else {
                        last_valid_code = code; 
                        last_valid_rf_time = now_ms;

                        if (code == rf_code_l1) { 
                            portENTER_CRITICAL(&state_mux); 
                            switch_state_ch1 = !switch_state_ch1; 
                            bool st = switch_state_ch1; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (st) {
                                Serial.printf("📻 [RF REMOTE] Switch 1 -> ON\n");
                                digitalWrite(relay1, HIGH);
                            } else {
                                Serial.printf("📻 [RF REMOTE] Switch 1 -> OFF\n");
                                digitalWrite(relay1, LOW);
                            }
                            
                            sendChannelState(1, st); 
                            schedule_nvs_save(); 
                        }
                        else if (code == rf_code_l2) { 
                            portENTER_CRITICAL(&state_mux); 
                            switch_state_ch2 = !switch_state_ch2; 
                            bool st = switch_state_ch2; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (st) {
                                Serial.printf("📻 [RF REMOTE] Switch 2 -> ON\n");
                                digitalWrite(relay2, HIGH);
                            } else {
                                Serial.printf("📻 [RF REMOTE] Switch 2 -> OFF\n");
                                digitalWrite(relay2, LOW);
                            }
                            
                            sendChannelState(2, st); 
                            schedule_nvs_save(); 
                        }
                        else if (code == rf_code_l3) { 
                            portENTER_CRITICAL(&state_mux); 
                            switch_state_ch3 = !switch_state_ch3; 
                            bool st = switch_state_ch3; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (st) {
                                Serial.printf("📻 [RF REMOTE] Switch 3 -> ON\n");
                                digitalWrite(relay3, HIGH);
                            } else {
                                Serial.printf("📻 [RF REMOTE] Switch 3 -> OFF\n");
                                digitalWrite(relay3, LOW);
                            }
                            
                            sendChannelState(3, st); 
                            schedule_nvs_save(); 
                        }
                        else if (code == rf_code_l4) { 
                            portENTER_CRITICAL(&state_mux); 
                            switch_state_ch4 = !switch_state_ch4; 
                            bool st = switch_state_ch4; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (st) {
                                Serial.printf("📻 [RF REMOTE] Switch 4 -> ON\n");
                                digitalWrite(relay4, HIGH);
                            } else {
                                Serial.printf("📻 [RF REMOTE] Switch 4 -> OFF\n");
                                digitalWrite(relay4, LOW);
                            }
                            
                            sendChannelState(4, st); 
                            schedule_nvs_save(); 
                        }
                        else if (code == rf_code_up) { 
                            int l_speed; 
                            
                            portENTER_CRITICAL(&state_mux); 
                            l_speed = curr_speed; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (l_speed < 4) { 
                                l_speed++; 
                                Serial.printf("📻 [RF REMOTE] Fan Speed UP -> %d\n", l_speed);
                                
                                if (l_speed == 1) {
                                    speed_1();
                                } else if (l_speed == 2) {
                                    speed_2();
                                } else if (l_speed == 3) {
                                    speed_3();
                                } else {
                                    speed_4();
                                }
                            } 
                        }
                        else if (code == rf_code_dw) { 
                            int l_speed; 
                            
                            portENTER_CRITICAL(&state_mux); 
                            l_speed = curr_speed; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (l_speed > 0) { 
                                l_speed--; 
                                Serial.printf("📻 [RF REMOTE] Fan Speed DOWN -> %d\n", l_speed);
                                
                                if (l_speed == 0) {
                                    speed_0();
                                } else if (l_speed == 1) {
                                    speed_1();
                                } else if (l_speed == 2) {
                                    speed_2();
                                } else {
                                    speed_3();
                                }
                            } 
                        }
                        else if (code == rf_code_fan_toggle) { 
                            bool f_pow; 
                            
                            portENTER_CRITICAL(&state_mux); 
                            f_pow = fan_power; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (!f_pow) {
                                Serial.printf("📻 [RF REMOTE] Fan Toggle -> ON\n");
                            } else {
                                Serial.printf("📻 [RF REMOTE] Fan Toggle -> OFF\n");
                            }
                            
                            if (f_pow) {
                                speed_0();
                            } else {
                                restore_fan_speed();
                            }
                        }
                        else if (code == rf_code_master) { 
                            Serial.println("📻 [RF REMOTE] Master ON/OFF Triggered!");
                            
                            bool s1;
                            bool s2;
                            bool s3;
                            bool s4;
                            bool f_pow; 
                            
                            portENTER_CRITICAL(&state_mux); 
                            s1 = switch_state_ch1; 
                            s2 = switch_state_ch2; 
                            s3 = switch_state_ch3; 
                            s4 = switch_state_ch4; 
                            f_pow = fan_power; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (s1 || s2 || s3 || s4 || f_pow) {
                                All_Off();
                            } else {
                                All_On();
                            }
                        }
                        
                        pairing_confirm_until_ms = now_ms + 200U; 
                    }
                }
                
                rf_available = false;
            }
        }

        int csw1 = digitalRead(switch1);
        
        if (csw1 != lsw1 && (now_ms - last_deb_sw1 > DEBOUNCE_MS)) {
            last_deb_sw1 = now_ms;
            lsw1 = csw1; 
            
            portENTER_CRITICAL(&state_mux); 
            
            if (csw1 == 0) {
                switch_state_ch1 = true;
            } else {
                switch_state_ch1 = false;
            }
            
            bool st = switch_state_ch1; 
            portEXIT_CRITICAL(&state_mux);
            
            if (st) {
                Serial.printf("🔘 [PHYSICAL SWITCH] Button 1 -> ON\n");
                digitalWrite(relay1, HIGH);
            } else {
                Serial.printf("🔘 [PHYSICAL SWITCH] Button 1 -> OFF\n");
                digitalWrite(relay1, LOW);
            }
            
            sendChannelState(1, st); 
            schedule_nvs_save(); 
            
            if (pairing_target == 0) {
                uint32_t t_diff = now_ms - last_switch1_toggle_time;
                
                if (t_diff > 1500) {
                    switch1_toggle_count = 1; 
                }
                else if (t_diff > 100) {
                    switch1_toggle_count++; 
                }
                
                last_switch1_toggle_time = now_ms;
                
                if (switch1_toggle_count >= SWITCH1_PAIR_TOGGLES) { 
                    Serial.println("⚙️ [SYSTEM] Triggering RF Pairing Mode via Switch 1!");
                    pairing_target = 1; 
                    pairing_timeout = now_ms; 
                    switch1_toggle_count = 0; 
                    pairing_confirm_until_ms = now_ms + PAIRING_CONFIRM_MS; 
                }
            }
        }

        int csw2 = digitalRead(switch2);
        
        if (csw2 != lsw2 && (now_ms - last_deb_sw2 > DEBOUNCE_MS)) {
            last_deb_sw2 = now_ms;
            lsw2 = csw2; 
            
            portENTER_CRITICAL(&state_mux); 
            
            if (csw2 == 0) {
                switch_state_ch2 = true;
            } else {
                switch_state_ch2 = false;
            }
            
            bool st = switch_state_ch2; 
            portEXIT_CRITICAL(&state_mux);
            
            if (st) {
                Serial.printf("🔘 [PHYSICAL SWITCH] Button 2 -> ON\n");
                digitalWrite(relay2, HIGH);
            } else {
                Serial.printf("🔘 [PHYSICAL SWITCH] Button 2 -> OFF\n");
                digitalWrite(relay2, LOW);
            }
            
            sendChannelState(2, st); 
            schedule_nvs_save(); 
            
            if (pairing_target == 0) {
                uint32_t t_diff = now_ms - last_switch2_toggle_time;
                
                if (t_diff > 1500) {
                    switch2_toggle_count = 1; 
                }
                else if (t_diff > 100) {
                    switch2_toggle_count++; 
                }
                
                last_switch2_toggle_time = now_ms;
                
                if (switch2_toggle_count >= SWITCH2_RESET_TOGGLES) { 
                    Serial.println("🚨 [SYSTEM] FACTORY RESET VIA SWITCH 2!");
                    preferences.begin("wifi", false); 
                    preferences.clear(); 
                    preferences.end(); 
                    delay(500); 
                    ESP.restart(); 
                }
            }
        }

        int csw3 = digitalRead(switch3);
        
        if (csw3 != lsw3 && (now_ms - last_deb_sw3 > DEBOUNCE_MS)) {
            last_deb_sw3 = now_ms;
            lsw3 = csw3; 
            
            portENTER_CRITICAL(&state_mux); 
            
            if (csw3 == 0) {
                switch_state_ch3 = true;
            } else {
                switch_state_ch3 = false;
            }
            
            bool st = switch_state_ch3; 
            portEXIT_CRITICAL(&state_mux); 
            
            if (st) {
                Serial.printf("🔘 [PHYSICAL SWITCH] Button 3 -> ON\n");
                digitalWrite(relay3, HIGH);
            } else {
                Serial.printf("🔘 [PHYSICAL SWITCH] Button 3 -> OFF\n");
                digitalWrite(relay3, LOW);
            }
            
            sendChannelState(3, st); 
            schedule_nvs_save(); 
        }

        int csw4 = digitalRead(switch4);
        
        if (csw4 != lsw4 && (now_ms - last_deb_sw4 > DEBOUNCE_MS)) {
            last_deb_sw4 = now_ms;
            lsw4 = csw4; 
            
            portENTER_CRITICAL(&state_mux); 
            
            if (csw4 == 0) {
                switch_state_ch4 = true;
            } else {
                switch_state_ch4 = false;
            }
            
            bool st = switch_state_ch4; 
            portEXIT_CRITICAL(&state_mux); 
            
            if (st) {
                Serial.printf("🔘 [PHYSICAL SWITCH] Button 4 -> ON\n");
                digitalWrite(relay4, HIGH);
            } else {
                Serial.printf("🔘 [PHYSICAL SWITCH] Button 4 -> OFF\n");
                digitalWrite(relay4, LOW);
            }
            
            sendChannelState(4, st); 
            schedule_nvs_save(); 
        }

        int cf_sw = digitalRead(fan_switch);
        
        if (cf_sw != lfan && (now_ms - last_deb_fan > DEBOUNCE_MS)) {
            last_deb_fan = now_ms;
            lfan = cf_sw; 
            
            if (cf_sw == 0) { 
                Serial.println("🔘 [PHYSICAL SWITCH] Fan Toggle ON");
                int l_speed; 
                
                portENTER_CRITICAL(&state_mux); 
                l_speed = curr_speed; 
                portEXIT_CRITICAL(&state_mux); 
                
                if (l_speed == 0) {
                    restore_fan_speed(); 
                }
            } else { 
                Serial.println("🔘 [PHYSICAL SWITCH] Fan Toggle OFF");
                speed_0(); 
            } 
        }

        if (digitalRead(s1) == LOW && speed1_flag == 1) { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary -> Speed 1"); 
            speed_1(); 
            speed1_flag = 0; 
            speed2_flag = 1; 
            speed3_flag = 1; 
            speed4_flag = 1; 
            speed0_flag = 1; 
        }
        
        if (digitalRead(s2) == LOW && digitalRead(s3) == HIGH && speed2_flag == 1) { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary -> Speed 2"); 
            speed_2(); 
            speed1_flag = 1; 
            speed2_flag = 0; 
            speed3_flag = 1; 
            speed4_flag = 1; 
            speed0_flag = 1; 
        }
        
        if (digitalRead(s2) == LOW && digitalRead(s3) == LOW && speed3_flag == 1) { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary -> Speed 3"); 
            speed_3(); 
            speed1_flag = 1; 
            speed2_flag = 1; 
            speed3_flag = 0; 
            speed4_flag = 1; 
            speed0_flag = 1; 
        }
        
        if (digitalRead(s4) == LOW && speed4_flag == 1) { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary -> Speed 4"); 
            speed_4(); 
            speed1_flag = 1; 
            speed2_flag = 1; 
            speed3_flag = 1; 
            speed4_flag = 0; 
            speed0_flag = 1; 
        }
        
        if (digitalRead(s1) == HIGH && digitalRead(s2) == HIGH && digitalRead(s3) == HIGH && digitalRead(s4) == HIGH && speed0_flag == 1) { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary -> Speed 0"); 
            speed_0(); 
            speed1_flag = 1; 
            speed2_flag = 1; 
            speed3_flag = 1; 
            speed4_flag = 1; 
            speed0_flag = 0; 
        }

        if (digitalRead(gpio_reset) == LOW) {
            if (reset_press_start == 0) {
                reset_press_start = now_ms;
            }
            else if (now_ms - reset_press_start >= 5000) { 
                if (!reset_triggered) { 
                    Serial.println("🚨 [SYSTEM] FACTORY RESET VIA BOOT BUTTON!");
                    reset_triggered = true; 
                    preferences.begin("wifi", false); 
                    preferences.clear(); 
                    preferences.end(); 
                    ESP.restart(); 
                } 
            }
        } else { 
            reset_press_start = 0; 
            reset_triggered = false; 
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
        Serial.println("🔵 [BLE] Phone Connected!"); 
    }
    
    void onDisconnect(BLEServer* pServer) { 
        Serial.println("🔵 [BLE] Phone Disconnected!"); 
        pServer->getAdvertising()->start(); 
    }
};

class WifiWriteCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String value = pChar->getValue().c_str();
        
        if (value.length() > 0) {
            StaticJsonDocument<256> doc; 
            DeserializationError error = deserializeJson(doc, value);
            
            if (!error) {
                const char* rSsid = doc["ssid"]; 
                const char* rPass = doc["pass"];
                
                if (rSsid) {
                    Serial.printf("🔑 [BLE] Received WiFi: %s\n", rSsid);
                    preferences.begin("wifi", false);
                    preferences.putString("ssid", rSsid); 
                    
                    if (rPass) {
                        preferences.putString("pass", rPass);
                    } else {
                        preferences.putString("pass", "");
                    }
                    
                    preferences.end();
                    
                    shouldReboot = true;
                }
            }
        }
    }
};

class DeviceIdWriteCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) { 
        if (pChar->getValue().length() > 0) { 
            shouldReboot = true; 
        } 
    }
};

void startSetupPortal() {
    inSetupMode = true; 
    char portalSSID[50]; 
    snprintf(portalSSID, sizeof(portalSSID), "SmartNest-Setup-%s", NODE_ID + 8);
    
    Serial.printf("⚙️ [SYSTEM] BLE Setup Active! Connect to Bluetooth: %s\n", portalSSID);

    BLEDevice::init(portalSSID);
    BLEServer *pServer = BLEDevice::createServer(); 
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
  
    BLECharacteristic *pWifiChar = pService->createCharacteristic(WIFI_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    pWifiChar->setCallbacks(new WifiWriteCallback());
  
    BLECharacteristic *pMacChar = pService->createCharacteristic(MAC_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
    pMacChar->setValue(NODE_ID);
  
    BLECharacteristic *pDevIdChar = pService->createCharacteristic(DEVICE_ID_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    pDevIdChar->setCallbacks(new DeviceIdWriteCallback());
  
    pService->start();
  
    BLEAdvertising *pAdv = BLEDevice::getAdvertising(); 
    pAdv->addServiceUUID(SERVICE_UUID); 
    pAdv->setScanResponse(true); 
    pAdv->setMinPreferred(0x06);  
    pAdv->setMinPreferred(0x12);
    
    BLEDevice::startAdvertising();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n===========================================");
    Serial.println("🚀 4Layers / Go Smart Firmware V12.4 BOOTING");
    Serial.println("===========================================\n");

    pinMode(relay1, OUTPUT); 
    pinMode(relay2, OUTPUT); 
    pinMode(relay3, OUTPUT); 
    pinMode(relay4, OUTPUT);
    
    pinMode(Speed1, OUTPUT); 
    pinMode(Speed2, OUTPUT); 
    pinMode(Speed4, OUTPUT); 
    pinMode(wifiLed, OUTPUT);
    
    pinMode(switch1, INPUT_PULLUP); 
    pinMode(switch2, INPUT); 
    pinMode(switch3, INPUT); 
    pinMode(switch4, INPUT);
    
    pinMode(fan_switch, INPUT_PULLUP); 
    pinMode(s1, INPUT_PULLUP); 
    pinMode(s2, INPUT_PULLUP); 
    pinMode(s3, INPUT_PULLUP); 
    pinMode(s4, INPUT_PULLUP);
    
    pinMode(gpio_reset, INPUT_PULLUP); 
    pinMode(RF_PIN, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(RF_PIN), rf_isr, CHANGE);

    switch_state_ch1 = load_state_from_nvs("R1", 0); 
    switch_state_ch2 = load_state_from_nvs("R2", 0);
    switch_state_ch3 = load_state_from_nvs("R3", 0); 
    switch_state_ch4 = load_state_from_nvs("R4", 0);
    
    curr_speed = load_state_from_nvs("F_S", 0); 
    fan_power = load_state_from_nvs("F_P", 0); 
    fan_speed_memory = load_state_from_nvs("L_S", 1);

    rf_code_l1 = load_code("rf1", 0); 
    rf_code_l2 = load_code("rf2", 0); 
    rf_code_l3 = load_code("rf3", 0); 
    rf_code_l4 = load_code("rf4", 0);
    
    rf_code_up = load_code("rfu", 0); 
    rf_code_dw = load_code("rfd", 0); 
    rf_code_fan_toggle = load_code("rft", 0); 
    rf_code_master = load_code("rfm", 0);

    if (switch_state_ch1) {
        digitalWrite(relay1, HIGH);
    } else {
        digitalWrite(relay1, LOW);
    }
    
    if (switch_state_ch2) {
        digitalWrite(relay2, HIGH);
    } else {
        digitalWrite(relay2, LOW);
    }
    
    if (switch_state_ch3) {
        digitalWrite(relay3, HIGH);
    } else {
        digitalWrite(relay3, LOW);
    }
    
    if (switch_state_ch4) {
        digitalWrite(relay4, HIGH);
    } else {
        digitalWrite(relay4, LOW);
    }
    
    if (curr_speed == 1) {
        speed_1();
    } else if (curr_speed == 2) {
        speed_2();
    } else if (curr_speed == 3) {
        speed_3();
    } else if (curr_speed == 4) {
        speed_4();
    } else {
        speed_0();
    }

    uint64_t mac = ESP.getEfuseMac(); 
    snprintf(NODE_ID, sizeof(NODE_ID), "4L-NODE-%06llX", mac & 0xFFFFFF);
    snprintf(command_topic, sizeof(command_topic), "home/device/%s/control", NODE_ID);
    snprintf(status_topic, sizeof(status_topic), "home/device/%s/status", NODE_ID);

    Serial.printf("📡 Node ID Generated: %s\n", NODE_ID);
    
    xTaskCreatePinnedToCore(system_task, "system_task", 8192, NULL, 5, NULL, 1);

    preferences.begin("wifi", true);
    saved_ssid = preferences.getString("ssid", ""); 
    saved_password = preferences.getString("pass", "");
    preferences.end();

    if (saved_ssid.length() > 0) {
        Serial.printf("🌐 Saved Wi-Fi found: %s\n", saved_ssid.c_str());
        
        WiFi.mode(WIFI_STA); 
        WiFi.disconnect(true); 
        delay(100);
        
        WiFi.setSleep(false);  
        WiFi.setAutoReconnect(true); 
        
        WiFi.begin(saved_ssid.c_str(), saved_password.c_str());
        
        int retries = 0;
        
        while (WiFi.status() != WL_CONNECTED && retries < 20) { 
            delay(500); 
            Serial.print("."); 
            retries++; 
        }
        
        espClient.setInsecure();
        client.setServer(mqtt_server, mqtt_port); 
        client.setCallback(mqtt_callback);

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\n✅ Wi-Fi Connected Instantly!");
        } else {
            Serial.println("\n⏳ Wi-Fi router offline or booting. ESP will keep trying in background...");
        }
    } else {
        Serial.println("🌐 No Wi-Fi Credentials found. Starting Setup Portal.");
        startSetupPortal();
    }
}

void loop() {
    unsigned long now_ms = millis();
    static unsigned long last_mqtt_reconnect = 0;
    
    if (inSetupMode) {
        if (shouldReboot) { 
            Serial.println("🔄 Rebooting in 1 sec..."); 
            delay(1000); 
            ESP.restart(); 
        }
        
        digitalWrite(wifiLed, (now_ms / 500) % 2); 
        delay(10); 
        return; 
    }

    if (WiFi.status() == WL_CONNECTED) {
        if (!client.connected()) {
            if (now_ms - last_mqtt_reconnect > 3000 || last_mqtt_reconnect == 0) {
                last_mqtt_reconnect = now_ms;
                
                Serial.println("☁️ Connecting to EMQX Cloud...");
                String clientId = "4L-Client-" + String(NODE_ID);
                
                if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
                    Serial.println("✅ Cloud Connected!");
                    client.subscribe(command_topic);
                    
                    sendChannelState(1, switch_state_ch1); 
                    sendChannelState(2, switch_state_ch2);
                    sendChannelState(3, switch_state_ch3); 
                    sendChannelState(4, switch_state_ch4);
                    sendChannelState(5, fan_power, curr_speed);
                    
                    bool master_state = switch_state_ch1 || switch_state_ch2 || switch_state_ch3 || switch_state_ch4 || fan_power;
                    sendChannelState(6, master_state);
                } else { 
                    Serial.print("❌ Cloud failed, rc="); 
                    Serial.println(client.state()); 
                }
            }
        } else {
            client.loop();
        }
    }

    if (nvs_dirty) {
        if (now_ms - nvs_dirty_time > 10000) {
            bool s1;
            bool s2;
            bool s3;
            bool s4;
            bool f_pow;
            int f_spd;
            int f_mem;
            
            portENTER_CRITICAL(&state_mux); 
            s1 = switch_state_ch1;
            s2 = switch_state_ch2;
            s3 = switch_state_ch3;
            s4 = switch_state_ch4;
            f_pow = fan_power;
            f_spd = curr_speed;
            f_mem = fan_speed_memory;
            nvs_dirty = false;
            portEXIT_CRITICAL(&state_mux); 
            
            save_state_to_nvs("R1", (uint8_t)s1);
            save_state_to_nvs("R2", (uint8_t)s2);
            save_state_to_nvs("R3", (uint8_t)s3);
            save_state_to_nvs("R4", (uint8_t)s4);
            save_state_to_nvs("F_S", (uint8_t)f_spd);
            save_state_to_nvs("F_P", (uint8_t)f_pow);
            save_state_to_nvs("L_S", (uint8_t)f_mem);
            
            Serial.println("💾 [NVS] States Saved to Flash Memory");
        }
    }

    if (pairing_confirm_until_ms > now_ms) {
        digitalWrite(wifiLed, HIGH);
    }
    else if (pairing_target > 0) {
        digitalWrite(wifiLed, (now_ms / 120) % 2);
    }
    else if (client.connected()) {
        digitalWrite(wifiLed, HIGH);
    }
    else {
        digitalWrite(wifiLed, (now_ms / 1000) % 2);
    }

    delay(50);
}

extern "C" void app_main() {
    initArduino();
    setup();
    
    while (true) {
        loop();
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}