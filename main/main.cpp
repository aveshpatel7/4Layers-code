#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <esp_task_wdt.h> 
#include <lwip/dns.h>
#include <lwip/ip_addr.h>

#define DEBOUNCE_MS 50
#define PAIRING_TIMEOUT_MS 25000U
#define PAIRING_CONFIRM_MS 1000U 
#define PAIRING_LED_BLINK_MS 120U
#define WIFI_STUCK_MS 180000U
#define SWITCH1_PAIR_TOGGLES 5
#define SWITCH2_RESET_TOGGLES 20

const char* mqtt_server = "i26a1c71.ala.asia-southeast1.emqxsl.com";
const int mqtt_port = 8883;
const char* mqtt_user = "smartnest_client";
const char* mqtt_pass = "D2m9ga8JynJDEM6";

// EMQX Cloud serves a DigiCert chain, NOT Let's Encrypt:
//   leaf   CN=*.ala.asia-southeast1.emqxsl.com
//   inter  CN=Encryption Everywhere DV TLS CA - G2
//   root   CN=DigiCert Global Root G2   <-- embedded below
// Verified with `openssl s_client -connect ...:8883 -showcerts`.
// Embedding the correct root lets the handshake verify the chain WITHOUT
// setInsecure() and WITHOUT the runtime CA bundle API (not available on
// arduino-esp32 3.3.10's WiFiClientSecure — setCACertBundle() takes
// (const uint8_t*, size_t), not a boolean).
const char* EMQX_ROOT_CA = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
)EOF";

char NODE_ID[32];
char command_topic[100]; 
char status_topic[100];  

WiFiClientSecure espClient;
PubSubClient client(espClient);
Preferences preferences;
WebServer server(80);

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

// ==========================================
// FREERTOS COMMAND QUEUE ARCHITECTURE (FIFO Buffer)
// ==========================================
typedef enum {
    CMD_CHANNEL_SET,
    CMD_FAN_SPEED_SET,
    CMD_BULK_ALL_ON,
    CMD_BULK_ALL_OFF
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    uint8_t channel;     // 1 - 7
    bool state;          // true = ON, false = OFF
    int8_t speed;        // 0 - 4 (-1 if unchanged)
    char source[32];     // Command origin icon & label
    bool from_cloud;     // true when the command arrived over MQTT
} switch_command_t;

static QueueHandle_t command_queue = NULL;

// PubSubClient and WiFiClientSecure are NOT thread-safe, but publishes originate
// from four tasks (cmd_worker, system_task, webserver_task, mqtt_task) across both
// cores. Two tasks writing the same TLS session interleaves their records, which
// the peer rejects as -29184 (invalid SSL record) / -29056 (bad record MAC) and
// tears the connection down. Every client.* call must hold this mutex.
//
// Recursive: mqtt_callback() runs from inside client.loop() while the lock is
// already held by this task, and it may call logRemote()/publishOTAStatus()
// which acquire it again. A plain mutex would self-deadlock there.
static SemaphoreHandle_t mqtt_lock = NULL;

#define MQTT_LOCK_TAKE(ticks) (mqtt_lock == NULL || xSemaphoreTakeRecursive(mqtt_lock, (ticks)) == pdTRUE)
#define MQTT_LOCK_GIVE()      do { if (mqtt_lock != NULL) xSemaphoreGiveRecursive(mqtt_lock); } while (0)

// Cached MQTT link state, published by mqtt_task. loop() reads it for the status
// LED instead of calling client.connected() unlocked from another task.
static volatile bool mqtt_link_up = false;

// Set while a cloud-originated command executes, so sendChannelState() skips the
// status publish for it. Commands run serially in command_worker_task, so a single
// flag is sufficient; the only other caller is the queue-full fallback path in
// process_channel_command(), which is rare and at worst re-enables one echo.
static volatile bool suppress_status_echo = false;

#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb"
#define WIFI_CHAR_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"
#define MAC_CHAR_UUID "0000ffe2-0000-1000-8000-00805f9b34fb"
#define DEVICE_ID_CHAR_UUID "0000ffe3-0000-1000-8000-00805f9b34fb"

bool inSetupMode = false; 
bool shouldReboot = false; 
// String, not char[64]: a WPA2 passphrase is legal up to 63 characters, and a
// fixed buffer silently truncates anything longer. Read once in setup().
String saved_ssid = "";
String saved_password = "";

// ==========================================
// SMART FLASH SAVER (NVS CACHE)
// ==========================================

typedef struct 
{
    const char *key;
    uint8_t value;
    bool dirty;
} nvs_cache_t;

static nvs_cache_t nvs_cache[] = 
{
    {"R1", 0, false}, 
    {"R2", 0, false}, 
    {"R3", 0, false}, 
    {"R4", 0, false},
    {"F_S", 0, false}, 
    {"F_P", 0, false}, 
    {"L_S", 1, false}
};

#define NVS_CACHE_LEN (sizeof(nvs_cache)/sizeof(nvs_cache[0]))

uint8_t load_state_from_nvs(const char *key, uint8_t def) 
{
    preferences.begin("storage", true);
    uint8_t val = preferences.getUInt(key, def);
    preferences.end();
    
    portENTER_CRITICAL(&state_mux);
    for (int i = 0; i < NVS_CACHE_LEN; i++) 
    {
        if (strcmp(nvs_cache[i].key, key) == 0) 
        {
            nvs_cache[i].value = val;
            nvs_cache[i].dirty = false;
            break;
        }
    }
    portEXIT_CRITICAL(&state_mux);
    
    return val;
}

void save_state_to_nvs(const char *key, uint8_t value) 
{
    portENTER_CRITICAL(&state_mux);
    for (int i = 0; i < NVS_CACHE_LEN; i++) 
    {
        if (strcmp(nvs_cache[i].key, key) == 0) 
        {
            if (nvs_cache[i].value != value) 
            {
                nvs_cache[i].value = value;
                nvs_cache[i].dirty = true;
            }
            portEXIT_CRITICAL(&state_mux);
            return;
        }
    }
    portEXIT_CRITICAL(&state_mux);
}

void nvs_commit_task(void *pv) 
{
    esp_task_wdt_add(NULL); 
    
    while (1) 
    {
        for (int w = 0; w < 10; w++) 
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset(); 
        }
        
        bool needs_commit = false;
        
        portENTER_CRITICAL(&state_mux);
        for (int i = 0; i < NVS_CACHE_LEN; i++) 
        {
            if (nvs_cache[i].dirty) 
            {
                needs_commit = true; 
                break; 
            }
        }
        portEXIT_CRITICAL(&state_mux);
        
        if (needs_commit) 
        {
            // Snapshot dirty entries under the lock; NVS/flash writes take mutexes
            // internally and must never run inside a critical section.
            uint8_t snapshot_val[NVS_CACHE_LEN];
            bool snapshot_dirty[NVS_CACHE_LEN] = {false};
            
            portENTER_CRITICAL(&state_mux);
            for (int i = 0; i < NVS_CACHE_LEN; i++) 
            {
                if (nvs_cache[i].dirty) 
                {
                    snapshot_dirty[i] = true;
                    snapshot_val[i] = nvs_cache[i].value;
                }
            }
            portEXIT_CRITICAL(&state_mux);
            
            preferences.begin("storage", false);
            for (int i = 0; i < NVS_CACHE_LEN; i++) 
            {
                if (snapshot_dirty[i]) 
                {
                    preferences.putUInt(nvs_cache[i].key, snapshot_val[i]);
                }
            }
            preferences.end();
            
            portENTER_CRITICAL(&state_mux);
            for (int i = 0; i < NVS_CACHE_LEN; i++) 
            {
                // Clear only if unchanged since the snapshot; if a newer value
                // arrived meanwhile keep it dirty for the next sync cycle.
                if (snapshot_dirty[i] && nvs_cache[i].value == snapshot_val[i]) 
                {
                    nvs_cache[i].dirty = false;
                }
            }
            portEXIT_CRITICAL(&state_mux);
            
            Serial.println("💾 [SYSTEM] Smart Saver: NVS Cache background sync successful!");
        }
    }
}

// ==========================================

uint32_t load_code(const char *key, uint32_t def) 
{
    preferences.begin("codes", true);
    uint32_t val = preferences.getUInt(key, def);
    preferences.end();
    return val;
}

void save_code(const char *key, uint32_t val) 
{
    preferences.begin("codes", false);
    preferences.putUInt(key, val);
    preferences.end();
    Serial.printf("💾 [SYSTEM] RF Code Saved for %s\n", key);
}

void sendChannelState(int channel, bool status, int val = -1) 
{
    // Cloud-originated commands are not echoed back; see suppress_status_echo.
    if (suppress_status_echo) 
    {
        return;
    }
    
    StaticJsonDocument<200> doc;
    doc["channel"] = channel;
    
    if (status) 
    {
        doc["status"] = "ON";
    }
    else 
    {
        doc["status"] = "OFF";
    }
    
    if (channel == 5) 
    {
        int safe_speed = (val != -1 && val > 0) ? val : (curr_speed > 0 ? curr_speed : fan_speed_memory);
        if (safe_speed < 1 || safe_speed > 4) safe_speed = 4;
        doc["speed"] = safe_speed;
        doc["value"] = safe_speed;
    }

    char buffer[256];
    serializeJson(doc, buffer);
    
    if (MQTT_LOCK_TAKE(pdMS_TO_TICKS(500)))
    {
        if (client.connected()) 
        {
            if(!client.publish(status_topic, buffer, true)) 
            {
                Serial.printf("❌ [ERROR] MQTT Publish Failed for Channel %d\n", channel);
            }
        }
        MQTT_LOCK_GIVE();
    }
    else
    {
        Serial.printf("⚠️ [MQTT] Busy, dropped state publish for Channel %d\n", channel);
    }
}

void pref_save_fan() 
{
    int safe_speed; 
    bool safe_power;
    
    portENTER_CRITICAL(&state_mux);
    safe_speed = curr_speed; 
    safe_power = fan_power;
    portEXIT_CRITICAL(&state_mux);
    
    save_state_to_nvs("F_S", (uint8_t)safe_speed);
    save_state_to_nvs("F_P", (uint8_t)safe_power);
    
    if (safe_speed > 0) 
    {
        portENTER_CRITICAL(&state_mux); 
        fan_speed_memory = safe_speed; 
        portEXIT_CRITICAL(&state_mux);
        
        save_state_to_nvs("L_S", (uint8_t)safe_speed);
    }
    
    sendChannelState(5, safe_power, safe_speed);
}

void set_fan_relays(int t_s1, int t_s2, int t_s4) 
{
    digitalWrite(Speed1, LOW); 
    digitalWrite(Speed2, LOW); 
    digitalWrite(Speed4, LOW);
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    if (t_s1 == 1) 
    {
        digitalWrite(Speed1, HIGH);
    }
    
    if (t_s2 == 1) 
    {
        digitalWrite(Speed2, HIGH);
    }
    
    if (t_s4 == 1) 
    {
        digitalWrite(Speed4, HIGH);
    }
}

void speed_0() 
{ 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 0; 
    fan_power = false; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(0, 0, 0); 
    pref_save_fan(); 
}

void speed_1() 
{ 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 1; 
    fan_power = true; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(1, 0, 0); 
    pref_save_fan(); 
}

void speed_2() 
{ 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 2; 
    fan_power = true; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(0, 1, 0); 
    pref_save_fan(); 
}

void speed_3() 
{ 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 3; 
    fan_power = true; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(1, 1, 0); 
    pref_save_fan(); 
}

void speed_4() 
{ 
    portENTER_CRITICAL(&state_mux); 
    curr_speed = 4; 
    fan_power = true; 
    portEXIT_CRITICAL(&state_mux); 
    
    set_fan_relays(0, 0, 1); 
    pref_save_fan(); 
}

void restore_fan_speed() 
{
    int mem_speed;
    
    portENTER_CRITICAL(&state_mux);
    if (fan_speed_memory < 1 || fan_speed_memory > 4) 
    {
        fan_speed_memory = 1;
    }
    mem_speed = fan_speed_memory;
    portEXIT_CRITICAL(&state_mux);
    
    if (mem_speed == 1) 
    {
        speed_1();
    }
    else if (mem_speed == 2) 
    {
        speed_2();
    }
    else if (mem_speed == 3) 
    {
        speed_3();
    }
    else if (mem_speed == 4) 
    {
        speed_4();
    }
}

// ==========================================
// DIRECT COMMAND EXECUTOR (Called from Command Worker Task)
// ==========================================
void execute_command_direct(const switch_command_t *cmd)
{
    if (cmd == NULL) return;

    // Cloud-originated commands must not echo their resulting state back on the
    // status topic. Set for the whole call so fan and bulk paths — which publish
    // from nested helpers like pref_save_fan() — are covered too.
    suppress_status_echo = cmd->from_cloud;

    if (cmd->type == CMD_CHANNEL_SET)
    {
        int channel = cmd->channel;
        bool turnOn = cmd->state;

        if (channel >= 1 && channel <= 4)
        {
            // Idempotency guard: a command that asks for the state the relay is
            // already in does nothing but wear the relay and generate MQTT
            // traffic. Drop it before touching hardware.
            bool already;
            portENTER_CRITICAL(&state_mux);
            if (channel == 1)      already = (switch_state_ch1 == turnOn);
            else if (channel == 2) already = (switch_state_ch2 == turnOn);
            else if (channel == 3) already = (switch_state_ch3 == turnOn);
            else                   already = (switch_state_ch4 == turnOn);
            portEXIT_CRITICAL(&state_mux);
            
            if (already)
            {
                Serial.printf("%s Command: Channel %d already %s, ignored\n",
                              cmd->source, channel, turnOn ? "ON" : "OFF");
                suppress_status_echo = false;
                return;
            }
            
            if (turnOn)
            {
                Serial.printf("%s Command: Channel %d -> ON\n", cmd->source, channel);
            }
            else
            {
                Serial.printf("%s Command: Channel %d -> OFF\n", cmd->source, channel);
            }

            portENTER_CRITICAL(&state_mux); 
            if (channel == 1) switch_state_ch1 = turnOn;
            else if (channel == 2) switch_state_ch2 = turnOn;
            else if (channel == 3) switch_state_ch3 = turnOn;
            else if (channel == 4) switch_state_ch4 = turnOn;
            portEXIT_CRITICAL(&state_mux);
            
            if (channel == 1)
            {
                digitalWrite(relay1, turnOn ? HIGH : LOW);
                sendChannelState(1, turnOn); 
                save_state_to_nvs("R1", turnOn ? 1 : 0);
            }
            else if (channel == 2)
            {
                digitalWrite(relay2, turnOn ? HIGH : LOW);
                sendChannelState(2, turnOn); 
                save_state_to_nvs("R2", turnOn ? 1 : 0);
            }
            else if (channel == 3)
            {
                digitalWrite(relay3, turnOn ? HIGH : LOW);
                sendChannelState(3, turnOn); 
                save_state_to_nvs("R3", turnOn ? 1 : 0);
            }
            else if (channel == 4)
            {
                digitalWrite(relay4, turnOn ? HIGH : LOW);
                sendChannelState(4, turnOn); 
                save_state_to_nvs("R4", turnOn ? 1 : 0);
            }
        }
        else if (channel == 5)
        {
            if (turnOn)
            {
                Serial.printf("%s Command: Fan Power -> ON\n", cmd->source);
                restore_fan_speed();
            }
            else
            {
                Serial.printf("%s Command: Fan Power -> OFF\n", cmd->source);
                speed_0();
            }
        }    }
    else if (cmd->type == CMD_FAN_SPEED_SET)
    {
        int speedVal = cmd->speed;
        Serial.printf("%s Command: Fan Speed -> %d\n", cmd->source, speedVal);
        if (speedVal == 1) speed_1();
        else if (speedVal == 2) speed_2();
        else if (speedVal == 3) speed_3();
        else if (speedVal == 4) speed_4();
        else speed_0();
    }
    else if (cmd->type == CMD_BULK_ALL_ON)
    {
        Serial.printf("%s Command: Master -> ON (Staggered Bulk Action via Queue)\n", cmd->source);
        
        portENTER_CRITICAL(&state_mux); 
        switch_state_ch1 = 1; 
        portEXIT_CRITICAL(&state_mux);
        Serial.println("⚙️ [SYSTEM] Bulk ON -> Relay 1 ON");
        digitalWrite(relay1, HIGH); 
        sendChannelState(1, true); 
        save_state_to_nvs("R1", 1); 
        vTaskDelay(pdMS_TO_TICKS(60)); 
        esp_task_wdt_reset();

        portENTER_CRITICAL(&state_mux); 
        switch_state_ch2 = 1; 
        portEXIT_CRITICAL(&state_mux);
        Serial.println("⚙️ [SYSTEM] Bulk ON -> Relay 2 ON");
        digitalWrite(relay2, HIGH); 
        sendChannelState(2, true); 
        save_state_to_nvs("R2", 1); 
        vTaskDelay(pdMS_TO_TICKS(60)); 
        esp_task_wdt_reset();

        portENTER_CRITICAL(&state_mux); 
        switch_state_ch3 = 1; 
        portEXIT_CRITICAL(&state_mux);
        Serial.println("⚙️ [SYSTEM] Bulk ON -> Relay 3 ON");
        digitalWrite(relay3, HIGH); 
        sendChannelState(3, true); 
        save_state_to_nvs("R3", 1); 
        vTaskDelay(pdMS_TO_TICKS(60)); 
        esp_task_wdt_reset();

        portENTER_CRITICAL(&state_mux); 
        switch_state_ch4 = 1; 
        portEXIT_CRITICAL(&state_mux);
        Serial.println("⚙️ [SYSTEM] Bulk ON -> Relay 4 ON");
        digitalWrite(relay4, HIGH); 
        sendChannelState(4, true); 
        save_state_to_nvs("R4", 1); 
        vTaskDelay(pdMS_TO_TICKS(60)); 
        esp_task_wdt_reset();

        bool f_pow; 
        portENTER_CRITICAL(&state_mux); 
        f_pow = fan_power; 
        portEXIT_CRITICAL(&state_mux);
        if (!f_pow) 
        {
            Serial.println("⚙️ [SYSTEM] Bulk ON -> Fan ON");
            restore_fan_speed();
        }
        
        sendChannelState(6, true);
        Serial.println("⚙️ [SYSTEM] Master Bulk ON Task Completed!");
    }
    else if (cmd->type == CMD_BULK_ALL_OFF)
    {
        Serial.printf("%s Command: Master -> OFF (Staggered Bulk Action via Queue)\n", cmd->source);

        portENTER_CRITICAL(&state_mux); 
        switch_state_ch1 = 0; 
        portEXIT_CRITICAL(&state_mux);
        Serial.println("⚙️ [SYSTEM] Bulk OFF -> Relay 1 OFF");
        digitalWrite(relay1, LOW); 
        sendChannelState(1, false); 
        save_state_to_nvs("R1", 0); 
        vTaskDelay(pdMS_TO_TICKS(60)); 
        esp_task_wdt_reset();

        portENTER_CRITICAL(&state_mux); 
        switch_state_ch2 = 0; 
        portEXIT_CRITICAL(&state_mux);
        Serial.println("⚙️ [SYSTEM] Bulk OFF -> Relay 2 OFF");
        digitalWrite(relay2, LOW); 
        sendChannelState(2, false); 
        save_state_to_nvs("R2", 0); 
        vTaskDelay(pdMS_TO_TICKS(60)); 
        esp_task_wdt_reset();

        portENTER_CRITICAL(&state_mux); 
        switch_state_ch3 = 0; 
        portEXIT_CRITICAL(&state_mux);
        Serial.println("⚙️ [SYSTEM] Bulk OFF -> Relay 3 OFF");
        digitalWrite(relay3, LOW); 
        sendChannelState(3, false); 
        save_state_to_nvs("R3", 0); 
        vTaskDelay(pdMS_TO_TICKS(60)); 
        esp_task_wdt_reset();

        portENTER_CRITICAL(&state_mux); 
        switch_state_ch4 = 0; 
        portEXIT_CRITICAL(&state_mux);
        Serial.println("⚙️ [SYSTEM] Bulk OFF -> Relay 4 OFF");
        digitalWrite(relay4, LOW); 
        sendChannelState(4, false); 
        save_state_to_nvs("R4", 0); 
        vTaskDelay(pdMS_TO_TICKS(60)); 
        esp_task_wdt_reset();

        Serial.println("⚙️ [SYSTEM] Bulk OFF -> Fan OFF");
        speed_0(); 
        sendChannelState(6, false);
        Serial.println("⚙️ [SYSTEM] Master Bulk OFF Task Completed!");
    }
    
    suppress_status_echo = false;
}

// ==========================================
// DEDICATED COMMAND QUEUE WORKER TASK (Core 1)
// ==========================================
void command_worker_task(void *pvParameters) 
{
    esp_task_wdt_add(NULL);
    switch_command_t cmd;
    Serial.println("⚙️ [SYSTEM] Command Queue Worker Task Started on Core 1!");

    while (true) 
    {
        esp_task_wdt_reset();
        
        if (xQueueReceive(command_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) 
        {
            execute_command_direct(&cmd);
        }
    }
}

// ==========================================
// MASTER CHANNEL CONTROLLER (Queue-Buffered)
// ==========================================
// fromCloud marks commands that arrived over MQTT. Those must NOT be echoed back
// on the status topic: the app already knows the state it just asked for, and
// echoing it re-triggers the app's own change listener, producing an endless
// OFF -> ON -> OFF command storm. Local sources (physical switch, RF remote,
// local web UI) still publish, because the app has no other way to learn about them.
void process_channel_command(int channel, bool turnOn, int speedVal, const char* sourceIcon, bool fromCloud = false)
{
    switch_command_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.from_cloud = fromCloud;
    if (sourceIcon) 
    {
        strncpy(cmd.source, sourceIcon, sizeof(cmd.source) - 1);
    }
    else 
    {
        strncpy(cmd.source, "⚙️ [SYSTEM]", sizeof(cmd.source) - 1);
    }

    if (channel >= 1 && channel <= 4)
    {
        cmd.type = CMD_CHANNEL_SET;
        cmd.channel = (uint8_t)channel;
        cmd.state = turnOn;
        cmd.speed = -1;
    }
    else if (channel == 5)
    {
        if (speedVal != -1)
        {
            cmd.type = CMD_FAN_SPEED_SET;
            cmd.channel = 5;
            cmd.state = (speedVal > 0);
            cmd.speed = (int8_t)speedVal;
        }
        else
        {
            cmd.type = CMD_CHANNEL_SET;
            cmd.channel = 5;
            cmd.state = turnOn;
            cmd.speed = -1;
        }
    }
    else if (channel == 6 || channel == 7)
    {
        cmd.type = turnOn ? CMD_BULK_ALL_ON : CMD_BULK_ALL_OFF;
        cmd.channel = (uint8_t)channel;
        cmd.state = turnOn;
        cmd.speed = -1;
    }
    else
    {
        return;
    }

    if (command_queue != NULL)
    {
        if (xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(50)) != pdPASS)
        {
            Serial.println("⚠️ [QUEUE] Command queue full (16 items buffered)! Executing fallback direct.");
            execute_command_direct(&cmd);
        }
    }
    else
    {
        execute_command_direct(&cmd);
    }
}

void All_On() 
{ 
    process_channel_command(6, true, -1, "⚙️ [SYSTEM]");
}

void All_Off() 
{ 
    process_channel_command(6, false, -1, "⚙️ [SYSTEM]");
}
// ==========================================

void IRAM_ATTR rf_isr() 
{
    if (rf_available == true) 
    {
        return;
    }
    
    static uint32_t lt = 0; 
    static uint16_t tm[64]; 
    static uint8_t ct = 0;
    
    uint32_t now = micros(); 
    uint32_t df = now - lt; 
    lt = now;
    
    if (df < 150) 
    { 
        ct = 0; 
        return; 
    }
    
    if (df > 4000 && df < 20000) 
    {
        if (ct >= 48) 
        {
            uint32_t c = 0;
            
            for (int i = 0; i < 48; i += 2) 
            {
                c = (c << 1) | (tm[i] > tm[i + 1]);
            }
            
            if (c > 0 && c != 0xFFFFFFFF) 
            { 
                rf_received_value = c; 
                rf_available = true; 
            }
        }
        
        ct = 0;
    } 
    else if (ct < 60) 
    {
        tm[ct++] = df;
    }
}

void logRemote(const String& msg) 
{
    Serial.println(msg); 
    
    if (MQTT_LOCK_TAKE(pdMS_TO_TICKS(500)))
    {
        if (client.connected()) 
        {
            char logTopic[120];
            snprintf(logTopic, sizeof(logTopic), "smartnest/devices/%s/logs", NODE_ID);
            client.publish(logTopic, msg.c_str());
        }
        MQTT_LOCK_GIVE();
    }
}

void publishOTAStatus(const char* status, int progress) 
{
    StaticJsonDocument<128> doc;
    doc["status"] = status;
    doc["progress"] = progress;
    
    char buffer[128];
    serializeJson(doc, buffer);
    
    char ota_status_topic[120];
    snprintf(ota_status_topic, sizeof(ota_status_topic), "smartnest/devices/%s/ota/status", NODE_ID);
    
    if (MQTT_LOCK_TAKE(pdMS_TO_TICKS(1000)))
    {
        bool up = client.connected();
        if (up)
        {
            client.publish(ota_status_topic, buffer, true);
        }
        MQTT_LOCK_GIVE();
        
        if (!up)
        {
            Serial.println("❌ [ERROR] Cannot publish OTA status. MQTT not connected.");
            return;
        }
    }
    
    logRemote("[OTA MQTT] Status: " + String(buffer));
}

void performOTAUpdate(const String& firmwareUrl) 
{
    int jitterMs = random(1000, 5000);
    logRemote("================================================");
    logRemote("[OTA] Starting Update. Jitter Delay: " + String(jitterMs) + "ms");
    delay(jitterMs);

    logRemote("[OTA] Target URL: " + firmwareUrl);
    WiFiClientSecure otaClient;
    otaClient.setInsecure();
    otaClient.setTimeout(15000);

    httpUpdate.onProgress([](int cur, int total) 
    {
        if (total <= 0) 
        {
            return;
        }
        
        int percent = (cur * 100) / total;
        static int lastPercent = -1;
        
        if (percent != lastPercent && (percent % 10 == 0 || percent == 100)) 
        {
            lastPercent = percent;
            publishOTAStatus("downloading", percent);
            logRemote("[OTA PROGRESS] Downloaded " + String(percent) + "%");
        }
        
        esp_task_wdt_reset(); 
    });

    publishOTAStatus("downloading", 0);
    Serial.println("⚙️ [SYSTEM] OTA Download starting...");
    
    t_httpUpdate_return ret = httpUpdate.update(otaClient, firmwareUrl);

    if (ret == HTTP_UPDATE_OK) 
    {
        logRemote("[OTA SUCCESS] Flashing complete! Rebooting...");
        publishOTAStatus("success", 100);
        delay(1000);
        ESP.restart();
    } 
    else 
    {
        logRemote("[OTA ERROR] Failed! Code: " + String(httpUpdate.getLastError()));
        publishOTAStatus("failed", 0);
    }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
    StaticJsonDocument<384> doc;
    
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) 
    {
        Serial.printf("❌ [ERROR] MQTT JSON Parse Failed: %s (Code: %s). Raw Payload (%u bytes): \"", 
                      error.c_str(), error.code() == DeserializationError::InvalidInput ? "InvalidInput" : "Error", length);
        Serial.write(payload, length);
        Serial.println("\"");
        return;
    }

    if (doc.containsKey("action") && doc["action"] == "OTA_UPDATE") 
    {
        const char* url_ptr = doc["firmware_url"];
        
        if (url_ptr && strlen(url_ptr) > 0) 
        {
            // Deep copy before leaving the callback: url_ptr points into
            // PubSubClient's receive buffer, and performOTAUpdate() sleeps for a
            // 1-5s jitter delay during which client.loop() on another task can
            // overwrite that buffer.
            String fw_url = String(url_ptr);
            
            Serial.println("📱 [APP/CLOUD] OTA Update Command Received!");
            logRemote("[MQTT] OTA Update Command Received!");
            performOTAUpdate(fw_url);
        }
        else 
        {
            Serial.println("❌ [ERROR] OTA Command missing firmware URL!");
        }
        return;
    }

    if (doc.containsKey("action") && doc["action"] == "factory_reset") 
    {
        Serial.println("🚨 [APP/CLOUD] Factory Reset Command Received!");
        preferences.begin("wifi", false); 
        preferences.clear(); 
        preferences.end(); 
        Serial.println("⚙️ [SYSTEM] Wi-Fi credentials erased. Rebooting...");
        delay(1000); 
        ESP.restart();
    }
    
    if (!doc.containsKey("channel") || (!doc.containsKey("status") && !doc.containsKey("state"))) 
    {
        Serial.println("❌ [ERROR] Invalid MQTT Command: Missing 'channel' or 'status'/'state'");
        return;
    }

    int channel = doc["channel"]; 
    bool state = false;
    
    if (doc.containsKey("status"))
    {
        if (doc["status"].is<const char*>())
        {
            const char* s = doc["status"].as<const char*>();
            state = (strcasecmp(s, "ON") == 0 || strcasecmp(s, "TRUE") == 0 || strcmp(s, "1") == 0);
        }
        else if (doc["status"].is<bool>())
        {
            state = doc["status"].as<bool>();
        }
        else if (doc["status"].is<int>())
        {
            state = (doc["status"].as<int>() != 0);
        }
    }
    else if (doc.containsKey("state"))
    {
        if (doc["state"].is<const char*>())
        {
            const char* s = doc["state"].as<const char*>();
            state = (strcasecmp(s, "ON") == 0 || strcasecmp(s, "TRUE") == 0 || strcmp(s, "1") == 0);
        }
        else if (doc["state"].is<bool>())
        {
            state = doc["state"].as<bool>();
        }
        else if (doc["state"].is<int>())
        {
            state = (doc["state"].as<int>() != 0);
        }
    }
    
    int spd = doc.containsKey("speed") ? (int)doc["speed"] : (doc.containsKey("value") && channel == 5 ? (int)doc["value"] : -1);
    
    process_channel_command(channel, state, spd, "📱 [APP/CLOUD]", true);
}

void setupLocalWebServer() 
{
    server.enableCORS(true);

    server.on("/state", HTTP_GET, []() 
    {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        StaticJsonDocument<512> doc;
        
        doc["node_id"] = NODE_ID;
        doc["local_ip"] = WiFi.localIP().toString();
        
        portENTER_CRITICAL(&state_mux);
        
        // Structured relays array for fast app parsing
        JsonArray relays = doc.createNestedArray("relays");
        relays.add(switch_state_ch1 ? 1 : 0);
        relays.add(switch_state_ch2 ? 1 : 0);
        relays.add(switch_state_ch3 ? 1 : 0);
        relays.add(switch_state_ch4 ? 1 : 0);
        
        // Structured fan object
        JsonObject fanObj = doc.createNestedObject("fan");
        fanObj["enabled"] = fan_power;
        fanObj["speed"] = curr_speed;
        
        // Key-value channels for backwards compatibility
        doc["channel_1"] = switch_state_ch1 ? "ON" : "OFF";
        doc["channel_2"] = switch_state_ch2 ? "ON" : "OFF";
        doc["channel_3"] = switch_state_ch3 ? "ON" : "OFF";
        doc["channel_4"] = switch_state_ch4 ? "ON" : "OFF";
        doc["channel_5"] = fan_power ? "ON" : "OFF";
        doc["speed"] = curr_speed;

        bool all_on = switch_state_ch1 && switch_state_ch2 && switch_state_ch3 && switch_state_ch4 && fan_power;
        bool all_off = !switch_state_ch1 && !switch_state_ch2 && !switch_state_ch3 && !switch_state_ch4 && !fan_power;
        doc["all_state"] = all_on ? "ALL_ON" : (all_off ? "ALL_OFF" : "MIXED");
        
        portEXIT_CRITICAL(&state_mux);
        
        String res;
        serializeJson(doc, res);
        server.send(200, "application/json", res);
        // Note: excessive Serial.println removed to ensure < 20ms response time
    });

    server.on("/control", HTTP_GET, []() 
    {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        
        if (!server.hasArg("channel") || !(server.hasArg("state") || server.hasArg("status"))) 
        {
            server.send(400, "application/json", "{\"error\":\"Missing args\"}");
            return;
        }
        
        int channel = server.arg("channel").toInt();
        String stateStr = server.hasArg("state") ? server.arg("state") : server.arg("status");
        stateStr.toUpperCase();
        
        bool turnOn = (stateStr == "ON" || stateStr == "TRUE" || stateStr == "1");
        int spd = server.hasArg("speed") ? server.arg("speed").toInt() : -1;

        process_channel_command(channel, turnOn, spd, "🌐 [APP/LOCAL]");
        
        StaticJsonDocument<128> respDoc;
        respDoc["success"] = true;
        respDoc["channel"] = channel;
        respDoc["state"] = turnOn ? "ON" : "OFF";
        if (spd != -1) respDoc["speed"] = spd;
        String resp;
        serializeJson(respDoc, resp);
        server.send(200, "application/json", resp);
    });

    server.onNotFound([]() {
        if (server.method() == HTTP_OPTIONS) {
            server.sendHeader("Access-Control-Allow-Origin", "*");
            server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
            server.sendHeader("Access-Control-Allow-Headers", "*");
            server.send(204);
        } else {
            server.send(404, "text/plain", "Not Found");
        }
    });

    server.begin();
    Serial.println("🌐 [LOCAL] HTTP Server Started on Port 80!");
}

void system_task(void *arg) 
{
    esp_task_wdt_add(NULL); 

    int lsw1 = digitalRead(switch1); 
    int lsw2 = digitalRead(switch2);
    int lsw3 = digitalRead(switch3); 
    int lsw4 = digitalRead(switch4);
    int lfan = digitalRead(fan_switch);

    uint32_t last_sw_toggle_ms[5] = {0, 0, 0, 0, 0};
    uint8_t sw_chatter_count[5] = {0, 0, 0, 0, 0};

    speed1_flag = 1; 
    speed2_flag = 1; 
    speed3_flag = 1; 
    speed4_flag = 1; 
    speed0_flag = 1;
    
    if (digitalRead(s1) == LOW) 
    {
        speed1_flag = 0;
    }
    else if (digitalRead(s2) == LOW && digitalRead(s3) == HIGH) 
    {
        speed2_flag = 0;
    }
    else if (digitalRead(s2) == LOW && digitalRead(s3) == LOW) 
    {
        speed3_flag = 0;
    }
    else if (digitalRead(s4) == LOW) 
    {
        speed4_flag = 0;
    }
    else 
    {
        speed0_flag = 0;
    }

    static uint64_t reset_press_start = 0; 
    static bool reset_triggered = false; 
    static uint32_t last_valid_code = 0; 
    static uint64_t last_valid_rf_time = 0;

    Serial.println("⚙️ [SYSTEM] Dual-Core Hardware Task Started on Core 1!");

    while (1) 
    {
        esp_task_wdt_reset(); 
        
        uint64_t now_ms = millis();

        if (pairing_target > 0) 
        {
            if (now_ms - pairing_timeout > PAIRING_TIMEOUT_MS) 
            {
                Serial.println("⏳ [SYSTEM] RF Pairing Timeout! Exiting mode.");
                pairing_target = 0; 
            }
        }

        int local_pending_fan = -1;
        
        portENTER_CRITICAL(&state_mux); 
        
        if (pending_fan_speed != -1) 
        { 
            local_pending_fan = pending_fan_speed; 
            pending_fan_speed = -1; 
        } 
        
        portEXIT_CRITICAL(&state_mux);
        
        if (local_pending_fan != -1) 
        {
            if (local_pending_fan == 1) speed_1();
            else if (local_pending_fan == 2) speed_2();
            else if (local_pending_fan == 3) speed_3();
            else if (local_pending_fan == 4) speed_4();
            else speed_0();
        }

        if (rf_available) 
        {
            uint32_t code = rf_received_value;

            if (pairing_target > 0) 
            {
                if (code == last_valid_code && (now_ms - last_valid_rf_time < 2000)) 
                { 
                    rf_available = false; 
                } 
                else 
                {
                    last_valid_code = code; 
                    last_valid_rf_time = now_ms;
                    
                    Serial.printf("📻 [RF PAIRING] Saving Code %lu to Slot %d\n", (unsigned long)code, pairing_target);
                    
                    switch (pairing_target) 
                    {
                        case 1: rf_code_l1 = code; save_code("rf1", code); break;
                        case 2: rf_code_l2 = code; save_code("rf2", code); break;
                        case 3: rf_code_l3 = code; save_code("rf3", code); break;
                        case 4: rf_code_l4 = code; save_code("rf4", code); break;
                        case 5: rf_code_up = code; save_code("rfu", code); break;
                        case 6: rf_code_dw = code; save_code("rfd", code); break;
                        case 7: rf_code_fan_toggle = code; save_code("rft", code); break;
                        case 8: rf_code_master = code; save_code("rfm", code); break;
                    }
                    
                    pairing_target++; 
                    pairing_timeout = now_ms; 
                    pairing_confirm_until_ms = now_ms + PAIRING_CONFIRM_MS;
                    
                    if (pairing_target > 8) 
                    { 
                        pairing_target = 0; 
                        Serial.println("✅ [RF PAIRING] All 8 Slots Complete!"); 
                    }
                    
                    rf_available = false;
                }
            } 
            else 
            {
                bool is_known_code = false;
                
                if (code == rf_code_l1 || code == rf_code_l2 || code == rf_code_l3 || code == rf_code_l4 || code == rf_code_up || code == rf_code_dw || code == rf_code_fan_toggle || code == rf_code_master) 
                {
                    is_known_code = true;
                }

                if (is_known_code) 
                {
                    if (code == last_valid_code && (now_ms - last_valid_rf_time < 1000)) 
                    { 
                        last_valid_rf_time = now_ms; 
                    } 
                    else 
                    {
                        last_valid_code = code; 
                        last_valid_rf_time = now_ms;

                        if (code == rf_code_l1) 
                        { 
                            process_channel_command(1, !switch_state_ch1, -1, "📻 [RF REMOTE]");
                        }
                        else if (code == rf_code_l2) 
                        { 
                            process_channel_command(2, !switch_state_ch2, -1, "📻 [RF REMOTE]");
                        }
                        else if (code == rf_code_l3) 
                        { 
                            process_channel_command(3, !switch_state_ch3, -1, "📻 [RF REMOTE]");
                        }
                        else if (code == rf_code_l4) 
                        { 
                            process_channel_command(4, !switch_state_ch4, -1, "📻 [RF REMOTE]");
                        }
                        else if (code == rf_code_up) 
                        { 
                            int l_speed; 
                            
                            portENTER_CRITICAL(&state_mux); 
                            l_speed = curr_speed; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (l_speed < 4) 
                            { 
                                l_speed++; 
                                Serial.printf("📻 [RF REMOTE] Fan Speed UP -> %d\n", l_speed);
                                
                                if (l_speed == 1) speed_1();
                                else if (l_speed == 2) speed_2();
                                else if (l_speed == 3) speed_3();
                                else speed_4();
                            } 
                            else 
                            {
                                Serial.println("📻 [RF REMOTE] Fan Speed already MAX");
                            }
                        }
                        else if (code == rf_code_dw) 
                        { 
                            int l_speed; 
                            
                            portENTER_CRITICAL(&state_mux); 
                            l_speed = curr_speed; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (l_speed > 0) 
                            { 
                                l_speed--; 
                                Serial.printf("📻 [RF REMOTE] Fan Speed DOWN -> %d\n", l_speed);
                                
                                if (l_speed == 0) speed_0();
                                else if (l_speed == 1) speed_1();
                                else if (l_speed == 2) speed_2();
                                else speed_3();
                            } 
                            else 
                            {
                                Serial.println("📻 [RF REMOTE] Fan Speed already MIN");
                            }
                        }
                        else if (code == rf_code_fan_toggle) 
                        { 
                            bool f_pow; 
                            
                            portENTER_CRITICAL(&state_mux); 
                            f_pow = fan_power; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (f_pow) 
                            {
                                Serial.println("📻 [RF REMOTE] Fan Toggle -> OFF");
                                speed_0();
                            }
                            else 
                            {
                                Serial.println("📻 [RF REMOTE] Fan Toggle -> ON");
                                restore_fan_speed();
                            }
                        }
                        else if (code == rf_code_master) 
                        { 
                            Serial.println("📻 [RF REMOTE] Master ON/OFF Button Pressed");
                            
                            bool s1, s2, s3, s4, f_pow; 
                            
                            portENTER_CRITICAL(&state_mux); 
                            s1 = switch_state_ch1; 
                            s2 = switch_state_ch2; 
                            s3 = switch_state_ch3; 
                            s4 = switch_state_ch4; 
                            f_pow = fan_power; 
                            portEXIT_CRITICAL(&state_mux);
                            
                            if (s1 || s2 || s3 || s4 || f_pow) 
                            {
                                All_Off();
                            }
                            else 
                            {
                                All_On();
                            }
                        }
                        
                        pairing_confirm_until_ms = now_ms + 200U; 
                    }
                }
                
                rf_available = false;
            }
        }

        // 1. Channel 1 Physical Switch (GPIO 32 - Pullup)
        int raw_sw1 = digitalRead(switch1);
        if (raw_sw1 != lsw1)
        {
            bool stable = true;
            for (int s = 0; s < 4; s++)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (digitalRead(switch1) != raw_sw1) { stable = false; break; }
            }
            if (stable)
            {
                uint32_t t_now = millis();
                if (t_now - last_sw_toggle_ms[0] > 250)
                {
                    last_sw_toggle_ms[0] = t_now;
                    lsw1 = raw_sw1;
                    bool is_pressed = (raw_sw1 == 0);
                    process_channel_command(1, is_pressed, -1, "🔘 [PHYSICAL SWITCH]");

                    if (pairing_target == 0) 
                    {
                        uint32_t t_diff = now_ms - last_switch1_toggle_time;
                        if (t_diff > 1500) switch1_toggle_count = 1; 
                        else if (t_diff > 100) switch1_toggle_count++; 
                        last_switch1_toggle_time = now_ms;
                        
                        if (switch1_toggle_count >= SWITCH1_PAIR_TOGGLES) 
                        { 
                            Serial.println("⚙️ [SYSTEM] Triggering RF Pairing Mode via Switch 1!");
                            pairing_target = 1; 
                            pairing_timeout = now_ms; 
                            switch1_toggle_count = 0; 
                            pairing_confirm_until_ms = now_ms + PAIRING_CONFIRM_MS; 
                        }
                    }
                }
            }
        }

        // 2. Channel 2 Physical Switch (GPIO 35 - Input Only, Filter Floating Noise)
        int raw_sw2 = digitalRead(switch2);
        if (raw_sw2 != lsw2)
        {
            bool stable = true;
            for (int s = 0; s < 4; s++)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (digitalRead(switch2) != raw_sw2) { stable = false; break; }
            }
            if (stable)
            {
                uint32_t t_now = millis();
                if (t_now - last_sw_toggle_ms[1] > 250)
                {
                    last_sw_toggle_ms[1] = t_now;
                    lsw2 = raw_sw2;
                    bool is_pressed = (raw_sw2 == 0);
                    process_channel_command(2, is_pressed, -1, "🔘 [PHYSICAL SWITCH]");

                    if (pairing_target == 0) 
                    {
                        uint32_t t_diff = now_ms - last_switch2_toggle_time;
                        if (t_diff > 1500) switch2_toggle_count = 1; 
                        else if (t_diff > 100) switch2_toggle_count++; 
                        last_switch2_toggle_time = now_ms;
                        
                        if (switch2_toggle_count >= SWITCH2_RESET_TOGGLES) 
                        { 
                            Serial.println("🚨 [SYSTEM] FACTORY RESET VIA SWITCH 2 Toggled 20 Times!");
                            preferences.begin("wifi", false); 
                            preferences.clear(); 
                            preferences.end(); 
                            delay(500); 
                            ESP.restart(); 
                        }
                    }
                }
            }
        }

        // 3. Channel 3 Physical Switch (GPIO 34 - Input Only, Filter Floating Noise)
        int raw_sw3 = digitalRead(switch3);
        if (raw_sw3 != lsw3)
        {
            bool stable = true;
            for (int s = 0; s < 4; s++)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (digitalRead(switch3) != raw_sw3) { stable = false; break; }
            }
            if (stable)
            {
                uint32_t t_now = millis();
                if (t_now - last_sw_toggle_ms[2] > 250)
                {
                    last_sw_toggle_ms[2] = t_now;
                    lsw3 = raw_sw3;
                    bool is_pressed = (raw_sw3 == 0);
                    process_channel_command(3, is_pressed, -1, "🔘 [PHYSICAL SWITCH]");
                }
            }
        }

        // 4. Channel 4 Physical Switch (GPIO 39 - Input Only, Filter Floating Noise)
        int raw_sw4 = digitalRead(switch4);
        if (raw_sw4 != lsw4)
        {
            bool stable = true;
            for (int s = 0; s < 4; s++)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (digitalRead(switch4) != raw_sw4) { stable = false; break; }
            }
            if (stable)
            {
                uint32_t t_now = millis();
                if (t_now - last_sw_toggle_ms[3] > 250)
                {
                    last_sw_toggle_ms[3] = t_now;
                    lsw4 = raw_sw4;
                    bool is_pressed = (raw_sw4 == 0);
                    process_channel_command(4, is_pressed, -1, "🔘 [PHYSICAL SWITCH]");
                }
            }
        }

        // 5. Fan Physical Switch (GPIO 33 - Pullup)
        int raw_fan = digitalRead(fan_switch);
        if (raw_fan != lfan)
        {
            bool stable = true;
            for (int s = 0; s < 4; s++)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (digitalRead(fan_switch) != raw_fan) { stable = false; break; }
            }
            if (stable)
            {
                uint32_t t_now = millis();
                if (t_now - last_sw_toggle_ms[4] > 250)
                {
                    last_sw_toggle_ms[4] = t_now;
                    lfan = raw_fan;
                    if (raw_fan == 0)
                    {
                        Serial.println("🔘 [PHYSICAL SWITCH] Fan Toggle Switch -> ON");
                        int l_speed; 
                        portENTER_CRITICAL(&state_mux); 
                        l_speed = curr_speed; 
                        portEXIT_CRITICAL(&state_mux); 
                        if (l_speed == 0) restore_fan_speed(); 
                    }
                    else
                    {
                        Serial.println("🔘 [PHYSICAL SWITCH] Fan Toggle Switch -> OFF");
                        speed_0(); 
                    }
                }
            }
        }

        if (digitalRead(s1) == LOW && speed1_flag == 1) 
        { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary Knob -> Speed 1");
            speed_1(); 
            
            speed1_flag = 0; 
            speed2_flag = 1; 
            speed3_flag = 1; 
            speed4_flag = 1; 
            speed0_flag = 1; 
        }
        
        if (digitalRead(s2) == LOW && digitalRead(s3) == HIGH && speed2_flag == 1) 
        { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary Knob -> Speed 2");
            speed_2(); 
            
            speed1_flag = 1; 
            speed2_flag = 0; 
            speed3_flag = 1; 
            speed4_flag = 1; 
            speed0_flag = 1; 
        }
        
        if (digitalRead(s2) == LOW && digitalRead(s3) == LOW && speed3_flag == 1) 
        { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary Knob -> Speed 3");
            speed_3(); 
            
            speed1_flag = 1; 
            speed2_flag = 1; 
            speed3_flag = 0; 
            speed4_flag = 1; 
            speed0_flag = 1; 
        }
        
        if (digitalRead(s4) == LOW && speed4_flag == 1) 
        { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary Knob -> Speed 4");
            speed_4(); 
            
            speed1_flag = 1; 
            speed2_flag = 1; 
            speed3_flag = 1; 
            speed4_flag = 0; 
            speed0_flag = 1; 
        }
        
        if (digitalRead(s1) == HIGH && digitalRead(s2) == HIGH && digitalRead(s3) == HIGH && digitalRead(s4) == HIGH && speed0_flag == 1) 
        { 
            Serial.println("🎛️ [FAN REGULATOR] Rotary Knob -> Speed 0 (OFF)");
            speed_0(); 
            
            speed1_flag = 1; 
            speed2_flag = 1; 
            speed3_flag = 1; 
            speed4_flag = 1; 
            speed0_flag = 0; 
        }

        if (digitalRead(gpio_reset) == LOW) 
        {
            if (reset_press_start == 0) 
            {
                reset_press_start = now_ms;
            }
            else if (now_ms - reset_press_start >= 5000) 
            { 
                if (!reset_triggered) 
                { 
                    Serial.println("🚨 [SYSTEM] FACTORY RESET VIA BOOT BUTTON PRESSED > 5 Sec!");
                    reset_triggered = true; 
                    preferences.begin("wifi", false); 
                    preferences.clear(); 
                    preferences.end(); 
                    ESP.restart(); 
                } 
            }
        } 
        else 
        { 
            reset_press_start = 0; 
            reset_triggered = false; 
        }

        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

class MyServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    { 
        Serial.println("🔵 [BLE] Phone Connected for Setup!"); 
    }
    
    void onDisconnect(BLEServer* pServer) 
    { 
        Serial.println("🔵 [BLE] Phone Disconnected from Setup!"); 
        pServer->getAdvertising()->start(); 
    }
};

class WifiWriteCallback: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pChar) 
    {
        String value = pChar->getValue().c_str();
        
        if (value.length() > 0) 
        {
            StaticJsonDocument<256> doc; 
            DeserializationError error = deserializeJson(doc, value);
            
            if (!error) 
            {
                const char* rSsid = doc["ssid"]; 
                const char* rPass = doc["pass"];
                
                if (rSsid) 
                {
                    Serial.printf("🔑 [BLE] Received New WiFi Credentials: %s\n", rSsid);
                    preferences.begin("wifi", false);
                    preferences.putString("ssid", rSsid); 
                    
                    if (rPass) 
                    {
                        preferences.putString("pass", rPass);
                    }
                    else 
                    {
                        preferences.putString("pass", "");
                    }
                    
                    preferences.end();
                    Serial.println("⚙️ [SYSTEM] Rebooting to connect to new Wi-Fi...");
                    shouldReboot = true;
                }
            }
        }
    }
};

class DeviceIdWriteCallback: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pChar) 
    { 
        if (pChar->getValue().length() > 0) 
        {
            shouldReboot = true; 
        }
    }
};

void startSetupPortal() 
{
    inSetupMode = true; 
    char portalSSID[50]; 
    snprintf(portalSSID, sizeof(portalSSID), "4Layers-ARQV2.0-%s", NODE_ID + 8);
    
    Serial.printf("⚙️ [SYSTEM] BLE Setup Active! Connect your Phone to Bluetooth: %s\n", portalSSID);

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
    Serial.println("🔵 [BLE] Advertising Started!");
}

void webserver_task(void *pvParameters) 
{
    esp_task_wdt_add(NULL); 
    
    while (true) 
    {
        esp_task_wdt_reset(); 
        
        if (WiFi.status() == WL_CONNECTED || inSetupMode) 
        {
            server.handleClient();
        }
        
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void mqtt_task(void *pvParameters) 
{
    esp_task_wdt_add(NULL);
    unsigned long last_heartbeat_ms = 0;
    
    while (true) 
    {
        esp_task_wdt_reset();
        
        if (WiFi.status() == WL_CONNECTED && !inSetupMode) 
        {
            bool link_up;
            if (MQTT_LOCK_TAKE(pdMS_TO_TICKS(1000)))
            {
                link_up = client.connected();
                MQTT_LOCK_GIVE();
            }
            else
            {
                // Another task holds the client; try again next tick.
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            mqtt_link_up = link_up;
            
            if (!link_up) 
            {
                Serial.printf("📊 [HEAP] Before TLS Connect - Free: %lu bytes, Min Free: %lu bytes, Max Alloc: %lu bytes\n", 
                              (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
                Serial.println("☁️ Connecting to EMQX Cloud...");
                
                // Explicitly stop previous client session to clear stale buffers and prevent TLS -29184 error
                espClient.stop();
                
                // Ensure DNS servers are active on lwIP
                ip_addr_t d1, d2;
                d1.type = IPADDR_TYPE_V4;
                d1.u_addr.ip4.addr = ipaddr_addr("8.8.8.8");
                dns_setserver(0, &d1);
                d2.type = IPADDR_TYPE_V4;
                d2.u_addr.ip4.addr = ipaddr_addr("1.1.1.1");
                dns_setserver(1, &d2);
                
                char clientId[50];
                // Unique per attempt: MQTT brokers evict an existing session when a
                // second connection presents the same client ID. A fixed ID means a
                // lingering server-side session (or any other client reusing it) kicks
                // us in a loop, which looks like a silent disconnect every few seconds.
                snprintf(clientId, sizeof(clientId), "4L-Client-%s-%04X", NODE_ID, (unsigned)(esp_random() & 0xFFFF));
                
                // client.connect() blocks for the whole TLS handshake, which is
                // allowed up to setHandshakeTimeout(30) seconds. The task WDT fires
                // at 5s, so this task must leave the WDT's watch list for the
                // duration of the call or the handshake itself trips the watchdog.
                // The mutex is held across connect + subscribe so no other task can
                // publish into a half-established session.
                if (!MQTT_LOCK_TAKE(pdMS_TO_TICKS(2000)))
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                
                esp_task_wdt_delete(NULL);
                bool mqtt_connected = client.connect(clientId, mqtt_user, mqtt_pass, status_topic, 1, false, "{\"status\":\"OFFLINE\",\"is_online\":false}");
                esp_task_wdt_add(NULL);
                esp_task_wdt_reset();
                
                if (mqtt_connected) 
                {
                    Serial.printf("✅ EMQX Cloud Connected! Free Heap: %lu bytes, Min Free: %lu bytes\n", 
                                  (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap());
                    client.subscribe(command_topic);
                    
                    char ota_topic_node[120];
                    snprintf(ota_topic_node, sizeof(ota_topic_node), "smartnest/devices/%s/ota", NODE_ID);
                    client.subscribe(ota_topic_node);
                    client.subscribe("smartnest/devices/all/ota");

                    StaticJsonDocument<128> ipDoc;
                    ipDoc["local_ip"] = WiFi.localIP().toString();
                    char ipBuffer[128];
                    serializeJson(ipDoc, ipBuffer);
                    
                    char infoTopic[120];
                    snprintf(infoTopic, sizeof(infoTopic), "home/device/%s/info", NODE_ID);
                    
                    if (client.publish(infoTopic, ipBuffer, true)) 
                    {
                        Serial.printf("📡 [INFO] Local IP Published to App: %s\n", WiFi.localIP().toString().c_str());
                    } 
                    else 
                    {
                        Serial.println("❌ [ERROR] Failed to publish Local IP to Cloud!");
                    }
                    
                    // Release before sendChannelState() — the mutex is not recursive
                    // and that helper acquires it for each publish.
                    MQTT_LOCK_GIVE();
                    
                    sendChannelState(1, switch_state_ch1); 
                    sendChannelState(2, switch_state_ch2);
                    sendChannelState(3, switch_state_ch3); 
                    sendChannelState(4, switch_state_ch4);
                    sendChannelState(5, fan_power, curr_speed);
                    
                    bool master_state = switch_state_ch1 || switch_state_ch2 || switch_state_ch3 || switch_state_ch4 || fan_power;
                    sendChannelState(6, master_state);
                    
                    last_heartbeat_ms = millis();
                } 
                else 
                { 
                    Serial.printf("❌ [ERROR] Cloud connection failed, rc=%d. Free Heap: %lu bytes. Next retry in 15s...\n", 
                                  client.state(), (unsigned long)ESP.getFreeHeap());
                    espClient.stop();
                    MQTT_LOCK_GIVE();
                    
                    // Non-blocking 15-second delay on Core 1 so CPU 0 and Local WebServer remain 100% responsive
                    for (int i = 0; i < 15; i++) 
                    {
                        esp_task_wdt_reset();
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                    continue;
                }
            } 
            else 
            {
                if (MQTT_LOCK_TAKE(pdMS_TO_TICKS(200)))
                {
                    client.loop();
                    MQTT_LOCK_GIVE();
                }

                unsigned long now_ms = millis();
                if (now_ms - last_heartbeat_ms >= 60000 || last_heartbeat_ms == 0) 
                {
                    last_heartbeat_ms = now_ms;
                    StaticJsonDocument<192> hbDoc;
                    hbDoc["is_online"] = true;
                    hbDoc["status"] = "HEARTBEAT";
                    hbDoc["local_ip"] = WiFi.localIP().toString();
                    hbDoc["rssi"] = WiFi.RSSI();
                    char hbBuffer[192];
                    serializeJson(hbDoc, hbBuffer);
                    
                    if (MQTT_LOCK_TAKE(pdMS_TO_TICKS(500)))
                    {
                        client.publish(status_topic, hbBuffer, false);
                        MQTT_LOCK_GIVE();
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() 
{
    Serial.begin(115200);
    delay(1000);
    
    esp_task_wdt_config_t twdt_config = 
    {
        .timeout_ms = 15000, 
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&twdt_config);
    esp_task_wdt_add(NULL); 

    Serial.println("\n\n===========================================");
    Serial.println("🚀 Go Smart Firmware V12.5 (PRO-OPTIMIZED)");
    Serial.println("🔍 LOGGING: ALL SENSORS & EVENTS ACTIVATED!");
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

    if (switch_state_ch1) digitalWrite(relay1, HIGH); else digitalWrite(relay1, LOW);
    if (switch_state_ch2) digitalWrite(relay2, HIGH); else digitalWrite(relay2, LOW);
    if (switch_state_ch3) digitalWrite(relay3, HIGH); else digitalWrite(relay3, LOW);
    if (switch_state_ch4) digitalWrite(relay4, HIGH); else digitalWrite(relay4, LOW);
    
    if (curr_speed == 1) speed_1();
    else if (curr_speed == 2) speed_2();
    else if (curr_speed == 3) speed_3();
    else if (curr_speed == 4) speed_4();
    else speed_0();

    uint64_t mac = ESP.getEfuseMac(); 
    uint8_t* m = (uint8_t*)&mac;
    snprintf(NODE_ID, sizeof(NODE_ID), "4L-NODE-%02X%02X%02X", m[3], m[4], m[5]);
    snprintf(command_topic, sizeof(command_topic), "home/device/%s/control", NODE_ID);
    snprintf(status_topic, sizeof(status_topic), "home/device/%s/status", NODE_ID);

    Serial.println("\n====================================");
    Serial.printf("📡 Node ID Generated: %s\n", NODE_ID);
    Serial.println("====================================\n");
    
    // Initialize Command Queue (depth 16) for buffering sequential channel and bulk commands
    command_queue = xQueueCreate(16, sizeof(switch_command_t));
    if (command_queue == NULL) 
    {
        Serial.println("❌ [ERROR] Failed to create FreeRTOS Command Queue!");
    }
    else 
    {
        Serial.println("✅ [SYSTEM] FreeRTOS Command Queue (depth 16) Initialized!");
    }

    // Serialises every PubSubClient access; must exist before any task can publish.
    mqtt_lock = xSemaphoreCreateRecursiveMutex();
    if (mqtt_lock == NULL) 
    {
        Serial.println("❌ [ERROR] Failed to create MQTT mutex!");
    }
    
    // Command Worker Task strictly on Core 1 (Priority 4, Stack 4KB)
    xTaskCreatePinnedToCore(command_worker_task, "cmd_worker", 4096, NULL, 4, NULL, 1);

    // RAM Reduced to 4KB (4096) for faster execution!
    xTaskCreatePinnedToCore(system_task, "system_task", 4096, NULL, 5, NULL, 1);
    
    xTaskCreatePinnedToCore(nvs_commit_task, "nvs_commit", 3072, NULL, 3, NULL, 1);

    preferences.begin("wifi", true);
    saved_ssid = preferences.getString("ssid", "");
    saved_password = preferences.getString("pass", "");
    preferences.end();

    if (saved_ssid.length() > 0) 
    {
        Serial.printf("🌐 Saved Wi-Fi found: %s\n", saved_ssid.c_str());
        WiFi.mode(WIFI_STA); 
        WiFi.disconnect(true); 
        delay(100);
        
        WiFi.setSleep(false); 
        WiFi.setAutoReconnect(true); 
        
        // Configure DNS via lwIP directly
        ip_addr_t d1, d2;
        d1.type = IPADDR_TYPE_V4;
        d1.u_addr.ip4.addr = ipaddr_addr("8.8.8.8");
        dns_setserver(0, &d1);
        d2.type = IPADDR_TYPE_V4;
        d2.u_addr.ip4.addr = ipaddr_addr("1.1.1.1");
        dns_setserver(1, &d2);

        bool connected = false;
        
        for (int attempt = 1; attempt <= 4; attempt++) 
        {
            esp_task_wdt_reset(); 
            
            Serial.printf("[WiFi] Attempt %d/4: Connecting to '%s'...\n", attempt, saved_ssid.c_str());
            WiFi.disconnect(); 
            delay(300);
            
            WiFi.begin(saved_ssid.c_str(), saved_password.c_str());

            int retries = 0;
            
            while (WiFi.status() != WL_CONNECTED && retries < 30) 
            {
                delay(500); 
                Serial.print(".");
                digitalWrite(wifiLed, !digitalRead(wifiLed)); 
                retries++;
            }
            
            Serial.println();

            if (WiFi.status() == WL_CONNECTED) 
            { 
                connected = true; 
                break; 
            }
            
            Serial.printf("❌ [WiFi] Attempt %d failed. Waiting 3 seconds before next retry...\n", attempt);
            digitalWrite(wifiLed, HIGH); 
            delay(3000);
        }
        
        // Re-set Google & Cloudflare DNS in lwIP after DHCP connection
        dns_setserver(0, &d1);
        dns_setserver(1, &d2);

        // SNTP / NTP time sync — CRITICAL FOR TLS.
        // ESP32 cold-boot has RTC = 0 (Jan 1970), so any valid server cert looks
        // "not yet valid" to mbedTLS and X509 verification fails (-9984) even with
        // the correct root CA. Block up to ~8s for NTP; if it succeeds verify the
        // chain against DigiCert Global Root G2, else fall back to insecure TLS so
        // MQTT still connects in degraded mode.
        configTime(0, 0, "time.google.com", "pool.ntp.org", "time.cloudflare.com");
        bool timeOk = false;
        for (int t = 0; t < 8; t++) {
            time_t now = time(nullptr);
            if (now > 1700000000) { timeOk = true; break; }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (timeOk) {
            Serial.println("🕒 [NTP] System time synced; TLS will verify EMQX cert.");
            espClient.setCACert(EMQX_ROOT_CA);
        } else {
            Serial.println("⚠️ [NTP] Time sync failed! Falling back to insecure TLS for MQTT.");
            espClient.setInsecure();
        }
        // Do NOT shorten the socket timeout here. setTimeout() drives SO_RCVTIMEO
        // for every read, and PubSubClient's loop() polls available() constantly.
        // With a 2s recv timeout mbedtls_ssl_read() returns NET_RECV_FAILED (-76)
        // on an idle socket, which NetworkClientSecure treats as fatal and closes
        // the connection — causing a reconnect every ~15-30s. The 30s library
        // default lets idle reads block harmlessly.
        espClient.setHandshakeTimeout(30);      // EMQX TLS handshake can take 10-25s on weak RSSI
        
        client.setServer(mqtt_server, mqtt_port); 
        client.setCallback(mqtt_callback);
        client.setKeepAlive(120); 
        client.setBufferSize(1024);
        // PubSubClient's default 15s socket timeout applies to readByte() while a
        // packet is mid-flight. Keep it well under the 60s keepalive so a stalled
        // read is detected, but long enough that a slow link is not mistaken for
        // a dead connection.
        client.setSocketTimeout(30);

        setupLocalWebServer(); 
        
        // Strict Core Isolation:
        // WebServer strictly on Core 0 (High Priority 5, Stack 4KB)
        xTaskCreatePinnedToCore(webserver_task, "webserver_task", 6144, NULL, 5, NULL, 0);
        
        // MQTT Task strictly on Core 1 (Priority 2, Stack 4KB)
        xTaskCreatePinnedToCore(mqtt_task, "mqtt_task", 12288, NULL, 2, NULL, 1);

        if (connected) 
        {
            Serial.println("\n✅ Wi-Fi Connected Successfully!");
            if (MDNS.begin(NODE_ID)) 
            {
                Serial.printf("🌐 mDNS Started! Access via: http://%s.local\n", NODE_ID);
            }
        } 
        else 
        {
            Serial.println("\n⏳ [WARNING] Wi-Fi router offline or connection failed. ESP will keep trying in background...");
        }
    } 
    else 
    {
        Serial.println("🌐 [WARNING] No Wi-Fi Credentials found. Starting Setup Portal.");
        startSetupPortal();
    }
}

void loop() 
{
    esp_task_wdt_reset(); 
    
    unsigned long now_ms = millis();
    
    if (inSetupMode) 
    {
        if (shouldReboot) 
        { 
            Serial.println("🔄 Rebooting in 1 sec..."); 
            delay(1000); 
            ESP.restart(); 
        }
        
        digitalWrite(wifiLed, (now_ms / 500) % 2); 
        delay(10); 
        return; 
    }

    if (pairing_confirm_until_ms > now_ms) 
    {
        digitalWrite(wifiLed, HIGH);
    }
    else if (pairing_target > 0) 
    {
        digitalWrite(wifiLed, (now_ms / 120) % 2);
    }
    else if (mqtt_link_up) 
    {
        digitalWrite(wifiLed, HIGH);
    }
    else 
    {
        digitalWrite(wifiLed, (now_ms / 1000) % 2);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}

extern "C" void app_main() 
{
    initArduino();
    setup();
    
    while (true) 
    {
        loop();
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}