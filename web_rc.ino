// Web remote server

#if WEB_RC_ENABLED

#include <WebServer.h>
#include <WiFi.h>
#include "web_rc_html.h"
#include "board_config.h"

// Shared flight control inputs used by the web remote, SBUS and MAVLink
extern float t;
extern float controlTime;
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode;

// ==================== Configuration ====================
#define WEB_RC_TIMEOUT_MS    10000          // Disconnect after this many milliseconds without a packet. Allow more than one heartbeat interval.
// battery.ino defines VBAT_ADC_PIN, VBAT_ADC_SAMPLES and VBAT_DIVIDER.

// ==================== Connection state ====================
bool webRCEnabled    = false;  // True for an active web connection. readWebRC() updates this each frame.
bool useWebRC        = false;  // True while web control is in use. Kept in sync with webRCEnabled.
bool webRCUpdated    = false;  // True after receiving the first stick update.
bool webConsoleEnabled = false; // POST /console/enable turns on the web console so print() writes to its buffer.
char webRCWarnMsg[64] = ""; // Pending warning for the browser. Clear it after sending once.

// ==================== Last request details used in the response ====================
static int lastProcType        = 0;   // 1 = stick data, 2 = button, 4 = heartbeat
static int lastProcButtonIdx   = -1;  // Button index (0 to 15), or -1 for other requests
static int lastProcButtonState = -1;  // Button pressed (1) or released (0), or -1 for other requests

// ==================== Last stick values for the status endpoint ====================
// Throttle is 0 to 100%, attitude axes are ±30° and buttons are pressed (1) or released (0).
float webRCRoll     = 0.0f;
float webRCPitch    = 0.0f;
float webRCYaw      = 0.0f;
float webRCThrottle = 0.0f;
uint16_t webRCButtons    = 0;       // Button bits: 0 arm, 1 disarm, 2 stop, 6 STAB, 7 ACRO, 8 ALTHOLD.
unsigned long webRCLastUpdate = 0;  // Time of the last packet from millis()

// ==================== Sensitivity settings ====================
// Write angle × scale / STICK_MAX to the shared control inputs (range -1 to 1).
// Lower scale gives a gentler response. Higher scale increases sensitivity.
float webRCThrottleScale = 1.0f;    // Maximum throttle scale. A value of 1.0 allows full throttle and 0.8 limits it to 80%.
float webRCStickScale    = 0.85f;   // Roll and pitch sensitivity (suggested range 0.5 to 1.0)
float webRCYawScale      = 0.68f;   // Yaw sensitivity is lower because yaw inertia is smaller (suggested range 0.3 to 0.8).

// ==================== Stick dead zones ====================
// Within the dead zone, small stick offsets produce zero output. Values are fractions of full travel.
// A wider dead zone ignores more accidental input. A narrower one responds sooner but may allow drift.
float stickDeadzone    = 0.06f;     // Shared 6% dead zone for roll, pitch and yaw
float throttleDeadzone = 0.06f;     // Low-end throttle dead zone. Output stays at zero below this point.

// ==================== Input ranges must match the browser. ====================
static const float THROTTLE_MIN = 0.0f;    // Minimum throttle output (%)
static const float THROTTLE_MAX = 100.0f;  // Maximum throttle output (%)
static const float STICK_MAX    = 30.0f;   // Angle at full stick travel (degrees). Adjust PID TILT_MAX if this changes.
static const float RAW_MAX      = 100.0f;  // The browser sends ±100 at full stick travel.

// ==================== Internal runtime state ====================
static float lastValidThrottle = 0.0f;     // Last valid throttle, used if a new value is invalid
static float lastValidRoll     = 0.0f;     // Last valid roll
static float lastValidPitch    = 0.0f;     // Last valid pitch
static float lastValidYaw      = 0.0f;     // Last valid yaw
static unsigned long lastDataErrorTime = 0; // Time of the last input error, used to limit log messages

// ==================== Web server ====================
WebServer webRCServer(80);          // Use standard HTTP port 80 so the URL needs no port number.

// ------ Redirect port 8080 to 80 with HTTP 301 for addresses printed on older boards. ------------
// Allocate the port 8080 server in setupWebRC(). Only the 4-byte pointer uses static RAM.
// The C3 has no older boards to support. Skip the extra socket and the 200 ms polling wait.
#ifndef CONFIG_IDF_TARGET_ESP32C3
static WiFiServer* redirectServer8080 = nullptr;
#endif
// ------ Redirect port 8080 to 80 with HTTP 301 for addresses printed on older boards. ------------

// ==================== Console log buffer ====================
#define CONSOLE_LINES    BOARD_CONSOLE_LINES    // Keep 20 lines on C3 (saves about 7 KB of RAM) or 50 on ESP32 and S3.
#define CONSOLE_LINE_LEN BOARD_CONSOLE_LINE_LEN  // Characters per line: 160 on C3 or 240 on ESP32 and S3.
static char consoleBuf[CONSOLE_LINES][CONSOLE_LINE_LEN];
static int  consoleTail   = 0;
static int  consoleFilled = 0;
static int  consoleTotal  = 0;   // Running line count so clients can request only new lines

#define CONSOLE_CMD_QUEUE_SIZE 4
#define CONSOLE_CMD_LEN 64
static char consoleCmdQueue[CONSOLE_CMD_QUEUE_SIZE][CONSOLE_CMD_LEN];
static int  consoleCmdHead  = 0;
static int  consoleCmdTail  = 0;
static int  consoleCmdCount = 0;

bool enqueueConsoleCmd(const char* cmd) {
    if (!cmd || !*cmd) return false;
    if (consoleCmdCount >= CONSOLE_CMD_QUEUE_SIZE) return false;
    strncpy(consoleCmdQueue[consoleCmdTail], cmd, CONSOLE_CMD_LEN - 1);
    consoleCmdQueue[consoleCmdTail][CONSOLE_CMD_LEN - 1] = '\0';
    consoleCmdTail = (consoleCmdTail + 1) % CONSOLE_CMD_QUEUE_SIZE;
    consoleCmdCount++;
    return true;
}

void processConsoleCommandQueue() {
    if (consoleCmdCount <= 0) return;

    String cmd = consoleCmdQueue[consoleCmdHead];
    consoleCmdHead = (consoleCmdHead + 1) % CONSOLE_CMD_QUEUE_SIZE;
    consoleCmdCount--;
    doCommand(cmd, false);
}

void webLog(const char* msg) {
    // Split on newlines before writing to keep line breaks out of the JSON strings.
    const char* start = msg;
    while (*start) {
        const char* end = strchr(start, '\n');
        int len = end ? (int)(end - start) : (int)strlen(start);
        if (len > 0) {
            int copy = (len < CONSOLE_LINE_LEN - 1) ? len : (CONSOLE_LINE_LEN - 1);
            strncpy(consoleBuf[consoleTail], start, copy);
            consoleBuf[consoleTail][copy] = '\0';
            consoleTail = (consoleTail + 1) % CONSOLE_LINES;
            if (consoleFilled < CONSOLE_LINES) consoleFilled++;
            consoleTotal++;
        }
        if (!end) break;
        start = end + 1;
    }
}

// ==================== Stick processing ====================

// Normalized dead zone with input and output in [-1, 1]
// Scale travel outside the dead zone linearly so full stick still gives full output.
float applyDeadzone(float norm, float deadzone) {
    if (fabsf(norm) < deadzone) return 0.0f;
    float sign = (norm > 0.0f) ? 1.0f : -1.0f;
    return sign * (fabsf(norm) - deadzone) / (1.0f - deadzone);
}

// Process roll, pitch and yaw.
// Map raw input from ±100 to ±STICK_MAX degrees.
float processAxis(float raw, float& lastValid) {
    if (isnan(raw) || isinf(raw) || fabsf(raw) > 1000.0f) {
        if (millis() - lastDataErrorTime > 2000) lastDataErrorTime = millis();
        return lastValid;
    }
    float norm = constrain(raw, -RAW_MAX, RAW_MAX) / RAW_MAX;
    norm = applyDeadzone(norm, stickDeadzone);
    lastValid = norm * STICK_MAX;
    return lastValid;
}

// Process throttle.
// The browser sends -100 at the bottom of the left stick and +100 at the top.
// Output range is 0 to 100%.
float processThrottle(float raw) {
    if (isnan(raw) || isinf(raw) || fabsf(raw) > 1000.0f) return lastValidThrottle;
    raw = constrain(raw, -RAW_MAX, RAW_MAX);
    // Map -100 to 0%, 0 to 50% and +100 to 100%.
    float pct = (raw + RAW_MAX) / (2.0f * RAW_MAX) * THROTTLE_MAX;
    if (pct < throttleDeadzone * THROTTLE_MAX) pct = 0.0f;
    pct = constrain(pct * webRCThrottleScale, THROTTLE_MIN, THROTTLE_MAX);
    lastValidThrottle = pct;
    return pct;
}

// ==================== Process control input ====================

void setWebRCInput(float roll, float pitch, float yaw, float throttle, uint16_t buttons) {
    float pThrottle = processThrottle(throttle);
    float pYaw      = processAxis(yaw,   lastValidYaw);
    float pPitch    = processAxis(pitch, lastValidPitch);
    float pRoll     = processAxis(roll,  lastValidRoll);

    // Store the processed values for the status endpoint.
    webRCThrottle = pThrottle;
    webRCYaw      = pYaw;
    webRCPitch    = pPitch;
    webRCRoll     = pRoll;
    webRCButtons  = buttons;
    webRCLastUpdate = millis();
    webRCUpdated  = true;

    // Update the same inputs used by SBUS and MAVLink.
    controlRoll     = constrain(pRoll  * webRCStickScale / STICK_MAX, -1.0f, 1.0f);
    controlPitch    = constrain(pPitch * webRCStickScale / STICK_MAX, -1.0f, 1.0f);
    controlYaw      = constrain(pYaw   * webRCYawScale   / STICK_MAX, -1.0f, 1.0f);
    controlThrottle = pThrottle / THROTTLE_MAX;
    controlMode     = NAN;
    controlTime     = t;

    static float lastPrintedThrottle = -1.0f;
    if (fabsf(pThrottle - lastPrintedThrottle) > 5.0f) {
        print("WebRC T=%.0f%% R=%.1f P=%.1f Y=%.1f Btn=0x%04X\n",
              pThrottle, pRoll, pPitch, pYaw, buttons);
        lastPrintedThrottle = pThrottle;
    }
}

// ==================== JSON request handling ====================

const char* findJsonValue(const char* json, const char* key) {
    const char* p = strstr(json, key);
    if (!p) return nullptr;
    p = strchr(p, ':');
    if (!p) return nullptr;
    p++;
    while (*p == ' ' || *p == '"') p++;
    return p;
}

bool handleJSONProtocol(String& body) {
    if (body.length() == 0 || body.indexOf('{') == -1) return false;
    const char* json = body.c_str();
    const char* typePos = strstr(json, "\"t\":");
    if (!typePos) return false;
    int type = atoi(typePos + 4);
    const char* v;

    lastProcType        = type;
    lastProcButtonIdx   = -1;
    lastProcButtonState = -1;

    switch (type) {
        case 1: { // Stick data
            float th = 0, r = 0, p = 0, y = 0;
            uint32_t ts = 0;
            if ((v = findJsonValue(json, "\"th\""))) th = atof(v);
            if ((v = findJsonValue(json, "\"r\"")))  r  = atof(v);
            if ((v = findJsonValue(json, "\"p\"")))  p  = atof(v);
            if ((v = findJsonValue(json, "\"y\"")))  y  = atof(v);
            if ((v = findJsonValue(json, "\"ts\""))) ts = atol(v);
            setWebRCInput(r, p, y, th, webRCButtons);
            break;
        }
        case 2: { // Button event
            int idx = 0, state = 0;
            uint32_t ts = 0;
            if ((v = findJsonValue(json, "\"b\"")))  idx   = atoi(v);
            if ((v = findJsonValue(json, "\"s\"")))  state = atoi(v);
            if ((v = findJsonValue(json, "\"ts\""))) ts    = atol(v);
            if (idx >= 0 && idx < 16) {
                if (state) webRCButtons |=  (1 << idx);
                else       webRCButtons &= ~(1 << idx);
                webRCLastUpdate = millis();
                webRCUpdated    = true;
                lastProcButtonIdx   = idx;
                lastProcButtonState = state;
            }
            break;
        }
        case 4: // Heartbeat
            webRCLastUpdate = millis();
            webRCUpdated    = true;
            break;
    }
    return true;
}

void setWebRCWarn(const char* msg) {
    strncpy(webRCWarnMsg, msg, sizeof(webRCWarnMsg) - 1);
    webRCWarnMsg[sizeof(webRCWarnMsg) - 1] = '\0';
}

// ==================== Last request details used in the response ====================
// ==================== HTTP request handling ====================

void handleWebRCRequest() {
    if (!webRCServer.hasArg("plain")) {
        webRCServer.send(400, "application/json", "{\"e\":\"no data\"}");
        return;
    }
    String body = webRCServer.arg("plain");
    if (handleJSONProtocol(body)) {
        char resp[320];
        // Send and clear pending warnings in button and heartbeat responses.
        // Button responses (rt=2) give immediate feedback such as an arming refusal.
        // Heartbeat responses (rt=4) deliver system warnings such as low-battery disarming about every 2 seconds.
        // Stick responses (rt=1) leave warnings pending so the button release can report them.
        bool deliverWarn = (lastProcType == 2 || lastProcType == 4) && webRCWarnMsg[0];
        if (deliverWarn) {
            if (lastProcButtonIdx >= 0) {
                snprintf(resp, sizeof(resp),
                    "{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"bi\":%d,\"bs\":%d,\"warn\":\"%s\"}",
                    mode, (int)armed, lastProcType, lastProcButtonIdx, lastProcButtonState, webRCWarnMsg);
            } else {
                snprintf(resp, sizeof(resp),
                    "{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"warn\":\"%s\"}",
                    mode, (int)armed, lastProcType, webRCWarnMsg);
            }
            webRCWarnMsg[0] = '\0'; // Clear after sending.
        } else {
            if (lastProcButtonIdx >= 0) {
                snprintf(resp, sizeof(resp),
                    "{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"bi\":%d,\"bs\":%d}",
                    mode, (int)armed, lastProcType, lastProcButtonIdx, lastProcButtonState);
            } else {
                snprintf(resp, sizeof(resp),
                    "{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d}",
                    mode, (int)armed, lastProcType);
            }
        }
        webRCServer.send(200, "application/json", resp);
    } else {
        webRCServer.send(400, "application/json", "{\"e\":\"parse failed\"}");
    }
}

// ==================== Connection status ====================

bool isWebRCEnabled() {
    return webRCUpdated && (millis() - webRCLastUpdate < WEB_RC_TIMEOUT_MS);
}

bool isUsingWebRC() {
    return useWebRC && isWebRCEnabled();
}

// ------ Redirect port 8080 to 80 with HTTP 301 for addresses printed on older boards. ------------
// Redirect the old remote port using HTTP 301. Parse HTTP directly from the WiFiServer TCP connection.
// Omit this on C3 so a 200 ms polling wait cannot starve Wi-Fi on its single core.
#ifndef CONFIG_IDF_TARGET_ESP32C3
static void handleRedirect8080() {
    if (!redirectServer8080) return;
    WiFiClient client = redirectServer8080->accept();
    if (!client) return;

    String path = "/";
    String host = "";
    unsigned long t0 = millis();
    while (!client.available() && millis() - t0 < 200) {}

    if (client.available()) {
        // Parse the request line, for example GET /path HTTP/1.1.
        String reqLine = client.readStringUntil('\n');
        int s1 = reqLine.indexOf(' ');
        int s2 = reqLine.indexOf(' ', s1 + 1);
        if (s1 >= 0 && s2 > s1) path = reqLine.substring(s1 + 1, s2);

        // Read the Host header and strip port 8080.
        while (client.available()) {
            String line = client.readStringUntil('\n');
            line.trim();
            if (line.length() == 0) break;
            if (line.startsWith("Host:") || line.startsWith("host:")) {
                host = line.substring(5);
                host.trim();
                int colon = host.indexOf(':');
                if (colon >= 0) host = host.substring(0, colon);
            }
        }
    }

    client.print("HTTP/1.1 301 Moved Permanently\r\nLocation: http://");
    client.print(host);
    client.print(path);
    client.print("\r\nConnection: close\r\nContent-Length: 0\r\n\r\n");
    client.stop();
}
#endif
// ------ Redirect port 8080 to 80 with HTTP 301 for addresses printed on older boards. ------------

// ==================== Server setup ====================

void setupWebRC() {
    print("Setup WEB RC\n");
    lastValidThrottle = THROTTLE_MIN;
    lastValidRoll = lastValidPitch = lastValidYaw = 0.0f;

    webRCServer.on("/", HTTP_GET, []() {
        webRCServer.send(200, "text/html", webRCIndexHtml);
    });
    webRCServer.on("/web_rc",           HTTP_POST, handleWebRCRequest);
    webRCServer.on("/web_rc/heartbeat", HTTP_POST, handleWebRCRequest);

    webRCServer.on("/console", HTTP_GET, []() {
        int since = -1;
        int limit = 20;
        if (webRCServer.hasArg("since")) since = webRCServer.arg("since").toInt();
        if (webRCServer.hasArg("limit")) limit = webRCServer.arg("limit").toInt();
        if (limit <= 0) limit = 20;
        if (limit > CONSOLE_LINES) limit = CONSOLE_LINES;

        int availableFrom = max(0, consoleTotal - consoleFilled);
        int sendFrom = (since >= 0) ? since : availableFrom;
        if (sendFrom < availableFrom) sendFrom = availableFrom;
        if (sendFrom > consoleTotal) sendFrom = consoleTotal;
        int sendTo = min(consoleTotal, sendFrom + limit);

        String json = "{\"total\":";
        json += consoleTotal;
        json += ",\"next\":";
        json += sendTo;
        json += ",\"has_more\":";
        json += (sendTo < consoleTotal) ? "true" : "false";
        json += ",\"lines\":[";

        bool first = true;
        for (int i = sendFrom; i < sendTo; i++) {
            int idx = i % CONSOLE_LINES;
            if (!first) json += ",";
            first = false;
            json += "\"";
            String line = consoleBuf[idx];
            line.replace("\\", "\\\\");  // \ → \\
            line.replace("\"", "\\\"");  // " → \"
            line.replace("\n", "\\n");    // Escape any remaining newline.
            line.replace("\r", "\\r");    // Escape any remaining carriage return.
            json += line + "\"";
        }
        json += "]}";
        webRCServer.send(200, "application/json", json);
    });

    webRCServer.on("/console/cmd", HTTP_POST, []() {
        String cmd = webRCServer.arg("plain");
        cmd.trim();
        if (cmd.length() == 0) {
            webRCServer.send(400, "application/json", "{\"ok\":0,\"e\":\"empty command\"}");
            return;
        }

        char buf[CONSOLE_LINE_LEN];
        snprintf(buf, sizeof(buf), "> %s", cmd.c_str());
        webLog(buf);

        if (!enqueueConsoleCmd(cmd.c_str())) {
            webLog("! command queue is full");
            webRCServer.send(503, "application/json", "{\"ok\":0,\"e\":\"queue full\"}");
            return;
        }

        webRCServer.send(200, "application/json", "{\"ok\":1,\"queued\":1}");
    });

    webRCServer.on("/console/enable", HTTP_POST, []() {
        webConsoleEnabled = true;
        webRCServer.send(200, "application/json", "{\"ok\":1}");
    });

    webRCServer.on("/console/disable", HTTP_POST, []() {
        webConsoleEnabled = false;
        webRCServer.send(200, "application/json", "{\"ok\":1}");
    });

    webRCServer.on("/web_rc/status", HTTP_GET, []() {
        float vbat = readBatteryVoltage();
        if (isnan(vbat) || vbat < 0.0f) vbat = 0.0f;
        char json[384];
        snprintf(json, sizeof(json),
            "{\"enabled\":%s,\"active\":%s,"
            "\"voltage\":%.2f,"
            "\"throttle\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
            isWebRCEnabled() ? "true" : "false",
            isUsingWebRC()   ? "true" : "false",
            vbat,
            webRCThrottle, webRCRoll, webRCPitch, webRCYaw);
        webRCServer.send(200, "application/json", json);
    });

    webRCServer.begin();

    // ------ Redirect port 8080 to 80 with HTTP 301 for addresses printed on older boards. ------------
#ifndef CONFIG_IDF_TARGET_ESP32C3
    redirectServer8080 = new WiFiServer(8080); // Allocate on the heap to save static RAM.
    redirectServer8080->begin(); // Use a small redirect server for the port printed on older boards.
#endif
    // ------ Redirect port 8080 to 80 with HTTP 301 for addresses printed on older boards. ------------

    print("Web remote ready at http://192.168.4.1\n");
    print("  Dead zones: stick=%.0f%% throttle=%.0f%% | Scale: stick=%.2f yaw=%.2f\n",
          stickDeadzone * 100.0f, throttleDeadzone * 100.0f,
          webRCStickScale, webRCYawScale);
}

// ==================== Main loop ====================

void readWebRC() {
    webRCServer.handleClient();

    // ------ Redirect port 8080 to 80 with HTTP 301 for addresses printed on older boards. ------------
#ifndef CONFIG_IDF_TARGET_ESP32C3
    handleRedirect8080(); // Handle redirects for older boards.
#endif
    // ------ Redirect port 8080 to 80 with HTTP 301 for addresses printed on older boards. ------------
    
    if (isWebRCEnabled()) {
        webRCEnabled = useWebRC = true;
    } else {
        webRCEnabled = useWebRC = false;
    }
}

#else
void setupWebRC() { print("Web remote disabled\n"); }
void readWebRC()  {}
void processConsoleCommandQueue() {}
#endif
