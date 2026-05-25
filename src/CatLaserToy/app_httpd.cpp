#include "esp_http_server.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include <ESP32Servo.h>
#include <DHT.h>


// Referinte din .ino
extern Servo servoPan;
extern Servo servoTilt;
extern int   panAngle;
extern int   tiltAngle;
extern DHT   dht;

// DHT cache (non-blocking)
volatile float cachedTemp = 0.0;
volatile float cachedHum  = 0.0;
volatile bool  cachedOk   = false;

#define LASER_PIN              13
#define PAN_MIN                 0
#define PAN_MAX               180
#define TILT_MIN                0
#define TILT_MAX              150

// Smooth movement
// Mai mare = mai LENT (mai smooth). Mai mic = mai RAPID.
// 50 ms = 20 grade/secunda, 30 ms = 33 grade/secunda, 80 ms = 12 grade/secunda
#define SERVO_SMOOTH_DELAY     50
#define PAN_HOME               90
#define TILT_HOME              20

// Stare miscare continua
volatile int  panDir         = 0;   // -1 stanga, 0 stop, +1 dreapta
volatile int  tiltDir        = 0;   // -1 jos, 0 stop, +1 sus
volatile bool returningHome  = false;

// Stream MJPEG
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb  = NULL;
  esp_err_t    res = ESP_OK;
  char         part_buf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { Serial.println("Frame fail"); res = ESP_FAIL; break; }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return res;
}

// Servo smooth task
static void servoSmoothTask(void *param) {
  while (true) {
    if (returningHome) {
      // Smooth return to center
      bool moved = false;
      if (panAngle > PAN_HOME)       { panAngle--; moved = true; }
      else if (panAngle < PAN_HOME)  { panAngle++; moved = true; }
      if (tiltAngle > TILT_HOME)     { tiltAngle--; moved = true; }
      else if (tiltAngle < TILT_HOME){ tiltAngle++; moved = true; }

      servoPan.write(panAngle);
      servoTilt.write(tiltAngle);

      if (!moved) returningHome = false;
    } else {
      // Continuous movement while button is held
      if (panDir != 0) {
        int newPan = panAngle + panDir;
        if (newPan >= PAN_MIN && newPan <= PAN_MAX) {
          panAngle = newPan;
          servoPan.write(panAngle);
        }
      }
      if (tiltDir != 0) {
        int newTilt = tiltAngle + tiltDir;
        if (newTilt >= TILT_MIN && newTilt <= TILT_MAX) {
          tiltAngle = newTilt;
          servoTilt.write(tiltAngle);
        }
      }
    }
    vTaskDelay(SERVO_SMOOTH_DELAY / portTICK_PERIOD_MS);
  }
}

// DHT task (non-blocking)
static void dhtTask(void *param) {
  while (true) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      cachedTemp = t;
      cachedHum  = h;
      cachedOk   = true;
    }
    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}

// Move handler 
// GET /move?dir=left|right|up|down|stop|center
static esp_err_t move_handler(httpd_req_t *req) {
  char buf[60];
  int  buf_len = httpd_req_get_url_query_len(req) + 1;

  if (buf_len > 1 && buf_len < 60) {
    httpd_req_get_url_query_str(req, buf, buf_len);
    char dir[10];
    if (httpd_query_key_value(buf, "dir", dir, sizeof(dir)) == ESP_OK) {
      if      (strcmp(dir, "left")   == 0) { panDir  =  1; tiltDir = 0; returningHome = false; }
      else if (strcmp(dir, "right")  == 0) { panDir  = -1; tiltDir = 0; returningHome = false; }
      else if (strcmp(dir, "up")     == 0) { tiltDir = -1; panDir  = 0; returningHome = false; }
      else if (strcmp(dir, "down")   == 0) { tiltDir =  1; panDir  = 0; returningHome = false; }
      else if (strcmp(dir, "stop")   == 0) { panDir  =  0; tiltDir = 0; }
      else if (strcmp(dir, "center") == 0) {
        panDir = 0; tiltDir = 0; returningHome = true;
      }
      // dir=poll - nu face nimic, doar returneaza pozitia curenta
      Serial.printf("dir=%s pan=%d tilt=%d\n", dir, panAngle, tiltAngle);
    }
  }

  char json[60];
  snprintf(json, sizeof(json), "{\"pan\":%d,\"tilt\":%d}", panAngle, tiltAngle);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

// Laser handler
// GET /laser?state=on   sau   GET /laser?state=off
static esp_err_t laser_handler(httpd_req_t *req) {
  char buf[40];
  int  buf_len = httpd_req_get_url_query_len(req) + 1;

  if (buf_len > 1 && buf_len < 40) {
    httpd_req_get_url_query_str(req, buf, buf_len);
    char state[6];
    if (httpd_query_key_value(buf, "state", state, sizeof(state)) == ESP_OK) {
      if (strcmp(state, "on") == 0) {
        digitalWrite(LASER_PIN, HIGH);
        Serial.println("Laser ON");
      } else {
        digitalWrite(LASER_PIN, LOW);
        Serial.println("Laser OFF");
      }
    }
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, "{\"ok\":true}");
  return ESP_OK;
}

// Climate handler (DHT11)
// GET /climate - returneaza valorile din cache (non-blocking)
static esp_err_t climate_handler(httpd_req_t *req) {
  char json[80];
  if (!cachedOk) {
    snprintf(json, sizeof(json), "{\"ok\":false,\"temp\":null,\"hum\":null}");
  } else {
    snprintf(json, sizeof(json), "{\"ok\":true,\"temp\":%.1f,\"hum\":%.1f}", cachedTemp, cachedHum);
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

// HTML
static const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Laser Cat Toy</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #104d78; color: #eee;
    font-family: sans-serif; display: flex;
    flex-direction: column; align-items: center;
    padding: 16px; gap: 16px;
  }
  h1 { font-size: 1.2rem; color: #a8d8ea; letter-spacing: 2px; }
  #stream-container {
    width: 100%; max-width: 640px;
    border: 2px solid #a8d8ea; border-radius: 8px; overflow: hidden;
  }
  #stream { width: 100%; display: block; transform: rotate(180deg); }
  #controls {
    display: grid;
    grid-template-columns: repeat(3, 64px);
    grid-template-rows:    repeat(3, 64px);
    gap: 8px;
  }
  .btn {
    background: #16213e; border: 2px solid #a8d8ea;
    color: #a8d8ea; border-radius: 8px;
    font-size: 1.8rem; cursor: pointer;
    display: flex; align-items: center; justify-content: center;
    user-select: none; transition: background 0.1s;
    -webkit-tap-highlight-color: transparent;
  }
  .btn:active { background: #a8d8ea; color: #1a1a2e; }
  .btn-center { font-size: 1rem; }
  #laser-row {
    display: flex; align-items: center; gap: 16px;
  }
  #btn-laser {
    padding: 12px 32px;
    background: #16213e; border: 2px solid #ff4444;
    color: #ff4444; border-radius: 8px;
    font-size: 1rem; cursor: pointer;
    transition: background 0.1s, color 0.1s;
    user-select: none;
  }
  #btn-laser.on {
    background: #ff4444; color: #fff;
  }
  #status { font-size: 0.85rem; color: #a8d8ea; opacity: 0.7; }
  #climate {
    display: flex; gap: 24px;
    font-size: 0.95rem; color: #a8d8ea;
    background: #16213e; border: 2px solid #a8d8ea;
    border-radius: 8px; padding: 10px 24px;
  }
  #climate span { font-weight: bold; color: #fff; }
</style>
</head>
<body>

<h1>Automatic Laser Toy for Cats</h1>

<div id="stream-container">
  <img id="stream" src="" alt="stream">
</div>

<div id="controls">
  <div></div>
  <button class="btn" data-dir="up">&#8679;</button>
  <div></div>

  <button class="btn" data-dir="left">&#8678;</button>
  <button class="btn btn-center" data-dir="center">&#9635;</button>
  <button class="btn" data-dir="right">&#8680;</button>

  <div></div>
  <button class="btn" data-dir="down">&#8681;</button>
  <div></div>
</div>

<div id="laser-row">
  <button id="btn-laser" onclick="toggleLaser()">LASER OFF</button>
</div>

<div id="climate">
  <div>Temp: <span id="temp">--</span> &deg;C</div>
  <div>Hum: <span id="hum">--</span> %</div>
</div>

<div id="status">Pan: 90&deg; &nbsp;|&nbsp; Tilt: 20&deg;</div>

<script>
document.getElementById('stream').src =
    'http://' + location.hostname + ':81/stream';

// Miscare smooth - tine apasat
function move(dir) {
  fetch('/move?dir=' + dir)
    .then(r => r.json())
    .then(d => {
      document.getElementById('status').textContent =
        'Pan: ' + d.pan + '\u00b0  |  Tilt: ' + d.tilt + '\u00b0';
    })
    .catch(() => {});
}

// Polling pentru status update in timpul miscarii continue
let statusInterval = null;
function startStatusPolling() {
  if (statusInterval) return;
  statusInterval = setInterval(() => {
    fetch('/move?dir=poll')
      .then(r => r.json())
      .then(d => {
        document.getElementById('status').textContent =
          'Pan: ' + d.pan + '\u00b0  |  Tilt: ' + d.tilt + '\u00b0';
      })
      .catch(() => {});
  }, 200);
}
function stopStatusPolling() {
  if (statusInterval) {
    clearInterval(statusInterval);
    statusInterval = null;
  }
}

document.querySelectorAll('.btn').forEach(btn => {
  const dir = btn.dataset.dir;

  if (dir === 'center') {
    // Centru = un singur click pentru revenire smooth
    btn.addEventListener('click', () => {
      move('center');
      startStatusPolling();
      setTimeout(stopStatusPolling, 5000);
    });
  } else {
    // Sageti = tine apasat pentru miscare continua
    btn.addEventListener('mousedown',  () => { move(dir); startStatusPolling(); });
    btn.addEventListener('mouseup',    () => { move('stop'); stopStatusPolling(); });
    btn.addEventListener('mouseleave', () => { move('stop'); stopStatusPolling(); });
    btn.addEventListener('touchstart', e => { e.preventDefault(); move(dir); startStatusPolling(); });
    btn.addEventListener('touchend',   () => { move('stop'); stopStatusPolling(); });
  }
});

// Tastatura - tine apasat pentru miscare continua
const keyMap = {
  ArrowLeft: 'left', ArrowRight: 'right',
  ArrowUp:   'up',   ArrowDown:  'down'
};
document.addEventListener('keydown', e => {
  if (keyMap[e.key] && !e.repeat) {
    e.preventDefault();
    move(keyMap[e.key]);
    startStatusPolling();
  }
  if (e.key === ' ' && !e.repeat) {
    e.preventDefault();
    move('center');
    startStatusPolling();
    setTimeout(stopStatusPolling, 5000);
  }
});
document.addEventListener('keyup', e => {
  if (keyMap[e.key]) {
    move('stop');
    stopStatusPolling();
  }
});

// Laser toggle 
let laserOn = false;

function toggleLaser() {
  laserOn = !laserOn;
  fetch('/laser?state=' + (laserOn ? 'on' : 'off'))
    .catch(() => { laserOn = !laserOn; });

  const btn = document.getElementById('btn-laser');
  if (laserOn) {
    btn.textContent = 'LASER ON';
    btn.classList.add('on');
  } else {
    btn.textContent = 'LASER OFF';
    btn.classList.remove('on');
  }
}

// Climate polling - la fiecare 10 secunde
function updateClimate() {
  fetch('/climate')
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        document.getElementById('temp').textContent = d.temp.toFixed(1);
        document.getElementById('hum').textContent  = d.hum.toFixed(1);
      }
    })
    .catch(() => {});
}
updateClimate();
setInterval(updateClimate, 60000);
</script>

</body>
</html>
)rawhtml";

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_sendstr(req, HTML_PAGE);
  return ESP_OK;
}

//Start server
void startCameraServer() {
  httpd_handle_t stream_httpd = NULL;
  httpd_handle_t camera_httpd = NULL;

  httpd_config_t stream_cfg = HTTPD_DEFAULT_CONFIG();
  stream_cfg.server_port    = 81;
  stream_cfg.ctrl_port      = 32769;

  if (httpd_start(&stream_httpd, &stream_cfg) == ESP_OK) {
    httpd_uri_t stream_uri = {
      .uri     = "/stream",
      .method  = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("Stream server pornit pe :81");
  }

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port    = 80;

  if (httpd_start(&camera_httpd, &cfg) == ESP_OK) {
    httpd_uri_t index_uri = {
      .uri     = "/",
      .method  = HTTP_GET,
      .handler = index_handler,
      .user_ctx = NULL
    };
    httpd_uri_t move_uri = {
      .uri     = "/move",
      .method  = HTTP_GET,
      .handler = move_handler,
      .user_ctx = NULL
    };
    httpd_uri_t laser_uri = {
      .uri     = "/laser",
      .method  = HTTP_GET,
      .handler = laser_handler,
      .user_ctx = NULL
    };
    httpd_uri_t climate_uri = {
      .uri     = "/climate",
      .method  = HTTP_GET,
      .handler = climate_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &move_uri);
    httpd_register_uri_handler(camera_httpd, &laser_uri);
    httpd_register_uri_handler(camera_httpd, &climate_uri);
    Serial.println("Camera server pornit pe :80");
  }

  // Start servo smooth task
  xTaskCreate(servoSmoothTask, "servoSmooth", 2048, NULL, 1, NULL);
  Serial.println("Servo smooth task started");

  // Start DHT task (non-blocking sensor read every 5s)
  xTaskCreate(dhtTask, "dhtTask", 2048, NULL, 1, NULL);
  Serial.println("DHT task started");
}
