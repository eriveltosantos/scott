/*******************************************************************************
* Scott Robot - Kit Robo Explorer - Joystick + Linha + Colisão + Exploração + Waypoints GPS + Odometria
* Controle o seu Rocket Tank pelo celular ou ative os modos autônomos.
*******************************************************************************/

// --------------------------------------------------
// Bibliotecas

#include <esp_arduino_version.h>  

#include <WiFi.h>
#include <AsyncTCP.h>           
#include <ESPAsyncWebServer.h>  
#include <ArduinoJson.h>        
#include <Preferences.h>
#include <RoboCore_Vespa.h>
#include <TinyGPS.h>            

// --------------------------------------------------
// Variaveis

// web server assincrono na porta 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// LED e Sensores Ultrassônicos
const uint8_t PINO_LED =  15;
const uint8_t PINO_HCSR04_ECHO = 26;
const uint8_t PINO_HCSR04_TRIGGER = 25;

// Configurações do GPS (UART2)
#define PINO_GPS_RX 16
#define PINO_GPS_TX 17
HardwareSerial SerialGPS(2);
TinyGPS gps;
uint32_t timeout_gps = 0; 

// Waypoints GPS Alvo
bool modo_waypoints = false;
float target_lat = 0.0;
float target_lon = 0.0;

// --- MÁQUINA DE ESTADOS WAYPOINT ---
enum EstadoWaypoint { WAY_LIVRE, WAY_RE, WAY_GIRO };
EstadoWaypoint estadoWay = WAY_LIVRE;
unsigned long posicao_inicial_way = 0;
uint32_t alvoPulsosWay = 0;
uint32_t tempo_inicio_manobra_way = 0;
int sentido_giro_way = 1; // 1 para direita, -1 para esquerda

// --- VARIÁVEIS DE ODOMETRIA E FUSÃO (Dead Reckoning) ---
const float CM_POR_PULSO = 30.0 / 100.0;
const float LARGURA_ESTEIRA_EFETIVA = (260.0 * 0.3 * 2.0) / PI;

float estimativa_lat = 0.0;
float estimativa_lon = 0.0;
float theta_rad = 0.0; 

unsigned long odo_esq_anterior = 0;
unsigned long odo_dir_anterior = 0;
bool odometria_inicializada = false;
bool heading_gps_valido = false; 

// Variáveis para a Calibração Geométrica do GPS (Ignora o vetor cego)
float lat_inicio_calibracao = 0.0;
float lon_inicio_calibracao = 0.0;
unsigned long pulsos_inicio_calibracao = 0; // NOSSA NOVA RÉGUA FÍSICA
bool registrou_inicio_calibracao = false;

int sinal_esq = 0; 
int sinal_dir = 0; 

// -------------------------------------------------------------------
// ALVOS DE PULSOS PARA MANOBRAS (Disco de 10 dentes)
// -------------------------------------------------------------------
const uint32_t PULSOS_30_CM = 100;  
const uint32_t PULSOS_20_CM = 67;  
const uint32_t PULSOS_15_CM = 50;  
const uint32_t PULSOS_180_GRAUS = 260; 
const uint32_t PULSOS_90_GRAUS = 130;  
const uint32_t PULSOS_45_GRAUS = 65;   

// -------------------------------------------------------------------
// MAPEAMENTO E VARIÁVEIS DOS ENCODERS
// -------------------------------------------------------------------
const int PINO_ENC_ESQ_A = 21; 
const int PINO_ENC_ESQ_B = 22; 
const int PINO_ENC_DIR_A = 19; 
const int PINO_ENC_DIR_B = 23; 

volatile unsigned long contador_esq_A = 0;
volatile unsigned long contador_esq_B = 0;
volatile unsigned long contador_dir_A = 0;
volatile unsigned long contador_dir_B = 0;

const int NUMERO_CONTADORES = 2;
const int NUMERO_LEITURAS = 2;
const int NUMERO_DENTES = 10; 
unsigned long tempo_antes_encoder = 0;
const long INTERVALO_CALCULO = 1000; 
float velocidade_rpm_esq = 0;
float velocidade_rpm_dir = 0;
float robot_speed_cms = 0;

// JSON aliases
const char *ALIAS_ANGULO = "angulo";
const char *ALIAS_DISTANCIA = "distancia";
const char *ALIAS_VELOCIDADE = "velocidade";
const char *ALIAS_VBAT = "vbat";
const char *ALIAS_LED = "led";
const char *ALIAS_IP = "ip"; 

const char *ALIAS_WIFI_SSID = "wifi_ssid";
const char *ALIAS_WIFI_PASS = "wifi_pass";

const char *ALIAS_ESPERA = "pausa";
const char *ALIAS_KP = "kp";
const char *ALIAS_KI = "ki";
const char *ALIAS_KD = "kd";
const char *ALIAS_PARA = "stop";
const char *ALIAS_LINHA = "linha";
const char *ALIAS_COLISAO = "colisao";
const char *ALIAS_EXPLORA = "explora";
const char *ALIAS_WAYPOINT = "waypoint";

// Wi-Fi
bool change_wifi_flag = false;
String new_wifi_ssid = "";
String new_wifi_pass = "";

VespaMotors motores;
VespaBattery vbat;
const uint32_t TEMPO_ATUALIZACAO_VBAT = 5000; 
uint32_t timeout_vbat;

const uint32_t TEMPO_ATUALIZACAO_DISTANCIA = 60;  
uint32_t timeout_distancia;
uint32_t timeout_distancia_ws = 0; 
uint32_t distancia = 100; 

const int SENSOR_LINHA_ESQUERDO = 36; 
const int SENSOR_LINHA_DIREITO = 39;  

int leitura_esquerdo = 0;
int leitura_direito = 0;

int limiarLinha = 3000;
bool linha_escura = true; 
const int LIMIAR_QUEDA = 3850; 

const int VELOCIDADE = 75;
const int VELOCIDADE_EXPLORACAO = 90; 
const int VELOCIDADE_GIRO = 75; 
const int VELOCIDADE_MAXIMA = 100;
const int VELOCIDADE_MINIMA = -100; 
int velocidade_direita = 0;
int velocidade_esquerda = 0;

const int DISTANCIA_OBSTACULO = 20;
const int DISTANCIA_EXPLORACAO = 20; 

const int CONTAGEM_MAXIMA = 150;
int contador_parada = 0;

float espera, Kp = 6, Ki = 0.2, Kd = 20, erro = 0.0, P = 0.0, I = 0.0, D = 0.0, erro_anterior = 0.0, resposta_PID = 0.0;
bool parada = true;

bool modo_linha = false;
bool modo_colisao = false;
bool modo_explora = false;

int contadorColisoes = 0;
uint32_t tempoUltimaColisao = 0;

enum EstadoManobra { LIVRE, MANOBRA_QUEDA_RE, MANOBRA_QUEDA_GIRO, MANOBRA_PAREDE_RE, MANOBRA_PAREDE_GIRO };
EstadoManobra estadoManobraAtual = LIVRE;
unsigned long posicao_inicial_manobra = 0;
uint32_t alvoPulsosManobra = 0;

// Máquina de estados para Exploração Autônoma
enum EstadoExploracao { EXPLORA_LIVRE, EXPLORA_RE, EXPLORA_GIRO, EXPLORA_PARADO };
EstadoExploracao estadoExplora = EXPLORA_LIVRE;
int giros_consecutivos = 0; 
unsigned long posicao_inicial_explora = 0;
uint32_t alvoPulsosExplora = 0; 
uint32_t tempo_inicio_manobra_explora = 0; 
const uint32_t TIMEOUT_MAX_EXPLORA = 6000;  

bool modoSegurancaBateria = false;
const uint32_t TENSAO_CRITICA = 6400;

Preferences SPIFFS;
const char* DIRETORIO_SPIFFS = "seguidor";
const char* ENDERECOS_SPIFFS[4] = {"espera", "Kp", "Ki", "Kd"};
const float VALORES_PADROES[4] = {10.0, 6.0, 0.2, 20};

// --------------------------------------------------
// Pagina web principal

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Scott - Dashboard</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, minimum-scale=1.0, maximum-scale=1.0, user-scalable=0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" onerror="this.onerror=null;this.disabled=true;" />
    <style>
        html, body {width: 100%; height: 100%; padding: 0; margin: 0; overflow: hidden; background-color: #f7f7f7;}
        .container { height: 26px; width: 50px; position: relative; }
        .container * { position: absolute; }
        .battery { top: 50%; left: 50%; transform: translate(-50%, -50%); height: 20px; width: 40px; border: 2px solid #F1F1F1; border-radius: 5px; padding: 1px; }
        .battery::before { content: ''; position: absolute; height: 13px; width: 3px; background: #F1F1F1; left: 44px; top: 50%; transform: translateY(-50%); border-radius: 0 3px 3px 0; }
        .part { background: #0F0; top: 1px; left: 1px; bottom: 1px; border-radius: 3px; }
        .grid-3x3 { border-collapse: collapse; margin: 0 auto; background: #fff; width: 110%; max-width: 400px; }
        .grid-3x3 td { padding: 8px; border: 2px solid #ECE5E5; text-align: center; }

        #btn-led-linha, #btn-led-colisao, #btn-led-explora {
            padding: 16px 20px;
            font-size: 16px;
            font-family: inherit;
            background-color: #444;
            color: #fff;
            border: 3px solid #666;
            border-radius: 10px;
            cursor: pointer;
            user-select: none;
            transition: background-color 0.15s;
            -webkit-tap-highlight-color: transparent;
            outline: none;
            width: 100px;
        }
        #btn-led-linha.on, #btn-led-colisao.on, #btn-led-explora.on { background-color: #f0be00; border-color: #c49a00; color: #000; font-weight:bold;}
        #btn-led-linha:disabled, #btn-led-colisao:disabled, #btn-led-explora:disabled { background-color: #eee; border-color: #ccc; color: #999; cursor: not-allowed; }

        .menu { display: flex; background-color: #222; border-bottom: 2px solid #444; height: 40px; }
        .menu-item { flex: 1; text-align: center; line-height: 40px; color: #ccc; font-size: 16px; cursor: pointer; user-select: none; transition: background-color 0.15s, color 0.15s; border-bottom: 3px solid transparent; }
        .menu-item:hover { background-color: #333; }
        .menu-item.active { color: #f0be00; border-bottom-color: #f0be00; background-color: #333; }
        .tab-content { display: none; height: calc(100% - 76px); overflow-y: auto; }
        .tab-content.active { display: block; }
        
        .input-group { display: table; width: 100%; margin-bottom: 20px; }
        .input-group-addon { width: 50px; padding: 6px 12px; font-size: 14px; text-align: center; background-color: #eee; border: 1px solid #ccc; border-radius: 4px; display: table-cell; border-right: 0; border-top-right-radius: 0px; border-bottom-right-radius: 0px;}
        .input { display: table-cell; font-size: 18px; width: calc(100% - 24px); height: 34px; padding: 6px 12px; border: 1px solid #ccc; }
        .disabled { background-color: #eee; }
        .btn { display: inline-block; font-size: 18px; text-align: center; cursor: pointer; border: 1px solid transparent; padding: 6px 12px; border-radius: 4px; color: #fff; width: 100%; height: 48px; margin-bottom: 10px;}
        .btn-warning { background-color: #f0be00; border-color: #f0be00; color: #000; font-weight: bold; }
        .setup-title { font-size: 20px; font-weight: bold; color: #333; width: 100%; text-align: center; margin-bottom: 15px; display: block; }   
        
        #map { height: 500px; width: 100%; border-radius: 8px; }
        .setup-container { display: flex; align-items: flex-start; justify-content: space-evenly; align-content: flex-start; flex-direction: row; flex-wrap: wrap; padding: 20px; }
        .setup-box { width: 100%; max-width: 320px; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); margin-bottom: 20px; }
    </style>
</head>
<body style="font-family: 'Gill Sans', 'Gill Sans MT', Calibri, 'Trebuchet MS', sans-serif ;">
    <div style="line-height: 26px; background-color: black; padding: 10px; padding-bottom: 0px;">
        <div class="container" style="float: right; margin-right: 10px;">
            <div class="battery"><div id="lbat" class="part"></div></div>
        </div>
        <div style="float: right; color: white; font-size: 18px; line-height: 26px; margin-right: 5px;"><span id="vbat">0</span> V</div>
        <div style="width: 100%; text-align: center;">
            <span style="color: #f0be00; font-weight: bold; font-size: 16px;">SCOTT ROBOT - WAYPOINTS GPS</span>
        </div>
    </div>
    <div class="menu">
        <div class="menu-item active" id="menu-display" onclick="showTab('display')">Display</div>
        <div class="menu-item" id="menu-map" onclick="showTab('map')">Map</div>
        <div class="menu-item" id="menu-waypoint" onclick="showTab('waypoint')">Waypoint</div>
        <div class="menu-item" id="menu-setup" onclick="showTab('setup')">Setup</div>
    </div>
    <div style="color:rgb(128, 128, 128); font-size: medium; text-align: left; width: 300px; position: absolute; top: 0px; left: 0px; visibility: hidden;">DEBUG: Vel: <span id="speed">0</span>%</div>
    
    <div id="tab-display" class="tab-content active">
        <div style="display: table; width:100%; height: 100%;">
            <div style="display: table-cell; vertical-align: middle;">
                <div style="display: flex; align-items: center; justify-content: space-evenly; align-content: center; flex-direction: row; flex-wrap: wrap;">
                    <div style="text-align: center; margin-bottom: 20px;">
                        <table id="radar" class="grid-3x3" style="font-size: 14px;">
                            <tr><td><b>Date</b></td><td><b>Time</b></td><td><b>Lat</b></td><td><b>Lon</b></td></tr>
                            <tr><td><span id="gps-date">--/--/----</span></td><td><span id="gps-time">--:--:--</span></td><td><span id="gps-lat">--</span></td><td><span id="gps-lon">--</span></td></tr>
                            <tr><td><b>Dist:</b> <span id="distance">--</span> cm</td><td><b>Power:</b> <span id="table-speed">0</span> %</td><td colspan="2"><b>Speed:</b> <span id="robot-speed">0.0</span> cm/s</td></tr>
                        </table>
                    </div>
                    <canvas id="canvas_joystick"></canvas>
                    <div style="display: flex; width: 100%; justify-content: space-around; align-items: center; margin-top: 15px;">
                        <div style="text-align: center;">
                            <button id="btn-led-linha" onclick="toggleLed('linha')">Line</button>
                            <div style="color: gray; margin-top: 5px; font-size: 10px;" id="led-status-linha">OFF</div>
                        </div>
                        <div style="text-align: center;">
                            <button id="btn-led-explora" onclick="toggleLed('explora')">Explore</button>
                            <div style="color: gray; margin-top: 5px; font-size: 10px;" id="led-status-explora">OFF</div>
                        </div>
                        <div style="text-align: center;">
                            <button id="btn-led-colisao" onclick="toggleLed('colisao')">Collision</button>
                            <div style="color: gray; margin-top: 5px; font-size: 10px;" id="led-status-colisao">OFF</div>
                        </div>
                    </div>
                    <div id="fall-message-container" style="width: 100%; text-align: center; margin-top: 15px;">
                        <span id="fall-status-text" style="font-size: 18px; font-weight: bold; color: #ccc; transition: color 0.2s;"></span><br>
                        <span id="stuck-status-text" style="font-size: 18px; font-weight: bold; color: #ccc; transition: color 0.2s;"></span>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <div id="tab-map" class="tab-content">
        <h3 style="color: #333; text-align: center; margin-top: 15px; font-weight: bold;">Real-Time GPS Position</h3>
        <div id="map-container" style="position: relative; width: 100%; height: 500px; background: #fff; border-radius: 8px; overflow: hidden; margin: 0 auto; max-width: 95%;">
            <div id="map" style="width: 100%; height: 100%; display: block;"></div>
        </div>
    </div>

    <div id="tab-waypoint" class="tab-content">
        <div class="setup-container">
            <div class="setup-box">
                <span class="setup-title">Navegação por Waypoints</span>
                <p style="font-size: 14px; color: gray; margin-top: 0;">Insira as coordenadas de destino (ao ar livre) para o Scott navegar autonomamente.</p>
                <div class="input-group"><div class="input-group-addon">Lat</div><input type="text" id="target_lat" placeholder="-22.85817" class="input"></div>
                <div class="input-group"><div class="input-group-addon">Lon</div><input type="text" id="target_lon" placeholder="-46.97656" class="input"></div>
                <button type="button" class="btn" onclick="usarLocalizacaoAtual()" style="background-color: #2196F3; color: white;">Usar Localização Atual</button>
                <button type="button" class="btn btn-warning" onclick="sendWaypoint()">Iniciar Waypoint</button>
                <button type="button" class="btn" onclick="stopWaypoint()" style="background-color: #f44336; color: white;">Parar Waypoint</button>
                <div id="waypoint_status" style="margin-top: 15px; font-weight: bold; text-align: center; color: #2196F3;">Status: Inativo</div>
            </div>
        </div>
    </div>

    <div id="tab-setup" class="tab-content">
        <div class="setup-container">
            <div class="setup-box">
                <span class="setup-title">PID Control</span>
                <div class="input-group"><div class="input-group-addon">Kp</div><input type="text" value="0000" id="kp" class="input disabled" disabled></div>
                <div class="input-group"><div class="input-group-addon">Ki</div><input type="text" value="0000" id="ki" class="input disabled" disabled></div>
                <div class="input-group"><div class="input-group-addon">Kd</div><input type="text" value="0000" id="kd" class="input disabled" disabled></div>
                <div class="input-group"><div class="input-group-addon">Pause</div><input type="text" value="0000" class="input disabled" id="pausa" disabled></div>
                <button type="button" id="change_pid" class="btn btn-warning" onclick="change_pid()">edit</button>
            </div>
            <div class="setup-box">
                <span class="setup-title">Line Calibration</span>
                <p style="font-size: 14px; color: gray; margin-top: 0;">Position the robot <b>WITH THE SENSORS OVER THE LINE</b> and start.</p>
                <button type="button" class="btn btn-warning" onclick="calibrarSensores()">Calibrate Sensors</button>
                <div id="statusCalibracao" style="margin-top: 10px; font-weight: bold; text-align:center; color: #4CAF50;"></div>
            </div>
            <div class="setup-box">
                <span class="setup-title">Connect to Wi-Fi</span>
                <p style="font-size: 14px; color: gray; margin-top: 0;">To access internet and maps.</p>
                <div class="input-group"><div class="input-group-addon">SSID</div><input type="text" id="wifi_ssid" placeholder="Network name" class="input"></div>
                <div class="input-group"><div class="input-group-addon">Pass</div><input type="password" id="wifi_pass" placeholder="Network password" class="input"></div>
                <button type="button" class="btn btn-warning" onclick="send_wifi()">Connect</button>
                <div id="wifi_ip_container" style="display: none; margin-top: 20px; font-size: 16px; text-align: center; border-top: 1px solid #eee; padding-top: 15px;">
                    <span style="color: #4CAF50; font-weight: bold;">Connected to your home!</span><br>
                    <a id="wifi_ip_link" href="#" target="_blank" style="display: inline-block; margin-top: 10px; color: #fff; background-color: #2196F3; padding: 10px 20px; border-radius: 8px; font-weight: bold; text-decoration: none; font-size: 18px;"></a><br>
                    <button type="button" id="btn-copy-ip" class="btn btn-warning" style="margin-top: 10px;" onclick="copiarIP()">Copy IP</button>
                </div>
            </div>
        </div>
    </div>
    
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" defer></script>
    <script>
        function safelySend(dataObj) { if (connection && connection.readyState === WebSocket.OPEN) { connection.send(JSON.stringify(dataObj)); } }
        function calibrarSensores() { document.getElementById('statusCalibracao').innerText = "Calibrating..."; document.getElementById('statusCalibracao').style.color = "#f0be00"; safelySend({ cmd: "calibrate_line" }); }

        function usarLocalizacaoAtual() {
            if (typeof lat !== 'undefined' && lat !== null) {
                document.getElementById("target_lat").value = lat.toFixed(6);
                document.getElementById("target_lon").value = lon.toFixed(6);
            } else {
                alert("Aguardando sinal válido de GPS do robô para preencher a localização.");
            }
        }

        function sendWaypoint() {
            var lat_target = parseFloat(document.getElementById("target_lat").value);
            var lon_target = parseFloat(document.getElementById("target_lon").value);
            if (!isNaN(lat_target) && !isNaN(lon_target)) {
                safelySend({ waypoint: true, target_lat: lat_target, target_lon: lon_target });
                document.getElementById("waypoint_status").innerText = "Status: Navegando para o Alvo...";
                document.getElementById("waypoint_status").style.color = "#4CAF50";
            } else {
                alert("Insira coordenadas válidas.");
            }
        }

        function stopWaypoint() {
            safelySend({ waypoint: false });
            document.getElementById("waypoint_status").innerText = "Status: Parado / Inativo";
            document.getElementById("waypoint_status").style.color = "#f44336";
        }

        let lat = -22.85817, lon = -46.97656, lastMapUpdate = 0, map = null, marcador = null;

        function checkAndInitMap() {
            if (typeof L === 'undefined') { document.getElementById('menu-map').style.display = 'none'; showTab('display'); return; }
            if (!map) {
                try {
                    document.getElementById('map').style.display = 'block';
                    map = L.map('map').setView([lat, lon], 15);
                    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '© OSM' }).addTo(map);
                    marcador = L.marker([lat, lon]).addTo(map);
                } catch(e) { document.getElementById('menu-map').style.display = 'none'; showTab('display'); }
            } else if (typeof map.invalidateSize === 'function') { setTimeout(function(){ map.invalidateSize(); }, 100); }
        }

        var connection = new WebSocket(`ws://${window.location.hostname}/ws`);
        connection.onopen = function () { console.log('Connection opened'); };
        connection.onmessage = function (e) {
            const data = JSON.parse(e.data);
            if (data["vbat"]){
                document.getElementById("vbat").innerText = (data["vbat"] / 1000).toFixed(1);
                var lbat = (data["vbat"] * 100 / 9000).toFixed(0);
                if(lbat > 100) lbat = 100; if(lbat < 2) lbat = 2;
                document.getElementById("lbat").style.width = lbat + '%';
                document.getElementById("lbat").style.backgroundColor = (lbat < 20) ? "#F00" : (lbat < 70) ? "orange" : "#0F0";
            }
            if (data["distancia"]) document.getElementById("distance").innerText = (data["distancia"]).toFixed(0);
            if (data["robot_speed"] !== undefined) document.getElementById("robot-speed").innerText = data["robot_speed"].toFixed(1);
            if (data["fall"] !== undefined) {
                let fallText = document.getElementById("fall-status-text");
                if (data["fall"]) { fallText.innerText = "Fall Detected"; fallText.style.color = "#FF0000"; } 
                else { fallText.innerText = ""; }
            }
            if (data["stuck"] !== undefined) {
                let stuckText = document.getElementById("stuck-status-text");
                if (data["stuck"]) { 
                    stuckText.innerText = "Caminho Bloqueado!"; 
                    stuckText.style.color = "#FFA500"; 
                } 
                else { 
                    stuckText.innerText = ""; 
                }
            }
            if (data["waypoint_status"]) {
                document.getElementById("waypoint_status").innerText = "Status: " + data["waypoint_status"];
                if (data["waypoint_status"] === "alvo_alcancado") {
                    document.getElementById("waypoint_status").style.color = "#4CAF50";
                }
            }
            if (data["lat"] !== undefined && data["lat"] !== null) {
                lat = data["lat"]; lon = data["lon"];
                document.getElementById("gps-lat").innerText = lat.toFixed(6);
                document.getElementById("gps-lon").innerText = lon.toFixed(6);
                let agora = Date.now();
                if (agora - lastMapUpdate >= 3000 && map && marcador) { marcador.setLatLng([lat, lon]); map.setView([lat, lon]); lastMapUpdate = agora; }
            }
            if (data["date"] !== undefined) document.getElementById("gps-date").innerText = data["date"];
            if (data["time"] !== undefined) document.getElementById("gps-time").innerText = data["time"];
            if (data["ip"]){
                ip_atual = data["ip"]; document.getElementById("wifi_ip_container").style.display = "block";
                document.getElementById("wifi_ip_link").innerText = data["ip"]; document.getElementById("wifi_ip_link").href = "http://" + data["ip"];
            }
            if (data["status"] === "calibrado") {
                let infoCor = data["escura"] ? " (Dark Line)" : " (Light Line)";
                document.getElementById('statusCalibracao').innerText = "Saved! Threshold: " + data["limiar"] + infoCor;
                document.getElementById('statusCalibracao').style.color = "#4CAF50";
            }
            if (data["alert"] === "LOW_BATTERY") { alert("⚠️ SAFETY ALERT: Battery at critical level! Motors disabled."); }
        };

        var ip_atual = "";
        function copiarIP() {
            if (!ip_atual) return;
            navigator.clipboard.writeText(ip_atual).then(() => {
                var btn = document.getElementById("btn-copy-ip"); let txt = btn.innerText; btn.innerText = "Copied!"; setTimeout(() => btn.innerText = txt, 1500);
            });
        }

        function send_wifi() {
            var ssid = document.getElementById("wifi_ssid").value, pass = document.getElementById("wifi_pass").value;
            if(ssid !== "") { safelySend({'wifi_ssid': ssid, 'wifi_pass': pass}); alert("Sending request to connect to '" + ssid + "'..."); } 
            else { alert("Please enter the network name (SSID)."); }
        }

        function change_pid() {
            if (document.getElementById("change_pid").innerHTML == "edit") {
                safelySend({'stop': 1});
                ["kp", "ki", "kd", "pausa"].forEach(id => { document.getElementById(id).removeAttribute("disabled"); document.getElementById(id).classList.remove("disabled"); });
                document.getElementById("change_pid").innerHTML = "save";
            } else {
                safelySend({'kp': parseFloat(document.getElementById("kp").value), 'ki': parseFloat(document.getElementById("ki").value), 'kd': parseFloat(document.getElementById("kd").value), 'pausa': parseFloat(document.getElementById("pausa").value)});
                ["kp", "ki", "kd", "pausa"].forEach(id => { document.getElementById(id).setAttribute("disabled", true); document.getElementById(id).classList.add("disabled"); });
                document.getElementById("change_pid").innerHTML = "edit";
            }
        }

        function showTab(tab) {
            document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
            document.querySelectorAll('.menu-item').forEach(el => el.classList.remove('active'));
            document.getElementById('tab-' + tab).classList.add('active');
            document.getElementById('menu-' + tab).classList.add('active');
            if (tab === 'display' && typeof resize === 'function') resize();
            if (tab === 'map') checkAndInitMap();
        }

        var led_state = { linha: false, colisao: false, explora: false };
        function toggleLed(tipo){
            led_state[tipo] = !led_state[tipo];
            if (led_state[tipo]) {
                Object.keys(led_state).forEach(k => {
                    if(k !== tipo) {
                        led_state[k] = false;
                        document.getElementById("btn-led-" + k).classList.remove("on");
                        document.getElementById("led-status-" + k).innerText = "OFF";
                    }
                });
            }
            var btn = document.getElementById("btn-led-" + tipo);
            var status = document.getElementById("led-status-" + tipo);
            if(led_state[tipo]){ btn.classList.add("on"); status.innerText = "ON"; } 
            else { btn.classList.remove("on"); status.innerText = "OFF"; }
            safelySend(led_state);
        }
    </script>
    <script>
        var canvas_joystick, ctx_joystick, width, height, radius, button_size;
        let origin_joystick = { x: 0, y: 0};
        const width_to_radius_ratio = 0.04, width_to_size_ratio = 0.15, radius_factor = 7;

        function initJoystick() {
            canvas_joystick = document.getElementById('canvas_joystick'); if (!canvas_joystick) return;
            ctx_joystick = canvas_joystick.getContext('2d'); resize();
            ['mousedown','touchstart'].forEach(evt => canvas_joystick.addEventListener(evt, startDrawing));
            ['mouseup','touchend','touchcancel'].forEach(evt => canvas_joystick.addEventListener(evt, stopDrawing));
            ['mousemove','touchmove'].forEach(evt => canvas_joystick.addEventListener(evt, Draw));
            window.addEventListener('resize', resize);
        }
        
        function updateMapVisibility() {
            var mapTabBtn = document.getElementById('menu-map');
            if (mapTabBtn) {
                if (navigator.onLine) { mapTabBtn.style.display = ''; } 
                else { mapTabBtn.style.display = 'none'; if (document.getElementById('tab-map').classList.contains('active')) showTab('display'); }
            }
        }
        window.addEventListener('online', updateMapVisibility); window.addEventListener('offline', updateMapVisibility);
        if (document.readyState === 'loading') { document.addEventListener('DOMContentLoaded', () => { initJoystick(); updateMapVisibility(); }); } 
        else { initJoystick(); updateMapVisibility(); }
            
        function resize() {
            if (!ctx_joystick) return;
            width = (window.innerWidth > window.innerHeight) ? window.innerHeight : window.innerWidth;
            radius = width_to_radius_ratio * width; button_size = width_to_size_ratio * width; height = radius * radius_factor * 2 + 100;
            ctx_joystick.canvas.width = width; ctx_joystick.canvas.height = height;
            origin_joystick.x = width / 2; origin_joystick.y = height / 2; joystick(origin_joystick.x, origin_joystick.y);
        }

        function joystick_background() {
            ctx_joystick.clearRect(0, 0, canvas_joystick.width, canvas_joystick.height);
            ctx_joystick.beginPath(); ctx_joystick.arc(origin_joystick.x, origin_joystick.y, radius * radius_factor, 0, Math.PI * 2, true); ctx_joystick.fillStyle = '#ECE5E5'; ctx_joystick.fill();
            [[-1, 0, -50, -25, 25, -25, -25], [0, -1, -50, 25, -25, -25, -25], [1, 0, 50, 25, 25, 25, -25], [0, 1, 50, 25, 25, -25, 25]].forEach(p => {
                ctx_joystick.beginPath();
                ctx_joystick.moveTo(origin_joystick.x + p[0]*(radius*radius_factor) + p[2], origin_joystick.y + p[1]*(radius*radius_factor) + (p[0]===0?p[2]:0));
                ctx_joystick.lineTo(origin_joystick.x + p[0]*(radius*radius_factor) + p[3] + (p[1]===0?0:p[4]), origin_joystick.y + p[1]*(radius*radius_factor) + p[5] + (p[0]===0?0:p[4]));
                ctx_joystick.lineTo(origin_joystick.x + p[0]*(radius*radius_factor) + p[3] + (p[1]===0?0:p[6]), origin_joystick.y + p[1]*(radius*radius_factor) + p[5] + (p[0]===0?0:p[6]));
                ctx_joystick.fill();
            });
        }

        function joystick(x, y) { joystick_background(); ctx_joystick.beginPath(); ctx_joystick.arc(x, y, radius*3, 0, Math.PI * 2, true); ctx_joystick.fillStyle = 'lightgray'; ctx_joystick.fill(); ctx_joystick.strokeStyle = 'lightgray'; ctx_joystick.lineWidth = 2; ctx_joystick.stroke(); }

        let coord = { x: 0, y: 0 }, paint = false, movimento = 0;
        function getPosition_joystick(event) { var mouse_x = event.clientX || event.touches[0].clientX; var mouse_y = event.clientY || event.touches[0].clientY; coord.x = mouse_x - canvas_joystick.offsetLeft; coord.y = mouse_y - canvas_joystick.offsetTop; }
        function in_circle() { return (radius * radius_factor) >= Math.sqrt(Math.pow(coord.x - origin_joystick.x, 2) + Math.pow(coord.y - origin_joystick.y, 2)); }

        function resetModosAutonomosVisual() {
            led_state.linha = false; led_state.colisao = false; led_state.explora = false;
            ['linha', 'colisao', 'explora'].forEach(k => { document.getElementById("btn-led-" + k).classList.remove("on"); document.getElementById("led-status-" + k).innerText = "OFF"; });
        }
        function definirBotoesModoAutonomo(habilitado) { ['linha', 'colisao', 'explora'].forEach(k => { if (habilitado) document.getElementById("btn-led-" + k).removeAttribute("disabled"); else document.getElementById("btn-led-" + k).setAttribute("disabled", true); }); }

        function startDrawing(event) { paint = true; document.activeElement.blur(); resetModosAutonomosVisual(); definirBotoesModoAutonomo(false); getPosition_joystick(event); if (in_circle()) { joystick(coord.x, coord.y); Draw(event); } }
        function stopDrawing() { paint = false; joystick(origin_joystick.x, origin_joystick.y); document.getElementById("speed").innerText = 0; document.getElementById("table-speed").innerText = 0; if (movimento == 1) { safelySend({"velocidade":0, "angulo":0}); movimento = 0; } resetModosAutonomosVisual(); definirBotoesModoAutonomo(true); }

        function Draw(event) {
            if (paint) {
                getPosition_joystick(event);
                var angle = Math.atan2((coord.y - origin_joystick.y), (coord.x - origin_joystick.x)), x, y;
                if (in_circle()) { x = coord.x - radius / 2; y = coord.y - radius / 2; } else { x = radius * radius_factor * Math.cos(angle) + origin_joystick.x; y = radius * radius_factor * Math.sin(angle) + origin_joystick.y; }
                var speed = Math.round(100 * Math.sqrt(Math.pow(x - origin_joystick.x, 2) + Math.pow(y - origin_joystick.y, 2)) / (radius * radius_factor));
                if (speed > 100) speed = 100;
                var angle_in_degrees = (Math.sign(angle) == -1) ? Math.round( - angle * 180 / Math.PI) : Math.round(360 - angle * 180 / Math.PI);
                joystick(x, y);
                document.getElementById("speed").innerText = speed; document.getElementById("table-speed").innerText = speed;
                safelySend({"velocidade":speed, "angulo":angle_in_degrees}); movimento = 1;
            }
        }
    </script>
</body>
</html>
)rawliteral";

// --------------------------------------------------
// Declaracao das funcoes do codigo

void configurar_servidor_web(void);
void handleWebSocketMessage(void *, uint8_t *, size_t);
int16_t ler_distancia(void);
void atualizar_sensor_ultrassonico(void);
void onEvent(AsyncWebSocket *, AsyncWebSocketClient *, AwsEventType, void *, uint8_t *, size_t);
void calcula_PID(void);
void segue_linha(void);
void evita_colisao(void);
void explora_casa(void);
void navega_waypoints(void);
void calibrarSensoresLinha(void);
void verificarSegurancaBateria(void);

// Funções para controle com rastreamento para Odometria
void mover_motores(int v_esq, int v_dir);
void parar_motores();
void atualizar_odometria_fusao();

// --------------------------------------------------
// INTERRUPÇÕES DOS ENCODERS

void IRAM_ATTR isr_esq_A() { contador_esq_A++; }
void IRAM_ATTR isr_esq_B() { contador_esq_B++; }
void IRAM_ATTR isr_dir_A() { contador_dir_A++; }
void IRAM_ATTR isr_dir_B() { contador_dir_B++; }

// --------------------------------------------------
// FUNÇÕES WRAPPER DE MOTORES PARA ODOMETRIA

void mover_motores(int v_esq, int v_dir) {
    sinal_esq = (v_esq > 0) ? 1 : ((v_esq < 0) ? -1 : 0);
    sinal_dir = (v_dir > 0) ? 1 : ((v_dir < 0) ? -1 : 0);
    motores.turn(v_esq, v_dir);
}

void parar_motores() {
    sinal_esq = 0;
    sinal_dir = 0;
    motores.stop();
}

// --------------------------------------------------

void setup() {
  Serial.begin(115200);
  
  SerialGPS.setRxBufferSize(1024);
  SerialGPS.begin(9600, SERIAL_8N1, PINO_GPS_RX, PINO_GPS_TX);
  
  Serial.println("RoboCore - Kit Robo Explorer - Waypoints GPS");

  pinMode(PINO_LED, OUTPUT);
  digitalWrite(PINO_LED, LOW);

  pinMode(PINO_HCSR04_ECHO, INPUT);
  pinMode(PINO_HCSR04_TRIGGER, OUTPUT);
  digitalWrite(PINO_HCSR04_TRIGGER, LOW);

  pinMode(SENSOR_LINHA_ESQUERDO, INPUT);
  pinMode(SENSOR_LINHA_DIREITO, INPUT);

  pinMode(PINO_ENC_ESQ_A, INPUT);
  pinMode(PINO_ENC_ESQ_B, INPUT);
  pinMode(PINO_ENC_DIR_A, INPUT);
  pinMode(PINO_ENC_DIR_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PINO_ENC_ESQ_A), isr_esq_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_ENC_ESQ_B), isr_esq_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_ENC_DIR_A), isr_dir_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_ENC_DIR_B), isr_dir_B, CHANGE);

  WiFi.mode(WIFI_AP);
#if ESP_ARDUINO_VERSION_MAJOR > 2
  WiFi.softAPdisconnect();
  delay(100);
  WiFi.softAP("Vespa", "12345");
  const char *mac = WiFi.softAPmacAddress().c_str();
#else
  const char *mac = WiFi.macAddress().c_str();
#endif
  
  char ssid[] = "Scott-Robot";
  char *senha = "Lucas@15";
  for (uint8_t i = 6; i < 11; i++) { ssid[i] = mac[i + 6]; }
  if (!WiFi.softAP(ssid, senha)) {
    while (1) { digitalWrite(PINO_LED, HIGH); delay(100); digitalWrite(PINO_LED, LOW); delay(100); }
  }
  
  configurar_servidor_web();
  server.begin();

  SPIFFS.begin(DIRETORIO_SPIFFS, false);
  espera = SPIFFS.getFloat(ENDERECOS_SPIFFS[0], 10);
  Kp = SPIFFS.getFloat(ENDERECOS_SPIFFS[1], 6);
  Ki = SPIFFS.getFloat(ENDERECOS_SPIFFS[2], 0.2);
  Kd = SPIFFS.getFloat(ENDERECOS_SPIFFS[3], 20);
  limiarLinha = SPIFFS.getInt("limiar", 3000); 
  linha_escura = SPIFFS.getBool("linha_escura", true);
  SPIFFS.end();
}

void loop() {
  ws.cleanupClients();
  
  while (SerialGPS.available() > 0) { gps.encode(SerialGPS.read()); }
  atualizar_sensor_ultrassonico();

  // Executa a malha de odometria e fusão a 20Hz (a cada 50ms)
  static unsigned long tempo_odo_fusao = 0;
  if (millis() - tempo_odo_fusao > 50) {
      atualizar_odometria_fusao();
      tempo_odo_fusao = millis();
  }

  // 1. Monitoramento da Segurança da Bateria
  static uint32_t ultimoCheckBateria = 0;
  if (millis() - ultimoCheckBateria > 2000) { ultimoCheckBateria = millis(); verificarSegurancaBateria(); }

  // 2. Cálculo de RPM e Velocidade em cm/s via Encoder
  static unsigned long last_contador_esq_A = 0;
  static unsigned long last_contador_esq_B = 0;
  static unsigned long last_contador_dir_A = 0;
  static unsigned long last_contador_dir_B = 0;

  if ((millis() - tempo_antes_encoder) > INTERVALO_CALCULO) {
    unsigned long delta_esq_A = contador_esq_A - last_contador_esq_A;
    unsigned long delta_esq_B = contador_esq_B - last_contador_esq_B;
    int media_esq = (delta_esq_A + delta_esq_B) / NUMERO_CONTADORES;
    velocidade_rpm_esq = (float)media_esq / (NUMERO_DENTES * NUMERO_LEITURAS) * (60000.0 / INTERVALO_CALCULO);

    unsigned long delta_dir_A = contador_dir_A - last_contador_dir_A;
    unsigned long delta_dir_B = contador_dir_B - last_contador_dir_B;
    int media_dir = (delta_dir_A + delta_dir_B) / NUMERO_CONTADORES;
    velocidade_rpm_dir = (float)media_dir / (NUMERO_DENTES * NUMERO_LEITURAS) * (60000.0 / INTERVALO_CALCULO);

    float media_rpm = (velocidade_rpm_esq + velocidade_rpm_dir) / 2.0;
    robot_speed_cms = (media_rpm * 20.4) / 60.0;

    if (ws.count() > 0) {
      JsonDocument json;
      json["robot_speed"] = robot_speed_cms;
      size_t msg_comp = measureJson(json); 
      char msg[msg_comp + 1];
      serializeJson(json, msg, msg_comp + 1); 
      msg[msg_comp] = 0; 
      ws.textAll(msg, msg_comp);
    }

    last_contador_esq_A = contador_esq_A;
    last_contador_esq_B = contador_esq_B;
    last_contador_dir_A = contador_dir_A;
    last_contador_dir_B = contador_dir_B;
    tempo_antes_encoder = millis();
  }

  // 3. Executa os modos autônomos
  if (modoSegurancaBateria) {
      modo_linha = false; modo_colisao = false; modo_explora = false; modo_waypoints = false;
      parar_motores();
  } else {
      if (modo_linha) segue_linha();
      else if (modo_colisao) evita_colisao();
      else if (modo_explora) explora_casa();
      else if (modo_waypoints) navega_waypoints();
  }

  // 4. Processa a troca de Wi-Fi
  if (change_wifi_flag) {
    change_wifi_flag = false;
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(new_wifi_ssid.c_str(), new_wifi_pass.c_str());
    uint32_t timeout_conexao = millis() + 15000; 
    bool conectado = false;
    while (millis() < timeout_conexao) {
      if (WiFi.status() == WL_CONNECTED) { conectado = true; break; }
      delay(500);
    }
    if (conectado) {
      String ipDaCasa = WiFi.localIP().toString();
      if (ws.count() > 0) {
        JsonDocument jsonIp; jsonIp[ALIAS_IP] = ipDaCasa;
        size_t msg_comp = measureJson(jsonIp); char msg[msg_comp + 1];
        serializeJson(jsonIp, msg, (msg_comp + 1)); msg[msg_comp] = 0;
        ws.textAll(msg, msg_comp);
      }
    } else {
      WiFi.mode(WIFI_AP); 
    }
  }

  // 5. VBAT
  if (millis() > timeout_vbat) {
    if (ws.count() > 0) {
      uint32_t tensao = vbat.readVoltage();
      JsonDocument json; json[ALIAS_VBAT] = tensao;
      size_t msg_comp = measureJson(json); char msg[msg_comp + 1];
      serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0; ws.textAll(msg, msg_comp);
    }
    timeout_vbat = millis() + TEMPO_ATUALIZACAO_VBAT;
  }
  
  // 6. Dados de GPS e Localização
  if (millis() > timeout_gps) {
    if (ws.count() > 0) {
        float flat, flon; unsigned long age;
        gps.f_get_position(&flat, &flon, &age);

        int year; byte month, day, hour, minute, second, hundredths;
        gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
        int local_hour = hour - 3; int local_day = day; int local_month = month; int local_year = year;
        
        if (local_hour < 0) {
            local_hour += 24; local_day -= 1;
            if (local_day == 0) {
                local_month -= 1; if (local_month == 0) { local_month = 12; local_year -= 1; }
                if (local_month == 4 || local_month == 6 || local_month == 9 || local_month == 11) { local_day = 30; } 
                else if (local_month == 2) { bool isLeap = ((local_year % 4 == 0 && local_year % 100 != 0) || (local_year % 400 == 0)); local_day = isLeap ? 29 : 28; } 
                else { local_day = 31; }
            }
        }
        char date_str[16]; char time_str[16];
        if (age == TinyGPS::GPS_INVALID_AGE) { sprintf(date_str, "--/--/----"); sprintf(time_str, "--:--:--"); } 
        else { sprintf(date_str, "%02d/%02d/%04d", local_day, local_month, local_year); sprintf(time_str, "%02d:%02d:%02d", local_hour, minute, second); }

        JsonDocument json;
        if (flat == TinyGPS::GPS_INVALID_F_ANGLE) { json["lat"] = nullptr; json["lon"] = nullptr; } 
        else { 
            json["lat"] = odometria_inicializada ? estimativa_lat : flat; 
            json["lon"] = odometria_inicializada ? estimativa_lon : flon; 
        }
        
        json["date"] = date_str; 
        json["time"] = time_str;
        
        size_t msg_comp = measureJson(json); char msg[msg_comp + 1];
        serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0; ws.textAll(msg, msg_comp);
    }
    timeout_gps = millis() + 1000;
  }
  
  delay(1); 
}

// --------------------------------------------------
// LÓGICA DE FUSÃO SENSORIAL COM CALIBRAÇÃO GEOMÉTRICA BLINDADA

void atualizar_odometria_fusao() {
    // 1. ODOMETRIA COM ENCODERS
    long pulsos_esq_atual = contador_esq_A;
    long pulsos_dir_atual = contador_dir_A;
    
    long delta_esq = (long)(pulsos_esq_atual - odo_esq_anterior) * sinal_esq;
    long delta_dir = (long)(pulsos_dir_atual - odo_dir_anterior) * sinal_dir;
    
    odo_esq_anterior = pulsos_esq_atual;
    odo_dir_anterior = pulsos_dir_atual;
    
    float dist_esq = delta_esq * CM_POR_PULSO;
    float dist_dir = delta_dir * CM_POR_PULSO;
    float dist_centro = (dist_esq + dist_dir) / 2.0;
    
    float delta_theta = (dist_esq - dist_dir) / LARGURA_ESTEIRA_EFETIVA;
    theta_rad += delta_theta;
    
    while (theta_rad >= TWO_PI) theta_rad -= TWO_PI;
    while (theta_rad < 0) theta_rad += TWO_PI;
    
    float delta_x_cm = dist_centro * cos(theta_rad); 
    float delta_y_cm = dist_centro * sin(theta_rad); 
    
    float delta_lat = delta_x_cm / 11132000.0; 
    float delta_lon = 0.0;
    
    if (odometria_inicializada) {
        delta_lon = delta_y_cm / (11132000.0 * cos(estimativa_lat * PI / 180.0));
        estimativa_lat += delta_lat;
        estimativa_lon += delta_lon;
    }

    // 2. FUSÃO SENSORIAL E CALIBRAÇÃO GEOMÉTRICA BLINDADA
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    static float ultimo_flat_processado = 0.0;
    
    if (flat != TinyGPS::GPS_INVALID_F_ANGLE && age < 1500) {
        if (!odometria_inicializada) {
            estimativa_lat = flat;
            estimativa_lon = flon;
            ultimo_flat_processado = flat;
            odometria_inicializada = true;
            heading_gps_valido = false; 
            registrou_inicio_calibracao = false;
        } else {
            if (flat != ultimo_flat_processado) {
                // Continua corrigindo o X e Y do robô em tempo real
                estimativa_lat = (estimativa_lat * 0.85) + (flat * 0.15);
                estimativa_lon = (estimativa_lon * 0.85) + (flon * 0.15);
                ultimo_flat_processado = flat;
                
                // --- CALIBRAÇÃO GEOMÉTRICA SEGURA ---
                if (!heading_gps_valido && modo_waypoints) {
                    if (!registrou_inicio_calibracao) {
                        lat_inicio_calibracao = flat;
                        lon_inicio_calibracao = flon;
                        pulsos_inicio_calibracao = contador_esq_A; // Salva o marco mecânico
                        registrou_inicio_calibracao = true;
                    } else {
                        // O robô agora usa AS ESTEIRAS como régua, não o GPS!
                        float dist_fisica_cm = (contador_esq_A - pulsos_inicio_calibracao) * CM_POR_PULSO;
                        
                        // Exige 2.5 metros FÍSICOS para ignorar qualquer ruído do satélite
                        if (dist_fisica_cm >= 250.0) {
                            float curso_calculado = TinyGPS::course_to(lat_inicio_calibracao, lon_inicio_calibracao, flat, flon);
                            theta_rad = curso_calculado * PI / 180.0;
                            heading_gps_valido = true; 
                        }
                    }
                }
            }
        }
    }
}

// --------------------------------------------------
void configurar_servidor_web(void) {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });
}

void controlar_motor(uint8_t *data, size_t length){
  JsonDocument json;
  deserializeJson(json, data, length);
  int16_t angulo = json[ALIAS_ANGULO], velocidade = json[ALIAS_VELOCIDADE];

  if (velocidade == 0) parar_motores();
  else if ((angulo >= 90) && (angulo <= 180)) mover_motores(velocidade * (135 - angulo) / 45, velocidade);
  else if ((angulo >= 0) && (angulo < 90)) mover_motores(velocidade, velocidade * (angulo - 45) / 45);
  else if ((angulo > 180) && (angulo <= 270)) mover_motores(-1 * velocidade, -1 * velocidade * (angulo - 225) / 45);
  else if (angulo > 270) mover_motores(-1 * velocidade * (315 - angulo) / 45, -1 * velocidade);
  else parar_motores();
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t length) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == length && info->opcode == WS_TEXT) {
    data[length] = 0;

    if (strstr(reinterpret_cast<char *>(data), ALIAS_WIFI_SSID) != nullptr) {
      JsonDocument json; deserializeJson(json, data, length);
      new_wifi_ssid = json[ALIAS_WIFI_SSID].as<String>();
      new_wifi_pass = json[ALIAS_WIFI_PASS].as<String>();
      change_wifi_flag = true;
    }
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_WAYPOINT) != nullptr) {
      JsonDocument json; deserializeJson(json, data, length);
      modo_waypoints = json[ALIAS_WAYPOINT].as<bool>();
      if (modo_waypoints) {
          modo_linha = false; modo_colisao = false; modo_explora = false;
          estadoWay = WAY_LIVRE;
          heading_gps_valido = false; 
          registrou_inicio_calibracao = false; // NOVA ÂNCORA
          target_lat = json["target_lat"].as<float>();
          target_lon = json["target_lon"].as<float>();
      } else {
          parar_motores();
      }
    }
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_LINHA) != nullptr || strstr(reinterpret_cast<char *>(data), ALIAS_COLISAO) != nullptr || strstr(reinterpret_cast<char *>(data), ALIAS_EXPLORA) != nullptr) {
      JsonDocument json; deserializeJson(json, data, length);
      
      if (json.containsKey(ALIAS_LINHA)) modo_linha = json[ALIAS_LINHA].as<bool>();
      if (json.containsKey(ALIAS_COLISAO)) modo_colisao = json[ALIAS_COLISAO].as<bool>();
      if (json.containsKey(ALIAS_EXPLORA)) modo_explora = json[ALIAS_EXPLORA].as<bool>();
      
      if (modo_linha) { 
          modo_colisao = false; modo_explora = false; modo_waypoints = false;
          erro = 0.0; I = 0.0; erro_anterior = 0.0; contador_parada = 0; 
      }
      else if (modo_colisao) { modo_linha = false; modo_explora = false; modo_waypoints = false; }
      else if (modo_explora) { modo_linha = false; modo_colisao = false; modo_waypoints = false; }

      if (!modo_linha && !modo_colisao && !modo_explora && !modo_waypoints) {
        parar_motores();
        estadoManobraAtual = LIVRE;
        estadoExplora = EXPLORA_LIVRE;
        ws.textAll("{\"fall\":false, \"stuck\":false}"); 
      }
    }
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_KP) != nullptr) {
      JsonDocument json; deserializeJson(json, data, length);
      parada = false; Kp = json[ALIAS_KP]; Ki = json[ALIAS_KI]; Kd = json[ALIAS_KD];
      SPIFFS.begin(DIRETORIO_SPIFFS, false);
      SPIFFS.putFloat(ENDERECOS_SPIFFS[0], espera); SPIFFS.putFloat(ENDERECOS_SPIFFS[1], Kp);
      SPIFFS.putFloat(ENDERECOS_SPIFFS[2], Ki); SPIFFS.putFloat(ENDERECOS_SPIFFS[3], Kd);
      SPIFFS.end();
    } 
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_VELOCIDADE) != nullptr) {
      if (modo_linha || modo_colisao || modo_explora || modo_waypoints) {
        modo_linha = false; modo_colisao = false; modo_explora = false; modo_waypoints = false;
        parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE;
        ws.textAll("{\"fall\":false, \"stuck\":false}"); 
      }
      controlar_motor(data, length);
    } 
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_PARA) != nullptr) {
      modo_linha = false; modo_colisao = false; modo_explora = false; modo_waypoints = false;
      parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE;
      ws.textAll("{\"fall\":false, \"stuck\":false}"); 
    }
    else if (strstr(reinterpret_cast<char *>(data), "cmd") != nullptr) {
      JsonDocument json; deserializeJson(json, data, length);
      if (json["cmd"].as<String>() == "calibrate_line") calibrarSensoresLinha();
    }
  }
}

// --------------------------------------------------
int16_t ler_distancia(void) {
  int16_t leituras[3];
  for (int i = 0; i < 3; i++) {
    digitalWrite(PINO_HCSR04_TRIGGER, HIGH); 
    delayMicroseconds(10); 
    digitalWrite(PINO_HCSR04_TRIGGER, LOW);
    unsigned long duracao = pulseIn(PINO_HCSR04_ECHO, HIGH, 25000);
    leituras[i] = (duracao == 0) ? -1 : (duracao / 58);
    delay(5);
  }
  
  for (int i = 0; i < 2; i++) {
    for (int j = i + 1; j < 3; j++) {
      if (leituras[j] < leituras[i]) {
        int16_t temp = leituras[i];
        leituras[i] = leituras[j];
        leituras[j] = temp;
      }
    }
  }
  return leituras[1]; 
}

void atualizar_sensor_ultrassonico(void) {
  if (millis() > timeout_distancia) {
    int16_t leitura = ler_distancia();
    if (leitura > 0 && leitura < 400) {
      distancia = (leitura * 0.2) + (distancia * 0.8); 
    }
    if (millis() > timeout_distancia_ws) {
        if (ws.count() > 0) {
          JsonDocument json; json[ALIAS_DISTANCIA] = (int)distancia;
          size_t msg_comp = measureJson(json); char msg[msg_comp + 1];
          serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0; ws.textAll(msg, msg_comp);
        }
        timeout_distancia_ws = millis() + 250; 
    }
    timeout_distancia = millis() + TEMPO_ATUALIZACAO_DISTANCIA;
  }
}

// --------------------------------------------------
void segue_linha() {
  leitura_esquerdo = analogRead(SENSOR_LINHA_ESQUERDO);
  leitura_direito = analogRead(SENSOR_LINHA_DIREITO);

  bool ve_esq = (leitura_esquerdo > limiarLinha);
  bool ve_dir = (leitura_direito > limiarLinha);

   if(ve_esq && ve_dir) {
      erro = 0.0; 
      calcula_PID(); 
      mover_motores(velocidade_esquerda, velocidade_direita); 
      contador_parada = 0; 
  }
  else if(!ve_esq && !ve_dir){
      if (erro == 1.0) { erro = 2.0; } 
      else if (erro < 0.0) { erro = -2.0; }
      calcula_PID(); 
      mover_motores(velocidade_esquerda, velocidade_direita); 
  }
  else if(ve_dir) {
      erro = 1.0; 
      calcula_PID(); 
      mover_motores(velocidade_esquerda, velocidade_direita); 
      contador_parada = 0; 
  }
  else if(ve_esq) {
      erro = -1.0; 
      calcula_PID(); 
      mover_motores(velocidade_esquerda, velocidade_direita); 
      contador_parada = 0; 
  }

  if (contador_parada >= CONTAGEM_MAXIMA) { 
     parar_motores(); P = 0; I = 0; D = 0; 
     contador_parada = CONTAGEM_MAXIMA;
  }
  delay(espera);
}

// --------------------------------------------------
void evita_colisao() {
  if (estadoManobraAtual != LIVRE) {
    bool transicao = false;

    if ((contador_esq_A - posicao_inicial_manobra) >= alvoPulsosManobra) {
        transicao = true;
    }

    if (!transicao) { return; } 
    else {
        if (estadoManobraAtual == MANOBRA_QUEDA_RE || estadoManobraAtual == MANOBRA_PAREDE_RE) {
            parar_motores();
            delay(250); 
            mover_motores(VELOCIDADE_GIRO, -VELOCIDADE_GIRO); 
            estadoManobraAtual = (estadoManobraAtual == MANOBRA_QUEDA_RE) ? MANOBRA_QUEDA_GIRO : MANOBRA_PAREDE_GIRO;
            posicao_inicial_manobra = contador_esq_A;
            alvoPulsosManobra = PULSOS_180_GRAUS;
            return;
        } else {
            parar_motores();
            delay(100);
            estadoManobraAtual = LIVRE;
            if (ws.count() > 0) ws.textAll("{\"fall\":false}");
            return;
        }
    }
  }

  int leitura_esq = analogRead(SENSOR_LINHA_ESQUERDO); int leitura_dir = analogRead(SENSOR_LINHA_DIREITO);
  bool obstaculo_frente = (distancia > 0) && (distancia <= DISTANCIA_OBSTACULO);
  bool queda_detectada = (leitura_esq > LIMIAR_QUEDA) || (leitura_dir > LIMIAR_QUEDA);

  static bool status_queda_ui = false;
  if (!queda_detectada && status_queda_ui) { if (ws.count() > 0) ws.textAll("{\"fall\":false}"); status_queda_ui = false; }

  if (queda_detectada || obstaculo_frente) {
      if (queda_detectada && !status_queda_ui) { if (ws.count() > 0) ws.textAll("{\"fall\":true}"); status_queda_ui = true; }
      parar_motores();
      delay(250); 
      mover_motores(-VELOCIDADE, -VELOCIDADE); 
      estadoManobraAtual = queda_detectada ? MANOBRA_QUEDA_RE : MANOBRA_PAREDE_RE;
      
      posicao_inicial_manobra = contador_esq_A;
      alvoPulsosManobra = queda_detectada ? PULSOS_30_CM : PULSOS_15_CM; 
      timeout_distancia = millis() + TEMPO_ATUALIZACAO_DISTANCIA;
  } else {
    mover_motores(VELOCIDADE, VELOCIDADE);
  }
}

// --------------------------------------------------
void explora_casa() {
  bool obstaculo_frente = (distancia > 0) && (distancia <= DISTANCIA_EXPLORACAO);
  bool queda_detectada = (analogRead(SENSOR_LINHA_ESQUERDO) > LIMIAR_QUEDA) || (analogRead(SENSOR_LINHA_DIREITO) > LIMIAR_QUEDA);
  
  static bool status_queda_ui_explora = false;

  if (estadoExplora == EXPLORA_PARADO) {
      parar_motores();
      return; 
  }

  if (estadoExplora != EXPLORA_LIVRE) {
      if (millis() - tempo_inicio_manobra_explora > TIMEOUT_MAX_EXPLORA) {
          estadoExplora = EXPLORA_LIVRE;
          giros_consecutivos = 0;
          mover_motores(VELOCIDADE_EXPLORACAO, VELOCIDADE_EXPLORACAO);
          return;
      }

      if (estadoExplora == EXPLORA_RE) {
          if ((contador_esq_A - posicao_inicial_explora) >= alvoPulsosExplora) {
              parar_motores();
              delay(250); 
              
              mover_motores(VELOCIDADE_GIRO, -VELOCIDADE_GIRO); 
              estadoExplora = EXPLORA_GIRO;
              posicao_inicial_explora = contador_esq_A;
              alvoPulsosExplora = PULSOS_45_GRAUS;
              tempo_inicio_manobra_explora = millis(); 
          }
      } 
      else if (estadoExplora == EXPLORA_GIRO) {
          if ((contador_esq_A - posicao_inicial_explora) >= alvoPulsosExplora) {
              parar_motores();
              delay(100);
              
              giros_consecutivos++;

              bool ainda_obstaculo = ((distancia > 0) && (distancia <= DISTANCIA_EXPLORACAO));
              bool ainda_queda = (analogRead(SENSOR_LINHA_ESQUERDO) > LIMIAR_QUEDA) || (analogRead(SENSOR_LINHA_DIREITO) > LIMIAR_QUEDA);

              if (ainda_obstaculo || ainda_queda) {
                  if (giros_consecutivos >= 8) {
                      estadoExplora = EXPLORA_PARADO;
                      if (ws.count() > 0) ws.textAll("{\"stuck\":true}"); 
                  } else {
                      mover_motores(VELOCIDADE_GIRO, -VELOCIDADE_GIRO); 
                      posicao_inicial_explora = contador_esq_A;
                      alvoPulsosExplora = PULSOS_45_GRAUS;
                      tempo_inicio_manobra_explora = millis();
                      
                      if (ainda_queda && !status_queda_ui_explora) {
                          if (ws.count() > 0) ws.textAll("{\"fall\":true}"); 
                          status_queda_ui_explora = true;
                      }
                  }
              } else {
                  estadoExplora = EXPLORA_LIVRE;
                  giros_consecutivos = 0;
                  if (status_queda_ui_explora) {
                      if (ws.count() > 0) ws.textAll("{\"fall\":false}");
                      status_queda_ui_explora = false;
                  }
                  mover_motores(VELOCIDADE_EXPLORACAO, VELOCIDADE_EXPLORACAO);
              }
          }
      }
      return;
  }

  if (queda_detectada || obstaculo_frente) {
      if (queda_detectada && !status_queda_ui_explora) { 
          if (ws.count() > 0) ws.textAll("{\"fall\":true}"); 
          status_queda_ui_explora = true; 
      }
      parar_motores();
      delay(250); 
      
      mover_motores(-VELOCIDADE_EXPLORACAO, -VELOCIDADE_EXPLORACAO);
      estadoExplora = EXPLORA_RE;
      giros_consecutivos = 0;
      posicao_inicial_explora = contador_esq_A;
      alvoPulsosExplora = PULSOS_20_CM;
      tempo_inicio_manobra_explora = millis();
  } else {
      if (status_queda_ui_explora) {
          if (ws.count() > 0) ws.textAll("{\"fall\":false}");
          status_queda_ui_explora = false;
      }
      mover_motores(VELOCIDADE_EXPLORACAO, VELOCIDADE_EXPLORACAO);
  }
}

// --------------------------------------------------
// --------------------------------------------------
void navega_waypoints() {
  if (!odometria_inicializada) {
      parar_motores();
      return; 
  }

  bool obstaculo_frente = false; 
  bool queda_detectada = false; 

  // FASE 1: CALIBRAÇÃO DA BÚSSOLA GEOMÉTRICA (Usa a régua dos encoders)
  if (!heading_gps_valido) {
      if (obstaculo_frente || queda_detectada) {
          parar_motores(); 
          if (ws.count() > 0) ws.textAll("{\"waypoint_status\":\"Bloqueado na Calibracao!\"}");
      } else {
          mover_motores(VELOCIDADE_EXPLORACAO, VELOCIDADE_EXPLORACAO);
          if (ws.count() > 0) ws.textAll("{\"waypoint_status\":\"Calibrando Direcao...\"}");
      }
      return; 
  }

  // FASE 2: VERIFICAÇÃO DE CHEGADA (Raio de 1.0m)
  float distancia_alvo = TinyGPS::distance_between(estimativa_lat, estimativa_lon, target_lat, target_lon);
  if (distancia_alvo < 1.0) {
      parar_motores();
      modo_waypoints = false;
      estadoWay = WAY_LIVRE;
      heading_gps_valido = false; 
      if (ws.count() > 0) ws.textAll("{\"waypoint_status\":\"alvo_alcancado\"}");
      return;
  }

  // FASE 3: NAVEGAÇÃO RETA CORRIGIDA POR GPS
  float curso_alvo = TinyGPS::course_to(estimativa_lat, estimativa_lon, target_lat, target_lon);
  
  // Atualiza o curso atual diretamente com o vetor real do GPS em movimento
  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  static float ant_lat = 0, ant_lon = 0;
  
  if (flat != TinyGPS::GPS_INVALID_F_ANGLE && (flat != ant_lat || flon != ant_lon)) {
      float curso_gps_real = TinyGPS::course_to(ant_lat, ant_lon, flat, flon);
      if (curso_gps_real >= 0) {
          // Mistura o curso das esteiras com o curso real do GPS para evitar círculos
          float erro_mix = curso_gps_real - (theta_rad * 180.0 / PI);
          while(erro_mix > 180) erro_mix -= 360;
          while(erro_mix < -180) erro_mix += 360;
          theta_rad += (erro_mix * 0.3) * PI / 180.0; // Correção suave
      }
      ant_lat = flat;
      ant_lon = flon;
  }

  float curso_atual = theta_rad * 180.0 / PI;
  float erro_angular = curso_alvo - curso_atual;
  if (erro_angular < -180.0) erro_angular += 360.0;
  if (erro_angular > 180.0) erro_angular -= 360.0;

  if (abs(erro_angular) > 35.0) {
      int sentido = (erro_angular > 0) ? 1 : -1;
      mover_motores(75 * sentido, -75 * sentido); // Giro controlado de correção
  } else {
      int vel_esq = VELOCIDADE_EXPLORACAO; 
      int vel_dir = VELOCIDADE_EXPLORACAO; 
      
      if (erro_angular > 6.0) { 
          vel_dir = 60;  
          vel_esq = 90; 
      } else if (erro_angular < -6.0) { 
          vel_esq = 60;
          vel_dir = 90;
      }
      mover_motores(vel_esq, vel_dir);
  }
}

// --------------------------------------------------
void calcula_PID() {
  P = erro; 
  I = I + erro; 

  if (I > 50) I = 50;
  else if (I < -50) I = -50;
  if (erro == 0.0) I = 0; 

  D = erro - erro_anterior; 

  resposta_PID = (Kp * P) + (Ki * I) + (Kd * D); 
  erro_anterior = erro;
  
  velocidade_esquerda = VELOCIDADE + resposta_PID; 
  velocidade_direita =  VELOCIDADE - resposta_PID;
  
  if (velocidade_esquerda > VELOCIDADE_MAXIMA) velocidade_esquerda = VELOCIDADE_MAXIMA; 
  else if (velocidade_esquerda < VELOCIDADE_MINIMA) velocidade_esquerda = VELOCIDADE_MINIMA;
  
  if (velocidade_direita > VELOCIDADE_MAXIMA) velocidade_direita = VELOCIDADE_MAXIMA; 
  else if (velocidade_direita < VELOCIDADE_MINIMA) velocidade_direita = VELOCIDADE_MINIMA;
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t length) {
  switch (type) {
    case WS_EVT_CONNECT: digitalWrite(PINO_LED, HIGH); break;
    case WS_EVT_DISCONNECT:
        if (ws.count() == 0) {
          digitalWrite(PINO_LED, LOW); modo_linha = false; modo_colisao = false; modo_explora = false; modo_waypoints = false; parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE;
        } break;
    case WS_EVT_DATA: handleWebSocketMessage(arg, data, length); break;
    case WS_EVT_PONG: case WS_EVT_ERROR: break;
  }
}

void calibrarSensoresLinha() {
    modo_linha = false; modo_colisao = false; modo_explora = false; modo_waypoints = false; parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE;
    int leituraInicialLinha = analogRead(SENSOR_LINHA_ESQUERDO); 
    int minVal = 4095; int maxVal = 0; uint32_t inicio = millis();
    
    while (millis() - inicio < 2500) {
        if (((millis() - inicio) / 250) % 2 == 0) mover_motores(60, -60); else mover_motores(-60, 60);
        int leituraEsq = analogRead(SENSOR_LINHA_ESQUERDO); int leituraDir = analogRead(SENSOR_LINHA_DIREITO);
        if (leituraEsq < minVal) minVal = leituraEsq; if (leituraDir < minVal) minVal = leituraDir;
        if (leituraEsq > maxVal) maxVal = leituraEsq; if (leituraDir > maxVal) maxVal = leituraDir;
        delay(10);
    }

    parar_motores();
    limiarLinha = (minVal + maxVal) / 2; linha_escura = (leituraInicialLinha > limiarLinha);
    SPIFFS.begin(DIRETORIO_SPIFFS, false); SPIFFS.putInt("limiar", limiarLinha); SPIFFS.putBool("linha_escura", linha_escura); SPIFFS.end();
    
    JsonDocument json; json["status"] = "calibrado"; json["limiar"] = limiarLinha; json["escura"] = linha_escura;
    size_t msg_comp = measureJson(json); char msg[msg_comp + 1]; serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0;
    if (ws.count() > 0) ws.textAll(msg, msg_comp);
}

void verificarSegurancaBateria() {
    uint32_t tensao_mv = vbat.readVoltage();
    static uint8_t leituras_criticas_consecutivas = 0;
    
    if (tensao_mv < TENSAO_CRITICA && tensao_mv > 3000) {
        leituras_criticas_consecutivas++;
        if (leituras_criticas_consecutivas >= 3) {
            if (!modoSegurancaBateria) {
                modoSegurancaBateria = true; modo_linha = false; modo_colisao = false; modo_explora = false; modo_waypoints = false; parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE;
                if (ws.count() > 0) ws.textAll("{\"alert\":\"LOW_BATTERY\"}");
            }
            digitalWrite(PINO_LED, (millis() / 150) % 2 ? HIGH : LOW);
        }
    } 
    else if (tensao_mv >= (TENSAO_CRITICA + 300)) { 
        leituras_criticas_consecutivas = 0; modoSegurancaBateria = false; digitalWrite(PINO_LED, LOW);
    }
}