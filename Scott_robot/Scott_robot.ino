/*******************************************************************************
* Scott Robot - Kit Robo Explorer - Joystick + Linha + Exploração + Waypoints GPS + Odometria + Grid Mapping
* (Versão sem Encoders - Estimativa por Tempo e GPS)
*******************************************************************************/

// --------------------------------------------------
// Bibliotecas

#include <esp_arduino_version.h>  

#include <WiFi.h>
#include <AsyncTCP.h>           
#include <ESPAsyncWebServer.h>  
#include <ArduinoJson.h>        
#include <Preferences.h>
#include <LittleFS.h>           
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
float robot_speed_cms = 0.0;

// Waypoints GPS Fila e Segurança
bool modo_waypoints = false;
float waypoints_lat[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float waypoints_lon[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
int total_waypoints = 0;
int waypoint_atual_idx = 0;
bool wp_evita_queda = true;
bool wp_evita_colisao = true;

// --- MÁQUINA DE ESTADOS WAYPOINT ---
enum EstadoWaypoint { WAY_LIVRE, WAY_RE, WAY_GIRO };
EstadoWaypoint estadoWay = WAY_LIVRE;
unsigned long posicao_inicial_way = 0;
uint32_t tempo_inicio_manobra_way = 0;
int sentido_giro_way = 1; 

// --- ANTI-STALL DA NAVEGAÇÃO POR WAYPOINT ---
const uint32_t TIMEOUT_GIRO_WAYPOINT = 4000;         
const uint32_t EMPURRAO_ANTISTALL_WAYPOINT_MS = 1200; 
const int LIMITE_GIROS_ANTISTALL_WAYPOINT = 4;       
int giros_consecutivos_way = 0;

// --- VARIÁVEIS DE ODOMETRIA E FUSÃO (Estimada por tempo) ---
float estimativa_lat = 0.0;
float estimativa_lon = 0.0;
float theta_rad = 0.0; 

bool odometria_inicializada = false;
bool heading_gps_valido = false; 

float lat_inicio_calibracao = 0.0;
float lon_inicio_calibracao = 0.0;
bool registrou_inicio_calibracao = false;

int sinal_esq = 0; 
int sinal_dir = 0; 

// --- MAPEAMENTO LOCAL (GRID MAPPING) ---
#define MAP_WIDTH 100   
#define MAP_HEIGHT 100  
#define MAP_RESOLUTION_CM 5
int8_t occupancyMap[MAP_WIDTH][MAP_HEIGHT]; 

float robot_local_x_cm = 0.0;
float robot_local_y_cm = 0.0;
const float MAX_SENSOR_DIST_CM = 150.0; 
uint32_t timeout_map_ws = 0;

// --- NOVA FUNÇÃO: ANTENA VIRTUAL ---
bool obstaculo_virtual_detectado(float distancia_projecao_cm) {
    float alvo_x = robot_local_x_cm + distancia_projecao_cm * cos(theta_rad);
    float alvo_y = robot_local_y_cm + distancia_projecao_cm * sin(theta_rad);

    int grid_x = (int)(alvo_x / MAP_RESOLUTION_CM) + (MAP_WIDTH / 2);
    int grid_y = (int)(alvo_y / MAP_RESOLUTION_CM) + (MAP_HEIGHT / 2);

    if (grid_x >= 0 && grid_x < MAP_WIDTH && grid_y >= 0 && grid_y < MAP_HEIGHT) {
        if (occupancyMap[grid_x][grid_y] == 2) {
            return true; 
        }
    }
    return false;
}

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
const char *ALIAS_EXPLORA = "explora";
const char *ALIAS_WAYPOINT = "waypoint";

// Wi-Fi
bool change_wifi_flag = false;
String new_wifi_ssid = "";
String new_wifi_pass = "";
bool wifi_conectando = false;
uint32_t tempo_inicio_wifi = 0;
const uint32_t TIMEOUT_WIFI_CONNECT_MS = 15000;

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
float leitura_esquerda_filtrada = 0.0;
float leitura_direita_filtrada = 0.0;
const float ALPHA_FILTRO_LINHA = 0.5;

const float FATOR_NORMALIZACAO_LINHA = 2500.0; 
unsigned long pid_tempo_anterior = 0;

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

float espera, Kp = 40, Ki = 0.2, Kd = 0.4, erro = 0.0, P = 0.0, I = 0.0, D = 0.0, erro_anterior = 0.0, resposta_PID = 0.0;
bool parada = true;

bool modo_linha = false;
bool modo_explora = false;

enum EstadoManobra { LIVRE, MANOBRA_QUEDA_RE, MANOBRA_QUEDA_GIRO, MANOBRA_PAREDE_RE, MANOBRA_PAREDE_GIRO };
EstadoManobra estadoManobraAtual = LIVRE;

// --- MÁQUINA DE ESTADOS: DESVIO DE OBSTÁCULO NO MODO LINHA ---
// Sequência: gira p/ fora da linha -> avança de lado -> gira p/ ficar paralelo ->
// avança paralelo (passa do objeto) -> gira de volta em direção à linha -> procura a linha
enum EstadoDesvioLinha {
  DESVIO_LINHA_LIVRE,
  DESVIO_GIRO_SAIDA,
  DESVIO_AVANCA_LADO,
  DESVIO_GIRO_RETORNO,
  DESVIO_AVANCA_PARALELO,
  DESVIO_GIRO_LINHA,
  DESVIO_PROCURA_LINHA
};
EstadoDesvioLinha estadoDesvioLinha = DESVIO_LINHA_LIVRE;
uint32_t tempo_inicio_desvio = 0;
uint32_t alvoTempoDesvio = 0;
int sentido_desvio = 1; // 1 = contorna pela direita, -1 = contorna pela esquerda
int tentativas_desvio_consecutivas = 0;
const int MAX_TENTATIVAS_DESVIO = 3;

const int DISTANCIA_OBSTACULO_LINHA = 20;          // cm - gatilho para iniciar o desvio durante o modo_linha
//const uint32_t TEMPO_GIRO_DESVIO_MS = 550;         // tempo de giro (~45°) em cada etapa do desvio
const uint32_t TEMPO_GIRO_DESVIO_MS = 1100;

//const uint32_t TEMPO_AVANCO_LADO_MS = 600;         // avanço lateral suficiente para ultrapassar a lateral do objeto
const uint32_t TEMPO_AVANCO_LADO_MS = 900;         // avanço lateral suficiente para ultrapassar a lateral do objeto
//const uint32_t TEMPO_AVANCO_PARALELO_MS = 900;     // avanço paralelo à linha, cobrindo o comprimento do objeto
const uint32_t TEMPO_AVANCO_PARALELO_MS = 1200;     // avanço paralelo à linha, cobrindo o comprimento do objeto
const uint32_t TEMPO_MAX_PROCURA_LINHA_MS = 3000;  // tempo máximo procurando reencontrar a linha antes de declarar "stuck"

// --- MÁQUINA DE ESTADOS EXPLORAÇÃO (Baseada em Tempo) ---
enum EstadoExploracao { EXPLORA_LIVRE, EXPLORA_RE, EXPLORA_GIRO, EXPLORA_PARADO };
EstadoExploracao estadoExplora = EXPLORA_LIVRE;
int giros_consecutivos = 0; 
uint32_t tempo_inicio_manobra_explora = 0;
uint32_t alvoTempoExplora = 0; 
int sentido_giro_atual = 1;
const int MAX_GIROS_CONSECUTIVOS = 3;
const uint32_t DURACAO_KICK_ANTISTALL_MS = 150;
const uint32_t TEMPO_RE_EXPLORA_MS = 1000;
const uint32_t TEMPO_GIRO_90_MS = 1200;
const uint32_t TEMPO_GIRO_45_MS = 600;

bool modoSegurancaBateria = false;
uint32_t ultima_tensao_bateria_mv = 0; 
const uint32_t TENSAO_CRITICA = 6400;

Preferences SPIFFS; 
const char* DIRETORIO_SPIFFS = "seguidor";
const char* ENDERECOS_SPIFFS[4] = {"espera", "Kp", "Ki", "Kd"};

// --------------------------------------------------
// Pagina web principal (Inalterada)
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
        
        .grid-3x3 { border-collapse: collapse; margin: 0 auto; background: #fff; width: 100%; max-width: 400px; font-size: 13px; }
        .grid-3x3 td { padding: 6px; border: 2px solid #ECE5E5; text-align: center; }

        #btn-led-linha, #btn-led-explora {
            padding: 16px 20px; font-size: 16px; font-family: inherit; background-color: #444; color: #fff; border: 3px solid #666; border-radius: 10px; cursor: pointer; user-select: none; transition: background-color 0.15s; outline: none; width: 100px;
        }
        #btn-led-linha.on, #btn-led-explora.on { background-color: #f0be00; border-color: #c49a00; color: #000; font-weight:bold;}
        #btn-led-linha:disabled, #btn-led-explora:disabled { background-color: #eee; border-color: #ccc; color: #999; cursor: not-allowed; }

        .menu { display: flex; background-color: #222; border-bottom: 2px solid #444; height: 40px; }
        .menu-item { flex: 1; text-align: center; line-height: 40px; color: #ccc; font-size: 16px; cursor: pointer; user-select: none; transition: background-color 0.15s, color 0.15s; border-bottom: 3px solid transparent; }
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
        
        #canvas_joystick { touch-action: none; }
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
            <span style="color: #f0be00; font-weight: bold; font-size: 16px;">SCOTT ROBOT</span>
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
                    <div style="text-align: center; margin-bottom: 20px; width: 100%;">
                        <table id="radar" class="grid-3x3">
                            <tr><td><b>Date</b></td><td><b>Time</b></td><td><b>Lat</b></td><td><b>Lon</b></td></tr>
                            <tr><td><span id="gps-date">--/--/----</span></td><td><span id="gps-time">--:--:--</span></td><td><span id="gps-lat">--</span></td><td><span id="gps-lon">--</span></td></tr>
                            <tr><td><b>Dist:</b> <span id="distance">--</span> cm</td><td><b>Power:</b> <span id="table-speed">0</span> %</td><td colspan="2"><b>Speed:</b> <span id="robot-speed">0.0</span> cm/s</td></tr>
                        </table>
                    </div>
                    <canvas id="canvas_joystick"></canvas>
                    <div style="display: flex; width: 100%; justify-content: space-around; align-items: center; margin-top: 15px;">
                        <div style="text-align: center;"><button id="btn-led-linha" onclick="toggleLed('linha')">Line</button><div style="color: gray; margin-top: 5px; font-size: 10px;" id="led-status-linha">OFF</div></div>
                        <div style="text-align: center;"><button id="btn-led-explora" onclick="toggleLed('explora')">Explore</button><div style="color: gray; margin-top: 5px; font-size: 10px;" id="led-status-explora">OFF</div></div>
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
            <div class="setup-box" style="max-width: 360px;">
                <span class="setup-title">Navegação por Waypoints</span>
                <div id="wp-current-loc" style="text-align: center; font-weight: bold; color: #2196F3; margin-bottom: 10px; font-size: 15px;">Lat: -- | Lon: --</div>
                
                <table style="width: 100%; text-align: center; border-collapse: collapse; margin-bottom: 15px; font-size: 13px;">
                    <tr style="background:#eee; height:30px;"><th>ID</th><th>Lat</th><th>Lon</th><th>Ação</th></tr>
                    <tr><td>1</td><td><input type="number" id="wp_lat_1" class="input" style="width:85%; padding:4px; height:24px;"></td><td><input type="number" id="wp_lon_1" class="input" style="width:85%; padding:4px; height:24px;"></td><td><button class="btn btn-warning" style="height:28px; padding:0; width:45px; margin:0;" onclick="setWp(1)">Set</button></td></tr>
                    <tr><td>2</td><td><input type="number" id="wp_lat_2" class="input" style="width:85%; padding:4px; height:24px;"></td><td><input type="number" id="wp_lon_2" class="input" style="width:85%; padding:4px; height:24px;"></td><td><button class="btn btn-warning" style="height:28px; padding:0; width:45px; margin:0;" onclick="setWp(2)">Set</button></td></tr>
                    <tr><td>3</td><td><input type="number" id="wp_lat_3" class="input" style="width:85%; padding:4px; height:24px;"></td><td><input type="number" id="wp_lon_3" class="input" style="width:85%; padding:4px; height:24px;"></td><td><button class="btn btn-warning" style="height:28px; padding:0; width:45px; margin:0;" onclick="setWp(3)">Set</button></td></tr>
                    <tr><td>4</td><td><input type="number" id="wp_lat_4" class="input" style="width:85%; padding:4px; height:24px;"></td><td><input type="number" id="wp_lon_4" class="input" style="width:85%; padding:4px; height:24px;"></td><td><button class="btn btn-warning" style="height:28px; padding:0; width:45px; margin:0;" onclick="setWp(4)">Set</button></td></tr>
                    <tr><td>5</td><td><input type="number" id="wp_lat_5" class="input" style="width:85%; padding:4px; height:24px;"></td><td><input type="number" id="wp_lon_5" class="input" style="width:85%; padding:4px; height:24px;"></td><td><button class="btn btn-warning" style="height:28px; padding:0; width:45px; margin:0;" onclick="setWp(5)">Set</button></td></tr>
                </table>

                <div style="display:flex; justify-content: space-around; margin-bottom: 15px;">
                   <button id="btn-wp-queda" onclick="toggleWpSafety('queda')" style="background-color: #f0be00; border:none; padding:8px 12px; border-radius:5px; font-weight:bold;">Queda: ON</button>
                   <button id="btn-wp-colisao" onclick="toggleWpSafety('colisao')" style="background-color: #f0be00; border:none; padding:8px 12px; border-radius:5px; font-weight:bold;">Colisão: ON</button>
                </div>

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
                <div style="display:flex; justify-content:center; gap:14px; margin-bottom:14px; font-size:14px;">
                    <label><input type="radio" name="lineType" value="auto" checked> Auto</label>
                    <label><input type="radio" name="lineType" value="escura"> Linha Escura</label>
                    <label><input type="radio" name="lineType" value="clara"> Linha Clara</label>
                </div>
                <button type="button" class="btn btn-warning" onclick="calibrarSensores()">Calibrate Sensors</button>
                <div id="statusCalibracao" style="margin-top: 10px; font-weight: bold; text-align:center; color: #4CAF50;"></div>
            </div>
            <div class="setup-box">
                <span class="setup-title">Connect to Wi-Fi</span>
                <div class="input-group"><div class="input-group-addon">SSID</div><input type="text" id="wifi_ssid" placeholder="Network name" class="input"></div>
                <div class="input-group"><div class="input-group-addon">Pass</div><input type="password" id="wifi_pass" placeholder="Network password" class="input"></div>
                <button type="button" class="btn btn-warning" onclick="send_wifi()">Connect</button>
                <div id="statusWifi" style="margin-top: 10px; font-weight: bold; text-align:center; color: #666;"></div>
            </div>
        </div>
    </div>
    
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" defer></script>
    <script>
        function safelySend(dataObj) { if (connection && connection.readyState === WebSocket.OPEN) { connection.send(JSON.stringify(dataObj)); } }
        function calibrarSensores() {
            document.getElementById('statusCalibracao').innerText = "Calibrating...";
            document.getElementById('statusCalibracao').style.color = "#f0be00";
            let tipo = document.querySelector('input[name="lineType"]:checked').value;
            let payload = { cmd: "calibrate_line" };
            if (tipo === "escura") payload.forcar_escura = true;
            else if (tipo === "clara") payload.forcar_escura = false;
            safelySend(payload);
        }

        function send_wifi() {
            let ssid = document.getElementById("wifi_ssid").value.trim();
            let pass = document.getElementById("wifi_pass").value;
            if (!ssid) { alert("Digite o nome da rede (SSID)."); return; }
            document.getElementById('statusWifi').innerText = "Conectando...";
            document.getElementById('statusWifi').style.color = "#f0be00";
            safelySend({ wifi_ssid: ssid, wifi_pass: pass });
        }

        let lat = null, lon = null, lastMapUpdate = 0, map = null, marcador = null;
        let wp_queda = true, wp_colisao = true;

        function toggleWpSafety(tipo) {
            if(tipo === 'queda') { wp_queda = !wp_queda; document.getElementById("btn-wp-queda").innerText = "Queda: " + (wp_queda?"ON":"OFF"); document.getElementById("btn-wp-queda").style.backgroundColor = wp_queda?"#f0be00":"#ccc"; }
            if(tipo === 'colisao') { wp_colisao = !wp_colisao; document.getElementById("btn-wp-colisao").innerText = "Colisão: " + (wp_colisao?"ON":"OFF"); document.getElementById("btn-wp-colisao").style.backgroundColor = wp_colisao?"#f0be00":"#ccc"; }
            safelySend({wp_queda: wp_queda, wp_colisao: wp_colisao});
        }

        function setWp(id) {
            if (lat !== null && lon !== null) {
                document.getElementById("wp_lat_" + id).value = lat.toFixed(6);
                document.getElementById("wp_lon_" + id).value = lon.toFixed(6);
            } else {
                alert("Aguardando sinal válido de GPS do robô.");
            }
        }

        function sendWaypoint() {
            let list = [];
            for(let i = 1; i <= 5; i++) {
                let lt = parseFloat(document.getElementById("wp_lat_" + i).value);
                let ln = parseFloat(document.getElementById("wp_lon_" + i).value);
                if(!isNaN(lt) && !isNaN(ln)) { list.push({lat: lt, lon: ln}); }
            }
            if(list.length > 0) {
                safelySend({waypoint: true, list: list, wp_queda: wp_queda, wp_colisao: wp_colisao});
                document.getElementById("waypoint_status").innerText = "Status: Iniciando rota...";
                document.getElementById("waypoint_status").style.color = "#4CAF50";
            } else {
                alert("Preencha ao menos 1 coordenada válida na tabela.");
            }
        }

        function stopWaypoint() {
            safelySend({ waypoint: false });
            document.getElementById("waypoint_status").innerText = "Status: Cancelado / Inativo";
            document.getElementById("waypoint_status").style.color = "#f44336";
        }

        function checkAndInitMap() {
            if (typeof L === 'undefined') { document.getElementById('menu-map').style.display = 'none'; showTab('display'); return; }
            if (!map) {
                try {
                    let mapLat = (lat !== null) ? lat : -22.85817; let mapLon = (lon !== null) ? lon : -46.97656;
                    document.getElementById('map').style.display = 'block';
                    map = L.map('map').setView([mapLat, mapLon], 15);
                    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '© OSM' }).addTo(map);
                    marcador = L.marker([mapLat, mapLon]).addTo(map);
                } catch(e) { document.getElementById('menu-map').style.display = 'none'; showTab('display'); }
            } else if (typeof map.invalidateSize === 'function') { setTimeout(function(){ map.invalidateSize(); }, 100); }
        }

        var connection = new WebSocket(`ws://${window.location.hostname}/ws`);
        connection.onopen = function () { console.log('Connection opened'); };
        connection.onmessage = function (e) {
            const data = JSON.parse(e.data);
            
            if (data["vbat"]) {
                let vbat_val = data["vbat"];
                document.getElementById("vbat").innerText = (vbat_val / 1000).toFixed(1);
                var lbat = (vbat_val * 100 / 9000).toFixed(0);
                if(lbat > 100) lbat = 100; if(lbat < 2) lbat = 2;
                    var lbat_el = document.getElementById("lbat");
                if(lbat_el) {
                    lbat_el.style.width = lbat + '%';
                    lbat_el.style.backgroundColor = (lbat < 20) ? "#F00" : (lbat < 70) ? "orange" : "#0F0";
                }
            } 

            if (data["distancia"]) document.getElementById("distance").innerText = (data["distancia"]).toFixed(0);
            if (data["robot_speed"] !== undefined) document.getElementById("robot-speed").innerText = data["robot_speed"].toFixed(1);
            if (data["fall"] !== undefined) {
                let fallEl = document.getElementById("fall-status-text");
                fallEl.innerText = data["fall"] ? "Fall Detected" : "";
                fallEl.style.color = data["fall"] ? "#f44336" : "#ccc";
            }
            if (data["stuck"] !== undefined) {
                let stuckEl = document.getElementById("stuck-status-text");
                stuckEl.innerText = data["stuck"] ? "Stuck!" : "";
                stuckEl.style.color = data["stuck"] ? "#f44336" : "#ccc";
            }
            if (data["line_status"]) {
                let lineEl = document.getElementById("stuck-status-text");
                lineEl.innerText = data["line_status"];
                lineEl.style.color = data["line_status"].toLowerCase().includes("reencontrada") ? "#4CAF50" : "#FFA500";
            }
            if (data["waypoint_status"]) {
                let textStatus = data["waypoint_status"];
                document.getElementById("waypoint_status").innerText = "Status: " + textStatus;
                document.getElementById("waypoint_status").style.color = textStatus.includes("Cancelado") ? "#f44336" : (textStatus.includes("Bloqueado") ? "#FFA500" : "#4CAF50");
            }
            if (data["lat"] !== undefined && data["lat"] !== null) {
                lat = data["lat"]; lon = data["lon"];
                document.getElementById("gps-lat").innerText = lat.toFixed(6);
                document.getElementById("gps-lon").innerText = lon.toFixed(6);
                document.getElementById("wp-current-loc").innerText = "GPS Atual - Lat: " + lat.toFixed(6) + " | Lon: " + lon.toFixed(6);
                let agora = Date.now();
                if (agora - lastMapUpdate >= 3000 && map && marcador) { marcador.setLatLng([lat, lon]); map.setView([lat, lon]); lastMapUpdate = agora; }
            }
            if (data["date"] !== undefined) document.getElementById("gps-date").innerText = data["date"];
            if (data["time"] !== undefined) document.getElementById("gps-time").innerText = data["time"];
            
            if (data["status"] === "calibrado") {
                let corDetectada = data["escura"] ? "Linha Escura" : "Linha Clara";
                document.getElementById('statusCalibracao').innerText = "OK! Limiar: " + data["limiar"] + " (" + corDetectada + ")";
                document.getElementById('statusCalibracao').style.color = "#4CAF50";
            }
            if (data["wifi_status"]) {
                document.getElementById('statusWifi').innerText = data["wifi_status"];
                document.getElementById('statusWifi').style.color = data["wifi_status"].toLowerCase().includes("falh") ? "#e53935" : "#4CAF50";
            }
            if (data["status"] === "calibracao_falhou") {
                let motivoTexto = (data["motivo"] === "fundo_ambiguo")
                    ? "leitura inicial ambigua"
                    : "contraste insuficiente";
                document.getElementById('statusCalibracao').innerText = "Calibration rejected! (" + motivoTexto + ") Deixe os 2 sensores sobre a fita, centralizados, e tente de novo.";
                document.getElementById('statusCalibracao').style.color = "#e53935";
            }
        };

        function showTab(tab) {
            document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
            document.querySelectorAll('.menu-item').forEach(el => el.classList.remove('active'));
            document.getElementById('tab-' + tab).classList.add('active');
            document.getElementById('menu-' + tab).classList.add('active');
            if (tab === 'display' && typeof resize === 'function') resize();
            if (tab === 'map') checkAndInitMap();
        }

        var led_state = { linha: false, explora: false };
        function toggleLed(tipo){
            led_state[tipo] = !led_state[tipo];
            if (led_state[tipo]) {
                Object.keys(led_state).forEach(k => {
                    if(k !== tipo) { led_state[k] = false; document.getElementById("btn-led-" + k).classList.remove("on"); document.getElementById("led-status-" + k).innerText = "OFF"; }
                });
            }
            var btn = document.getElementById("btn-led-" + tipo); var status = document.getElementById("led-status-" + tipo);
            if(led_state[tipo]){ btn.classList.add("on"); status.innerText = "ON"; } else { btn.classList.remove("on"); status.innerText = "OFF"; }
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
            ['mousedown'].forEach(evt => canvas_joystick.addEventListener(evt, startDrawing));
            ['touchstart'].forEach(evt => canvas_joystick.addEventListener(evt, startDrawing, { passive: false }));
            ['mouseup'].forEach(evt => canvas_joystick.addEventListener(evt, stopDrawing));
            ['touchend','touchcancel'].forEach(evt => canvas_joystick.addEventListener(evt, stopDrawing, { passive: false }));
            ['mousemove'].forEach(evt => canvas_joystick.addEventListener(evt, Draw));
            ['touchmove'].forEach(evt => canvas_joystick.addEventListener(evt, Draw, { passive: false }));
            window.addEventListener('resize', resize);
        }
        if (document.readyState === 'loading') { document.addEventListener('DOMContentLoaded', () => { initJoystick(); }); } else { initJoystick(); }
            
        function resize() {
            if (!ctx_joystick) return;
            width = (window.innerWidth > window.innerHeight) ? window.innerHeight : window.innerWidth;
            radius = width_to_radius_ratio * width; height = radius * radius_factor * 2 + 100;
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
            led_state.linha = false; led_state.explora = false;
            ['linha', 'explora'].forEach(k => { document.getElementById("btn-led-" + k).classList.remove("on"); document.getElementById("led-status-" + k).innerText = "OFF"; });
            if(document.getElementById("waypoint_status").innerText.includes("Iniciando") || document.getElementById("waypoint_status").innerText.includes("Navegando")) {
                document.getElementById("waypoint_status").innerText = "Status: Cancelado pelo Joystick";
                document.getElementById("waypoint_status").style.color = "#f44336";
            }
        }

        function startDrawing(event) { if (event.cancelable) event.preventDefault(); paint = true; document.activeElement.blur(); resetModosAutonomosVisual(); getPosition_joystick(event); if (in_circle()) { joystick(coord.x, coord.y); Draw(event); } }
        function stopDrawing(event) { if (event && event.cancelable) event.preventDefault(); paint = false; joystick(origin_joystick.x, origin_joystick.y); document.getElementById("speed").innerText = 0; document.getElementById("table-speed").innerText = 0; if (movimento == 1) { safelySend({"velocidade":0, "angulo":0}); movimento = 0; } resetModosAutonomosVisual(); }

        function Draw(event) {
            if (paint) {
                if (event.cancelable) event.preventDefault();
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
void executa_desvio_obstaculo_linha(void);
void explora_casa(void);
void navega_waypoints(void);
void calibrarSensoresLinha(bool forcarPolaridade, bool valorForcado);
void verificarSegurancaBateria(void);
void processar_mapeamento_e_telemetria(void);
void carregar_mapa_salvo(void);

// Funções para controle com rastreamento para Odometria
void mover_motores(int v_esq, int v_dir);
void parar_motores();
void atualizar_odometria_fusao();

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
  delay(300);
  Serial.println("=====================================");
  Serial.println("FIRMWARE Scott_robot - Sem Encoders (Estimativa Tempo/GPS)");
  Serial.println("=====================================");
  
  SerialGPS.setRxBufferSize(1024);
  SerialGPS.begin(9600, SERIAL_8N1, PINO_GPS_RX, PINO_GPS_TX);
  
  pinMode(PINO_LED, OUTPUT);
  digitalWrite(PINO_LED, LOW);

  pinMode(PINO_HCSR04_ECHO, INPUT);
  pinMode(PINO_HCSR04_TRIGGER, OUTPUT);
  digitalWrite(PINO_HCSR04_TRIGGER, LOW);

  pinMode(SENSOR_LINHA_ESQUERDO, INPUT);
  pinMode(SENSOR_LINHA_DIREITO, INPUT);

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
  Kp = 40.0; 
  Ki = SPIFFS.getFloat(ENDERECOS_SPIFFS[2], 0.2);
  Kd = SPIFFS.getFloat(ENDERECOS_SPIFFS[3], 0.4);
  limiarLinha = SPIFFS.getInt("limiar", 3000); 
  linha_escura = SPIFFS.getBool("linha_escura", true);
  String saved_wifi_ssid = SPIFFS.getString("wifi_ssid_sta", "");
  String saved_wifi_pass = SPIFFS.getString("wifi_pass_sta", "");
  SPIFFS.end();

  // Tenta reconectar a uma rede Wi-Fi salva anteriormente pelo dashboard (opcional)
  if (saved_wifi_ssid.length() > 0) {
      WiFi.mode(WIFI_AP_STA); // AP do robo continua ativo mesmo se essa conexao falhar
      WiFi.begin(saved_wifi_ssid.c_str(), saved_wifi_pass.c_str());
      uint32_t inicio_wifi_boot = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - inicio_wifi_boot < 8000) { delay(200); }
      if (WiFi.status() != WL_CONNECTED) { WiFi.mode(WIFI_AP); }
  }

  memset(occupancyMap, 0, sizeof(occupancyMap));
}

void loop() {
  ws.cleanupClients();
  
  while (SerialGPS.available() > 0) { gps.encode(SerialGPS.read()); }
  atualizar_sensor_ultrassonico();
  processar_mapeamento_e_telemetria();

  static unsigned long tempo_odo_fusao = 0;
  if (millis() - tempo_odo_fusao > 50) {
      atualizar_odometria_fusao();
      tempo_odo_fusao = millis();
  }

  static uint32_t ultimoCheckBateria = 0;
  if (millis() - ultimoCheckBateria > 2000) { ultimoCheckBateria = millis(); verificarSegurancaBateria(); }

  // --- CONEXAO A REDE WI-FI EXTERNA (nao bloqueia o restante do loop) ---
  if (change_wifi_flag) {
      change_wifi_flag = false;
      WiFi.mode(WIFI_AP_STA); // mantem o AP do robo ativo enquanto tenta conectar como estacao
      WiFi.begin(new_wifi_ssid.c_str(), new_wifi_pass.c_str());
      wifi_conectando = true;
      tempo_inicio_wifi = millis();
      if (ws.count() > 0) ws.textAll("{\"wifi_status\":\"Conectando a rede...\"}");
  }

  if (wifi_conectando) {
      if (WiFi.status() == WL_CONNECTED) {
          wifi_conectando = false;
          String ip = WiFi.localIP().toString();
          char msgWifi[96];
          snprintf(msgWifi, sizeof(msgWifi), "{\"wifi_status\":\"Conectado! IP: %s\"}", ip.c_str());
          if (ws.count() > 0) ws.textAll(msgWifi);

          SPIFFS.begin(DIRETORIO_SPIFFS, false);
          SPIFFS.putString("wifi_ssid_sta", new_wifi_ssid);
          SPIFFS.putString("wifi_pass_sta", new_wifi_pass);
          SPIFFS.end();
      } else if (millis() - tempo_inicio_wifi > TIMEOUT_WIFI_CONNECT_MS) {
          wifi_conectando = false;
          WiFi.mode(WIFI_AP); // desiste e volta a operar apenas como ponto de acesso
          if (ws.count() > 0) ws.textAll("{\"wifi_status\":\"Falha ao conectar - verifique SSID/senha\"}");
      }
  }

  if (modoSegurancaBateria) {
      modo_linha = false; modo_explora = false; modo_waypoints = false;
      parar_motores();
  } else {
      if (modo_linha) segue_linha();
      else if (modo_explora) explora_casa();
      else if (modo_waypoints) navega_waypoints();
  }

  if (millis() > timeout_vbat) {
    if (ws.count() > 0) {
      uint32_t tensao = vbat.readVoltage();
      JsonDocument json; json[ALIAS_VBAT] = tensao;
      size_t msg_comp = measureJson(json); char msg[msg_comp + 1];
      serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0; ws.textAll(msg, msg_comp);
    }
    timeout_vbat = millis() + TEMPO_ATUALIZACAO_VBAT;
  }
  
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
            json["lat"] = flat; 
            json["lon"] = flon; 
        }
        json["date"] = date_str; json["time"] = time_str;
        
        // --- VELOCIDADE DO GPS ---
        float speed_kmph = gps.f_speed_kmph();
        robot_speed_cms = (speed_kmph == TinyGPS::GPS_INVALID_F_SPEED) ? 0.0 : (speed_kmph * 27.7778);
        json["robot_speed"] = robot_speed_cms;
        
        size_t msg_comp = measureJson(json); char msg[msg_comp + 1];
        serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0; ws.textAll(msg, msg_comp);
    }
    timeout_gps = millis() + 1000;
  }
  delay(1); 
}

// --------------------------------------------------
void processar_mapeamento_e_telemetria(void) {
    if (distancia > 0 && distancia < MAX_SENSOR_DIST_CM) {
        float obs_x = robot_local_x_cm + distancia * cos(theta_rad);
        float obs_y = robot_local_y_cm + distancia * sin(theta_rad);

        int grid_x = (int)(obs_x / MAP_RESOLUTION_CM) + (MAP_WIDTH / 2);
        int grid_y = (int)(obs_y / MAP_RESOLUTION_CM) + (MAP_HEIGHT / 2);

        if (grid_x >= 0 && grid_x < MAP_WIDTH && grid_y >= 0 && grid_y < MAP_HEIGHT) {
            occupancyMap[grid_x][grid_y] = 2; 
        }
    }

    if (millis() > timeout_map_ws) {
        if (ws.count() > 0) {
            JsonDocument json;
            json["map_raw_x"] = robot_local_x_cm;
            json["map_raw_y"] = robot_local_y_cm;
            json["map_raw_theta"] = theta_rad;
            json["map_raw_dist"] = distancia;
            
            size_t msg_comp = measureJson(json); 
            char msg[msg_comp + 1]; 
            serializeJson(json, msg, (msg_comp + 1)); 
            msg[msg_comp] = 0; 
            ws.textAll(msg, msg_comp); 
        }
        timeout_map_ws = millis() + 100;
    }
}

// --------------------------------------------------
void carregar_mapa_salvo() {
    if (!LittleFS.begin(true)) { return; }
    File file = LittleFS.open("/mapa_salvo.json", "r");
    if (!file) { return; }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    if (error) { file.close(); return; }

    JsonObject obj = doc.as<JsonObject>();
    for (JsonPair kv : obj) {
        String chave = kv.key().c_str();
        int virgula = chave.indexOf(',');
        int x = chave.substring(0, virgula).toInt();
        int y = chave.substring(virgula + 1).toInt();
        int score = kv.value().as<int>();

        int grid_x = x + (MAP_WIDTH / 2);
        int grid_y = y + (MAP_HEIGHT / 2);

        if (grid_x >= 0 && grid_x < MAP_WIDTH && grid_y >= 0 && grid_y < MAP_HEIGHT) {
            occupancyMap[grid_x][grid_y] = (score >= 70) ? 2 : ((score <= 30) ? 1 : 0);
        }
    }
    file.close();
}

// --------------------------------------------------

void atualizar_odometria_fusao() {
    static unsigned long tempo_anterior = millis();
    unsigned long agora = millis();
    float dt = (agora - tempo_anterior) / 1000.0;
    tempo_anterior = agora;

    // Sem encoders, fazemos odometria open-loop estimada pelos comandos do motor
    // para tentar manter o Grid Mapping mais ou menos funcional localmente
    float dist_centro = 0.0;
    float delta_theta = 0.0;

    // Estimativas de 30 cm/s para avanço e 1.5 rad/s para giro (ajuste se necessário)
    if (sinal_esq == 1 && sinal_dir == 1) { dist_centro = 30.0 * dt; }
    else if (sinal_esq == -1 && sinal_dir == -1) { dist_centro = -30.0 * dt; }
    else if (sinal_esq == -1 && sinal_dir == 1) { delta_theta = 1.5 * dt; }
    else if (sinal_esq == 1 && sinal_dir == -1) { delta_theta = -1.5 * dt; }

    theta_rad += delta_theta;
    while (theta_rad >= TWO_PI) theta_rad -= TWO_PI; 
    while (theta_rad < 0) theta_rad += TWO_PI;
    
    float delta_x_cm = dist_centro * cos(theta_rad); 
    float delta_y_cm = dist_centro * sin(theta_rad); 
    
    robot_local_x_cm += delta_x_cm;
    robot_local_y_cm += delta_y_cm;

    float flat, flon; unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    static float ultimo_flat_processado = 0.0;
    
    if (flat != TinyGPS::GPS_INVALID_F_ANGLE) {
        if (!odometria_inicializada) {
            estimativa_lat = flat; estimativa_lon = flon; ultimo_flat_processado = flat;
            odometria_inicializada = true; heading_gps_valido = false; registrou_inicio_calibracao = false;
        } else {
            if (flat != ultimo_flat_processado && age < 3000) { 
                estimativa_lat = flat;
                estimativa_lon = flon; 
                ultimo_flat_processado = flat;
            }
        }
    }
}

// --------------------------------------------------
void configurar_servidor_web(void) {
  ws.onEvent(onEvent); server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", index_html); });
}

void controlar_motor(uint8_t *data, size_t length){
  JsonDocument json; deserializeJson(json, data, length);
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

    if (strstr(reinterpret_cast<char *>(data), "calibrate_line") != nullptr) {
      JsonDocument json; deserializeJson(json, data, length);
      bool forcarPolaridade = json.containsKey("forcar_escura");
      bool valorForcado = forcarPolaridade ? json["forcar_escura"].as<bool>() : true;
      calibrarSensoresLinha(forcarPolaridade, valorForcado);
    }
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_WAYPOINT) != nullptr) {
      JsonDocument json; deserializeJson(json, data, length);
      if(json.containsKey("wp_queda")) wp_evita_queda = json["wp_queda"].as<bool>();
      if(json.containsKey("wp_colisao")) wp_evita_colisao = json["wp_colisao"].as<bool>();
      modo_waypoints = json[ALIAS_WAYPOINT].as<bool>();
      if (modo_waypoints) {
          modo_linha = false; modo_explora = false;
          estadoWay = WAY_LIVRE; heading_gps_valido = false; registrou_inicio_calibracao = false;
          total_waypoints = 0; waypoint_atual_idx = 0; giros_consecutivos_way = 0;
          
          JsonArray list = json["list"].as<JsonArray>();
          for(JsonVariant v : list) {
              if (total_waypoints < 5) {
                  waypoints_lat[total_waypoints] = v["lat"].as<float>();
                  waypoints_lon[total_waypoints] = v["lon"].as<float>();
                  total_waypoints++;
              }
          }
      } else {
          parar_motores(); total_waypoints = 0;
      }
    }
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_WIFI_SSID) != nullptr) {
        JsonDocument json; deserializeJson(json, data, length);
        if (json.containsKey(ALIAS_WIFI_SSID)) {
            new_wifi_ssid = json[ALIAS_WIFI_SSID].as<String>();
            new_wifi_pass = json.containsKey(ALIAS_WIFI_PASS) ? json[ALIAS_WIFI_PASS].as<String>() : "";
            change_wifi_flag = true;
        }
    }
    else if (strstr(reinterpret_cast<char *>(data), "wp_queda") != nullptr) {
        JsonDocument json; deserializeJson(json, data, length);
        if(json.containsKey("wp_queda")) wp_evita_queda = json["wp_queda"].as<bool>();
        if(json.containsKey("wp_colisao")) wp_evita_colisao = json["wp_colisao"].as<bool>();
    }
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_LINHA) != nullptr || strstr(reinterpret_cast<char *>(data), ALIAS_EXPLORA) != nullptr) {
      JsonDocument json; deserializeJson(json, data, length);
      if (json.containsKey(ALIAS_LINHA)) modo_linha = json[ALIAS_LINHA].as<bool>();
      if (json.containsKey(ALIAS_EXPLORA)) modo_explora = json[ALIAS_EXPLORA].as<bool>();
      
      if (modo_linha) { modo_explora = false; modo_waypoints = false; erro = 0.0; I = 0.0; erro_anterior = 0.0; contador_parada = 0; leitura_esquerda_filtrada = 0.0; leitura_direita_filtrada = 0.0; pid_tempo_anterior = 0; estadoDesvioLinha = DESVIO_LINHA_LIVRE; tentativas_desvio_consecutivas = 0; }
      else if (modo_explora) { modo_linha = false; modo_waypoints = false; }

      if (!modo_linha && !modo_explora && !modo_waypoints) {
        parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE; estadoDesvioLinha = DESVIO_LINHA_LIVRE; ws.textAll("{\"fall\":false, \"stuck\":false}"); 
      }
    }
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_VELOCIDADE) != nullptr) {
      if (modo_linha || modo_explora || modo_waypoints) {
        modo_linha = false; modo_explora = false; modo_waypoints = false; total_waypoints = 0;
        parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE; estadoDesvioLinha = DESVIO_LINHA_LIVRE;
        ws.textAll("{\"fall\":false, \"stuck\":false, \"waypoint_status\":\"Cancelado pelo Joystick\"}"); 
      }
      controlar_motor(data, length);
    } 
  }
}

// --------------------------------------------------
int16_t ler_distancia(void) {
  int16_t leituras[3];
  for (int i = 0; i < 3; i++) {
    digitalWrite(PINO_HCSR04_TRIGGER, HIGH); delayMicroseconds(10); digitalWrite(PINO_HCSR04_TRIGGER, LOW);
    unsigned long duracao = pulseIn(PINO_HCSR04_ECHO, HIGH, 25000); leituras[i] = (duracao == 0) ? -1 : (duracao / 58); delay(5);
  }
  for (int i = 0; i < 2; i++) {
    for (int j = i + 1; j < 3; j++) {
      if (leituras[j] < leituras[i]) { int16_t temp = leituras[i]; leituras[i] = leituras[j]; leituras[j] = temp; }
    }
  }
  return leituras[1]; 
}

void atualizar_sensor_ultrassonico(void) {
  if (millis() > timeout_distancia) {
    int16_t leitura = ler_distancia();
    if (leitura > 0 && leitura < 400) { distancia = (leitura * 0.2) + (distancia * 0.8); }
    if (millis() > timeout_distancia_ws) {
        if (ws.count() > 0) {
          JsonDocument json; json[ALIAS_DISTANCIA] = (int)distancia;
          size_t msg_comp = measureJson(json); char msg[msg_comp + 1]; serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0; ws.textAll(msg, msg_comp);
        }
        timeout_distancia_ws = millis() + 250; 
    }
    timeout_distancia = millis() + TEMPO_ATUALIZACAO_DISTANCIA;
  }
}

// --------------------------------------------------
void segue_linha() {
  // --- Se já está executando a manobra de desvio, continua ela e sai ---
  if (estadoDesvioLinha != DESVIO_LINHA_LIVRE) {
      executa_desvio_obstaculo_linha();
      return;
  }

  // --- Detecção de obstáculo à frente durante o seguimento de linha ---
  bool obstaculo_frente = (distancia > 0) && (distancia <= DISTANCIA_OBSTACULO_LINHA);
  if (obstaculo_frente) {
      if (tentativas_desvio_consecutivas >= MAX_TENTATIVAS_DESVIO) {
          // Já tentou desviar várias vezes seguidas sem sucesso: para e avisa
          parar_motores();
          if (ws.count() > 0) ws.textAll("{\"line_status\":\"Obstaculo persistente - travado\", \"stuck\":true}");
          return;
      }

      parar_motores();
      delay(150);
      if (ws.count() > 0) ws.textAll("{\"line_status\":\"Obstaculo detectado - contornando\"}");

      sentido_desvio = 1; // contorna sempre pela direita (poderia alternar se preferir)
      mover_motores(VELOCIDADE_GIRO * sentido_desvio, -VELOCIDADE_GIRO * sentido_desvio);
      estadoDesvioLinha = DESVIO_GIRO_SAIDA;
      alvoTempoDesvio = TEMPO_GIRO_DESVIO_MS;
      tempo_inicio_desvio = millis();
      return;
  }

  leitura_esquerdo = analogRead(SENSOR_LINHA_ESQUERDO); 
  leitura_direito = analogRead(SENSOR_LINHA_DIREITO);

  if (leitura_esquerda_filtrada == 0.0 && leitura_direita_filtrada == 0.0) {
      leitura_esquerda_filtrada = leitura_esquerdo;
      leitura_direita_filtrada  = leitura_direito;
  } else {
      leitura_esquerda_filtrada = (ALPHA_FILTRO_LINHA * leitura_esquerdo) + ((1.0 - ALPHA_FILTRO_LINHA) * leitura_esquerda_filtrada);
      leitura_direita_filtrada  = (ALPHA_FILTRO_LINHA * leitura_direito)  + ((1.0 - ALPHA_FILTRO_LINHA) * leitura_direita_filtrada);
  }

  float sinalPolaridade = linha_escura ? 1.0 : -1.0;
  erro = sinalPolaridade * (leitura_direita_filtrada - leitura_esquerda_filtrada) / FATOR_NORMALIZACAO_LINHA;
  if (erro > 2.0) erro = 2.0;
  if (erro < -2.0) erro = -2.0;

  bool erro_saturado = (fabs(erro) >= 1.9);
  bool ambos_fora_da_fita = linha_escura
      ? (leitura_esquerda_filtrada < limiarLinha && leitura_direita_filtrada < limiarLinha)
      : (leitura_esquerda_filtrada > limiarLinha && leitura_direita_filtrada > limiarLinha);

  if (erro_saturado || ambos_fora_da_fita) {
      contador_parada++;
  } else {
      contador_parada = 0;
  }

  if (contador_parada >= CONTAGEM_MAXIMA) {
      parar_motores();
      P = 0; I = 0; D = 0; erro = 0.0; erro_anterior = 0.0;
      contador_parada = CONTAGEM_MAXIMA;
  } else if (ambos_fora_da_fita) {
      mover_motores(VELOCIDADE, VELOCIDADE);
  } else {
      calcula_PID();
      mover_motores(velocidade_esquerda, velocidade_direita);
  }
  delay(espera);
}

// --------------------------------------------------
// Executa, passo a passo e sem bloquear o loop (baseada em millis()), a manobra
// de contornar um obstáculo detectado durante o modo_linha e retornar à linha.
void executa_desvio_obstaculo_linha() {

  // Durante a etapa de "procura", verifica a cada iteração se a linha já foi reencontrada
  if (estadoDesvioLinha == DESVIO_PROCURA_LINHA) {
      int leituraEsq = analogRead(SENSOR_LINHA_ESQUERDO);
      int leituraDir = analogRead(SENSOR_LINHA_DIREITO);
      bool linha_encontrada = linha_escura
          ? (leituraEsq >= limiarLinha || leituraDir >= limiarLinha)
          : (leituraEsq <= limiarLinha || leituraDir <= limiarLinha);

      if (linha_encontrada) {
          parar_motores();
          estadoDesvioLinha = DESVIO_LINHA_LIVRE;
          tentativas_desvio_consecutivas = 0;
          // Reseta o PID para retomar o seguimento de forma suave
          erro = 0.0; I = 0.0; erro_anterior = 0.0; contador_parada = 0;
          leitura_esquerda_filtrada = 0.0; leitura_direita_filtrada = 0.0; pid_tempo_anterior = 0;
          if (ws.count() > 0) ws.textAll("{\"line_status\":\"Linha reencontrada\"}");
          return;
      }
  }

  // Durante os avanços, se surgir outro obstáculo bem próximo, pausa até ele se afastar —
  // mas com um limite de tempo, para nunca travar o robô para sempre caso o objeto não se
  // afaste (ex.: o próprio obstáculo original ainda dentro do alcance do sensor após o giro).
  static uint32_t tempo_inicio_pausa_seguranca = 0;
  bool bloqueio_proximo = (estadoDesvioLinha == DESVIO_AVANCA_LADO || estadoDesvioLinha == DESVIO_AVANCA_PARALELO) &&
      distancia > 0 && distancia <= (DISTANCIA_OBSTACULO_LINHA / 3);

  if (bloqueio_proximo) {
      if (tempo_inicio_pausa_seguranca == 0) tempo_inicio_pausa_seguranca = millis();
      parar_motores();

      const uint32_t TIMEOUT_PAUSA_SEGURANCA_MS = 1500;
      if (millis() - tempo_inicio_pausa_seguranca > TIMEOUT_PAUSA_SEGURANCA_MS) {
          // Ficou bloqueado por tempo demais: desiste desta tentativa de desvio,
          // devolve o controle a segue_linha() para reavaliar (ou tentar de novo)
          estadoDesvioLinha = DESVIO_LINHA_LIVRE;
          tentativas_desvio_consecutivas++;
          tempo_inicio_pausa_seguranca = 0;
          if (ws.count() > 0) ws.textAll("{\"line_status\":\"Desvio bloqueado - reavaliando\", \"stuck\":true}");
      }
      return;
  } else {
      tempo_inicio_pausa_seguranca = 0;
  }

  // Ainda dentro do tempo da etapa atual: nada a fazer, continua o movimento em curso
  if (millis() - tempo_inicio_desvio < alvoTempoDesvio) return;

  parar_motores();
  delay(100);

  switch (estadoDesvioLinha) {
      case DESVIO_GIRO_SAIDA:
          // Já girou para fora da linha; agora avança de lado para passar da lateral do objeto
          mover_motores(VELOCIDADE_EXPLORACAO, VELOCIDADE_EXPLORACAO);
          estadoDesvioLinha = DESVIO_AVANCA_LADO;
          alvoTempoDesvio = TEMPO_AVANCO_LADO_MS;
          tempo_inicio_desvio = millis();
          break;

      case DESVIO_AVANCA_LADO:
          // Gira de volta, ficando de frente paralela à linha original (objeto ao lado)
          mover_motores(-VELOCIDADE_GIRO * sentido_desvio, VELOCIDADE_GIRO * sentido_desvio);
          estadoDesvioLinha = DESVIO_GIRO_RETORNO;
          alvoTempoDesvio = TEMPO_GIRO_DESVIO_MS;
          tempo_inicio_desvio = millis();
          break;

      case DESVIO_GIRO_RETORNO:
          // Avança paralelo à linha, cobrindo o comprimento do objeto
          mover_motores(VELOCIDADE_EXPLORACAO, VELOCIDADE_EXPLORACAO);
          estadoDesvioLinha = DESVIO_AVANCA_PARALELO;
          alvoTempoDesvio = TEMPO_AVANCO_PARALELO_MS;
          tempo_inicio_desvio = millis();
          break;

      case DESVIO_AVANCA_PARALELO:
          // Gira de volta em direção à linha
          mover_motores(-VELOCIDADE_GIRO * sentido_desvio, VELOCIDADE_GIRO * sentido_desvio);
          estadoDesvioLinha = DESVIO_GIRO_LINHA;
          alvoTempoDesvio = TEMPO_GIRO_DESVIO_MS;
          tempo_inicio_desvio = millis();
          break;

      case DESVIO_GIRO_LINHA:
          // Avança devagar procurando reencontrar a linha (verificado no topo da função)
          mover_motores(VELOCIDADE, VELOCIDADE);
          estadoDesvioLinha = DESVIO_PROCURA_LINHA;
          alvoTempoDesvio = TEMPO_MAX_PROCURA_LINHA_MS;
          tempo_inicio_desvio = millis();
          break;

      case DESVIO_PROCURA_LINHA:
          // Esgotou o tempo máximo de procura sem reencontrar a linha
          parar_motores();
          estadoDesvioLinha = DESVIO_LINHA_LIVRE;
          tentativas_desvio_consecutivas++;
          if (ws.count() > 0) ws.textAll("{\"line_status\":\"Linha nao encontrada apos desvio\", \"stuck\":true}");
          break;

      default:
          estadoDesvioLinha = DESVIO_LINHA_LIVRE;
          break;
  }
}

// --------------------------------------------------
void explora_casa() {
    bool obstaculo_frente = ((distancia > 0) && (distancia <= DISTANCIA_EXPLORACAO));
    bool queda_detectada = (analogRead(SENSOR_LINHA_ESQUERDO) > LIMIAR_QUEDA) || (analogRead(SENSOR_LINHA_DIREITO) > LIMIAR_QUEDA);
    static bool status_queda_ui_explora = false;

    if (estadoExplora == EXPLORA_PARADO) { parar_motores(); return; }

    if (estadoExplora != EXPLORA_LIVRE) {
        
        // Verifica conclusão do segmento baseado apenas no temporizador
        if (millis() - tempo_inicio_manobra_explora >= alvoTempoExplora) {
            if (estadoExplora == EXPLORA_RE) {
                parar_motores(); delay(250); 
                sentido_giro_atual = (random(0, 2) == 0) ? 1 : -1;
                
                mover_motores(-VELOCIDADE_MAXIMA * sentido_giro_atual, VELOCIDADE_MAXIMA * sentido_giro_atual);
                delay(DURACAO_KICK_ANTISTALL_MS);
                mover_motores(-VELOCIDADE_GIRO * sentido_giro_atual, VELOCIDADE_GIRO * sentido_giro_atual); 
                
                estadoExplora = EXPLORA_GIRO; 
                alvoTempoExplora = TEMPO_GIRO_90_MS;
                tempo_inicio_manobra_explora = millis(); 
            }
            else if (estadoExplora == EXPLORA_GIRO) {
                parar_motores(); delay(100); giros_consecutivos++;

                bool ainda_obstaculo = ((distancia > 0) && (distancia <= DISTANCIA_EXPLORACAO));
                bool ainda_queda = (analogRead(SENSOR_LINHA_ESQUERDO) > LIMIAR_QUEDA) || (analogRead(SENSOR_LINHA_DIREITO) > LIMIAR_QUEDA);

                if (ainda_obstaculo || ainda_queda) {
                    if (giros_consecutivos >= MAX_GIROS_CONSECUTIVOS) {
                        estadoExplora = EXPLORA_PARADO; if (ws.count() > 0) ws.textAll("{\"stuck\":true}"); 
                    } else {
                        mover_motores(-VELOCIDADE_MAXIMA * sentido_giro_atual, VELOCIDADE_MAXIMA * sentido_giro_atual);
                        delay(DURACAO_KICK_ANTISTALL_MS);
                        mover_motores(-VELOCIDADE_GIRO * sentido_giro_atual, VELOCIDADE_GIRO * sentido_giro_atual); 
                        
                        alvoTempoExplora = TEMPO_GIRO_45_MS;
                        tempo_inicio_manobra_explora = millis();
                        if (ainda_queda && !status_queda_ui_explora) { if (ws.count() > 0) ws.textAll("{\"fall\":true}"); status_queda_ui_explora = true; }
                    }
                } else {
                    estadoExplora = EXPLORA_LIVRE; giros_consecutivos = 0;
                    if (status_queda_ui_explora) { if (ws.count() > 0) ws.textAll("{\"fall\":false}"); status_queda_ui_explora = false; }
                    mover_motores(VELOCIDADE_EXPLORACAO, VELOCIDADE_EXPLORACAO);
                }
            }
        }
        return;
    }

    if (queda_detectada || obstaculo_frente) {
        if (queda_detectada && !status_queda_ui_explora) { if (ws.count() > 0) ws.textAll("{\"fall\":true}"); status_queda_ui_explora = true; }
        parar_motores(); delay(250); mover_motores(-VELOCIDADE_EXPLORACAO, -VELOCIDADE_EXPLORACAO);
        estadoExplora = EXPLORA_RE; giros_consecutivos = 0;
        alvoTempoExplora = TEMPO_RE_EXPLORA_MS;
        tempo_inicio_manobra_explora = millis();
    } else {
        if (status_queda_ui_explora) { if (ws.count() > 0) ws.textAll("{\"fall\":false}"); status_queda_ui_explora = false; }
        mover_motores(VELOCIDADE_EXPLORACAO, VELOCIDADE_EXPLORACAO);
    }
}

// --------------------------------------------------
void navega_waypoints() {
  if (!odometria_inicializada || total_waypoints == 0 || waypoint_atual_idx >= total_waypoints) {
      parar_motores();
      if (ws.count() > 0) {
          if (!odometria_inicializada) ws.textAll("{\"waypoint_status\":\"Cancelado: sem sinal de GPS valido\"}");
          else if (total_waypoints == 0) ws.textAll("{\"waypoint_status\":\"Cancelado: nenhum waypoint valido recebido\"}");
      }
      modo_waypoints = false;
      return; 
  }

  float target_lat = waypoints_lat[waypoint_atual_idx];
  float target_lon = waypoints_lon[waypoint_atual_idx];

  bool obstaculo_frente = wp_evita_colisao && ((distancia > 0) && (distancia <= DISTANCIA_OBSTACULO));
  bool queda_detectada = wp_evita_queda && ((analogRead(SENSOR_LINHA_ESQUERDO) > LIMIAR_QUEDA) || (analogRead(SENSOR_LINHA_DIREITO) > LIMIAR_QUEDA));

  if (obstaculo_frente || queda_detectada) {
      parar_motores();
      if (ws.count() > 0) ws.textAll("{\"waypoint_status\":\"Bloqueado (Queda/Obstáculo)!\"}");
      return;
  }

  // FASE 1: CALIBRAÇÃO DA BÚSSOLA GEOMÉTRICA VIA GPS
  float flat, flon; unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  
  if (!heading_gps_valido) {
      if (flat != TinyGPS::GPS_INVALID_F_ANGLE) {
          if (!registrou_inicio_calibracao) {
              lat_inicio_calibracao = flat; lon_inicio_calibracao = flon; registrou_inicio_calibracao = true;
          } else {
              float dist_fisica_m = TinyGPS::distance_between(lat_inicio_calibracao, lon_inicio_calibracao, flat, flon);
              if (dist_fisica_m >= 2.5) { // Espera andar 2.5 metros reais no GPS para calibrar a bússola
                  float curso_calculado = TinyGPS::course_to(lat_inicio_calibracao, lon_inicio_calibracao, flat, flon);
                  theta_rad = curso_calculado * PI / 180.0; heading_gps_valido = true; 
              }
          }
      }
      mover_motores(VELOCIDADE_EXPLORACAO, VELOCIDADE_EXPLORACAO);
      if (ws.count() > 0) ws.textAll("{\"waypoint_status\":\"Calibrando Direcao...\"}");
      return; 
  }

  // FASE 2: VERIFICAÇÃO DE CHEGADA (Raio de 1.0m)
  float distancia_alvo = TinyGPS::distance_between(estimativa_lat, estimativa_lon, target_lat, target_lon);
  if (distancia_alvo < 1.0) {
      parar_motores();
      giros_consecutivos_way = 0;
      waypoint_atual_idx++;
      if (waypoint_atual_idx >= total_waypoints) {
          modo_waypoints = false;
          estadoWay = WAY_LIVRE;
          heading_gps_valido = false; 
          total_waypoints = 0;
          if (ws.count() > 0) ws.textAll("{\"waypoint_status\":\"Destino Final Alcancado!\"}");
      } else {
          if (ws.count() > 0) {
              char msgStatus[64];
              sprintf(msgStatus, "{\"waypoint_status\":\"Ponto %d/%d alcancado. Indo para o prox...\"}", waypoint_atual_idx, total_waypoints);
              ws.textAll(msgStatus);
          }
          delay(1000); 
      }
      return;
  }

  if (ws.count() > 0) {
      char msgStatus[64];
      sprintf(msgStatus, "{\"waypoint_status\":\"Navegando para o Ponto %d/%d\"}", waypoint_atual_idx + 1, total_waypoints);
      ws.textAll(msgStatus);
  }

  // FASE 3: NAVEGAÇÃO RETA CORRIGIDA POR GPS E FILTRO ANTI-STALL
  float curso_alvo = TinyGPS::course_to(estimativa_lat, estimativa_lon, target_lat, target_lon);
  
  static float ant_lat = 0.0, ant_lon = 0.0;
  static int estado_motor_anterior = 0; 

  if (flat != TinyGPS::GPS_INVALID_F_ANGLE) {
      if (ant_lat == 0.0 && ant_lon == 0.0) {
          ant_lat = flat; ant_lon = flon;
      } else if (flat != ant_lat || flon != ant_lon) {
          float dist_movida = TinyGPS::distance_between(ant_lat, ant_lon, flat, flon);
          if (dist_movida >= 1.0) {
              float curso_gps_real = TinyGPS::course_to(ant_lat, ant_lon, flat, flon);
              if (curso_gps_real >= 0) {
                  float erro_mix = curso_gps_real - (theta_rad * 180.0 / PI);
                  while(erro_mix > 180.0) erro_mix -= 360.0;
                  while(erro_mix < -180.0) erro_mix += 360.0;
                  theta_rad += (erro_mix * 0.3) * PI / 180.0;
              }
              ant_lat = flat; ant_lon = flon;
          }
      }
  }

  float curso_atual = theta_rad * 180.0 / PI;
  float erro_angular = curso_alvo - curso_atual;
  if (erro_angular < -180.0) erro_angular += 360.0;
  if (erro_angular > 180.0) erro_angular -= 360.0;

  if (abs(erro_angular) > 50.0) {
      if (estado_motor_anterior == 0) { parar_motores(); delay(250); tempo_inicio_manobra_way = millis(); }
      estado_motor_anterior = 1;

      if (millis() - tempo_inicio_manobra_way > TIMEOUT_GIRO_WAYPOINT) {
          giros_consecutivos_way++;
          if (giros_consecutivos_way >= LIMITE_GIROS_ANTISTALL_WAYPOINT) {
              parar_motores();
              modo_waypoints = false;
              giros_consecutivos_way = 0;
              if (ws.count() > 0) ws.textAll("{\"waypoint_status\":\"Cancelado: travado (terreno dificil)\"}");
              return;
          }

          if (ws.count() > 0) ws.textAll("{\"waypoint_status\":\"Anti-stall: forcando tracao reto\"}");
          parar_motores(); delay(150);
          mover_motores(VELOCIDADE_MAXIMA, VELOCIDADE_MAXIMA);
          delay(EMPURRAO_ANTISTALL_WAYPOINT_MS);
          parar_motores(); delay(150);
          estado_motor_anterior = 0;
          tempo_inicio_manobra_way = millis();
          return;
      }

      int sentido = (erro_angular > 0) ? 1 : -1;
      mover_motores(65 * sentido, -65 * sentido); 
  } else {
      if (estado_motor_anterior == 1) { parar_motores(); delay(250); }
      estado_motor_anterior = 0;
      giros_consecutivos_way = 0;

      int vel_esq = VELOCIDADE_EXPLORACAO; int vel_dir = VELOCIDADE_EXPLORACAO; 
      
      if (erro_angular > 12.0) { 
          vel_dir = 35;
          vel_esq = VELOCIDADE_MAXIMA; 
      } else if (erro_angular < -12.0) { 
          vel_esq = 35; 
          vel_dir = VELOCIDADE_MAXIMA; 
      }
      mover_motores(vel_esq, vel_dir);
  }
}

// --------------------------------------------------
void calcula_PID() {
  unsigned long agora = millis();
  float dt = (pid_tempo_anterior == 0) ? 0.02 : (agora - pid_tempo_anterior) / 1000.0;
  if (dt <= 0.0001) dt = 0.0001; 
  pid_tempo_anterior = agora;

  P = erro;
  I = I + (erro * dt);
  if (I > 50) I = 50; else if (I < -50) I = -50;
  if (fabs(erro) < 0.05) I = 0; 

  D = (erro - erro_anterior) / dt;
  if (D > 10.0) D = 10.0; else if (D < -10.0) D = -10.0; 

  resposta_PID = (Kp * P) + (Ki * I) + (Kd * D);
  erro_anterior = erro;

  const float RESPOSTA_PID_MAX = VELOCIDADE - 5.0;
  if (resposta_PID > RESPOSTA_PID_MAX) resposta_PID = RESPOSTA_PID_MAX;
  if (resposta_PID < -RESPOSTA_PID_MAX) resposta_PID = -RESPOSTA_PID_MAX;

  velocidade_esquerda = VELOCIDADE + resposta_PID; velocidade_direita =  VELOCIDADE - resposta_PID;
  if (velocidade_esquerda > VELOCIDADE_MAXIMA) velocidade_esquerda = VELOCIDADE_MAXIMA; else if (velocidade_esquerda < VELOCIDADE_MINIMA) velocidade_esquerda = VELOCIDADE_MINIMA;
  if (velocidade_direita > VELOCIDADE_MAXIMA) velocidade_direita = VELOCIDADE_MAXIMA; else if (velocidade_direita < VELOCIDADE_MINIMA) velocidade_direita = VELOCIDADE_MINIMA;
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t length) {
  switch (type) {
    case WS_EVT_CONNECT: digitalWrite(PINO_LED, HIGH); break;
    case WS_EVT_DISCONNECT:
        if (ws.count() == 0) {
          digitalWrite(PINO_LED, LOW); modo_linha = false; modo_explora = false; modo_waypoints = false; parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE; estadoDesvioLinha = DESVIO_LINHA_LIVRE; total_waypoints = 0;
        } break;
    case WS_EVT_DATA: handleWebSocketMessage(arg, data, length); break;
    case WS_EVT_PONG: case WS_EVT_ERROR: break;
  }
}

const int LIMIAR_LINHA_SPAN_MINIMO = 500;

// forcarPolaridade = true: usuário escolheu manualmente "Linha Escura"/"Linha Clara" (valorForcado),
// ignorando a leitura inicial ambígua. forcarPolaridade = false: modo Auto (tenta adivinhar pela leitura inicial).
void calibrarSensoresLinha(bool forcarPolaridade, bool valorForcado) {
    modo_linha = false; modo_explora = false; modo_waypoints = false; parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE; estadoDesvioLinha = DESVIO_LINHA_LIVRE; total_waypoints = 0;

    long somaFundo = 0; const int N_AMOSTRAS_FUNDO = 10;
    for (int i = 0; i < N_AMOSTRAS_FUNDO; i++) {
        somaFundo += (analogRead(SENSOR_LINHA_ESQUERDO) + analogRead(SENSOR_LINHA_DIREITO)) / 2;
        delay(5);
    }
    int fundoInicial = somaFundo / N_AMOSTRAS_FUNDO;

    int minVal = 4095; int maxVal = 0; uint32_t inicio = millis();

    // Varredura mais longa e com bursts maiores (400ms em vez de 250ms) para dar tempo real de
    // deslocamento aos motores e aumentar a chance dos sensores cruzarem a borda da fita mesmo
    // que o robô não esteja perfeitamente centralizado nela.
    while (millis() - inicio < 3500) {
        if (((millis() - inicio) / 400) % 2 == 0) mover_motores(65, -65); else mover_motores(-65, 65);
        int leituraEsq = analogRead(SENSOR_LINHA_ESQUERDO); int leituraDir = analogRead(SENSOR_LINHA_DIREITO);
        if (leituraEsq < minVal) minVal = leituraEsq; if (leituraDir < minVal) minVal = leituraDir;
        if (leituraEsq > maxVal) maxVal = leituraEsq; if (leituraDir > maxVal) maxVal = leituraDir;
        delay(10);
    }
    parar_motores();

    int span = maxVal - minVal;
    int novoLimiar = (minVal + maxVal) / 2;

    bool span_ok = (span >= LIMIAR_LINHA_SPAN_MINIMO);
    // A checagem de "fundo ambíguo" só faz sentido no modo Auto: ela existe para evitar confiar
    // numa leitura inicial que não deixa claro se o robô começou em cima da linha ou do fundo.
    // Quando o usuário força a polaridade manualmente, essa leitura é irrelevante.
    bool fundo_ambiguo = (!forcarPolaridade) && (abs(fundoInicial - novoLimiar) < (span / 4));

    if (!span_ok || fundo_ambiguo) {
        JsonDocument json;
        json["status"] = "calibracao_falhou";
        json["motivo"] = !span_ok ? "contraste_insuficiente" : "fundo_ambiguo";
        json["min"] = minVal; json["max"] = maxVal; json["span"] = span; json["fundoInicial"] = fundoInicial;
        size_t msg_comp = measureJson(json); char msg[msg_comp + 1]; serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0;
        if (ws.count() > 0) ws.textAll(msg, msg_comp);
        return;
    }

    limiarLinha = novoLimiar;
    linha_escura = forcarPolaridade ? valorForcado : (fundoInicial < limiarLinha);

    SPIFFS.begin(DIRETORIO_SPIFFS, false); SPIFFS.putInt("limiar", limiarLinha); SPIFFS.putBool("linha_escura", linha_escura); SPIFFS.end();

    JsonDocument json; json["status"] = "calibrado"; json["limiar"] = limiarLinha; json["escura"] = linha_escura; json["span"] = span;
    size_t msg_comp = measureJson(json); char msg[msg_comp + 1]; serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0;
    if (ws.count() > 0) ws.textAll(msg, msg_comp); 
}

void verificarSegurancaBateria() {
    uint32_t tensao_mv = vbat.readVoltage();
    ultima_tensao_bateria_mv = tensao_mv;
    static uint8_t leituras_criticas_consecutivas = 0;
    
    if (tensao_mv < TENSAO_CRITICA && tensao_mv > 3000) {
        leituras_criticas_consecutivas++;
        if (leituras_criticas_consecutivas >= 3) {
            if (!modoSegurancaBateria) {
                modoSegurancaBateria = true; modo_linha = false; modo_explora = false; modo_waypoints = false; parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE; estadoDesvioLinha = DESVIO_LINHA_LIVRE; total_waypoints = 0;
                if (ws.count() > 0) ws.textAll("{\"alert\":\"LOW_BATTERY\"}");
            }
            digitalWrite(PINO_LED, (millis() / 150) % 2 ? HIGH : LOW);
        }
    } 
    else if (tensao_mv >= (TENSAO_CRITICA + 300)) { 
        leituras_criticas_consecutivas = 0; modoSegurancaBateria = false; digitalWrite(PINO_LED, LOW);
    }
}