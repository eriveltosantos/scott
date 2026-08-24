/*******************************************************************************
* Scott Robot - Kit Robo Explorer - Joystick + Linha + Exploração + Waypoints GPS + Odometria + Grid Mapping
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
uint32_t alvoPulsosWay = 0;
uint32_t tempo_inicio_manobra_way = 0;
int sentido_giro_way = 1; 

// --- ANTI-STALL DA NAVEGAÇÃO POR WAYPOINT ---
const uint32_t TIMEOUT_GIRO_WAYPOINT = 4000;         
const uint32_t EMPURRAO_ANTISTALL_WAYPOINT_MS = 1200; 
const int LIMITE_GIROS_ANTISTALL_WAYPOINT = 4;       
int giros_consecutivos_way = 0;

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

float lat_inicio_calibracao = 0.0;
float lon_inicio_calibracao = 0.0;
unsigned long pulsos_inicio_calibracao = 0; 
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

// ALVOS DE PULSOS PARA MANOBRAS (usados nos deslocamentos retos, ex. PULSOS_20_CM no reposicionamento em ré)
// Os alvos de GIRO em graus (antigos PULSOS_45_GRAUS/90/180, removidos) foram
// substituídos em explora_casa() por um alvo de ROTAÇÃO MEDIDA (theta_rad, ver
// alvoAnguloGiroRad), já que contar pulsos de uma roda só presumia as duas
// esteiras girando em sincronia, o que não é verdade quando uma delas
// patina/derrapa parada durante o giro.
const uint32_t PULSOS_30_CM = 100;  
const uint32_t PULSOS_20_CM = 67;  
const uint32_t PULSOS_15_CM = 50;  

// MAPEAMENTO E VARIÁVEIS DOS ENCODERS
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

// --- Leitura filtrada dos sensores de linha (média móvel exponencial p/ reduzir ruído do ADC) ---
float leitura_esquerda_filtrada = 0.0;
float leitura_direita_filtrada = 0.0;
const float ALPHA_FILTRO_LINHA = 0.5; // 0 = sem filtro, 1 = ignora leitura nova (mais suave = mais perto de 0)

// --- Normalização do erro proporcional e controle de tempo (dt) do PID ---
const float FATOR_NORMALIZACAO_LINHA = 2500.0; // diferença de leitura (ADC) que equivale a erro = 1.0
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
unsigned long posicao_inicial_manobra = 0;
uint32_t alvoPulsosManobra = 0;

// --- MÁQUINA DE ESTADOS EXPLORAÇÃO SIMPLIFICADA E SORTEADA ---
enum EstadoExploracao { EXPLORA_LIVRE, EXPLORA_RE, EXPLORA_GIRO, EXPLORA_PARADO };
EstadoExploracao estadoExplora = EXPLORA_LIVRE;
int giros_consecutivos = 0; 
unsigned long posicao_inicial_explora = 0;
uint32_t alvoPulsosExplora = 0; 
uint32_t tempo_inicio_manobra_explora = 0; 
const uint32_t TIMEOUT_MAX_EXPLORA = 6000;  
int sentido_giro_atual = 1;
const int MAX_GIROS_CONSECUTIVOS = 3;

// --- GIRO POR ODOMETRIA (usa os DOIS encoders, não só o esquerdo) ---
// theta_rad já é atualizado em atualizar_odometria_fusao() a partir da
// DIFERENÇA entre os pulsos esquerdo e direito — é uma medida melhor de
// quanto o robô realmente girou do que contar pulsos de uma roda só,
// porque não assume que as duas esteiras avançam na mesma proporção.
float giro_acumulado_rad = 0.0;
float alvoAnguloGiroRad = 0.0;

// --- DETECÇÃO DE ESTEIRA PATINANDO (SKID) DURANTE O GIRO ---
unsigned long posicao_inicial_dir_explora = 0;
unsigned long ultimo_check_stall_ms = 0;
unsigned long ultima_contagem_esq_stall = 0;
unsigned long ultima_contagem_dir_stall = 0;
const uint32_t INTERVALO_CHECK_STALL_MS = 200;
const uint32_t PULSOS_MINIMOS_NAO_TRAVADO = 2;
const uint32_t DURACAO_KICK_ANTISTALL_MS = 150;

bool modoSegurancaBateria = false;
uint32_t ultima_tensao_bateria_mv = 0; // exposta p/ debug (atualizada em verificarSegurancaBateria)
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
                <button type="button" class="btn btn-warning" onclick="calibrarSensores()">Calibrate Sensors</button>
                <div id="statusCalibracao" style="margin-top: 10px; font-weight: bold; text-align:center; color: #4CAF50;"></div>
            </div>
            <div class="setup-box">
                <span class="setup-title">Connect to Wi-Fi</span>
                <div class="input-group"><div class="input-group-addon">SSID</div><input type="text" id="wifi_ssid" placeholder="Network name" class="input"></div>
                <div class="input-group"><div class="input-group-addon">Pass</div><input type="password" id="wifi_pass" placeholder="Network password" class="input"></div>
                <button type="button" class="btn btn-warning" onclick="send_wifi()">Connect</button>
            </div>
        </div>
    </div>
    
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" defer></script>
    <script>
        function safelySend(dataObj) { if (connection && connection.readyState === WebSocket.OPEN) { connection.send(JSON.stringify(dataObj)); } }
        function calibrarSensores() { document.getElementById('statusCalibracao').innerText = "Calibrating..."; document.getElementById('statusCalibracao').style.color = "#f0be00"; safelySend({ cmd: "calibrate_line" }); }

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
            if (data["fall"] !== undefined) { document.getElementById("fall-status-text").innerText = data["fall"] ? "Fall Detected" : ""; }
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
            
            // --- BLOCO NOVO: Tratamento do retorno da calibração ---
            if (data["status"] === "calibrado") {
                let corDetectada = data["escura"] ? "Linha Escura" : "Linha Clara";
                document.getElementById('statusCalibracao').innerText = "OK! Limiar: " + data["limiar"] + " (" + corDetectada + ")";
                document.getElementById('statusCalibracao').style.color = "#4CAF50";
            }
            if (data["status"] === "calibracao_falhou") {
                let motivoTexto = (data["motivo"] === "fundo_ambiguo")
                    ? "leitura inicial ambigua"
                    : "contraste insuficiente";
                document.getElementById('statusCalibracao').innerText = "Calibration rejected! (" + motivoTexto + ") Posicione o robo sobre a fita e tente de novo.";
                document.getElementById('statusCalibracao').style.color = "#e53935";
            }
            // --------------------------------------------------------
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
            ['mousedown','touchstart'].forEach(evt => canvas_joystick.addEventListener(evt, startDrawing));
            ['mouseup','touchend','touchcancel'].forEach(evt => canvas_joystick.addEventListener(evt, stopDrawing));
            ['mousemove','touchmove'].forEach(evt => canvas_joystick.addEventListener(evt, Draw));
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

        function startDrawing(event) { paint = true; document.activeElement.blur(); resetModosAutonomosVisual(); getPosition_joystick(event); if (in_circle()) { joystick(coord.x, coord.y); Draw(event); } }
        function stopDrawing() { paint = false; joystick(origin_joystick.x, origin_joystick.y); document.getElementById("speed").innerText = 0; document.getElementById("table-speed").innerText = 0; if (movimento == 1) { safelySend({"velocidade":0, "angulo":0}); movimento = 0; } resetModosAutonomosVisual(); }

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
void explora_casa(void);
void navega_waypoints(void);
void calibrarSensoresLinha(void);
void verificarSegurancaBateria(void);
void processar_mapeamento_e_telemetria(void);
void carregar_mapa_salvo(void);

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
  delay(300); // dá tempo do monitor serial conectar antes da primeira mensagem
  Serial.println("=====================================");
  Serial.println("FIRMWARE Scott_robot - build debug-v5 (contador_parada por erro, nao mais por sensor absoluto)");
  Serial.println("(erro proporcional + PID por dt + vbat/segBat/contParada no log)");
  Serial.println("=====================================");
  
  SerialGPS.setRxBufferSize(1024);
  SerialGPS.begin(9600, SERIAL_8N1, PINO_GPS_RX, PINO_GPS_TX);
  
  Serial.println("RoboCore - Kit Robo Explorer - Waypoints GPS Multi");

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
 // Kp = SPIFFS.getFloat(ENDERECOS_SPIFFS[1], 40);
  Kp = 40.0; 
  Ki = SPIFFS.getFloat(ENDERECOS_SPIFFS[2], 0.2);
  Kd = SPIFFS.getFloat(ENDERECOS_SPIFFS[3], 0.4);
  limiarLinha = SPIFFS.getInt("limiar", 3000); 
  linha_escura = SPIFFS.getBool("linha_escura", true);
  SPIFFS.end();

  Serial.printf("[BOOT] Calibracao de linha carregada: limiar=%d linha_escura=%d\n", limiarLinha, linha_escura);

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
      JsonDocument json; json["robot_speed"] = robot_speed_cms;
      size_t msg_comp = measureJson(json); char msg[msg_comp + 1];
      serializeJson(json, msg, msg_comp + 1); msg[msg_comp] = 0; ws.textAll(msg, msg_comp);
    }

    last_contador_esq_A = contador_esq_A;
    last_contador_esq_B = contador_esq_B;
    last_contador_dir_A = contador_dir_A;
    last_contador_dir_B = contador_dir_B;
    tempo_antes_encoder = millis();
  }

  if (modoSegurancaBateria) {
      modo_linha = false; modo_explora = false; modo_waypoints = false;
      parar_motores();
  } else {
      if (modo_linha) segue_linha();
      else if (modo_explora) explora_casa();
      else if (modo_waypoints) navega_waypoints();
  }

  // --- STATUS GERAL (sempre roda, independente do modo) ---
  // Existe pra diagnosticar quando segue_linha() nem chega a ser chamado
  // (ex.: preso em modoSegurancaBateria mesmo com o app mostrando "Linha" ativa).
  static uint32_t timeout_debug_status = 0;
  if (millis() > timeout_debug_status) {
      Serial.printf(
          "[STATUS] modo_linha=%d modo_explora=%d modo_waypoints=%d | modoSegurancaBateria=%d vbat=%4lumV | contador_parada=%d | pulsos_brutos_esq=%lu pulsos_brutos_dir=%lu\n",
          modo_linha, modo_explora, modo_waypoints,
          modoSegurancaBateria, (unsigned long)ultima_tensao_bateria_mv,
          contador_parada, (unsigned long)contador_esq_A, (unsigned long)contador_dir_A
      );
      timeout_debug_status = millis() + 500;
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
            json["lat"] = odometria_inicializada ? estimativa_lat : flat; 
            json["lon"] = odometria_inicializada ? estimativa_lon : flon; 
        }
        json["date"] = date_str; json["time"] = time_str;
        
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
    if (!LittleFS.begin(true)) {
        Serial.println("Erro ao montar o sistema de arquivos LittleFS!");
        return;
    }

    File file = LittleFS.open("/mapa_salvo.json", "r");
    if (!file) {
        Serial.println("Arquivo de mapa não encontrado na memória flash.");
        return;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
        Serial.println("Falha ao ler o JSON do mapa.");
        file.close();
        return;
    }

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
            // Se o score salvo for >= 70, marca como obstáculo (2). Se for <= 30, livre (1).
            occupancyMap[grid_x][grid_y] = (score >= 70) ? 2 : ((score <= 30) ? 1 : 0);
        }
    }
    file.close();
    Serial.println("Mapa carregado com sucesso na RAM! Scott agora conhece o ambiente.");
}

// --------------------------------------------------

void atualizar_odometria_fusao() {
    long pulsos_esq_atual = contador_esq_A;
    long pulsos_dir_atual = contador_dir_A;
    
    long delta_esq = (long)(pulsos_esq_atual - odo_esq_anterior) * sinal_esq;
    long delta_dir = (long)(pulsos_dir_atual - odo_dir_anterior) * sinal_dir;
    
    odo_esq_anterior = pulsos_esq_atual; odo_dir_anterior = pulsos_dir_atual;
    
    float dist_esq = delta_esq * CM_POR_PULSO; float dist_dir = delta_dir * CM_POR_PULSO;
    float dist_centro = (dist_esq + dist_dir) / 2.0;
    
    float delta_theta = (dist_esq - dist_dir) / LARGURA_ESTEIRA_EFETIVA;
    theta_rad += delta_theta;
    
    // Acumulador de rotação SEM wraparound, usado por explora_casa() para medir
    // o giro em andamento. theta_rad (abaixo) é mantido "enrolado" em [0, 2π) —
    // ótimo para heading absoluto, mas ruim para medir "quanto girei desde que
    // comecei este giro" quando um único salto (ex.: depois de um kick
    // anti-derrapagem bloqueante) pode passar de 180°. Somando o delta bruto
    // aqui, giro nenhum fica escondido, não importa o tamanho do salto.
    giro_acumulado_rad += delta_theta;
    
    while (theta_rad >= TWO_PI) theta_rad -= TWO_PI; while (theta_rad < 0) theta_rad += TWO_PI;
    
    float delta_x_cm = dist_centro * cos(theta_rad); float delta_y_cm = dist_centro * sin(theta_rad); 
    
    robot_local_x_cm += delta_x_cm;
    robot_local_y_cm += delta_y_cm;

    float delta_lat = delta_x_cm / 11132000.0; float delta_lon = 0.0;
    
    if (odometria_inicializada) {
        delta_lon = delta_y_cm / (11132000.0 * cos(estimativa_lat * PI / 180.0));
        estimativa_lat += delta_lat; estimativa_lon += delta_lon;
    }

    float flat, flon; unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    static float ultimo_flat_processado = 0.0;
    
    if (flat != TinyGPS::GPS_INVALID_F_ANGLE) {
        if (!odometria_inicializada) {
            estimativa_lat = flat; estimativa_lon = flon; ultimo_flat_processado = flat;
            odometria_inicializada = true; heading_gps_valido = false; registrou_inicio_calibracao = false;
        } else {
            if (flat != ultimo_flat_processado && age < 3000) { 
                estimativa_lat = (estimativa_lat * 0.85) + (flat * 0.15); 
                estimativa_lon = (estimativa_lon * 0.85) + (flon * 0.15); 
                ultimo_flat_processado = flat;
                
                if (!heading_gps_valido && modo_waypoints) {
                    if (!registrou_inicio_calibracao) {
                        lat_inicio_calibracao = flat; lon_inicio_calibracao = flon; pulsos_inicio_calibracao = contador_esq_A; registrou_inicio_calibracao = true;
                    } else {
                        float dist_fisica_cm = (contador_esq_A - pulsos_inicio_calibracao) * CM_POR_PULSO;
                        if (dist_fisica_cm >= 250.0) {
                            float curso_calculado = TinyGPS::course_to(lat_inicio_calibracao, lon_inicio_calibracao, flat, flon);
                            theta_rad = curso_calculado * PI / 180.0; heading_gps_valido = true; 
                        }
                    }
                }
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
      calibrarSensoresLinha();
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
    else if (strstr(reinterpret_cast<char *>(data), "wp_queda") != nullptr) {
        JsonDocument json; deserializeJson(json, data, length);
        if(json.containsKey("wp_queda")) wp_evita_queda = json["wp_queda"].as<bool>();
        if(json.containsKey("wp_colisao")) wp_evita_colisao = json["wp_colisao"].as<bool>();
    }
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_LINHA) != nullptr || strstr(reinterpret_cast<char *>(data), ALIAS_EXPLORA) != nullptr) {
      JsonDocument json; deserializeJson(json, data, length);
      if (json.containsKey(ALIAS_LINHA)) modo_linha = json[ALIAS_LINHA].as<bool>();
      if (json.containsKey(ALIAS_EXPLORA)) modo_explora = json[ALIAS_EXPLORA].as<bool>();
      
      if (modo_linha) { modo_explora = false; modo_waypoints = false; erro = 0.0; I = 0.0; erro_anterior = 0.0; contador_parada = 0; leitura_esquerda_filtrada = 0.0; leitura_direita_filtrada = 0.0; pid_tempo_anterior = 0; }
      else if (modo_explora) { modo_linha = false; modo_waypoints = false; }

      if (!modo_linha && !modo_explora && !modo_waypoints) {
        parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE; ws.textAll("{\"fall\":false, \"stuck\":false}"); 
      }
    }
    else if (strstr(reinterpret_cast<char *>(data), ALIAS_VELOCIDADE) != nullptr) {
      if (modo_linha || modo_explora || modo_waypoints) {
        modo_linha = false; modo_explora = false; modo_waypoints = false; total_waypoints = 0;
        parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE;
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
// --------------------------------------------------
void segue_linha() {
  leitura_esquerdo = analogRead(SENSOR_LINHA_ESQUERDO); 
  leitura_direito = analogRead(SENSOR_LINHA_DIREITO);

  // Filtro passa-baixa (média móvel exponencial) para não deixar ruído do ADC,
  // perto do limiar, virar um "salto" de estado e disparar o derivativo à toa.
  if (leitura_esquerda_filtrada == 0.0 && leitura_direita_filtrada == 0.0) {
      leitura_esquerda_filtrada = leitura_esquerdo;
      leitura_direita_filtrada  = leitura_direito;
  } else {
      leitura_esquerda_filtrada = (ALPHA_FILTRO_LINHA * leitura_esquerdo) + ((1.0 - ALPHA_FILTRO_LINHA) * leitura_esquerda_filtrada);
      leitura_direita_filtrada  = (ALPHA_FILTRO_LINHA * leitura_direito)  + ((1.0 - ALPHA_FILTRO_LINHA) * leitura_direita_filtrada);
  }

  // --- ERRO PROPORCIONAL ---
  // Em vez de degraus fixos (0/±1/±2), o erro agora reflete o QUANTO a linha
  // está desviada, calculado direto da diferença analógica entre os sensores.
  // Sinal: positivo = linha mais para a direita (precisa virar à direita).
  // IMPORTANTE: por ser uma DIFERENÇA entre os dois sensores, esse erro é
  // imune a uma queda de leitura que afete os dois sensores igualmente
  // (ex.: alimentação caindo um pouco sob carga dos motores) — só reage
  // quando os sensores discordam entre si, que é o que realmente importa.
  float sinalPolaridade = linha_escura ? 1.0 : -1.0;
  erro = sinalPolaridade * (leitura_direita_filtrada - leitura_esquerda_filtrada) / FATOR_NORMALIZACAO_LINHA;
  if (erro > 2.0) erro = 2.0;
  if (erro < -2.0) erro = -2.0;

  // --- DETECÇÃO DE LINHA PERDIDA ---
  // O erro proporcional (diferença ENTRE os sensores) satura quando a linha
  // está claramente descentralizada, mas fica cego a um caso: os DOIS
  // sensores saindo da fita ao mesmo tempo, lendo valores parecidos entre si
  // (erro ~0) sem que nenhum deles esteja de fato sobre a fita. Por isso,
  // além da saturação do erro relativo, checamos também se AMBOS os
  // sensores estão do lado "fundo" (fora da fita) do limiar absoluto.
  bool erro_saturado = (fabs(erro) >= 1.9);
  // "Fora da fita" = os dois sensores lendo o valor de FUNDO (piso), não de linha.
  // Se linha_escura=true, a fita é o extremo ALTO e o piso é o BAIXO -> fora da
  // fita = os dois ABAIXO do limiar. Se linha_escura=false (seu caso: piso alto,
  // fita baixa), fora da fita = os dois ACIMA do limiar. (Corrigido: a versão
  // anterior tinha os dois ramos trocados, fazendo o robô se achar "fora da
  // fita" bem no meio do seguimento normal.)
  bool ambos_fora_da_fita = linha_escura
      ? (leitura_esquerda_filtrada < limiarLinha && leitura_direita_filtrada < limiarLinha)
      : (leitura_esquerda_filtrada > limiarLinha && leitura_direita_filtrada > limiarLinha);

  if (erro_saturado || ambos_fora_da_fita) {
      contador_parada++;
  } else {
      contador_parada = 0;
  }

  if (contador_parada >= CONTAGEM_MAXIMA) {
      // Ficou tempo demais com a linha genuinamente perdida: para e zera o PID.
      // Isso roda ANTES de calcular/mandar velocidade, então não fica mais
      // mandando "anda" e "para" no mesmo loop.
      parar_motores();
      P = 0; I = 0; D = 0; erro = 0.0; erro_anterior = 0.0;
      contador_parada = CONTAGEM_MAXIMA;
  } else if (ambos_fora_da_fita) {
      // Nenhum sensor está de fato sobre a fita agora — a diferença calculada
      // em "erro" não reflete a posição da linha, é só descasamento/ruído entre
      // os dois canais do ADC enquanto ambos enxergam apenas o piso. Alimentar
      // isso no PID gera giros que não têm relação com onde a fita realmente
      // está (é o que fazia o robô virar sozinho antes mesmo de detectar a
      // fita pela primeira vez). Enquanto estiver assim, anda reto e preserva
      // o estado do PID (P/I/D/erro_anterior) intacto, para retomar suave
      // quando a fita reaparecer sob um dos sensores.
      mover_motores(VELOCIDADE, VELOCIDADE);
  } else {
      calcula_PID();
      mover_motores(velocidade_esquerda, velocidade_direita);
  }

  // --- DEBUG DE TUNING (remover/comentar depois de ajustar Kp/Ki/Kd) ---
  // Limitado a ~5x/s para não sobrecarregar a serial nem atrasar o loop.
  static uint32_t timeout_debug_linha = 0;
  if (millis() > timeout_debug_linha) {
      Serial.printf(
          "esq=%4d dir=%4d | filt_esq=%6.0f filt_dir=%6.0f | erro=%+.2f P=%+.2f I=%+.2f D=%+.2f resp=%+.1f | v_esq=%4d v_dir=%4d | vbat=%4lumV segBat=%d contParada=%d\n",
          leitura_esquerdo, leitura_direito,
          leitura_esquerda_filtrada, leitura_direita_filtrada,
          erro, P, I, D, resposta_PID,
          velocidade_esquerda, velocidade_direita,
          (unsigned long)ultima_tensao_bateria_mv, modoSegurancaBateria, contador_parada
      );
      timeout_debug_linha = millis() + 200;
  }

  delay(espera);
}
// --------------------------------------------------
// NOVA FUNÇÃO DO MODO DE EXPLORAÇÃO
// O Robô dá ré e em seguida sorteia aleatoriamente se irá girar 90 graus para a esquerda ou para a direita
void explora_casa() {
    bool obstaculo_frente = ((distancia > 0) && (distancia <= DISTANCIA_EXPLORACAO));
    bool queda_detectada = (analogRead(SENSOR_LINHA_ESQUERDO) > LIMIAR_QUEDA) || (analogRead(SENSOR_LINHA_DIREITO) > LIMIAR_QUEDA);
    static bool status_queda_ui_explora = false;

    if (estadoExplora == EXPLORA_PARADO) { parar_motores(); return; }

    if (estadoExplora != EXPLORA_LIVRE) {
        if (millis() - tempo_inicio_manobra_explora > TIMEOUT_MAX_EXPLORA) {
            // Segmento (ré ou giro) não terminou dentro do prazo. Trata isso
            // como uma TENTATIVA FALHA, sujeita ao MESMO limite de tentativas
            // (MAX_GIROS_CONSECUTIVOS) usado quando o giro termina mas o
            // obstáculo continua lá. Antes, um timeout resetava
            // giros_consecutivos = 0 e voltava pra EXPLORA_LIVRE — como o
            // sensor de obstáculo tende a continuar "bloqueado" logo em
            // seguida (o giro não tinha terminado de verdade), isso reiniciava
            // ré+giro do zero indefinidamente, e cada tentativa nova podia
            // gastar até TIMEOUT_MAX_EXPLORA girando — several reinícios somam
            // várias voltas completas, mesmo com o teto de 180° por episódio.
            parar_motores(); delay(100); giros_consecutivos++;

            if (giros_consecutivos >= MAX_GIROS_CONSECUTIVOS || estadoExplora == EXPLORA_RE) {
                // Limite de tentativas estourado, OU travou tentando dar ré
                // (sem uma estratégia melhor de recuperação pra isso) -
                // mais seguro parar e sinalizar do que insistir sem fim.
                estadoExplora = EXPLORA_PARADO; if (ws.count() > 0) ws.textAll("{\"stuck\":true}");
            } else {
                // Ainda dentro do limite de tentativas: continua girando na
                // mesma direção sorteada, com um novo alvo (45°) e prazo frescos.
                mover_motores(-VELOCIDADE_MAXIMA * sentido_giro_atual, VELOCIDADE_MAXIMA * sentido_giro_atual);
                delay(DURACAO_KICK_ANTISTALL_MS);
                mover_motores(-VELOCIDADE_GIRO * sentido_giro_atual, VELOCIDADE_GIRO * sentido_giro_atual);
                posicao_inicial_explora = contador_esq_A;
                posicao_inicial_dir_explora = contador_dir_A;
                giro_acumulado_rad = 0.0;
                alvoAnguloGiroRad = PI / 4.0;
                tempo_inicio_manobra_explora = millis();
                ultimo_check_stall_ms = millis();
                ultima_contagem_esq_stall = contador_esq_A;
                ultima_contagem_dir_stall = contador_dir_A;
            }
            return;
        }
        
        if (estadoExplora == EXPLORA_RE) {
            // --- VIGIA ANTI-DERRAPAGEM (mesma ideia usada no giro) ---
            // Também dando ré, uma esteira pode ficar presa (atrito estático)
            // enquanto a outra se move sozinha — o mesmo sintoma relatado para
            // o giro, só que em linha reta. Sem essa checagem o robô podia
            // "achar" que andou 20cm de ré usando só o encoder esquerdo,
            // mesmo com o direito parado (ou vice-versa).
            if (millis() - ultimo_check_stall_ms >= INTERVALO_CHECK_STALL_MS) {
                unsigned long delta_esq_stall = contador_esq_A - ultima_contagem_esq_stall;
                unsigned long delta_dir_stall = contador_dir_A - ultima_contagem_dir_stall;
                ultima_contagem_esq_stall = contador_esq_A;
                ultima_contagem_dir_stall = contador_dir_A;
                ultimo_check_stall_ms = millis();

                if (delta_esq_stall < PULSOS_MINIMOS_NAO_TRAVADO || delta_dir_stall < PULSOS_MINIMOS_NAO_TRAVADO) {
                    if (ws.count() > 0) ws.textAll("{\"aviso\":\"esteira patinando na re, reforcando torque\"}");
                    mover_motores(-VELOCIDADE_MAXIMA, -VELOCIDADE_MAXIMA);
                    delay(DURACAO_KICK_ANTISTALL_MS);
                    mover_motores(-VELOCIDADE_EXPLORACAO, -VELOCIDADE_EXPLORACAO);
                }
            }

            // --- CONCLUSÃO DA RÉ PELA MÉDIA DOS DOIS ENCODERS ---
            // Antes usava só contador_esq_A, presumindo as duas esteiras andando
            // em sincronia — a mesma suposição frágil que causava giros
            // incompletos. A média dos dois é a estimativa correta de quanto o
            // CENTRO do robô andou, mesmo que uma esteira tenha avançado mais
            // que a outra.
            long delta_esq_re = (long)(contador_esq_A - posicao_inicial_explora);
            long delta_dir_re = (long)(contador_dir_A - posicao_inicial_dir_explora);
            float pulsos_percorridos_re = (fabs((float)delta_esq_re) + fabs((float)delta_dir_re)) / 2.0;

            if (pulsos_percorridos_re >= alvoPulsosExplora) {
                parar_motores(); delay(250); 
                
                // Sorteia a direção do giro (1 para Esquerda, -1 para Direita)
                sentido_giro_atual = (random(0, 2) == 0) ? 1 : -1;
                
                // "Kickstart": um pulso curto em potência máxima antes de assentar
                // na velocidade normal de giro. Partindo do zero, o atrito estático
                // (stiction) de uma esteira pode ser maior que o da outra — com os
                // dois lados na mesma potência "de cruzeiro", às vezes só a esteira
                // com menos atrito solta e gira, e a outra fica presa derrapando no
                // lugar (a causa provável do robô não completar o giro pedido).
                // O pulso de potência máxima ajuda a vencer esse atrito nos dois
                // lados antes de reduzir para a velocidade de giro sustentada.
                mover_motores(-VELOCIDADE_MAXIMA * sentido_giro_atual, VELOCIDADE_MAXIMA * sentido_giro_atual);
                delay(DURACAO_KICK_ANTISTALL_MS);
                mover_motores(-VELOCIDADE_GIRO * sentido_giro_atual, VELOCIDADE_GIRO * sentido_giro_atual); 
                
                estadoExplora = EXPLORA_GIRO; 
                posicao_inicial_explora = contador_esq_A; 
                posicao_inicial_dir_explora = contador_dir_A;
                giro_acumulado_rad = 0.0;
                alvoAnguloGiroRad = PI / 2.0; // 90 graus
                tempo_inicio_manobra_explora = millis(); 
                ultimo_check_stall_ms = millis();
                ultima_contagem_esq_stall = contador_esq_A;
                ultima_contagem_dir_stall = contador_dir_A;
                Serial.printf("[GIRO] INICIO episodio: alvo=90 graus | sentido=%d | CM_POR_PULSO=%.4f LARGURA_ESTEIRA_EFETIVA=%.3f\n",
                              sentido_giro_atual, CM_POR_PULSO, LARGURA_ESTEIRA_EFETIVA);
            }
        } 
        else if (estadoExplora == EXPLORA_GIRO) {

            // --- VIGIA ANTI-DERRAPAGEM ---
            // Se uma das esteiras não acumulou pulsos suficientes num intervalo
            // curto enquanto a outra girou normalmente, essa esteira está
            // patinando parada em vez de girar — exatamente o sintoma relatado
            // ("às vezes ele só movimenta um dos motores"). Detectado isso,
            // aplica-se um novo empurrão de potência máxima pra tentar destravá-la.
            if (millis() - ultimo_check_stall_ms >= INTERVALO_CHECK_STALL_MS) {
                unsigned long delta_esq_stall = contador_esq_A - ultima_contagem_esq_stall;
                unsigned long delta_dir_stall = contador_dir_A - ultima_contagem_dir_stall;
                ultima_contagem_esq_stall = contador_esq_A;
                ultima_contagem_dir_stall = contador_dir_A;
                ultimo_check_stall_ms = millis();

                if (delta_esq_stall < PULSOS_MINIMOS_NAO_TRAVADO || delta_dir_stall < PULSOS_MINIMOS_NAO_TRAVADO) {
                    if (ws.count() > 0) ws.textAll("{\"aviso\":\"esteira patinando no giro, reforcando torque\"}");
                    Serial.printf("[GIRO][KICK] esteira patinando (delta_esq=%lu delta_dir=%lu em %lums) - reforcando torque. giro_acumulado=%.1f graus\n",
                                  delta_esq_stall, delta_dir_stall, INTERVALO_CHECK_STALL_MS, giro_acumulado_rad * 180.0 / PI);
                    mover_motores(-VELOCIDADE_MAXIMA * sentido_giro_atual, VELOCIDADE_MAXIMA * sentido_giro_atual);
                    delay(DURACAO_KICK_ANTISTALL_MS);
                    mover_motores(-VELOCIDADE_GIRO * sentido_giro_atual, VELOCIDADE_GIRO * sentido_giro_atual);
                }
            }

            // --- DEBUG: progresso do giro em graus, ~5x/s ---
            // Pulsos brutos de cada encoder desde o início DESTE segmento, e a
            // conversão giro_acumulado_rad->graus, lado a lado. Serve pra
            // calibrar LARGURA_ESTEIRA_EFETIVA/CM_POR_PULSO com dados reais:
            // meça o giro físico de fato (ex. com um transferidor) e compare
            // com "acumulado" no momento em que o robô parar.
            static uint32_t timeout_debug_giro = 0;
            if (millis() > timeout_debug_giro) {
                long delta_esq_dbg = (long)(contador_esq_A - posicao_inicial_explora);
                long delta_dir_dbg = (long)(contador_dir_A - posicao_inicial_dir_explora);
                Serial.printf("[GIRO] acumulado=%.1f graus | alvo=%.1f graus | pulsos_esq=%ld pulsos_dir=%ld | sentido=%d | t_segmento=%lums\n",
                              giro_acumulado_rad * 180.0 / PI, alvoAnguloGiroRad * 180.0 / PI,
                              delta_esq_dbg, delta_dir_dbg, sentido_giro_atual,
                              millis() - tempo_inicio_manobra_explora);
                timeout_debug_giro = millis() + 200;
            }

            // --- CONCLUSÃO DO GIRO PELO HEADING MEDIDO (theta_rad), NÃO POR PULSO DE UMA RODA SÓ ---
            // theta_rad é atualizado a partir da diferença entre os DOIS encoders
            // (ver atualizar_odometria_fusao), então já reflete corretamente o
            // quanto o robô girou de fato — mesmo que uma esteira tenha girado
            // mais que a outra durante a manobra.
            // giro_acumulado_rad soma o delta bruto a cada atualização de
            // odometria (ver atualizar_odometria_fusao) e é zerado no início
            // deste segmento — sem wraparound, então nenhuma volta inteira
            // escondida dentro de um kick anti-derrapagem passa despercebida.
            float girado_rad = fabs(giro_acumulado_rad);

            if (girado_rad >= alvoAnguloGiroRad) {
                Serial.printf("[GIRO] SEGMENTO CONCLUIDO: acumulado=%.1f graus (alvo era %.1f) | pulsos_esq=%ld pulsos_dir=%ld | t_segmento=%lums\n",
                              girado_rad * 180.0 / PI, alvoAnguloGiroRad * 180.0 / PI,
                              (long)(contador_esq_A - posicao_inicial_explora),
                              (long)(contador_dir_A - posicao_inicial_dir_explora),
                              millis() - tempo_inicio_manobra_explora);
                parar_motores(); delay(100); giros_consecutivos++;

                bool ainda_obstaculo = ((distancia > 0) && (distancia <= DISTANCIA_EXPLORACAO));
                bool ainda_queda = (analogRead(SENSOR_LINHA_ESQUERDO) > LIMIAR_QUEDA) || (analogRead(SENSOR_LINHA_DIREITO) > LIMIAR_QUEDA);

                if (ainda_obstaculo || ainda_queda) {
                    // Prevenção de bloqueio: se girar demais e não sair, ele para.
                    // Total de rotação até desistir = 90° (1ª tentativa) + 45° por
                    // tentativa adicional. Com MAX_GIROS_CONSECUTIVOS=3, o teto é
                    // 90+45+45=180° — no máximo meia-volta — em vez das 405° (mais
                    // de uma volta completa) que o limite antigo de 8 permitia.
                    if (giros_consecutivos >= MAX_GIROS_CONSECUTIVOS) {
                        estadoExplora = EXPLORA_PARADO; if (ws.count() > 0) ws.textAll("{\"stuck\":true}"); 
                    } else {
                        // Mantém a direção sorteada e tenta girar mais um pouco para se livrar do obstáculo
                        mover_motores(-VELOCIDADE_MAXIMA * sentido_giro_atual, VELOCIDADE_MAXIMA * sentido_giro_atual);
                        delay(DURACAO_KICK_ANTISTALL_MS);
                        mover_motores(-VELOCIDADE_GIRO * sentido_giro_atual, VELOCIDADE_GIRO * sentido_giro_atual); 
                        posicao_inicial_explora = contador_esq_A; 
                        posicao_inicial_dir_explora = contador_dir_A;
                        giro_acumulado_rad = 0.0;
                        alvoAnguloGiroRad = PI / 4.0; // 45 graus
                        tempo_inicio_manobra_explora = millis();
                        ultimo_check_stall_ms = millis();
                        ultima_contagem_esq_stall = contador_esq_A;
                        ultima_contagem_dir_stall = contador_dir_A;
                        if (ainda_queda && !status_queda_ui_explora) { if (ws.count() > 0) ws.textAll("{\"fall\":true}"); status_queda_ui_explora = true; }
                    }
                } else {
                    // Caminho livre, volta a navegar em frente
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
        posicao_inicial_explora = contador_esq_A;
        posicao_inicial_dir_explora = contador_dir_A;
        alvoPulsosExplora = PULSOS_20_CM; tempo_inicio_manobra_explora = millis();
        ultimo_check_stall_ms = millis();
        ultima_contagem_esq_stall = contador_esq_A;
        ultima_contagem_dir_stall = contador_dir_A;
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

  //bool obstaculo_frente = wp_evita_colisao && (((distancia > 0) && (distancia <= DISTANCIA_OBSTACULO)) || obstaculo_virtual_detectado(DISTANCIA_OBSTACULO));
  bool obstaculo_frente = wp_evita_colisao && ((distancia > 0) && (distancia <= DISTANCIA_OBSTACULO));
  bool queda_detectada = wp_evita_queda && ((analogRead(SENSOR_LINHA_ESQUERDO) > LIMIAR_QUEDA) || (analogRead(SENSOR_LINHA_DIREITO) > LIMIAR_QUEDA));

  if (obstaculo_frente || queda_detectada) {
      parar_motores();
      if (ws.count() > 0) ws.textAll("{\"waypoint_status\":\"Bloqueado (Queda/Obstáculo)!\"}");
      return;
  }

  // FASE 1: CALIBRAÇÃO DA BÚSSOLA GEOMÉTRICA
  if (!heading_gps_valido) {
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
  
  float flat, flon; unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  
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

  // Tolerância relaxada para 50 graus. Ele aceita andar "torto" e corrigir em movimento.
  if (abs(erro_angular) > 50.0) {
      if (estado_motor_anterior == 0) { parar_motores(); delay(250); tempo_inicio_manobra_way = millis(); }
      estado_motor_anterior = 1;

      // ANTI-STALL
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
          mover_motores(VELOCIDADE_MAXIMA, VELOCIDADE_MAXIMA); // Avanço com potência total para vencer o piso
          delay(EMPURRAO_ANTISTALL_WAYPOINT_MS);
          parar_motores(); delay(150);
          estado_motor_anterior = 0;
          tempo_inicio_manobra_way = millis();
          return;
      }

      int sentido = (erro_angular > 0) ? 1 : -1;
      // Reduzida a potência do giro no eixo (de 85 para 65) para as esteiras "morderem" o chão sem pular
      mover_motores(65 * sentido, -65 * sentido); 
  } else {
      if (estado_motor_anterior == 1) { parar_motores(); delay(250); }
      estado_motor_anterior = 0;
      giros_consecutivos_way = 0;

      int vel_esq = VELOCIDADE_EXPLORACAO; int vel_dir = VELOCIDADE_EXPLORACAO; 
      
      // Curvas dinâmicas muito mais agressivas para compensar a folga dos 50 graus iniciais
      if (erro_angular > 12.0) { 
          vel_dir = 35; // Freia bem mais a esteira de dentro da curva
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
  // dt real entre chamadas, para I e D terem significado físico (e não dependerem
  // de quantas vezes por segundo o loop() consegue chamar segue_linha()).
  unsigned long agora = millis();
  float dt = (pid_tempo_anterior == 0) ? 0.02 : (agora - pid_tempo_anterior) / 1000.0;
  if (dt <= 0.0001) dt = 0.0001; // proteção contra divisão por ~zero
  pid_tempo_anterior = agora;

  P = erro;
  I = I + (erro * dt);
  if (I > 50) I = 50; else if (I < -50) I = -50;
  if (fabs(erro) < 0.05) I = 0; // zona morta: robô centralizado zera o acumulado

  D = (erro - erro_anterior) / dt;
  if (D > 10.0) D = 10.0; else if (D < -10.0) D = -10.0; // evita picos do derivativo com dt muito pequeno

  resposta_PID = (Kp * P) + (Ki * I) + (Kd * D);
  erro_anterior = erro;

  // --- TRAVA DE SEGURANÇA ---
  // Nunca deixa a resposta do PID ser forte a ponto de inverter o sentido de uma
  // roda (isso é o que fazia o robô girar sobre o próprio eixo em vez de curvar).
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
          digitalWrite(PINO_LED, LOW); modo_linha = false; modo_explora = false; modo_waypoints = false; parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE; total_waypoints = 0;
        } break;
    case WS_EVT_DATA: handleWebSocketMessage(arg, data, length); break;
    case WS_EVT_PONG: case WS_EVT_ERROR: break;
  }
}


// Span mínimo entre o valor mais claro e o mais escuro vistos durante o giro
// de calibração. Abaixo disso, a roleta de leitura não cruzou de fato a fita
// (ex.: calibração feita inteira sobre o piso claro) e o limiar resultante
// não representa a fronteira real fita/piso.
const int LIMIAR_LINHA_SPAN_MINIMO = 500;

void calibrarSensoresLinha() {
    modo_linha = false; modo_explora = false; modo_waypoints = false; parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE; total_waypoints = 0;

    // Média de várias amostras (em vez de uma leitura instantânea) para reduzir
    // o efeito do ruído do ADC nesse valor de referência.
    long somaFundo = 0; const int N_AMOSTRAS_FUNDO = 10;
    for (int i = 0; i < N_AMOSTRAS_FUNDO; i++) {
        somaFundo += (analogRead(SENSOR_LINHA_ESQUERDO) + analogRead(SENSOR_LINHA_DIREITO)) / 2;
        delay(5);
    }
    int fundoInicial = somaFundo / N_AMOSTRAS_FUNDO;

    int minVal = 4095; int maxVal = 0; uint32_t inicio = millis();

    while (millis() - inicio < 2500) {
        if (((millis() - inicio) / 250) % 2 == 0) mover_motores(60, -60); else mover_motores(-60, 60);
        int leituraEsq = analogRead(SENSOR_LINHA_ESQUERDO); int leituraDir = analogRead(SENSOR_LINHA_DIREITO);
        if (leituraEsq < minVal) minVal = leituraEsq; if (leituraDir < minVal) minVal = leituraDir;
        if (leituraEsq > maxVal) maxVal = leituraEsq; if (leituraDir > maxVal) maxVal = leituraDir;
        delay(10);
    }
    parar_motores();

    int span = maxVal - minVal;
    int novoLimiar = (minVal + maxVal) / 2;

    // 1) O giro precisa ter cruzado fita e piso com contraste real. Se o span
    //    for pequeno, a calibração provavelmente foi feita inteira sobre uma
    //    única superfície (ex.: só piso claro, sem a fita embaixo) — nesse caso
    //    NÃO sobrescrevemos a calibração/persistência anterior.
    // 2) A leitura inicial (fundoInicial) precisa cair claramente para um dos
    //    lados do novo limiar. Se estiver perto demais do meio do range, ela
    //    não é confiável para decidir a polaridade (linha_escura) — sinal de
    //    que o robô não começou de fato sobre a fita.
    bool span_ok = (span >= LIMIAR_LINHA_SPAN_MINIMO);
    bool fundo_ambiguo = abs(fundoInicial - novoLimiar) < (span / 4);

    if (!span_ok || fundo_ambiguo) {
        Serial.printf("[CALIBRACAO] REJEITADA: min=%d max=%d span=%d fundoInicial=%d limiar_calc=%d "
                      "(span_ok=%d fundo_ambiguo=%d). Posicione o robo COM os sensores sobre a fita "
                      "antes de iniciar a calibracao.\n",
                      minVal, maxVal, span, fundoInicial, novoLimiar, span_ok, fundo_ambiguo);

        JsonDocument json;
        json["status"] = "calibracao_falhou";
        json["motivo"] = !span_ok ? "contraste_insuficiente" : "fundo_ambiguo";
        json["min"] = minVal; json["max"] = maxVal; json["span"] = span; json["fundoInicial"] = fundoInicial;
        size_t msg_comp = measureJson(json); char msg[msg_comp + 1]; serializeJson(json, msg, (msg_comp + 1)); msg[msg_comp] = 0;
        if (ws.count() > 0) ws.textAll(msg, msg_comp);
        return;
    }

    limiarLinha = novoLimiar;
    linha_escura = (fundoInicial < limiarLinha);

    Serial.printf("[CALIBRACAO] OK: min=%d max=%d span=%d fundoInicial=%d limiar=%d linha_escura=%d\n",
                  minVal, maxVal, span, fundoInicial, limiarLinha, linha_escura);

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
                modoSegurancaBateria = true; modo_linha = false; modo_explora = false; modo_waypoints = false; parar_motores(); estadoManobraAtual = LIVRE; estadoExplora = EXPLORA_LIVRE; total_waypoints = 0;
                if (ws.count() > 0) ws.textAll("{\"alert\":\"LOW_BATTERY\"}");
            }
            digitalWrite(PINO_LED, (millis() / 150) % 2 ? HIGH : LOW);
        }
    } 
    else if (tensao_mv >= (TENSAO_CRITICA + 300)) { 
        leituras_criticas_consecutivas = 0; modoSegurancaBateria = false; digitalWrite(PINO_LED, LOW);
    }
}