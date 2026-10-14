import asyncio
import websockets
import json
import pygame
import math

# ==========================================
# CONFIGURAÇÕES DA ESTAÇÃO BASE
# ==========================================
ROBOT_IP = "192.168.4.1"  # Mude para o IP do seu Robô se estiver no Wi-Fi de casa
WS_URI = f"ws://{ROBOT_IP}/ws"

# Configurações de Tela e Grid
WIDTH, HEIGHT = 800, 800
RESOLUTION_CM = 5      # 1 célula = 5cm reais
CELL_SIZE = 8          # Tamanho da célula em pixels na tela do Mac
GRID_OFFSET_X = WIDTH // 2
GRID_OFFSET_Y = HEIGHT // 2

# Cores (RGB)
COLOR_BG = (20, 20, 25)
COLOR_GRID = (40, 40, 45)
COLOR_OBSTACLE = (255, 60, 60)      # Vermelho para objetos confirmados
COLOR_FREE = (30, 40, 30)           # Verde escuro para área livre mapeada
COLOR_ROBOT = (50, 255, 100)
COLOR_RAY = (200, 200, 50)

# Lógica de Probabilidade (0 a 100)
HIT_SCORE = 25          # Pontos somados ao bater em um obstáculo
MISS_SCORE = 15         # Pontos subtraídos ao confirmar que o espaço está livre
THRESHOLD_OCCUPIED = 70 # Acima disso, pintamos de vermelho
THRESHOLD_FREE = 30     # Abaixo disso, pintamos o fundo de livre

# Estado Global do Grid de Probabilidades
occupancy_grid = {} # Dicionário: {(x, y): score}
robot_state = {"x": 0.0, "y": 0.0, "theta": 0.0, "dist": 0.0}

def bresenham_line(x0, y0, x1, y1):
    """ Calcula todas as células da matriz que são atravessadas pelo feixe do sensor. """
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells

def world_to_screen(x_cm, y_cm):
    """ Converte as coordenadas reais (cm) para pixels da tela do Pygame. """
    px = int(GRID_OFFSET_X + (x_cm / RESOLUTION_CM) * CELL_SIZE)
    py = int(GRID_OFFSET_Y - (y_cm / RESOLUTION_CM) * CELL_SIZE)
    return px, py

async def receive_telemetry():
    """ Conecta ao WebSocket do ESP32 e atualiza as probabilidades do mapa. """
    print(f"Tentando conectar ao Robô em {WS_URI}...")
    try:
        async with websockets.connect(WS_URI, ping_interval=None) as websocket:
            print("Conexão estabelecida! Processando Raycasting...")
            while True:
                message = await websocket.recv()
                try:
                    data = json.loads(message)
                    if "map_raw_x" in data:
                        robot_state["x"] = data["map_raw_x"]
                        robot_state["y"] = data["map_raw_y"]
                        robot_state["theta"] = data["map_raw_theta"]
                        robot_state["dist"] = data["map_raw_dist"]
                        
                        dist = robot_state["dist"]
                        
                        # Processa apenas se for uma leitura válida e próxima
                        if 0 < dist < 150:
                            # 1. Posição do robô na matriz
                            robot_grid_x = int(robot_state["x"] / RESOLUTION_CM)
                            robot_grid_y = int(robot_state["y"] / RESOLUTION_CM)
                            
                            # 2. Posição do obstáculo detectado na matriz
                            obs_x = robot_state["x"] + dist * math.cos(robot_state["theta"])
                            obs_y = robot_state["y"] + dist * math.sin(robot_state["theta"])
                            obs_grid_x = int(obs_x / RESOLUTION_CM)
                            obs_grid_y = int(obs_y / RESOLUTION_CM)
                            
                            # 3. Traça a linha de visão do robô
                            line_cells = bresenham_line(robot_grid_x, robot_grid_y, obs_grid_x, obs_grid_y)
                            
                            # 4. Decaimento: Células intermediárias ganham status "Livre" (subtrai pontos)
                            for cell in line_cells[:-1]:
                                current_score = occupancy_grid.get(cell, 50)
                                occupancy_grid[cell] = max(0, current_score - MISS_SCORE)
                                
                            # 5. Confirmação: Célula final ganha status "Ocupada" (soma pontos)
                            obs_cell = line_cells[-1]
                            current_score = occupancy_grid.get(obs_cell, 50)
                            occupancy_grid[obs_cell] = min(100, current_score + HIT_SCORE)
                            
                except json.JSONDecodeError:
                    pass
    except Exception as e:
        print(f"Erro na conexão WebSocket: {e}")

async def main_loop():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Arwen - Mapeamento de Ocupação com Raycasting")
    clock = pygame.time.Clock()

    asyncio.create_task(receive_telemetry())

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # --- EVENTO PARA SALVAR O MAPA NO HD ---
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    print("Salvando mapa no Mac...")
                    # JSON não aceita tuplas como chave, então convertemos (x,y) para string "x,y"
                    grid_para_salvar = {f"{x},{y}": score for (x, y), score in occupancy_grid.items()}
                    
                    try:
                        with open("mapa_salvo.json", "w") as f:
                            json.dump(grid_para_salvar, f, indent=4)
                        print("Mapa salvo com sucesso em 'mapa_salvo.json'!")
                    except Exception as e:
                        print(f"Erro ao salvar o mapa: {e}")

        screen.fill(COLOR_BG)

        for x in range(0, WIDTH, CELL_SIZE):
            pygame.draw.line(screen, COLOR_GRID, (x, 0), (x, HEIGHT))
        for y in range(0, HEIGHT, CELL_SIZE):
            pygame.draw.line(screen, COLOR_GRID, (0, y), (WIDTH, y))

        # Desenha o Grid Baseado nas Probabilidades
        for (gx, gy), score in occupancy_grid.items():
            px = GRID_OFFSET_X + gx * CELL_SIZE
            py = GRID_OFFSET_Y - gy * CELL_SIZE
            
            if score >= THRESHOLD_OCCUPIED:
                # Obstáculo confirmado (Vermelho)
                pygame.draw.rect(screen, COLOR_OBSTACLE, (px, py, CELL_SIZE, CELL_SIZE))
            elif score <= THRESHOLD_FREE:
                # Espaço limpo e explorado (Verde Escuro)
                pygame.draw.rect(screen, COLOR_FREE, (px, py, CELL_SIZE, CELL_SIZE))

        rx, ry = world_to_screen(robot_state["x"], robot_state["y"])

        dist = robot_state["dist"]
        if 0 < dist < 150:
            ray_x, ray_y = world_to_screen(
                robot_state["x"] + dist * math.cos(robot_state["theta"]),
                robot_state["y"] + dist * math.sin(robot_state["theta"])
            )
            pygame.draw.line(screen, COLOR_RAY, (rx, ry), (ray_x, ray_y), 1)

        pygame.draw.circle(screen, COLOR_ROBOT, (rx, ry), CELL_SIZE)
        
        end_x = rx + int(CELL_SIZE * 2 * math.cos(robot_state["theta"]))
        end_y = ry - int(CELL_SIZE * 2 * math.sin(robot_state["theta"])) 
        pygame.draw.line(screen, (255, 255, 255), (rx, ry), (end_x, end_y), 2)

        pygame.display.flip()
        await asyncio.sleep(0.01)
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    asyncio.run(main_loop())