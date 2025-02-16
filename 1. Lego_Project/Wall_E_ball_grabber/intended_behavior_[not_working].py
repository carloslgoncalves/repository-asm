
#!/usr/bin/env pybricks-micropython

import math
import time

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import UltrasonicSensor as UltSensor
from pybricks.parameters import Port, Direction, Button, Stop
from pybricks.tools import wait, StopWatch,DataLog
from pybricks.robotics import DriveBase

# Initializa o EV3 Brick.
ev3 = EV3Brick()

# Initializa o Ultrasonic Sensor. Um é utilizado para ler as paredes do dominio
# O outro ler ao nivel do solo para detetar a bola
wall_sensor = UltSensor(Port.S3)
ball_sensor = UltrasonicSensor(Port.S1)


GYRO_CALIBRATION_LOOP_COUNT = 200  # Loop de Calibração
GYRO_OFFSET_FACTOR = 0.0005        # Offset do gyroscópio
gyro=GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)

# Initialize two motors with default settings on Port B and Port C.
# These will be the left and right motors of the drive base.
right_motor = Motor(Port.B)
left_motor = Motor(Port.C)
claw_motor = Motor(Port.D)

states={0:"Inicialise" , 1:"Define target" , 2:"Path finder" , 3:"Target atcheaved" , 4:"System reset"}
currentState = 0

# O diâmetro da roda (wheel_diameter) e a distância entre os eixos (axle_track)
# são usados para calcular a velocidade correta dos motores ao executar comandos de movimento.
# A distância entre eixos (axle_track) é a distância entre os pontos onde as rodas
# tocam o chão.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=110.5)  # Definindo as especificações do robô
robot.settings(100, 100, 100, 100)  # Define a velocidade e aceleração para movimentos em linha reta e curvas

# Comandos comentados para movimentação
# robot.straight(500)  # Faz o robô mover-se para frente em linha reta por 500 mm
# robot.turn(360)      # Faz o robô girar 360 graus no lugar

# Exibe a configuração do PID para controle de distância e direção
# Tentativa de alterar estes valores para melhor resultados, nenhuma informação
# encontrada sobre a alteração destes valores e os seus resultados
print(robot.distance_control.pid())
print(robot.heading_control.pid())


# Dimensões da área de trabalho (campo)
Feald_dimensions = (1120, 708)  # Largura e altura em milímetros

# Definir o offset dos sensores em relação às rodas
ofset_wall_sensor = -60  # Distância do sensor de parede em mm
ofset_ball_sensor = +100  # Distância do sensor de bola em mm

# Cálculo da distância máxima da área de trabalho com base nas dimensões
max_measure = (Feald_dimensions[0]**2 + Feald_dimensions[1]**2)**0.5

# Coordenadas iniciais e ângulo do robô
pos_x = 210  # Posição x inicial
pos_y = 180  # Posição y inicial
ang = 0      # Ângulo inicial do robô (0 a 359 graus, sentido anti-horário)

# Estados iniciais
have_ball = False       # Indica se o robô possui uma bola na garra
ball_locat = []         # Lista de possíveis localizações da bola
at_origin = False       # Indica se o robô está na posição inicial

# Variáveis para calibração do giroscópio
gyro_sum = 0                    # Soma das leituras do giroscópio
gyro_min, gyro_max = 440, -440  # Valores iniciais extremos para min e max

# Coleta de dados para compensação inicial de desvio do giroscópio
for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
    gyro_speed = gyro.speed()  # Leitura da velocidade do giroscópio
    gyro_sum += gyro_speed     # Acumula a leitura para cálculo da média
    gyro_min = min(gyro_min, gyro_speed)  # Atualiza o valor mínimo se necessário
    gyro_max = max(gyro_max, gyro_speed)  # Atualiza o valor máximo se necessário
    wait(5)  # Pausa de 5 milissegundos entre leituras

# Define a compensação inicial como a taxa média de desvio
gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

# Emite um som para indicar que o robô está pronto para iniciar o movimento
ev3.speaker.beep()


# Função para realizar medições usando um dispositivo ultrasonico e aplicar um filtro para obter uma média ajustada
def make_mesure(device, ofset_wall_sensor=ofset_wall_sensor, ofset_ball_sensor=ofset_ball_sensor):
    mesures = []
    
    # Realiza 10 medições com o dispositivo e armazena os valores
    for n in range(10):
        mesures.append(device.distance())
        wait(100)  # Pausa de 100 ms entre as medições
    
    # Calcula a média das medições
    mean = sum(mesures) / len(mesures)

    
    # Calcula a variância e o desvio padrão das medições
    sum_squared_diff = 0
    for value in mesures:
        sum_squared_diff += (value - mean) ** 2
    variance = sum_squared_diff / len(mesures)
    std_dev = variance ** 0.5  # O desvio padrão é a raiz quadrada da variância
    
    # Filtra as medições que estão além de um limite de desvio padrão
    filtered_measures = []
    threshold = 2  # Define o limite de 2 desvios padrão
    for value in mesures:
        if abs(value - mean) <= threshold * std_dev:
            filtered_measures.append(value)
    
    # Calcula a média ajustada das medições filtradas
    if filtered_measures:
        adjusted_average = sum(filtered_measures) / len(filtered_measures)
    else:
        adjusted_average = None  # Nenhuma medida válida após o filtro

    # Se a média ajustada exceder a medida máxima, calcula a distância corrigida
    if adjusted_average >= max_measure:
        adjusted_average = calculate_distance(pos_x, pos_y, get_corrected_angle())

    # Retorna o valor ajustado, adicionando o offset do sensor de bola ou parede
    if device == ball_sensor:
        return adjusted_average + ofset_ball_sensor
    elif device == wall_sensor:
        return adjusted_average + ofset_wall_sensor

# Função para normalizar o ângulo entre 0 e 359 graus no sentido anti-horário
def norm_ang(ang):
    if abs(ang) >= 360 or ang < 0:
        n = abs(ang) // 360  # Calcula o número de voltas completas de 360 graus
        if abs(ang) % 360 == 0:
            ang = 0
        elif ang >= 0:
            ang = ang - 360 * n
        elif ang < 0:
            ang = ang + (360 * (n + 1))
    else:
        ang = ang
    return ang

# Função para calcular a distância até as bordas do campo, a partir da posição e ângulo fornecidos
def calculate_distance(X_p, Y_p, theta, X_m=Feald_dimensions[0], Y_m=Feald_dimensions[1]):
    # Converte o ângulo para radianos
    theta_rad = math.radians(theta)
    cos_theta = round(math.cos(theta_rad), 5)
    sin_theta = round(math.sin(theta_rad), 5)
    
    # Lista para armazenar as distâncias válidas até as bordas
    distances = []
    
    # Verifica a interseção com a borda esquerda (x = 0)
    if cos_theta != 0:  # Evita divisão por zero
        t_left = -X_p / cos_theta
        y_left = Y_p + t_left * sin_theta
        if t_left > 0 and 0 <= y_left <= Y_m:
            distances.append(t_left)
    
    # Verifica a interseção com a borda direita (x = X_m)
    if cos_theta != 0:  # Evita divisão por zero
        t_right = (X_m - X_p) / cos_theta
        y_right = Y_p + t_right * sin_theta
        if t_right > 0 and 0 <= y_right <= Y_m:
            distances.append(t_right)
    
    # Verifica a interseção com a borda inferior (y = 0)
    if sin_theta != 0:  # Evita divisão por zero
        t_bottom = -Y_p / sin_theta
        x_bottom = X_p + t_bottom * cos_theta
        if t_bottom > 0 and 0 <= x_bottom <= X_m:
            distances.append(t_bottom)
            
    # Verifica a interseção com a borda superior (y = Y_m)
    if sin_theta != 0:  # Evita divisão por zero
        t_top = (Y_m - Y_p) / sin_theta
        x_top = X_p + t_top * cos_theta
        if t_top > 0 and 0 <= x_top <= X_m:
            distances.append(t_top)
    
    # Retorna a menor distância positiva (interseção mais próxima)
    if distances:
        return round(min(distances), 3)
    else:
        return None  # Nenhuma interseção válida encontrada

#  Função para calcular o ângulo corrigido levando em conta o drift do giroscópio
## por algum motivo o drift era demasiado inconstante apesar da compensação do drift gradual
def get_corrected_angle():
    global gyro_offset
    
    # Ajuste dinâmico do offset do giroscópio para compensar o drift gradual
    gyro_speed = gyro.speed()
    gyro_offset = (1 - GYRO_OFFSET_FACTOR) * gyro_offset + GYRO_OFFSET_FACTOR * gyro_speed

    # Calcula o ângulo corrigido
    corrected_angle = gyro.angle() - gyro_offset
    return corrected_angle



# Função para inicializar o robô e calibrar os sensores
def inicialise():
    
    # Move o motor da garra até ele ficar "preso" (posição inicial)
    # 60: Velocidade do motor em graus por segundo
    # Stop.HOLD: Mantém o motor travado após atingir a posição
    # 40: Limite de força para detectar a posição travada
    claw_motor.run_until_stalled(60, Stop.HOLD, 40)
    
    # Leitura inicial do giroscópio e início da contagem de tempo
    initial_angle = gyro.angle()
    
    # Define a variável global `start_time` para armazenar o tempo de início
    global start_time
    start_time = time.time()  # Marca o tempo inicial em segundos

    # Mede a taxa de desvio do giroscópio ao longo de alguns segundos
    wait(10000)  # Aguarda 10 segundos para estabilizar a leitura do giroscópio
    
    # Define a variável global `drift_rate` para a taxa de desvio
    global drift_rate
    # Calcula a taxa de desvio do giroscópio
    drift_rate = (gyro.angle() - initial_angle) / (time.time() 



# Define a função "define_target" que calcula as coordenadas alvo (target_x, target_y) com base no ciclo, posição do robô e presença de bola.
def define_target(cycle, pos_x, pos_y):
    print("in define_target")  # Indica que a função foi chamada
    target_x = 0
    target_y = 0

    # Verifica se o robô não tem a bola e está no primeiro ciclo
    if have_ball == False and cycle == 1:  
        print("if have_ball==False and cycle==1:")
        frst_mesure = make_mesure(wall_sensor)  # Toma a primeira medição do sensor de parede
    
        # Realiza 12 iterações para ajustar a orientação do robô com base na proximidade de uma parede
        for i in range(12):
            new_mesure = make_mesure(wall_sensor)  # Toma uma nova medição do sensor de parede
            print(new_mesure)
            
            # Se a nova medição exceder a medida máxima permitida, o loop é interrompido
            if new_mesure >= max_measure:
                break
            
            # Se a nova medição for maior que a primeira, o robô vira 5 graus à esquerda
            elif frst_mesure < new_mesure:
                robot.turn(5)
                print(new_mesure)
                frst_mesure = new_mesure
            
            # Se a nova medição for menor que a primeira (ou na primeira iteração), o robô vira 5 graus à direita
            elif frst_mesure > new_mesure or i == 0:
                robot.turn(-5)
            wait(100)  # Espera 100 ms antes de fazer a próxima medição
            
        # Calcula o ângulo corrigido e normaliza-o
        ang = get_corrected_angle()
        ang = norm_ang(ang)
        
        print("ang=", ang)  # Mostra o ângulo corrigido
        # Calcula as coordenadas alvo com base na posição atual e no ângulo
        # Coordenadas do centro do campo medido
        target_x = pos_x + (math.cos(math.radians(ang)) * new_mesure) / 2
        target_y = pos_y + (math.sin(math.radians(ang)) * new_mesure) / 2

        # Retorna as coordenadas alvo como uma lista [target_x, target_y]
        return [target_x, target_y]
        

    # Caso o robô já tenha a bola
    elif have_ball == True:
        print("reajusting")
        # Reajusta a orientação do robô
        atual_ang = get_corrected_angle()
        robot.turn(0 - atual_ang)  # Gira o robô para alinhar a 0 graus
        reajust = []  # Lista para armazenar medições de reajuste

        # Realiza 4 medições enquanto gira 90 graus, para determinar a posição ajusatda do robô
        reajust.append(make_mesure(wall_sensor))
        for k in range(3):
            robot.turn(90)
            reajust.append(make_mesure(wall_sensor))

        # Calcula as novas coordenadas da posição do robô com base nas medições
        pos_x = (reajust[0] + reajust[2]) / 2
        pos_y = (reajust[1] + reajust[3]) / 2
        ang = get_corrected_angle()
        ang = norm_ang(ang)
        
        # Retorna uma posição de largada da bola, [200, 200]
        return [200, 200]

    # Caso a bola ainda não tenha sido localizada
    elif not ball_locat:
        print("locating ball")
        claw_motor.run_until_stalled(-50, Stop.HOLD, 40)  # Aciona o motor da garra até encontrar resistência de modo a 
                                                          # evitar interferencia com o sensor ultrasónico durante as leituras
        min_dis = 0                                       

        # Enquanto a bola não é localizada, o robô roda e mede distâncias
        while not ball_locat:
            robot.turn(5)  # Roda o robô em intervalos de 5 graus
            wait(100)      # Esperam 100ms de modo evitar sobreposição de sinais
            dis = make_mesure(ball_sensor)  # Mede a distância ao objeto com o sensor da bola
            
            # Verifica se a distância medida indica que a bola está mais próxima do que a parede com 5% de erro
            if dis < make_mesure(wall_sensor) * (1 + 0.05):
                print("mesured distance ", dis)
                print("Wall dist", make_mesure(wall_sensor))
                ang = get_corrected_angle()
                ang = norm_ang(ang)

                # Se a distância medida for menor que 85% da distância calculada, regista a posição como uma bola
                if dis < calculate_distance(pos_x, pos_y, ang) * 0.85:
                    # Calcula as coordenadas alvo da bola e adiciona-as à lista ball_locat
                    target_x = pos_x + (math.cos(math.radians(ang)) * dis) - (200 * math.cos(math.radians(ang)))
                    target_y = pos_y + (math.sin(math.radians(ang)) * dis) - (200 * math.sin(math.radians(ang)))
                    
                    ball_locat.append([target_x, target_y])
                    print(ball_locat)  # Mostra a posição estimada da bola

        # Retorna a primeira posição registada da bola
        return ball_locat[0]


        


# Define a função "path_finder" que orienta o robô a mover-se de uma posição inicial (x0, y0) até uma posição alvo (pos_t),
# com uma orientação final opcional (final_angle)
def path_finder(x0, y0, theta0, pos_t, final_angle=None):
    print("Atual position ", x0, y0)  # Imprime a posição atual do robô
    print("going to ", pos_t)  # Imprime a posição alvo do robô
    
    robot.reset()  # Reinicia os sensores e os contadores de distância do robô
    xt, yt = pos_t  # Define as coordenadas alvo (x e y)
    
    # Calcula a diferença entre as coordenadas iniciais e as coordenadas alvo
    dx = xt - x0
    dy = yt - y0
    
    # Se o robô tiver a bola, define o ângulo final para 180 graus (orientado para trás)
    if have_ball:
        final_angle = 180
    
    # Calcula o ângulo alvo necessário para o robô apontar para o ponto alvo
    # (ângulo é calculado em sentido anti-horário a partir do eixo x)
    target_angle = math.degrees(math.atan2(dy, dx))
    
    # Calcula a rotação necessária do ângulo inicial até o ângulo alvo
    rotation_to_target = target_angle - theta0
    
    # Normaliza o ângulo de rotação para o intervalo [-180, 180], para evitar rotações desnecessárias
    rotation_to_target = (rotation_to_target + 180) % 360 - 180
    
    # Calcula a distância entre a posição atual e a posição alvo
    distance = math.sqrt(dx**2 + dy**2)
    
    # Gira o robô para a orientação do ponto alvo (anti-horário se positivo, horário se negativo)
    robot.turn(rotation_to_target)
    
    # Move-se para a frente até atingir a distância alvo
    while robot.distance() <= distance:
        robot.drive(65, 0)  # Define a velocidade do robô (65 unidades) e move-se em linha reta (0 graus)
        
        # Verifica se a bola está a menos de 250 unidades de distância
        if make_mesure(ball_sensor) < 250:
            global pos_x, pos_y  # Atualiza as coordenadas globais do robô

            # Calcula e atualiza as novas coordenadas da posição do robô
            pos_x = pos_x + (math.cos(math.radians(ang)) * robot.distance())
            pos_y = pos_y + (math.sin(math.radians(ang)) * robot.distance())
            
            robot.stop()  # Para o robô
            target_atcheaved()  # Função que indica que o alvo foi atingido
            break  # Sai do loop de movimento

    robot.stop()  # Para o robô caso tenha terminado o loop sem interrupções
    
    # Se um ângulo final foi especificado, o robô gira para atingir esse ângulo
    if final_angle is not None:
        # Calcula a rotação final necessária
        final_rotation = final_angle - target_angle
        
        # Normaliza a rotação final para o intervalo [-180, 180] para evitar rotações excessivas
        final_rotation = (final_rotation + 180) % 360 - 180
        robot.turn(final_rotation)  # Gira o robô para o ângulo final desejado



# Define a função "approach_ball" que faz com que o robô se aproxime de uma bola numa posição alvo especificada (target)
def approach_ball(target):
    x, y = target  # Extrai as coordenadas x e y do alvo

    # Constante para determinar a distância mínima aceitável antes de realizar uma verificação
    distance_threshold = 50  # Em milímetros, ajustável conforme necessário
    target_distance = (x**2 + y**2)**0.5  # Calcula a distância hipotética até o alvo
    measured_distance = make_mesure(ball_sensor)  # Mede a distância até à bola usando o sensor de bola
    
    # Loop de aproximação
    while target_distance > 350:  # Continua enquanto estiver a mais de 350 mm da bola
        # Atualiza a medição da distância até à bola
        measured_distance = make_mesure(ball_sensor)
        
        # Verifica se há uma grande discrepância entre a distância esperada e a distância medida
        if abs(target_distance - measured_distance) > distance_threshold:
            # Se a discrepância for significativa, realiza uma varredura angular para ajustar a orientação
            best_distance = measured_distance  # Inicia com a melhor distância como a medida atual
            best_angle = 0  # Ângulo correspondente à melhor distância
            
            # Varre ângulos entre -10° e +10° para encontrar o melhor alinhamento
            for angle in range(-10, 10, 5):
                robot.turn(angle)  # Vira o robô em "angle" graus
                current_distance = make_mesure(ball_sensor)  # Mede a nova distância à bola
                print(current_distance)
                
                # Atualiza a melhor leitura de distância e o ângulo correspondente, se encontrar uma mais próxima
                if current_distance < best_distance:
                    best_distance = current_distance
                    best_angle = angle
                
                # Retorna à orientação inicial para continuar a varredura
                robot.turn(-angle)
            
            # Atualiza a distância alvo com base na nova orientação
            target_distance = best_distance  # Atribui a melhor distância obtida como nova distância alvo
            robot.turn(best_angle)  # Vira o robô para o melhor ângulo encontrado
            robot.straight(50)  # Move-se para a frente 5 cm (ajustável)

            # Calcula o ângulo corrigido após o movimento e normaliza-o
            ang = get_corrected_angle()
            ang = norm_ang(ang)
            
            # Declara pos_x e pos_y como variáveis globais para atualização da posição
            global pos_x, pos_y
            # Atualiza as coordenadas da posição atual do robô com base no movimento
            pos_x = pos_x + math.cos(math.radians(ang)) * 50
            pos_y = pos_y + math.sin(math.radians(ang)) * 50
        
        # Atualiza a distância alvo para a condição do loop
        target_distance = measured_distance
        wait(100)  # Curto atraso para permitir uma nova verificação de distância

    



# Função "target_atcheaved" que executa ações para indicar que o alvo foi alcançado
def target_atcheaved():
    # Ativa o motor da garra (claw_motor) para agarrar ou segurar, girando a 60 unidades de potência até ficar travado
    claw_motor.run_until_stalled(60, Stop.HOLD, 40)
    
    # Move o robô para a frente 100 mm (10 cm)
    robot.straight(100)
    
    # Reverte o movimento do motor da garra para soltar ou libertar a carga, girando a -60 unidades de potência até ficar travado
    claw_motor.run_until_stalled(-60, Stop.HOLD, 40)


# Função "system_reset" para reiniciar as variáveis e o estado inicial do sistema
def system_reset():
    print("System reset.")  # Indica que o sistema está a ser reiniciado
    
    # Define as coordenadas iniciais do robô e o ângulo
    pos_x = 210
    pos_y = 170
    ang = 0  # Ângulo inicial definido para 0 (em sentido anti-horário, de 0 a 359 graus)
    
    # Define o estado do robô e outros parâmetros
    cycle = 0  # Define o ciclo para 0
    have_ball = False  # Indica que o robô não está a segurar a bola
    ball_locat = []  # Lista para armazenar a posição da bola
    at_origin = False  # Indica se o robô está na posição inicial

    path_finder(pos_x, pos_y, ang, [210,170], 0) # Desloca-se para a posição e orientação pretendidos
    ev3.speaker.beep()  # Emite um sinal sonoro para indicar a reinicialização
    
    # Aguarda até que o botão central seja pressionado para terminar a reinicialização
    while Button.CENTER not in ev3.buttons.pressed():
        wait(100)  
        
        # Define o estado atual como 0 para indicar a reinicialização
        global currentState
        currentState = 0


# Define o ciclo para 0 fora das funções, inicializando-o para começar o programa
cycle = 0

# Loop principal do programa, que espera até que o botão central seja pressionado para iniciar
while Button.CENTER not in ev3.buttons.pressed():
    wait(100)  


# Loop principal que corre continuamente, a executar a lógica do sistema
while True: 
    # Imprime o valor de "cycle" para verificar a iteração atual do ciclo
    print(cycle)   
    
    # Estado 0: Inicialização do sistema
    if currentState == 0:
        inicialise()  # Chama a função de inicialização
        currentState = 1  # Altera o estado para 1 (proximo passo após inicialização)
                
    # Estado 1: Definir o alvo
    elif currentState == 1:
        target = []  # Cria uma lista vazia para armazenar as coordenadas do alvo
        target = define_target(cycle, pos_x, pos_y)  # Chama a função "define_target" para calcular o alvo
        currentState = 2  # Altera o estado para 2 (aproximação ao alvo)
        print(target)  # Imprime as coordenadas do alvo calculado

    # Estado 2: Aproximação ao alvo
    elif currentState == 2:
        # Obtém o ângulo corrigido do robô e normaliza-o
        ang = get_corrected_angle()
        ang = norm_ang(ang)
        
        # Se a bola foi localizada mas o robô não está a segurar a bola, aproxima-se da bola
        if ball_locat and not have_ball:
            claw_motor.run_until_stalled(-60, Stop.HOLD, 40)  # Ativa a garra para tentar apanhar a bola
            approach_ball(target)  # Aproxima-se da bola usando a função "approach_ball"
        
        # Caso contrário, segue para o alvo
        else:
            path_finder(pos_x, pos_y, ang, target)  # Envia o robô para o alvo usando a função "path_finder"
        
        # Atualiza a posição do robô com as novas coordenadas do alvo
        pos_x = target[0]
        pos_y = target[1]
        
        # Verifica se o robô chegou à origem (coordenada x < posição inicial)
        if pos_x < 270:
            at_origin = True  # Define que o robô está na origem
        
        # Se a bola não foi localizada, volta ao estado 1 para procurar novamente
        if not ball_locat:
            currentState = 1
        
        # Se a bola foi localizada, avança para o estado 3 para pegar a bola
        elif ball_locat:
            currentState = 3

    # Estado 3: O robô está a pegar a bola
    elif currentState == 3:
        # Se o robô não estiver na origem, ativa a função de captura da bola
        if not at_origin:
            target_atcheaved()  # Indica que o alvo foi atingido (garra pega a bola)
            have_ball = True  # Define que o robô agora tem a bola
        # Se o robô estiver na origem, move-se para trás e vai para o estado 4 (finalizar a operação)
        elif at_origin:
            robot.straight(-100)  # Move para trás 100 mm
            currentState = 4  # Altera o estado para 4 para reiniciar o sistema
        else:
            currentState = 1  # Caso contrário, volta a procurar um alvo no estado 1
    
    # Estado 4: Reinicialização do sistema
    elif currentState == 4:
        system_reset()  # Chama a função de reinicialização do sistema

    # Imprime a posição atual e o ângulo corrigido
    print("Atual position ", pos_x, pos_y)
    print("ang= ", norm_ang(get_corrected_angle()))    
    
    # Incrementa o ciclo para o próximo ciclo de execução
    cycle += 1
