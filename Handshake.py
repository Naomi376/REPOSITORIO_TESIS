import utime
from machine import Pin, SPI
import nrf24l01

#----------------------------------------
#         Definición de Constantes
#----------------------------------------

# Configuración de la radio
RF_CHANNEL = 100
PAYLOADSIZE = 16

# Direcciones Físicas (Tuberías)
ADDR_MAESTRO = b"MST01"
ADDR_SALIDA  = b"SLD01"
ADDR_META    = b"MET01"

# Identificadores de Mensajes (Opcodes)
HOLA_AMIGOS      = const(0x80)
HOLA_MET_SOY_SLD = const(0x81)
HOLA_SOY_MET     = const(0x82)

# Roles de las Placas
ROL_MAESTRO        = const(0)
ROL_ESCLAVO_SALIDA = const(1)
ROL_ESCLAVO_META   = const(2)

# Asignación Numérica de Estados (Todos juntos y ordenados aquí arriba)
M_INICIO                 = const(3)
M_ESPERANDO_META         = const(4)

ES_INICIO                = const(5)
ES_ESPERANDO_BOTON       = const(6)
ES_ESPERANDO_META        = const(7)

EM_INICIO                = const(8)
EM_ESPERANDO_SALIDA      = const(9)

#----------------------------------------
#         FUNCIONES GENERALES
#----------------------------------------

def roles():
    # Configuración de Jumpers
    ROL1 = Pin(16, Pin.IN, Pin.PULL_DOWN)
    ROL2 = Pin(17, Pin.IN, Pin.PULL_DOWN)
    utime.sleep_ms(20) # Pequeño tiempo para estabilizar lectura eléctrica

    if ROL1.value() == 0 and ROL2.value() == 0:
        return ROL_MAESTRO
    elif ROL1.value() == 0 and ROL2.value() == 1:
        return ROL_ESCLAVO_SALIDA
    elif ROL1.value() == 1 and ROL2.value() == 0:
        return ROL_ESCLAVO_META
    return -1
    
def init_radio():
    CE = Pin(6, Pin.OUT, value=0)
    CSN = Pin(5, Pin.OUT, value=1)
    Bus_SPI = SPI(0, baudrate=1000000, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
    return nrf24l01.NRF24L01(Bus_SPI, CE, CSN, channel=RF_CHANNEL, payload_size=PAYLOADSIZE)

def enviar_mensaje(radio, destino, msg_id):
    buf = bytearray(PAYLOADSIZE)
    buf[0] = msg_id
    radio.open_tx_pipe(destino)
    for _ in range(5):
        if radio.send(buf): 
            return True
        utime.sleep_ms(10)
    return False

#----------------------------------------
#         FUNCIONES PRINCIPALES
#----------------------------------------  

def maestro():
    print("[SISTEMA] Arrancando modo: MAESTRO")
    Boton_Inicio = Pin(1, Pin.IN, Pin.PULL_DOWN)
    
    radio = init_radio()
    radio.open_rx_pipe(1, ADDR_MAESTRO)
    radio.start_listening()
    
    Estado = M_INICIO
    
    while True:
        if Estado == M_INICIO:
            if Boton_Inicio.value() == 1:
                print("[MAESTRO] Botón presionado. Saludando al circuito...")
                # Corregido: Se usan las direcciones ADDR, no los ROLES numéricos
                enviar_mensaje(radio, ADDR_SALIDA, HOLA_AMIGOS)
                enviar_mensaje(radio, ADDR_META, HOLA_AMIGOS)
                
                # Volver a escuchar
                radio.open_rx_pipe(1, ADDR_MAESTRO)
                radio.start_listening()
                
                Estado = M_ESPERANDO_META
            
        elif Estado == M_ESPERANDO_META:
            if radio.any():
                mensaje = radio.recv()
                if mensaje[0] == HOLA_SOY_MET:
                    print("[MAESTRO] Gracias por el mensaje, ESCLAVO_META. ¡Handshake completo!")
                    Estado = M_INICIO
        utime.sleep_ms(20)

def esclavo_salida():
    print("[SISTEMA] Arrancando modo: ESCLAVO SALIDA")
    Boton_mensaje = Pin(1, Pin.IN, Pin.PULL_DOWN)
    
    radio = init_radio()
    radio.open_rx_pipe(1, ADDR_SALIDA)
    radio.start_listening()
    
    Estado = ES_INICIO
    
    while True:
        # Bloque de recepción de Radio
        if radio.any():
            mensaje = radio.recv()            
            if Estado == ES_INICIO:
                if mensaje[0] == HOLA_AMIGOS:
                    print("[SALIDA] ¡Maestro saludó! Esperando disparo de salida...")
                    Estado = ES_ESPERANDO_BOTON
                 
            elif Estado == ES_ESPERANDO_META:
                if mensaje[0] == HOLA_SOY_MET:
                    print("[SALIDA] Meta confirmó de recibido. Reiniciando.")
                    Estado = ES_INICIO 
                    
        # Bloque de lectura de botón (Independiente del radio)
        if Estado == ES_ESPERANDO_BOTON:
            if Boton_mensaje.value() == 1: # Corregido de Boton_Mensaje a Boton_mensaje
                print("[SALIDA] ¡Botón presionado! Avisando a Meta...")
                enviar_mensaje(radio, ADDR_META, HOLA_MET_SOY_SLD)
                
                # Volver a escuchar
                radio.open_rx_pipe(1, ADDR_SALIDA)
                radio.start_listening()
                
                Estado = ES_ESPERANDO_META
        utime.sleep_ms(20)

def esclavo_meta():
    print("[SISTEMA] Arrancando modo: ESCLAVO META")
    radio = init_radio()
    radio.open_rx_pipe(1, ADDR_META)
    radio.start_listening()
    
    Estado = EM_INICIO
    
    while True:
        if radio.any():
            mensaje = radio.recv()            
        
            if Estado == EM_INICIO: # Corregido: cambiados los números quemados por las constantes
                if mensaje[0] == HOLA_AMIGOS:
                    print("[META] ¡Maestro saludó! Esperando señal de Salida...")
                    Estado = EM_ESPERANDO_SALIDA
        
            elif Estado == EM_ESPERANDO_SALIDA:
                if mensaje[0] == HOLA_MET_SOY_SLD:
                    print("[META] ¡Salida activada! Respondiendo a todos...")
                    enviar_mensaje(radio, ADDR_SALIDA, HOLA_SOY_MET)
                    enviar_mensaje(radio, ADDR_MAESTRO, HOLA_SOY_MET)
                    
                    # Volver a escuchar
                    radio.open_rx_pipe(1, ADDR_META)
                    radio.start_listening()
                    
                    Estado = EM_INICIO
        utime.sleep_ms(20)

#----------------------------------------
#              MAIN DRIVER
#----------------------------------------
def main():
    rol = roles()
    if rol == ROL_MAESTRO: 
        maestro()
    elif rol == ROL_ESCLAVO_SALIDA: 
        esclavo_salida()
    elif rol == ROL_ESCLAVO_META: 
        esclavo_meta()
    else: 
        print("[ERROR] Revisa los Jumpers en GP16/GP17. Combinación inválida.")

main()