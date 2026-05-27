# =============================================================================
#  main.py  –  PISTA DE ARRANCONES  |  Red Malla NRF24L01
#  Hardware  : Raspberry Pi Pico  +  NRF24L01  (x3)
# =============================================================================
#  DETECCIÓN DE ROL POR HARDWARE
#  ─────────────────────────────
#  Conectar jumper/resistencia en GP20 y GP21 antes de energizar:
#
#    GP20   GP21  │  ROL
#    ─────────────┼─────────────
#    GND    GND   │  MAESTRO
#    3.3V   GND   │  SALIDA
#    GND    3.3V  │  META
# =============================================================================
#  PINOUT NRF24L01  (igual en los 3 Picos)
#  ────────────────────────────────────────
#  VCC  → 3.3V      GND  → GND
#  CE   → GP6       CSN  → GP5
#  SCK  → GP2       MOSI → GP3
#  MISO → GP4
# =============================================================================
#  PINOUT PERIFÉRICOS
#  ──────────────────
#  MAESTRO  : BTN_START      → GP1   (pull-up, activo en LOW)
#             BTN_RESET      → GP0   (pull-up, activo en LOW)
#             LED_OK_SALIDA  → GP25  (verde onboard: radio salida viva)
#             LED_OK_META    → GP24  (verde externo: radio meta viva)
#
#  SALIDA   : LED_ROJO       → GP15
#             LED_AMARILLO   → GP14
#             LED_VERDE      → GP13
#             LED_ERROR      → GP10  (salida en falso)
#             SENSOR_SALIDA  → GP1   (pull-up, activo en LOW)
#
#  META     : SENSOR_META    → GP1   (pull-up, activo en LOW)
#             LED_ERROR      → GP7   (cruce prematuro)
# =============================================================================

import utime
import machine
from machine import Pin, SPI, WDT
from nrf24l01 import NRF24L01

# ─────────────────────────────────────────────────────────────────────────────
#  PARÁMETROS GLOBALES
# ─────────────────────────────────────────────────────────────────────────────
RF_CHANNEL       = 100      # Canal RF libre de interferencia WiFi
PAYLOAD_SIZE     = 16       # Bytes por paquete
RF_REINTENTOS    = 5
RF_REINTENTO_MS  = 50       # FIX: era 15 ms, aumentado para dar margen al ACK
HEARTBEAT_MS     = 600
TIMEOUT_RADIO_MS = 2500
DEBOUNCE_MS      = 60
WDT_MS           = 8000

ADDR_MAESTRO  = b'MST01'
ADDR_SALIDA   = b'SLD01'
ADDR_META     = b'MET01'

MSG_PREPARAR_PISTA = const(0x01)
MSG_START_CRONO    = const(0x02)
MSG_ARM_META       = const(0x03)
MSG_FINISH         = const(0x04)
MSG_ERROR_FALSO    = const(0x05)
MSG_HEARTBEAT      = const(0x06)
MSG_RESET          = const(0x07)

ROL_MAESTRO = const(0)
ROL_SALIDA  = const(1)
ROL_META    = const(2)

T_ROJO_MS     = 2000
T_AMARILLO_MS = 1500

S_IDLE      = const(0)
S_LISTO     = const(1)
S_CORRIENDO = const(2)
S_FINISH    = const(3)
S_ERROR     = const(4)
S_ROJO      = const(5)
S_AMARILLO  = const(6)
S_VERDE     = const(7)
S_ARRANCADO = const(8)

# Estados de Meta
M_IDLE        = const(9)
M_PREPARADO   = const(10)
M_ARMADO      = const(11)
M_REGISTRADO  = const(12)
M_ERROR       = const(13)

# ─────────────────────────────────────────────────────────────────────────────
#  DETECCIÓN DE ROL POR HARDWARE
# ─────────────────────────────────────────────────────────────────────────────
def detectar_rol():
    p20 = Pin(20, Pin.IN, Pin.PULL_DOWN)
    p21 = Pin(21, Pin.IN, Pin.PULL_DOWN)
    utime.sleep_ms(20)
    v20 = p20.value()
    v21 = p21.value()

    if   v20 == 0 and v21 == 0: return ROL_MAESTRO
    elif v20 == 1 and v21 == 0: return ROL_SALIDA
    elif v20 == 0 and v21 == 1: return ROL_META
    else:
        led = Pin(25, Pin.OUT)
        print("[ERROR] Combinación no válida. Revise GP20/GP21.")
        while True:
            led.toggle()
            utime.sleep_ms(80)

# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS COMUNES
# ─────────────────────────────────────────────────────────────────────────────
def build_payload(msg_id, valor_ms=0):
    buf    = bytearray(PAYLOAD_SIZE)
    buf[0] = msg_id
    buf[1] = (valor_ms      ) & 0xFF
    buf[2] = (valor_ms >>  8) & 0xFF
    buf[3] = (valor_ms >> 16) & 0xFF
    buf[4] = (valor_ms >> 24) & 0xFF
    return bytes(buf)

def parse_payload(data):
    if len(data) < 5:
        return 0, 0
    msg_id   = data[0]
    valor_ms = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24)
    return msg_id, valor_ms

def ms_a_str(ms):
    cs         = ms // 10
    centesimas = cs % 100
    segundos   = (cs // 100) % 60
    minutos    = (cs // 100) // 60
    return "{:02d}:{:02d}.{:02d}".format(minutos, segundos, centesimas)

def init_radio():
    """
    Inicializa el radio NRF24L01 con reintentos y deshabilita
    la retransmisión automática por hardware para evitar conflicto
    con los reintentos por software.
    """
    Chip_Enable = 6
    Chip_Select = 5
    CE  = Pin(Chip_Enable, Pin.OUT, value=0)
    CSN = Pin(Chip_Select, Pin.OUT, value=1)

    try:
        Bus_spi = SPI(0, baudrate=1000000, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
        radio = NRF24L01(Bus_spi, CSN, CE, channel=RF_CHANNEL, payload_size=PAYLOAD_SIZE)
        print("[RADIO] Inicializada OK (Intento 1)")
        return radio
    except OSError:
        print("[RADIO] Fallo inicial. Reiniciando bus SPI...")
        try:
            Bus_spi.deinit()
        except:
            pass
        utime.sleep_ms(500)

        try:
            Bus_spi = SPI(0, baudrate=1000000, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
            radio = NRF24L01(Bus_spi, CSN, CE, channel=RF_CHANNEL, payload_size=PAYLOAD_SIZE)
            # FIX: ídem en recuperación
            print("[RADIO] Inicializada OK (Recuperación)")
            return radio
        except Exception as e:
            print("[ERROR FATAL] No se puede inicializar el radio NRF24L01:", e)
            led = Pin(25, Pin.OUT)
            while True:
                led.toggle()
                utime.sleep_ms(200)

def enviar_robusto(radio, addr, msg_id, valor=0, intentos=RF_REINTENTOS, pausa_ms=RF_REINTENTO_MS):
    """
    FIX PRINCIPAL:
    - Llama stop_listening() antes de cada intento para salir del modo RX.
      Sin esto el NRF24L01 ignora send() porque sigue escuchando.
    - El bloque finally garantiza que el radio siempre vuelva a modo RX,
      incluso si send() lanza una excepción.
    - Se elimina la necesidad de llamar open_rx_pipe + start_listening
      después de cada enviar_robusto() en el loop principal.
    """
    payload = build_payload(msg_id, valor)

    for i in range(intentos):
        try:
            radio.stop_listening()          # FIX: salir de RX antes de TX
            radio.open_tx_pipe(addr)
            utime.sleep_ms(5)              # settling time del módulo

            ok = radio.send(payload)        # bloqueante ~4 ms
            if ok:
                return True

        except Exception as e:
            print("[RF] Error send intento {}: {}".format(i + 1, e))
        finally:
            radio.start_listening()         # FIX: siempre volver a RX

        utime.sleep_ms(pausa_ms)

    print("[RF] FALLO envío msg={} a {} tras {} intentos".format(msg_id, addr, intentos))
    return False

# ─────────────────────────────────────────────────────────────────────────────
#  NODO MAESTRO
# ─────────────────────────────────────────────────────────────────────────────
def run_maestro():
    print("=" * 44)
    print("  PISTA DE ARRANCONES  –  NODO MAESTRO")
    print("=" * 44)

    wdt = WDT(timeout=WDT_MS)

    btn_start     = Pin(1,  Pin.IN,  Pin.PULL_DOWN)
    btn_reset     = Pin(0,  Pin.IN,  Pin.PULL_DOWN)
    led_ok_salida = Pin(25, Pin.OUT, value=0)
    led_ok_meta   = Pin(24, Pin.OUT, value=0)

    radio = init_radio()
    radio.open_rx_pipe(1, ADDR_MAESTRO)
    radio.start_listening()

    estado           = S_IDLE
    t_inicio_ms      = 0
    t_fin_ms         = 0
    ultimo_hb_salida = utime.ticks_ms()
    ultimo_hb_meta   = utime.ticks_ms()
    ultimo_start     = 0
    ultimo_reset     = 0
    t_ultimo_print   = 0

    print("[MAESTRO] Sistema listo. Esperando BTN_START...")
    print("[MAESTRO] LEDs piloto: GP25=SALIDA  GP24=META")

    while True:
        try:
            ahora = utime.ticks_ms()
            wdt.feed()

            s_vivo = utime.ticks_diff(ahora, ultimo_hb_salida) < TIMEOUT_RADIO_MS
            m_vivo = utime.ticks_diff(ahora, ultimo_hb_meta)   < TIMEOUT_RADIO_MS
            led_ok_salida.value(1 if s_vivo else 0)
            led_ok_meta.value(  1 if m_vivo else 0)

            if btn_start.value() == 1 and estado == S_IDLE:
                if utime.ticks_diff(ahora, ultimo_start) > DEBOUNCE_MS:
                    ultimo_start = ahora
                    utime.sleep_ms(DEBOUNCE_MS)

                    if not s_vivo or not m_vivo:
                        print("[MAESTRO] !! ALERTA: Radio(s) sin señal.")
                        print("          SALIDA={} | META={}".format(
                              "OK" if s_vivo else "SIN SEÑAL",
                              "OK" if m_vivo else "SIN SEÑAL"))
                    else:
                        print("[MAESTRO] BTN_START → enviando PREPARAR_PISTA...")
                        ok_s = enviar_robusto(radio, ADDR_SALIDA, MSG_PREPARAR_PISTA)
                        ok_m = enviar_robusto(radio, ADDR_META,   MSG_PREPARAR_PISTA)
                        # FIX: ya no es necesario llamar open_rx_pipe + start_listening
                        # aquí porque enviar_robusto() lo hace en su bloque finally

                        if ok_s and ok_m:
                            estado = S_LISTO
                            print("[MAESTRO] PREPARAR_PISTA confirmado. Esperando salida...")
                        else:
                            print("[MAESTRO] !! Error al enviar PREPARAR_PISTA.")

            if btn_reset.value() == 1:
                if utime.ticks_diff(ahora, ultimo_reset) > DEBOUNCE_MS:
                    ultimo_reset = ahora
                    utime.sleep_ms(DEBOUNCE_MS)
                    print("\n[MAESTRO] ──── RESET ────")
                    enviar_robusto(radio, ADDR_SALIDA, MSG_RESET)
                    enviar_robusto(radio, ADDR_META,   MSG_RESET)
                    # FIX: ídem, ya no necesita restaurar RX manualmente
                    estado      = S_IDLE
                    t_inicio_ms = 0
                    t_fin_ms    = 0
                    print("[MAESTRO] Sistema reiniciado. Listo para nueva carrera.\n")

            if radio.any():
                try:
                    data          = radio.recv()
                    msg_id, valor = parse_payload(data)
                    ahora         = utime.ticks_ms()

                    if msg_id == MSG_HEARTBEAT:
                        if valor == ROL_SALIDA:
                            ultimo_hb_salida = ahora
                        elif valor == ROL_META:
                            ultimo_hb_meta = ahora
                    elif msg_id == MSG_START_CRONO and estado == S_LISTO:
                        t_inicio_ms = ahora
                        estado      = S_CORRIENDO
                        print("[MAESTRO] >>> CARRERA INICIADA <<<")
                    elif msg_id == MSG_FINISH and estado == S_CORRIENDO:
                        t_fin_ms   = ahora
                        t_total_ms = utime.ticks_diff(t_fin_ms, t_inicio_ms)
                        estado     = S_FINISH
                        print("\n[MAESTRO] ╔══════════════════════════╗")
                        print("[MAESTRO] ║  TIEMPO FINAL: {}  ║".format(ms_a_str(t_total_ms)))
                        print("[MAESTRO] ╚══════════════════════════╝")
                    elif msg_id == MSG_ERROR_FALSO:
                        estado = S_ERROR
                        print("\n[MAESTRO] !!  ERROR: SALIDA EN FALSO  !!")

                except Exception as e:
                    print("[MAESTRO] Error procesando paquete RF:", e)

            if estado == S_CORRIENDO:
                if utime.ticks_diff(ahora, t_ultimo_print) >= 100:
                    t_ultimo_print = ahora
                    elapsed = utime.ticks_diff(ahora, t_inicio_ms)
                    print("\r[CRONO] {} ".format(ms_a_str(elapsed)), end="")

        except Exception as e:
            print("[MAESTRO] Excepción en loop:", e)
            utime.sleep_ms(100)
        utime.sleep_ms(20)

# ─────────────────────────────────────────────────────────────────────────────
#  NODO ESCLAVO SALIDA
# ─────────────────────────────────────────────────────────────────────────────
def run_salida():
    print("=" * 44)
    print("  PISTA DE ARRANCONES  –  NODO SALIDA")
    print("=" * 44)

    wdt = WDT(timeout=WDT_MS)

    led_rojo     = Pin(15, Pin.OUT, value=0)
    led_amarillo = Pin(14, Pin.OUT, value=0)
    led_verde    = Pin(13, Pin.OUT, value=0)
    led_error    = Pin(10, Pin.OUT, value=0)
    sensor       = Pin(1,  Pin.IN, Pin.PULL_DOWN)

    radio = init_radio()
    radio.open_rx_pipe(1, ADDR_SALIDA)
    radio.start_listening()

    estado           = S_IDLE
    t_sema           = 0
    sensor_disparado = False
    t_ultimo_hb      = utime.ticks_ms()
    t_ultimo_sensor  = 0

    def isr_sensor(pin):
        nonlocal sensor_disparado, t_ultimo_sensor
        ahora = utime.ticks_ms()
        if utime.ticks_diff(ahora, t_ultimo_sensor) > DEBOUNCE_MS:
            sensor_disparado = True
            t_ultimo_sensor  = ahora

    def apagar_semaforo():
        led_rojo.value(0)
        led_amarillo.value(0)
        led_verde.value(0)
        led_error.value(0)

    def semaforo_error():
        led_rojo.value(1)
        led_amarillo.value(1)
        led_verde.value(1)
        led_error.value(1)

    def deshabilitar_sensor():
        try: sensor.irq(handler=None)
        except Exception: pass

    def habilitar_sensor():
        try: sensor.irq(trigger=Pin.IRQ_RISING, handler=isr_sensor)
        except Exception as e: print("[SALIDA] Error habilitando IRQ sensor:", e)

    print("[SALIDA] Listo. Esperando PREPARAR_PISTA...")

    while True:
        try:
            ahora = utime.ticks_ms()
            wdt.feed()

            if utime.ticks_diff(ahora, t_ultimo_hb) >= HEARTBEAT_MS:
                enviar_robusto(radio, ADDR_MAESTRO, MSG_HEARTBEAT, ROL_SALIDA, intentos=2, pausa_ms=10)
                # FIX: ya no necesita restaurar RX manualmente aquí
                t_ultimo_hb = ahora

            if radio.any():
                try:
                    data          = radio.recv()
                    msg_id, valor = parse_payload(data)

                    if msg_id == MSG_PREPARAR_PISTA and estado == S_IDLE:
                        print("[SALIDA] PREPARAR_PISTA → semáforo ROJO")
                        deshabilitar_sensor()
                        apagar_semaforo()
                        sensor_disparado = False
                        led_rojo.value(1)
                        estado = S_ROJO
                        t_sema = utime.ticks_ms()

                    elif msg_id == MSG_RESET:
                        print("[SALIDA] RESET recibido → IDLE")
                        deshabilitar_sensor()
                        apagar_semaforo()
                        sensor_disparado = False
                        estado = S_IDLE
                except Exception as e:
                    print("[SALIDA] Error procesando paquete RF:", e)

            if estado == S_ROJO:
                if utime.ticks_diff(ahora, t_sema) >= T_ROJO_MS:
                    led_rojo.value(0)
                    led_amarillo.value(1)
                    estado = S_AMARILLO
                    t_sema = ahora
                    print("[SALIDA] Semáforo → AMARILLO")

            elif estado == S_AMARILLO:
                if utime.ticks_diff(ahora, t_sema) >= T_AMARILLO_MS:
                    led_amarillo.value(0)
                    led_verde.value(1)
                    sensor_disparado = False
                    habilitar_sensor()
                    estado = S_VERDE
                    print("[SALIDA] Semáforo → VERDE  |  Sensor ACTIVO")

            if sensor_disparado:
                sensor_disparado = False
                deshabilitar_sensor()

                if estado == S_VERDE:
                    print("[SALIDA] Cruce VÁLIDO → START_CRONO + ARM_META")
                    enviar_robusto(radio, ADDR_MAESTRO, MSG_START_CRONO)
                    enviar_robusto(radio, ADDR_META,    MSG_ARM_META)
                    # FIX: ya no necesita restaurar RX manualmente aquí
                    estado = S_ARRANCADO

                elif estado in (S_ROJO, S_AMARILLO):
                    print("[SALIDA] !! SALIDA EN FALSO")
                    semaforo_error()
                    enviar_robusto(radio, ADDR_MAESTRO, MSG_ERROR_FALSO)
                    # FIX: ya no necesita restaurar RX manualmente aquí
                    estado = S_ERROR

        except Exception as e:
            print("[SALIDA] Excepción en loop:", e)
            utime.sleep_ms(100)
        utime.sleep_ms(10)

# ─────────────────────────────────────────────────────────────────────────────
#  NODO ESCLAVO META
# ─────────────────────────────────────────────────────────────────────────────
def run_meta():
    print("=" * 44)
    print("  PISTA DE ARRANCONES  –  NODO META")
    print("=" * 44)

    wdt = WDT(timeout=WDT_MS)

    sensor_meta = Pin(1,  Pin.IN,  Pin.PULL_DOWN)
    led_error   = Pin(7,  Pin.OUT, value=0)

    radio = init_radio()
    radio.open_rx_pipe(1, ADDR_META)
    radio.start_listening()

    estado           = M_IDLE
    sensor_disparado = False
    t_arm            = 0
    t_ultimo_hb      = utime.ticks_ms()
    t_ultimo_sensor  = 0

    def isr_meta(pin):
        nonlocal sensor_disparado, t_ultimo_sensor
        ahora = utime.ticks_ms()
        if utime.ticks_diff(ahora, t_ultimo_sensor) > DEBOUNCE_MS:
            sensor_disparado = True
            t_ultimo_sensor  = ahora

    def deshabilitar_sensor():
        try: sensor_meta.irq(handler=None)
        except Exception: pass

    def habilitar_sensor():
        try: sensor_meta.irq(trigger=Pin.IRQ_RISING, handler=isr_meta)
        except Exception as e: print("[META] Error habilitando IRQ sensor:", e)

    print("[META] Listo. Esperando PREPARAR_PISTA...")

    while True:
        try:
            ahora = utime.ticks_ms()
            wdt.feed()

            if utime.ticks_diff(ahora, t_ultimo_hb) >= HEARTBEAT_MS:
                enviar_robusto(radio, ADDR_MAESTRO, MSG_HEARTBEAT, ROL_META, intentos=2, pausa_ms=10)
                # FIX: ya no necesita restaurar RX manualmente aquí
                t_ultimo_hb = ahora

            if radio.any():
                try:
                    data          = radio.recv()
                    msg_id, valor = parse_payload(data)

                    if msg_id == MSG_PREPARAR_PISTA:
                        print("[META] PREPARAR_PISTA → preparado, sensor DESARMADO")
                        deshabilitar_sensor()
                        sensor_disparado = False
                        led_error.value(0)
                        estado = M_PREPARADO

                    elif msg_id == MSG_ARM_META and estado == M_PREPARADO:
                        print("[META] ARM_META → sensor ARMADO, esperando cruce")
                        sensor_disparado = False
                        t_arm = utime.ticks_ms()
                        habilitar_sensor()
                        estado = M_ARMADO

                    elif msg_id == MSG_RESET:
                        print("[META] RESET → IDLE")
                        deshabilitar_sensor()
                        sensor_disparado = False
                        led_error.value(0)
                        estado = M_IDLE
                except Exception as e:
                    print("[META] Error procesando paquete RF:", e)

            if sensor_disparado:
                sensor_disparado = False
                deshabilitar_sensor()
                t_cruce = utime.ticks_ms()

                if estado == M_ARMADO:
                    t_carrera = utime.ticks_diff(t_cruce, t_arm)
                    print("[META] Cruce VÁLIDO → FINISH  tiempo: {}".format(ms_a_str(t_carrera)))
                    enviar_robusto(radio, ADDR_MAESTRO, MSG_FINISH, t_carrera)
                    # FIX: ya no necesita restaurar RX manualmente aquí
                    estado = M_REGISTRADO

                elif estado == M_PREPARADO:
                    print("[META] !! ERROR: cruce prematuro antes de ARM_META")
                    led_error.value(1)
                    enviar_robusto(radio, ADDR_MAESTRO, MSG_ERROR_FALSO)
                    # FIX: ya no necesita restaurar RX manualmente aquí
                    estado = M_ERROR

        except Exception as e:
            print("[META] Excepción en loop:", e)
            utime.sleep_ms(100)
        utime.sleep_ms(10)

# ─────────────────────────────────────────────────────────────────────────────
#  PUNTO DE ENTRADA
# ─────────────────────────────────────────────────────────────────────────────
def main():
    utime.sleep_ms(300)
    rol = detectar_rol()

    if rol == ROL_MAESTRO:
        run_maestro()
    elif rol == ROL_SALIDA:
        run_salida()
    elif rol == ROL_META:
        run_meta()

main()