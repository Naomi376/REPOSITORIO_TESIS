# =============================================================================
#  main.py  –  PISTA DE ARRANCONES  |  Red Malla NRF24L01
#  Hardware  : Raspberry Pi Pico  +  NRF24L01  (x3)
#              + RP2040 Zero con ST7789V2 en cada nodo esclavo
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
#  MAESTRO  : BTN_START      → GP1   (pull-down, activo en HIGH)
#             BTN_RESET      → GP0   (pull-down, activo en HIGH)
#             LED_OK_SALIDA  → GP25  (verde onboard: radio salida viva)
#             LED_OK_META    → GP24  (verde externo: radio meta viva)
#
#  SALIDA   : LED_ROJO       → GP15
#             LED_AMARILLO   → GP14
#             LED_VERDE      → GP13
#             LED_ERROR      → GP10  (salida en falso)
#             SENSOR_SALIDA  → GP1   (pull-down, activo en HIGH)
#             UART TX (display) → GP8 (UART1 TX)
#
#  META     : SENSOR_META    → GP1   (pull-down, activo en HIGH)
#             LED_ERROR      → GP7   (cruce prematuro)
#             UART TX (display) → GP8 (UART1 TX)
# =============================================================================
#  PROTOCOLO UART → RP2040 Zero
#  ─────────────────────────────
#  Mensajes simples terminados en \n que el Zero interpreta:
#    "PREPARAR\n"   → "¡ATENTOS! Semáforo inicia..."  (SALIDA)
#                   → "Salida preparada"               (META)
#    "ROJO\n"       → Semáforo ROJO encendido
#    "AMARILLO\n"   → Semáforo AMARILLO encendido
#    "VERDE\n"      → "¡VAMOOOOOS!"
#    "FALSO\n"      → Salida en falso / Error
#    "CRUCE\n"      → "Auto cruzó la meta"             (META)
#    "RESET\n"      → "Sistema reseteado\nPista lista"
#    "RADIO:1\n"    → Estado radio = ACTIVO
#    "RADIO:0\n"    → Estado radio = SIN SEÑAL
# =============================================================================

import utime
from machine import Pin, SPI, WDT, UART
from nrf24l01 import NRF24L01

# ─────────────────────────────────────────────────────────────────────────────
#  PARÁMETROS GLOBALES
# ─────────────────────────────────────────────────────────────────────────────
RF_CHANNEL       = 100
PAYLOAD_SIZE     = 16
HEARTBEAT_MS     = 800
TIMEOUT_RADIO_MS = 3000
DEBOUNCE_MS      = 60
WDT_MS           = 8000

REPEAT_CRITICO   = 3
REPEAT_HB        = 1
PAUSA_REPEAT_MS  = 8

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
#  HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def build_payload(msg_id, valor_us=0):
    """
    Payload de 16 bytes.
    Los bytes 1-4 ahora llevan microsegundos (uint32).
    """
    buf    = bytearray(PAYLOAD_SIZE)
    buf[0] = msg_id
    buf[1] = (valor_us      ) & 0xFF
    buf[2] = (valor_us >>  8) & 0xFF
    buf[3] = (valor_us >> 16) & 0xFF
    buf[4] = (valor_us >> 24) & 0xFF
    return bytes(buf)

def parse_payload(data):
    if len(data) < 5:
        return 0, 0
    msg_id   = data[0]
    valor_us = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24)
    return msg_id, valor_us

def us_a_str(us):
    """Convierte microsegundos a MM:SS.mmm (milisegundos en display)."""
    ms         = us // 1000
    milisimas  = ms % 1000
    segundos   = (ms // 1000) % 60
    minutos    = (ms // 1000) // 60
    return "{:02d}:{:02d}.{:03d}".format(minutos, segundos, milisimas)

def ms_a_str(ms):
    """Compatibilidad: convierte ms a MM:SS.mmm."""
    return us_a_str(ms * 1000)

# ─────────────────────────────────────────────────────────────────────────────
#  UART para pantalla (UART1: TX=GP8, RX=GP9 — solo usamos TX)
# ─────────────────────────────────────────────────────────────────────────────
def init_uart_display():
    """Inicializa UART1 para comunicación con RP2040 Zero."""
    try:
        uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
        utime.sleep_ms(50)
        return uart
    except Exception as e:
        print("[UART] Error init:", e)
        return None

def display_send(uart, msg):
    """Envía un mensaje al display por UART."""
    if uart is None:
        return
    try:
        uart.write(msg + "\n")
    except Exception as e:
        print("[UART] Error enviando '{}': {}".format(msg, e))

# ─────────────────────────────────────────────────────────────────────────────
#  RADIO  –  sin ACK (send_start) para evitar colisiones con heartbeats
# ─────────────────────────────────────────────────────────────────────────────
def init_radio(addr_rx):
    CE  = Pin(6, Pin.OUT, value=0)
    CSN = Pin(5, Pin.OUT, value=1)

    for intento in range(3):
        try:
            spi   = SPI(0, baudrate=1000000, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
            radio = NRF24L01(spi, CSN, CE, channel=RF_CHANNEL, payload_size=PAYLOAD_SIZE)
            radio.open_rx_pipe(1, addr_rx)
            radio.start_listening()
            print("[RADIO] OK en intento {}  escuchando en {}".format(intento + 1, addr_rx))
            return radio
        except Exception as e:
            print("[RADIO] Fallo intento {}: {}".format(intento + 1, e))
            try: spi.deinit()
            except: pass
            utime.sleep_ms(400)

    print("[ERROR FATAL] No se puede inicializar NRF24L01")
    led = Pin(25, Pin.OUT)
    while True:
        led.toggle()
        utime.sleep_ms(200)


def enviar(radio, addr_rx_propia, addr_dest, msg_id, valor=0, repeticiones=REPEAT_CRITICO):
    payload  = build_payload(msg_id, valor)
    enviados = 0
    try:
        radio.stop_listening()
        radio.open_tx_pipe(addr_dest)
        utime.sleep_ms(3)

        for _ in range(repeticiones):
            try:
                radio.send_start(payload)
                utime.sleep_ms(PAUSA_REPEAT_MS)
                radio.send_done()
                enviados += 1
            except Exception as e:
                print("[RF] send error msg={}: {}".format(msg_id, e))
                utime.sleep_ms(PAUSA_REPEAT_MS)

    except Exception as e:
        print("[RF] Error TX setup msg={}: {}".format(msg_id, e))
    finally:
        try:
            radio.open_rx_pipe(1, addr_rx_propia)
            radio.start_listening()
        except Exception as e:
            print("[RF] Error restaurando RX: {}".format(e))

    return enviados > 0

# ─────────────────────────────────────────────────────────────────────────────
#  NODO MAESTRO  (sin pantalla, sin UART — igual que antes)
# ─────────────────────────────────────────────────────────────────────────────
def run_maestro():
    print("=" * 44)
    print("  PISTA DE ARRANCONES  –  NODO MAESTRO")
    print("=" * 44)

    wdt = WDT(timeout=WDT_MS)

    btn_start     = Pin(1,  Pin.IN, Pin.PULL_DOWN)
    btn_reset     = Pin(0,  Pin.IN, Pin.PULL_DOWN)
    led_ok_salida = Pin(25, Pin.OUT, value=0)
    led_ok_meta   = Pin(24, Pin.OUT, value=0)

    radio = init_radio(ADDR_MAESTRO)

    def tx(addr, msg, val=0, rep=REPEAT_CRITICO):
        return enviar(radio, ADDR_MAESTRO, addr, msg, val, rep)

    estado           = S_IDLE
    t_inicio_us      = 0
    t_fin_us         = 0
    ultimo_hb_salida = utime.ticks_ms() - TIMEOUT_RADIO_MS
    ultimo_hb_meta   = utime.ticks_ms() - TIMEOUT_RADIO_MS
    ultimo_start     = 0
    ultimo_reset     = 0
    t_ultimo_print   = 0

    print("[MAESTRO] Listo. BTN_START=GP1  BTN_RESET=GP0")
    print("[MAESTRO] LED_SALIDA=GP25  LED_META=GP24")

    while True:
        try:
            ahora = utime.ticks_ms()
            wdt.feed()

            # ── Indicadores de enlace ──────────────────────────────────────
            s_vivo = utime.ticks_diff(ahora, ultimo_hb_salida) < TIMEOUT_RADIO_MS
            m_vivo = utime.ticks_diff(ahora, ultimo_hb_meta)   < TIMEOUT_RADIO_MS
            led_ok_salida.value(1 if s_vivo else 0)
            led_ok_meta.value(  1 if m_vivo else 0)

            # ── BTN_START ──────────────────────────────────────────────────
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
                        print("[MAESTRO] BTN_START → PREPARAR_PISTA")
                        tx(ADDR_SALIDA, MSG_PREPARAR_PISTA)
                        tx(ADDR_META,   MSG_PREPARAR_PISTA)
                        estado = S_LISTO
                        print("[MAESTRO] Estado → LISTO. Esperando cruce de salida...")

            # ── BTN_RESET ──────────────────────────────────────────────────
            if btn_reset.value() == 1:
                if utime.ticks_diff(ahora, ultimo_reset) > DEBOUNCE_MS:
                    ultimo_reset = ahora
                    utime.sleep_ms(DEBOUNCE_MS)
                    print("\n[MAESTRO] ──── RESET ────")
                    tx(ADDR_SALIDA, MSG_RESET)
                    tx(ADDR_META,   MSG_RESET)
                    estado      = S_IDLE
                    t_inicio_us = 0
                    t_fin_us    = 0
                    print("[MAESTRO] Reiniciado. Listo para nueva carrera.\n")

            # ── Recepción ──────────────────────────────────────────────────
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
                        # El valor recibido NO se usa aquí; el Maestro mide su propio tiempo
                        # (el tiempo de carrera se calcula en el nodo META con mayor precisión)
                        t_inicio_us = utime.ticks_us()
                        estado      = S_CORRIENDO
                        print("[MAESTRO] >>> CARRERA INICIADA <<<")

                    elif msg_id == MSG_FINISH and estado == S_CORRIENDO:
                        # valor contiene los microsegundos medidos por el nodo META
                        t_total_us = valor
                        estado     = S_FINISH
                        print("\n[MAESTRO] ╔══════════════════════════════╗")
                        print("[MAESTRO] ║  TIEMPO FINAL: {}  ║".format(us_a_str(t_total_us)))
                        print("[MAESTRO] ╚══════════════════════════════╝")

                    elif msg_id == MSG_ERROR_FALSO:
                        estado = S_ERROR
                        print("\n[MAESTRO] !!  ERROR: SALIDA EN FALSO  !!")

                except Exception as e:
                    print("[MAESTRO] Error RX:", e)

            # ── Cronómetro en consola ──────────────────────────────────────
            if estado == S_CORRIENDO:
                if utime.ticks_diff(ahora, t_ultimo_print) >= 100:
                    t_ultimo_print = ahora
                    elapsed_us = utime.ticks_diff(utime.ticks_us(), t_inicio_us)
                    print("\r[CRONO] {} ".format(us_a_str(elapsed_us)), end="")

        except Exception as e:
            print("[MAESTRO] Excepción loop:", e)
            utime.sleep_ms(100)
        utime.sleep_ms(10)

# ─────────────────────────────────────────────────────────────────────────────
#  NODO SALIDA
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
    sensor       = Pin(1,  Pin.IN,  Pin.PULL_DOWN)

    # UART hacia RP2040 Zero con pantalla (GP8=TX, GP9=RX)
    uart_display = init_uart_display()
    display_send(uart_display, "BOOT_SALIDA")   # notifica arranque al display

    radio = init_radio(ADDR_SALIDA)

    def tx(addr, msg, val=0, rep=REPEAT_CRITICO):
        return enviar(radio, ADDR_SALIDA, addr, msg, val, rep)

    estado           = S_IDLE
    t_sema           = 0
    sensor_disparado = False
    t_sensor_us      = 0          # timestamp en µs capturado en ISR
    t_ultimo_hb      = utime.ticks_ms()
    t_ultimo_sensor  = 0
    ultimo_estado_radio = None    # para actualizar display solo cuando cambia

    def isr_sensor(pin):
        nonlocal sensor_disparado, t_ultimo_sensor, t_sensor_us
        ahora_ms = utime.ticks_ms()
        if utime.ticks_diff(ahora_ms, t_ultimo_sensor) > DEBOUNCE_MS:
            t_sensor_us      = utime.ticks_us()   # captura precisa en µs
            sensor_disparado = True
            t_ultimo_sensor  = ahora_ms

    def apagar_semaforo():
        led_rojo.value(0); led_amarillo.value(0)
        led_verde.value(0); led_error.value(0)

    def semaforo_error():
        led_rojo.value(1); led_amarillo.value(1)
        led_verde.value(1); led_error.value(1)

    def dis_sensor():
        try: sensor.irq(handler=None)
        except: pass

    def hab_sensor():
        try: sensor.irq(trigger=Pin.IRQ_RISING, handler=isr_sensor)
        except Exception as e: print("[SALIDA] IRQ error:", e)

    print("[SALIDA] Listo. Esperando PREPARAR_PISTA...")

    while True:
        try:
            ahora = utime.ticks_ms()
            wdt.feed()

            # ── Heartbeat ──────────────────────────────────────────────────
            if utime.ticks_diff(ahora, t_ultimo_hb) >= HEARTBEAT_MS:
                ok = tx(ADDR_MAESTRO, MSG_HEARTBEAT, ROL_SALIDA, REPEAT_HB)
                t_ultimo_hb = utime.ticks_ms()

                # Actualizar indicador de radio en display (solo si cambia)
                radio_activo = 1   # si pudo enviar HB, asumimos radio OK
                if radio_activo != ultimo_estado_radio:
                    display_send(uart_display, "RADIO:{}".format(radio_activo))
                    ultimo_estado_radio = radio_activo

            # ── Recepción ──────────────────────────────────────────────────
            if radio.any():
                try:
                    data          = radio.recv()
                    msg_id, valor = parse_payload(data)

                    if msg_id == MSG_PREPARAR_PISTA and estado == S_IDLE:
                        print("[SALIDA] PREPARAR_PISTA → ROJO")
                        dis_sensor(); apagar_semaforo()
                        sensor_disparado = False
                        led_rojo.value(1)
                        estado = S_ROJO
                        t_sema = utime.ticks_ms()
                        hab_sensor()
                        display_send(uart_display, "PREPARAR")

                    elif msg_id == MSG_ERROR_FALSO:
                        print("[SALIDA] ERROR_FALSO recibido desde META → semáforo error")
                        dis_sensor()
                        semaforo_error()
                        sensor_disparado = False
                        estado = S_ERROR
                        display_send(uart_display, "FALSO")

                    elif msg_id == MSG_RESET:
                        print("[SALIDA] RESET → IDLE")
                        dis_sensor(); apagar_semaforo()
                        sensor_disparado = False
                        estado = S_IDLE
                        display_send(uart_display, "RESET")

                except Exception as e:
                    print("[SALIDA] Error RX:", e)

            # ── Semáforo ───────────────────────────────────────────────────
            if estado == S_ROJO:
                if utime.ticks_diff(ahora, t_sema) >= T_ROJO_MS:
                    led_rojo.value(0); led_amarillo.value(1)
                    estado = S_AMARILLO; t_sema = ahora
                    print("[SALIDA] → AMARILLO")
                    display_send(uart_display, "AMARILLO")

            elif estado == S_AMARILLO:
                if utime.ticks_diff(ahora, t_sema) >= T_AMARILLO_MS:
                    led_amarillo.value(0); led_verde.value(1)
                    sensor_disparado = False
                    estado = S_VERDE
                    print("[SALIDA] → VERDE  |  Sensor ACTIVO")
                    display_send(uart_display, "VERDE")

            # ── Sensor disparado ───────────────────────────────────────────
            if sensor_disparado:
                sensor_disparado = False
                dis_sensor()

                if estado == S_VERDE:
                    print("[SALIDA] Cruce VÁLIDO → START_CRONO + ARM_META")
                    # Enviamos el timestamp de cruce al Maestro (solo referencia, no se usa en tiempo)
                    tx(ADDR_MAESTRO, MSG_START_CRONO)
                    tx(ADDR_META,    MSG_ARM_META)
                    estado = S_ARRANCADO

                elif estado in (S_ROJO, S_AMARILLO):
                    print("[SALIDA] !! SALIDA EN FALSO")
                    semaforo_error()
                    tx(ADDR_MAESTRO, MSG_ERROR_FALSO)
                    tx(ADDR_META,    MSG_ERROR_FALSO)
                    estado = S_ERROR
                    display_send(uart_display, "FALSO")

        except Exception as e:
            print("[SALIDA] Excepción loop:", e)
            utime.sleep_ms(100)
        utime.sleep_ms(10)

# ─────────────────────────────────────────────────────────────────────────────
#  NODO META
# ─────────────────────────────────────────────────────────────────────────────
def run_meta():
    print("=" * 44)
    print("  PISTA DE ARRANCONES  –  NODO META")
    print("=" * 44)

    wdt = WDT(timeout=WDT_MS)

    sensor_meta = Pin(1, Pin.IN, Pin.PULL_DOWN)
    led_error   = Pin(7, Pin.OUT, value=0)

    # UART hacia RP2040 Zero con pantalla (GP8=TX, GP9=RX)
    uart_display = init_uart_display()
    display_send(uart_display, "BOOT_META")    # notifica arranque al display

    radio = init_radio(ADDR_META)

    def tx(addr, msg, val=0, rep=REPEAT_CRITICO):
        return enviar(radio, ADDR_META, addr, msg, val, rep)

    estado           = M_IDLE
    sensor_disparado = False
    t_arm_us         = 0          # timestamp en µs cuando llega ARM_META
    t_sensor_us      = 0          # timestamp en µs capturado en ISR
    t_ultimo_hb      = utime.ticks_ms()
    t_ultimo_sensor  = 0
    ultimo_estado_radio = None

    def isr_meta(pin):
        nonlocal sensor_disparado, t_ultimo_sensor, t_sensor_us
        ahora_ms = utime.ticks_ms()
        if utime.ticks_diff(ahora_ms, t_ultimo_sensor) > DEBOUNCE_MS:
            t_sensor_us      = utime.ticks_us()   # captura precisa en µs
            sensor_disparado = True
            t_ultimo_sensor  = ahora_ms

    def dis_sensor():
        try: sensor_meta.irq(handler=None)
        except: pass

    def hab_sensor():
        try: sensor_meta.irq(trigger=Pin.IRQ_RISING, handler=isr_meta)
        except Exception as e: print("[META] IRQ error:", e)

    print("[META] Listo. Esperando PREPARAR_PISTA...")

    while True:
        try:
            ahora = utime.ticks_ms()
            wdt.feed()

            # ── Heartbeat ──────────────────────────────────────────────────
            if utime.ticks_diff(ahora, t_ultimo_hb) >= HEARTBEAT_MS:
                ok = tx(ADDR_MAESTRO, MSG_HEARTBEAT, ROL_META, REPEAT_HB)
                t_ultimo_hb = utime.ticks_ms()

                radio_activo = 1
                if radio_activo != ultimo_estado_radio:
                    display_send(uart_display, "RADIO:{}".format(radio_activo))
                    ultimo_estado_radio = radio_activo

            # ── Recepción ──────────────────────────────────────────────────
            if radio.any():
                try:
                    data          = radio.recv()
                    msg_id, valor = parse_payload(data)

                    if msg_id == MSG_PREPARAR_PISTA:
                        print("[META] PREPARAR_PISTA → preparado")
                        dis_sensor(); sensor_disparado = False
                        led_error.value(0); estado = M_PREPARADO
                        hab_sensor()
                        display_send(uart_display, "PREPARAR")

                    elif msg_id == MSG_ARM_META and estado == M_PREPARADO:
                        print("[META] ARM_META → sensor ARMADO")
                        sensor_disparado = False
                        t_arm_us = utime.ticks_us()   # marca de tiempo precisa en µs
                        hab_sensor(); estado = M_ARMADO

                    elif msg_id == MSG_ERROR_FALSO:
                        print("[META] ERROR_FALSO recibido → LED error ON")
                        dis_sensor(); sensor_disparado = False
                        led_error.value(1)
                        estado = M_ERROR
                        display_send(uart_display, "FALSO")

                    elif msg_id == MSG_RESET:
                        print("[META] RESET → IDLE")
                        dis_sensor(); sensor_disparado = False
                        led_error.value(0); estado = M_IDLE
                        display_send(uart_display, "RESET")

                except Exception as e:
                    print("[META] Error RX:", e)

            # ── Sensor disparado ───────────────────────────────────────────
            if sensor_disparado:
                sensor_disparado = False
                dis_sensor()

                if estado == M_ARMADO:
                    # Tiempo medido completamente en µs dentro del nodo Meta
                    # → máxima precisión posible sin latencia de red
                    t_carrera_us = utime.ticks_diff(t_sensor_us, t_arm_us)
                    print("[META] FINISH  tiempo: {}".format(us_a_str(t_carrera_us)))
                    tx(ADDR_MAESTRO, MSG_FINISH, t_carrera_us)
                    estado = M_REGISTRADO
                    display_send(uart_display, "CRUCE")

                elif estado == M_PREPARADO:
                    print("[META] !! ERROR: cruce prematuro en META")
                    led_error.value(1)
                    tx(ADDR_MAESTRO, MSG_ERROR_FALSO)
                    tx(ADDR_SALIDA,  MSG_ERROR_FALSO)
                    estado = M_ERROR
                    display_send(uart_display, "FALSO")

        except Exception as e:
            print("[META] Excepción loop:", e)
            utime.sleep_ms(100)
        utime.sleep_ms(10)

# ─────────────────────────────────────────────────────────────────────────────
#  PUNTO DE ENTRADA
# ─────────────────────────────────────────────────────────────────────────────
def main():
    utime.sleep_ms(300)
    rol = detectar_rol()
    if   rol == ROL_MAESTRO: run_maestro()
    elif rol == ROL_SALIDA:  run_salida()
    elif rol == ROL_META:    run_meta()

main()
