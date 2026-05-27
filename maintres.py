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
#
#  META     : SENSOR_META    → GP1   (pull-down, activo en HIGH)
#             LED_ERROR      → GP7   (cruce prematuro)
# =============================================================================

import utime
from machine import Pin, SPI, WDT
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

# Cuántas veces se repite cada mensaje crítico (sin ACK, compensamos con repetición)
REPEAT_CRITICO   = 3
REPEAT_HB        = 1
PAUSA_REPEAT_MS  = 8    # pausa entre repeticiones del mismo mensaje

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

# ─────────────────────────────────────────────────────────────────────────────
#  RADIO  –  sin ACK (send_start) para evitar colisiones con heartbeats
# ─────────────────────────────────────────────────────────────────────────────
def init_radio(addr_rx):
    """
    Inicializa el NRF24L01 en modo NO-ACK.
    addr_rx : dirección en la que este nodo escucha.
    """
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
    """
    Envío sin ACK usando send_start() / send_done().
    - Cambia a TX, envía N veces el mismo mensaje con pequeña pausa,
      luego vuelve a RX.  No bloquea esperando confirmación.
    - addr_rx_propia: pipe RX de este nodo (para restaurar al terminar).
    """
    payload = build_payload(msg_id, valor)
    enviados = 0
    try:
        radio.stop_listening()
        radio.open_tx_pipe(addr_dest)
        utime.sleep_ms(3)                   # settling

        for _ in range(repeticiones):
            try:
                radio.send_start(payload)   # no-blocking, no espera ACK
                utime.sleep_ms(PAUSA_REPEAT_MS)
                radio.send_done()           # libera el buffer TX
                enviados += 1
            except Exception as e:
                print("[RF] send error msg={}: {}".format(msg_id, e))
                utime.sleep_ms(PAUSA_REPEAT_MS)

    except Exception as e:
        print("[RF] Error TX setup msg={}: {}".format(msg_id, e))
    finally:
        # Siempre restaurar a modo RX
        try:
            radio.open_rx_pipe(1, addr_rx_propia)
            radio.start_listening()
        except Exception as e:
            print("[RF] Error restaurando RX: {}".format(e))

    return enviados > 0

# ─────────────────────────────────────────────────────────────────────────────
#  NODO MAESTRO
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
    t_inicio_ms      = 0
    t_fin_ms         = 0
    ultimo_hb_salida = utime.ticks_ms() - TIMEOUT_RADIO_MS   # inicia como "sin señal"
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
                    t_inicio_ms = 0
                    t_fin_ms    = 0
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
                    print("[MAESTRO] Error RX:", e)

            # ── Cronómetro en consola ──────────────────────────────────────
            if estado == S_CORRIENDO:
                if utime.ticks_diff(ahora, t_ultimo_print) >= 100:
                    t_ultimo_print = ahora
                    elapsed = utime.ticks_diff(ahora, t_inicio_ms)
                    print("\r[CRONO] {} ".format(ms_a_str(elapsed)), end="")

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

    radio = init_radio(ADDR_SALIDA)

    def tx(addr, msg, val=0, rep=REPEAT_CRITICO):
        return enviar(radio, ADDR_SALIDA, addr, msg, val, rep)

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
                tx(ADDR_MAESTRO, MSG_HEARTBEAT, ROL_SALIDA, REPEAT_HB)
                t_ultimo_hb = utime.ticks_ms()

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

                    elif msg_id == MSG_RESET:
                        print("[SALIDA] RESET → IDLE")
                        dis_sensor(); apagar_semaforo()
                        sensor_disparado = False
                        estado = S_IDLE

                except Exception as e:
                    print("[SALIDA] Error RX:", e)

            # ── Semáforo ───────────────────────────────────────────────────
            if estado == S_ROJO:
                if utime.ticks_diff(ahora, t_sema) >= T_ROJO_MS:
                    led_rojo.value(0); led_amarillo.value(1)
                    estado = S_AMARILLO; t_sema = ahora
                    print("[SALIDA] → AMARILLO")

            elif estado == S_AMARILLO:
                if utime.ticks_diff(ahora, t_sema) >= T_AMARILLO_MS:
                    led_amarillo.value(0); led_verde.value(1)
                    sensor_disparado = False
                    hab_sensor()
                    estado = S_VERDE
                    print("[SALIDA] → VERDE  |  Sensor ACTIVO")

            # ── Sensor disparado ───────────────────────────────────────────
            if sensor_disparado:
                sensor_disparado = False
                dis_sensor()

                if estado == S_VERDE:
                    print("[SALIDA] Cruce VÁLIDO → START_CRONO + ARM_META")
                    tx(ADDR_MAESTRO, MSG_START_CRONO)
                    tx(ADDR_META,    MSG_ARM_META)
                    estado = S_ARRANCADO

                elif estado in (S_ROJO, S_AMARILLO):
                    print("[SALIDA] !! SALIDA EN FALSO")
                    semaforo_error()
                    tx(ADDR_MAESTRO, MSG_ERROR_FALSO)
                    estado = S_ERROR

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

    radio = init_radio(ADDR_META)

    def tx(addr, msg, val=0, rep=REPEAT_CRITICO):
        return enviar(radio, ADDR_META, addr, msg, val, rep)

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
                tx(ADDR_MAESTRO, MSG_HEARTBEAT, ROL_META, REPEAT_HB)
                t_ultimo_hb = utime.ticks_ms()

            # ── Recepción ──────────────────────────────────────────────────
            if radio.any():
                try:
                    data          = radio.recv()
                    msg_id, valor = parse_payload(data)

                    if msg_id == MSG_PREPARAR_PISTA:
                        print("[META] PREPARAR_PISTA → preparado")
                        dis_sensor(); sensor_disparado = False
                        led_error.value(0); estado = M_PREPARADO

                    elif msg_id == MSG_ARM_META and estado == M_PREPARADO:
                        print("[META] ARM_META → sensor ARMADO")
                        sensor_disparado = False
                        t_arm = utime.ticks_ms()
                        hab_sensor(); estado = M_ARMADO

                    elif msg_id == MSG_RESET:
                        print("[META] RESET → IDLE")
                        dis_sensor(); sensor_disparado = False
                        led_error.value(0); estado = M_IDLE

                except Exception as e:
                    print("[META] Error RX:", e)

            # ── Sensor disparado ───────────────────────────────────────────
            if sensor_disparado:
                sensor_disparado = False
                dis_sensor()
                t_cruce = utime.ticks_ms()

                if estado == M_ARMADO:
                    t_carrera = utime.ticks_diff(t_cruce, t_arm)
                    print("[META] FINISH  tiempo: {}".format(ms_a_str(t_carrera)))
                    tx(ADDR_MAESTRO, MSG_FINISH, t_carrera)
                    estado = M_REGISTRADO

                elif estado == M_PREPARADO:
                    print("[META] !! ERROR: cruce prematuro")
                    led_error.value(1)
                    tx(ADDR_MAESTRO, MSG_ERROR_FALSO)
                    estado = M_ERROR

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