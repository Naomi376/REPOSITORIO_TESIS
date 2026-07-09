# =============================================================================
#  display_salida.py  –  RP2040 Zero  |  Pantalla ST7789V2  |  Nodo SALIDA
# =============================================================================
#  PINOUT RP2040 Zero ↔ ST7789V2
#  ──────────────────────────────
#  SPI1 SCK  → GP10
#  SPI1 MOSI → GP11
#  CS        → GP9
#  DC        → GP8
#  RST       → GP7
#  BL        → GP6  (backlight)
#
#  UART desde Pico SALIDA
#  ──────────────────────
#  UART0 RX  → GP1   (conectar al GP8/TX del Pico SALIDA)
#  GND       → GND común con el Pico
#
#  PROTOCOLO UART (mensajes terminados en \n)
#  ──────────────────────────────────────────
#  "BOOT_SALIDA"  → Pantalla de bienvenida
#  "PREPARAR"     → "¡ATENTOS!\nSemáforo inicia..."
#  "ROJO"         → "ROJO - Preparate"
#  "AMARILLO"     → "AMARILLO - Ya casi..."
#  "VERDE"        → "¡VAMOOOOOS!"
#  "FALSO"        → "!! SALIDA EN FALSO !!"
#  "RESET"        → "SISTEMA RESETEADO\nPista lista"
#  "RADIO:1"      → Estado radio = ACTIVO
#  "RADIO:0"      → Estado radio = SIN SEÑAL
# =============================================================================

from machine import SPI, Pin, UART
import framebuf
import utime

# ─────────────────────────────────────────────────────────────────────────────
#  COLORES RGB565
# ─────────────────────────────────────────────────────────────────────────────
NEGRO     = 0x0000
BLANCO    = 0xFFFF
ROJO      = 0xF800
VERDE     = 0x07E0
AZUL      = 0x001F
AMARILLO  = 0xFFE0
NARANJA   = 0xFC00
CYAN      = 0x07FF
GRIS      = 0x7BEF
GRIS_OSC  = 0x39E7

# ─────────────────────────────────────────────────────────────────────────────
#  DIMENSIONES PANTALLA
# ─────────────────────────────────────────────────────────────────────────────
W = 240
H = 320

# ─────────────────────────────────────────────────────────────────────────────
#  DRIVER MÍNIMO ST7789  (sin dependencia de st7789py.py)
#  Adaptado para RP2040 Zero con SPI1
# ─────────────────────────────────────────────────────────────────────────────
class ST7789:
    def __init__(self, spi, cs, dc, rst, bl):
        self._spi = spi
        self._cs  = cs
        self._dc  = dc
        self._rst = rst
        self._bl  = bl
        self._init()

    def _write_cmd(self, cmd):
        self._dc.value(0); self._cs.value(0)
        self._spi.write(bytes([cmd]))
        self._cs.value(1)

    def _write_data(self, data):
        self._dc.value(1); self._cs.value(0)
        self._spi.write(data if isinstance(data, (bytes, bytearray)) else bytes([data]))
        self._cs.value(1)

    def _init(self):
        self._bl.value(1)
        self._rst.value(1); utime.sleep_ms(50)
        self._rst.value(0); utime.sleep_ms(50)
        self._rst.value(1); utime.sleep_ms(150)

        for cmd, data in [
            (0x11, None),                               # Sleep out
        ]:
            self._write_cmd(cmd)
            if data: self._write_data(data)
            utime.sleep_ms(120)

        init_cmds = [
            (0x36, bytes([0x00])),                      # MADCTL normal
            (0x3A, bytes([0x55])),                      # Pixel format RGB565
            (0xB2, bytes([0x0C,0x0C,0x00,0x33,0x33])), # Porch control
            (0xB7, bytes([0x35])),                      # Gate control
            (0xBB, bytes([0x19])),                      # VCOMS
            (0xC0, bytes([0x2C])),                      # LCM control
            (0xC2, bytes([0x01])),                      # VDV/VRH enable
            (0xC3, bytes([0x12])),                      # VRH set
            (0xC4, bytes([0x20])),                      # VDV set
            (0xC6, bytes([0x0F])),                      # Frame rate
            (0xD0, bytes([0xA4,0xA1])),                 # Power control 1
            (0xE0, bytes([0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23])),
            (0xE1, bytes([0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23])),
            (0x21, None),                               # Display inversion ON
            (0x29, None),                               # Display ON
        ]
        for cmd, data in init_cmds:
            self._write_cmd(cmd)
            if data: self._write_data(data)
        utime.sleep_ms(10)

    def set_window(self, x0, y0, x1, y1):
        self._write_cmd(0x2A)
        self._write_data(bytes([x0>>8, x0&0xFF, x1>>8, x1&0xFF]))
        self._write_cmd(0x2B)
        self._write_data(bytes([y0>>8, y0&0xFF, y1>>8, y1&0xFF]))
        self._write_cmd(0x2C)

    def blit_buffer(self, buf, x, y, w, h):
        self.set_window(x, y, x+w-1, y+h-1)
        self._dc.value(1); self._cs.value(0)
        self._spi.write(buf)
        self._cs.value(1)


# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS DE DIBUJO
# ─────────────────────────────────────────────────────────────────────────────
fbuf_bytes = bytearray(W * H * 2)
fb = framebuf.FrameBuffer(fbuf_bytes, W, H, framebuf.RGB565)


def texto_escalado(cadena, x, y, color, escala=1):
    """Dibuja texto escalado en el framebuffer global fb."""
    ancho_texto = len(cadena) * 8
    if ancho_texto == 0:
        return
    tmp_buf = bytearray(ancho_texto * 8)
    tmp_fb  = framebuf.FrameBuffer(tmp_buf, ancho_texto, 8, framebuf.MONO_VLSB)
    tmp_fb.text(cadena, 0, 0, 1)
    for py in range(8):
        for px in range(ancho_texto):
            if tmp_fb.pixel(px, py):
                fb.fill_rect(x + px * escala, y + py * escala, escala, escala, color)


def centrar_texto(cadena, y, color, escala=3):
    """Centra texto horizontalmente."""
    ancho = len(cadena) * 8 * escala
    x = (W - ancho) // 2
    if x < 0:
        x = 0
    texto_escalado(cadena, x, y, color, escala)


def volcar():
    """Envía el framebuffer completo a la pantalla."""
    tft.blit_buffer(fbuf_bytes, 0, 0, W, H)


# ─────────────────────────────────────────────────────────────────────────────
#  BARRA DE ESTADO RADIO (esquina superior derecha)
# ─────────────────────────────────────────────────────────────────────────────
estado_radio = True    # asume activo al inicio

def dibujar_barra_radio():
    """Dibuja indicador de radio en esquina superior izquierda. No vuelca pantalla."""
    # Rectángulo de fondo
    fb.fill_rect(0, 0, 140, 18, NEGRO)
    if estado_radio:
        fb.fill_rect(2, 2, 12, 14, VERDE)
        texto_escalado("RADIO:OK", 16, 4, VERDE, escala=1)
    else:
        fb.fill_rect(2, 2, 12, 14, ROJO)
        texto_escalado("RADIO:--", 16, 4, ROJO, escala=1)


# ─────────────────────────────────────────────────────────────────────────────
#  PANTALLAS
# ─────────────────────────────────────────────────────────────────────────────
def pantalla_bienvenida():
    fb.fill(NEGRO)
    centrar_texto("DRAG", 60, CYAN, escala=5)
    centrar_texto("RACER", 105, CYAN, escala=5)
    centrar_texto("NODO SALIDA", 170, BLANCO, escala=2)
    centrar_texto("Esperando...", 210, GRIS, escala=2)
    dibujar_barra_radio()
    volcar()


def pantalla_preparar():
    fb.fill(NEGRO)
    fb.fill_rect(0, 22, W, 4, AMARILLO)   # línea separadora
    centrar_texto("ATENTOS!", 50, AMARILLO, escala=4)
    centrar_texto("Semaforo", 120, BLANCO, escala=3)
    centrar_texto("iniciando...", 160, BLANCO, escala=2)
    dibujar_barra_radio()
    volcar()


def pantalla_rojo():
    fb.fill(NEGRO)
    # Círculo simulado con rectángulo relleno grande
    fb.fill_rect(80, 60, 80, 80, ROJO)
    centrar_texto("ALTO", 160, ROJO, escala=4)
    centrar_texto("Prepárate...", 220, BLANCO, escala=2)
    dibujar_barra_radio()
    volcar()


def pantalla_amarillo():
    fb.fill(NEGRO)
    fb.fill_rect(80, 60, 80, 80, AMARILLO)
    centrar_texto("LISTO", 160, AMARILLO, escala=4)
    centrar_texto("Ya casi...", 220, BLANCO, escala=2)
    dibujar_barra_radio()
    volcar()


def pantalla_verde():
    fb.fill(NEGRO)
    fb.fill_rect(80, 40, 80, 80, VERDE)
    centrar_texto("VAMOS!", 150, VERDE, escala=5)
    centrar_texto("Suerte!", 230, BLANCO, escala=2)
    dibujar_barra_radio()
    volcar()


def pantalla_falso():
    fb.fill(NEGRO)
    # Parpadeo visual: raya diagonal de error
    fb.fill_rect(0, 0, W, H, NEGRO)
    centrar_texto("!! FALSO !!", 80, ROJO, escala=3)
    centrar_texto("SALIDA", 130, ROJO, escala=4)
    centrar_texto("EN FALSO", 180, ROJO, escala=3)
    dibujar_barra_radio()
    volcar()


def pantalla_reset():
    fb.fill(NEGRO)
    centrar_texto("RESET", 60, CYAN, escala=4)
    centrar_texto("Sistema listo", 130, BLANCO, escala=2)
    centrar_texto("Nueva carrera", 170, GRIS, escala=2)
    dibujar_barra_radio()
    volcar()


def refrescar_radio():
    """Actualiza solo la barra de radio y vuelca (para cuando cambia el estado)."""
    dibujar_barra_radio()
    # Volcar solo la franja superior (y=0..19) es más rápido
    # Pero blit_buffer parcial requiere calcular offset; volcamos todo por simplicidad
    volcar()


# ─────────────────────────────────────────────────────────────────────────────
#  HARDWARE INIT
# ─────────────────────────────────────────────────────────────────────────────
spi = SPI(1, baudrate=40_000_000, polarity=1, phase=1, sck=Pin(10), mosi=Pin(11))
cs  = Pin(9, Pin.OUT, value=1)
dc  = Pin(8, Pin.OUT)
rst = Pin(7, Pin.OUT)
bl  = Pin(6, Pin.OUT, value=1)

tft = ST7789(spi, cs, dc, rst, bl)

# UART0 RX en GP1 (UART0 default en RP2040)
uart = UART(0, baudrate=115200, rx=Pin(1))

pantalla_bienvenida()
print("[DISPLAY SALIDA] Listo, esperando comandos UART en GP1...")

# ─────────────────────────────────────────────────────────────────────────────
#  LOOP PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────
buf_uart = b""

while True:
    # Leer UART byte a byte hasta encontrar \n
    if uart.any():
        nuevo = uart.read(uart.any())
        if nuevo:
            buf_uart += nuevo

    # Procesar líneas completas
    while b"\n" in buf_uart:
        linea, buf_uart = buf_uart.split(b"\n", 1)
        cmd = linea.decode("utf-8", "ignore").strip()
        print("[CMD]", cmd)

        if cmd == "BOOT_SALIDA":
            pantalla_bienvenida()

        elif cmd == "PREPARAR":
            pantalla_preparar()

        elif cmd == "ROJO":
            pantalla_rojo()

        elif cmd == "AMARILLO":
            pantalla_amarillo()

        elif cmd == "VERDE":
            pantalla_verde()

        elif cmd == "FALSO":
            pantalla_falso()

        elif cmd == "RESET":
            pantalla_reset()

        elif cmd.startswith("RADIO:"):
            val = cmd[6:]
            estado_radio = (val == "1")
            refrescar_radio()

    utime.sleep_ms(5)