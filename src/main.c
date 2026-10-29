/********************************************************************************************* */
//    Eduboard2 ESP32-S3 - TMP112 Temperatur auf dem Display anzeigen
//
//    - Liest jede Sekunde den TMP112 (I2C, Adresse 0x48) aus
//    - Zeigt Temperatur und die empfangenen Rohdaten (binaer) auf dem ILI9488 Display an
//    - Sendet das Paket (Gray Code) als QAM-4 ueber den MCP4802 Dual-DAC:
//        DAC A = I-Anteil  I * cos(wt)
//        DAC B = Q-Anteil  Q * sin(wt)
//      Fuer den Demodulator (ein ADC-Eingang) beide Ausgaenge ueber je einen Widerstand
//      (z.B. 2x 10k) zusammenfuehren -> I*cos + Q*sin. Alternativ QAM_DAC_A_SUMME = 1 setzen.
//
//    Pins laut https://juventus-technikerschule-hf.github.io/eduboard2/docs/hw-baseboard
/********************************************************************************************* */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_timer.h"

// ---------------- Pins ----------------
#define PIN_I2C_SDA     2
#define PIN_I2C_SCL     1

#define PIN_LCD_MOSI    13
#define PIN_LCD_SCK     14
#define PIN_LCD_CS      11
#define PIN_LCD_DC      16
#define PIN_LCD_RESET   8

// MCP4802 haengt am gleichen SPI Bus wie das Display (MOSI 13, SCK 14)
#define PIN_DAC_CS      3       // geteilt mit Flash_CS (Flash normalerweise nicht bestueckt)
#define PIN_DAC_LDAC    10      // geteilt mit SD_CS -> keine SD Karte einstecken

#define TMP112_ADDR     0x48
#define TMP112_REG_TEMP 0x00

#define LCD_WIDTH       320
#define LCD_HEIGHT      480
#define FONT_SCALE      2       // 5x7 Font, 2-fach vergroessert (26 Zeichen pro Zeile)

// ---------------- Datenpaket: START / DATA / CHECK (je 12 Bit) ----------------
// Der TMP112 liefert nur Werte von -55 bis +150 Grad C:
//   +150 C = 0x960,  -55 C = 0xC90  ->  0x961..0xC8F kommen als Temperatur nie vor.
// Daraus wird die Startsequenz gewaehlt (abwechselndes Bitmuster, gut erkennbar).
#define PACKET_START    0xA5A   // 1010 0101 1010 = 165.6 C -> kein gueltiger Messwert

// ---------------- QAM-4 Parameter (muessen zum Demodulator passen) ----------------
// Traeger 1 kHz, 4 Perioden pro Symbol -> 250 Symbole/s, 2 Bit/Symbol -> 500 bit/s
#define CARRIER_HZ              1000
#define CARRIER_CYCLES_SYMBOL   4
// Ausgaberate 32 kS/s: pro Sample 2 SPI Transfers, das reicht zeitlich sicher.
// Der Empfaenger tastet trotzdem mit 64 kS/s ab (die Treppenstufen stoeren die 1 kHz Korrelation nicht).
#define DAC_SAMPLES_PER_CARRIER 32
#define DAC_SAMPLE_RATE_HZ      (CARRIER_HZ * DAC_SAMPLES_PER_CARRIER)
#define DAC_SAMPLES_PER_SYMBOL  (DAC_SAMPLES_PER_CARRIER * CARRIER_CYCLES_SYMBOL)

#define PREAMBLE_SYMBOLS        8       // 00,11,00,11,... zur Synchronisation des Empfaengers
#define PAYLOAD_SYMBOLS         18      // 36 Bit START|DATA|CHECK / 2 Bit pro Symbol

#define DAC_MID                 128     // Arbeitspunkt: Mitte des 8 Bit DAC (1.024 V bei Gain 1x)
#define DAC_AMPLITUDE           85      // I*cos + Q*sin erreicht max. sqrt(2) * 85 = 120 -> kein Clipping

// 0: DAC A = I*cos, DAC B = Q*sin
// 1: DAC A = I*cos + Q*sin (fertiges QAM Signal direkt auf den ADC), DAC B = Q*sin
#define QAM_DAC_A_SUMME         0

// MCP4802 Kommandowort: Bit 15 = Kanal, Bit 13 = Gain 1x, Bit 12 = Ausgang aktiv, Bit 11..4 = Daten
#define MCP4802_DAC_A           0x0000
#define MCP4802_DAC_B           0x8000
#define MCP4802_GAIN_1X         0x2000
#define MCP4802_ACTIVE          0x1000

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static i2c_master_dev_handle_t tmp112;
static spi_device_handle_t lcd;
static spi_device_handle_t dac;

// Traegertabellen fuer eine Periode, bereits mit DAC_AMPLITUDE skaliert
static int8_t carrier_cos[DAC_SAMPLES_PER_CARRIER];
static int8_t carrier_sin[DAC_SAMPLES_PER_CARRIER];

// ---------------- Temperatursensor ----------------
static void tmp112_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TMP112_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &tmp112));
}

// Liest das Temperaturregister: raw[0] = MSB, raw[1] = LSB
// Rueckgabe: Temperatur in Grad Celsius
static float tmp112_read(uint8_t raw[2])
{
    uint8_t reg = TMP112_REG_TEMP;
    i2c_master_transmit_receive(tmp112, &reg, 1, raw, 2, 100);

    // 12 Bit Wert steht linksbuendig in den 16 Bit -> 4 Bit nach rechts schieben
    int16_t value = (int16_t)((raw[0] << 8) | raw[1]) >> 4;
    return value * 0.0625f;     // 1 LSB = 0.0625 Grad C
}

// ---------------- Display (ILI9488 ueber SPI) ----------------
static void lcd_cmd(uint8_t cmd)
{
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    gpio_set_level(PIN_LCD_DC, 0);
    spi_device_polling_transmit(lcd, &t);
}

static void lcd_data(const uint8_t *data, int len)
{
    spi_transaction_t t = { .length = len * 8, .tx_buffer = data };
    gpio_set_level(PIN_LCD_DC, 1);
    spi_device_polling_transmit(lcd, &t);
}

static void lcd_init(void)
{
    gpio_set_direction(PIN_LCD_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LCD_RESET, GPIO_MODE_OUTPUT);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_LCD_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &lcd));

    // Hardware Reset
    gpio_set_level(PIN_LCD_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_LCD_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(0x3A); lcd_data((uint8_t[]){0x66}, 1);  // 18 Bit pro Pixel (SPI kann kein 16 Bit beim ILI9488)
    lcd_cmd(0x36); lcd_data((uint8_t[]){0x48}, 1);  // Ausrichtung / BGR
    lcd_cmd(0x11);                                  // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x29);                                  // Display On
}

static void lcd_set_window(int x, int y, int w, int h)
{
    int x2 = x + w - 1, y2 = y + h - 1;
    lcd_cmd(0x2A); lcd_data((uint8_t[]){x >> 8, x & 0xFF, x2 >> 8, x2 & 0xFF}, 4);
    lcd_cmd(0x2B); lcd_data((uint8_t[]){y >> 8, y & 0xFF, y2 >> 8, y2 & 0xFF}, 4);
    lcd_cmd(0x2C);  // ab hier folgen Pixeldaten
}

static void lcd_clear(void)
{
    static uint8_t line[LCD_WIDTH * 3];     // eine Zeile schwarz (3 Byte pro Pixel)
    memset(line, 0, sizeof(line));
    lcd_set_window(0, 0, LCD_WIDTH, LCD_HEIGHT);
    for (int y = 0; y < LCD_HEIGHT; y++) {
        lcd_data(line, sizeof(line));
    }
}

// 5x7 Font fuer ASCII ' ' bis 'Z' (spaltenweise, Bit 0 = oberste Zeile)
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14}, //  !"#
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, // $%&'
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08}, // ()*+
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02}, // ,-./
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, // 0123
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, // 4567
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00}, // 89:;
    {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14}, {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06}, // <=>?
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22}, // @ABC
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x01,0x01}, {0x3E,0x41,0x41,0x51,0x32}, // DEFG
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, // HIJK
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E}, // LMNO
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31}, // PQRS
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F}, // TUVW
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03}, {0x61,0x51,0x49,0x45,0x43},                              // XYZ
};

#define CHAR_W  (6 * FONT_SCALE)    // 5 Pixel + 1 Pixel Abstand
#define CHAR_H  (8 * FONT_SCALE)

// Zeichnet ein Zeichen weiss auf schwarz (Kleinbuchstaben werden als Grossbuchstaben dargestellt)
static void lcd_char(int x, int y, char c)
{
    static uint8_t buf[CHAR_W * CHAR_H * 3];
    c = toupper((unsigned char)c);
    if (c < ' ' || c > 'Z') c = ' ';
    const uint8_t *glyph = font5x7[c - ' '];

    for (int py = 0; py < CHAR_H; py++) {
        for (int px = 0; px < CHAR_W; px++) {
            int col = px / FONT_SCALE, row = py / FONT_SCALE;
            uint8_t on = (col < 5) && (glyph[col] & (1 << row));
            memset(&buf[(py * CHAR_W + px) * 3], on ? 0xFC : 0x00, 3);
        }
    }
    lcd_set_window(x, y, CHAR_W, CHAR_H);
    lcd_data(buf, sizeof(buf));
}

static void lcd_text(int x, int y, const char *s)
{
    while (*s) {
        lcd_char(x, y, *s++);
        x += CHAR_W;
    }
}

// Wandelt die untersten 'bits' Bits von value in einen String aus '0' und '1' um
static void to_binary(uint16_t value, int bits, char *out)
{
    for (int i = 0; i < bits; i++) {
        out[i] = (value & (1 << (bits - 1 - i))) ? '1' : '0';
    }
    out[bits] = '\0';
}

// Wandelt einen Binaerwert in Gray Code um (benachbarte Werte unterscheiden sich in nur 1 Bit)
static uint16_t to_gray(uint16_t value)
{
    return value ^ (value >> 1);
}

typedef struct {
    uint16_t start;     // alle Felder bereits in Gray Code
    uint16_t data;
    uint16_t check;
} packet_t;

// Checksumme: Summe der drei 4-Bit Nibbles des Temperaturwerts, nur die untersten 4 Bit
// -> Wert 0x000..0x00F, kann darum nie wie die Startsequenz aussehen
static uint16_t checksum(uint16_t value12)
{
    return (((value12 >> 8) & 0xF) + ((value12 >> 4) & 0xF) + (value12 & 0xF)) & 0xF;
}

// Baut das Paket START / DATA / CHECK und wandelt jedes Feld in Gray Code um
static packet_t build_packet(uint16_t value12)
{
    packet_t p = {
        .start = to_gray(PACKET_START),
        .data  = to_gray(value12),
        .check = to_gray(checksum(value12)),
    };
    return p;
}

// ---------------- DAC MCP4802 / QAM-4 Modulator ----------------
static void dac_write_channel(uint16_t channel, uint8_t value)
{
    uint16_t word = channel | MCP4802_GAIN_1X | MCP4802_ACTIVE | ((uint16_t)value << 4);
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 16,
        .tx_data = { word >> 8, word & 0xFF },
    };
    spi_device_polling_transmit(dac, &t);
}

// Schreibt beide Kanaele und uebernimmt sie mit dem LDAC Puls gleichzeitig an die Ausgaenge
static void dac_write(uint8_t a, uint8_t b)
{
    dac_write_channel(MCP4802_DAC_A, a);
    dac_write_channel(MCP4802_DAC_B, b);
    gpio_set_level(PIN_DAC_LDAC, 0);
    gpio_set_level(PIN_DAC_LDAC, 1);
}

static void dac_init(void)
{
    gpio_set_direction(PIN_DAC_LDAC, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DAC_LDAC, 1);

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 20 * 1000 * 1000,     // MCP4802 max. 20 MHz
        .mode = 0,
        .spics_io_num = PIN_DAC_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &dac));     // Bus wurde in lcd_init erstellt

    for (int n = 0; n < DAC_SAMPLES_PER_CARRIER; n++) {
        float phase = 2.0f * (float)M_PI * n / DAC_SAMPLES_PER_CARRIER;
        carrier_cos[n] = (int8_t)lroundf(DAC_AMPLITUDE * cosf(phase));
        carrier_sin[n] = (int8_t)lroundf(DAC_AMPLITUDE * sinf(phase));
    }
    dac_write(DAC_MID, DAC_MID);    // kein Traeger bis zum ersten Paket
}

// Sendet Preamble + Paket als QAM-4. Mapping: Bit 1 -> I, Bit 0 -> Q (0 = -1, 1 = +1)
//   00 -> I=-1 Q=-1,  01 -> I=-1 Q=+1,  10 -> I=+1 Q=-1,  11 -> I=+1 Q=+1
// Blockiert (8 + 18) Symbole * 4 ms = 104 ms. Rueckgabe: Anzahl zu spaet ausgegebener Samples
static int qam_send_packet(const packet_t *p)
{
    uint8_t symbols[PREAMBLE_SYMBOLS + PAYLOAD_SYMBOLS];
    int count = 0;

    for (int k = 0; k < PREAMBLE_SYMBOLS; k++) {
        symbols[count++] = (k % 2) ? 0x3 : 0x0;
    }

    // 36 Bit START|DATA|CHECK, hoechstwertige Bits zuerst (so setzt der Empfaenger das Paket zusammen)
    uint64_t frame = ((uint64_t)p->start << 24) | ((uint64_t)p->data << 12) | p->check;
    for (int s = PAYLOAD_SYMBOLS - 1; s >= 0; s--) {
        symbols[count++] = (frame >> (2 * s)) & 0x3;
    }

    // Bus fuer die ganze Ausgabe reservieren -> Display wartet, SPI Transfers werden schneller
    spi_device_acquire_bus(dac, portMAX_DELAY);

    int late = 0;
    uint32_t n = 0;
    int64_t t0 = esp_timer_get_time();

    for (int k = 0; k < count; k++) {
        int i = (symbols[k] & 0x2) ? 1 : -1;
        int q = (symbols[k] & 0x1) ? 1 : -1;

        // Jedes Symbol beginnt bei Traegerphase 0 (ganze Anzahl Perioden pro Symbol)
        for (int m = 0; m < DAC_SAMPLES_PER_SYMBOL; m++, n++) {
            int i_part = i * carrier_cos[m % DAC_SAMPLES_PER_CARRIER];
            int q_part = q * carrier_sin[m % DAC_SAMPLES_PER_CARRIER];
#if QAM_DAC_A_SUMME
            dac_write(DAC_MID + i_part + q_part, DAC_MID + q_part);
#else
            dac_write(DAC_MID + i_part, DAC_MID + q_part);
#endif
            // Auf den naechsten Abtastzeitpunkt warten (absolut ab t0 gerechnet -> kein Driften)
            int64_t next = t0 + ((int64_t)(n + 1) * 1000000) / DAC_SAMPLE_RATE_HZ;
            if (esp_timer_get_time() > next) late++;
            while (esp_timer_get_time() < next) { }
        }
    }

    dac_write(DAC_MID, DAC_MID);    // Traeger aus -> Pause, damit der Empfaenger neu triggert
    spi_device_release_bus(dac);
    return late;
}

// ---------------- Task ----------------
static void temperature_task(void *arg)
{
    char line[32], bits[13];
    uint8_t raw[2];

    lcd_text(10, 10,  "TMP112 -> QAM PAKET");
    lcd_text(10, 70,  "Rohdaten I2C:");
    lcd_text(10, 190, "Paket in Gray Code:");

    while (1) {
        float temp = tmp112_read(raw);
        snprintf(line, sizeof(line), "T = %.4f C  ", temp);
        lcd_text(10, 35, line);

        // Rohdaten vom Sensor
        to_binary(raw[0], 8, bits);
        snprintf(line, sizeof(line), "MSB   %s", bits);
        lcd_text(10, 95, line);
        to_binary(raw[1], 8, bits);
        snprintf(line, sizeof(line), "LSB   %s", bits);
        lcd_text(10, 120, line);
        snprintf(line, sizeof(line), "HEX   0x%02X%02X", raw[0], raw[1]);
        lcd_text(10, 145, line);

        // 12 Bit Zweierkomplement -> Paket START / DATA / CHECK in Gray Code
        uint16_t value12 = ((raw[0] << 8) | raw[1]) >> 4;
        packet_t p = build_packet(value12);

        to_binary(p.start, 12, bits);
        snprintf(line, sizeof(line), "START %s", bits);
        lcd_text(10, 215, line);
        to_binary(p.data, 12, bits);
        snprintf(line, sizeof(line), "DATA  %s", bits);
        lcd_text(10, 240, line);
        to_binary(p.check, 12, bits);
        snprintf(line, sizeof(line), "CHECK %s", bits);
        lcd_text(10, 265, line);
        snprintf(line, sizeof(line), "HEX   %03X/%03X/%03X", p.start, p.data, p.check);
        lcd_text(10, 300, line);

        int late = qam_send_packet(&p);
        lcd_text(10, 335, "QAM-4 GESENDET");
        snprintf(line, sizeof(line), "SPAETE SAMPLES %-5d", late);
        lcd_text(10, 360, line);

        printf("Temp: %.4f C  Paket (Gray): %03X/%03X/%03X  QAM-4 gesendet, %d Samples zu spaet\n",
               temp, p.start, p.data, p.check, late);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    lcd_init();
    lcd_clear();
    tmp112_init();
    dac_init();

    // Core 1: die Sample-Ausgabe wartet aktiv und soll moeglichst nicht unterbrochen werden
    xTaskCreatePinnedToCore(temperature_task, "temperature", 4096, NULL, 5, NULL, 1);
}
