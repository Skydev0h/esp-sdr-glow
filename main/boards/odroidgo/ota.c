/* Wireless updates. Hold MENU while switching the GO on: instead of the
 * receiver it joins the Wi-Fi stored with OGWIFI/OGWIPW, shows its address and
 * takes an app image over HTTP into the other OTA slot, then restarts into it:
 *   curl --data-binary @build-odroid-go/esp_sdr.bin http://<ip>/ota
 * Every second it also broadcasts "esp-sdr-glow <ip> <version> <slot> <build
 * date and time>" to UDP 3232 so a script can find it. B leaves without
 * updating. Anyone on the LAN can push an image while this screen is up. */
#include "odroidgo.h"
#include "driver/gpio.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "burst_serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W OG_LCD_W
#define H OG_LCD_H
#define PIN_MENU 13
#define BEACON_PORT 3232
#define CHUNK 4096

static char ssid[33], pass[65], state[48], result[48];
static esp_ip4_addr_t ip;
static volatile unsigned got, total;
static volatile bool connected, receiving, done, dirty = true;

static bool load(const char *key, char *out, size_t size) {
    nvs_handle_t h;
    out[0] = 0;
    if (nvs_open("ota", NVS_READONLY, &h) != ESP_OK) return false;
    bool ok = nvs_get_str(h, key, out, &size) == ESP_OK;
    nvs_close(h);
    return ok && out[0];
}

static void say(char *line, const char *text) {
    snprintf(line, 48, "%s", text);
    dirty = true;
}

static void draw(void) {
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *run = esp_ota_get_running_partition();
    char t[8][72];
    snprintf(t[0], sizeof(t[0]), "%s %s", app->project_name, app->version);
    snprintf(t[1], sizeof(t[1]), "running from %s", run ? run->label : "?");
    snprintf(t[2], sizeof(t[2]), "Wi-Fi: %s", ssid[0] ? ssid : "not set (OGWIFI/OGWIPW over USB)");
    snprintf(t[3], sizeof(t[3]), "%s", state);
    if (connected) {
        snprintf(t[4], sizeof(t[4]), "curl --data-binary @esp_sdr.bin \\");
        snprintf(t[5], sizeof(t[5]), "  http://" IPSTR "/ota", IP2STR(&ip));
    } else t[4][0] = t[5][0] = 0;
    if (receiving || done || result[0])
        snprintf(t[6], sizeof(t[6]), "%s", result[0] ? result : "receiving...");
    else t[6][0] = 0;
    snprintf(t[7], sizeof(t[7]), "B: leave");
    const uint16_t bg = sv_rgb(0, 0, 0), dim = sv_rgb(110, 110, 110), fg = sv_rgb(220, 220, 220);
    const int bar_y = 176, bar_w = total ? (int)((uint64_t)got * (W - 28) / total) : 0;
    for (int y = 0; y < H; y += OG_STRIP_H) {
        uint16_t *s = og_lcd_strip();
        sv_fill(s, W, OG_STRIP_H, bg);
        sv_text(s, W, OG_STRIP_H, 14, 12 - y, "OTA UPDATE", sv_rgb(0, 200, 230), 2);
        for (int i = 0; i < 8; i++)
            sv_text(s, W, OG_STRIP_H, 14, (i < 6 ? 44 + 14 * i : 150 + 50 * (i - 6)) - y, t[i],
                    i == 3 || i == 6 ? sv_rgb(255, 220, 0) : i == 7 ? dim : fg, 1);
        for (int r = 0; r < OG_STRIP_H; r++) {
            int yy = y + r;
            if (yy < bar_y || yy >= bar_y + 8) continue;
            for (int x = 14; x < W - 14; x++)
                s[r * W + x] = x - 14 < bar_w ? sv_rgb(40, 220, 40) : sv_rgb(40, 40, 40);
        }
        og_lcd_blit(0, y, W, OG_STRIP_H, s);
    }
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        connected = false;
        char t[48];
        snprintf(t, sizeof(t), "connecting... (reason %d)", ((wifi_event_sta_disconnected_t *)data)->reason);
        say(state, t);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip = ((ip_event_got_ip_t *)data)->ip_info.ip;
        connected = true;
        char t[48];
        snprintf(t, sizeof(t), "connected, " IPSTR, IP2STR(&ip));
        say(state, t);
    }
}

/* Fills buf with exactly n bytes of the request body. */
static bool recv_all(httpd_req_t *req, char *buf, size_t n) {
    for (size_t off = 0, tries = 0; off < n;) {
        int r = httpd_req_recv(req, buf + off, n - off);
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++tries < 4) continue;
        if (r <= 0) return false;
        off += (size_t)r;
    }
    return true;
}

/* The first chunk must carry this project's app description. */
static bool same_project(const char *buf) {
    const esp_app_desc_t *in = (const void *)(buf + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t));
    return in->magic_word == ESP_APP_DESC_MAGIC_WORD &&
           !strncmp(in->project_name, esp_app_get_description()->project_name, sizeof(in->project_name));
}

static esp_err_t ota_post(httpd_req_t *req) {
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part || receiving || done || req->content_len < CHUNK || req->content_len > part->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need one app image that fits the slot\n");
        return ESP_OK;
    }
    receiving = true;
    result[0] = 0;
    total = req->content_len;
    got = 0;
    dirty = true;
    char *buf = malloc(CHUNK);
    esp_ota_handle_t h = 0;
    esp_err_t e = buf ? esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &h) : ESP_ERR_NO_MEM;
    while (e == ESP_OK && got < total) {
        size_t n = total - got < CHUNK ? total - got : CHUNK;
        if (!recv_all(req, buf, n)) e = ESP_ERR_TIMEOUT;
        else if (got == 0 && !same_project(buf)) e = ESP_ERR_INVALID_VERSION;
        else e = esp_ota_write(h, buf, n);
        if (e == ESP_OK) got += n;
    }
    free(buf);
    if (e == ESP_OK) e = esp_ota_end(h); /* verifies the image */
    else if (h) esp_ota_abort(h);
    if (e == ESP_OK) e = esp_ota_set_boot_partition(part);
    char t[48];
    if (e == ESP_OK) snprintf(t, sizeof(t), "%u bytes into %s, restarting", total, part->label);
    else snprintf(t, sizeof(t), "FAILED: %s", esp_err_to_name(e));
    say(result, t);
    strcat(t, "\n");
    if (e == ESP_OK) httpd_resp_sendstr(req, t);
    else httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, t);
    done = e == ESP_OK;
    receiving = false;
    return ESP_OK;
}

static esp_err_t status_get(httpd_req_t *req) {
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *run = esp_ota_get_running_partition();
    char t[128];
    snprintf(t, sizeof(t), "%s %s, running from %s; POST the app image to /ota\n", app->project_name,
             app->version, run ? run->label : "?");
    return httpd_resp_sendstr(req, t);
}

static void run(void) {
    og_lcd_init();
    og_io_init();
    og_lcd_backlight(80);
    load("ssid", ssid, sizeof(ssid));
    load("pass", pass, sizeof(pass));
    say(state, ssid[0] ? "connecting..." : "nothing to connect to");
    draw();
    dirty = false;
    og_ota_confirm(); /* an image that gets this far can take the next one */

    if (ssid[0]) {
        ESP_ERROR_CHECK(esp_netif_init());
        esp_netif_t *netif = esp_netif_create_default_wifi_sta();
        esp_netif_set_hostname(netif, "esp-sdr-glow");
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        wifi_config_t wc = {0};
        memcpy(wc.sta.ssid, ssid, strlen(ssid));
        memcpy(wc.sta.password, pass, strlen(pass));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
        ESP_ERROR_CHECK(esp_wifi_start());
        esp_wifi_set_ps(WIFI_PS_NONE);

        httpd_handle_t server = NULL;
        httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
        hc.recv_wait_timeout = 10;
        hc.stack_size = 8192; /* image verification at the end */
        ESP_ERROR_CHECK(httpd_start(&server, &hc));
        const httpd_uri_t post = {.uri = "/ota", .method = HTTP_POST, .handler = ota_post};
        const httpd_uri_t get = {.uri = "/", .method = HTTP_GET, .handler = status_get};
        httpd_register_uri_handler(server, &post);
        httpd_register_uri_handler(server, &get);
    }

    int sock = -1;
    int64_t beacon_at = 0, drawn_at = 0;
    unsigned drawn_got = 0;
    for (;;) {
        int64_t now = esp_timer_get_time();
        if (connected && now >= beacon_at) {
            if (sock < 0) {
                sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                int on = 1;
                setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
            }
            const esp_app_desc_t *app = esp_app_get_description();
            const esp_partition_t *slot = esp_ota_get_running_partition();
            char msg[128];
            int n = snprintf(msg, sizeof(msg), "esp-sdr-glow " IPSTR " %s %s %s %s\n", IP2STR(&ip), app->version,
                             slot ? slot->label : "?", app->date, app->time);
            struct sockaddr_in to = {.sin_family = AF_INET, .sin_port = htons(BEACON_PORT),
                                     .sin_addr.s_addr = htonl(INADDR_BROADCAST)};
            sendto(sock, msg, n, 0, (struct sockaddr *)&to, sizeof(to));
            beacon_at = now + 1000000;
        }
        if (dirty || (got != drawn_got && now - drawn_at > 250000)) {
            dirty = false;
            drawn_got = got;
            drawn_at = now;
            draw();
        }
        if (done) {
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }
        if (!receiving && (og_buttons() & OG_B)) esp_restart();
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void board_boot(void) {
    const gpio_config_t in = {.pin_bit_mask = 1ULL << PIN_MENU, .mode = GPIO_MODE_INPUT,
                              .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&in);
    esp_rom_delay_us(1000);
    if (gpio_get_level(PIN_MENU) == 0) run();
}

/* A started image is good once the UI runs; until then a reset rolls back. */
void og_ota_confirm(void) {
    esp_ota_mark_app_valid_cancel_rollback();
}

/* OGWIFI <ssid> / OGWIPW <password> store the network for OTA mode (empty
 * clears it); OGWIFI? tells what is stored, never the password. */
bool og_ota_command(const char *line) {
    char buf[96];
    const char *key = !strncmp(line, "OGWIFI ", 7) || !strcmp(line, "OGWIFI") ? "ssid"
                      : !strncmp(line, "OGWIPW ", 7) || !strcmp(line, "OGWIPW") ? "pass" : NULL;
    if (!strcmp(line, "OGWIFI?")) {
        char s[33], p[65];
        load("ssid", s, sizeof(s));
        int n = snprintf(buf, sizeof(buf), "OGWIFI ssid \"%s\" password %s\n", s,
                         load("pass", p, sizeof(p)) ? "set" : "none");
        burst_serial_send(buf, n);
        return true;
    }
    if (!key) return false;
    const char *value = line[6] ? line + 7 : "";
    nvs_handle_t h;
    bool ok = strlen(value) < (key[0] == 's' ? 33 : 65) && nvs_open("ota", NVS_READWRITE, &h) == ESP_OK;
    if (ok) {
        ok = nvs_set_str(h, key, value) == ESP_OK && nvs_commit(h) == ESP_OK;
        nvs_close(h);
    }
    burst_serial_send(ok ? "OK\n" : "ERR\n", ok ? 3 : 4);
    return true;
}
