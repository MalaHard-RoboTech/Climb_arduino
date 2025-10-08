#include "EspNow.h"
#include <string.h>

static esp_now_peer_info_t peer{};
static EspNowCommandCallback g_cb = nullptr;
static QueueHandle_t g_queue = nullptr;
static volatile uint32_t g_txCount = 0;

// ---- RX callback (IDF 5.x uses esp_now_recv_info) ----
#if ESP_IDF_VERSION_MAJOR >= 5
static void onRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (!g_queue || !data || len <= 0) return;

  // info->src_addr has sender MAC (6 bytes)
  // You can also read info->des_addr, info->rx_ctrl.rssi, etc. if needed.

  String s;
  s.reserve(len + 1);
  for (int i = 0; i < len; ++i) s += char(data[i]);
  s.trim();
  xQueueSend(g_queue, &s, 0);
}
#else
// ---- Legacy (IDF 4.x) signature ----
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  if (!g_queue || !data || len <= 0) return;
  String s;
  s.reserve(len + 1);
  for (int i = 0; i < len; ++i) s += char(data[i]);
  s.trim();
  xQueueSend(g_queue, &s, 0);
}
#endif

// ---- TX callback (IDF 5.x uses wifi_tx_info_t) ----
#if ESP_IDF_VERSION_MAJOR >= 5
static void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  (void)tx_info; // tx_info->dest_mac, retry_count, etc.
  if (status == ESP_NOW_SEND_SUCCESS) g_txCount++;
}
#else
static void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
  if (status == ESP_NOW_SEND_SUCCESS) g_txCount++;
}
#endif

bool EspNow_init(const uint8_t peer_mac[6]) {
  WiFi.mode(WIFI_STA);  // required
  if (esp_now_init() != ESP_OK) return false;

  // Register callbacks with the correct signatures
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  // Add peer
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, peer_mac, 6);
  peer.channel = 0;              // use current WiFi channel
  peer.ifidx   = WIFI_IF_STA;    // explicit for clarity
  peer.encrypt = false;          // set true if you later set an LMK
  if (esp_now_add_peer(&peer) != ESP_OK) return false;

  // RX command queue
  g_queue = xQueueCreate(8, sizeof(String));
  return (g_queue != nullptr);
}

bool EspNow_send(const String& line) {
  if (!line.length()) return true;
  return esp_now_send(peer.peer_addr,
                      reinterpret_cast<const uint8_t*>(line.c_str()),
                      line.length()) == ESP_OK;
}

void EspNow_loop() {
  if (!g_queue || !g_cb) return;
  String s;
  while (xQueueReceive(g_queue, &s, 0) == pdTRUE) {
    g_cb(s);
  }
}

void EspNow_setCommandCallback(EspNowCommandCallback cb) { g_cb = cb; }
uint32_t EspNow_txCount() { return g_txCount; }
