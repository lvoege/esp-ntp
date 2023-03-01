// barebones stratum 1 NTPv3 server for an ESP32-C3 with a W5500-lite ethernet adapter and a ublox neo-6 GPS unit with PPS.
#include <algorithm>
#include <cctype>
#include <cstring>
#include <ctime>
#include <random>
#include <ranges>
#include <string_view>

#include <esp_eth_netif_glue.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <lwip/sockets.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include <rom/ets_sys.h>
#include <esp_wifi.h>

using namespace std::literals;

constexpr int NMEA_MAX_SENTENCE_LENGTH = 79;

#if FIRST_PROTOTYPE
constexpr std::string_view hostname { "esp-ntp" };
constexpr gpio_num_t UART_GPIO_TX = gpio_num_t::GPIO_NUM_2;//0;
constexpr gpio_num_t UART_GPIO_RX = gpio_num_t::GPIO_NUM_3;
constexpr gpio_num_t GPIO_PPS = gpio_num_t::GPIO_NUM_1;
constexpr gpio_num_t SPI_GPIO_MISO = gpio_num_t::GPIO_NUM_10;
constexpr gpio_num_t SPI_GPIO_MOSI = gpio_num_t::GPIO_NUM_9;
constexpr gpio_num_t SPI_GPIO_SCLK = gpio_num_t::GPIO_NUM_8;
constexpr gpio_num_t PHY_GPIO_RST = gpio_num_t::GPIO_NUM_NC; // not hooked
constexpr gpio_num_t SPI_GPIO_CS = gpio_num_t::GPIO_NUM_7;
constexpr gpio_num_t W5500_GPIO_INT = gpio_num_t::GPIO_NUM_6;
#elif SECOND_PROTOTYPE
constexpr std::string_view hostname { "esp-ntp2" };
constexpr gpio_num_t UART_GPIO_TX = gpio_num_t::GPIO_NUM_9;//0;
constexpr gpio_num_t UART_GPIO_RX = gpio_num_t::GPIO_NUM_5;
constexpr gpio_num_t GPIO_PPS = gpio_num_t::GPIO_NUM_7;
constexpr gpio_num_t SPI_GPIO_MISO = gpio_num_t::GPIO_NUM_6;
constexpr gpio_num_t SPI_GPIO_MOSI = gpio_num_t::GPIO_NUM_2;
constexpr gpio_num_t SPI_GPIO_SCLK = gpio_num_t::GPIO_NUM_3;
constexpr gpio_num_t PHY_GPIO_RST = gpio_num_t::GPIO_NUM_NC; // not hooked
constexpr gpio_num_t SPI_GPIO_CS = gpio_num_t::GPIO_NUM_0;
constexpr gpio_num_t W5500_GPIO_INT = gpio_num_t::GPIO_NUM_1;
#else
constexpr std::string_view hostname { "esp-ntp3" };
constexpr gpio_num_t UART_GPIO_TX = gpio_num_t::GPIO_NUM_NC;//0;
constexpr gpio_num_t UART_GPIO_RX = gpio_num_t::GPIO_NUM_2;
constexpr gpio_num_t GPIO_PPS = gpio_num_t::GPIO_NUM_3;
constexpr gpio_num_t SPI_GPIO_MISO = gpio_num_t::GPIO_NUM_7;
constexpr gpio_num_t SPI_GPIO_MOSI = gpio_num_t::GPIO_NUM_10;
constexpr gpio_num_t SPI_GPIO_SCLK = gpio_num_t::GPIO_NUM_1;
constexpr gpio_num_t PHY_GPIO_RST = gpio_num_t::GPIO_NUM_NC; // not hooked
constexpr gpio_num_t SPI_GPIO_CS = gpio_num_t::GPIO_NUM_0;
constexpr gpio_num_t W5500_GPIO_INT = gpio_num_t::GPIO_NUM_5;
#endif

const char LOG_TAG[] = "esp-ntp";

static uint32_t gps_time_in_seconds = 0;
static uint64_t us_since_boot_of_last_pps;

static void pps_handler(void *) {
    vPortEnterCritical();
    auto us_since_boot_now = esp_timer_get_time();
    if (gps_time_in_seconds != 0 && us_since_boot_now - us_since_boot_of_last_pps < 1'100'000) {
        ++gps_time_in_seconds;
        us_since_boot_of_last_pps = us_since_boot_now;
    } else gps_time_in_seconds = 0;
    vPortExitCritical();
}

static void setup_pps() {
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_PPS, pps_handler, nullptr));
    gpio_config_t pps_config;
    memset(&pps_config, 0, sizeof(pps_config));
    pps_config.intr_type = gpio_int_type_t::GPIO_INTR_POSEDGE;
    pps_config.mode = gpio_mode_t::GPIO_MODE_INPUT;
    pps_config.pin_bit_mask = BIT64(GPIO_PPS);
    pps_config.pull_up_en = gpio_pullup_t::GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&pps_config));
}

// open up the UART that has the GPS receiver on it. listen to it until the
// first valid $GPRMC sentence. parse the time and date from that, turn it into
// an ntp timestamp, then shut down the UART because that's all we need. from
// then on we use the PPS signal.
static void setup_gps() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUFSIZ * 2, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_GPIO_TX, UART_GPIO_RX, -1, -1));

    std::string sentence_in_progress;
    static_assert(BUFSIZ > NMEA_MAX_SENTENCE_LENGTH);
    char buf[BUFSIZ + 1];
    buf[BUFSIZ] = 0;
    esp_log_write(ESP_LOG_INFO, LOG_TAG, "Waiting for GPS fix\n");
    while (true) {
        auto num_read = uart_read_bytes(UART_NUM_1, buf, BUFSIZ, 20 /  portTICK_PERIOD_MS);
        if (num_read == -1) {
            ESP_LOGE("UART read", "eep");
            continue;
        }

        if (num_read == 0)
            continue;

        sentence_in_progress += std::string(buf, num_read);

        // look for a GPRMC sentence, because that has the three things we need: an
        // indicator if we have a good fix or not, a UTC timestamp and a date.
        for (auto line: sentence_in_progress
                | std::ranges::views::split('$')
                | std::ranges::views::transform([](auto&& str) { return std::string_view(&*str.begin(), std::ranges::distance(str)); })) {

            if (line.ends_with('\n'))
                line = line.substr(0, line.size() - 1);
            if (line.ends_with('\r'))
                line = line.substr(0, line.size() - 1);
            if (line.starts_with("GPRMC") || line.starts_with("GNRMC")) {
                struct tm tm;
                memset(&tm, 0, sizeof(tm));

                auto split = line
                             | std::ranges::views::split(',')
                             | std::ranges::views::transform([](auto&& str) { return std::string_view(&*str.begin(), std::ranges::distance(str)); });
                bool okay = true;
                int wordno = 0;
                for (auto word: split) {
                    auto is_six_digits = [&] {
                        return word.size() >= 6 && isdigit(word[0]) && isdigit(word[1]) && isdigit(word[2]) && isdigit(word[3]) && isdigit(word[4]) && isdigit(word[5]);
                    };
                    switch (wordno++) {
                        case 0: assert(word == "GPRMC" || word == "GNRMC"); break;
                        case 1: {
                            if (is_six_digits()) {
                                tm.tm_hour = (word[0] - '0') * 10 + (word[1] - '0');
                                tm.tm_min = (word[2] - '0') * 10 + (word[3] - '0');
                                tm.tm_sec = (word[4] - '0') * 10 + (word[5] - '0');
                            } else okay = false;
                            break;
                        }
                        case 2:
                            if (!word.empty() && word[0] != 'A')
                                okay = false;
                            break;
                        case 9: {
                            if (is_six_digits()) {
                                tm.tm_mday = (word[0] - '0') * 10 + (word[1] - '0');
                                tm.tm_mon = (word[2] - '0') * 10 + (word[3] - '0') - 1;
                                tm.tm_year = 100 + (word[4] - '0') * 10 + (word[5] - '0');
                            } else okay = false;
                            break;
                        }
                        default: break;
                    }
                    if (!okay)
                        break;
                }

                if (okay && wordno > 9) {
                    auto unix_timestamp = mktime(&tm);
                    esp_log_write(ESP_LOG_INFO, LOG_TAG, "Got unix timestamp of %llu\n", unix_timestamp);
                    auto us_since_boot_now = esp_timer_get_time();
                    vPortEnterCritical();
                    gps_time_in_seconds = (uint32_t)unix_timestamp + 25567U * 24 * 3600; // there were, according to some googling, 22567 days between 1/1/1900 and 1/1/1970.
                    us_since_boot_of_last_pps = us_since_boot_now;
                    vPortExitCritical();
                    esp_log_write(ESP_LOG_INFO, LOG_TAG, "Got GPS timestamp of %lu\n", gps_time_in_seconds);

                    ESP_ERROR_CHECK(uart_driver_delete(UART_NUM_1));
                    return;
                }
            }
        }
        auto pos = sentence_in_progress.rfind('$');
        if (pos != std::string::npos)
            sentence_in_progress = sentence_in_progress.substr(pos);
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(LOG_TAG, "Ethernet Got IP Address");
    ESP_LOGI(LOG_TAG, "~~~~~~~~~~~");
    ESP_LOGI(LOG_TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(LOG_TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(LOG_TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(LOG_TAG, "~~~~~~~~~~~");

}

static void setup_w5500() {
    spi_bus_config_t spi_bus_config;
    memset(&spi_bus_config, 0, sizeof(spi_bus_config));
    spi_bus_config.miso_io_num = SPI_GPIO_MISO;
    spi_bus_config.mosi_io_num = SPI_GPIO_MOSI;
    spi_bus_config.sclk_io_num = SPI_GPIO_SCLK;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.quadwp_io_num = -1;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.reset_gpio_num = PHY_GPIO_RST;
    spi_device_interface_config_t spi_device_config;
    memset(&spi_device_config, 0, sizeof(spi_device_config));
    spi_device_config.mode = 0;
    spi_device_config.clock_speed_hz = 50 * 1000 * 1000;
    spi_device_config.queue_size = 20;
    spi_device_config.spics_io_num = SPI_GPIO_CS;
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &spi_device_config);
    w5500_config.int_gpio_num = W5500_GPIO_INT;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
    esp_eth_handle_t eth_handle = nullptr;
    esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config_spi, &eth_handle));

#if 1
    uint8_t mac_addr[6] = { 0xba, 0x34, 0x88, 0xc9, 0x48, 0x6a };
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));
#else
    {
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_int_distribution<uint8_t> dist(0, 255);

        uint8_t mac_addr[6] = {dist(mt), dist(mt), dist(mt), dist(mt), dist(mt), dist(mt) };
        mac_addr[0] = (mac_addr[0] & 0b11111100) | 0b10; // force locally-administered unicast
        ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));
    }
#endif

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_set_hostname(eth_netif, hostname.data()));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, nullptr));

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

struct ntp_timestamp {
    uint32_t seconds;
    uint32_t fraction;
};

struct ntp_short {
    uint16_t seconds;
    uint16_t fraction;
};

struct ntp_packet {
    uint8_t li_vn_mode;
    uint8_t stratum;
    uint8_t poll;
    uint8_t precision;
    ntp_short root_delay;
    ntp_short root_dispersion;
    char reference_id[4];
    ntp_timestamp reference;
    ntp_timestamp origin;
    ntp_timestamp receive;
    ntp_timestamp transmit;
} __attribute__((packed));

static ntp_timestamp time_at_boot;

static bool stamp(ntp_timestamp *ts) {
    vPortEnterCritical();
    ts->seconds = gps_time_in_seconds;
    auto usec = us_since_boot_of_last_pps;
    vPortExitCritical();
    if (ts->seconds == 0)
        return false;
    usec = esp_timer_get_time() - usec;
    if (usec > 1'000'000) {
        ts->seconds++;
        usec -= 1'000'000;
    }
    if (usec > 1'000'000) {
        vPortEnterCritical();
        gps_time_in_seconds = 0;
        vPortExitCritical();
        return false;
    }
    ts->fraction = uint32_t((uint64_t(usec) * (1LL << 32)) / 1'000'000);
    ts->seconds = htonl(ts->seconds);
    ts->fraction = htonl(ts->fraction);
    return true;
}

static void ntp_handle(ntp_packet &packet) {
    if (!stamp(&packet.receive)) {
        // it's been too long since the last PPS. resync to GPS.
        ESP_LOGW(LOG_TAG, "Lost PPS signal\n");
        setup_gps();
        ESP_LOGI(LOG_TAG, "Got GPS signal\n");
    }

    packet.origin = packet.transmit;
    packet.li_vn_mode = 0b00011100;
    packet.stratum = 1;
    packet.precision = 0xe9;
    packet.root_delay.seconds = packet.root_delay.fraction = 0;
    packet.root_dispersion.seconds = packet.root_dispersion.fraction = 0;
    packet.reference_id[0] = 'P';
    packet.reference_id[1] = 'P';
    packet.reference_id[2] = 'S';
    packet.reference_id[3] = 0;
    packet.reference = time_at_boot;

    packet.transmit = packet.receive;
}

extern "C" {

void app_main(void) {
    esp_log_write(ESP_LOG_INFO, LOG_TAG, "BOOT\n");
    esp_wifi_stop();
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    setup_w5500();
    esp_log_write(ESP_LOG_INFO, LOG_TAG, "Finished w5500 setup\n");
    setup_pps();
    if (gps_time_in_seconds == 0)
        esp_log_write(ESP_LOG_INFO, LOG_TAG, "PPS is ticking\n");
    else {
        do {
            esp_log_write(ESP_LOG_INFO, LOG_TAG, "Waiting for GPS fix\n");
            sleep(1);
        } while (gps_time_in_seconds == 0);
    }

    esp_log_write(ESP_LOG_INFO, LOG_TAG, "Finished PPS setup\n");
    setup_gps();
    esp_log_write(ESP_LOG_INFO, LOG_TAG, "Finished GPS setup\n");
    stamp(&time_at_boot);
    sockaddr_in saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(123);
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        ESP_LOGE(LOG_TAG, "Can't create socket: %s\n", strerror(errno));
        return;
    }

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    int status = setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    if (status < 0) {
        ESP_LOGE(LOG_TAG, "Can't set timeout on socket: %s\n", strerror(errno));
        return;
    }
    status = bind(sockfd, (struct sockaddr *)&saddr, sizeof(saddr));
    if (status < 0) {
        ESP_LOGE(LOG_TAG, "Can't bind socket: %s\n", strerror(errno));
        return;
    }

    uint8_t packet[128];
    esp_log_write(ESP_LOG_INFO, LOG_TAG, "Starting main loop\n");
    sockaddr_in from;
    while (true) {
        memset(&from, 0, sizeof(from));
        socklen_t fromlen = sizeof(from);
        auto len = recvfrom(sockfd, packet, sizeof(packet), 0, (sockaddr *)&from, &fromlen);
        ntp_handle(*(ntp_packet *)packet);
        if (len >= (ssize_t)sizeof(ntp_packet)) {
            auto sent = sendto(sockfd, packet, sizeof(ntp_packet), 0, (const sockaddr *)&from, fromlen);
            if (sent == -1) {
                ESP_LOGE(LOG_TAG, "Error sending reply: %s\n", strerror(errno));
            }
            esp_log_write(ESP_LOG_INFO, LOG_TAG, "Replied to %s\n", inet_ntoa(from.sin_addr));
        }
    }
}

} // extern "C"
