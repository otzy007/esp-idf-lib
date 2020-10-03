#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bmp280.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"

#include "tftspi.h"
#include "tft.h"
// #if defined(CONFIG_IDF_TARGET_ESP8266)
// #define SDA_GPIO 4
// #define SCL_GPIO 5
// #else
#define SDA_GPIO 15
#define SCL_GPIO 17
// #endif
#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
char payload[150];

#define SPI_BUS TFT_HSPI_HOST

static char tmp_buff[64];

//---------------------
static void _dispTime()
{
	Font curr_font = tft_cfont;
    // if (tft_width < 240) TFT_setFont(DEF_SMALL_FONT, NULL);
	// else TFT_setFont(DEFAULT_FONT, NULL);
	TFT_setFont(DEFAULT_FONT, NULL);
	sprintf(tmp_buff, "TESTEZ SI EU");
	TFT_print(tmp_buff, CENTER, tft_height-TFT_getfontheight()-5);

    tft_cfont = curr_font;
}
//---------------------------------
static void disp_header(char *info)
{
	TFT_fillScreen(TFT_BLACK);
	TFT_resetclipwin();

	tft_fg = TFT_YELLOW;
	tft_bg = (color_t){ 64, 64, 64 };

    // if (tft_width < 240) TFT_setFont(DEF_SMALL_FONT, NULL);
	// else TFT_setFont(DEFAULT_FONT, NULL);
	TFT_setFont(UBUNTU16_FONT, NULL);
	TFT_fillRect(0, 0, tft_width-1, TFT_getfontheight()+8, tft_bg);
	TFT_drawRect(0, 0, tft_width-1, TFT_getfontheight()+8, TFT_CYAN);

	TFT_fillRect(0, tft_height-TFT_getfontheight()-9, tft_width-1, TFT_getfontheight()+8, tft_bg);
	TFT_drawRect(0, tft_height-TFT_getfontheight()-9, tft_width-1, TFT_getfontheight()+8, TFT_CYAN);

	TFT_print(info, CENTER, 4);
	_dispTime();

	tft_bg = TFT_BLACK;
	TFT_setclipwin(0,TFT_getfontheight()+9, tft_width-1, tft_height-TFT_getfontheight()-10);
}

struct SensorData {
  float temperature;
  float humidity;
  float pressure;
} sensor_data;

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    printf("!!!!!pvParamters temp: %.2f\n", sensor_data.temperature);
#if defined(CONFIG_EXAMPLE_IPV4)
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
    struct sockaddr_in6 dest_addr = { 0 };
    inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(PORT);
    dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
    struct sockaddr_in6 dest_addr = { 0 };
    ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

    while (1) {
        sprintf(payload, "bmp280,bmp=1 temperature=%.2f,pressure=%.2f,humidity=%.2f", sensor_data.temperature, sensor_data.pressure, sensor_data.humidity);
        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Message sent");

        struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }
        // Data received
        else {
            rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
            ESP_LOGI(TAG, "%s", rx_buffer);
            if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                ESP_LOGI(TAG, "Received expected message, reconnecting");
                break;
            }
        }
        break; // just go out
        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }

    if (sock != -1) {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
  vTaskDelete(NULL);
}

void bmp280_test(void *pvParamters)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    float pressure, temperature, humidity;

    while (1)
    {
        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        if (bme280p) {
            printf(", Humidity: %.2f\n", humidity);
            sensor_data.pressure = pressure;
            sensor_data.temperature = temperature;
            sensor_data.humidity = humidity;
            xTaskCreate(udp_client_task, "udp_client", 4096, &sensor_data, 5, NULL);

            TFT_print("           ", CENTER, CENTER-TFT_getfontheight() - 15);
            sprintf(tmp_buff, "P: %.2f Pa", pressure);
            TFT_print(tmp_buff, CENTER, CENTER-TFT_getfontheight() - 15);

            TFT_print("           ", CENTER, CENTER);
            sprintf(tmp_buff, "T: %.2f C\nH: %.2f", temperature, humidity);
            TFT_print(tmp_buff, CENTER, CENTER);

            // TFT_print("           ", CENTER, CENTER + 10);
            // sprintf(tmp_buff, "H: %.2f", humidity);
            // TFT_print(tmp_buff, CENTER, CENTER + 10);
        }
        else
            printf("\n");

        vTaskDelay(50000 / portTICK_PERIOD_MS);

    }
}

void app_main()
{
    esp_err_t ret;

    // === SET GLOBAL VARIABLES ==========================

    // ===================================================
    // ==== Set maximum spi clock for display read    ====
    //      operations, function 'find_rd_speed()'    ====
    //      can be used after display initialization  ====
    tft_max_rdclock = 8000000;
    // ===================================================

     // ====================================================================
     // === Pins MUST be initialized before SPI interface initialization ===
     // ====================================================================
     TFT_PinsInit();

     // ====  CONFIGURE SPI DEVICES(s)  ====================================================================================

     spi_lobo_device_handle_t spi;

     spi_lobo_bus_config_t buscfg={
         .miso_io_num=PIN_NUM_MISO,				// set SPI MISO pin
         .mosi_io_num=PIN_NUM_MOSI,				// set SPI MOSI pin
         .sclk_io_num=PIN_NUM_CLK,				// set SPI CLK pin
         .quadwp_io_num=-1,
         .quadhd_io_num=-1,
     .max_transfer_sz = 6*1024,
     };
     spi_lobo_device_interface_config_t devcfg={
         .clock_speed_hz=8000000,                // Initial clock out at 8 MHz
         .mode=0,                                // SPI mode 0
         .spics_io_num=-1,                       // we will use external CS pin
     .spics_ext_io_num=PIN_NUM_CS,           // external CS pin
     .flags=LB_SPI_DEVICE_HALFDUPLEX,        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
     };

     printf("==============================\r\n\r\n");

    // ==================================================================
    // ==== Initialize the SPI bus and attach the LCD to the SPI bus ====

    ret=spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
      assert(ret==ESP_OK);
    printf("SPI: display device added to spi bus (%d)\r\n", SPI_BUS);
    tft_disp_spi = spi;

    // ==== Test select/deselect ====
    ret = spi_lobo_device_select(spi, 1);
      assert(ret==ESP_OK);
    ret = spi_lobo_device_deselect(spi);
      assert(ret==ESP_OK);

    printf("SPI: attached display device, speed=%u\r\n", spi_lobo_get_speed(spi));
    printf("SPI: bus uses native pins: %s\r\n", spi_lobo_uses_native_pins(spi) ? "true" : "false");

    printf("SPI: display init...\r\n");
    TFT_display_init();

    tft_max_rdclock = find_rd_speed();
    printf("SPI: Max rd speed = %u\r\n", tft_max_rdclock);

      // ==== Set SPI clock used for display operations ====
    spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
    printf("SPI: Changed speed to %u\r\n", spi_lobo_get_speed(spi));

      printf("\r\n---------------------\r\n");
    printf("Graphics demo started\r\n");
    printf("---------------------\r\n");


    tft_font_rotate = 0;
    tft_text_wrap = 0;
    tft_font_transparent = 0;
    tft_font_forceFixed = 0;
    tft_gray_scale = 0;
    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
    TFT_setRotation(LANDSCAPE);
    TFT_setFont(TOONEY32_FONT, NULL);
    TFT_resetclipwin();

    disp_header("Temp monitor");
      tft_fg = TFT_CYAN;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(bmp280_test, "bmp280_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
