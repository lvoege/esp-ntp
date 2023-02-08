set(COMPONENT_SRCS "esp-ntp.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "")

set(COMPONENTS bootloader driver esp32 esp_adc_cal
	espcoredump esp_event esp_ringbuf esptool_py heap
	idf_test freertos jsmn log micro-ecc newlib nvs_flash
	pthread soc spiffs spi_flash ulp unity
	app_trace ethernet bootloader_support
	vfs app_update esp-ntp partition_table efuse
	bootloader_support esp_common esp_rom esp_timer
	esp_system cnip)

register_component()
