deps_config := \
	/home/hellboy/esp/esp-idf/components/app_trace/Kconfig \
	/home/hellboy/esp/esp-idf/components/aws_iot/Kconfig \
	/home/hellboy/esp/esp-idf/components/bt/Kconfig \
	/home/hellboy/esp/esp-idf/components/esp32/Kconfig \
	/home/hellboy/esp/esp-idf/components/ethernet/Kconfig \
	/home/hellboy/esp/esp-idf/components/fatfs/Kconfig \
	/home/hellboy/esp/esp-idf/components/freertos/Kconfig \
	/home/hellboy/esp/esp-idf/components/heap/Kconfig \
	/home/hellboy/esp/esp-idf/components/libsodium/Kconfig \
	/home/hellboy/esp/esp-idf/components/log/Kconfig \
	/home/hellboy/esp/esp-idf/components/lwip/Kconfig \
	/home/hellboy/esp/esp-idf/components/mbedtls/Kconfig \
	/home/hellboy/esp/esp-idf/components/openssl/Kconfig \
	/home/hellboy/esp/esp-idf/components/pthread/Kconfig \
	/home/hellboy/esp/esp-idf/components/spi_flash/Kconfig \
	/home/hellboy/esp/esp-idf/components/spiffs/Kconfig \
	/home/hellboy/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/hellboy/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/hellboy/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/hellboy/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/hellboy/esp/AccesPoint/main/Kconfig.projbuild \
	/home/hellboy/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/hellboy/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
