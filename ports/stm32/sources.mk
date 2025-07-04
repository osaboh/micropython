SRC_C = \
	main.c \
	system_stm32.c \
	stm32_it.c \
	usbd_conf.c \
	usbd_desc.c \
	usbd_cdc_interface.c \
	usbd_hid_interface.c \
	usbd_msc_storage.c \
	mphalport.c \
	mpthreadport.c \
	irq.c \
	pendsv.c \
	systick.c  \
	pybthread.c \
	timer.c \
	led.c \
	pin.c \
	pin_defs_stm32.c \
	pin_named_pins.c \
	bufhelper.c \
	dma.c \
	i2c.c \
	pyb_i2c.c \
	spi.c \
	qspi.c \
	uart.c \
	can.c \
	usb.c \
	wdt.c \
	gccollect.c \
	help.c \
	machine_i2c.c \
	modmachine.c \
	modpyb.c \
	modstm.c \
	moduos.c \
	modutime.c \
	modusocket.c \
	modnetwork.c \
	extint.c \
	usrsw.c \
	rng.c \
	rtc.c \
	flash.c \
	flashbdev.c \
	spibdev.c \
	storage.c \
\
\
	lcd.c \
	accel.c \
	servo.c \
	dac.c \
	adc.c \
	$(wildcard boards/$(BOARD)/*.c)


	# sdcard.c
	# fatfs_port.c
