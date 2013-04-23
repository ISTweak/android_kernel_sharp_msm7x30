#
# Makefile for the Linux kernel device drivers.
#
# 15 Sep 2000, Christoph Hellwig <hch@infradead.org>
# Rewritten to use lists instead of if-statements.
#

obj-y				+= gpio/
obj-$(CONFIG_PCI)		+= pci/
obj-$(CONFIG_PARISC)		+= parisc/
obj-$(CONFIG_RAPIDIO)		+= rapidio/
obj-y				+= video/
obj-y				+= idle/
obj-$(CONFIG_ACPI)		+= acpi/
obj-$(CONFIG_SFI)		+= sfi/
# PnP must come after ACPI since it will eventually need to check if acpi
# was used and do nothing if so
obj-$(CONFIG_PNP)		+= pnp/
obj-$(CONFIG_ARM_AMBA)		+= amba/

obj-$(CONFIG_VIRTIO)		+= virtio/
obj-$(CONFIG_XEN)		+= xen/

# regulators early, since some subsystems rely on them to initialize
obj-$(CONFIG_REGULATOR)		+= regulator/

# char/ comes before serial/ etc so that the VT console is the boot-time
# default.
obj-y				+= char/

# gpu/ comes after char for AGP vs DRM startup
obj-y				+= gpu/

obj-$(CONFIG_CONNECTOR)		+= connector/

# i810fb and intelfb depend on char/agp/
obj-$(CONFIG_FB_I810)           += video/i810/
obj-$(CONFIG_FB_INTEL)          += video/intelfb/

obj-y				+= serial/
obj-$(CONFIG_PARPORT)		+= parport/
obj-y				+= base/ block/ misc/ mfd/
obj-$(CONFIG_NUBUS)		+= nubus/
obj-y				+= macintosh/
obj-$(CONFIG_IDE)		+= ide/
obj-$(CONFIG_SCSI)		+= scsi/
obj-$(CONFIG_ATA)		+= ata/
obj-$(CONFIG_MTD)		+= mtd/
obj-$(CONFIG_SPI)		+= spi/
obj-y				+= net/
obj-$(CONFIG_ATM)		+= atm/
obj-$(CONFIG_FUSION)		+= message/
obj-$(CONFIG_FIREWIRE)		+= firewire/
obj-y				+= ieee1394/
obj-$(CONFIG_UIO)		+= uio/
obj-y				+= cdrom/
obj-y				+= auxdisplay/
obj-$(CONFIG_PCCARD)		+= pcmcia/
obj-$(CONFIG_DIO)		+= dio/
obj-$(CONFIG_SBUS)		+= sbus/
obj-$(CONFIG_ZORRO)		+= zorro/
obj-$(CONFIG_MAC)		+= macintosh/
obj-$(CONFIG_ATA_OVER_ETH)	+= block/aoe/
obj-$(CONFIG_PARIDE) 		+= block/paride/
obj-$(CONFIG_TC)		+= tc/
obj-$(CONFIG_UWB)		+= uwb/
obj-$(CONFIG_USB_SWIC) 			+= sharp/shswic/
obj-$(CONFIG_USB_OTG_UTILS)	+= usb/otg/
obj-$(CONFIG_USB)		+= usb/
obj-$(CONFIG_USB_MUSB_HDRC)	+= usb/musb/
obj-$(CONFIG_PCI)		+= usb/
obj-$(CONFIG_USB_GADGET)	+= usb/gadget/
obj-$(CONFIG_USB_FUNCTION)	+= usb/function/
obj-$(CONFIG_DIAG_CHAR)         += char/diag/
obj-$(CONFIG_SERIO)		+= input/serio/
obj-$(CONFIG_GAMEPORT)		+= input/gameport/
obj-$(CONFIG_INPUT)		+= input/
obj-$(CONFIG_I2O)		+= message/
obj-$(CONFIG_RTC_LIB)		+= rtc/
obj-y				+= i2c/ media/
obj-$(CONFIG_PPS)		+= pps/
obj-$(CONFIG_W1)		+= w1/
obj-$(CONFIG_POWER_SUPPLY)	+= power/
obj-$(CONFIG_HWMON)		+= hwmon/
obj-$(CONFIG_THERMAL)		+= thermal/
obj-$(CONFIG_WATCHDOG)		+= watchdog/
obj-$(CONFIG_PHONE)		+= telephony/
obj-$(CONFIG_MD)		+= md/
obj-$(CONFIG_BT)		+= bluetooth/
obj-$(CONFIG_ACCESSIBILITY)	+= accessibility/
obj-$(CONFIG_ISDN)		+= isdn/
obj-$(CONFIG_EDAC)		+= edac/
obj-$(CONFIG_MCA)		+= mca/
obj-$(CONFIG_EISA)		+= eisa/
obj-y				+= lguest/
obj-$(CONFIG_CPU_FREQ)		+= cpufreq/
obj-$(CONFIG_CPU_IDLE)		+= cpuidle/
obj-$(CONFIG_MMC)		+= mmc/
obj-$(CONFIG_MEMSTICK)		+= memstick/
obj-$(CONFIG_NEW_LEDS)		+= leds/
obj-$(CONFIG_SWITCH)		+= switch/
obj-$(CONFIG_INFINIBAND)	+= infiniband/
obj-$(CONFIG_SGI_SN)		+= sn/
obj-y				+= firmware/
obj-$(CONFIG_CRYPTO)		+= crypto/
obj-$(CONFIG_SUPERH)		+= sh/
obj-$(CONFIG_ARCH_SHMOBILE)	+= sh/
obj-$(CONFIG_GENERIC_TIME)	+= clocksource/
obj-$(CONFIG_DMA_ENGINE)	+= dma/
obj-$(CONFIG_DCA)		+= dca/
obj-$(CONFIG_HID)		+= hid/
obj-$(CONFIG_PPC_PS3)		+= ps3/
obj-$(CONFIG_OF)		+= of/
obj-$(CONFIG_SSB)		+= ssb/
obj-$(CONFIG_VHOST_NET)		+= vhost/
obj-$(CONFIG_VLYNQ)		+= vlynq/
obj-$(CONFIG_STAGING)		+= staging/
obj-y				+= platform/
obj-y				+= ieee802154/
obj-$(CONFIG_SHLCDC_BOARD)		+= sharp/shlcdc/
obj-$(CONFIG_SHLCDC_BOARD)		+= sharp/shdisp/
obj-$(CONFIG_SHSYS_CUST)		+= sharp/shsys/
obj-$(CONFIG_SH_AUDIO_DRIVER)		+= sharp/shspamp/
obj-$(CONFIG_SH_AUDIO_DRIVER)		+= sharp/shvibrator/
obj-$(CONFIG_SHFELICA)			+= sharp/mfc/
obj-$(CONFIG_TOUCHSCREEN_SHTPS) += sharp/shtps/
obj-$(CONFIG_SHTERM)			+= sharp/shterm/
# obj-$(CONFIG_SHRLOG)			+= sharp/shrlog/
obj-$(CONFIG_SHTIMER)			+= sharp/shtimer/
# obj-$(CONFIG_SH_CAMERA)			+= sharp/shcamsensor/
obj-$(CONFIG_MSM_CAMERA)		+= sharp/shcamsensor/tools/
obj-$(CONFIG_SH_CAMLED)			+= sharp/shcamled/
obj-$(CONFIG_SHSD_DRIVER)		+= sharp/shsd/
obj-$(CONFIG_SHSD_DRIVER)		+= sharp/euryale/
obj-$(CONFIG_SHSD_DRIVER)		+= sharp/stheno/
obj-$(CONFIG_BTPM)				+= sharp/btpm/
obj-$(CONFIG_SENSORS_AMI602)	+= sharp/shmds/ami602/
obj-$(CONFIG_SENSORS_AMI603)	+= sharp/shmds/ami603/ami_sensor/
obj-$(CONFIG_SENSORS_AMI603_FLIP)	+= sharp/shmds/ami603/ami_flip/
obj-$(CONFIG_MPU_SENSORS_MPU3050)	+= sharp/shmds/mpu3050/
obj-$(CONFIG_SH_YAS530)			+= sharp/shmds/yas530/
obj-$(CONFIG_BATTERY_SH)		+= sharp/shbattery/
obj-$(CONFIG_SHIRDA)			+= sharp/irda/
obj-$(CONFIG_SHPRESENTER)		+= sharp/presenter/
obj-$(CONFIG_DTV_TUNERDEV)		+= sharp/tunerdrv/
obj-$(CONFIG_SHHDMI_DRIVER)		+= sharp/shhdmi/
obj-$(CONFIG_WIFIPM)			+= sharp/wifi/
obj-$(CONFIG_SHBOOT_CUST)		+= sharp/shboot/
obj-$(CONFIG_SHSMD_FIRM_READ)	+= sharp/smd_firm_read/
obj-$(CONFIG_SHDIAG)			+= sharp/shdiag/
obj-$(CONFIG_SHIRPM)			+= sharp/shirpm/
obj-$(CONFIG_SHIRRC)			+= sharp/shirrc/
obj-$(CONFIG_SHLOG_SYSTEM)		+= sharp/shlog/
obj-$(CONFIG_SHTERM)			+= sharp/shterm/
obj-$(CONFIG_SH_DETECT_HIGH_TEMP) += sharp/shtemp_moni/
obj-$(CONFIG_SENSORS_PROXIMITY)	+= sharp/proximity/
obj-$(CONFIG_LEDS_KEY_LED)		+= sharp/shkeyled/
obj-$(CONFIG_SHSENSORS_LIGHT)		+= sharp/light_inpdev/
obj-$(CONFIG_SHVOLD_KILLPROCESS) += sharp/vold/
obj-$(CONFIG_SHSUPPORT_CUST)	+= sharp/shsupport/
obj-$(CONFIG_SHLOG_SYSTEM)		+= sharp/shlog/
obj-$(CONFIG_SHTERM)			+= sharp/shterm/
obj-$(CONFIG_SH_DETECT_HIGH_TEMP)	+= sharp/shtemp_moni/
obj-$(CONFIG_LEDS_KEYBOARD_LED) += sharp/shkeyboardled/
obj-$(CONFIG_BROADCOM_BTHID_DRIVER) += sharp/bthid/
obj-$(CONFIG_SHRMTS)			+= sharp/shrmts/
