MODULE_NAME = wimaxuart
KDIR = /lib/modules/$(CURRENT)/build

obj-m	:= $(MODULE_NAME).o
all: 
	@echo "making $(MODULE_NAME)"
	@echo "PWD=$(PWD)"
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
