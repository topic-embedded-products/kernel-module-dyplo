KERNEL_SRC ?= "/lib/modules/$(shell uname -r)/build"

# Only build the PCIe module when PCI_MSI is defined
OPTIONALMODULE-$(CONFIG_PCI_MSI) = dyplo-pcie.o

obj-m += dyplo.o $(OPTIONALMODULE-y) $(OPTIONALMODULE-m)

dyplo-y := dyplo-core.o
# Only add the devicetree/platform binding when OpenFirmware is defined
dyplo-$(CONFIG_OF) += dyplo-of.o

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

