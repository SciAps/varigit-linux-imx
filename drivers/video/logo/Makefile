# SPDX-License-Identifier: GPL-2.0
# Makefile for the Linux logos

obj-$(CONFIG_LOGO)			+= logo.o
obj-$(CONFIG_LOGO_LINUX_MONO)		+= logo_linux_mono.o
obj-$(CONFIG_LOGO_LINUX_VGA16)		+= logo_linux_vga16.o
obj-$(CONFIG_LOGO_LINUX_CLUT224)	+= logo_linux_clut224.o
obj-$(CONFIG_LOGO_DEC_CLUT224)		+= logo_dec_clut224.o
obj-$(CONFIG_LOGO_MAC_CLUT224)		+= logo_mac_clut224.o
obj-$(CONFIG_LOGO_PARISC_CLUT224)	+= logo_parisc_clut224.o
obj-$(CONFIG_LOGO_SGI_CLUT224)		+= logo_sgi_clut224.o
obj-$(CONFIG_LOGO_SUN_CLUT224)		+= logo_sun_clut224.o
obj-$(CONFIG_LOGO_SUPERH_MONO)		+= logo_superh_mono.o
obj-$(CONFIG_LOGO_SUPERH_VGA16)		+= logo_superh_vga16.o
obj-$(CONFIG_LOGO_SUPERH_CLUT224)	+= logo_superh_clut224.o
obj-$(CONFIG_LOGO_VARISCITE_CLUT224)	+= logo_variscite_clut224.o
obj-$(CONFIG_LOGO_SCIAPS_RGB888) 	+= logo_sciaps_qvga_rgb888.o
obj-$(CONFIG_LOGO_SCIAPS_RGB888)	+= logo_sciaps_hvga_rgb888.o

obj-$(CONFIG_SPU_BASE)			+= logo_spe_clut224.o

# How to generate logo's

hostprogs := pnmtologo

ifeq ($(CONFIG_LOGO_SCIAPS_RGB888),y)

ifneq ($(filter ngsr nir,$(SCIAPS_INSTRUMENT_TYPE)),)
SCIAPS_HVGA_LOGO_PPM := logo_sciaps_n_hvga_rgb888.ppm
#Just as a placeholder
SCIAPS_QVGA_LOGO_PPM := logo_sciaps_x_qvga_rgb888.ppm
endif

ifneq ($(filter ngx btx,$(SCIAPS_INSTRUMENT_TYPE)),)
SCIAPS_HVGA_LOGO_PPM := logo_sciaps_x_hvga_rgb888.ppm
SCIAPS_QVGA_LOGO_PPM := logo_sciaps_x_qvga_rgb888.ppm
endif

ifeq ($(SCIAPS_INSTRUMENT_TYPE),ngl)
SCIAPS_HVGA_LOGO_PPM := logo_sciaps_z_hvga_rgb888.ppm
SCIAPS_QVGA_LOGO_PPM := logo_sciaps_z_qvga_rgb888.ppm
endif

$(warning ------> SCIAPS LOGO: SCIAPS_INSTRUMENT_TYPE='$(SCIAPS_INSTRUMENT_TYPE)')
$(warning ------> SCIAPS LOGO: SCIAPS_HVGA_LOGO_PPM='$(SCIAPS_HVGA_LOGO_PPM)')
$(warning ------> SCIAPS LOGO: SCIAPS_QVGA_LOGO_PPM='$(SCIAPS_QVGA_LOGO_PPM)')

ifdef SCIAPS_HVGA_LOGO_PPM

quiet_cmd_sciaps_hvga_logo = LOGO    $@
      cmd_sciaps_hvga_logo = $(obj)/pnmtologo -p -t rgb888 \
		-n logo_sciaps_hvga_rgb888 \
        -o $@ $(srctree)/$(src)/$(SCIAPS_HVGA_LOGO_PPM)

$(obj)/logo_sciaps_hvga_rgb888.c: \
		$(src)/$(SCIAPS_HVGA_LOGO_PPM) $(obj)/pnmtologo FORCE
	$(call if_changed,sciaps_hvga_logo)

endif

ifdef SCIAPS_QVGA_LOGO_PPM

quiet_cmd_sciaps_qvga_logo = LOGO    $@
      cmd_sciaps_qvga_logo = $(obj)/pnmtologo -p -t rgb888 \
		-n logo_sciaps_qvga_rgb888 \
        -o $@ $(srctree)/$(src)/$(SCIAPS_QVGA_LOGO_PPM)

$(obj)/logo_sciaps_qvga_rgb888.c: \
		$(src)/$(SCIAPS_QVGA_LOGO_PPM) $(obj)/pnmtologo FORCE
	$(call if_changed,sciaps_qvga_logo)

endif

endif

# Create commands like "pnmtologo -t mono -n logo_mac_mono -o ..."
quiet_cmd_logo = LOGO    $@
      cmd_logo = $(obj)/pnmtologo -t $(lastword $(subst _, ,$*)) -n $* -o $@ $<

$(obj)/%.c: $(src)/%.pbm $(obj)/pnmtologo FORCE
	$(call if_changed,logo)

$(obj)/%.c: $(src)/%.ppm $(obj)/pnmtologo FORCE
	$(call if_changed,logo)

$(obj)/%.c: $(src)/%.pgm $(obj)/pnmtologo FORCE
	$(call if_changed,logo)

# generated C files
targets += *_mono.c *_vga16.c *_clut224.c *_gray256.c
