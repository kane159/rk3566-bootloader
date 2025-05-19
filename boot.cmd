setenv bootmenu_0 "Boot Miyoo OS (SPI-NOR)=run boot_flash"
setenv bootmenu_1 "Boot SD Card System=run boot_sd"
setenv boot_flash "sf probe; sf read ${kernel_addr_r} 0x100000 0x600000; bootm ${kernel_addr_r}"
setenv boot_sd "mmc dev 1; fatload mmc 1 ${kernel_addr_r} Image; fatload mmc 1 ${fdt_addr_r} rk3566-miyoo.dtb; booti ${kernel_addr_r} - ${fdt_addr_r}"
bootmenu
