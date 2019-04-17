onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib dist_sdmem_ip_opt

do {wave.do}

view wave
view structure
view signals

do {dist_sdmem_ip.udo}

run -all

quit -force
