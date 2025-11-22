.program home_x
main:
    pull block                  ; osr := sleeps_per_step
    out pins, 1                 ; write direction bit (LSB of speed)
    mov y, osr                  ; y   := osr (sleeps_per_step)
    set pins, 0                 ; reset pins
loop:
    jmp pin end                 ; if limit switch is set, goto out
    set pins, 1                 ; send pulse
    set pins, 0                 ; drop pulse
    mov x, y                    ; x := y (sleeps_per_step)
sleep:                          ; sleep for x cycles
    jmp x-- sleep
    jmp loop                    ; send next pulse
end:
    irq 0 rel
