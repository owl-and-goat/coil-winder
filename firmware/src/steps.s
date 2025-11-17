.program steps
main:
    pull block    ; osr := steps
    mov x, osr    ; x   := osr (steps)
    pull block    ; osr := sleeps_per_step
    set pins, 0   ; reset pins
    jmp x-- loop  ; decrement loop counter at start of loop (loops are always do
                  ; while)
    jmp end       ; skip the loop if x is 0
loop:
    mov y, osr    ; y   := osr (sleeps_per_step)
    set pins, 1   ; send pulse
    ;; note we've set up the clock such that the cycle time is equal to the
    ;; intended pulse width (2 μs)
    set pins, 0   ; drop pulse
sleep:            ; sleep for y cycles
    jmp y-- sleep
    jmp x-- loop  ; loop again
end:
    irq 0 rel     ; done; re-sync with firmware
