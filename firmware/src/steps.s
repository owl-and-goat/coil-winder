.program steps
start:
    pull block
    mov x, osr                  ; steps
    pull block                  ; fill osr in preparation for sleep
loop:
    jmp x-- start
    set pins, 1
    set pins, 0
    mov y, osr                  ; sleep time
sleep:
    jmp y-- sleep
    jmp loop
