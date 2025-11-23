{ pkgs ? import <nixpkgs> { } }:

with pkgs;

mkShell {
  buildInputs = [
    # Firmware
    elf2uf2-rs
    probe-rs
    netcat

    # Slicer
    clojure
  ];
  PROBE_RS_CHIP = "rp2040";
}
