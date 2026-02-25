{ pkgs ? import <nixpkgs> { } }:

with pkgs;

mkShell {
  buildInputs = [
    # Firmware
    elf2uf2-rs
    probe-rs-tools
    netcat

    # Slicer
    clojure
  ];
  PROBE_RS_CHIP = "rp2040";
}
