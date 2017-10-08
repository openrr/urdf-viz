#!/bin/bash
BINARY=$(grep name Cargo.toml | cut -d \" -f 2 | head -n1)
OS=ubuntu"$(lsb_release -r | cut -f2)"
ARCH=$(uname -m)
OUT_FILE=${BINARY}-$(grep version Cargo.toml | cut -d \" -f 2 | head -n1)-${OS}-${ARCH}.tar.gz
BUILD=false
BIN_DIR=./target/release/

if [[ "$1" == "-b" ]]; then
    BUILD=true
fi

main() {
    if [[ "$BUILD" == "true" ]]; then
        cargo build --release
    fi
    if [[ -d "$BIN_DIR" ]]; then
        tar -C "$BIN_DIR" -czvf "${OUT_FILE}" ./urdf-viz
        echo "${OUT_FILE} is generated"
    else
        echo "build at first, use -b option"
    fi
}

main
