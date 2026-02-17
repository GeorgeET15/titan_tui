#!/bin/bash

# TRIDENT TITAN - Installation Script 🔱🚀
# Automates the build and global installation of the trident command.

# Colors for output
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${CYAN}"
echo "  _______ _____  _____ _____  ______ _   _ _______ "
echo " |__   __|  __ \|_   _|  __ \|  ____| \ | |__   __|"
echo "    | |  | |__) | | | | |  | | |__  |  \| |  | |   "
echo "    | |  |  _  /  | | | |  | |  __| | . \` |  | |   "
echo "    | |  | | \ \ _| |_| |__| | |____| |\  |  | |   "
echo "    |_|  |_|  \_\_____|_____/|______|_| \_|  |_|   "
echo -e "${NC}"
echo -e "${YELLOW}>> Starting Global Installation...${NC}"

# Check for Cargo
if ! command -v cargo &> /dev/null
then
    echo -e "${RED}[ERROR] Rust/Cargo not found.${NC}"
    echo "Please install Rust first: curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
    exit 1
fi

# Build and Install
echo -e "${CYAN}>> Building and installing trident to ~/.cargo/bin/...${NC}"
if cargo install --path . --force; then
    echo -e "${GREEN}✓ TRIDENT installed successfully!${NC}"
    echo ""
    echo -e "${YELLOW}Usage:${NC}"
    echo "  trident"
    echo ""
    echo -e "${CYAN}Note: Ensure ~/.cargo/bin is in your PATH.${NC}"
else
    echo -e "${RED}[ERROR] Installation failed.${NC}"
    exit 1
fi
