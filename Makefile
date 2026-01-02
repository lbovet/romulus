# Makefile for Romulus MIDI Looper LV2 Plugin

# Plugin details
PLUGIN_NAME = romulus
PLUGIN_SO = $(PLUGIN_NAME).so

# Directories
SRC_DIR = src
BUILD_DIR = build
PLUGIN_DIR = plugins/$(PLUGIN_NAME).lv2

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -fPIC -DPIC -std=c99 -O2
LDFLAGS = -shared -lm

# LV2 paths (adjust if needed)
LV2_INCLUDE = /usr/include
INSTALL_DIR = ~/.lv2

# Source files
SRC = $(wildcard $(SRC_DIR)/*.c)
OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRC))

# Targets
.PHONY: all clean install uninstall

all: $(PLUGIN_DIR)/$(PLUGIN_SO)

$(PLUGIN_DIR)/$(PLUGIN_SO): $(OBJ) | $(PLUGIN_DIR)
	$(CC) $(LDFLAGS) -o $@ $^

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(LV2_INCLUDE) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(PLUGIN_DIR):
	mkdir -p $(PLUGIN_DIR)

clean:
	rm -rf $(BUILD_DIR) $(PLUGIN_DIR)/$(PLUGIN_SO)

install: all
	mkdir -p $(INSTALL_DIR)/$(PLUGIN_NAME).lv2
	cp -r $(PLUGIN_DIR)/* $(INSTALL_DIR)/$(PLUGIN_NAME).lv2/

uninstall:
	rm -rf $(INSTALL_DIR)/$(PLUGIN_NAME).lv2
