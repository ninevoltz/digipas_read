# Compiler and flags
CC = gcc
CFLAGS = -Wall -g
LIBS = -lusb-1.0

# Source and target
SRC = main.c
OUT = digipas_read

# Build target
all: $(OUT)

$(OUT): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(OUT) $(LIBS)

# Clean up generated files
clean:
	rm -f $(OUT)
