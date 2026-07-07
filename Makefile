# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -g
LIBS = -lusb-1.0 -lpololu-tic-1 -lm

# Source and target
SRC = main.c
OUT = digipas_closed_loop

# Include paths for libpololu-tic
INCLUDES = -I/usr/local/include/libpololu-tic-1

# Build target
all: $(OUT)

$(OUT): $(SRC)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRC) -o $(OUT) $(LIBS)

# Clean up generated files
clean:
	rm -f $(OUT)
