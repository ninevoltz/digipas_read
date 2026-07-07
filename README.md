A C program to communicate with the Digi-Pas level sensor using libusb-1.0 on Linux and run a closed-loop X-Y goniometer stage with two Pololu Tic T500 controllers.

Build:

```
make
```

Run:

```
./digipas_closed_loop <target_x_deg> <target_y_deg>
```

Targets are limited to the PJ110-15 stage travel of +/-15 degrees. Press ESC to abort.
