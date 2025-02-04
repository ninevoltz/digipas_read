#include <stdio.h>
#include <libusb-1.0/libusb.h>

#define VENDOR_ID 0x24cc
#define PRODUCT_ID 0x00ad
#define HID_REPORT_ID 55

// USB endpoint addresses for communication
#define HID_REPORT_IN_ENDPOINT  0x81 // In endpoint (USB endpoint 1 for input)
#define HID_REPORT_OUT_ENDPOINT 0x01 // Out endpoint (USB endpoint 1 for output)

int main() {
    libusb_context *ctx = NULL;
    libusb_device_handle *handle = NULL;
    libusb_device **dev_list;
    struct libusb_device_descriptor desc;
    ssize_t cnt;
    int r;
	uint16_t angleraw;
	float x_angle, y_angle;
	
    // Initialize libusb
    r = libusb_init(&ctx);
    if (r < 0) {
        fprintf(stderr, "libusb_init failed\n");
        return 1;
    }

    // Get a list of all USB devices connected to the system
    cnt = libusb_get_device_list(ctx, &dev_list);
    if (cnt < 0) {
        fprintf(stderr, "libusb_get_device_list failed\n");
        libusb_exit(ctx);
        return 1;
    }

    // Loop through the devices and find the matching device by vendor ID and product ID
    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device *dev = dev_list[i];

        // Get the device descriptor to check its Vendor ID and Product ID
        r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0) {
            fprintf(stderr, "Failed to get device descriptor\n");
            continue;
        }

        if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID) {
            // Found the device
            printf("Found USB device: Vendor ID 0x%04x, Product ID 0x%04x\n", VENDOR_ID, PRODUCT_ID);

            // Open the device
            r = libusb_open(dev, &handle);
            if (r < 0) {
                fprintf(stderr, "Failed to open device\n");
            } else {
				r = libusb_set_auto_detach_kernel_driver(handle, 1);
				if (r != LIBUSB_SUCCESS) {
					fprintf(stderr, "Cannot set auto-detach kernel driver: %s\n", libusb_error_name(r));
					return 1;
				}
				
                // Claim the interface
                r = libusb_claim_interface(handle, 0);
                if (r != LIBUSB_SUCCESS) {
                    printf("Failed to claim interface, error 0x%04x\n", r);
                    libusb_close(handle);
                    libusb_exit(ctx);
                    return 1;
                }

                // Send HID report with ID 55
                unsigned char report[4] = {HID_REPORT_ID, 0x37, 0xFF, 0xFF};
                r = libusb_interrupt_transfer(handle, HID_REPORT_OUT_ENDPOINT, report, sizeof(report), NULL, 0);
                if (r != LIBUSB_SUCCESS) {
                    fprintf(stderr, "Failed to send report\n");
                } else {
                    printf("Sent HID report: ID 0x%02x, Data 0x37, 0xFF, 0xFF\n", HID_REPORT_ID);
                }

                // Read the response report
                unsigned char response[64]; // Assuming the response fits in 64 bytes
                int transferred;
                r = libusb_interrupt_transfer(handle, HID_REPORT_IN_ENDPOINT, response, sizeof(response), &transferred, 1000);
                if (r != LIBUSB_SUCCESS) {
                    fprintf(stderr, "Failed to read response\n");
                } else {
                    printf("Received HID report: ");
                    for (int i = 0; i < transferred; i++) {
                        printf("0x%02x ", response[i]);
                    }
					
                    printf("\n");
					angleraw = (response[2] << 8) | response[1];
					x_angle = (angleraw - 9000) / 100.0;
					angleraw = (response[4] << 8) | response[3];
					y_angle = (angleraw - 9000) / 100.0;
					printf("%6.3f %6.3f \n", x_angle, y_angle);
                }

                // Release the interface
                libusb_release_interface(handle, 0);

                // Close the device handle
                libusb_close(handle);
            }
            break;
        }
    }

    // Free the device list
    libusb_free_device_list(dev_list, 1);

    // Exit libusb
    libusb_exit(ctx);

    return 0;
}
