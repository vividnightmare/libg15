/*
    This file is part of g15tools.

    g15tools is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    g15tools is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libg15; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    (c) 2006-2007 The G15tools Project - g15tools.sf.net

    $Revision: 324 $ -  $Date: 2011-02-24 00:13:57 +0000 (Thu, 24 Feb 2011) $ $Author: SteelSide $
*/

#include <pthread.h>
#include "libg15.h"
#include <stdio.h>
#include <stdarg.h>
#include <usb.h>
#include <string.h>
#include <errno.h>
#include "config.h"

static usb_dev_handle *keyboard_device = 0;
static int libg15_debugging_enabled = 0;
static int enospc_slowdown = 0;

static int found_devicetype = -1;
static int shared_device = 0;
static int g15_keys_endpoint = 0;
static int g15_lcd_endpoint = 0;
static pthread_mutex_t libusb_mutex;
static int light_state = 0;
static int joystick_x = 0;
static int joystick_y = 0;
static int last_pressed_keys = -1;

/* to add a new device, simply create a new DEVICE() in this list */
/* Fields are: "Name",VendorID,ProductID,Capabilities */
const libg15_devices_t g15_devices[] = {
    DEVICE("Logitech G15",0x46d,0xc222,G15_LCD|G15_KEYS),
    DEVICE("Logitech G11",0x46d,0xc225,G15_KEYS),
    DEVICE("Logitech Z-10",0x46d,0x0a07,G15_LCD|G15_KEYS|G15_DEVICE_IS_SHARED),
    DEVICE("Logitech G15 v2",0x46d,0xc227,G15_LCD|G15_KEYS|G15_DEVICE_5BYTE_RETURN),
    DEVICE("Logitech Gamepanel",0x46d,0xc251,G15_LCD|G15_KEYS|G15_DEVICE_IS_SHARED),
    DEVICE("Logitech G13",0x46d,0xc21c,G15_LCD|G15_KEYS|G15_DEVICE_G13|G15_DEVICE_COLOUR|G15_STORAGE),
    DEVICE("Logitech G110",0x46d,0xc22b,G15_KEYS|G15_DEVICE_G110|G15_DEVICE_COLOUR),
    DEVICE("Logitech G510",0x46d,0xc22d,G15_LCD|G15_KEYS|G15_DEVICE_IS_SHARED|G15_DEVICE_G510|G15_DEVICE_COLOUR), /* without audio activated */
    DEVICE("Logitech G510",0x46d,0xc22e,G15_LCD|G15_KEYS|G15_DEVICE_IS_SHARED|G15_DEVICE_G510|G15_DEVICE_COLOUR), /* with audio activated */
    DEVICE(NULL,0,0,0)
};

/* return device capabilities */
int g15DeviceCapabilities() {
    if(found_devicetype>-1)
        return g15_devices[found_devicetype].caps;
    else
        return -1;
}

/* get the current state of the backlight */
int getBacklightState() {
	return light_state;
}

/* get the current joystick X position  */
int getJoystickX() {
	return joystick_x;
}

/* get the current joystick X position  */
int getJoystickY() {
	return joystick_y;
}


/* enable or disable debugging */
void libg15Debug(int option) {

    libg15_debugging_enabled = option;
    usb_set_debug(option);
}

/* debugging wrapper */
static int g15_log (FILE *fd, unsigned int level, const char *fmt, ...) {

    if (libg15_debugging_enabled && libg15_debugging_enabled>=level) {
        fprintf(fd,"libg15: ");
        va_list argp;
        va_start (argp, fmt);
        vfprintf(fd,fmt,argp);
        va_end (argp);
    }

    return 0;
}

/* return number of connected and supported devices */
int g15NumberOfConnectedDevices() {
    struct usb_bus *bus = 0;
    struct usb_device *dev = 0;
    int i=0;
    unsigned int found = 0;

    for (i=0; g15_devices[i].name !=NULL;i++)
        for (bus = usb_busses; bus; bus = bus->next)
    {
        for (dev = bus->devices; dev; dev = dev->next)
        {
            if ((dev->descriptor.idVendor == g15_devices[i].vendorid && dev->descriptor.idProduct == g15_devices[i].productid))
                found++;
        }
    }
    g15_log(stderr,G15_LOG_INFO,"Found %i supported devices\n",found);
    return found;
}

static int initLibUsb()
{
    g15_log(stderr,G15_LOG_INFO,"Initialising USB\n");
    usb_init();

  /**
     *  usb_find_busses and usb_find_devices both report the number of devices
     *  / busses added / removed since the last call. since this is the first
   *  call we have to return values != 0 or else we didnt find anything */

    if (!usb_find_busses())
        return G15_ERROR_OPENING_USB_DEVICE;

    if (!usb_find_devices())
        return G15_ERROR_OPENING_USB_DEVICE;

    return G15_NO_ERROR;
}

static usb_dev_handle * findAndOpenDevice(libg15_devices_t handled_device, int device_index)
{
    struct usb_bus *bus = 0;
    struct usb_device *dev = 0;
    int retries=0;
    int j,i,k,l;
    int interface=0;

    for (bus = usb_busses; bus; bus = bus->next)
    {
        for (dev = bus->devices; dev; dev = dev->next)
        {
            if ((dev->descriptor.idVendor == handled_device.vendorid && dev->descriptor.idProduct == handled_device.productid))
            {
                int ret=0;
                char name_buffer[65535];
                name_buffer[0] = 0;
                usb_dev_handle *devh = 0;
                found_devicetype = device_index;
                g15_log(stderr,G15_LOG_INFO,"Found %s, trying to open it\n",handled_device.name);
#if 0
                devh = usb_open(dev);
                usb_reset(devh);
                usleep(50*1000);
                usb_close(devh);
#endif
                devh = usb_open(dev);
                if (!devh)
                {
                    g15_log(stderr,G15_LOG_INFO, "Error, could not open the keyboard\n");
                    g15_log(stderr,G15_LOG_INFO, "Perhaps you dont have enough permissions to access it\n");
                    return 0;
                }

                usleep(50*1000);

                g15_log(stderr, G15_LOG_INFO, "Device has %i possible configurations\n",dev->descriptor.bNumConfigurations);

                /* if device is shared with another driver, such as the Z-10 speakers sharing with alsa, we have to disable some calls */
                if(g15DeviceCapabilities() & G15_DEVICE_IS_SHARED)
                  shared_device = 1;

                for (j = 0; j<dev->descriptor.bNumConfigurations;j++){
                    struct usb_config_descriptor *cfg = &dev->config[j];

                    for (i=0;i<cfg->bNumInterfaces; i++){
                        if (g15DeviceCapabilities()&G15_DEVICE_G510){
                            if (i==G510_STANDARD_KEYBOARD_INTERFACE) continue;
                        }

                        struct usb_interface *ifp = &cfg->interface[i];
                        /* if endpoints are already known, finish up */
                        if(g15_keys_endpoint && g15_lcd_endpoint)
                          break;
                        g15_log(stderr, G15_LOG_INFO, "Device has %i Alternate Settings\n", ifp->num_altsetting);

                        for(k=0;k<ifp->num_altsetting;k++){
                            struct usb_interface_descriptor *as = &ifp->altsetting[k];
                            /* verify that the interface is for a HID device */
                            if(as->bInterfaceClass==USB_CLASS_HID){
                                g15_log(stderr, G15_LOG_INFO, "Interface %i has %i Endpoints\n", i, as->bNumEndpoints);
                                usleep(50*1000);
        			/* libusb functions ending in _np are not portable between OS's
                                * Non-linux users will need some way to detach the HID driver from
                                * the G15 until we work out how to do this for other OS's automatically.
                                * For the moment, we just skip this code..
                                */
#ifdef LIBUSB_HAS_GET_DRIVER_NP
                                ret = usb_get_driver_np(devh, i, name_buffer, 65535);
        			/* some kernel versions say that a driver is attached even though there is none
                                in this case the name buffer has not been changed
                                thanks to RobEngle for pointing this out */
                                if (!ret && name_buffer[0])
                                {
                                    g15_log(stderr,G15_LOG_INFO,"Trying to detach driver currently attached: \"%s\"\n",name_buffer);

                                    ret = usb_detach_kernel_driver_np(devh, i);
                                    if (!ret)
                                    {
                                        g15_log(stderr,G15_LOG_INFO,"Success, detached the driver\n");
                                    }
                                    else
                                    {
                                        g15_log(stderr,G15_LOG_INFO,"Sorry, I could not detach the driver, giving up\n");
                                        return 0;
                                    }

                                }
#endif
                                /* don't set configuration if device is shared */
                                if(0 == shared_device) {
                                  ret = usb_set_configuration(devh, 1);
                                  if (ret)
                                  {
                                    g15_log(stderr,G15_LOG_INFO,"Error setting the configuration, this is fatal. Returned %d\n", ret);
                                    return 0;
                                  }
                                }
                                usleep(50*1000);
                                while((ret = usb_claim_interface(devh,i)) && retries <10) {
                                    usleep(50*1000);
                                    retries++;
                                    g15_log(stderr,G15_LOG_INFO,"Trying to claim interface\n");
                                }

                                if (ret)
                                {
                                    g15_log(stderr,G15_LOG_INFO,"Error claiming interface, good day cruel world\n");
                                    return 0;
                                }

                                for (l=0; l< as->bNumEndpoints;l++){
                                    struct usb_endpoint_descriptor *ep=&as->endpoint[l];
                                    g15_log(stderr, G15_LOG_INFO, "Found %s endpoint %i with address 0x%X maxtransfersize=%i \n",
                                            0x80&ep->bEndpointAddress?"\"Extra Keys\"":"\"LCD\"",
                                            ep->bEndpointAddress&0x0f,ep->bEndpointAddress, ep->wMaxPacketSize
                                           );

                                    if(0x80 & ep->bEndpointAddress) {
                                        g15_keys_endpoint = ep->bEndpointAddress;
                                    } else {
                                        g15_lcd_endpoint = ep->bEndpointAddress;
                                    }
#if 0
                                    usb_resetep(devh,ep->bEndpointAddress);
#endif
                                }

                                if (ret)
                                {
                                    g15_log(stderr, G15_LOG_INFO, "Error setting Alternate Interface\n");
                                }
                            }
                        }
                    }
                }


                g15_log(stderr,G15_LOG_INFO,"Done opening the keyboard\n");
                usleep(500*1000); // sleep a bit for good measure
                return devh;
            }
        }
    }
    return 0;
}

static usb_dev_handle * findAndOpen(unsigned int vendorid, unsigned int productid) {
    int i;
    for (i=0; g15_devices[i].name != NULL ;i++){
        if( ( vendorid == 0 || g15_devices[i].vendorid == vendorid ) &
        	( productid == 0 || g15_devices[i].productid == productid ) ) {
            g15_log(stderr,G15_LOG_INFO,"Trying to find %s\n",g15_devices[i].name);
            keyboard_device = findAndOpenDevice(g15_devices[i],i);
			if(keyboard_device){
				break;
			}
			else {
				g15_log(stderr,G15_LOG_INFO,"%s not found\n",g15_devices[i].name);
			}
        }
        else {
			g15_log(stderr,G15_LOG_INFO,"%s skipped\n",g15_devices[i].name);
        }
    }
    return keyboard_device;
}

static usb_dev_handle * findAndOpenG15() {
	return findAndOpen(0, 0);
}


int re_initLibG15()
{

    usb_init();

  /**
     *  usb_find_busses and usb_find_devices both report the number of devices
     *  / busses added / removed since the last call. since this is the first
   *  call we have to return values != 0 or else we didnt find anything */

    if (!usb_find_devices())
        return G15_ERROR_OPENING_USB_DEVICE;

    keyboard_device = findAndOpenG15();
    if (!keyboard_device)
        return G15_ERROR_OPENING_USB_DEVICE;

    return G15_NO_ERROR;
}

int setupLibG15(unsigned int vendorId, unsigned int productId, unsigned int init_usb)
{
	int retval = initLibUsb();
	if(init_usb && retval) {
		return retval;
	}
	else {
	    g15_log(stderr,G15_LOG_INFO,"Skipping libusb initialise\n");
	}


    usb_init();
    usb_find_busses();
    usb_find_devices();
    g15_log(stderr,G15_LOG_INFO,"%s\n",PACKAGE_STRING);

#ifdef SUN_LIBUSB
    g15_log(stderr,G15_LOG_INFO,"Using Sun libusb.\n");
#endif

    g15NumberOfConnectedDevices();
    last_pressed_keys = -1;
    keyboard_device = findAndOpen(vendorId, productId);
    if (!keyboard_device)
        return G15_ERROR_OPENING_USB_DEVICE;

    pthread_mutex_init(&libusb_mutex, NULL);

    if (g15DeviceCapabilities()&G15_DEVICE_G13){
    	unsigned int *pk;
    	getPressedKeys(pk, 2000);
	    g15_log(stderr,G15_LOG_INFO,"Initial keypress %d\n", pk);
    }

    if (g15DeviceCapabilities()&G15_DEVICE_G510){
        g15_log(stderr,G15_LOG_INFO,"Sending G510 initialisation.\n");
		unsigned char usb_data[] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0 };
        pthread_mutex_lock(&libusb_mutex);
        int retval = 0;
        retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x301, 1, (char*)usb_data, 19, 10000);
        unsigned char usb_data_2[] = { 0x09, 0x02, 0, 0, 0, 0, 0, 0 };
        retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x309, 1, (char*)usb_data_2, 8, 10000);
        pthread_mutex_unlock(&libusb_mutex);
    }

    return G15_NO_ERROR;
}

int initLibG15()
{
	return setupLibG15(0, 0, 1);
}

/* reset the keyboard, returning it to a known state */
int exitLibG15()
{
    int retval = G15_NO_ERROR;
    g15_keys_endpoint = 0;
    g15_lcd_endpoint = 0;
    if (keyboard_device){
#ifndef SUN_LIBUSB
        retval = usb_release_interface (keyboard_device, 0);
        usleep(50*1000);
#endif
#if 0
        retval = usb_reset(keyboard_device);
        usleep(50*1000);
#endif
        usb_close(keyboard_device);
        keyboard_device=0;
        pthread_mutex_destroy(&libusb_mutex);
        return retval;
    }
    return -1;
}


static void dumpPixmapIntoLCDFormat(unsigned char *lcd_buffer, unsigned char const *data)
{
/*

  For a set of bytes (A, B, C, etc.) the bits representing pixels will appear on the LCD like this:

	A0 B0 C0
	A1 B1 C1
	A2 B2 C2
	A3 B3 C3 ... and across for G15_LCD_WIDTH bytes
	A4 B4 C4
	A5 B5 C5
	A6 B6 C6
	A7 B7 C7

	A0
	A1  <- second 8-pixel-high row starts straight after the last byte on
	A2     the previous row
	A3
	A4
	A5
	A6
	A7
	A8

	A0
	...
	A0
	...
	A0
	...
	A0
	A1 <- only the first three bits are shown on the bottom row (the last three
	A2    pixels of the 43-pixel high display.)


*/

    unsigned int output_offset = G15_LCD_OFFSET;
    unsigned int base_offset = 0;
    unsigned int curr_row = 0;
    unsigned int curr_col = 0;

    /* Five 8-pixel rows + a little 3-pixel row.  This formula will calculate
       the minimum number of bytes required to hold a complete column.  (It
       basically divides by eight and rounds up the result to the nearest byte,
       but at compile time.
      */

#define G15_LCD_HEIGHT_IN_BYTES  ((G15_LCD_HEIGHT + ((8 - (G15_LCD_HEIGHT % 8)) % 8)) / 8)

    for (curr_row = 0; curr_row < G15_LCD_HEIGHT_IN_BYTES; ++curr_row)
    {
        for (curr_col = 0; curr_col < G15_LCD_WIDTH; ++curr_col)
        {
            unsigned int bit = curr_col % 8;
		/* Copy a 1x8 column of pixels across from the source image to the LCD buffer. */

            lcd_buffer[output_offset] =
			(((data[base_offset                        ] << bit) & 0x80) >> 7) |
			(((data[base_offset +  G15_LCD_WIDTH/8     ] << bit) & 0x80) >> 6) |
			(((data[base_offset + (G15_LCD_WIDTH/8 * 2)] << bit) & 0x80) >> 5) |
			(((data[base_offset + (G15_LCD_WIDTH/8 * 3)] << bit) & 0x80) >> 4) |
			(((data[base_offset + (G15_LCD_WIDTH/8 * 4)] << bit) & 0x80) >> 3) |
			(((data[base_offset + (G15_LCD_WIDTH/8 * 5)] << bit) & 0x80) >> 2) |
			(((data[base_offset + (G15_LCD_WIDTH/8 * 6)] << bit) & 0x80) >> 1) |
			(((data[base_offset + (G15_LCD_WIDTH/8 * 7)] << bit) & 0x80) >> 0);
            ++output_offset;
            if (bit == 7)
              base_offset++;
        }
	/* Jump down seven pixel-rows in the source image, since we've just
	   done a row of eight pixels in one pass (and we counted one pixel-row
  	   while we were going, so now we skip the next seven pixel-rows.) */
	base_offset += G15_LCD_WIDTH - (G15_LCD_WIDTH / 8);
    }
}

int handle_usb_errors(const char *prefix, int ret) {

    switch (ret){
        case -ETIMEDOUT:
            return G15_ERROR_READING_USB_DEVICE;  /* backward-compatibility */
            break;
		case -ENOSPC: /* the we dont have enough bandwidth, apparently.. something has to give here.. */
			g15_log(stderr,G15_LOG_INFO,"usb error: ENOSPC.. reducing speed\n");
			enospc_slowdown = 1;
			break;
		case -ENODEV: /* the device went away - we probably should attempt to reattach */
		case -ENXIO: /* host controller bug */
		case -EINVAL: /* invalid request */
		case -EAGAIN: /* try again */
		case -EFBIG: /* too many frames to handle */
		case -EMSGSIZE: /* msgsize is invalid */
			 g15_log(stderr,G15_LOG_INFO,"usb error: %s %s (%i)\n",prefix,usb_strerror(),ret);
			 break;
		case -EPIPE: /* endpoint is stalled */
			 g15_log(stderr,G15_LOG_INFO,"usb error: %s EPIPE! clearing...\n",prefix);
			 pthread_mutex_lock(&libusb_mutex);
			 usb_clear_halt(keyboard_device, 0x81);
			 pthread_mutex_unlock(&libusb_mutex);
			 break;
		default: /* timed out */
			 g15_log(stderr,G15_LOG_INFO,"Unknown usb error: %s !! (err is %i (%s))\n",prefix,ret,usb_strerror());
			 break;
    }
    return ret;
}

int writePixmapToLCD(unsigned char const *data)
{
    int ret = 0;
    int transfercount=0;
    unsigned char lcd_buffer[G15_BUFFER_LEN];
    /* The pixmap conversion function will overwrite everything after G15_LCD_OFFSET, so we only need to blank
       the buffer up to this point.  (Even though the keyboard only cares about bytes 0-23.) */
    memset(lcd_buffer, 0, G15_LCD_OFFSET);  /* G15_BUFFER_LEN); */

    dumpPixmapIntoLCDFormat(lcd_buffer, data);

    if(!(g15_devices[found_devicetype].caps & G15_LCD))
        return 0;

    /* the keyboard needs this magic byte */
    lcd_buffer[0] = 0x03;
  /* in an attempt to reduce peak bus utilisation, we break the transfer into 32 byte chunks and sleep a bit in between.
    It shouldnt make much difference, but then again, the g15 shouldnt be flooding the bus enough to cause ENOSPC, yet
    apparently does on some machines...
    I'm not sure how successful this will be in combatting ENOSPC, but we'll give it try in the real-world. */

    if(enospc_slowdown != 0){
#ifndef LIBUSB_BLOCKS
        pthread_mutex_lock(&libusb_mutex);
#endif
        for(transfercount = 0;transfercount<=31;transfercount++){
            ret = usb_interrupt_write(keyboard_device, g15_lcd_endpoint, (char*)lcd_buffer+(32*transfercount), 32, 1000);
            if (ret != 32)
            {
                handle_usb_errors ("LCDPixmap Slow Write",ret);
                return G15_ERROR_WRITING_PIXMAP;
            }
            usleep(100);
        }
#ifndef LIBUSB_BLOCKS
        pthread_mutex_unlock(&libusb_mutex);
#endif
    }else{
        /* transfer entire buffer in one hit */
#ifdef LIBUSB_BLOCKS
        ret = usb_interrupt_write(keyboard_device, g15_lcd_endpoint, (char*)lcd_buffer, G15_BUFFER_LEN, 1000);
#else
        pthread_mutex_lock(&libusb_mutex);
        ret = usb_interrupt_write(keyboard_device, g15_lcd_endpoint, (char*)lcd_buffer, G15_BUFFER_LEN, 1000);
        pthread_mutex_unlock(&libusb_mutex);
#endif
        if (ret != G15_BUFFER_LEN)
        {
            handle_usb_errors ("LCDPixmap Write",ret);
            return G15_ERROR_WRITING_PIXMAP;
        }
        usleep(100);
    }

    return 0;
}

//int readProfile(unsigned char const *data)
//{
//	int retval = 0;
//	unsigned char usb_data[] = { 4, 2, 0, 0, 0 };
//
//	if(shared_device>0 || !( g15DeviceCapabilities() & G15_STORAGE ))
//		return G15_ERROR_UNSUPPORTED;
//
//	// Send request
//    g15_log(stderr,G15_LOG_INFO,"Send profile data read request\n");
//	pthread_mutex_lock(&libusb_mutex);
//	retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x304, 0, (char*)usb_data, 4, 10000);
//	pthread_mutex_unlock(&libusb_mutex);
//
//    unsigned char buffer[258];
//	for(int i = 0 ; i < 128; i++) {
//	    g15_log(stderr,G15_LOG_INFO,"Reading block %i\n", i);
//		int realNumBytes = usb_control_msg(
//				keyboard_device,             // handle obtained with usb_open()
//		        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, // bRequestType
//		        1,      // bRequest
//		        0x0306,              // wValue
//		        0,              // wIndex
//		        buffer,             // pointer to destination buffer
//		        258,  // wLength
//		        10000
//		    );
//	    g15_log(stderr,G15_LOG_INFO,"Got back %i\n", realNumBytes);
//	}
//
//	usleep(100);
//
//    return 0;
//}

int setLCDContrast(unsigned int level)
{
    int retval = 0;
    unsigned char usb_data[] = { 2, 32, 129, 0 };

    if(shared_device>0)
        return G15_ERROR_UNSUPPORTED;

    switch(level)
    {
        case 1:
            usb_data[3] = 22;
            break;
        case 2:
            usb_data[3] = 26;
            break;
        default:
            usb_data[3] = 18;
            break;
    }
    pthread_mutex_lock(&libusb_mutex);
    retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x302, 0, (char*)usb_data, 4, 10000);
    pthread_mutex_unlock(&libusb_mutex);
    return retval;
}

int setLEDs(unsigned int leds)
{
    int retval = 0;

    pthread_mutex_lock(&libusb_mutex);

    if (g15DeviceCapabilities()&G15_DEVICE_G13){
        unsigned char m_led_buf[5] = { 5, (unsigned char)leds, 0, 0, 0 };
    	retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x305, 0, (char*)m_led_buf, 5, 10000);
    }
    else if (g15DeviceCapabilities()&G15_DEVICE_G110){
        unsigned char m_led_buf[2] = { 3, (unsigned char)leds };
    	retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x303, 1, (char*)m_led_buf, 2, 10000);
    }
    else if (g15DeviceCapabilities()&G15_DEVICE_G510){
    	// M-key light mask is different on this model
    	int new_leds = 0;
    	if(leds & 0x01) {
    		new_leds += 0x80;
    	}
    	if(leds & 0x02) {
    		new_leds += 0x40;
    	}
    	if(leds & 0x04) {
    		new_leds += 0x20;
    	}
    	if(leds & 0x08) {
    		new_leds += 0x10;
    	}
        unsigned char m_led_buf[2] = { 4, (unsigned char)new_leds };
    	retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x304, 1, (char*)m_led_buf, 2, 10000);
    }
    else {
    	if(shared_device>0) {
    		return G15_ERROR_UNSUPPORTED;
    	}
    	else {
			unsigned char m_led_buf[4] = { 2, 4, ~(unsigned char)leds, 0 };
			retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x302, 0, (char*)m_led_buf, 4, 10000);
    	}
    }
    pthread_mutex_unlock(&libusb_mutex);
    return retval;
}

int setLCDBrightness(unsigned int level)
{
    int retval = 0;
    unsigned char usb_data[] = { 2, 2, 0, 0 };

    if(shared_device>0 || ( g15DeviceCapabilities() & G15_DEVICE_COLOUR ))
        return G15_ERROR_UNSUPPORTED;

    switch(level)
    {
        case 1 :
            usb_data[2] = 0x10;
            break;
        case 2 :
            usb_data[2] = 0x20;
            break;
        default:
            usb_data[2] = 0x00;
            break;
    }
    pthread_mutex_lock(&libusb_mutex);
    retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x302, 0, (char*)usb_data, 4, 10000);
    pthread_mutex_unlock(&libusb_mutex);
    return retval;
}

/* set the keyboard backlight. doesnt affect lcd backlight. 0==off,1==medium,2==high */
int setKBBrightness(unsigned int level)
{
    int retval = 0;
    unsigned char usb_data[] = { 2, 1, 0, 0 };

    if(shared_device>0 || ( g15DeviceCapabilities() & G15_DEVICE_COLOUR ) )
        return G15_ERROR_UNSUPPORTED;

    switch(level)
    {
        case 1 :
            usb_data[2] = 0x1;
            break;
        case 2 :
            usb_data[2] = 0x2;
            break;
        default:
            usb_data[2] = 0x0;
            break;
    }
    pthread_mutex_lock(&libusb_mutex);
    retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x302, 0, (char*)usb_data, 4, 10000);
    pthread_mutex_unlock(&libusb_mutex);
    return retval;
}

int setG510LEDColor(unsigned char r, unsigned char g, unsigned char b)
{
    if((!g15DeviceCapabilities() & G15_DEVICE_COLOUR))
        return G15_ERROR_UNSUPPORTED;

    int retval = 0;
    unsigned char usb_data[] = { 4, 0, 0, 0, 0 };

    usb_data[1] = r;
    usb_data[2] = g;
    usb_data[3] = b;

    pthread_mutex_lock(&libusb_mutex);

    if (g15DeviceCapabilities()&G15_DEVICE_G110){
	    usb_data[0] = 7;
	   // If the intensities are the same, "colour" is 0x80
	    if ( r == b ) {
	    	usb_data[1] = 0x80;
	    	usb_data[4] = b>>4;
	    }
	   // If the blue value is higher
	    else if ( b > r ) {
	    	usb_data[1] = 0xff - ( 0x80 * r ) / b;
	    	usb_data[4] = b>>4;
	    }
	   // If the red value is higher
	    else {
	    	usb_data[1] = ( 0x80 * b ) / r;
	    	usb_data[4] = r>>4;
	    }
    	retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x307, 0, (char*)usb_data, 5, 10000);
    }
    else if (g15DeviceCapabilities()&G15_DEVICE_G13){
	    usb_data[0] = 5;
    	retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x307, 0, (char*)usb_data, 4, 10000);
    }
    else {
	    usb_data[0] = 5;
    	retval = usb_control_msg(keyboard_device, USB_TYPE_CLASS + USB_RECIP_INTERFACE, 9, 0x305, 1, (char*)usb_data, 4, 10000);
    }
    pthread_mutex_unlock(&libusb_mutex);
    return retval;
}

static unsigned char g15KeyToLogitechKeyCode(int key)
{
   // first 12 G keys produce F1 - F12, thats 0x3a + key
    if (key < 12)
    {
        return 0x3a + key;
    }
   // the other keys produce Key '1' (above letters) + key, thats 0x1e + key
    else
    {
        return 0x1e + key - 12; // sigh, half an hour to find  -12 ....
    }
}

static void processKeyEventG13(unsigned int *pressed_keys, unsigned char *buffer)
{
    int i;

    *pressed_keys = 0;

    if (buffer[0] == 0x01)
    {
        if (buffer[3]&0x01)
      *pressed_keys |= G15_KEY_G1;
        if (buffer[3]&0x02)
      *pressed_keys |= G15_KEY_G2;
        if (buffer[3]&0x04)
      *pressed_keys |= G15_KEY_G3;
        if (buffer[3]&0x08)
      *pressed_keys |= G15_KEY_G4;
        if (buffer[3]&0x10)
      *pressed_keys |= G15_KEY_G5;
        if (buffer[3]&0x20)
      *pressed_keys |= G15_KEY_G6;
        if (buffer[3]&0x40)
      *pressed_keys |= G15_KEY_G7;
        if (buffer[3]&0x80)
      *pressed_keys |= G15_KEY_G8;

        if (buffer[4]&0x01)
      *pressed_keys |= G15_KEY_G9;
        if (buffer[4]&0x02)
      *pressed_keys |= G15_KEY_G10;
        if (buffer[4]&0x04)
      *pressed_keys |= G15_KEY_G11;
        if (buffer[4]&0x08)
      *pressed_keys |= G15_KEY_G12;
        if (buffer[4]&0x10)
      *pressed_keys |= G15_KEY_G13;
        if (buffer[4]&0x20)
      *pressed_keys |= G15_KEY_G14;
        if (buffer[4]&0x40)
      *pressed_keys |= G15_KEY_G15;
        if (buffer[4]&0x80)
      *pressed_keys |= G15_KEY_G16;
        if (buffer[5]&0x01)
      *pressed_keys |= G15_KEY_G17;
        if (buffer[5]&0x02)
      *pressed_keys |= G15_KEY_G18;
        if (buffer[5]&0x80)
      *pressed_keys |= G15_KEY_LIGHT;

        if (buffer[6]&0x01)
      *pressed_keys |= G15_KEY_L1;
        if (buffer[6]&0x02)
      *pressed_keys |= G15_KEY_L2;
        if (buffer[6]&0x04)
      *pressed_keys |= G15_KEY_L3;
        if (buffer[6]&0x08)
      *pressed_keys |= G15_KEY_L4;
        if (buffer[6]&0x10)
      *pressed_keys |= G15_KEY_L5;

        if (buffer[6]&0x20)
      *pressed_keys |= G15_KEY_M1;
        if (buffer[6]&0x40)
      *pressed_keys |= G15_KEY_M2;
        if (buffer[6]&0x80)
      *pressed_keys |= G15_KEY_M3;
        if (buffer[7]&0x01)
      *pressed_keys |= G15_KEY_MR;

    }

}

static void processKeyEventG13Extended(unsigned int *pressed_keys, unsigned char *buffer)
{
    int i;

    *pressed_keys = 0;

    if (buffer[0] == 0x01)
    {

		if (buffer[5]&0x04) {
		*pressed_keys |= G15_KEY_G19;
		*pressed_keys |= G15_EXTENDED_KEY;
		}
		if (buffer[5]&0x08) {
		*pressed_keys |= G15_KEY_G20;
		*pressed_keys |= G15_EXTENDED_KEY;
		}
		if (buffer[5]&0x10) {
		*pressed_keys |= G15_KEY_G21;
		*pressed_keys |= G15_EXTENDED_KEY;
		}
		if (buffer[5]&0x20) {
		*pressed_keys |= G15_KEY_G22;
		*pressed_keys |= G15_EXTENDED_KEY;
		}
		if (buffer[7]&0x02) {
		*pressed_keys |= G15_KEY_JOYBL;
		*pressed_keys |= G15_EXTENDED_KEY;
		}
		if (buffer[7]&0x04) {
		*pressed_keys |= G15_KEY_JOYBD;
		*pressed_keys |= G15_EXTENDED_KEY;
		}
		if (buffer[7]&0x08) {
		*pressed_keys |= G15_KEY_JOYBS;
		*pressed_keys |= G15_EXTENDED_KEY;
		}
    }

	  // Bytes 1 and 2 are the joystick positions
	  int jx = buffer[1];
	  int jy = buffer[2];
	  if(jx != joystick_x || jy != joystick_y) {
		  joystick_x = jx;
		  joystick_y = jy;
		  *pressed_keys |= G15_JOY;
		  *pressed_keys |= G15_EXTENDED_KEY;
	  }
}


static void processG510AudioKeyEvent(unsigned int *pressed_keys, unsigned char *buffer)
{
    int i;
    *pressed_keys = 0;
    if (buffer[0] == 0x03)
    {

		if (buffer[4]&0x20) {
		*pressed_keys |= G15_KEY_MUTE_OUTPUT;
		*pressed_keys |= G15_EXTENDED_KEY;
		}
		if (buffer[4]&0x40) {
		*pressed_keys |= G15_KEY_MUTE_INPUT;
		*pressed_keys |= G15_EXTENDED_KEY;
		}
    }
}

static void processKeyEvent9Byte(unsigned int *pressed_keys, unsigned char *buffer)
{
    int i;

    *pressed_keys = 0;

    g15_log(stderr,G15_LOG_INFO,"Keyboard: %x, %x, %x, %x, %x, %x, %x, %x, %x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8]);

    if (buffer[0] == 0x02)
    {
        if (buffer[1]&0x01)
            *pressed_keys |= G15_KEY_G1;

        if (buffer[2]&0x02)
            *pressed_keys |= G15_KEY_G2;

        if (buffer[3]&0x04)
            *pressed_keys |= G15_KEY_G3;

        if (buffer[4]&0x08)
            *pressed_keys |= G15_KEY_G4;

        if (buffer[5]&0x10)
            *pressed_keys |= G15_KEY_G5;

        if (buffer[6]&0x20)
            *pressed_keys |= G15_KEY_G6;


        if (buffer[2]&0x01)
            *pressed_keys |= G15_KEY_G7;

        if (buffer[3]&0x02)
            *pressed_keys |= G15_KEY_G8;

        if (buffer[4]&0x04)
            *pressed_keys |= G15_KEY_G9;

        if (buffer[5]&0x08)
            *pressed_keys |= G15_KEY_G10;

        if (buffer[6]&0x10)
            *pressed_keys |= G15_KEY_G11;

        if (buffer[7]&0x20)
            *pressed_keys |= G15_KEY_G12;

        if (buffer[1]&0x04)
            *pressed_keys |= G15_KEY_G13;

        if (buffer[2]&0x08)
            *pressed_keys |= G15_KEY_G14;

        if (buffer[3]&0x10)
            *pressed_keys |= G15_KEY_G15;

        if (buffer[4]&0x20)
            *pressed_keys |= G15_KEY_G16;

        if (buffer[5]&0x40)
            *pressed_keys |= G15_KEY_G17;

        if (buffer[8]&0x40)
            *pressed_keys |= G15_KEY_G18;

        if (buffer[6]&0x01)
            *pressed_keys |= G15_KEY_M1;
        if (buffer[7]&0x02)
            *pressed_keys |= G15_KEY_M2;
        if (buffer[8]&0x04)
            *pressed_keys |= G15_KEY_M3;
        if (buffer[7]&0x40)
            *pressed_keys |= G15_KEY_MR;

        if (buffer[8]&0x80)
            *pressed_keys |= G15_KEY_L1;
        if (buffer[2]&0x80)
            *pressed_keys |= G15_KEY_L2;
        if (buffer[3]&0x80)
            *pressed_keys |= G15_KEY_L3;
        if (buffer[4]&0x80)
            *pressed_keys |= G15_KEY_L4;
        if (buffer[5]&0x80)
            *pressed_keys |= G15_KEY_L5;

        if (buffer[1]&0x80)
            *pressed_keys |= G15_KEY_LIGHT;

    }
}

static void processKeyEvent5Byte(unsigned int *pressed_keys, unsigned char *buffer)
{
    int i;

    *pressed_keys = 0;

    g15_log(stderr,G15_LOG_INFO,"Keyboard: %x, %x, %x, %x, %x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);

    if (buffer[0] == 0x02)
    {
        if (buffer[1]&0x01)
            *pressed_keys |= G15_KEY_G1;

        if (buffer[1]&0x02)
            *pressed_keys |= G15_KEY_G2;

        if (buffer[1]&0x04)
            *pressed_keys |= G15_KEY_G3;

        if (buffer[1]&0x08)
            *pressed_keys |= G15_KEY_G4;

        if (buffer[1]&0x10)
            *pressed_keys |= G15_KEY_G5;

        if (buffer[1]&0x20)
            *pressed_keys |= G15_KEY_G6;

        if (buffer[1]&0x40)
            *pressed_keys |= G15_KEY_M1;

        if (buffer[1]&0x80)
            *pressed_keys |= G15_KEY_M2;

        if (buffer[2]&0x20)
            *pressed_keys |= G15_KEY_M3;

        if (buffer[2]&0x40)
            *pressed_keys |= G15_KEY_MR;

        if (buffer[2]&0x80)
            *pressed_keys |= G15_KEY_L1;

        if (buffer[2]&0x2)
            *pressed_keys |= G15_KEY_L2;

        if (buffer[2]&0x4)
            *pressed_keys |= G15_KEY_L3;

        if (buffer[2]&0x8)
            *pressed_keys |= G15_KEY_L4;

        if (buffer[2]&0x10)
            *pressed_keys |= G15_KEY_L5;

        if (buffer[2]&0x1)
            *pressed_keys |= G15_KEY_LIGHT;
    }

    // G510
    if (buffer[0] == 0x03)
    {
        if (buffer[1]&0x01)
            *pressed_keys |= G15_KEY_G1;

        if (buffer[1]&0x02)
            *pressed_keys |= G15_KEY_G2;

        if (buffer[1]&0x04)
            *pressed_keys |= G15_KEY_G3;

        if (buffer[1]&0x08)
            *pressed_keys |= G15_KEY_G4;

        if (buffer[1]&0x10)
            *pressed_keys |= G15_KEY_G5;

        if (buffer[1]&0x20)
            *pressed_keys |= G15_KEY_G6;

        if (buffer[1]&0x40)
            *pressed_keys |= G15_KEY_G7;

        if (buffer[1]&0x80)
            *pressed_keys |= G15_KEY_G8;

        if (buffer[2]&0x01)
            *pressed_keys |= G15_KEY_G9;

        if (buffer[2]&0x02)
            *pressed_keys |= G15_KEY_G10;

        if (buffer[2]&0x04)
            *pressed_keys |= G15_KEY_G11;

        if (buffer[2]&0x08)
            *pressed_keys |= G15_KEY_G12;

        if (buffer[2]&0x10)
            *pressed_keys |= G15_KEY_G13;

        if (buffer[2]&0x20)
            *pressed_keys |= G15_KEY_G14;

        if (buffer[2]&0x40)
            *pressed_keys |= G15_KEY_G15;

        if (buffer[2]&0x80)
            *pressed_keys |= G15_KEY_G16;

        if (buffer[3]&0x01)
            *pressed_keys |= G15_KEY_G17;

        if (buffer[3]&0x02)
            *pressed_keys |= G15_KEY_G18;

        if (buffer[3]&0x10)
            *pressed_keys |= G15_KEY_M1;

        if (buffer[3]&0x20)
            *pressed_keys |= G15_KEY_M2;

        if (buffer[3]&0x40)
            *pressed_keys |= G15_KEY_M3;

        if (buffer[3]&0x80)
            *pressed_keys |= G15_KEY_MR;

        if (buffer[4]&0x1)
            *pressed_keys |= G15_KEY_L1;

        if (buffer[4]&0x2)
            *pressed_keys |= G15_KEY_L2;

        if (buffer[4]&0x4)
            *pressed_keys |= G15_KEY_L3;

        if (buffer[4]&0x8)
            *pressed_keys |= G15_KEY_L4;

        if (buffer[4]&0x10)
            *pressed_keys |= G15_KEY_L5;

        if (buffer[3]&0x8)
            *pressed_keys |= G15_KEY_LIGHT;
    }
}

static void processKeyEvent4Byte(unsigned int *pressed_keys, unsigned char *buffer)
{
	int i;

	*pressed_keys = 0;

	g15_log(stderr,G15_LOG_INFO,"Keyboard: %x, %x, %x, %x\n",buffer[0],buffer[1],buffer[2],buffer[3]);

	if (buffer[0] == 0x02)
	{
		if (buffer[1]&0x01)
			*pressed_keys |= G15_KEY_G1;

		if (buffer[1]&0x02)
			*pressed_keys |= G15_KEY_G2;

		if (buffer[1]&0x04)
			*pressed_keys |= G15_KEY_G3;

		if (buffer[1]&0x08)
			*pressed_keys |= G15_KEY_G4;

		if (buffer[1]&0x10)
			*pressed_keys |= G15_KEY_G5;

		if (buffer[1]&0x20)
			*pressed_keys |= G15_KEY_G6;

		if (buffer[1]&0x40)
			*pressed_keys |= G15_KEY_G7;

		if (buffer[1]&0x80)
			*pressed_keys |= G15_KEY_G8;


		if (buffer[2]&0x01)
			*pressed_keys |= G15_KEY_G9;

		if (buffer[2]&0x02)
			*pressed_keys |= G15_KEY_G10;

		if (buffer[2]&0x04)
			*pressed_keys |= G15_KEY_G11;

		if (buffer[2]&0x08)
			*pressed_keys |= G15_KEY_G12;

		if (buffer[2]&0x10)
			*pressed_keys |= G15_KEY_M1;

		if (buffer[2]&0x20)
			*pressed_keys |= G15_KEY_M2;

		if (buffer[2]&0x40)
			*pressed_keys |= G15_KEY_M3;

		if (buffer[2]&0x80)
			*pressed_keys |= G15_KEY_MR;

		if (buffer[3]&0x1)
			*pressed_keys |= G15_KEY_LIGHT;
	}
}


static void processKeyEvent2Byte(unsigned int *pressed_keys, unsigned char *buffer)
{
	*pressed_keys = 0;

    g15_log(stderr,G15_LOG_WARN,"Keyboard: %x, %x\n", buffer[0], buffer[1]);

    if (buffer[0] == 0x02)
    {
        // XF86AudioPlay
        if (buffer[1] & 0x08) {
            *pressed_keys |= G15_KEY_PLAY;
			*pressed_keys |= G15_EXTENDED_KEY;
        }

        // XF86AudioStop
        if (buffer[1] & 0x04) {
            *pressed_keys |= G15_KEY_STOP;
			*pressed_keys |= G15_EXTENDED_KEY;
        }

        // XF86AudioPrev
        if (buffer[1] & 0x02) {
            *pressed_keys |= G15_KEY_PREV;
			*pressed_keys |= G15_EXTENDED_KEY;
        }

        // XF86AudioNext
        if (buffer[1] & 0x01) {
            *pressed_keys |= G15_KEY_NEXT;
			*pressed_keys |= G15_EXTENDED_KEY;
        }

        // XF86AudioMute
        if (buffer[1] & 0x10) {
            *pressed_keys |= G15_KEY_MUTE;
			*pressed_keys |= G15_EXTENDED_KEY;
        }

        // XF86AudioRaiseVolume
        if (buffer[1] & 0x20) {
            *pressed_keys |= G15_KEY_RAISE_VOLUME;
			*pressed_keys |= G15_EXTENDED_KEY;
        }

        // XF86AudioLowerVolume
        if (buffer[1] & 0x40) {
            *pressed_keys |= G15_KEY_LOWER_VOLUME;
			*pressed_keys |= G15_EXTENDED_KEY;
        }
    }
}


int getPressedKeys(unsigned int *pressed_keys, unsigned int timeout)
{
	if(last_pressed_keys > -1) {
		/* if we buffered keypresses because there was a 'extended' event */
		*pressed_keys = last_pressed_keys;
		last_pressed_keys = -1;
        return G15_NO_ERROR;
	}

    unsigned char buffer[G15_KEY_READ_LENGTH];
    int x = 0;
    int ret = 0;
    int caps = g15DeviceCapabilities();
    int read_length = G15_KEY_READ_LENGTH;

    if(caps & G15_DEVICE_G13) {
    	read_length = G13_KEY_READ_LENGTH;
    }
    
    // G510/G510s has 8 byte read length
    if(caps & G15_DEVICE_G510) {
        read_length = G13_KEY_READ_LENGTH;
    }

    for( x = 0 ; x < read_length; x++) {
        buffer[x] = 0;
    }

#ifdef LIBUSB_BLOCKS
    ret = usb_interrupt_read(keyboard_device, g15_keys_endpoint, (char*)buffer, read_length, timeout);
#else
    //pthread_mutex_lock(&libusb_mutex);
    ret = usb_interrupt_read(keyboard_device, g15_keys_endpoint, (char*)buffer, read_length, timeout);
    //pthread_mutex_unlock(&libusb_mutex);
#endif

    if(ret > 0 && libg15_debugging_enabled == G15_LOG_INFO) {
    	g15_log(stderr,G15_LOG_INFO,"rl: %d Ret: %x, xBuf[0]: %x\n",read_length, ret, buffer[0]);
    	int i;
    	for(i = 0 ; i < ret; i++) {
        	g15_log(stderr,G15_LOG_INFO,"    %x\n", buffer[i]);
    	}
    }

	if(caps & G15_DEVICE_G13){
		// G13 sometimes returns -110, but has filled the buffer with keydata
		if(ret < 1 && buffer[0] == 1) {
			ret = 0;
		}
	}
	else if(ret > 0 && buffer[0] == 1) {
		return G15_ERROR_TRY_AGAIN;
	}


    if((caps & G15_DEVICE_G13) && buffer[0]==0x01){

      // The top bit of the the 6th byte indicates whether the backlight is on or off
      // Get the state of it, then mask out the bit
      if(buffer[5] & 0x80) {
    	  if(light_state == 0) {
    	      light_state = 1;
    	  }
    	  buffer[5] &= ~(0x80);
      }
      else {
    	  if(light_state == 1) {
    	      light_state = 0;
    	  }
      }

      // The top bit of the 8th byte flaps about, no idea why so we just clear it
	  buffer[7] &= ~(0x80);
	  processKeyEventG13(pressed_keys, buffer);

	  /* Handle the "extended" keys. If one if pressed, then store what
	   * non-extended keys were pressed for the next call
	   */
	  unsigned int pressed_ext_keys = 0;
	  processKeyEventG13Extended(&pressed_ext_keys, buffer);
	  if(pressed_ext_keys > 0) {
		  last_pressed_keys = *pressed_keys;
		  *pressed_keys = pressed_ext_keys;
		  return G15_NO_ERROR;
	  }

      return G15_NO_ERROR;
    }


    switch(ret) {
      case 4:
          processKeyEvent4Byte(pressed_keys, buffer);
          return G15_NO_ERROR;
      case 5:
          processKeyEvent5Byte(pressed_keys, buffer);
          if(caps & G15_DEVICE_G510){
        	  unsigned int pressed_ext_keys = 0;
        	  processG510AudioKeyEvent(&pressed_ext_keys, buffer);
			  if(pressed_ext_keys > 0) {
				  last_pressed_keys = *pressed_keys;
				  *pressed_keys = pressed_ext_keys;
				  return G15_NO_ERROR;
			  }
          }
          return G15_NO_ERROR;
      case 9:
          processKeyEvent9Byte(pressed_keys, buffer);
          return G15_NO_ERROR;
      case 2:
          if(caps & G15_DEVICE_G510){
         	  processKeyEvent2Byte(pressed_keys, buffer);
         	  return G15_NO_ERROR;
          } // Deliberate fallthrough
      default:
          return handle_usb_errors("Keyboard Read", ret); /* allow the app to deal with errors */
    }
}
