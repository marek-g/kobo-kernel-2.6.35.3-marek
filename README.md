Original kobo kernel was copied from here:
https://github.com/kobolabs/Kobo-Reader/blob/master/hw/imx507/linux-2.6.35.3.tar.gz

It is kernel for Kobo Glo, Mini and newer Touch but changes were tested only with Kobo Touch N905C.
Compilation was tested with Sourcery-G++ 2010-q1.

Changes:

1)

eInk display driver (frame buffer)

After enabling auto_update (with eink_enable_autoupdate util) the changes of the framebuffer are automatically updated on the screen.

Every time period (Hz/10) changes to the frame buffer are detected with the help of MMU (deffered I/O mechanism). It detects changed pages (blocks of 4kB). The CRC16 of smaller blocks (16x8 pixels) is calculated and finally only changed regions are updated.

2)

Keyboard driver.

Original driver was sending EV_KEY event but without EV_SYN events. It is fixed here.

3)

In the config - framebuffer console is enabled (you need to pass "console=tty0" parameter to the uboot (insted of "console=ttymxc0,115200").

4)

Touch driver (zForce infra red) is enhanced to support multitouch (two fingers). For compatibility with original kobo software it sends both - multitouch and singletouch events now.
