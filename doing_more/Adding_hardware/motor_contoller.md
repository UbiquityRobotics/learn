We developed Magni for people who wanted to build their own robot for a variety of purposes. 
As such, we wanted it to be easy to add powered accessories or sensors to our robot. The heart of Magni is 
the Motor Controller board, which we refer to as the "MC"." While the MC's main purpose is to control
the two drive wheels, it also provides dual 5v and 12v power supply circuits, each is rated at 7amps.
There are a variety of ways to connect to these power sources, including PC style Molex and USB jacks, so if your ever 
a black out, you can charge your mobile device from your robot(Just kidding.)
 
Additionally, the MC contains a real time cock (RTC). We included this because our default CPU, the Raspberry Pi 3, 
does not have one, and time sync is important for ROS when working with multiple robots and computers.
The MC has a lot of safety features including over and under voltage and current protection, automatic redundancy on the 5v
power suppy, reversed polarity. While the default Raspberry Pi 3 connects directly to the MC, an independent connection for 
a USB to TTL dongle allows you to connect any Ubuntu computer.

Recent additions include CLIFF for power management of accessories, and a builtin self-test (BIST) routine to quickly 
diagnose any issues.
