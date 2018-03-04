# TFTBREW
Beer Brewing and Sousvide PID controller using Arduino

I tried one or two variants of the PID controlled Sous Vide and Beer Brewing Arduino controllers.

Each relied on a 20 by 2 LCD display which I found limiting compared to the more visually friendly LCD TFT touch screen displays available for under £10.

Another benefit being the lack of bounce that the tactile buttons tend to suffer from on the 20 by 2 LCD variants.

So I set about putting together the system I would like Myself for use on Brewday or cooking a steak.

There has been a major rework of the code for the Sous Vide and Ebrew systems.

I think the screen layout is user friendly and as I think of improvements I will provide them as quickly as possible,

The controller is split into 3 main displays:

1) A graph showing the current temperature on a radar like display. It wraps around every 265 readings. Real time, 30 minute, 1 hour, 12 hour, 24 hour, weekly, fortnightly and monthly cycles are available. The Output can be logged to a graph using a Raspberry Pi or whatever You prefer.

2) A graphical output of the PID controlled heating function which again can be logged and saved for use in a graph.

3) A traditional PID like display showing the Setpoint, Process and Output values in alpha numerical form.

All the useful settings can be modified via the touch screen on the Arduino TFT LCD.

My system uses a Raspberry Pi3 for data logging and for storing the various sketches opening the system up for many more projects.

An oscilliscope being the next project I have in mind.

 
