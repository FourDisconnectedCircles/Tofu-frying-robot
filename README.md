# Tofu-frying-robot
The CAD file and code for the tofu-frying-robot I made (only works with round circular tofu).

You can download the full Arduino code with the library on Platformio with the two zip files included.

To control the robot, connect your webcam to your pc and change the number of camera outputs in the Python code (I have commented it very clearly in the Python code).
Connect the Arduino to the pc via a USB port and specify it in the Python code. The pc basically handles most of the heavy lifting, calculates the tofu offset through the webcam, and sends it to the Arduino.
Put the best.pt file into the python code file yours like the image I put in (it is the only PNG file here), make sure to download all the necessary libraries for it in order for it to work

The offset might change if you change the camera height and other stuff of the robot so because of that, I also put the cube calibration file and the cube tracking file in it for calibration of 1 mm equal how many pixels on the screen, you can change it however you like

The camera can also be used as a phone camera using the app Droid cam, check Droid cam and how to connect to it then connect the phone to the pc in order for it to work.

Have fun :)
