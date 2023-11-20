# TheMelobottle


All up to date code will be here (in src folder)
https://youtu.be/tRZGeaHPoaw?si=M6d74r01X-D-Ogb2 great vid on how git and github works, worth a watch.
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Instructions
- Get GIT - https://git-scm.com/downloads
- On the main Melobottle page on github click code - download a zip of all the files (replace current on local computer if necessary)
- Make your nessersary edits
- Open gitbash
- In the command line
     - cd path/to/the unzipped folder(The Melobottle.main)
     - git init
     - git add .(the '.' is to include all files in directory)
     - git commit -m "Your commit message e.g. second commit"
     - git push -u origin master
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
1. **Input Processing Functions:**
   - **Ultrasonic Sensor:**
     - Function to read data from the ultrasonic sensor.
     - Function to convert the sensor data into a usable format (e.g., map the distance to a range of harmonics).

   - **Accelerometer:**
     - Function to read rotational data from the accelerometer.
     - Function to convert rotational data into pitch information.

2. **Signal Generation Functions:**
   - **Waveform Generation:**
     - Functions to generate different waveforms (sawtooth wave, sine wave, Karplus-Strong).
     - Parameters for controlling the characteristics of the generated waveforms (frequency, amplitude, etc.).

  

5. **System Initialization and Control:**
   - **Initialization Function:**
     - Initialize all necessary hardware and software components.
     - 
   - **Main Control Loop:**
     - The main loop that coordinates the system's operation.

The above is the priority at the moment

 ---------------------------------------------------------------------------------------------------------------------------------------------------------

**Mode Switching:**
     - Function to switch between different synthesis modes (sawtooth, sine, Karplus-Strong).
     - Each mode might have its own set of parameters.


3. **User Interface Functions:**
   - **Display Functions:**
     - Functions for updating and interacting with the display.
     - Display mode indicators, parameter values, etc.

   - **User Input Handling:**
     - Functions to handle user inputs (buttons, rotary encoder, etc.).
     - Interpret user inputs for mode switching, parameter adjustments, etc.

4. **Audio Output Functions:**
   - **Audio Output Control:**
     - Function to control the audio output.
     - Adjust volume, mute, etc.

   - **Communication with Audio Hardware:**
     - Interface functions for sending audio data to the audio hardware.

   

Remember to modularize the code, keeping related functions together in separate files. This approach makes it easier to manage, debug, and extend your software. Additionally, it's beneficial to add comments to your code to explain the purpose and functionality of each function.
