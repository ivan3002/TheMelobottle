# TheMelobottle


All up to date code will be here (in src folder)
https://youtu.be/tRZGeaHPoaw?si=M6d74r01X-D-Ogb2 great vid on how git and github works, worth a watch.
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Instructions
- Get GIT - https://git-scm.com/downloads
- Open gitbash
- In the command line
     - cd (path to where you want to store files)
     - git clone https://github.com/ivan3002/TheMelobottle.git
     - cd repository_directory (Replace "repository_directory" with the name of the directory created by the cloning process.) -  e.g. cd M:/TheMelobottle
  It's a good practice to work on a separate branch for your changes. Create and switch to a new branch: 
     - git checkout -b branch_name (e.g. ivan's branch)
  Open the files, make your changes, and save them.
     - git add .
     - git commit -m "Your commit message here"
     - git push origin branch_name

    
 
  - You can use the command git status to see the files that need to be pushed

If you are working on a shared repository and want to merge your changes into the main branch, you typically create a pull request. This can be done on the GitHub website.

Go to your repository on GitHub.
Switch to the branch you just pushed.
Click on "New Pull Request."
Review your changes and create the pull request.
After the pull request is reviewed and accepted, your changes will be merged into the main branch.
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
