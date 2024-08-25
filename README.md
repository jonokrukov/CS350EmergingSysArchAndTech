

**Portfolio Journal**

**1. Summarize the project and what problem it was solving.**

The project uploaded to this repository involved developing a smart thermostat prototype utilizing the TI CC3320S-LAUNCHXL board. My code integrates the board's TMP006 sensor for temperature data via I2C, an LED to indicate whether or not the heater has been activated via GPIO, and two buttons to adjust the set-point temperature, which dictates when the heater will be activated, via GPIO interrupts. I also implemented UART communication to simulate sending data to a server. The project's documentation also compares different hardware architectures with regard to their peripheral support, Wi-Fi connectivity, and Flash and RAM. 

**2. What did you do particularly well?**

The seamless integration of multiple hardware components and the implementation of a task scheduler to efficiently manage multiple tasks are two areas I feel I did well in. Additionally, my code is clearly commented, and the documentation provided details the design choices I made and explains the algorithm implemented. 

**3. Where could you improve?**

While my system functions well, my task scheduler could further be improved by enhancing its scalability and enabling it to support more complex algorithms or more peripherals. Error handling is another aspect of my code that could be improved, as my code largely overlooks this currently. 

**4. What tools and/or resources are you adding to your support network?**

Throughout this course, I was able to gain experience working with technical reference manuals and microcontroller data sheets that will prove invaluable if I work with these types of systems in the future. My experience during this course also taught me that there are many online resources and forums that can have tons of helpful inputs or fresh perspectives whenever an obstacle is encountered.

**5. What skills from this project will be particularly transferable to other projects and/or course work?**

My work on this project gave me experience in hardware-software integration, embedded C programming, and real-time task management. All of these skills would be directly applicable if working with Internet of Things (IoT) devices or other embedded devices that integrate hardware with software. 

**6. How did you make this project maintainable, readable, and adaptable?**

I made my project maintainable, readable, and adaptable by making sure it is clearly commented, formatted, and organized. One example of this is the clear separation between hardware control, task scheduling, and data transmission logic. Alongside my code being well documented with comments, the documentation provided clearly explains how each part of my code works. The code could easily be adapted for future enhancements by adding new scheduling intervals for additional tasks. 
