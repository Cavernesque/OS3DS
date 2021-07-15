# Open Source 3D Scanner - OS3DS

The Open Source 3D scanner is a project with the aims to develop a low-cost, open source hardware and software 3D LiDAR and SONAR scanner.
This project is an academic research project with MacEwan University, with funding from the Undergraduate Student Research Initiative Grant.

#### The Project Team
The project team consists of: Darren Paetz - Software and Electrical design, Matt Kantor - Mechanical and Software design, and Dr. Philip Mees - Project Mentor.

## The Hardware

The OS3DS has several key hardware components. It has an Arduino Uno for I/O management, the Benewake TF-02 Pro/Mini Plus for LiDAR
data acquisition, 2 5V 28BYJ-48 Stepper motors (64:1) to control its position, and has a 3D printed (PLA and PETG) enclosure
that allows proper cable management and tripod mounting.

## The Software

The OS3DS firmware is written in Arduino's flavor of C++. It is fully software configurable from the included application, which can either
be built directly from the included Python module, or can be run from the included compressed .7z archive. The included Python module provides a GUI that
you can use to send configuration data to the scanner, and also then read the scan data back from the Arduino. It saves scan information in the platform-agnostic
and ASTM standardized E57 format, as well as to a file in ASCII format for manipulation and direct viewing.