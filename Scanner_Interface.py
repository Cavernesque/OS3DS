"""A module that creates the GUI application for the Open Source Scanner.

Open-Source 3D Scanner Project
Created by: Darren Paetz, Matt Kantor, Dr. Philip Mees
MacEwan University, Edmonton, Alberta, Canada, 2021

This application is designed to be used with an Arduino-based 3D scanner.
The firmware running on the Arduino will communicate with this application,
allowing the application to convert sensor data into an ASTM standard 3D-scan
file format (E57).

This project was made possible with the following funding:
    MacEwan University's Undergraduate Student Research Initiative Grant (USRI)

"""

import sys
import pye57
import numpy as np
import math
import serial
import serial.tools.list_ports
from time import sleep
from datetime import datetime
from PyQt5 import QtWidgets, uic
import uuid

from typing import Dict
from pye57.__version__ import __version__
from pye57 import libe57

qtcreator_file = "3D_Scan_App_UI.ui"  # Load the ui file
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtcreator_file)

SUPPORTED_POINT_FIELDS = {
    "cartesianX": "d",
    "cartesianY": "d",
    "cartesianZ": "d",
    "intensity": "f",
    "colorRed": "B",
    "colorGreen": "B",
    "colorBlue": "B",
    "rowIndex": "H",
    "columnIndex": "H",
    "cartesianInvalidState": "b",
}


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    """Application GUI object.

    Inherits from QtWidgets.QMainWindow.

    """

    def __init__(self):
        """Initialize the application window object.

        Connect the signal and slots of the various objects within the GUI.
        Initialize the scan points and scan time labels with zeros.

        Returns
        -------
        None.

        """
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)

        # Initialize the properties of scan points and estimated time
        self.scan_point_count = 0
        self.scan_time_estimate = 0

        # Serial communication flag, put serial object in namespace
        self.connected_to_arduino = False
        self.arduino = None
        self.abort_scan = False

        # Lookup table for resolution multipliers
        # Ex Fine, Very Fine, Fine, Medium, Coarse, Very Coarse
        self.resolution_options = [4, 2, 1, 0.5, 0.25, 0.1]

        # Connect the signal slots for the file menu
        self.actionExit_2.triggered.connect(self.close)
        self.actionOpen_Settings_File.triggered.connect(self.loadSettings)
        self.actionSave_Settings_File.triggered.connect(self.saveSettings)
        self.actionAbout.triggered.connect(self.aboutDialog)

        # Initialize the com ports available to connect to
        self.refcomButton.clicked.connect(self.getSerialPorts)

        # Connect all combo boxes to the logic that calculates scan time
        # and number of scan points
        self.angleBox.activated.connect(self.calcScanProperties)
        self.resolutionBox.activated.connect(self.calcScanProperties)

        # Connect the browse for filename button to an open filename dialog
        self.outputfileButton.clicked.connect(self.getE57Filename)

        # Connect the scanner buttons to their respective methods
        self.connectButton.clicked.connect(self.connectToScanner)
        self.scanButton.clicked.connect(self.scan)
        self.abortScanButton.clicked.connect(self.abortScan)

        # Set the scan information with preliminary settings
        self.calcScanProperties()

        # Connect the jog buttons to the jog motor method
        self.jogscannerUp.clicked.connect(
                lambda: self.jogDirection("up"))
        self.jogscannerDown.clicked.connect(
                lambda: self.jogDirection("down"))
        self.jogscannerLeft.clicked.connect(
                lambda: self.jogDirection("left"))
        self.jogscannerRight.clicked.connect(
                lambda: self.jogDirection("right"))

    """
    --------------------------------------------------------------
                File menu methods
    --------------------------------------------------------------
    """

    def loadSettings(self):
        """Load saved settings for the application from a CSV file.

        Returns
        -------
        None.

        """
        options = QtWidgets.QFileDialog.Options()
        # Get the filename of the settings file
        # Discard the filetype that the method returns
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open a scanner configuration file", "",
            "Scanner Configuration Files (*.3dscancfg);;All Files (*)",
            options=options)
        if fileName != '':
            with open(fileName, "r") as fh:
                data = fh.readline()
                data = data.split(",")
                self.sensorBox.setCurrentIndex(int(data[0]))
                self.angleBox.setCurrentIndex(int(data[1]))
                self.highAngleBox.setValue(int(data[2]))
                self.lowAngleBox.setValue(int(data[3]))
                self.resolutionBox.setCurrentIndex(int(data[4]))
                self.datavalidationBox.setChecked(bool(data[5]))
                self.pointtrimmingBox.setChecked(bool(data[6]))
            self.calcScanProperties()

    def saveSettings(self):
        """Save current settings for the application to a CSV file.

        Returns
        -------
        None.

        """
        options = QtWidgets.QFileDialog.Options()
        fileName, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save scanner configuration to file", "",
            "Scanner Configuration Files (*.3dscancfg);;All Files (*)",
            options=options)
        if fileName != '':
            with open(fileName, "w") as fh:
                fh.write(repr(self.sensorBox.currentIndex()) + ",")
                fh.write(repr(self.angleBox.currentIndex()) + ",")
                fh.write(repr(self.highAngleBox.value()) + ",")
                fh.write(repr(self.lowAngleBox.value()) + ",")
                fh.write(repr(self.resolutionBox.currentIndex()) + ",")
                fh.write(repr(self.datavalidationBox.isChecked()))
                fh.write(repr(self.pointtrimmingBox.isChecked()))

    def aboutDialog(self):
        """Open the About dialog box.

        Returns
        -------
        None.

        """
        mesg = """Open Source 3D Scanning Utility \n
        This utility will allow you to connect to the OSS 3D Scanner, upload
        configurations, and then retrieve data from the scanner.\n
        Developed by Darren Paetz, Matt Kantor, Dr. Philip Mees
        MacEwan University, Edmonton, Alberta, Canada, 2021
        """
        QtWidgets.QMessageBox.about(
            self, "About the 3D Scanning Utility", mesg)

    """
    --------------------------------------------------------------
                Main window methods
    --------------------------------------------------------------
    """

    def getSerialPorts(self):
        """Add the available COM ports to the COM port combobox.

        Returns
        -------
        None.

        """
        self.comBox.clear()
        self.comBox.addItem("---")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.comBox.addItem(port.device)

    def calcScanProperties(self):
        """Change the value of both the scan time and scan points labels.

        Returns
        -------
        None.

        """
        # TODO: Re-time the position change time with the reduced duty cycle
        pos_change_time = 0.125  # Time is in seconds
        vertical_angle = self.lowAngleBox.value() - self.highAngleBox.value()
        scan_angles = [360, 270, 180, 135, 90, 45, 22, 10]
        angle = scan_angles[self.angleBox.currentIndex()]
        resolution = self.resolution_options[self.resolutionBox.currentIndex()]
        vert_res = round(vertical_angle*resolution)
        hor_res = round(angle*resolution)
        points = int(vert_res*hor_res)
        # TODO: This estimate needs to be updated when trimming is fully done
        scan_time_estimate = round(points*pos_change_time/60, 2)
        self.scan_point_count = points
        self.scan_time_estimate = scan_time_estimate
        self.scanpoints_outLabel.setText(str(points))
        if scan_time_estimate < 1.0:
            self.scantime_outLabel.setText(
                str(round(scan_time_estimate*60, 2))
                + " seconds")
        elif scan_time_estimate < 90:
            self.scantime_outLabel.setText(
                str(scan_time_estimate)
                + " minutes")
        else:
            self.scantime_outLabel.setText(
                str(round(scan_time_estimate/60, 2))
                + " hours")

    def getE57Filename(self):
        """Open file dialog and use selected filename as plaintext box text.

        Returns
        -------
        None.

        """
        options = QtWidgets.QFileDialog.Options()
        default_fill = ""
        if self.sensorBox.currentIndex:
            sensortext = "LiDAR"
        else:
            sensortext = "Sonar"
        angletext = self.angleBox.currentText()[:-1]
        restext = self.resolutionBox.currentText()
        moddate = datetime.now().strftime("%y%m%d-%H%M-")
        default_fill += moddate + sensortext + "-" + angletext + "-" + restext
        fileName, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Select a location for the E57 file", default_fill,
            "E57 File (*.e57)", options=options)
        self.outputfileEdit.setPlainText(fileName)

    def settingsScanState(self, state):
        """Change all form objects' enabled property during a scan event.

        To be used during an active scan. Will enable the Abort scan button,
        but disable all other widgets.

        Parameters
        ----------
        state : Boolean
            True -> Enabled
            False -> Disabled

        Returns
        -------
        None.

        """
        self.comBox.setEnabled(state)
        self.refcomButton.setEnabled(state)
        self.sensorBox.setEnabled(state)
        self.angleBox.setEnabled(state)
        self.highAngleBox.setEnabled(state)
        self.lowAngleBox.setEnabled(state)
        self.resolutionBox.setEnabled(state)
        self.outputfileEdit.setEnabled(state)
        self.outputfileButton.setEnabled(state)
        self.datavalidationBox.setEnabled(state)
        self.connectButton.setEnabled(state)
        self.jogMode(state)
        self.abortScanButton.setEnabled(not state)

    def jogMode(self, state):
        """Control the jog mode function of the application.

        Parameters
        ----------
        state : Boolean
            True -> Enabled
            False -> Disabled

        Returns
        -------
        None.

        """
        self.jogscannerUp.setEnabled(state)
        self.jogscannerDown.setEnabled(state)
        self.jogscannerLeft.setEnabled(state)
        self.jogscannerRight.setEnabled(state)

    def jogDirection(self, direction):
        """Write serial message to scanner containing jog direction."""
        self.arduino.write(bytes("B", encoding="ASCII"))
        sleep(0.05)
        if direction == "up":
            self.arduino.write(bytes("u", encoding="ASCII"))
        if direction == "down":
            self.arduino.write(bytes("d", encoding="ASCII"))
        if direction == "left":
            self.arduino.write(bytes("l", encoding="ASCII"))
        if direction == "right":
            self.arduino.write(bytes("r", encoding="ASCII"))

    """
    --------------------------------------------------------------
                Connection and communication methods
    --------------------------------------------------------------
    """

    def connectToScanner(self):
        """Attempt to open a serial (COM) connection to the scanner.

        Returns
        -------
        None.

        """
        portname = self.comBox.currentText()
        if self.connected_to_arduino:
            self.connectFdbkLabel.setText("Already connected to scanner.")
            return
        if self.outputfileEdit.toPlainText() != '':
            try:
                self.arduino = serial.Serial(port=portname, baudrate=115200,
                                             timeout=3)
                self.connectFdbkLabel.setText("Successfully connected.")
                self.connected_to_arduino = True
                # If the scan button is hit immediately after connect, an
                # error will be generated. Simply pause the application for
                # one second before passing control back to user.
                sleep(1)
                self.jogMode(True)
                self.scanButton.setEnabled(True)
            except serial.SerialException:
                self.connectFdbkLabel.setText(
                    """Error: Could not connect to scanner.""")
                self.connected_to_arduino = False
        else:
            self.connectFdbkLabel.setText(
                """Please choose a destination for the output file first.""")
        return

    def sendConfiguration(self):
        """Push scan configuration to Arduino.

        Returns
        -------
        bool
            True -> Configuration valid and accepted.
            False -> Configuration invalid or not accepted.

        """
        sensor = str(self.sensorBox.currentIndex())
        horiz_angle = str(self.angleBox.currentText()[:-1])
        vert_hi_angle = str(self.highAngleBox.value())
        vert_lo_angle = str(self.lowAngleBox.value())
        # Ex Fine, Very Fine, Fine, Medium, Coarse, Very Coarse
        resolution = str(
            self.resolution_options[self.resolutionBox.currentIndex()])
        validate = str(int(self.datavalidationBox.isChecked()))
        trimming = str(int(self.pointtrimmingBox.isChecked()))
        # Send configuration instruction code
        self.arduino.write(bytes("A", encoding="ASCII"))
        sleep(1)
        # Push the configuration to scanner
        self.arduino.write(bytes(sensor + '\n', encoding="ASCII"))
        sleep(0.01)
        self.arduino.write(bytes(horiz_angle + '\n', encoding="ASCII"))
        sleep(0.01)
        self.arduino.write(bytes(vert_hi_angle + '\n', encoding="ASCII"))
        sleep(0.01)
        self.arduino.write(bytes(vert_lo_angle + '\n', encoding="ASCII"))
        sleep(0.01)
        self.arduino.write(bytes(resolution + '\n', encoding="ASCII"))
        sleep(0.01)
        self.arduino.write(bytes(validate + '\n', encoding="ASCII"))
        sleep(0.01)
        self.arduino.write(bytes(trimming + '\n', encoding="ASCII"))
        total_bytes = (len(sensor)
                       + len(horiz_angle)
                       + len(vert_hi_angle)
                       + len(vert_lo_angle)
                       + len(validate)
                       + len(trimming)
                       + 5)  # commas and trim /r/n
        # Read the echo
        sleep(0.01)
        response = self.arduino.read(size=total_bytes)
        response = response.decode(encoding='UTF-8')
        parsed = response.split(",")
        print("Configuration frame from scanner: " + repr(parsed))
        # If the response matches the transmission, give clearance for scan.
        # Don't check the resolution, floating point comparison is meaningless.
        if((parsed[0] == sensor) and (parsed[1] == horiz_angle)
            and (parsed[2] == vert_hi_angle) and (parsed[3] == vert_lo_angle)
                and (parsed[4] == validate) and (parsed[5] == trimming)):
            self.arduino.write(bytes("scan\n", "ascii"))
            print("Succesfully configured.")
            return True
        return False

    def scan(self):
        """Gather and process scan data from Arduino. Update window as needed.

        This method performs the following actions:

        Configuration
        -------------
        Pushes the current settings as a configuration frame to the scanner.
        If the scanner does not successfully read back the settings, close the
        connection. If the scanner accepts the settings, proceed to scan.

        Scan
        ----
        Reads data from the scanner over serial connection. If the serial
        connection times out, method retries until serial connection is
        re-established. If there is an exception on the serial connection, the
        data that has been gathered up until that point will be written to
        file. The user has the option to abort the scan in progress. If the
        scan is aborted, the data gathered up until that point will be written
        to file. Once the amount of points required for the scan are read, the
        data is written to file.

        Write to E57 File
        -----------------
        Once whatever data is collected and is ready for writing to file, it
        first must be processed. scan() calls the processPolar() method which
        will convert the polar coordinate data that the scanner outputs to more
        "human-legible" Cartesian coordinates. Once the data is converted,
        scan() utilizes the E57Cust class, which has a writer method that will
        create the E57 file.

        Returns
        -------
        None.

        """
        # Clear the output and update the widget
        self.outputText.clear()
        self.scanButton.setEnabled(False)
        # Disable interaction with software while scan is taking place
        # The exception is the Abort Scan button
        self.settingsScanState(False)
        QtWidgets.QApplication.processEvents()
        # Initialize all the scan variables
        # Set a zero at the beginning of every scan to give a reference
        # when viewing the raw point cloud
        distance = [0]
        intensity = [0]
        theta = [0]
        phi = [0]
        points_scanned = 0
        outstr = ""
        scan_complete = False
        valid_fails = 0
        self.scan_start_time = datetime.now()
        fileName = self.outputfileEdit.toPlainText()
        # Open the file where the data will be stored during the scan.
        tmp_file = open(
            (fileName[:-4] + "_scandata.txt"), mode='w', encoding='utf-8')
        # Send the configuration data to the Arduino
        # On failure, cancel scan
        scanner_configured = self.sendConfiguration()
        if not scanner_configured:
            self.setScanResult(False, cfg=False)
            return
        # Purge the serial buffer before we read data from the scanner
        self.arduino.reset_input_buffer()
        # Send the scan command
        self.arduino.write(bytes("C", encoding="ASCII"))
        # Obtain scan information from the Arduino
        while not scan_complete:
            try:
                data = self.arduino.readline()
                # Strip CRLF
                data = data.decode('utf8')[:-2]
                parsed = data.split(",")
                print(repr(parsed))
                if parsed[0] == "complete":
                    scan_complete = True
                    valid_fails = int(parsed[1])
                else:
                    # TODO: Should be writing these values to an interim file
                    #       then getting them from that file
                    #       and pushing afterwards
                    # Parse and separate the sent parameters from the Arduino
                    # Data frame is structured as:
                    # DATA: [Distance, Itensity,
                    #       Horizontal stepper angle (theta),
                    #       Vertical stepper angle (phi), Checksum]
                    # TYPE: [unsigned int, unsigned int, float,
                    #       float, unsigned long]
                    # UNIT: [mm], [value], [angle°], [angle°], [integer]
                    distance.append(float(parsed[0]))
                    intensity.append(float(parsed[1]))
                    theta.append(float(parsed[2]))
                    phi.append(float(parsed[3]))
                    # Accumulate total point count
                    points_scanned += 1
                    # Write the scan data we received to a file
                    # to preserve it in case of scanner failure
                    tmp_file.write(repr(parsed[:-1]) + '\n')
            # If at any point, a serial exception is generated, abort the scan
            # This operation will preserve any scan data acquired
            except serial.SerialException:
                self.setScanResult(False, points=points_scanned)
                self.processAndExport(
                    fileName, distance, intensity, theta, phi)
                tmp_file.close()
                return
            # If the serial connection times out, keep trying
            except serial.SerialTimeoutException:
                print("Timeout!")
                pass
            # Arduino passed bad data, set at origin
            except ValueError:
                distance.append(0)
                intensity.append(0)
                theta.append(0)
                phi.append(0)
                pass
            else:
                # If the scan got aborted by the user, break out from scan
                if self.abort_scan:
                    self.setScanResult(
                        False, points=points_scanned, usr_abt=True)
                    self.processAndExport(
                        fileName, distance, intensity, theta, phi)
                    tmp_file.close()
                    return
                # Print points completed in divisions of the modulus
                # Update the progress bar
                if (points_scanned % 10) == 0:
                    outstr = str(points_scanned) + " points scanned...\n"
                    self.outputText.setText(outstr)
                    progress = int((points_scanned/self.scan_point_count)*100)
                    self.progressBar.setValue(progress)
                # Update the application screen
                # TODO: Should be in separate thread?
                # QtWidgets.QApplication.processEvents()
        # Print the final message to the output
        tmp_file.close()
        self.setScanResult(True, points=points_scanned, failedpts=valid_fails)
        self.processAndExport(fileName, distance, intensity, theta, phi)
        return

    def abortScan(self):
        """Set the abort scan flag."""
        self.abort_scan = True

    def setScanResult(self, result_success=False, points=0, failedpts=0,
                      usr_abt=False, cfg=True):
        """Update the form objects with scan progress and result.

        Will provide a feedback message with some basic scan statistics
        to the user.

        Parameters
        ----------
        result_success : Boolean, optional
            True -> Scan complete.
            False -> Scan incomplete.
        points : Integer, optional
            Number of points completed in scan.
        usr_abt : Boolean, optional
            True -> Scan aborted by user.
            False -> Scan not aborted by user.
        cfg : Boolean, optional
            True -> Scanner had been successfully configured.
            False -> Scanner had not been successfully configured.

        Returns
        -------
        None.

        """
        self.arduino.close()
        self.connected_to_arduino = False
        self.connectFdbkLabel.setText("Reconnect to begin another scan.")
        self.scan_fin_time = datetime.now()
        self.scan_completion_time = str(self.scan_fin_time
                                        - self.scan_start_time)[:-7]
        scan_stats = ("SCAN STATISTICS:\n"
                      + "----------------\n"
                      + str(points)
                      + " points collected.\n"
                      + str(failedpts)
                      + " points failed data validation.\n"
                      + "Scan time: "
                      + self.scan_completion_time
                      + "\nAn E57 file has been saved to the directory:\n"
                      + self.outputfileEdit.toPlainText()
                      + "\n\nConnection to scanner closed.")
        if result_success:
            self.progressBar.setValue(100)
            self.outputfileEdit.setPlainText('')
            self.outputText.setText(
                "SCAN COMPLETE.\n\n"
                + scan_stats)
        else:
            self.outputText.clear()
            self.progressBar.reset()
            if usr_abt:
                self.outputText.setText(
                    "SCAN ABORTED.\n\n"
                    + "The current scan data has been saved to file.\n\n"
                    + scan_stats)
                self.progressBar.reset()
                self.abort_scan = False
            elif not cfg:
                self.outputText.setText(
                    "ERROR!\n\n"
                    + "The scanner has not been successfully configured.\n"
                    + "Please attempt to upload configuration again\n"
                    + "by pressing the \'CONNECT\' button.")
            else:
                self.outputText.setText(
                    "ERROR!\n\n"
                    + "An unexpected communication error has occurred.\n"
                    + "The current scan data has been saved to file.\n\n"
                    + scan_stats)
        self.settingsScanState(True)  # Re-enable the settings
        self.jogMode(False)  # Turn off jog mode
        QtWidgets.QApplication.processEvents()

    """
    --------------------------------------------------------------
                Data processing and export methods
    --------------------------------------------------------------
    """

    def processPolar(self, distance, theta, phi):
        """Convert from polar coordinates to Cartesian coordinates.

        Parameters
        ----------
        distance : List [Integer,Integer,...]
            An ordered list containing all distances obtained in scan.
        theta : List [Integer,Integer,...]
            An ordered list containing all angles theta obtained in scan.
        phi : List [Integer,Integer,...]
            An ordered list containing all angles phi obtained in scan.

        Returns
        -------
        x : List [Integer,Integer,...]
            An ordered list containing all Cartesian x coordinates of scan.
        y : List [Integer,Integer,...]
            An ordered list containing all Cartesian y coordinates of scan.
        z : List [Integer,Integer,...]
            An ordered list containing all Cartesian z coordinates of scan.

        """
        x = []
        y = []
        z = []
        theta_rad = []
        phi_rad = []
        # Convert all angles to radians
        for angle in theta:
            rad_angle = math.radians(angle)
            theta_rad.append(rad_angle)
        for angle in phi:
            rad_angle = math.radians(angle)
            phi_rad.append(rad_angle)
        i = 0
        # Do polar conversion
        for radius in distance:
            x.append(radius*math.sin(phi_rad[i])*math.cos(theta_rad[i]))
            y.append(radius*math.sin(phi_rad[i])*math.sin(theta_rad[i]))
            z.append(radius*math.cos(phi_rad[i]))
            i += 1
        return x, y, z

    def exportE57(self, fileName, cartX, cartY, cartZ, intensity=None):
        """Export the scan data to E57 file.

        Parameters
        ----------
        fileName : String
            The destination for the E57 file.
        cartX : Numpy.array
            A Numpy array of the cartesian x coordinates of scan data.
        cartY : TYPE
            A Numpy array of the cartesian y coordinates of scan data.
        cartZ : TYPE
            A Numpy array of the cartesian z coordinates of scan data.

        Returns
        -------
        None.

        """
        # Create numpy arrays of int32
        cartesianX = np.array(cartX)
        cartesianY = np.array(cartY)
        cartesianZ = np.array(cartZ)
        # If intensity data not generated (using sonar sensor), set as zeros
        if intensity is not None:
            # If we have valid intensity data, set the zero coordinate's
            # intensity value to the smallest measured value
            intensity[0] = min(intensity[1:])
            intensity = np.array(intensity)
        else:
            intensity = np.zeros_like(cartesianX, dtype="intc")
        cartesianInvalidState = np.zeros_like(cartesianX, dtype="intc")
        # Get the name for E57 file from app window
        # If it's blank, set to default name
        if self.e57NameEdit.toPlainText() != '':
            export_file_name = self.e57NameEdit.toPlainText()
        else:
            export_file_name = "OS3DS Cloud"
        # Set the dictionary that we will pass to the writer
        data = {
            "cartesianInvalidState": cartesianInvalidState,
            "cartesianX": cartesianX,
            "cartesianY": cartesianY,
            "cartesianZ": cartesianZ,
            "intensity": intensity
            }
        # Instantiate the writer
        writefile = E57Cust(fileName, mode='w')
        try:
            writefile.write_scan_raw(data, name=export_file_name)
        # TODO: Exception handling - what happens when the file write fails?
        finally:
            writefile.close()

    def processAndExport(self, fileName, distance, intensity, theta, phi):
        """Block for processing and export.

        This method combines the processPolar and exportE57 methods for
        more simple re-use through the scan method.

        Parameters
        ----------
        fileName : String
            The destination for the E57 file.
        distance : List [Integer,Integer,...]
            An ordered list containing all distances obtained in scan.
        theta : List [Integer,Integer,...]
            An ordered list containing all angles theta obtained in scan.
        phi : List [Integer,Integer,...]
            An ordered list containing all angles phi obtained in scan.

        Returns
        -------
        None.

        """
        # TODO: Add in checks to ensure data is present.
        #       Or read from file.
        # Convert the data from the Arduino from spherical to cartesian
        cartX, cartY, cartZ = self.processPolar(distance, theta, phi)
        # Export all cartesian data as E57 file
        self.exportE57(fileName, cartX, cartY, cartZ, intensity)
        return


class E57Cust(pye57.E57):
    """Custom E57 class with overwritten writer method.

    Inherits from the E57 class in pye57.
    write_scan_raw() method has been implemented so that scan_header is not
    unnecessarily bound by header properties such as rotation and pose.

    """

    def write_default_header(self):
        """Write the header information that comes before the scan data."""
        imf = self.image_file
        imf.extensionsAdd("", libe57.E57_V1_0_URI)
        self.root.set(
            "formatName",
            libe57.StringNode(imf, "ASTM E57 3D Imaging Data File"))
        self.root.set(
            "guid",
            libe57.StringNode(imf, "{%s}" % uuid.uuid4()))
        self.root.set(
            "versionMajor",
            libe57.IntegerNode(imf, libe57.E57_FORMAT_MAJOR))
        self.root.set(
            "versionMinor",
            libe57.IntegerNode(imf, libe57.E57_FORMAT_MINOR))
        self.root.set(
            "e57LibraryVersion",
            libe57.StringNode(imf, libe57.E57_LIBRARY_ID))
        self.root.set(
            "coordinateMetadata",
            libe57.StringNode(imf, ""))
        creation_date_time = libe57.StructureNode(imf)
        creation_date_time.set(
            "dateTimeValue",
            libe57.FloatNode(imf, datetime.now().timestamp()))
        creation_date_time.set(
            "isAtomicClockReferenced",
            libe57.IntegerNode(imf, 0))
        self.root.set(
            "creationDateTime",
            creation_date_time)
        self.root.set(
            "data3D",
            libe57.VectorNode(imf, True))
        self.root.set(
            "images2D",
            libe57.VectorNode(imf, True))

    def write_scan_raw(self, data: Dict, *, name=None, scan_header=None):
        """Write raw scan data to file using C++ library libe57.

        Modified from original class so as to not require certain scan header
        properties and parameters.

        Parameters
        ----------
        data : Dict
            DESCRIPTION.
        name : String, optional, keyword argument mandatory
            DESCRIPTION. The default is None.
        scan_header : scanHeader, optional, keyword argument mandatory
            DESCRIPTION. The default is None.

        Raises
        ------
        ValueError
            DESCRIPTION.

        Returns
        -------
        None.

        """
        for field in data.keys():
            if field not in SUPPORTED_POINT_FIELDS:
                raise ValueError("Unsupported point field: %s" % field)

        if scan_header is None:
            scan_header = self.image_file.root()

        # temperature = getattr(scan_header, "temperature", 0)

        scan_node = libe57.StructureNode(self.image_file)
        scan_node.set("guid", libe57.StringNode(
            self.image_file, "{%s}" % uuid.uuid4()))
        scan_node.set("name", libe57.StringNode(self.image_file, name))
        scan_node.set("description",
                      libe57.StringNode(self.image_file,
                                        "pye57 v%s" % __version__))

        # Count the number of points in the scan by looking at how large
        # the x entry is in the data dictionary
        n_points = data["cartesianX"].shape[0]

        # Set up the index bounds structure
        ibox = libe57.StructureNode(self.image_file)
        if "rowIndex" in data and "columnIndex" in data:
            min_row = np.min(data["rowIndex"])
            max_row = np.max(data["rowIndex"])
            min_col = np.min(data["columnIndex"])
            max_col = np.max(data["columnIndex"])
            ibox.set("rowMinimum",
                     libe57.IntegerNode(self.image_file, min_row))
            ibox.set("rowMaximum",
                     libe57.IntegerNode(self.image_file, max_row))
            ibox.set("columnMinimum",
                     libe57.IntegerNode(self.image_file, min_col))
            ibox.set("columnMaximum",
                     libe57.IntegerNode(self.image_file, max_col))
        else:
            ibox.set("rowMinimum", libe57.IntegerNode(self.image_file, 0))
            ibox.set("rowMaximum",
                     libe57.IntegerNode(self.image_file, n_points - 1))
            ibox.set("columnMinimum", libe57.IntegerNode(self.image_file, 0))
            ibox.set("columnMaximum", libe57.IntegerNode(self.image_file, 0))
        ibox.set("returnMinimum", libe57.IntegerNode(self.image_file, 0))
        ibox.set("returnMaximum", libe57.IntegerNode(self.image_file, 0))

        # Don't bother setting it for now, we're not going to use it
        # scan_node.set("indexBounds", ibox)

        # Look in the data dictionary for the key "intensity"
        # If it is in there, look in the scan header
        # for the properties of intensity and set the
        # intensity box appropriately

        if "intensity" in data:
            int_min = getattr(scan_header, "intensityMinimum",
                              np.min(data["intensity"]))
            int_max = getattr(scan_header, "intensityMaximum",
                              np.max(data["intensity"]))
            intbox = libe57.StructureNode(self.image_file)
            intbox.set("intensityMinimum",
                       libe57.FloatNode(self.image_file, int_min))
            intbox.set("intensityMaximum",
                       libe57.FloatNode(self.image_file, int_max))
            scan_node.set("intensityLimits", intbox)

        # Look for various color parameters in the data array
        # Set the colorbox with their minimum and maximum
        color = all(c in data for c in ["colorRed", "colorGreen", "colorBlue"])
        if color:
            colorbox = libe57.StructureNode(self.image_file)
            colorbox.set("colorRedMinimum",
                         libe57.IntegerNode(self.image_file, 0))
            colorbox.set("colorRedMaximum",
                         libe57.IntegerNode(self.image_file, 255))
            colorbox.set("colorGreenMinimum",
                         libe57.IntegerNode(self.image_file, 0))
            colorbox.set("colorGreenMaximum",
                         libe57.IntegerNode(self.image_file, 255))
            colorbox.set("colorBlueMinimum",
                         libe57.IntegerNode(self.image_file, 0))
            colorbox.set("colorBlueMaximum",
                         libe57.IntegerNode(self.image_file, 255))
            scan_node.set("colorLimits", colorbox)

        # Instantiate the structure node for the bounding box
        bbox_node = libe57.StructureNode(self.image_file)
        # Get the various cartesian elements from the data
        # dictionary passed to function
        x, y, z = data["cartesianX"], data["cartesianY"], data["cartesianZ"]
        # Validate the data, comparing to the cartesianInvalidState key
        valid = None
        if "cartesianInvalidState" in data:
            # Get an array of the valid entries by
            # taking the complement of invalid
            valid = ~data["cartesianInvalidState"].astype("?")
            x, y, z = x[valid], y[valid], z[valid]
        # Store minimum respective values
        bb_min = np.array([x.min(), y.min(), z.min()])
        bb_max = np.array([x.max(), y.max(), z.max()])
        # Clear the namespace
        del valid, x, y, z

        bbox_node.set("xMinimum", libe57.FloatNode(self.image_file, bb_min[0]))
        bbox_node.set("xMaximum", libe57.FloatNode(self.image_file, bb_max[0]))
        bbox_node.set("yMinimum", libe57.FloatNode(self.image_file, bb_min[1]))
        bbox_node.set("yMaximum", libe57.FloatNode(self.image_file, bb_max[1]))
        bbox_node.set("zMinimum", libe57.FloatNode(self.image_file, bb_min[2]))
        bbox_node.set("zMaximum", libe57.FloatNode(self.image_file, bb_max[2]))
        # Write the cartesian bounds into the new structure
        scan_node.set("cartesianBounds", bbox_node)

        points_prototype = libe57.StructureNode(self.image_file)

        is_scaled = False
        precision = libe57.E57_DOUBLE if is_scaled else libe57.E57_SINGLE

        center = (bb_max + bb_min) / 2

        chunk_size = 5000000

        x_node = libe57.FloatNode(self.image_file, center[0],
                                  precision, bb_min[0], bb_max[0])
        y_node = libe57.FloatNode(self.image_file, center[1],
                                  precision, bb_min[1], bb_max[1])
        z_node = libe57.FloatNode(self.image_file, center[2],
                                  precision, bb_min[2], bb_max[2])
        points_prototype.set("cartesianX", x_node)
        points_prototype.set("cartesianY", y_node)
        points_prototype.set("cartesianZ", z_node)

        field_names = ["cartesianX", "cartesianY", "cartesianZ"]

        if "intensity" in data:
            intensity_min = np.min(data["intensity"])
            intensity_max = np.max(data["intensity"])
            intensity_node = libe57.FloatNode(self.image_file,
                                              intensity_min, precision,
                                              intensity_min, intensity_max)
            points_prototype.set("intensity", intensity_node)
            field_names.append("intensity")

        if all(color in data for color in
               ["colorRed", "colorGreen", "colorBlue"]):
            points_prototype.set("colorRed",
                                 libe57.IntegerNode(
                                     self.image_file, 0, 0, 255))
            points_prototype.set("colorGreen",
                                 libe57.IntegerNode(
                                     self.image_file, 0, 0, 255))
            points_prototype.set("colorBlue",
                                 libe57.IntegerNode(
                                     self.image_file, 0, 0, 255))
            field_names.append("colorRed")
            field_names.append("colorGreen")
            field_names.append("colorBlue")

        if "rowIndex" in data and "columnIndex" in data:
            min_row = np.min(data["rowIndex"])
            max_row = np.max(data["rowIndex"])
            min_col = np.min(data["columnIndex"])
            max_col = np.max(data["columnIndex"])
            points_prototype.set("rowIndex",
                                 libe57.IntegerNode(self.image_file, 0,
                                                    min_row, max_row))
            field_names.append("rowIndex")
            points_prototype.set("columnIndex",
                                 libe57.IntegerNode(self.image_file, 0,
                                                    min_col, max_col))
            field_names.append("columnIndex")

        if "cartesianInvalidState" in data:
            min_state = np.min(data["cartesianInvalidState"])
            max_state = np.max(data["cartesianInvalidState"])
            points_prototype.set("cartesianInvalidState",
                                 libe57.IntegerNode(self.image_file, 0,
                                                    min_state, max_state))
            field_names.append("cartesianInvalidState")

        """
        other possible fields for points
            "sphericalRange"
            "sphericalAzimuth"
            "sphericalElevation"
            "timeStamp"
            "sphericalInvalidState"
            "isColorInvalid"
            "isIntensityInvalid"
            "isTimeStampInvalid"
        """

        arrays, buffers = self.make_buffers(field_names, chunk_size)

        codecs = libe57.VectorNode(self.image_file, True)
        points = libe57.CompressedVectorNode(
            self.image_file, points_prototype, codecs)
        scan_node.set("points", points)

        self.data3d.append(scan_node)

        writer = points.writer(buffers)

        current_index = 0
        while current_index != n_points:
            current_chunk = min(n_points - current_index, chunk_size)

            for type_ in SUPPORTED_POINT_FIELDS:
                if type_ in arrays:
                    arrays[type_][:current_chunk] = data[type_][
                        current_index:current_index + current_chunk]

            writer.write(current_chunk)

            current_index += current_chunk

        writer.close()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
