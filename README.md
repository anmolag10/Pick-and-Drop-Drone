
<h1> Pick and Drop Drone </h1>
<p><ul><li>A gazebo simulation of a pick and drop drone was implemented by my team under the theme Vitarana Drone, EYRC-2021. The drone model was provided  EYRC. The drone consists of a gps, imu, laserscanners and a camera.</li> <br> <li>The drone is capable of picking up a box from a given gps coordinates, then scans the QR code on the box of the delivery location, when it reaches to scanned QR codes coordinates, due to inaccurracy it goes into a search pattern and uses Harr Cascade method to detect thedelivery pad. </li><br> <li>A PID has been tuned for stable flight of the drone. </li></ul></p>
<br>
<hr>
<br>
<h2> Scripts </h2>
<p> The repository consists of following scripts:</p>
<ul> <li><bold>attitude_controller:</bold>< A pid script for controlling the roll, pitch and yaw of the drone and publising the commands to the drone./li>
     <li><bold>position_controller: A pid script for controlling the position (in the form of way points)</li>
     <li><bold>detect_logo:</bold> A Harr Cascade model for detecting the landing pad, implemented using openCV</li>
  <li><bold>schedule:</bold> A scheduler script for the delivery and return of the packages</li>
  </ul>
  <br><hr>
     <h2>Simulation Video</h2>
<div align="center">
  <a href="https://youtu.be/G9XYRhGov2A"><img src="MetaCodes/Screenshot from 2021-03-22 00-48-48.png" alt="IMAGE ALT TEXT"></a>
</div>
     <h4>DISCLAIMER</h4>
     <p> All the codes and models are property of EYRC, IIT BOMBAY as the clause of the competition states.</p>
