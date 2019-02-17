In order to set up limelight we need to setup static IP for Limelight which is already done. The static IP for Limelight is 10.2.63.33
To access limelight web page, we can use http://10.2.63.33:5801
To view the camera stream, we can use http://10.2.63.33:5800
We can also use http://limelight.local:5801 to view the page
If connect the usb camera to limelight usb port, we need to restart
camMode     Sets limelight’s operation mode
0     Vision processor
1     Driver Camera (Increases exposure, disables vision processing)
Java sample code to access network table
NetworkTableInstance.getDefault().getTable("limelight").getEntry("").getDouble(0);
