The current release (8.0) is ready for flight testing. Here are some notes about the current state of the app.

First Stage Engine Burnout Detection
Currently the app does not have an algorithm to detect first stage engine burnout. This is only important if you decide to 
use the app to ignite a second stage engine whose ignition timing is dependent on detecting first stage burnout.

The settings file should be saved onto the SD card before starting the app. The settings from the SD card are loaded
into the app during startup and control how the app works. Here is an explanation of each of the settings:
  bat1VoltFull: Specifies the voltage level above which the main battery is considered to be fully charged
  bat1VoltLow: Specifies the voltage level below which the main battery is considered to be unacceptable for flight
  bat1VoltOK: Specifies the voltage level above which the main battery is considered ready for flight
  bat2Present: True specifies that the app should assume the "initiator" battery is installed on the rocket
               False specifies that the "initiator" battery is not installed
  bat2VoltFull: Specifies the voltage level above which the initiator battery is considered to be fully charged
  bat2VoltLow: Specifies the voltage level below which the initiator battery is considered to be unacceptable for flight
  bat2VoltOK: Specifies the voltage level above which the initiator battery is considered ready for flight
  drogueParachute: A value of 0 indicates that there is no drogue parachute installed
                   A value of 1, 2, 3, or 4 indicates which initiator terminal the drougue chute initiator is wired to.
  drogueChuteReleaseDelay: Specifies the time in milliseconds from apogee to drogue parachute release.
  parachute: A value of 0 indicates that there is no parachute installed
             A value of 1, 2, 3, or 4 indicates which initiator terminal the parachute initiator is wired to.
  parachuteReleaseDelay: If the drogue chute is installed; specifies the time in milliseconds from drogue chute release to parachute release
                         If there is no drogue chute; specifies the time in milliseconds from apogee to parachute release
  secondStage: A value of 0 indicates that there is no second stage engine installed
               A value of 1, 2, 3, or 4 indicates which initiator terminal the second stage motor initiator is wired to.
  secondStageIgnitionDelay: The time in milliseconds from first stage motor burnout to second stage motor ignition
  firstStageIgnition: A value of 0 indicates that the app will not fire the first stage motor (Currently the app has no logic to fire the first stage motor)
                      A value of 1, 2, 3, or 4 indicates which initiator terminal the first stage motor initiator is wired to.
  flightTimeOut: The time in seconds after launch after which the flight will be assumed to be completed if landing is not detected.
  landedThreshold: The axial rotation in deg/sec below which it will be assumed that the rocket has landed
  launchAccelThreshold: The acceleration in G's above which it will be assumed that the rocket has launched.
  seaLevelPressure: The sea level pressure in hPa used to calibrate the altimeter
  upGoAddr: The integer address of this instance of UpGoV avionics package for LoRa radio addressing purposes. Current only a value of 2 is supported
  groundTest: True will put the app in ground test mode which is used for test on the bench. Currently this bypasses the need for a GPS fix.
              A setting of false should be used for flight testing
