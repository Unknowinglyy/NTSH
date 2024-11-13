# CSCE-462
For labs!!


# Ballance

code 0 = ABS_X
code 1 = ABS_Y

code 53 = ABS_MT_POSITION_X
code 54 = ABS_MT_POSITION_Y
code 57 = ABS_MT_TRACKING_ID

code 330 = BTN_TOUCH

To disable touchscreen from being registered as mouse input:
in the CLI type: 
xinput
xinput --disable <touchscreen ID>

```

    --------------------------------
    |(3800, 150)       (3800, 3940)|
    |                              |
    |                              |
====|          (2025,2045)         |
    |                              |
 ^  |                              |
 |  |(250, 150)         (250, 3940)|
+x  --------------------------------  
+y ->    

```

# Motor Assignments
Motor 1 : GPIO 23,24 (3685,200)
Motor 2 : GPIO 20,21 (3685,3820)
Motor 3 : GPIO 5,6 (390,1810)

# Resources / Citations
https://github.com/mattzzw/Arduino-mpu6050

Touchscreen: https://www.amazon.com/dp/B07TZGVY8K?ref=ppx_yo2ov_dt_b_fed_asin_title

we have the motors at 0.67 A and 0.94 as the Vref



