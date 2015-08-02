part of [camodocal](https://github.com/hengli/camodocal)

[Google Ceres](http://ceres-solver.org) is needed.

# Calibration:

Use intrinsic_calib.cc to calibrate your camera.

# Undistortion:

See Camera.h for general interface: 

 - liftProjective: Lift points from the image plane to the projective space.
 - spaceToPlane: Projects 3D points to the image plane (Pi function)

