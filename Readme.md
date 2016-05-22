# Calibration

Calibration是一个用于多视角相机标定的项目，该项目参考[Beeler10](https://graphics.ethz.ch/publications/papers/paperBee10.php)的工作完成，由于使用到cvsba（由VS2010编译），Calibration项目只可在VS2010中使用/MD或/MDd选项编译。

# 项目依赖

- [cvsba](http://www.uco.es/investiga/grupos/ava/node/39) 已在VS2010下编译成lib，有debug和release版。
- OpenCV动态库，已包含在项目中