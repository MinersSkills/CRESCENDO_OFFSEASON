
# OFF Season CRESCENDO <strong>Trevitck</strong> - Cuiabá 2024

Esse é o nosso código do Robô do ano de 2024 do OFF SEASON de Cuiabá, Mato Grosso.
É majoritariamente baseado em um YAGSL, o que facilitou muito o desenvolvimento ao longo da temporada


## Overview

### Installation

Vendor URL:

```
https://broncbotz3481.github.io/YAGSL-Lib/yagsl/yagsl.json
```

[Javadocs here](https://broncbotz3481.github.io/YAGSL/)  
[Library here](https://github.com/BroncBotz3481/YAGSL/)  
[Code here](https://github.com/BroncBotz3481/YAGSL/tree/main/swervelib)  
[WIKI](https://github.com/BroncBotz3481/YAGSL/wiki)  
[Config Generation](https://broncbotz3481.github.io/YAGSL-Example/)


## Development

* Development happens here on `YAGSL-Example`. `YAGSL` and `YAGSL-Lib` are updated on a nightly
  basis.

# Support our developers!
<a href='https://ko-fi.com/yagsl' target='_blank'><img height='35' style='border:0px;height:46px;' src='https://az743702.vo.msecnd.net/cdn/kofi3.png?v=0' border='0' alt='Buy Me a Robot at ko-fi.com'></a>

### TL;DR Generate and download your configuration [here](https://broncbotz3481.github.io/YAGSL-Example/) and unzip it so that it follows structure below:

```text
deploy
└── swerve
    ├── controllerproperties.json
    ├── modules
    │   ├── backleft.json
    │   ├── backright.json
    │   ├── frontleft.json
    │   ├── frontright.json
    │   ├── physicalproperties.json
    │   └── pidfproperties.json
    └── swervedrive.json
```

### Then create your SwerveDrive object like this.

```java
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;


SwerveDrive swerveDrive=new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Units.feetToMeters(14.5));
```

```java
double DriveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(WHEEL_DIAMETER), GEAR_RATIO, ENCODER_RESOLUTION);
double SteeringConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(GEAR_RATIO, ENCODER_RESOLUTION);
SwerveDrive swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, SteeringConversionFactor, DriveConversionFactor);
```

### Falcon Support would not have been possible without support from Team 1466 Webb Robotics!

# Configuration Tips

### My Robot Spins around uncontrollably during autonomous or when attempting to set the heading!

* Invert the gyro scope.
* Invert the drive motors for every module. (If front and back become reversed when turning)

 
