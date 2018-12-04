# MSDF(multi-sensor data fusion) Requirements for simulator

## 1. Coordinates Agreements

### 1.1 World coordinates
In the world frame, the x/y/z axes point to East/North/Up (ENU).
The definition of the heading of the car, used below, is permanently fixed: 0 at East, pi/2 at North, -pi/2 at South.

### 1.2 Ego vehicle coordinates
Respect to World coordinates
Vehicle reference frame follows NovAtel's convention: Right/Forward/Up (RFU) respectively for the axes x/y/z.
![](./pics_fusion/coordination_apollo.png) 
In particular, we describe the orientation of the car by three angles:
 * 1) the pitch 俯仰角, in (-pi/2, pi/2)(避免万向节锁), corresponds to a rotation around the x-axis;
 * 2) the roll 翻滚角, in ==[-pi, pi),== corresponds to a rotation around the y-axis;
 * 3) the yaw 偏航角, in [-pi, pi), corresponds to a rotation around the z-axis.

 ```
 return pitch_ < M_PI_2 && pitch_ > -M_PI_2;
 ```
 
 * The pitch is zero when the car is level and positive when the nose is up.
 * The roll is zero when the car is level and positive when the left part is up.
 * The yaw is zero when the car is== facing North==, and positive when facing West.
 
 [@cranewhy conflict. euler_angles.yaw() is zero when the car is pointing North, but the heading is zero when the car is pointing East.]

### 1.3 Sensor coordinates
 Respect to Ego vehicle coordinates
 

## 2. Sensor setup parameters

### 2.1 extrinsics
>sensor_id
>position(x, y, z)
>rotation(pitch, yaw, roll)

###2.2 intrinsics
* For Lidar

```
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;
  	
  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;
```

* For Radar
```
  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;
  
  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  
  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
```
>For Camera
>>height, width, ==Output vector of distortion coefficients  [k1, k2, p1, p2, k3, k4, k5, k6]  of 4, 5, or 8 elements.==

## 3. datafile output 
* txt format;
* each row represents a sensor measurement 

> For Radar data, the columns are:
>>sensor_id, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth, x_ego, y_ego, v_ego, yaw_ego,yawrate_ego. 

> For lidar data, the columns are:
>>sensor_id, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth, x_ego, y_ego, v_ego, yaw_ego,yawrate_ego. 

> For camera data, the columns are:
>> sensor_id, ...

<table>
        <tr>
            <th>sensor_id(Radar)</th>
            <th>rho_measured</th>
            <th>phi_measured</th>
            <th>rhodot_measured</th>
            <th>timestamp</th>
            <th>x_groundtruth</th>
            <th>y_groundtruth</th>
            <th>vx_groundtruth</th>
            <th>vy_groundtruth</th>
            <th>yaw_groundtruth</th>
            <th>yawrate_groundtruth</th>
            <th>x_ego</th>
            <th>y_ego</th>
            <th>v_ego</th>
            <th>yaw_ego</th>
            <th>yawrate_ego</th>
        </tr>
        <tr>
            <th>sensor_id(Lidar)</th>
            <th>x_measured</th>
            <th>y_measured</th>
            <th>timestamp</th>
            <th>x_groundtruth</th>
            <th>y_groundtruth</th>
            <th>vx_groundtruth</th>
            <th>vy_groundtruth</th>
            <th>yaw_groundtruth</th>
            <th>yawrate_groundtruth</th>
            <th>x_ego</th>
            <th>y_ego</th>
            <th>v_ego</th>
            <th>yaw_ego</th>
            <th>yawrate_ego</th>
        </tr>
    </table>

## 4. Airsim

### Coordinate System

All AirSim API uses NED coordinate system, i.e., +X is North, +Y is East and +Z is Down. All units are in SI system. 

`OriginGeopoint` This setting specifies the latitude, longitude and altitude of the Player Start component placed in the Unreal environment. The vehicle's home point is computed using this transformation. Note that all coordinates exposed via APIs are using NED system in SI units which means each vehicle starts at (0, 0, 0) in NED system. 


### Lidar
X Y Z : Position of the lidar relative to the vehicle (in NED, in meters)

Roll Pitch Yaw : Orientation of the lidar relative to the vehicle (in degrees, yaw-pitch-roll order to front vector +X)

no noise

```
                "LidarSensor1": { 
                    "SensorType": 6,
                    "Enabled" : true,
                    "NumberOfChannels": 16,
                    "RotationsPerSecond": 10,
                    "PointsPerSecond": 100000,
                    "X": 0, "Y": 0, "Z": -1,
                    "Roll": 0, "Pitch": 0, "Yaw" : 0,
                    "VerticalFOVUpper": -15,
                    "VerticalFOVLower": -25,
                    "HorizontalFOVStart": -20,
                    "HorizontalFOVEnd": 20,
                    "DrawDebugPoints": true,
                    "DataFrame": "SensorLocalFrame"
                },
```

### Camera

`Gimbal` Pitch, Roll and Yaw

Add noise

