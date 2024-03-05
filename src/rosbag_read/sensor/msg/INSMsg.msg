# 帧头，包含时间戳信息
std_msgs/Header header

# 模块类型
uint8 module_type

# error_code
uint32 error_code

# 模块信息
string ins_data

# INS_Acc (0X500)
uint16 ACC_X
uint16 ACC_Y
uint16 ACC_Z

# INS_GYRO (0X501)
uint16 GYRO_X
uint16 GYRO_Y
uint16 GYRO_Z

# INS_HeadingPitchRoll (0X502)
uint16 INS_PitchAngle
uint16 INS_RollAngle
uint16 INS_HeadingAngle

# INS_HeightAndTime (0X503)
uint32 INS_LocatHeight
uint32 INS_Time

# INS_LatitudeLongitude (0X504)
uint32 INS_Latitude
uint32 INS_Longitude

# INS_Speed (0X505)
uint16 INS_NorthSpd
uint16 INS_EastSpd
uint16 INS_ToGroundSpd

# INS_DataInfo (0X506)
uint8 INS_GpsFlag_Pos
uint8 INS_NumSV     #星数
uint8 INS_GpsFlag_Heading
uint8 INS_Gps_Age
uint8 INS_Car_Status
uint8 INS_Status

# INS_Std (0X507)
uint16 INS_Std_Lat
uint16 INS_Std_Lon
uint16 INS_Std_LocatHeight
uint16 INS_Std_Heading
