from djitellopy import Tello

# create and connect
# 创建Tello对象并连接
tello = Tello()
tello.connect()
# configure drone
# 设置无人机
tello.enable_mission_pads()
tello.set_mission_pad_detection_direction(0)  # forward detection only  只识别前方




tello.takeoff()

pad = tello.get_mission_pad_id()

# detect and react to pads until we see pad #1
# 发现并识别挑战卡直到看见1号挑战卡
while pad != 1:
    #if pad == 3:
       # tello.move_back(30)
        #tello.rotate_clockwise(90)

    if pad != -1:
        #tello.move_up(30)
        #tello.flip_forward()
        print("Pad detected : ",pad)
        tello.land()
    if pad == 6:
        tello.land()

    pad = tello.get_mission_pad_id()
    print("PADDING: ", pad)

# graceful termination
# 安全结束程序
print(pad)
tello.disable_mission_pads()
tello.land()
tello.end()
