import cv2
# Always import cv2 at the top
import time
import commons as cm
import zed
from track_and_move import TrackObject
import shared_variables as sv
import auv


threshold = 0.2
top_k = 5
first_time = 1
auv = AUV()
tr = TrackObject(auv=auv, tolerance=0.02)
zed = Zed(640, 640)
servo_cam = cv2.VideoCapture(0)
bottom_cam = cv2.VideoCapture(1)

prev_command = None

detect_fn, labels = cm.load_model(sv.PATH_TO_MODEL, sv.PATH_TO_LABELS)


def main():
    arr_dur = [0, 0, 0]
    zed.start_capture_thread()

    global first_time, prev_command
    while True:

        if sv.USE_SERVO_CAM:
            frame = servo_cam.read()[1]
        elif sv.USE_ZED:
            frame, depth = zed.get_new_image()
        elif sv.USE_BOTTOM_CAM:
            frame = bottom_cam.read()[1]

        detections = cm.predictions(detect_fn, frame, (640, 640))

        objs, obj_lst = cm.get_output(
            detections=detections, score_threshold=threshold, top_k=top_k, labels=labels)

        if sv.MOVE_DOWN_FLAG:
            auv.set_depth(0.5)
            sv.MOVE_DOWN_FLAG = 0
            
        if not sv.TASK1:
            print("Task 1")
            if not sv.FIND_GATE_FLAG:
                if 'gate' in obj_lst:
                    sv.START_TRACKING_FLAG = 1
                    sv.FIND_GATE_FLAG = 1
                    sv.USE_SERVO_CAM = 0
                    sv.USE_ZED = 1
'''
                if 'gate' not in obj_lst and not sv.TURN_LEFT_FLAG:
                    auv.turn_cam_left()
                    sv.TURN_LEFT_FLAG = 1
                    continue
                elif 'gate' not in obj_lst and not sv.TURN_RIGHT_FLAG:
                    sv.TURN_LEFT_FLAG = 0
                    auv.turn_cam_right()
                    sv.TURN_RIGHT_FLAG = 1
                    continue	

            elif sv.START_TRACKING_FLAG and sv.FIND_GATE_FLAG and not sv.STOP_TRACKING:
                if (sv.TURN_LEFT_FLAG or sv.TURN_RIGHT_FLAG) and not sv.CHANGE_CAMERA_DIRECTION_FLAG:
                    sv.CHANGE_CAMERA_DIRECTION_FLAG = 1
                    sv.TURN_LEFT_FLAG = 0
                    auv.set_cam_center()

                if sv.TURN_RIGHT_FLAG and not sv.CHANGE_CAMERA_DIRECTION_FLAG:
                    sv.CHANGE_CAMERA_DIRECTION_FLAG = 1
                    sv.TURN_RIGHT_FLAG = 0
                    auv.set_cam_center()
                tr.track_object(objs, labels, 'gate', obj_lst)
'''
            elif sv.STOP_TRACKING:
                if 'red_flare' in obj_lst:
                    dist = cm.get_distance(objs, labels, 'red_flare')
                    print(dist)
                    if dist > .75:
                        cmd = "forward"
                        if prev_command != cmd:
                            auv.forward()
                            prev_command = cmd
                    elif dist <= .75:
                        cmd = "stop"
                        if prev_command != cmd:
                            auv.left()
                            time.sleep(1)
                            auv.forward()
                            time.sleep(2)
                            auv.right()
                            time.sleep(1)
                            auv.stop_motion()
                            auv.set_depth(0.7)
                            prev_command = cmd
                elif 'gate' in obj_lst:
                    cmd = "forward"
                    if prev_command != cmd:
                        auv.forward()
                        prev_command = cmd
                elif 'gate' not in obj_lst:
                    auv.stop_motion()
                    prev_command = None
                    sv.TASK1 = 1
                    sv.TURN_RIGHT_FLAG = 0
                    sv.TURN_LEFT_FLAG = 0
                    sv.START_TRACKING_FLAG = 0
                    sv.CHANGE_CAMERA_DIRECTION_FLAG = 0
                    sv.STOP_TRACKING = 0
                    sv.USE_ZED = 0
                    sv.USE_SERVO_CAM = 1



'''
qwertyuiopasdfghjklzxcvbnm
'''


        elif not sv.TASK2_PART1:
            print("Task 2 I")
            if first_time:
                auv.forward()
                time.sleep(3)
                auv.stop_motion()
                first_time = 0

            if not sv.FIND_TUB_FLAG:
                if 'tub' in obj_lst:
                    sv.START_TRACKING_FLAG = 1
                    sv.FIND_TUB_FLAG = 1
                    sv.USE_SERVO_CAM = 0
                    sv.USE_ZED = 1
                elif 'tub' not in obj_lst and not sv.TURN_LEFT_FLAG:
                    auv.turn_cam_left()
                    sv.TURN_LEFT_FLAG = 1
                    continue

            elif sv.START_TRACKING_FLAG and sv.FIND_TUB_FLAG and not sv.STOP_TRACKING:
                if sv.TURN_LEFT_FLAG and not sv.CHANGE_CAMERA_DIRECTION_FLAG:
                    sv.CHANGE_CAMERA_DIRECTION_FLAG = 1
                    sv.TURN_LEFT_FLAG = 0
                    auv.set_cam_center()
                tr.track_object(objs, labels, 'tub', obj_lst)

            elif sv.STOP_TRACKING:
                if 'tub' in obj_lst:
                    dist = cm.get_distance(objs, labels, 'tub')
                    print(dist)
                    if dist > .07:
                        cmd = "forward"
                        if prev_command != cmd:
                            auv.forward()
                            prev_command = cmd
                    elif dist <= .07:
                        auv.stop_motion()
                        sv.TASK2_PART1 = 1
                        sv.TURN_RIGHT_FLAG = 0
                        sv.TURN_LEFT_FLAG = 0
                        sv.START_TRACKING_FLAG = 0
                        sv.CHANGE_CAMERA_DIRECTION_FLAG = 0
                        sv.STOP_TRACKING = 0
                        first_time = 1
                        sv.USE_ZED = 1
                        sv.USE_SERVO_CAM = 0

        elif not sv.TASK2_PART2:
            print("Task 2 II")
            tr.track_object(objs, labels, 'blue_tub', obj_lst)

            if sv.STOP_TRACKING:
                sv.TASK2_PART2 = 1
                sv.STOP_TRACKING = 0
                sv.USE_ZED = 0
                sv.USE_SERVO_CAM = 0
                sv.USE_BOTTOM_CAM = 11
                prev_command = None

        elif not sv.TASK2_PART3:
            print("Task 2 III")
            if first_time:
                sv.STOP_TRACKING = 0
                auv.stop_motion()
                first_time = 0
                obj_lst = []

            print(obj_lst)

            if 'blue_tub' in obj_lst:
                sv.START_TRACKING_FLAG = 1

            if 'blue_tub' not in obj_lst:
                cmd = "forward"
                if prev_command != cmd:
                    auv.forward()
                continue

            if sv.START_TRACKING_FLAG and not sv.STOP_TRACKING:
                if 'blue_tub' not in obj_lst:
                    auv.backward()
                    time.sleep(1.5)
                    auv.stop_motion()
                tr.track_object(objs,
                                labels, 'blue_tub', obj_lst, "task2_part3")
            if sv.STOP_TRACKING:
                sv.TASK2_PART3 = 1
                sv.STOP_TRACKING = 0
                auv.stop_motion()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # frame = tr.draw_overlays(frame, objs, labels, arr_dur)
        cv2.imshow('Object Tracking', cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    cv2.destroyAllWindows()


main()
