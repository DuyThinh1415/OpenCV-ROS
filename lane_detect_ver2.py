import cv2
import numpy as np
import time

AER = 7  # Acceptable error range

# 640*360
vid_file = "test2.mp4"
video = cv2.VideoCapture(vid_file)

left_center_pre_index = 300
right_center_pre_index = 350
previous_index = 0

list_of_center=np.array(range(150))
list_of_index=np.array(range(150))
size_of_index_list=0
flag_for_road_detected=0

now_curve=0
now_turn=0


while video.isOpened():
    time.sleep(0.033)
    ret, frame = video.read()
    if not ret:
        break
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny_frame = cv2.Canny(gray_frame, threshold1=50, threshold2=250)

    row_index = 340
    col_index = 340
    size_of_index_list=0

    frame = cv2.line(frame, (left_center_pre_index, 350), (right_center_pre_index, 350), (255, 0, 255), 3, cv2.LINE_AA)

    while row_index > 250:
        row_index = row_index - 2
        left_col_index = col_index + 1
        while left_col_index < 630:
            if canny_frame[row_index, left_col_index] == 0:
                left_col_index = left_col_index + 1
            else:
                break
        if left_col_index == 630:
            continue

        # -------------------------------------------------------------------------------------------310-340
        right_col_index = col_index - 1
        while right_col_index > 10:
            if canny_frame[row_index, right_col_index] == 0:
                right_col_index = right_col_index - 1
            else:
                break
        if right_col_index == 10:
            continue

        center_col_index = (left_col_index + right_col_index) // 2
        mid_center_pre_index = (left_center_pre_index + right_center_pre_index) // 2
        if row_index == 338:
            if ((left_center_pre_index - center_col_index) * (right_center_pre_index - center_col_index)) > 0:
                frame = cv2.rectangle(frame, (center_col_index + 3, row_index + 1),
                                      (center_col_index - 3, row_index - 1), (200, 100, 16), 1)
                flag_for_road_detected=0
                continue
            else:
                flag_for_road_detected=1
            if center_col_index > mid_center_pre_index:
                left_center_pre_index = left_center_pre_index + 1
                right_center_pre_index = right_center_pre_index + 1
            else:
                left_center_pre_index = left_center_pre_index - 1
                right_center_pre_index = right_center_pre_index - 1
        else:  # trust check
            if ((center_col_index - previous_index) > AER) or (center_col_index - previous_index) < (-1 * AER):
                frame = cv2.rectangle(frame, (center_col_index + 3, row_index + 1),
                                      (center_col_index - 3, row_index - 1), (255, 255, 255), 1)
                #col_index = center_col_index
                continue
            # frame = cv2.line(frame, (previous_index, row_index + 5), (center_col_index, row_index), (0, 0, 255), 1,
            #                cv2.LINE_AA)
        col_index = center_col_index

        previous_index = center_col_index

        frame = cv2.rectangle(frame, (right_col_index + 3, row_index + 1),
                              (right_col_index - 3, row_index - 1), (0, 255, 0), 1)
        frame = cv2.rectangle(frame, (left_col_index + 3, row_index + 1),
                              (left_col_index - 3, row_index - 1), (255, 0, 0), 1)
        frame = cv2.rectangle(frame, (center_col_index , row_index ),
                              (center_col_index , row_index ), (0, 255, 255), 1)

        list_of_center[size_of_index_list]=center_col_index
        list_of_index[size_of_index_list]=row_index
        size_of_index_list=size_of_index_list+1

    #write output
    size_of_index_list=size_of_index_list-3
    if (size_of_index_list > 10) and (flag_for_road_detected == 1):
        curve=(((list_of_center[10]-list_of_center[0])*(list_of_index[size_of_index_list]-list_of_index[0]))
        //(list_of_index[10]-list_of_index[0])) +list_of_center[0]-list_of_center[size_of_index_list]
        now_curve=0.9*now_curve+0.1*curve
        tmp_turn=list_of_center[0] - list_of_center[10]
        now_turn=now_turn*0.95+tmp_turn*0.05
        frame = cv2.putText(frame, "Detected Road, curvature of the Road is "+str(int(now_curve)), (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 2)

        tmp=curve+list_of_center[size_of_index_list]
        frame = cv2.rectangle(frame, (tmp + 3, list_of_index[size_of_index_list] + 1),
                              (tmp - 3, list_of_index[size_of_index_list] - 1), (255, 0, 255), 1)
    else:
        frame = cv2.putText(frame, "Undetected road, maintain movement "+str(int(now_curve)), (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0,100,255), 2)
    frame = cv2.putText(frame, "The curvature of the wheel is " + str(int(now_turn)), (100, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 100, 255), 2)
    frame=cv2.resize(frame,dsize=None,fx=2,fy=2)
    cv2.imshow("Input", frame)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
