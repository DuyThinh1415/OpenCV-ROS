# Hướng dẫn sử dụng gói demo

## Tải về 
vì file này có quá nhiều thư mục lặt vặt nên git không thể chứa được, nên tôi đã up lên google drive, các bạn vào link sau bằng mail trường để tải 

```shell
https://drive.google.com/drive/folders/1NdmE5xjwv0-Kwys7V3jiaCKLvWr3YgZN?usp=sharing
```

## Bắt đầu
bạn cần phải bảo đảm máy của bạn có thư viện openCV cho C++

để thử xem máy bạn có đáp ứng yêu cầu này không, tạo một file test.cpp và copy nội dung sau

```shell
#include "stdlib.h"
#include <unistd.h>
#include <chrono>
#include <thread>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(){
  printf(" Hello !");
  return 0;
}
```
sau đó mở terminal và gõ lệnh 

```shell
g++ test.cpp -o test `pkg-config --cflags --libs opencv4`
```

nếu nó có biên dịch được và không lỗi, bạn có thể tới bước tiếp theo

## Sử dụng gói
### tải về và cài đặt

sau khi tải thư mục demo từ git, bạn để nó ở một nơi bất kì trong máy (không nên để trong thư mục catkin_ws). 

Sau đó cd vào thư mục demo_package và gõ lệnh
```shell
catkin_make
```
quá trình này sẽ tiến hành cài đặt các nội dung cần thiết. Nếu máy bạn không chạy được 100%, liên hệ tôi để được sửa lỗi

### khởi chạy các chương trình

bạn mở một terminal mới, cd vào thư mục demo và gõ lệnh
```shell
source devel/setup.bash
```
giờ đây, cửa sổ terminal đó có khả năng chạy các node trong gói.
### Để chạy một gói, gõ lệnh
```shell
rosrun demo_package <tên node>
```
có hai node bạn cần quan tâm là lane_detect.py và lane_detect_V2

lane_detect_V2 được code bằng C++. bạn có thể sửa code để chạy thử, sau khi sửa code C++, bạn lưu lại, cd ra thư mục demo_package và bấm catkin_make để biên dịch code C++

trong code đó, ở hàng 28 là tốc độ mặc định của robot, bạn có thể thay đổi để nó đi nhanh hoặc chậm hơn
