# Danh sách các file trong src
> **auto_distance_avoid.cpp** : module phát hiện vật cản trước mặt và tính khoảng rộng nhất để xoay về hướng đó và đi
> 
> **controller.py** : module controller
> > 
> **lane_change.py** : module dùng để kiểm tra tính đúng đắn của giải thuật chuyển làn
> > 
> **lane_detect.py** : module dò đường cũ
> > 
> **lane_detect_V2.cpp** : module dò đường mới
> > 
> **laser_control.cpp** : cũng là phát hiện vật cản nhưng gà hơn

các file khác không được đề cập phía trên nghĩa là không quan trọng, nó chỉ dùng để test những thứ nhảm nhí thôi.

  - - - -
# Hướng dẫn sử dụng gói demo

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
### tải về
Bạn phải tải đầy đủ thư mục src, resource, msg và hai file Cmake và package.xml
model .h5 của AI, bạn phải tải riêng ở drive hoặc nơi khác 
Thư mục config là tùy chọn nêu bạn muốn sử dụng khả năng đọc file yaml
thư mục launch là tùy chọn nếu bạn muốn sử dụng roslaunch
## cài đặt
bạn bỏ các thư mục đã tải vào một pakage trống và sửa đổi một số thông tin, s

