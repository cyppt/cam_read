# 从rosbag中读取图片和位姿的功能包

本功能包用于从NAS中（/ALAM/Datasets/ININ/车载多相机传感器平台-1代/dataset-20211229）
读取图片以及位姿信息，可以修改bag中图片发布的频率（用于标定），也可以将图片解压到指定文件夹中，并且输出json格式的索引文件。


## 参数文件说明
0. 参数文件位置
    ```
        cam_read/param.yaml
        cam_read/bag_param.yaml
    ``` 
    - 放入上述位置，编译时，编译器会自动寻找到其路径
    - cam_read/bag_param.yaml 为旧版本参数文件
    - cam_read/param.yaml 为新版本参数文件


## Version
1. ver 0.0.1  **data:**23.03.05 初始版本