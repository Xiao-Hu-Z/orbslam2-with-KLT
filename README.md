# orbslam2-with-KLT

1.改进点思路：

orbslam也只会优化关键帧，关键帧只占所有帧中的很小一部分，提取特征点是orbslam中非常耗时的部分。

- 在Track前面加金字塔光流，根据光流结果判断是否为关键帧。可以根据光流跟踪到的点数量。若数量太少，就需要加入关键帧

- 对于关键帧，提取特征点，然后重新采用特征点法跟踪这些特征点。

- 对于非关键帧，不会进入orbslam2的三个线程，阻止了非关键帧提取特征点和后续的计算，只通过3D点到2D平面的投影（PnPRansac）来计算相机位姿。



2.主要工作：

- 在include文件夹内增加LK.h文件，实现cv::Mat computeMtcwUseLK(KeyFrame
  *lastKeyFrame, Mat color, bool lastColorIsKeyFrame, Mat K, Mat mDistCoef)函数
- 在orbslam计算相机位姿前，使用光流法跟踪上一关键帧的特征点，并用pnpransac计算相机位姿。

3.效果

在精度下降较小的情况下，加速了orbslam# orbslam2-with-LK-optical-flow![Screenshot from 2020-01-15 21-15-24](/media/xiaohu/xiaohu/slam/开源方案（修改）/orbslam2-with-KLT/result/Screenshot from 2020-01-15 21-15-24.png)







![Screenshot from 2020-01-15 21-15-24](/media/xiaohu/xiaohu/slam/开源方案（修改）/orbslam2-with-KLT/result/Screenshot from 2020-01-15 21-15-25.png)