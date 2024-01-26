# pcl_cpp ros2 foxy

Tool of ros2 versions for pcl voxels, noise cancellation, and cutting removal libraries

Environment  --> Ros1 noetic, Ros2 foxy(if you want to use foxy, Use foxy branch)

**Study with reference to the link below**


https://pcl.gitbook.io/tutorial/part-1/part01-chapter04/part01-chapter04-pcl-cpp
https://limhyungtae.github.io/2021-09-09-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4-(1)-shared_ptr/

# Tools currently available

1. Voxelization
2. PassThrough
3. Statistics-based noise remover(통계 기반 잡음 제거)

3.1 Parameter

setInputCloud --> 입력 포인트 클라우드

setMean       --> **분석시 고려할 이웃 점의 수**

setStddevMulThresh --> **Outlier로 처리할 거리 정보, 표준편차에 기반한 임계치 값으로 값이 작을수록 aggressive 하게 적용**

![Untitled (2)](https://github.com/lidarmansiwon/pcl_cpp/assets/117976120/fe1028a6-c231-4adc-b59a-a39e9dd667ca)
