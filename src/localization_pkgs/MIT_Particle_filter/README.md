# MIT Particle filter

map server 설치

환경에 따라 rosdep이 설치되어 있지 않은 경우 설치 후 진행

```bash
sudo apt-get update
rosdep install -r --from-paths src --ignore-src --rosdistro (ROS 버전) -y
```

RangeLibc 설치
```bash
sudo pip install cython
git clone https://github.com/DCUSnSLab/range_libc_for_ROS_noetic.git
cd range_libc-for-Noetic/pywrapper
# on VM
./compile.sh
# on car - compiles GPU ray casting methods
./compile_with_cuda.sh
```


아래 명령어로 파티클 필터 실행
```bash
roslaunch particle_filter localize.launch
```
