# MIT Racecar Labs

## 설치

### 시뮬레이터

아래 주소로 이동 후 설치
http://itgit.cu.ac.kr/AutonomousDriving/MIT-Simulator-Noetic

### 파티클 필터

아래 주소로 이동 후 설치
http://itgit.cu.ac.kr/AutonomousDriving/MIT_Particle-filter

소스코드 다운로드 및 빌드
```
cd catkin_ws/src
git clone http://itgit.cu.ac.kr/AutonomousDriving/MIT-Racecar-Labs-Noetic
cd .. & catkin_make
```


필수 패키지 설치 진행

```
pip install recordclass
# this one will take a while
pip install scikit-image
```

워크스페이스 루트 경로에서 아래 명령어 입력

```
sudo apt-get update
rosdep install -r --from-paths src --ignore-src --rosdistro noetic -y
```

## 실행

```
# 실제 차량
(작성 예정)

# 시뮬레이터
roslaunch racecar_simulator simulate.launch
roslaunch particle_filter localize.launch
roslaunch ta_lab6 waypoint_control.launch
roslaunch ta_lab6 follow_trajectory.launch
```

이후 지도상에서 Goal 지점을 선택하면 해당 위치로 차량이 이동

원본 Github 주소 : https://github.com/vivekchand/MIT-Racecar-Labs

//TODO : Kinetic에 맞게 설정된 Dependency 설치 파일 수정 부분 설명 및 설치 방법 작성 필요
// 해당 시뮬레이터와 아래 파티클 필터를 사용한 Localization 방법 설명

// Particle filter repo : http://itgit.cu.ac.kr/AutonomousDriving/MIT_Particle-filter
