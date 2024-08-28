실습용 파일에 대한 설명
================
폴더 : my_pkg1

하위 폴더 : launch
	- pub-sub.launch : pub.py와  sub.py 구동을 위한 launch 파일
	- pub-sub-param.launch : pub-param.py에 파라미터를 부여함과 동시에 pub-param.py와  sub.py 구동을 위한 launch 파일
	-파라미터 circle_size의 값을 2 또는 4로 변경할 수 있음 (거북이 회전반경이 바뀜) 

하위 폴더 : src 
	- pub.py : 노드를 만들고, publisher 객체를 생성하여 거북이가 1Hz 주기로 회전하게 만드는 코드
	- sub.py : 노드를 만들고, subsciber 객체를 생성함. 메세지를 수신하여 함수를 호출하는 코드.
	- pub-param.py : 노드를 만들고, pub-sub-param.launch에서 파라미터를 받아와서 거북이가 회전하게 만드는 코드
 	- 파라미터 circle_size의 값을 읽어들여 linear_x 값으로 사용 (거북이 회전반경이 바뀜) 