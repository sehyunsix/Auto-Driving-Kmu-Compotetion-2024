�ǽ��� ���Ͽ� ���� ����
================
���� : my_pkg1

���� ���� : launch
	- pub-sub.launch : pub.py��  sub.py ������ ���� launch ����
	- pub-sub-param.launch : pub-param.py�� �Ķ���͸� �ο��԰� ���ÿ� pub-param.py��  sub.py ������ ���� launch ����
	-�Ķ���� circle_size�� ���� 2 �Ǵ� 4�� ������ �� ���� (�ź��� ȸ���ݰ��� �ٲ�) 

���� ���� : src 
	- pub.py : ��带 �����, publisher ��ü�� �����Ͽ� �ź��̰� 1Hz �ֱ�� ȸ���ϰ� ����� �ڵ�
	- sub.py : ��带 �����, subsciber ��ü�� ������. �޼����� �����Ͽ� �Լ��� ȣ���ϴ� �ڵ�.
	- pub-param.py : ��带 �����, pub-sub-param.launch���� �Ķ���͸� �޾ƿͼ� �ź��̰� ȸ���ϰ� ����� �ڵ�
 	- �Ķ���� circle_size�� ���� �о�鿩 linear_x ������ ��� (�ź��� ȸ���ݰ��� �ٲ�) 