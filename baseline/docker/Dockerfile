FROM yanyanlidocker/venom:v1.2

RUN	apt-get update && apt-get install -y \
	libgoogle-glog-dev \
	libboost-all-dev \
	libsuitesparse-dev \
	libglib2.0-doc \
	libgtk2.0-doc \
	libgl1-mesa-dev \
	libglew-dev \          
	libxkbcommon-x11-dev   #for keyboard

ENV	DEBIAN_FRONTEND noninteractive
RUN	apt-get update -y && apt-get install -y \
	openssh-server xfce4 xfce4-terminal xfce4-goodies x11vnc sudo bash xvfb && \
	useradd -ms /bin/bash ubuntu && echo 'ubuntu:ubuntu' | chpasswd && \
	echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && apt-get clean && \
	rm -rf /var/lib/apt/lists/*

COPY 	x11vnc /etc/init.d/
COPY 	xvfb /etc/init.d/
COPY 	entry.sh /

RUN 	sudo chmod +x /entry.sh /etc/init.d/* 

EXPOSE 	5900

ENTRYPOINT [ "/entry.sh" ]
