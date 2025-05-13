### Dockerfile per Convince UC3 con model2code e UC3 (aggiornato con remote SSH)

# 1) Base image
FROM ste93/convince:qt_6.7.3_ubuntu22.04_iron_stable_sim_completed_iros

# 2) Usa root per installare le dipendenze di sistema
USER root
# ENV DEBIAN_FRONTEND=noninteractive TZ=UTC SSH_AUTH_SOCK=/ssh-agent
RUN apt-get update && apt-get install -y sudo

# Aggiungi user1 al gruppo sudo
RUN usermod -aG sudo user1

# (Opzionale ma consigliato) Disabilita password per sudo per user1
RUN echo 'user1 ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers.d/user1 && chmod 0440 /etc/sudoers.d/user1

# 2.a) Aggiorna e installa strumenti di build, CMake, Git, ROS2, ecc.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake cmake-curses-gui cmake-qt-gui \
      clang \
      git git-lfs \
      gdb gdbserver \
      ca-certificates \
      libboost-all-dev libxerces-c-dev \
      nano openssh-client tzdata ros-iron-sensor-msgs && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 3) Configura Git con variabili build-arg
ARG GIT_USER_NAME="Your Name"
ARG GIT_USER_EMAIL="you@example.com"
RUN git config --global user.name "${GIT_USER_NAME}" && \
    git config --global user.email "${GIT_USER_EMAIL}" 


# 4) Imposta hosts di GitHub per SSH (root e user1)
RUN mkdir -p /root/.ssh && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts && \
    mkdir -p /home/user1/.ssh && \
    ssh-keyscan github.com >> /home/user1/.ssh/known_hosts && \
    chown -R user1:user1 /home/user1/.ssh

# 5) Installa vcpkg e tinyxml2
RUN git clone https://github.com/Microsoft/vcpkg.git /opt/vcpkg && \
    cd /opt/vcpkg && \
    ./bootstrap-vcpkg.sh && \
    ./vcpkg integrate install && \
    ./vcpkg install tinyxml2 && \
    chmod -R a+r /opt/vcpkg/installed

# # 6) Clona e builda model2code con toolchain di vcpkg
# RUN git clone https://github.com/convince-project/model2code.git /opt/model2code && \
#     mkdir /opt/model2code/build && \
#     cd /opt/model2code/build && \
#     cmake .. \
#       -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake && \
#     make install && \
#     #Scrivibile da user1 per eventuali modifiche
#     chown -R user1:user1 /opt/model2code

# PASSIAMO AL BRANCH MODIFICATO DI model2code
RUN rm -rf /home/user1/model2code /home/user1/UC3 && \
    \
    # model2code
    git clone https://github.com/convince-project/model2code.git /home/user1/model2code && \
    cd /home/user1/model2code && \
    git fetch origin test/new_features && \
    git checkout test/new_features && \
    mkdir build && cd build && \
    cmake .. \
      -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake && \
    make install && \
    chown -R user1:user1 /home/user1/model2code



# 7) Clona UC3 e configura remote SSH
RUN rm -rf /home/user1/UC3 && \
    git clone --depth 1 --branch HardwareFaultSkillTest \
      https://github.com/convince-project/UC3.git /home/user1/UC3 && \
    # Imposta remote SSH al posto di HTTPS
    git -C /home/user1/UC3 remote set-url origin git@github.com:convince-project/UC3.git && \
    chown -R user1:user1 /home/user1/UC3

# 8) Rimuovi source obsoleto da .bashrc di user1
RUN sed -i '/laboratory-tour\/install\/setup.bash/d' /home/user1/.bashrc

# 9) Source ROS2 in ogni nuova shell
RUN echo "source /opt/ros/iron/setup.bash" >> /etc/bash.bashrc && \
    echo "source /opt/ros/iron/setup.bash" >> /root/.bashrc

# 10) Torna a user1 e set bash come shell di default
USER user1
WORKDIR /home/user1
RUN git config --global --add safe.directory /home/user1/UC3 && \
    sudo chown -R user1:user1 /home/user1/UC3/model-high-level/Skills 





CMD ["terminator"]


# docker build . \
#   -t convince-uc3 \
#   --build-arg GIT_USER_NAME="BeniaminoSquitieri" \
#   --build-arg GIT_USER_EMAIL="bennibeniamino@gmail.com"

# docker build .   -t convince-uc3   --build-arg GIT_USER_NAME="BeniaminoSquitieri"   --build-arg GIT_USER_EMAIL="bennibeniamino@gmail.com"


#sudo xhost +; docker run --rm -it --privileged --network host --pid host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  -e QT_X11_NO_MITSHM=1 -v /home/bsquitieri-iit.local/Desktop/UC3:/home/user1/UC3 --name uc3-shell convince-uc3

# it ->interactive 
# privileged ->permessi di amministratore nel sistema , serve per device audio ecc...
# network host -> ip è dato sulla scheda di rete e non solo internamente
# pid host-> id dei processi sono sul sistema vero, non solo sul docker, senza questo come processi vedi solo quelli del sistema docker
#-e DISPLAY ->monta variabile di ambiente del sistema
# -v -> volumi, Dice , prendi la cartella /tmp/.X11-unixe montala /tmp/.X11-unix del remoto
# -e QT_X11_NO_MITSHM=1 ->altra variabile di ambiente 

#se voglio montare la chiave SSH 