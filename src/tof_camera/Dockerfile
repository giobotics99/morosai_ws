FROM ros:humble

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" > /root/.bashrc 

# Copy in tof library 
COPY tof-linux-x86_64.tar.gz /.

# Extract tof library
RUN tar -xzf tof-linux-x86_64.tar.gz -C /usr && \
  rm tof-linux-x86_64.tar.gz