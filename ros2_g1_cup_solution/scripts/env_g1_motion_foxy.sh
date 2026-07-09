#!/bin/bash
if [ -f /opt/ros/foxy/setup.bash ]; then source /opt/ros/foxy/setup.bash; fi
if [ -f ~/g1_foxy_ws/install/setup.bash ]; then source ~/g1_foxy_ws/install/setup.bash; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ROBOT_IP="${UNITREE_ROBOT_IP:-192.168.123.161}"
IFACE="$(ip route get "$ROBOT_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++){if($i=="dev"){print $(i+1); exit}}}')"
if [ -z "$IFACE" ]; then IFACE="${UNITREE_NET_IFACE:-eth0}"; fi
export UNITREE_NET_IFACE="$IFACE"
cat > /tmp/cyclonedds_g1_motion.xml <<XML
<CycloneDDS>
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface name="$IFACE" multicast="true"/></Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
XML
export CYCLONEDDS_URI=file:///tmp/cyclonedds_g1_motion.xml
echo "UNITREE_NET_IFACE=$UNITREE_NET_IFACE"
