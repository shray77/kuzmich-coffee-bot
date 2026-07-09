#!/bin/bash
if [ -f /opt/ros/foxy/setup.bash ]; then source /opt/ros/foxy/setup.bash; fi
if [ -f ~/g1_foxy_ws/install/setup.bash ]; then source ~/g1_foxy_ws/install/setup.bash; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# CycloneDDS 0.7.0 (реально установлено) требует старую плоскую схему
# <NetworkInterfaceAddress> — вложенный <Interfaces> появился в 0.8+.
cat > /tmp/cyclonedds_lo.xml <<'XML'
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
XML
export CYCLONEDDS_URI=file:///tmp/cyclonedds_lo.xml
