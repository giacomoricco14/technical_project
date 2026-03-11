#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

SDF_CONTENT = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="dynamic_obstacle">
    <static>true</static> <link name="link">
      <pose>0 0 0 0 0 0</pose> <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius> <length>1.0</length>  </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.5</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1.0 0.0 0.0 1</ambient> <diffuse>1.0 0.0 0.0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

def main(args=None):
    # Inizializziamo ROS 2 e creiamo il nodo e il publisher
    rclpy.init(args=args)
    node = rclpy.create_node('generatore_ostacoli')
    publisher = node.create_publisher(Bool, '/obstacle_status', 10)

    print("\n--- GENERATORE DI OSTACOLI DINAMICI ---")
    
    file_path = "/tmp/dynamic_obstacle.sdf"
    with open(file_path, "w") as f:
        f.write(SDF_CONTENT)
        
    NOME_MONDO = "default"
    COORD_X = "-1.0" 
    COORD_Y = "0.35" 
    COORD_Z = "0.5"
    
    comando_spawn = (
        f"ign service -s /world/{NOME_MONDO}/create "
        f"--reqtype ignition.msgs.EntityFactory "
        f"--reptype ignition.msgs.Boolean "
        f"--timeout 2000 "
        f"--req 'sdf_filename: \"{file_path}\" name: \"dynamic_obstacle\" pose {{ position {{ x: {COORD_X} y: {COORD_Y} z: {COORD_Z} }} }}' > /dev/null 2>&1"
    )
    
    print(f"⚠️  Lancio l'ostacolo rosso alle coordinate X:{COORD_X}, Y:{COORD_Y}!")
    os.system(comando_spawn)
    
    # PUBBLICHIAMO IL SEGNALE TRUE (Ostacolo presente)
    publisher.publish(Bool(data=True))
    node.get_logger().info("Segnale /obstacle_status: TRUE trasmesso!")
    
    print("\n✅ Ostacolo piazzato! Guarda RViz e Gazebo.")
    input("👉 PREMI [INVIO] QUI QUANDO VUOI FARLO SPARIRE... ")
    
    print("🧹 Rimozione ostacolo in corso...")
    comando_remove = (
        f"ign service -s /world/{NOME_MONDO}/remove "
        f"--reqtype ignition.msgs.Entity "
        f"--reptype ignition.msgs.Boolean "
        f"--timeout 2000 "
        f"--req 'name: \"dynamic_obstacle\" type: MODEL' > /dev/null 2>&1"
    )
    os.system(comando_remove)
    
    # PUBBLICHIAMO IL SEGNALE FALSE (Ostacolo rimosso)
    publisher.publish(Bool(data=False))
    node.get_logger().info("Segnale /obstacle_status: FALSE trasmesso!")
    print("✨ Ostacolo rimosso con successo!\n")

    # Pulizia finale del nodo
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()